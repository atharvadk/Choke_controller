#!/usr/bin/env python3
"""
Python Web Server for the 2026 Autonomous Choke Controller Web Dashboard.
Serves static frontend files (HTML/CSS/JS) and provides REST API endpoints for Python simulation physics and controllers.
"""

import os
import sys
import json
import urllib.parse
from http.server import HTTPServer, SimpleHTTPRequestHandler
import webbrowser

# Add project root to sys.path
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

from simulator.simulation import Simulator
from controllers.pid_controller import PIDChokeController
from controllers.rule_based import RuleBasedChokeController
from controllers.mpc_controller import ModelPredictiveChokeController
from controllers.rl_controller import RLChokeController

STATIC_DIR = os.path.join(os.path.dirname(__file__), "static")

# Persistent simulator instances per session
class ServerState:
    sim = None
    controller = None
    controller_type = "pid"

state_container = ServerState()

class DashboardRequestHandler(SimpleHTTPRequestHandler):
    def translate_path(self, path):
        # Cleanly resolve static files relative to STATIC_DIR
        parsed = urllib.parse.urlparse(path)
        req_path = parsed.path.lstrip("/")

        if req_path == "" or req_path == "index.html":
            return os.path.join(STATIC_DIR, "index.html")

        # Handle requests starting with static/ or directly
        if req_path.startswith("static/"):
            req_path = req_path[len("static/"):]

        full_path = os.path.join(STATIC_DIR, req_path)
        if os.path.exists(full_path):
            return full_path

        return super().translate_path(path)

    def do_GET(self):
        if self.path == "/api/status":
            self._send_json({
                "status": "online",
                "engine": "Python Hydrodynamic Physics Engine",
                "version": "2026.1",
                "controllers": ["fixed", "rule_based", "pid", "mpc", "rl"]
            })
            return
        super().do_GET()

    def do_POST(self):
        content_length = int(self.headers.get('Content-Length', 0))
        post_data = self.rfile.read(content_length) if content_length > 0 else b'{}'
        
        try:
            body = json.loads(post_data.decode('utf-8'))
        except Exception:
            body = {}

        if self.path == "/api/step":
            self._handle_step(body)
        elif self.path == "/api/simulate":
            self._handle_simulate(body)
        elif self.path == "/api/benchmark":
            self._handle_benchmark(body)
        else:
            self._send_json({"error": "Unknown API endpoint"}, status=404)

    def _handle_step(self, body):
        ctrl_name = body.get('controller', 'pid')
        target_oil = float(body.get('target_oil', 120.0))
        min_whp = float(body.get('min_whp', 210.0))
        dt = float(body.get('dt', 1.0))

        if state_container.sim is None:
            state_container.sim = Simulator()

        sim = state_container.sim
        sim.dt = dt

        # Instantiate or update controller
        if state_container.controller is None or state_container.controller_type != ctrl_name:
            state_container.controller_type = ctrl_name
            if ctrl_name == 'pid':
                state_container.controller = PIDChokeController(target_oil_bbl_hr=target_oil, min_whp_psi=min_whp, dt=dt)
            elif ctrl_name == 'rule_based':
                state_container.controller = RuleBasedChokeController(target_oil_bbl_hr=target_oil, min_whp_psi=min_whp)
            elif ctrl_name == 'mpc':
                state_container.controller = ModelPredictiveChokeController(target_oil_bbl_hr=target_oil, min_whp_psi=min_whp)
            elif ctrl_name == 'rl':
                state_container.controller = RLChokeController(target_oil_bbl_hr=target_oil, min_whp_psi=min_whp)
            else:
                state_container.controller = None

        # Compute Action
        obs = sim._get_observation()
        if state_container.controller is not None:
            cmd = state_container.controller.compute_action(obs)
        else:
            cmd = 30.0

        # Step Simulator
        sim.step(cmd)

        res_state = {
            "time": sim.state.time,
            "opening_actual": sim.state.opening_actual,
            "opening_target": sim.state.opening_target,
            "oil_rate": sim.state.oil_rate,
            "gas_rate": sim.state.gas_rate,
            "water_rate": sim.state.water_rate,
            "total_flow": sim.state.total_flow,
            "Pwf": sim.state.Pwf,
            "Pwh": sim.state.Pwh,
            "Pr": sim.state.Pr,
        }

        self._send_json({"status": "ok", "state": res_state})

    def _handle_simulate(self, body):
        ctrl_name = body.get('controller', 'pid')
        target_oil = float(body.get('target_oil', 120.0))
        min_whp = float(body.get('min_whp', 210.0))
        duration = float(body.get('duration', 600.0))
        dt = float(body.get('dt', 1.0))

        sim = Simulator()
        sim.dt = dt

        if ctrl_name == 'pid':
            ctrl = PIDChokeController(target_oil_bbl_hr=target_oil, min_whp_psi=min_whp, dt=dt)
        elif ctrl_name == 'rule_based':
            ctrl = RuleBasedChokeController(target_oil_bbl_hr=target_oil, min_whp_psi=min_whp)
        elif ctrl_name == 'mpc':
            ctrl = ModelPredictiveChokeController(target_oil_bbl_hr=target_oil, min_whp_psi=min_whp)
        elif ctrl_name == 'rl':
            ctrl = RLChokeController(target_oil_bbl_hr=target_oil, min_whp_psi=min_whp)
        else:
            ctrl = None

        time_series = []
        steps = int(duration / dt)

        for _ in range(steps):
            obs = sim._get_observation()
            cmd = ctrl.compute_action(obs) if ctrl else 30.0
            sim.step(cmd)
            time_series.append({
                "time": sim.state.time,
                "choke_actual": sim.state.opening_actual,
                "choke_target": sim.state.opening_target,
                "oil_rate": sim.state.oil_rate * 22643.4,  # bbl/hr
                "whp": sim.state.Pwh * 14.5037738,          # psi
                "bhp": sim.state.Pwf * 14.5037738,
                "pr": sim.state.Pr
            })

        self._send_json({"status": "ok", "time_series": time_series})

    def _handle_benchmark(self, body):
        target_oil = float(body.get('target_oil', 120.0))
        min_whp = float(body.get('min_whp', 210.0))
        duration = float(body.get('duration', 600.0))
        dt = float(body.get('dt', 1.0))

        controllers = {
            "Fixed (30%)": None,
            "Rule-Based": RuleBasedChokeController(target_oil_bbl_hr=target_oil, min_whp_psi=min_whp),
            "PID": PIDChokeController(target_oil_bbl_hr=target_oil, min_whp_psi=min_whp, dt=dt),
            "MPC": ModelPredictiveChokeController(target_oil_bbl_hr=target_oil, min_whp_psi=min_whp),
            "RL Agent": RLChokeController(target_oil_bbl_hr=target_oil, min_whp_psi=min_whp)
        }

        benchmark_results = []
        for name, ctrl in controllers.items():
            sim = Simulator()
            sim.dt = dt
            cum_oil = 0.0
            iae = 0.0
            wear = 0.0
            whp_violations = 0
            prev_choke = 30.0

            steps = int(duration / dt)
            for _ in range(steps):
                obs = sim._get_observation()
                cmd = ctrl.compute_action(obs) if ctrl else 30.0
                sim.step(cmd)
                
                oil_bbl = sim.state.oil_rate * 22643.4
                whp_psi = sim.state.Pwh * 14.5037738
                cum_oil += (oil_bbl * dt) / 3600.0
                iae += abs(target_oil - oil_bbl) * dt
                wear += abs(sim.state.opening_actual - prev_choke)
                if whp_psi < min_whp:
                    whp_violations += 1
                prev_choke = sim.state.opening_actual

            benchmark_results.append({
                "controller": name,
                "cum_oil": round(cum_oil, 1),
                "avg_rate": round(cum_oil * (3600.0 / duration), 1),
                "iae": round(iae, 1),
                "wear": round(wear, 1),
                "whp_violations": whp_violations
            })

        self._send_json({"status": "ok", "results": benchmark_results})

    def _send_json(self, data, status=200):
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode('utf-8'))

    def log_message(self, format, *args):
        try:
            msg = format % args if args else str(format)
            if "GET /api/" in msg or "POST /api/" in msg:
                return
        except Exception:
            pass
        super().log_message(format, *args)

def run_server(port=8050, open_browser=True):
    server_address = ('', port)
    httpd = HTTPServer(server_address, DashboardRequestHandler)
    url = f"http://localhost:{port}"
    print("=" * 70)
    print(f"🚀 AUTONOMOUS CHOKE CONTROLLER 2026 WEB DASHBOARD RUNNING")
    print(f"📍 Web App URL: {url}")
    print(f"💡 Matplotlib Backup Dashboard: python dashboard/dashboard_mpl.py")
    print("=" * 70)

    if open_browser:
        try:
            webbrowser.open(url)
        except Exception:
            pass

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping dashboard server...")
        httpd.server_close()

if __name__ == "__main__":
    port = 8050
    if len(sys.argv) > 1 and sys.argv[1].isdigit():
        port = int(sys.argv[1])
    run_server(port=port)
