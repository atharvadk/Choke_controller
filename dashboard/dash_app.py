#!/usr/bin/env python3
"""
Gamified Dash‑based dashboard for the choke‑control simulation.
Keeps the original Matplotlib dashboard (renamed to dashboard_mpl.py) as a fallback.
"""

import os
import sys
import base64
from collections import deque

import dash
from dash import dcc, html, Input, Output, State, ctx
import dash_bootstrap_components as dbc
import plotly.graph_objs as go
import numpy as np

# ----------------------------------------------------------------------
# 1️⃣ Make sure the project root is on PYTHONPATH so we can import the
#    simulator and controller packages exactly as the original script does.
# ----------------------------------------------------------------------
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

from simulator.simulation import Simulator
from controllers.pid_controller import PIDChokeController
from controllers.rl_controller import RLChokeController
from controllers.mpc_controller import ModelPredictiveChokeController
from controllers.rule_based import RuleBasedChokeController

# ----------------------------------------------------------------------
# 2️⃣ Constants (mirror the values from dashboard.py)
# ----------------------------------------------------------------------
PSI_TO_BAR = 0.0689475729
BAR_TO_PSI = 14.5037738
M3S_TO_BBL_HR = 22643.4

DEFAULT_DT = 1.0
DEFAULT_DURATION = 3600.0
DEFAULT_TARGET_OIL = 100.0   # bbl/hr
DEFAULT_MIN_WHP = 210.0      # psi
DEFAULT_UPDATE_MS = 500      # UI refresh interval (ms)

# CONFIG_PATH points to the default simulation config
CONFIG_PATH = "configs/default.yaml"

# ----------------------------------------------------------------------
# 3️⃣ Helper to instantiate the correct controller
# ----------------------------------------------------------------------
def get_controller(ctype: str, target_oil: float, min_whp: float, dt: float):
    """Return a controller instance (or None for fixed choke)."""
    if ctype == "fixed":
        return None
    if ctype == "rule_based":
        return RuleBasedChokeController(target_oil_bbl_hr=target_oil,
                                        min_whp_psi=min_whp)
    if ctype == "pid":
        return PIDChokeController(kp=1.2, ki=0.08, kd=0.3,
                                  target_oil_bbl_hr=target_oil,
                                  min_whp_psi=min_whp,
                                  dt=dt)
    if ctype == "mpc":
        return ModelPredictiveChokeController(config_path=CONFIG_PATH,
                                              horizon=6,
                                              dt_control=60.0,
                                              target_oil_bbl_hr=target_oil,
                                              min_whp_psi=min_whp)
    if ctype == "rl":
        return RLChokeController(model_path="models/rl_choke_policy.npz",
                                 target_oil_bbl_hr=target_oil,
                                 min_whp_psi=min_whp)
    raise ValueError(f"Unknown controller type: {ctype}")

# ----------------------------------------------------------------------
# 4️⃣ Initialise the Dash app (Bootstrap dark theme for a modern look)
# ----------------------------------------------------------------------
app = dash.Dash(__name__,
                external_stylesheets=[dbc.themes.CYBORG],
                suppress_callback_exceptions=True,
                meta_tags=[{"name": "viewport",
                            "content": "width=device-width, initial-scale=1"}])
server = app.server  # expose for gunicorn etc.

# ----------------------------------------------------------------------
# 5️⃣ Layout – sidebar + main content + modal/toast/audio for gamification
# ----------------------------------------------------------------------
def make_sidebar():
    return dbc.Col(
        [
            html.H4("Controls", className="display-6 text-center mb-4"),
            # Controller selector
            dbc.Label("Controller type"),
            dcc.RadioItems(
                id="controller-type",
                options=[
                    {"label": "Fixed", "value": "fixed"},
                    {"label": "Rule‑Based", "value": "rule_based"},
                    {"label": "PID", "value": "pid"},
                    {"label": "MPC", "value": "mpc"},
                    {"label": "RL", "value": "rl"},
                ],
                value="pid",
                inline=False,
                className="mb-3",
            ),
            html.Hr(),
            # Numeric inputs
            dbc.Label("Duration (s)"),
            dbc.Input(id="duration", type="number", value=DEFAULT_DURATION, step=1),
            html.Br(),
            dbc.Label("dt (s)"),
            dbc.Input(id="dt", type="number", value=DEFAULT_DT, step=0.1),
            html.Br(),
            dbc.Label("Target Oil (bbl/hr)"),
            dbc.Input(id="target-oil", type="number", value=DEFAULT_TARGET_OIL, step=1),
            html.Br(),
            dbc.Label("Min WHP (psi)"),
            dbc.Input(id="min-whp", type="number", value=DEFAULT_MIN_WHP, step=1),
            html.Br(),
            dbc.Label("Update interval (ms)"),
            dbc.Input(id="update-interval", type="number", value=DEFAULT_UPDATE_MS, step=100),
            html.Br(),
            html.Br(),
            dbc.Button("Start Simulation", id="start-btn", color="success", className="me-2 w-100"),
            dbc.Button("Stop Simulation", id="stop-btn", color="danger", className="w-100"),
            html.Br(),
            html.Br(),
            dbc.Button("Show Stats", id="stats-btn", color="info", className="w-100"),
        ],
        width=2,
        style={"backgroundColor": "#1e1e1f", "minHeight": "100vh", "padding": "1.5rem"},
    )


def make_main():
    # Six graphs in a 2x3 grid
    graphs_top = dbc.Row(
        [
            dbc.Col(dbc.Spinner(dcc.Graph(id="choke-plot"), color="light"), width=4),
            dbc.Col(dbc.Spinner(dcc.Graph(id="oil-plot"), color="light"), width=4),
            dbc.Col(dbc.Spinner(dcc.Graph(id="whp-plot"), color="light"), width=4),
        ],
        className="g-3",
    )
    graphs_bottom = dbc.Row(
        [
            dbc.Col(dbc.Spinner(dcc.Graph(id="bhp-plot"), color="light"), width=4),
            dbc.Col(dbc.Spinner(dcc.Graph(id="res-plot"), color="light"), width=4),
            dbc.Col(dbc.Spinner(dcc.Graph(id="cmd-plot"), color="light"), width=4),
        ],
        className="g-3",
    )
    # Gamification sidebar (right side)
    gamification = dbc.Card(
        [
            dbc.CardHeader(html.H5("🏆 Gamification", className="mb-0")),
            dbc.CardBody(
                [
                    dbc.Row(
                        [
                            dbc.Col(html.Div(id="score-display"), width=6),
                            dbc.Col(dbc.Badge("Level: 0", id="level-display", color="secondary"), width=6),
                        ],
                        className="mb-2",
                    ),
                    dbc.Progress(id="xp-bar", value=0, max=1000, striped=True, animated=True, className="mb-2"),
                    html.Div(id="badges-display", className="mb-2"),
                ]
            ),
        ],
        style={"marginTop": "1rem"},
    )
    return dbc.Col(
        [
            html.Div([html.H2("Choke Control Dashboard – Gamified", className="text-center mb-4")]),
            html.Hr(),
            graphs_top,
            html.Hr(),
            graphs_bottom,
            html.Hr(),
            gamification,
        ],
        width=10,
        style={"backgroundColor": "#1e1e1f", "minHeight": "100vh", "padding": "1.5rem"},
    )


def make_modal():
    return dbc.Modal(
        [
            dbc.ModalHeader(dbc.ModalTitle("Live Statistics")),
            dbc.ModalBody(id="stats-body"),
            dbc.ModalFooter(dbc.Button("Close", id="close-stats", className="ms-auto", n_clicks=0)),
        ],
        id="stats-modal",
        size="lg",
        is_open=False,
        backdrop="static",
    )


def make_toast():
    # Toast positioned at top‑right via inline style
    return dbc.Toast(
        id="achievement-toast",
        header="🏅 Achievement Unlocked!",
        is_open=False,
        dismissable=True,
        duration=4000,
        icon="success",
        style={"position": "fixed", "top": 20, "right": 20, "width": 350},
    )


def make_audio():
    # A short beep (approx 0.2s) encoded as base64 wav
    wav_bytes = b"""\
RIFF$\x00\x00\x00WAVEfmt \x10\x00\x00\x00\x01\x00\x01\x00D\xac\x00\x00\x88X\x01\x00\x02\x00\x10\x00data\x00\x00\x00\x00"""
    b64 = base64.b64encode(wav_bytes).decode()
    return html.Audio(id="achievement-sound", src=f"data:audio/wav;base64,{b64}", autoPlay=False, controls=False)


app.layout = dbc.Container(
    [
        dcc.Interval(id="tick-interval", interval=DEFAULT_UPDATE_MS, n_intervals=0, disabled=True),
        dcc.Store(id="sim-data", data={}),  # holds time series as lists
        dcc.Store(id="game-state", data={}),  # holds score, level, badges, etc.
        make_sidebar(),
        make_main(),
        make_modal(),
        make_toast(),
        make_audio(),
    ],
    fluid=True,
    style={"backgroundColor": "#1e1e1f", "minHeight": "100vh"},
)


# ----------------------------------------------------------------------
# 6️⃣ Callback: start / stop simulation
# ----------------------------------------------------------------------
@app.callback(
    Output("tick-interval", "disabled"),
    Output("sim-data", "data"),
    Output("game-state", "data"),
    Input("start-btn", "n_clicks"),
    Input("stop-btn", "n_clicks"),
    State("controller-type", "value"),
    State("duration", "value"),
    State("dt", "value"),
    State("target-oil", "value"),
    State("min-whp", "value"),
    prevent_initial_call=True,
)
def start_stop_sim(start_clicks, stop_clicks, ctype, duration, dt, target_oil, min_whp):
    triggered = ctx.triggered_id
    if triggered == "start-btn":
        # Initialise simulation and controller
        sim = Simulator(CONFIG_PATH)
        sim.dt = float(dt)
        sim.reset()
        controller = get_controller(ctype, float(target_oil), float(min_whp), float(dt))
        if controller is not None:
            controller.reset()
        # Store objects on Flask app for later retrieval in callbacks
        server.config["sim"] = sim
        server.config["controller"] = controller
        # Initialise empty data stores
        data = {
            "times": [],
            "choke": [],
            "oil": [],
            "whp": [],
            "bhp": [],
            "res": [],
            "cmd": [],
        }
        # Initialise game state
        game_state = {
            "score": 0,
            "level": 0,
            "badges": [],
            "_last_achievement_check": 0,
            "_whp_ok_seconds": 0,
            "_oil_on_target_seconds": 0,
        }
        return False, data, game_state  # enable interval
    elif triggered == "stop-btn":
        # Stop the interval timer
        return True, dash.no_update, dash.no_update
    return dash.no_update, dash.no_update, dash.no_update


# ----------------------------------------------------------------------
# 7️⃣ Helper: simulation step (handles both step and step_with_info)
# ----------------------------------------------------------------------
def step_simulation(sim, choke_cmd):
    """Perform one simulation step, returning (obs, info)."""
    try:
        # Newer simulator may have step_with_info returning (obs, info)
        obs, info = sim.step_with_info(choke_cmd)
        return obs, info
    except Exception:
        # Fallback: use step and then _get_observation
        sim.step(choke_cmd)
        obs = sim._get_observation()
        info = {}  # no extra info
        return obs, info


# ----------------------------------------------------------------------
# 8️⃣ Callback: simulation step + figure updates + gamification
# ----------------------------------------------------------------------
@app.callback(
    Output("choke-plot", "figure"),
    Output("oil-plot", "figure"),
    Output("whp-plot", "figure"),
    Output("bhp-plot", "figure"),
    Output("res-plot", "figure"),
    Output("cmd-plot", "figure"),
    Output("score-display", "children"),
    Output("level-display", "children"),
    Output("xp-bar", "value"),
    Output("xp-bar", "max"),
    Output("badges-display", "children"),
    Output("stats-body", "children"),
    Output("achievement-toast", "is_open"),
    Output("achievement-toast", "children"),
    Output("achievement-sound", "src"),
    Input("tick-interval", "n_intervals"),
    State("sim-data", "data"),
    State("game-state", "data"),
    State("controller-type", "value"),
    State("duration", "value"),
    State("dt", "value"),
    State("target-oil", "value"),
    State("min-whp", "value"),
    prevent_initial_call=True,
)
def update_simulation(n, sim_data, game_state, ctype, duration, dt, target_oil, min_whp):
    # Debug print to see if callback is firing
    print(f"[DEBUG] tick {n}, sim_data keys: {list(sim_data.keys()) if sim_data else None}")
    # If simulation not running, return empty figures
    if not sim_data or not sim_data.get("times"):
        empty = go.Figure()
        empty.update_layout(template="plotly_dark", margin=dict(l=20, r=20, t=30, b=20))
        print("[DEBUG] No sim data, returning empty")
        return (
            empty,
            empty,
            empty,
            empty,
            empty,
            empty,
            "Score: 0",
            "Level: 0",
            0,
            1000,
            [],
            "",
            False,
            "",
            "",
        )
def update_simulation(n, sim_data, game_state, ctype, duration, dt, target_oil, min_whp):
    # If simulation not running, return empty figures
    if not sim_data or not sim_data.get("times"):
        empty = go.Figure()
        empty.update_layout(template="plotly_dark", margin=dict(l=20, r=20, t=30, b=20))
        return (
            empty,
            empty,
            empty,
            empty,
            empty,
            empty,
            "Score: 0",
            "Level: 0",
            0,
            1000,
            [],
            "",
            False,
            "",
            "",
        )

    sim = server.config.get("sim")
    controller = server.config.get("controller")
    if sim is None or controller is None:
        # Should not happen if start was pressed
        empty = go.Figure()
        empty.update_layout(template="plotly_dark", margin=dict(l=20, r=20, t=30, b=20))
        return (empty,) * 6 + ("Score: 0", "Level: 0", 0, 1000, [], "", False, "", "")

    # Determine choke command
    if ctype == "fixed":
        choke_cmd = 30.0  # fixed choke opening %
    else:
        obs = sim._get_observation()
        choke_cmd = controller.compute_action(obs)

    # Step simulation
    obs, _ = step_simulation(sim, choke_cmd)

    # Extract values from simulator state (adjust attribute names if needed)
    state = sim.state
    t = state.time
    choke_open = getattr(state, "opening_actual", 0.0)  # percent
    oil_rate = getattr(state, "oil_rate", 0.0) * M3S_TO_BBL_HR  # bbl/hr
    whp_psi = getattr(state, "Pwh", 0.0) * BAR_TO_PSI  # psi
    bhp_psi = getattr(state, "Pwf", 0.0) * BAR_TO_PSI  # psi
    res_bar = getattr(state, "Pr", 0.0)  # bar

    # Append to buffers
    sim_data["times"].append(t)
    sim_data["choke"].append(choke_open)
    sim_data["oil"].append(oil_rate)
    sim_data["whp"].append(whp_psi)
    sim_data["bhp"].append(bhp_psi)
    sim_data["res"].append(res_bar)
    sim_data["cmd"].append(choke_cmd)

    # Keep only last N points (optional, for performance)
    max_points = int(float(duration) / float(dt))
    if len(sim_data["times"]) > max_points:
        for key in sim_data:
            sim_data[key] = sim_data[key][-max_points:]

    # ----------------- Gamification logic -----------------
    target_oil_f = float(target_oil)
    min_whp_f = float(min_whp)

    # Error in oil production (relative)
    oil_error = abs(oil_rate - target_oil_f) / target_oil_f if target_oil_f != 0 else 0
    whp_ok = whp_psi >= min_whp_f

    # Reward: closeness to target, penalty for low WHP
    step_reward = max(0.0, 1.0 - oil_error) * 10.0
    if not whp_ok:
        step_reward -= 5.0

    new_score = game_state["score"] + step_reward
    new_level = int(new_score // 1000)  # each 1000 points = new level

    # Track seconds where conditions hold (assuming dt seconds per tick)
    dt_sec = float(dt)
    if whp_ok:
        game_state["_whp_ok_seconds"] = game_state.get("_whp_ok_seconds", 0) + dt_sec
    else:
        game_state["_whp_ok_seconds"] = 0

    if oil_error < 0.05:  # within 5% of target
        game_state["_oil_on_target_seconds"] = game_state.get("_oil_on_target_seconds", 0) + dt_sec
    else:
        game_state["_oil_on_target_seconds"] = 0

    # Achievement checks (simple examples)
    new_badges = list(game_state.get("badges", []))
    toast_msg = ""
    toast_show = False
    sound_src = ""

    # Define achievement thresholds
    achievements = [
        ("Starter", lambda s: s["score"] >= 100, "Earned 100 points"),
        ("Steady Hand", lambda s: s.get("_oil_on_target_seconds", 0) >= 10, "Maintained target oil ≥10 s"),
        ("Pressure Master", lambda s: s.get("_whp_ok_seconds", 0) >= 15, "KEPT WHP above min for 15 s"),
        ("Oil Baron", lambda s: s["score"] >= 500 and s.get("_oil_on_target_seconds", 0) >= 20, "High score + steady oil"),
        ("Champion", lambda s: s["level"] >= 3, "Reached level 3"),
    ]

    for badge_name, condition, message in achievements:
        if badge_name not in new_badges and condition(game_state):
            new_badges.append(badge_name)
            toast_msg = f"{badge_name}: {message}"
            toast_show = True
            # Use the audio source from layout (we'll fetch later)
            sound_src = app.layout.children[-1].src  # the audio component is last
            break  # only show one toast per tick for simplicity

    # Update game state
    game_state["score"] = new_score
    game_state["level"] = new_level
    game_state["badges"] = new_badges

    # Prepare figures (dark template)
    def make_fig(x, y, name, color, yaxis_title):
        fig = go.Figure()
        fig.add_trace(go.Scatter(x=list(x), y=list(y), mode="lines", name=name, line=dict(color=color)))
        fig.update_layout(
            template="plotly_dark",
            margin=dict(l=20, r=20, t=30, b=20),
            yaxis_title=yaxis_title,
            hovermode="x unified",
        )
        return fig

    fig_choke = make_fig(sim_data["times"], sim_data["choke"], "Choke Opening (%)", "#ff9800", "Choke Opening (%)")
    fig_oil = make_fig(sim_data["times"], sim_data["oil"], "Oil Rate (bbl/hr)", "#4caf50", "Oil Rate (bbl/hr)")
    fig_whp = make_fig(sim_data["times"], sim_data["whp"], "WHP (psi)", "#f44336", "WHP (psi)")
    fig_bhp = make_fig(sim_data["times"], sim_data["bhp"], "BHP (psi)", "#9c27b0", "BHP (psi)")
    fig_res = make_fig(sim_data["times"], sim_data["res"], "Reservoir Pressure (bar)", "#2196f3", "Reservoir Pressure (bar)")
    fig_cmd = make_fig(sim_data["times"], sim_data["cmd"], "Choke Command (%)", "#607d8b", "Choke Command (%)")

    # Stats modal body (latest values)
    stats_md = f"""
    **Time:** {t:.1f} s
    **Choke Opening:** {choke_open:.1f}%
    **Oil Rate:** {oil_rate:.1f} bbl/hr
    **WHP:** {whp_psi:.1f} psi
    **BHP:** {bhp_psi:.1f} psi
    **Reservoir Pressure:** {res_bar:.2f} bar
    **Choke Command:** {choke_cmd:.1f}%
    """

    # Badges display
    badges_html = [
        dbc.Badge(b, color="info", className="me-1 mb-1") for b in new_badges
    ] if new_badges else html.Span("No badges yet", style={"color": "#888"})

    return (
        fig_choke,
        fig_oil,
        fig_whp,
        fig_bhp,
        fig_res,
        fig_cmd,
        f"Score: {int(new_score)}",
        f"Level: {new_level}",
        int(new_score % 1000),  # progress within current level
        1000,                   # max per level
        badges_html,
        stats_md,
        toast_show,
        toast_msg,
        sound_src,
    )


# ----------------------------------------------------------------------
# 9️⃣ Callback: open/close stats modal
# ----------------------------------------------------------------------
@app.callback(
    Output("stats-modal", "is_open"),
    Input("stats-btn", "n_clicks"),
    Input("close-stats", "n_clicks"),
    State("stats-modal", "is_open"),
    prevent_initial_call=True,
)
def toggle_modal(open_click, close_click, is_open):
    if open_click or close_click:
        return not is_open
    return is_open


# ----------------------------------------------------------------------
# 🔟 Run the app (when executed directly)
# ----------------------------------------------------------------------
if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8050, debug=True)