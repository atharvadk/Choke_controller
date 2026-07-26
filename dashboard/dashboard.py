#!/usr/bin/env python3
"""
Real-time dashboard for choke control simulation.
Displays live plots of key variables while simulation runs.
"""

import sys
import os
import argparse
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Add parent directory to path to import simulator and controllers
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from simulator.simulation import Simulator
from controllers.pid_controller import PIDChokeController
from controllers.pid_controller import PIDChokeController
from controllers.rl_controller import RLChokeController
from controllers.mpc_controller import ModelPredictiveChokeController
from controllers.rule_based import RuleBasedChokeController

# Unit conversion constants
PSI_TO_BAR = 0.0689475729
BAR_TO_PSI = 14.5037738
M3S_TO_BBL_HR = 22643.4

CONFIG_PATH = "configs/default.yaml"


def create_controller(controller_type, dt, target_oil_bbl_hr=100.0, min_whp_psi=210.0):
    """Factory to create controller instances."""
    if controller_type == "fixed":
        return None  # Fixed choke handled separately
    elif controller_type == "rule_based":
        return RuleBasedChokeController(target_oil_bbl_hr=target_oil_bbl_hr, min_whp_psi=min_whp_psi)
    elif controller_type == "pid":
        return PIDChokeController(kp=1.2, ki=0.08, kd=0.3, target_oil_bbl_hr=target_oil_bbl_hr, min_whp_psi=min_whp_psi, dt=dt)
    elif controller_type == "mpc":
        return ModelPredictiveChokeController(config_path=CONFIG_PATH, horizon=6, dt_control=60.0,
                                              target_oil_bbl_hr=target_oil_bbl_hr, min_whp_psi=min_whp_psi)
    elif controller_type == "rl":
        return RLChokeController(model_path="models/rl_choke_policy.npz", target_oil_bbl_hr=target_oil_bbl_hr,
                                 min_whp_psi=min_whp_psi)
    else:
        raise ValueError(f"Unknown controller type: {controller_type}")


def simulate_step(sim, controller, controller_type, choke_cmd):
    """Perform one simulation step and return observation and choke command."""
    if controller_type == "fixed":
        choke_cmd = 30.0
        obs = sim.step(choke_cmd)
    else:
        obs = sim._get_observation()
        choke_cmd = controller.compute_action(obs)
        obs, _ = sim.step_with_info(choke_cmd)
    return obs, choke_cmd


def main():
    parser = argparse.ArgumentParser(description="Real-time dashboard for choke control simulation.")
    parser.add_argument("--controller", type=str, default="pid",
                        choices=["fixed", "rule_based", "pid", "mpc", "rl"],
                        help="Controller type to use (default: pid)")
    parser.add_argument("--duration", type=float, default=3600.0,
                        help="Simulation duration in seconds (default: 3600s = 1 hour)")
    parser.add_argument("--dt", type=float, default=1.0,
                        help="Simulation time step in seconds (default: 1.0)")
    parser.add_argument("--target_oil", type=float, default=100.0,
                        help="Target oil rate in bbl/hr (default: 100)")
    parser.add_argument("--min_whp", type=float, default=210.0,
                        help="Minimum wellhead pressure safety limit in psi (default: 210)")
    parser.add_argument("--update_interval", type=int, default=1,
                        help="Update plot every N simulation steps (default: 1)")
    args = parser.parse_args()

    # Initialize simulator
    sim = Simulator(CONFIG_PATH)
    sim.dt = args.dt
    sim.reset()

    # Initialize controller
    controller = create_controller(args.controller, args.dt,
                                   target_oil_bbl_hr=args.target_oil,
                                   min_whp_psi=args.min_whp)

    # Data storage for plotting (use deque with maxlen for rolling window)
    max_points = int(args.duration / args.dt)  # total steps
    times = deque(maxlen=max_points)
    choke_openings = deque(maxlen=max_points)
    oil_rates = deque(maxlen=max_points)
    whp_psi = deque(maxlen=max_points)
    bhp_psi = deque(maxlen=max_points)
    reservoir_pressure = deque(maxlen=max_points)
    choke_commands = deque(maxlen=max_points)

    # Set up the figure and axes
    plt.style.use('seaborn-v0_8')
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle(f"Real-time Choke Control Dashboard - {args.controller.upper()} Controller", fontsize=16)

    # Flatten axes for easier indexing
    ax_choke = axes[0, 0]
    ax_oil = axes[0, 1]
    ax_whp = axes[1, 0]
    ax_bhp = axes[1, 1]
    ax_res = axes[2, 0]
    ax_cmd = axes[2, 1]

    # Initialize empty lines
    line_choke, = ax_choke.plot([], [], 'b-', linewidth=2)
    line_oil, = ax_oil.plot([], [], 'g-', linewidth=2)
    line_whp, = ax_whp.plot([], [], 'r-', linewidth=2)
    line_bhp, = ax_bhp.plot([], [], 'm-', linewidth=2)
    line_res, = ax_res.plot([], [], 'c-', linewidth=2)
    line_cmd, = ax_cmd.plot([], [], 'k--', linewidth=2)

    # Set axis labels and limits (will update dynamically)
    ax_choke.set_ylabel("Choke Opening (%)")
    ax_choke.grid(True, alpha=0.3)
    ax_oil.set_ylabel("Oil Rate (bbl/hr)")
    ax_oil.grid(True, alpha=0.3)
    ax_whp.set_ylabel("Wellhead Pressure (psi)")
    ax_whp.grid(True, alpha=0.3)
    ax_bhp.set_ylabel("Bottom Hole Pressure (psi)")
    ax_bhp.grid(True, alpha=0.3)
    ax_res.set_ylabel("Reservoir Pressure (bar)")
    ax_res.set_xlabel = ax_res.set_xlabel("Time (s)")
    ax_res.grid(True, alpha=0.3)
    ax_cmd.set_ylabel("Choke Command (%)")
    ax_cmd.set_xlabel("Time (s)")
    ax_cmd.grid(True, alpha=0.3)

    # Add target oil rate line to oil plot
    target_line = ax_oil.axhline(y=args.target_oil, color='k', linestyle='--', alpha=0.7, label='Target')
    ax_oil.legend(loc='upper right')

    # Add min WHP line to whp plot
    whp_limit_line = ax_whp.axhline(y=args.min_whp, color='r', linestyle=':', alpha=0.7, label='Min WHP')
    ax_whp.legend(loc='upper right')

    # Text annotation for current stats
    stats_text = ax_choke.text(0.02, 0.98, '', transform=ax_choke.transAxes,
                               verticalalignment='top', fontsize=10,
                               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    # Animation update function
    def update(frame):
        # Perform multiple simulation steps if update_interval > 1
        for _ in range(args.update_interval):
            obs, choke_cmd = simulate_step(sim, controller, args.controller, 0.0)
            # Store data
            times.append(sim.state.time)
            choke_openings.append(sim.state.opening_actual)
            oil_rates.append(sim.state.oil_rate * M3S_TO_BBL_HR)
            whp_psi.append(sim.state.Pwh * BAR_TO_PSI)
            bhp_psi.append(sim.state.Pwf * BAR_TO_PSI)
            reservoir_pressure.append(sim.state.Pr)
            choke_commands.append(choke_cmd)

            # Stop if simulation time exceeds duration
            if sim.state.time >= args.duration:
                anim.event_source.stop()
                print("Simulation finished.")
                break

        # Update data for lines
        if len(times) > 0:
            line_choke.set_data(times, choke_openings)
            line_oil.set_data(times, oil_rates)
            line_whp.set_data(times, whp_psi)
            line_bhp.set_data(times, bhp_psi)
            line_res.set_data(times, reservoir_pressure)
            line_cmd.set_data(times, choke_commands)

            # Update axis limits dynamically
            for ax, data_y in [(ax_choke, choke_openings), (ax_oil, oil_rates),
                               (ax_whp, whp_psi), (ax_bhp, bhp_psi),
                               (ax_res, reservoir_pressure), (ax_cmd, choke_commands)]:
                if len(data_y) > 0:
                    y_min = min(data_y)
                    y_max = max(data_y)
                    y_range = y_max - y_min
                    if y_range == 0:
                        y_range = 1
                    ax.set_ylim(y_min - 0.1 * y_range, y_max + 0.1 * y_range)
            # X-axis: time
            if len(times) > 0:
                t_min = min(times)
                t_max = max(times)
                t_range = t_max - t_min
                if t_range == 0:
                    t_range = 1
                for ax in [ax_choke, ax_oil, ax_whp, ax_bhp, ax_res, ax_cmd]:
                    ax.set_xlim(t_min - 0.1 * t_range, t_max + 0.1 * t_range)

            # Update stats text
            latest_oil = oil_rates[-1] if oil_rates else 0
            latest_whp = whp_psi[-1] if whp_psi else 0
            latest_bhp = bhp_psi[-1] if bhp_psi else 0
            latest_choke = choke_openings[-1] if choke_openings else 0
            latest_res = reservoir_pressure[-1] if reservoir_pressure else 0
            stats_text.set_text(
                f"Time: {sim.state.time:.1f} s\n"
                f"Oil Rate: {latest_oil:.1f} bbl/hr\n"
                f"WHP: {latest_whp:.1f} psi\n"
                f"BHP: {latest_bhp:.1f} psi\n"
                f"Reservoir: {latest_res:.1f} bar\n"
                f"Choke Opening: {latest_choke:.1f}%\n"
                f"Choke Command: {choke_cmd:.1f}%"
            )

        return (line_choke, line_oil, line_whp, line_bhp, line_res, line_cmd, stats_text)

    # Set up animation
    anim = animation.FuncAnimation(fig, update, interval=100, blit=False, repeat=False)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()