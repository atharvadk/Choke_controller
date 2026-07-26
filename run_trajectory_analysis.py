#!/usr/bin/env python3
"""
Trajectory Inspection & Constraint Audit Suite.

Audits whether controllers make distinct decisions and checks for constraint violations:
- Minimum WHP Safety Limit Violations (Pwh < 210 psi)
- Maximum Drawdown Violations (Pr - Pwf > 35 bar)
- Actuator Slew Rate Violations (|Δu| > max_slew)

Generates overlay plots in plots/trajectory_inspection_scenario_a.png.
"""

import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from simulator.simulation import Simulator
from controllers.rule_based import RuleBasedChokeController
from controllers.pid_controller import PIDChokeController
from controllers.mpc_controller import ModelPredictiveChokeController
from controllers.rl_controller import RLChokeController

PSI_TO_BAR = 0.0689475729
BAR_TO_PSI = 14.5037738
M3S_TO_BBL_HR = 22643.4
CONFIG_PATH = "configs/default.yaml"

def run_trajectory_inspection(dt=2.0, duration_hours=12.0):
    sc_name = "Scenario A (Step Setpoints)"
    controllers = ["rule_based", "pid", "mpc", "rl"]
    
    trajectory_data = {}
    constraint_audit = {}

    for ctrl_name in controllers:
        sim = Simulator(CONFIG_PATH)
        sim.dt = dt
        sim.reset()
        
        if ctrl_name == "rule_based":
            ctrl = RuleBasedChokeController(target_oil_bbl_hr=100.0, min_whp_psi=210.0)
        elif ctrl_name == "pid":
            ctrl = PIDChokeController(kp=1.2, ki=0.08, kd=0.3, target_oil_bbl_hr=100.0, min_whp_psi=210.0, dt=dt)
        elif ctrl_name == "mpc":
            ctrl = ModelPredictiveChokeController(config_path=CONFIG_PATH, horizon=6, dt_control=60.0, target_oil_bbl_hr=100.0, min_whp_psi=210.0)
        elif ctrl_name == "rl":
            ctrl = RLChokeController(model_path="models/rl_choke_policy.npz", target_oil_bbl_hr=100.0, min_whp_psi=210.0)
            
        ctrl.reset()
        total_steps = int(duration_hours * 3600 / dt)
        
        records = []
        choke_cmd = 30.0
        
        whp_violations = 0
        drawdown_violations = 0
        slew_violations = 0
        prev_choke = 30.0

        for i in range(total_steps):
            t_sec = i * dt
            t_hr = t_sec / 3600.0
            
            # Target setpoint profile
            if t_hr < 3.0:
                target_oil = 100.0
            elif t_hr < 6.0:
                target_oil = 140.0
            elif t_hr < 9.0:
                target_oil = 110.0
            else:
                target_oil = 130.0
            ctrl.target_oil_bbl_hr = target_oil

            # Compute Action
            obs = sim._get_observation()
            if ctrl_name == "mpc":
                if i % int(60.0 / dt) == 0:
                    info = {"true_state": sim._state_to_dict(), "time": sim.state.time}
                    choke_cmd = ctrl.compute_action(obs, info=info)
                else:
                    choke_cmd = ctrl.current_choke
            else:
                choke_cmd = ctrl.compute_action(obs)

            # Step Simulator
            obs, info = sim.step_with_info(choke_cmd)
            
            actual_choke = sim.state.opening_actual
            whp_psi = sim.state.Pwh * BAR_TO_PSI
            drawdown_bar = sim.state.Pr - sim.state.Pwf
            delta_choke = abs(actual_choke - prev_choke)
            prev_choke = actual_choke
            
            # Audit violations
            if whp_psi < 209.5:
                whp_violations += 1
            if drawdown_bar > 35.0:
                drawdown_violations += 1
            if delta_choke > (1.0 * (dt / 1.0) + 1e-4):
                slew_violations += 1
                
            records.append({
                'Time_hr': t_hr,
                'Choke_actual': actual_choke,
                'Choke_cmd': choke_cmd,
                'OilRate_bbl_hr': sim.state.oil_rate * M3S_TO_BBL_HR,
                'WHP_psi': whp_psi,
                'BHP_psi': sim.state.Pwf * BAR_TO_PSI,
                'Target_Oil': target_oil
            })

        trajectory_data[ctrl_name] = pd.DataFrame(records)
        constraint_audit[ctrl_name] = {
            'WHP_Violations': whp_violations,
            'Drawdown_Violations': drawdown_violations,
            'Slew_Violations': slew_violations
        }

    return trajectory_data, constraint_audit

def main():
    print("==========================================================================")
    print("      TRAJECTORY INSPECTION & CONSTRAINT ACTIVITY AUDIT SUITE             ")
    print("==========================================================================")

    data, audit = run_trajectory_inspection(dt=2.0)

    print("\n---> Constraint Violation Audit Results:")
    for ctrl, res in audit.items():
        print(f"  [{ctrl.upper():12s}] -> WHP Violations (<210 psi): {res['WHP_Violations']:3d} | Drawdown Violations (>35 bar): {res['Drawdown_Violations']:3d} | Slew Violations: {res['Slew_Violations']:3d}")

    # Generate Detailed Multi-Panel Inspection Plot
    os.makedirs("plots", exist_ok=True)
    fig, axes = plt.subplots(3, 1, figsize=(11, 10), sharex=True)

    colors = {'rule_based': 'b', 'pid': 'g', 'mpc': 'm', 'rl': 'c'}
    styles = {'rule_based': '--', 'pid': '-.', 'mpc': '-', 'rl': '-'}

    # Plot 1: Choke Trajectory Overlays
    for ctrl, df in data.items():
        axes[0].plot(df['Time_hr'], df['Choke_actual'], color=colors[ctrl], linestyle=styles[ctrl], label=ctrl.upper(), linewidth=2)
    axes[0].set_ylabel('Choke Opening (%)')
    axes[0].set_title('Controller Decision Trajectory Inspection (Scenario A)')
    axes[0].legend(loc='upper right')
    axes[0].grid(True)

    # Plot 2: Oil Rate Response Overlays
    df_ref = data['rule_based']
    axes[1].plot(df_ref['Time_hr'], df_ref['Target_Oil'], 'r--', label='Target Setpoint', linewidth=1.5)
    for ctrl, df in data.items():
        axes[1].plot(df['Time_hr'], df['OilRate_bbl_hr'], color=colors[ctrl], linestyle=styles[ctrl], label=ctrl.upper(), linewidth=2)
    axes[1].set_ylabel('Oil Rate (bbl/hr)')
    axes[1].legend(loc='upper right')
    axes[1].grid(True)

    # Plot 3: Wellhead Pressure Overlays & Safety Boundary
    for ctrl, df in data.items():
        axes[2].plot(df['Time_hr'], df['WHP_psi'], color=colors[ctrl], linestyle=styles[ctrl], label=ctrl.upper(), linewidth=2)
    axes[2].axhline(210.0, color='r', linestyle=':', label='Min WHP Safety Boundary (210 psi)', linewidth=1.5)
    axes[2].set_ylabel('Wellhead Pressure (psi)')
    axes[2].set_xlabel('Time (hours)')
    axes[2].legend(loc='upper right')
    axes[2].grid(True)

    plt.tight_layout()
    plot_path = "plots/trajectory_inspection_scenario_a.png"
    plt.savefig(plot_path, dpi=150)
    plt.close()
    print(f"\nTrajectory overlay plot saved to {plot_path}.")

if __name__ == "__main__":
    main()
