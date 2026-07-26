#!/usr/bin/env python3
"""
Controller Benchmark & Evaluation Suite for Autonomous Choke Control System.

Compares:
1. Uncontrolled / Fixed Choke Baseline (30%)
2. Rule-Based Heuristic Controller
3. Industrial PID Controller

Across 3 operational benchmark scenarios:
- Scenario A: Setpoint Step Tracking (100 -> 140 -> 110 -> 130 bbl/hr)
- Scenario B: Separator Pressure Disturbance Rejection (+5 bar / -4 bar surges)
- Scenario C: Dynamic Water Cut Breakthrough (0% -> 25%)

Logs diagnostic info (solver convergence, mass balance residual) and saves comparison plots.
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

def run_scenario(controller_type, scenario_name, duration_hours=12.0, dt=1.0):
    sim = Simulator(CONFIG_PATH)
    sim.dt = dt
    sim.reset()
    
    total_steps = int(duration_hours * 3600 / dt)
    
    # Instantiate controller
    if controller_type == "fixed":
        ctrl = None
    elif controller_type == "rule_based":
        ctrl = RuleBasedChokeController(target_oil_bbl_hr=100.0, min_whp_psi=210.0)
        ctrl.reset()
    elif controller_type == "pid":
        ctrl = PIDChokeController(kp=1.2, ki=0.08, kd=0.3, target_oil_bbl_hr=100.0, min_whp_psi=210.0, dt=dt)
        ctrl.reset()
    elif controller_type == "mpc":
        ctrl = ModelPredictiveChokeController(config_path=CONFIG_PATH, horizon=6, dt_control=60.0, target_oil_bbl_hr=100.0, min_whp_psi=210.0)
        ctrl.reset()
    elif controller_type == "rl":
        ctrl = RLChokeController(model_path="models/rl_choke_policy.npz", target_oil_bbl_hr=100.0, min_whp_psi=210.0)
        ctrl.reset()
    else:
        raise ValueError(f"Unknown controller type: {controller_type}")

    records = []
    choke_cmd = 30.0
    
    for i in range(total_steps):
        t_sec = i * dt
        t_hr = t_sec / 3600.0
        
        # Scenario profile configuration
        if scenario_name == "Scenario A (Step Setpoints)":
            if t_hr < 3.0:
                target_oil = 100.0
            elif t_hr < 6.0:
                target_oil = 140.0
            elif t_hr < 9.0:
                target_oil = 110.0
            else:
                target_oil = 130.0
            if ctrl:
                ctrl.target_oil_bbl_hr = target_oil
                
        elif scenario_name == "Scenario B (Separator Surges)":
            target_oil = 120.0
            if ctrl:
                ctrl.target_oil_bbl_hr = target_oil
            # Pressure disturbance on separator
            if 3.0 <= t_hr < 5.0:
                sim.state.separator_pressure = 25.0  # +5 bar surge
            elif 7.0 <= t_hr < 9.0:
                sim.state.separator_pressure = 16.0  # -4 bar drop
            else:
                sim.state.separator_pressure = 20.0
                
        elif scenario_name == "Scenario C (Water Breakthrough)":
            target_oil = 110.0
            if ctrl:
                ctrl.target_oil_bbl_hr = target_oil
            # Water cut rises from 0% to 25% over 12 hours
            sim.state.water_cut = min(0.25, (t_hr / 12.0) * 0.25)

        # Compute Action (MPC executes every 60 seconds, PID/Rule-Based/RL every step)
        if controller_type == "fixed":
            choke_cmd = 30.0
        elif controller_type == "mpc":
            if i % int(60.0 / dt) == 0:
                obs = sim._get_observation()
                info = {
                    "true_state": sim._state_to_dict(),
                    "time": sim.state.time
                }
                choke_cmd = ctrl.compute_action(obs, info=info)
            else:
                choke_cmd = ctrl.current_choke
        else:
            obs = sim._get_observation()
            choke_cmd = ctrl.compute_action(obs)

        # Step Simulator
        obs, info = sim.step_with_info(choke_cmd)
        
        # Log data every 60 seconds
        if i % int(60 / dt) == 0:
            oil_bbl_hr = sim.state.oil_rate * M3S_TO_BBL_HR
            whp_psi = sim.state.Pwh * BAR_TO_PSI
            bhp_psi = sim.state.Pwf * BAR_TO_PSI
            flp_psi = sim.state.separator_pressure * BAR_TO_PSI
            
            records.append({
                'Time_hr': t_hr,
                'Controller': controller_type.upper(),
                'Target_Oil': target_oil if 'target_oil' in locals() else 100.0,
                'Choke_pct': sim.state.opening_actual,
                'Choke_cmd': choke_cmd,
                'OilRate_bbl_hr': oil_bbl_hr,
                'WHP_psi': whp_psi,
                'FLP_psi': flp_psi,
                'BHP_psi': bhp_psi,
                'Pr_bar': sim.state.Pr,
                'WaterCut_pct': sim.state.water_cut * 100.0,
                'Converged': info['flow_solver_converged'],
                'Residual': info['mass_balance_error'],
            })
            
    return pd.DataFrame(records)

def main():
    print("==========================================================================")
    print("   AUTONOMOUS CHOKE CONTROL SYSTEM: CONTROLLER BENCHMARK EVALUATION      ")
    print("==========================================================================")
    
    scenarios = [
        "Scenario A (Step Setpoints)",
        "Scenario B (Separator Surges)",
        "Scenario C (Water Breakthrough)"
    ]
    
    controllers = ["fixed", "rule_based", "pid", "mpc", "rl"]
    
    os.makedirs("plots", exist_ok=True)
    
    for sc_name in scenarios:
        print(f"\n---> Running Benchmark: {sc_name}")
        results = {}
        for ctrl_name in controllers:
            df = run_scenario(ctrl_name, sc_name, duration_hours=12.0, dt=5.0)
            results[ctrl_name] = df
            
            # Compute KPI Summary Metrics
            cum_oil = np.trapezoid(df['OilRate_bbl_hr'].values, df['Time_hr'].values)
            iae = np.trapezoid(np.abs(df['OilRate_bbl_hr'].values - df['Target_Oil'].values), df['Time_hr'].values)
            choke_wear = np.sum(np.abs(np.diff(df['Choke_pct'].values)))
            min_whp = np.min(df['WHP_psi'].values)
            all_converged = np.all(df['Converged'].values)
            max_residual = np.max(df['Residual'].values)
            
            print(f"  [{ctrl_name.upper():12s}] -> Cum Oil: {cum_oil:7.1f} bbl | IAE: {iae:6.1f} bbl·h | Wear: {choke_wear:5.1f}% | Min WHP: {min_whp:5.1f} psi | Solver Converged: {all_converged}")

        # Plot Scenario Results
        fig, axes = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
        colors = {'fixed': 'k', 'rule_based': 'b', 'pid': 'g', 'mpc': 'm', 'rl': 'c'}
        styles = {'fixed': ':', 'rule_based': '--', 'pid': '-.', 'mpc': '-', 'rl': '-'}
        
        # Plot 0: Choke Opening
        for c_name, df_res in results.items():
            axes[0].plot(df_res['Time_hr'], df_res['Choke_pct'], color=colors[c_name], linestyle=styles[c_name], label=f"{c_name.upper()}", linewidth=2)
        axes[0].set_ylabel('Choke Opening (%)')
        axes[0].set_title(f"Controller Performance Comparison: {sc_name}")
        axes[0].legend(loc='upper right')
        axes[0].grid(True)

        # Plot 1: Oil Rate & Target
        # Plot Target setpoint from PID run
        df_pid = results['pid']
        axes[1].plot(df_pid['Time_hr'], df_pid['Target_Oil'], 'r--', label='Setpoint Target', linewidth=1.5)
        for c_name, df_res in results.items():
            axes[1].plot(df_res['Time_hr'], df_res['OilRate_bbl_hr'], color=colors[c_name], linestyle=styles[c_name], label=f"{c_name.upper()}", linewidth=2)
        axes[1].set_ylabel('Oil Rate (bbl/hr)')
        axes[1].legend(loc='upper right')
        axes[1].grid(True)

        # Plot 2: WHP
        for c_name, df_res in results.items():
            axes[2].plot(df_res['Time_hr'], df_res['WHP_psi'], color=colors[c_name], linestyle=styles[c_name], label=f"{c_name.upper()}", linewidth=2)
        axes[2].axhline(210.0, color='r', linestyle=':', label='Min WHP Safety Limit (210 psi)')
        axes[2].set_ylabel('Wellhead Pressure (psi)')
        axes[2].legend(loc='upper right')
        axes[2].grid(True)

        # Plot 3: BHP
        for c_name, df_res in results.items():
            axes[3].plot(df_res['Time_hr'], df_res['BHP_psi'], color=colors[c_name], linestyle=styles[c_name], label=f"{c_name.upper()}", linewidth=2)
        axes[3].set_ylabel('Bottom Hole Pressure (psi)')
        axes[3].set_xlabel('Time (hours)')
        axes[3].legend(loc='upper right')
        axes[3].grid(True)

        plt.tight_layout()
        plot_name = f"plots/benchmark_{sc_name.split()[0].lower()}_{sc_name.split()[1].lower()}.png"
        plt.savefig(plot_name, dpi=150)
        plt.close()
        print(f"  Saved benchmark plot to {plot_name}")

if __name__ == "__main__":
    main()
