#!/usr/bin/env python3
"""
Validation and Plotting Script for Autonomous Choke Control Simulator.

Runs the calibrated simulator against the reference dataset:
`c5c8d485-e827-4cd6-a3f3-631921a2bfd3Autonomous_Choke_Control_Simulated_Dataset.csv`

Computes metrics (RMSE, MAE) and generates comparison plots.
"""

import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from simulator.simulation import Simulator
from simulator.config import load_config

# Unit Conversion Constants
PSI_TO_BAR = 0.0689475729
BAR_TO_PSI = 14.5037738
M3S_TO_BBL_HR = 22643.4

DATASET_PATH = "c5c8d485-e827-4cd6-a3f3-631921a2bfd3Autonomous_Choke_Control_Simulated_Dataset.csv"
CONFIG_PATH = "configs/default.yaml"

def main():
    print("=== Loading Reference Dataset ===")
    df_ref = pd.read_csv(DATASET_PATH)
    print(f"Dataset loaded successfully: {len(df_ref)} rows.")

    print("\n=== Running Calibrated Simulator ===")
    sim = Simulator(CONFIG_PATH)
    dt_step = 10.0  # 10s timestep
    sim.dt = dt_step
    sim.reset()
    steps_per_hour = int(3600 / dt_step)

    sim_records = []
    for idx, row in df_ref.iterrows():
        t_hr = row['Time_hr']
        choke_target = row['Choke_pct']
        flp_psi = row['FLP_psi']
        
        sim.state.separator_pressure = flp_psi * PSI_TO_BAR
        
        for _ in range(steps_per_hour):
            obs = sim.step(choke_target)
            
        oil_bbl_hr = sim.state.oil_rate * M3S_TO_BBL_HR
        whp_psi = sim.state.Pwh * BAR_TO_PSI
        bhp_psi = sim.state.Pwf * BAR_TO_PSI
        
        sim_records.append({
            'Time_hr': t_hr,
            'Choke_pct': sim.state.opening_actual,
            'OilRate_bbl_hr': oil_bbl_hr,
            'WHP_psi': whp_psi,
            'FLP_psi': flp_psi,
            'BHP_psi': bhp_psi,
            'Pr_bar': sim.state.Pr,
        })
        
    df_sim = pd.DataFrame(sim_records)

    print("\n=== Validation Metrics (Simulator vs Reference Dataset) ===")
    metrics = {}
    for col in ['OilRate_bbl_hr', 'WHP_psi', 'BHP_psi']:
        y_true = df_ref[col].values
        y_pred = df_sim[col].values
        rmse = np.sqrt(np.mean((y_true - y_pred) ** 2))
        mae = np.mean(np.abs(y_true - y_pred))
        rel_err = (mae / np.mean(y_true)) * 100.0
        metrics[col] = {'RMSE': rmse, 'MAE': mae, 'RelErr_pct': rel_err}
        print(f"  {col:15s} -> RMSE: {rmse:8.3f}, MAE: {mae:8.3f}, Mean Rel Error: {rel_err:6.2f}%")

    # Generate plots
    os.makedirs("plots", exist_ok=True)
    fig, axes = plt.subplots(4, 1, figsize=(10, 12), sharex=True)

    # Plot 0: Choke Opening
    axes[0].plot(df_ref['Time_hr'], df_ref['Choke_pct'], 'b-', label='Choke Opening (%)', linewidth=2)
    axes[0].set_ylabel('Choke (%)')
    axes[0].legend(loc='upper right')
    axes[0].set_title('Autonomous Choke Controller Simulator: Calibrated Model vs Reference Dataset')
    axes[0].grid(True)

    # Plot 1: Oil Rate
    axes[1].plot(df_ref['Time_hr'], df_ref['OilRate_bbl_hr'], 'k-', label='Reference Dataset', linewidth=2)
    axes[1].plot(df_sim['Time_hr'], df_sim['OilRate_bbl_hr'], 'g--', label='Calibrated Simulator', linewidth=2)
    axes[1].set_ylabel('Oil Rate (bbl/hr)')
    axes[1].legend(loc='upper right')
    axes[1].grid(True)

    # Plot 2: WHP
    axes[2].plot(df_ref['Time_hr'], df_ref['WHP_psi'], 'k-', label='Reference Dataset', linewidth=2)
    axes[2].plot(df_sim['Time_hr'], df_sim['WHP_psi'], 'g--', label='Calibrated Simulator', linewidth=2)
    axes[2].set_ylabel('Wellhead Pressure (psi)')
    axes[2].legend(loc='upper right')
    axes[2].grid(True)

    # Plot 3: BHP
    axes[3].plot(df_ref['Time_hr'], df_ref['BHP_psi'], 'k-', label='Reference Dataset', linewidth=2)
    axes[3].plot(df_sim['Time_hr'], df_sim['BHP_psi'], 'g--', label='Calibrated Simulator', linewidth=2)
    axes[3].set_ylabel('Bottom Hole Pressure (psi)')
    axes[3].set_xlabel('Time (hours)')
    axes[3].legend(loc='upper right')
    axes[3].grid(True)

    plt.tight_layout()
    plot_path = "plots/dataset_calibration_comparison.png"
    plt.savefig(plot_path, dpi=150)
    plt.close()
    print(f"\nCalibration comparison plot saved to {plot_path}.")

if __name__ == "__main__":
    main()
