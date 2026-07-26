#!/usr/bin/env python3
"""
Validation and Calibration Script for Autonomous Choke Control Simulator.

Compares simulator outputs against reference dataset:
`c5c8d485-e827-4cd6-a3f3-631921a2bfd3Autonomous_Choke_Control_Simulated_Dataset.csv`

Fine-tunes reservoir, tubing, and choke orifice model parameters.
"""

import os
import yaml
import pandas as pd
import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt

from simulator.simulation import Simulator
from simulator.config import load_config

# Unit Conversion Constants
PSI_TO_BAR = 0.0689475729
BAR_TO_PSI = 14.5037738
M3S_TO_BBL_HR = 22643.4
BBL_HR_TO_M3S = 1.0 / 22643.4

DATASET_PATH = "c5c8d485-e827-4cd6-a3f3-631921a2bfd3Autonomous_Choke_Control_Simulated_Dataset.csv"
CONFIG_PATH = "configs/default.yaml"

def run_simulation_with_params(params, df_ref, dt_step=3600.0):
    """
    params: [Pr_bar, PI_day_bar, rho_mix, K_tub, A_max, exponent, Cd, K_flow, reservoir_volume]
    """
    Pr_bar, PI, rho_mix, K_tub, A_max, exponent, Cd, K_flow, res_vol = params
    
    cfg = load_config(CONFIG_PATH)
    cfg.reservoir.pressure = float(Pr_bar)
    cfg.reservoir.productivity_index = float(PI)
    cfg.reservoir.volume = float(res_vol)
    cfg.fluid.density = float(rho_mix)
    cfg.fluid.water_cut = 0.0  # pure oil assumption matching dataset liquid rates
    cfg.well.friction_coefficient = float(K_tub)
    cfg.choke.max_area = float(A_max)
    cfg.choke.exponent = float(exponent)
    cfg.choke.cd = float(Cd)
    cfg.surface.flowline_coefficient = float(K_flow)

    sim = Simulator(CONFIG_PATH)
    sim.config = cfg
    sim.dt = dt_step
    sim.reset()
    
    steps_per_hour = max(1, int(3600 / dt_step))
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
        
    return pd.DataFrame(sim_records)

def compute_metrics(df_ref, df_sim):
    """Compute RMSE, MAE, R2 for key output variables."""
    metrics = {}
    for col in ['OilRate_bbl_hr', 'WHP_psi', 'BHP_psi']:
        y_true = df_ref[col].values
        y_pred = df_sim[col].values
        rmse = np.sqrt(np.mean((y_true - y_pred) ** 2))
        mae = np.mean(np.abs(y_true - y_pred))
        y_mean = np.mean(y_true)
        ss_res = np.sum((y_true - y_pred) ** 2)
        ss_tot = np.sum((y_true - y_mean) ** 2) if y_mean != 0 else 1.0
        r2 = 1.0 - (ss_res / ss_tot)
        metrics[col] = {'RMSE': rmse, 'MAE': mae, 'R2': r2}
    return metrics

eval_count = 0
def objective_function(params, df_ref):
    global eval_count
    eval_count += 1
    try:
        df_sim = run_simulation_with_params(params, df_ref, dt_step=3600.0)
        
        # Absolute RMSE normalized by mean target
        rmse_oil = np.sqrt(np.mean((df_ref['OilRate_bbl_hr'].values - df_sim['OilRate_bbl_hr'].values) ** 2))
        rmse_whp = np.sqrt(np.mean((df_ref['WHP_psi'].values - df_sim['WHP_psi'].values) ** 2))
        rmse_bhp = np.sqrt(np.mean((df_ref['BHP_psi'].values - df_sim['BHP_psi'].values) ** 2))
        
        loss = (rmse_oil / 120.0) + (rmse_whp / 250.0) + (rmse_bhp / 3000.0)
        if eval_count % 50 == 0:
            print(f"Eval {eval_count:4d}: Loss = {loss:.6f} | Oil RMSE={rmse_oil:.2f} bbl/hr, WHP RMSE={rmse_whp:.2f} psi, BHP RMSE={rmse_bhp:.2f} psi")
        if np.isnan(loss) or np.isinf(loss):
            return 1e9
        return loss
    except Exception as e:
        return 1e9

def main():
    print("=== Loading Reference Dataset ===")
    df_ref = pd.read_csv(DATASET_PATH)
    print(f"Dataset loaded successfully: {len(df_ref)} rows.")

    # Initial guess tuned for physical match:
    # [Pr_bar, PI_day_bar, rho_mix, K_tub, A_max, exponent, Cd, K_flow, reservoir_volume]
    x0 = [215.0, 800.0, 635.0, 1500.0, 0.00016, 0.55, 0.82, 5000.0, 3e7]
    
    print("\n=== Running Fine-Tuning Parameter Optimization ===")
    res = minimize(
        objective_function,
        x0,
        args=(df_ref,),
        method='Nelder-Mead',
        options={'maxiter': 600, 'disp': True}
    )

    best_params = res.x
    print("\nOptimal Calibrated Parameters:")
    print(f"  Pr (Reservoir Pressure): {best_params[0]:.2f} bar ({best_params[0]*BAR_TO_PSI:.1f} psi)")
    print(f"  Productivity Index:     {best_params[1]:.2f} m3/day/bar")
    print(f"  Multiphase Density:     {best_params[2]:.2f} kg/m3")
    print(f"  Tubing Friction K_tub:  {best_params[3]:.2e}")
    print(f"  Choke Max Area A_max:   {best_params[4]:.6f} m2")
    print(f"  Choke Exponent n:       {best_params[5]:.4f}")
    print(f"  Choke Cd:               {best_params[6]:.4f}")
    print(f"  Flowline K_flow:        {best_params[7]:.2e}")
    print(f"  Reservoir Volume:       {best_params[8]:.2e} m3")

    df_sim_calib = run_simulation_with_params(best_params, df_ref, dt_step=3600.0)
    calib_metrics = compute_metrics(df_ref, df_sim_calib)

    print("\n=== Final Calibrated Simulator Metrics ===")
    for var, m in calib_metrics.items():
        print(f"  {var:15s} -> RMSE: {m['RMSE']:8.3f}, MAE: {m['MAE']:8.3f}, R2: {m['R2']:8.3f}")

    # Update default.yaml
    with open(CONFIG_PATH, "r") as f:
        yaml_data = yaml.safe_load(f)

    yaml_data['reservoir']['pressure'] = float(best_params[0])
    yaml_data['reservoir']['productivity_index'] = float(best_params[1])
    yaml_data['reservoir']['volume'] = float(best_params[8])
    yaml_data['fluid']['density'] = float(best_params[2])
    yaml_data['fluid']['water_cut'] = 0.0
    yaml_data['well']['friction_coefficient'] = float(best_params[3])
    yaml_data['choke']['max_area'] = float(best_params[4])
    yaml_data['choke']['exponent'] = float(best_params[5])
    yaml_data['choke']['cd'] = float(best_params[6])
    yaml_data['surface']['flowline_coefficient'] = float(best_params[7])

    with open(CONFIG_PATH, "w") as f:
        yaml.dump(yaml_data, f, default_flow_style=False)
    print(f"\nUpdated {CONFIG_PATH} with calibrated parameters.")

    # Plot Comparison
    os.makedirs("plots", exist_ok=True)
    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

    # Plot 1: Oil Rate
    axes[0].plot(df_ref['Time_hr'], df_ref['OilRate_bbl_hr'], 'k-', label='Reference Dataset', linewidth=2)
    axes[0].plot(df_sim_calib['Time_hr'], df_sim_calib['OilRate_bbl_hr'], 'g--', label='Calibrated Simulator', linewidth=2)
    axes[0].set_ylabel('Oil Rate (bbl/hr)')
    axes[0].legend()
    axes[0].set_title('Autonomous Choke Controller Simulator: Calibrated vs Dataset')
    axes[0].grid(True)

    # Plot 2: WHP
    axes[1].plot(df_ref['Time_hr'], df_ref['WHP_psi'], 'k-', label='Reference Dataset', linewidth=2)
    axes[1].plot(df_sim_calib['Time_hr'], df_sim_calib['WHP_psi'], 'g--', label='Calibrated Simulator', linewidth=2)
    axes[1].set_ylabel('Wellhead Pressure (psi)')
    axes[1].legend()
    axes[1].grid(True)

    # Plot 3: BHP
    axes[2].plot(df_ref['Time_hr'], df_ref['BHP_psi'], 'k-', label='Reference Dataset', linewidth=2)
    axes[2].plot(df_sim_calib['Time_hr'], df_sim_calib['BHP_psi'], 'g--', label='Calibrated Simulator', linewidth=2)
    axes[2].set_ylabel('Bottom Hole Pressure (psi)')
    axes[2].set_xlabel('Time (hours)')
    axes[2].legend()
    axes[2].grid(True)

    plt.tight_layout()
    plot_path = "plots/dataset_calibration_comparison.png"
    plt.savefig(plot_path, dpi=150)
    plt.close()
    print(f"Calibration comparison plot saved to {plot_path}.")

if __name__ == "__main__":
    main()
