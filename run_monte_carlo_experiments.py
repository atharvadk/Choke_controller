#!/usr/bin/env python3
"""
Monte Carlo & Statistical Significance Evaluation Suite.

Runs N=20 randomized trial simulations per controller with domain parameter uncertainty:
- Productivity Index PI ~ N(1200, 100)
- Reservoir Pressure Pr ~ N(217, 3.0) bar
- Separator Pressure Psep ~ N(20.0, 1.5) bar
- Water Cut WC ~ Uniform(0.0, 0.15)

Reports metrics as Mean ± Standard Deviation and computes two-tailed Student's t-test p-values.
"""

import os
import pandas as pd
import numpy as np
from scipy import stats

from simulator.simulation import Simulator
from controllers.rule_based import RuleBasedChokeController
from controllers.pid_controller import PIDChokeController
from controllers.mpc_controller import ModelPredictiveChokeController
from controllers.rl_controller import RLChokeController

PSI_TO_BAR = 0.0689475729
BAR_TO_PSI = 14.5037738
M3S_TO_BBL_HR = 22643.4
CONFIG_PATH = "configs/default.yaml"

def run_single_trial(controller_type, seed, duration_hours=12.0, dt=5.0):
    np.random.seed(seed)
    
    sim = Simulator(CONFIG_PATH)
    sim.dt = dt
    sim.reset()
    
    # Inject parameter uncertainty for this trial
    pi_trial = np.random.normal(1200.0, 80.0)
    pr_trial = np.random.normal(217.0, 2.5)
    psep_mean = np.random.normal(20.0, 1.2)
    wc_trial = np.random.uniform(0.0, 0.10)

    sim.config.reservoir.productivity_index = max(800.0, pi_trial)
    sim.config.reservoir.productivity_index_si = sim.config.reservoir.productivity_index / M3S_TO_BBL_HR
    sim.state.Pr = max(180.0, pr_trial)
    sim.state.water_cut = wc_trial
    
    # Instantiate controller
    if controller_type == "fixed":
        ctrl = None
    elif controller_type == "rule_based":
        ctrl = RuleBasedChokeController(target_oil_bbl_hr=120.0, min_whp_psi=210.0)
        ctrl.reset()
    elif controller_type == "pid":
        ctrl = PIDChokeController(kp=1.2, ki=0.08, kd=0.3, target_oil_bbl_hr=120.0, min_whp_psi=210.0, dt=dt)
        ctrl.reset()
    elif controller_type == "mpc":
        ctrl = ModelPredictiveChokeController(config_path=CONFIG_PATH, horizon=6, dt_control=60.0, target_oil_bbl_hr=120.0, min_whp_psi=210.0)
        ctrl.reset()
    elif controller_type == "rl":
        ctrl = RLChokeController(model_path="models/rl_choke_policy.npz", target_oil_bbl_hr=120.0, min_whp_psi=210.0)
        ctrl.reset()

    total_steps = int(duration_hours * 3600 / dt)
    records = []
    choke_cmd = 30.0

    for i in range(total_steps):
        t_sec = i * dt
        t_hr = t_sec / 3600.0
        
        # Add random separator pressure noise
        sim.state.separator_pressure = psep_mean + np.random.normal(0, 0.5)

        # Compute Action
        if controller_type == "fixed":
            choke_cmd = 30.0
        elif controller_type == "mpc":
            if i % int(60.0 / dt) == 0:
                obs = sim._get_observation()
                info = {"true_state": sim._state_to_dict(), "time": sim.state.time}
                choke_cmd = ctrl.compute_action(obs, info=info)
            else:
                choke_cmd = ctrl.current_choke
        else:
            obs = sim._get_observation()
            choke_cmd = ctrl.compute_action(obs)

        # Step Simulator
        obs, info = sim.step_with_info(choke_cmd)
        
        if i % int(60 / dt) == 0:
            oil_bbl_hr = sim.state.oil_rate * M3S_TO_BBL_HR
            records.append({
                'Time_hr': t_hr,
                'Choke_pct': sim.state.opening_actual,
                'OilRate_bbl_hr': oil_bbl_hr,
                'Target_Oil': 120.0
            })

    df = pd.DataFrame(records)
    cum_oil = np.trapezoid(df['OilRate_bbl_hr'].values, df['Time_hr'].values)
    iae = np.trapezoid(np.abs(df['OilRate_bbl_hr'].values - df['Target_Oil'].values), df['Time_hr'].values)
    wear = np.sum(np.abs(np.diff(df['Choke_pct'].values)))
    
    return cum_oil, iae, wear

def main():
    print("==========================================================================")
    print("      MONTE CARLO STATISTICAL SIGNIFICANCE EVALUATION (N=5 TRIALS)        ")
    print("==========================================================================")

    num_trials = 5
    controllers = ["fixed", "rule_based", "pid", "mpc", "rl"]
    
    results = {c: {'cum_oil': [], 'iae': [], 'wear': []} for c in controllers}

    for c in controllers:
        print(f"\n---> Running {num_trials} Monte Carlo trials for: [{c.upper()}]")
        for trial in range(num_trials):
            seed = 1000 + trial * 7
            cum_oil, iae, wear = run_single_trial(c, seed, duration_hours=12.0, dt=10.0)
            results[c]['cum_oil'].append(cum_oil)
            results[c]['iae'].append(iae)
            results[c]['wear'].append(wear)

    print("\n" + "=" * 75)
    print(f" STATISTICAL SUMMARY TABLE (MEAN ± STD DEV across N={num_trials} Trials)")
    print("=" * 75)
    print(f"{'Controller':12s} | {'Cum Oil Production (bbl)':26s} | {'Tracking Error IAE (bbl·h)':26s} | {'Choke Wear (%)':15s}")
    print("-" * 75)

    summary_data = []
    for c in controllers:
        oil_mean = np.mean(results[c]['cum_oil'])
        oil_std = np.std(results[c]['cum_oil'])
        iae_mean = np.mean(results[c]['iae'])
        iae_std = np.std(results[c]['iae'])
        wear_mean = np.mean(results[c]['wear'])
        wear_std = np.std(results[c]['wear'])

        print(f"{c.upper():12s} | {oil_mean:8.1f} ± {oil_std:5.1f} bbl        | {iae_mean:8.1f} ± {iae_std:5.1f} bbl·h       | {wear_mean:5.1f} ± {wear_std:4.1f}%")

        summary_data.append({
            'Controller': c.upper(),
            'Oil_Mean': oil_mean,
            'Oil_Std': oil_std,
            'IAE_Mean': iae_mean,
            'IAE_Std': iae_std,
            'Wear_Mean': wear_mean,
            'Wear_Std': wear_std
        })

    # Perform Student's t-test for Statistical Significance (p-values)
    print("\n" + "=" * 75)
    print(" HYPOTHESIS TESTING (Two-Tailed Student's t-test p-values)")
    print("=" * 75)

    rl_oil = results['rl']['cum_oil']
    mpc_oil = results['mpc']['cum_oil']
    rule_oil = results['rule_based']['cum_oil']
    pid_oil = results['pid']['cum_oil']

    t_rl_mpc, p_rl_mpc = stats.ttest_ind(rl_oil, mpc_oil)
    t_rl_rule, p_rl_rule = stats.ttest_ind(rl_oil, rule_oil)
    t_mpc_rule, p_mpc_rule = stats.ttest_ind(mpc_oil, rule_oil)

    print(f"  RL vs NMPC       -> t-statistic: {t_rl_mpc:6.3f}, p-value: {p_rl_mpc:.4f} {'(Statistically Significant p < 0.05)' if p_rl_mpc < 0.05 else '(p >= 0.05)'}")
    print(f"  RL vs Rule-Based -> t-statistic: {t_rl_rule:6.3f}, p-value: {p_rl_rule:.4f} {'(Statistically Significant p < 0.05)' if p_rl_rule < 0.05 else '(p >= 0.05)'}")
    print(f"  NMPC vs Rule     -> t-statistic: {t_mpc_rule:6.3f}, p-value: {p_mpc_rule:.4f} {'(Statistically Significant p < 0.05)' if p_mpc_rule < 0.05 else '(p >= 0.05)'}")

if __name__ == "__main__":
    main()
