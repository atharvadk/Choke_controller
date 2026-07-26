#!/usr/bin/env python3
"""
Master Reproduction Script for Autonomous Choke Control System.

Executes the entire end-to-end simulation, validation, RL training, controller benchmarking,
trajectory inspection, Monte Carlo statistical analysis, and physics ablation suite with a single command.

Usage:
    python3 run_all_benchmarks.py
"""

import sys
import time
import subprocess

def run_step(script_name, description):
    print(f"\n==========================================================================")
    print(f" STEP: {description}")
    print(f" Executing: python3 {script_name}")
    print(f"==========================================================================")
    t0 = time.time()
    result = subprocess.run([sys.executable, script_name], check=True)
    t1 = time.time()
    print(f"✓ Completed {script_name} in {t1 - t0:.2f} seconds.")

def main():
    print("""
==============================================================================
   AUTONOMOUS CHOKE CONTROL SYSTEM: MASTER REPRODUCTION SUITE
==============================================================================
    """)

    # 1. Dataset Validation & Physical Calibration
    run_step("validate_and_plot.py", "Simulator Validation & Calibration against Reference Dataset")

    # 2. Trajectory Inspection & Constraint Audit
    run_step("run_trajectory_analysis.py", "Controller Trajectory Inspection & Constraint Violation Audit")

    # 3. RL Policy Training
    run_step("train_rl.py", "Reinforcement Learning (Actor-Critic) Policy Training")

    # 4. Multi-Controller Benchmark Evaluation (Fixed, Rule-Based, PID, NMPC, RL)
    run_step("benchmark.py", "5-Controller Comparative Benchmark (Scenarios A, B, C)")

    # 5. Monte Carlo Statistical Significance Evaluation
    run_step("run_monte_carlo_experiments.py", "Monte Carlo Parameter Uncertainty & Student's t-test Evaluation")

    # 6. Physics Ablation & Decision Latency Benchmark
    run_step("run_ablation_and_sensitivity.py", "Physics Ablation Study & Decision Latency Benchmark")

    print("""
==============================================================================
 ALL EXPERIMENTS & BENCHMARKS REPRODUCED SUCCESSFULLY!
 All figures saved to: plots/
 Model weights saved to: models/
==============================================================================
    """)

if __name__ == "__main__":
    main()
