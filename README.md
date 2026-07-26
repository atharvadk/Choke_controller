# Autonomous Choke Control System: Dynamic Digital Twin & Benchmark Suite

[![Python Version](https://img.shields.io/badge/python-3.10%2B-blue.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

A reduced-order physics digital twin and controller benchmarking platform for Autonomous Production Choke Control in oil and gas wells. The simulator models coupled reservoir inflow (IPR), wellbore hydraulics, choke orifice dynamics, flowline backpressure, and dynamic fluid storage lag.

Includes 5 control strategies: Fixed Choke, Rule-Based Heuristic, Industrial PID with Anti-Windup, Non-Linear Model Predictive Control (NMPC), and an Actor-Critic Reinforcement Learning (RL) Agent.

---

## System Architecture & Overview

```text
               ┌────────────────────────────────────────────────────────┐
               │           AUTONOMOUS CHOKE CONTROL SYSTEM              │
               └───────────────────────────┬────────────────────────────┘
                                           │
           ┌─────────────────────────────┴─────────────────────────────┐
           ▼                                                           ▼
┌─────────────────────────┐                                 ┌─────────────────────────┐
│   PHYSICS DIGITAL TWIN  │                                 │   CONTROLLER SUPPORT    │
├─────────────────────────┤                                 ├─────────────────────────┤
│ • Reservoir Inflow (IPR)│                                 │ • Fixed Baseline (30%)  │
│ • Perforation Loss      │ ◄── [ Choke Command u(t) ] ──── │ • Rule-Based Heuristic  │
│ • Tubing Hydrostatics   │                                 │ • PID + Anti-Windup     │
│ • Orifice Flow Solver   │ ─── [ Sensed Obs obs(t) ] ────► │ • Non-Linear MPC (NMPC) │
│ • Dynamic Storage Tau   │                                 │ • Actor-Critic RL Agent │
└─────────────────────────┘                                 └─────────────────────────┘
           │                                                           │
           └─────────────────────────────┬─────────────────────────────┘
                                         │
                                         ▼
                               ┌─────────────────────────────┐
                               │  BENCHMARK & AUDIT SUITE    │
                               ├─────────────────────────────┤
                               │ • Reference Calibration     │
                               │ • Monte Carlo Trials (N=20) │
                               │ • Constraint Audit (WHP)    │
                               │ • Physics Ablation Study    │
                               └─────────────────────────────┘
```

---

## Quick Start & Installation

### 1. Clone & Setup Environment

```bash
# Clone the repository
git clone https://github.com/your-org/autonomous-choke-control.git
cd autonomous-choke-control

# Create and activate Python virtual environment
python3 -m venv .venv
source .venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

### 2. Single-Command Reproduction

To run the complete end-to-end dataset calibration, RL training, 5-controller benchmark, Monte Carlo statistical analysis, constraint audit, and physics ablation suite:

```bash
python3 run_all_benchmarks.py
```

This command generates all figures in `plots/`, model weights in `models/`, and prints statistical metrics to the console.

---

## Example Python Usage

### Running the Simulator & PID Controller

```python
from simulator.simulation import Simulator
from controllers.pid_controller import PIDChokeController

# Load simulator from default config
sim = Simulator("configs/default.yaml")
sim.reset()

# Instantiate PID controller targeting 120 bbl/hr oil rate
controller = PIDChokeController(target_oil_bbl_hr=120.0, min_whp_psi=210.0, dt=1.0)
controller.reset()

# Simulation loop over 1 hour
for t in range(3600):
    obs = sim._get_observation()
    choke_cmd = controller.compute_action(obs)
    
    # Step simulator with diagnostic tracking
    obs, info = sim.step_with_info(choke_cmd)
    
    if t % 600 == 0:
        print(f"Time {t}s -> Choke: {sim.state.opening_actual:.1f}%, Oil Rate: {sim.state.oil_rate*22643.4:.1f} bbl/hr, WHP: {sim.state.Pwh*14.5038:.1f} psi")
```

---

## Benchmark Performance Results

Evaluation across 12-hour simulation horizons ($dt = 5.0\text{s}$, $8,640\text{ steps}$ per scenario):

| Scenario | Control Strategy | Cum. Oil Produced | Tracking Error (IAE) | Choke Wear ($\sum \|\Delta u\|$) | Min WHP Recorded | Solver Converged |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: |
| **Scenario A** | Fixed Choke (30%) | $945.1\text{ bbl}$ | $493.0\text{ bbl}\cdot\text{h}$ | $0.0\%$ | $305.2\text{ psi}$ | True |
| *(Step Setpoints)* | Rule-Based | $1203.4\text{ bbl}$ | $234.7\text{ bbl}\cdot\text{h}$ | $69.8\%$ | $305.3\text{ psi}$ | True |
| | PID Controller | $1190.9\text{ bbl}$ | $247.1\text{ bbl}\cdot\text{h}$ | $118.9\%$ | $305.3\text{ psi}$ | True |
| | **NMPC** | $1204.7\text{ bbl}$ | **$233.4\text{ bbl}\cdot\text{h}$** | **$69.3\%$** | $305.4\text{ psi}$ | True |
| | **RL (Actor-Critic)** | **$1223.8\text{ bbl}$** | $235.7\text{ bbl}\cdot\text{h}$ | $69.7\%$ | $305.3\text{ psi}$ | True |
| **Scenario B** | Fixed Choke (30%) | $912.7\text{ bbl}$ | $525.3\text{ bbl}\cdot\text{h}$ | $0.0\%$ | $294.2\text{ psi}$ | True |
| *(Separator Surges)*| Rule-Based | $1168.6\text{ bbl}$ | $285.1\text{ bbl}\cdot\text{h}$ | $131.9\%$ | $305.3\text{ psi}$ | True |
| | PID Controller | $1144.5\text{ bbl}$ | $293.9\text{ bbl}\cdot\text{h}$ | $163.2\%$ | $305.3\text{ psi}$ | True |
| | **NMPC** | $1168.2\text{ bbl}$ | **$286.9\text{ bbl}\cdot\text{h}$** | $138.0\%$ | $305.4\text{ psi}$ | True |
| | **RL (Actor-Critic)** | **$1181.6\text{ bbl}$** | $291.7\text{ bbl}\cdot\text{h}$ | **$69.7\%$** | $305.3\text{ psi}$ | True |

---

## Computational Latency & Speed Benchmark

| Controller | Mean Decision Latency ($\text{ms/step}$) | Max Latency ($\text{ms}$) | Field PLC Compliance |
| :--- | :---: | :---: | :--- |
| **Rule-Based** | $0.0007 \pm 0.0003\text{ ms}$ | $0.0057\text{ ms}$ | Instantaneous |
| **PID** | $0.0010 \pm 0.0002\text{ ms}$ | $0.0068\text{ ms}$ | Instantaneous |
| **RL Agent** | $0.0198 \pm 0.0039\text{ ms}$ | $0.0868\text{ ms}$ | Sub-millisecond Neural Net |
| **NMPC** | $0.1551 \pm 1.1805\text{ ms}$ | $10.8322\text{ ms}$ | Real-time ($< 15\text{ ms}$) |

---

## Repository Structure

```text
.
├── configs/
│   └── default.yaml               # Calibrated simulator parameters
├── controllers/
│   ├── __init__.py                # Controllers package init
│   ├── base_controller.py         # Abstract Base Class
│   ├── rule_based.py              # Rule-Based Heuristic Controller
│   ├── pid_controller.py          # Industrial PID with Anti-Windup
│   ├── mpc_controller.py          # Non-Linear Model Predictive Controller
│   ├── rl_controller.py           # Trained Policy RL Controller Wrapper
│   └── gym_env.py                 # Gymnasium Compatible Environment
├── docs/
│   └── API_DOCUMENTATION.md       # API Documentation & Class Methods
├── models/
│   └── rl_choke_policy.npz        # Trained Actor-Critic policy weights
├── plots/                         # Generated figures and charts
├── simulator/                     # Physics engine sub-package
│   ├── simulation.py              # Simulator core & coupled solver
│   ├── reservoir.py               # Reservoir IPR & material balance
│   ├── tubing.py                  # Hydrostatics & tubing friction
│   ├── choke.py                   # Orifice equation & actuator lag
│   ├── surface.py                 # Surface backpressure dynamics
│   ├── state.py                   # WellState dataclass
│   └── config.py                  # Dataclass configurations
├── dashboard/                     # Web & Matplotlib dashboards
│   ├── static/                    # Frontend HTML, CSS, JS assets
│   ├── server.py                  # Python web dashboard server
│   ├── dashboard_mpl.py           # Matplotlib backup dashboard
│   └── README.md                  # Dashboard documentation
├── DASHBOARD.md                   # Web dashboard documentation
├── TECHNICAL_REPORT.md            # Mathematical physics specification
├── run_dashboard.py               # Launcher script for web & matplotlib dashboards
├── validate_and_plot.py           # Calibration against reference dataset
├── run_trajectory_analysis.py     # Trajectory overlay & constraint audit
├── train_rl.py                    # RL Actor-Critic policy training script
├── benchmark.py                   # 5-Controller benchmark suite
├── run_monte_carlo_experiments.py # Statistical significance evaluation
├── run_ablation_and_sensitivity.py# Physics ablation & latency benchmark
├── run_all_benchmarks.py          # Master single-command reproduction script
└── requirements.txt               # Exact Python dependencies
```

---

## Real-Time Interactive Web Dashboard

An interactive single-screen web dashboard is available for real-time visualization and multi-scenario analysis.

### Running the Web Dashboard

```bash
python3 run_dashboard.py
```

This launches the web server on `http://localhost:8050` with:
- Left-side control panel for controller selection, playback controls, target oil rate, and minimum WHP safety limit.
- 2D animated wellbore schematic depicting downhole perforations, fluid movement, and real-time physical choke opening.
- 4 real-time line charts fitting entirely within the screen viewport without scrolling.
- Page 2 Monte Carlo & Scenario testing studio for evaluating parameter uncertainty, ablation dynamics, and controller decision latencies.

### Matplotlib Backup Dashboard

To run the Matplotlib dashboard version:

```bash
python3 run_dashboard.py --mpl
# or
python3 dashboard/dashboard_mpl.py
```

See [DASHBOARD.md](DASHBOARD.md) for further configuration details.

---

## Documentation

- **Technical Report**: See [TECHNICAL_REPORT.md](TECHNICAL_REPORT.md)
- **API Reference**: See [docs/API_DOCUMENTATION.md](docs/API_DOCUMENTATION.md)
- **Dashboard Reference**: See [DASHBOARD.md](DASHBOARD.md)

---

## License

Distributed under the MIT License. See `LICENSE` for more information.