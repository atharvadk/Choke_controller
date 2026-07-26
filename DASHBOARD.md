# Dashboard Documentation

This document describes the design, features, and usage of the Autonomous Choke Control System dashboard interface.

---

## Architecture Overview

The system includes two dashboard implementations:

1. **Interactive Web Dashboard (Recommended)**: A 2-page, single-screen web interface built using HTML, CSS, and native JavaScript canvas rendering, supported by a Python HTTP backend (`dashboard/server.py`).
2. **Matplotlib Backup Dashboard**: A Matplotlib GUI application (`dashboard/dashboard_mpl.py`) available as a fallback interface.

---

## 1. Web Dashboard Features

### Page 1: Live Well Hydrodynamics & Simulation

- **Left Control Panel**: Positioned on the left side of the screen. Includes:
  - Navigation tab switcher between Live Well and Monte Carlo pages.
  - Controller strategy selection (Fixed Choke, Rule-Based, PID Control, MPC, RL Agent).
  - Simulation execution controls (Start/Pause, Reset, Speed selector: 1x, 2x, 5x, 10x).
  - Target Oil Rate slider (20 to 250 bbl/hr) and Minimum WHP Safety Limit slider (100 to 350 psi).
- **Center Column**: Contains 4 compact telemetry cards (Choke %, Oil Rate, WHP, Reservoir Pressure) and an interactive 2D Hydrodynamic Wellbore Schematic.
  - The schematic displays downhole perforations, upward fluid particles with flow velocity scaled to produced oil rate $Q$, and surface choke valve plungers that physically open and close in real time.
- **Right Column**: Displays 4 real-time line charts in a 2x2 grid fitted to the screen height:
  1. Choke Opening & Actuator Command (%)
  2. Oil Production Rate vs Target (bbl/hr)
  3. Wellhead Pressure Safety Limit (psi)
  4. Bottom Hole & Reservoir Pressure (bar)

### Page 2: Monte Carlo & Scenario Evaluation Suite

- **Scenario Selector**: Dropdown to select and run predefined benchmark suites:
  - Monte Carlo 20-Trial Parameter Uncertainty Sweep (PI, $P_r$, Water Cut, Separator P).
  - Dynamic Ablation Study (Wellbore Storage Lag $\tau_{\text{well}}$).
  - PI & Reservoir Pressure Sensitivity Analysis.
  - Controller Decision Latency Benchmark ($\text{ms/step}$).
- **Performance Charts & Statistics**: Displays multi-trial trajectory variance lines, distribution charts, and a summary data table reporting Mean $\pm$ Standard Deviation and Student's t-test p-values.

---

## 2. Running the Dashboard

### Web Dashboard

To launch the web dashboard:

```bash
python3 run_dashboard.py
```

Access the interface at `http://localhost:8050`.

### Matplotlib Backup Dashboard

To launch the Matplotlib dashboard:

```bash
python3 run_dashboard.py --mpl
# or
python3 dashboard/dashboard_mpl.py
```

---

## 3. Implementation Files

- `run_dashboard.py`: Launcher script for web and matplotlib dashboards.
- `dashboard/server.py`: Python web server serving static files and JSON API endpoints (`/api/step`, `/api/simulate`, `/api/benchmark`).
- `dashboard/static/index.html`: Web dashboard HTML template.
- `dashboard/static/style.css`: Viewport-fitted flex/grid stylesheet.
- `dashboard/static/app.js`: Client simulation engine, 2D schematic animator, and native canvas chart renderer.
- `dashboard/dashboard_mpl.py`: Backup Matplotlib interface script.
