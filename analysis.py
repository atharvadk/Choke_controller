#!/usr/bin/env python3
"""
Analysis script for the choke control simulator.
Runs standard scenarios and generates engineering plots.
"""

import os
import matplotlib.pyplot as plt
import numpy as np
from simulator.simulation import Simulator

# Ensure plots directory exists
os.makedirs("plots", exist_ok=True)

def run_scenario(choke_func, duration_hours=3.0, dt=1.0):
    """
    Run a scenario with a given choke function of time.

    Args:
        choke_func: function that takes time in seconds and returns choke opening (%)
        duration_hours: total duration in hours
        dt: time step in seconds (should match simulator dt)

    Returns:
        dict of time series data
    """
    sim = Simulator("configs/default.yaml")
    total_seconds = int(duration_hours * 3600)
    steps = int(total_seconds / dt)

    # Time array
    times = np.arange(0, total_seconds, dt)  # seconds

    # Containers for data
    data = {
        'time': times,
        'Pr': [],      # Reservoir pressure (bar)
        'Pwf': [],     # Bottom hole pressure (bar)
        'Pth': [],     # Tubing head pressure (bar)
        'Pwh': [],     # Wellhead pressure (bar)
        'oil_rate': [], # Oil rate (m3/s)
        'water_rate': [], # Water rate (m3/s)
        'opening_target': [], # Target choke opening (%)
        'opening_actual': [], # Actual choke opening (%)
        'total_flow': [], # Total flow rate (m3/s)
    }

    for i, t in enumerate(times):
        choke_command = choke_func(t)
        obs = sim.step(choke_command)
        # Store data
        data['Pr'].append(obs['Pr'])
        data['Pwf'].append(obs['Pwf'])
        data['Pth'].append(obs['Pth'])
        data['Pwh'].append(obs['Pwh'])
        data['oil_rate'].append(obs['oil_rate'])
        data['water_rate'].append(obs['water_rate'])
        data['opening_target'].append(obs['opening_target'])
        data['opening_actual'].append(obs['opening_actual'])
        data['total_flow'].append(obs['total_flow'])

        # Progress indicator
        if i % 3600 == 0:
            print(f"  {t/3600:.1f} h completed")

    return data

def plot_scenario(data, scenario_name):
    """
    Generate plots for a scenario and save to plots directory.
    """
    # Convert time to hours for plotting
    time_hours = np.array(data['time']) / 3600.0

    # Create figure with subplots
    fig, axes = plt.subplots(3, 2, figsize=(12, 10))
    fig.suptitle(f"Scenario: {scenario_name}", fontsize=16)

    # Reservoir Pressure
    axes[0, 0].plot(time_hours, data['Pr'], 'b-')
    axes[0, 0].set_ylabel('Pressure (bar)')
    axes[0, 0].set_title('Reservoir Pressure')
    axes[0, 0].grid(True)

    # Tubing Head Pressure
    axes[0, 1].plot(time_hours, data['Pth'], 'r-')
    axes[0, 1].set_ylabel('Pressure (bar)')
    axes[0, 1].set_title('Tubing Head Pressure')
    axes[0, 1].grid(True)

    # Wellhead Pressure
    axes[1, 0].plot(time_hours, data['Pwh'], 'g-')
    axes[1, 0].set_ylabel('Pressure (bar)')
    axes[1, 0].set_title('Wellhead Pressure')
    axes[1, 0].grid(True)

    # Oil Rate
    axes[1, 1].plot(time_hours, data['oil_rate'], 'm-')
    axes[1, 1].set_ylabel('Flow Rate (m3/s)')
    axes[1, 1].set_title('Oil Rate')
    axes[1, 1].grid(True)

    # Water Rate
    axes[2, 0].plot(time_hours, data['water_rate'], 'c-')
    axes[2, 0].set_ylabel('Flow Rate (m3/s)')
    axes[2, 0].set_title('Water Rate')
    axes[2, 0].grid(True)

    # Choke Opening
    axes[2, 1].plot(time_hours, data['opening_target'], 'k--', label='Target')
    axes[2, 1].plot(time_hours, data['opening_actual'], 'k-', label='Actual')
    axes[2, 1].set_ylabel('Opening (%)')
    axes[2, 1].set_xlabel('Time (hours)')
    axes[2, 1].set_title('Choke Opening')
    axes[2, 1].legend()
    axes[2, 1].grid(True)

    plt.tight_layout()
    plt.savefig(f"plots/{scenario_name.replace(' ', '_').lower()}.png", dpi=150)
    plt.close()

def scenario_constant_choke(t):
    """Constant 30% choke"""
    return 30.0

def scenario_step_up(t):
    """Step from 30% to 60% at 1 hour"""
    if t < 3600:  # 1 hour
        return 30.0
    else:
        return 60.0

def scenario_step_down(t):
    """Step from 60% to 20% at 2 hours (after initial 30% for 1 hour?)"""
    # We'll do: 0-1h: 30%, 1-2h: 60%, 2-3h: 20%
    if t < 3600:
        return 30.0
    elif t < 7200:
        return 60.0
    else:
        return 20.0

def scenario_ramp(t):
    """Ramp from 20% to 80% over 5 minutes starting at 1 hour"""
    # 0-1h: 20%
    # 1h-1h5m: ramp from 20 to 80
    # after 1h5m: 80%
    if t < 3600:
        return 20.0
    elif t < 3600 + 300:  # 5 minutes = 300 seconds
        # Linear ramp
        frac = (t - 3600) / 300.0
        return 20.0 + frac * (80.0 - 20.0)
    else:
        return 80.0

def main():
    print("Running standard scenarios for simulator verification...")

    scenarios = [
        ("Constant Choke 30%", scenario_constant_choke, 3.0),
        ("Step Up 30% to 60%", scenario_step_up, 3.0),
        ("Step Down 60% to 20%", scenario_step_down, 3.0),
        ("Ramp 20% to 80%", scenario_ramp, 2.0),  # 2 hours enough for ramp
    ]

    for name, choke_func, duration in scenarios:
        print(f"\nRunning: {name}")
        data = run_scenario(choke_func, duration_hours=duration)
        print(f"  Generating plots...")
        plot_scenario(data, name)
        print(f"  Done.")

    print("\nAll scenarios completed. Plots saved in 'plots/' directory.")

if __name__ == "__main__":
    main()