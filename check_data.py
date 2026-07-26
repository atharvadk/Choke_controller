#!/usr/bin/env python3
"""Quick check of simulator output for constant choke."""

from simulator.simulation import Simulator

sim = Simulator("configs/default.yaml")
# Run for 1 hour at 30% choke
for i in range(3600):
    obs = sim.step(30.0)
    if i % 600 == 0:  # every 10 minutes
        print(f"T={i/3600:.2f}h: Pr={obs['Pr']:.2f} bar, Pwf={obs['Pwf']:.2f}, Pth={obs['Pth']:.2f}, Pwh={obs['Pwh']:.2f}, oil={obs['oil_rate']:.6f} m3/s, water={obs['water_rate']:.6f} m3/s")

print("\nFinal state:")
print(f"  Pr: {sim.state.Pr:.2f} bar")
print(f"  Pwf: {sim.state.Pwf:.2f} bar")
print(f"  Pth: {sim.state.Pth:.2f} bar")
print(f"  Pwh: {sim.state.Pwh:.2f} bar")
print(f"  oil_rate: {sim.state.oil_rate:.6f} m3/s")
print(f"  water_rate: {sim.state.water_rate:.6f} m3/s")