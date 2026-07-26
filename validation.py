#!/usr/bin/env python3
"""
Simple validation: run simulator with varying choke and print results.
"""

from simulator.simulation import Simulator

def main():
    sim = Simulator("configs/default.yaml")
    # simulate 10 hours with dt=1s (as in config)
    hours = 10
    steps = int(hours * 3600)  # seconds
    # We'll change choke every hour
    for i in range(steps):
        t = i  # seconds
        hour = t / 3600.0
        if hour < 3.0:
            choke = 30.0
        elif hour < 6.0:
            choke = 50.0
        else:
            choke = 20.0
        obs = sim.step(choke)
        if i % 3600 == 0:  # print hourly
            print(f"Time {hour:.1f} h: choke={obs.get('opening_actual',0):.1f}%, "
                  f"Pwh={obs.get('Pwh',0):.2f} bar, "
                  f"OilRate={obs.get('oil_rate',0):.4f} m3/s, "
                  f"WaterRate={obs.get('water_rate',0):.4f} m3/s")
    # final stats
    print("\nFinal state:")
    print(f"  Reservoir Pressure: {sim.state.Pr:.2f} bar")
    print(f"  Bottom-hole Pressure: {sim.state.Pwf:.2f} bar")
    print(f"  Tubing Head Pressure: {sim.state.Pth:.2f} bar")
    print(f"  Wellhead Pressure: {sim.state.Pwh:.2f} bar")
    print(f"  Oil rate: {sim.state.oil_rate:.6f} m3/s")
    print(f"  Water rate: {sim.state.water_rate:.6f} m3/s")

if __name__ == "__main__":
    main()