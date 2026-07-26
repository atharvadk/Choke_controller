#!/usr/bin/env python3
"""
Simple test to verify the simulator can be instantiated and stepped.
"""

from simulator.simulation import Simulator

def main():
    sim = Simulator("configs/default.yaml")
    # Access internal state for printing
    print("Initial state:")
    print(f"  Pressure: {sim.state.Pr:.2f} bar")
    print(f"  Choke opening: {sim.state.opening_actual:.2f}%")
    # Run a few steps with constant choke command
    for i in range(5):
        obs = sim.step(choke_command=40.0)
        print(f"Step {i+1}: t={obs['time']:.1f}s, "
              f"Pwh={obs['Pwh']:.2f} bar, "
              f"Q={obs['total_flow']:.6f} m3/s, "
              f"oil={obs['oil_rate']:.6f} m3/s")
    # Test logging
    sim.start_logging("test_log.csv")
    for i in range(5):
        sim.step(choke_command=50.0)
    sim.stop_logging()
    print("Log written to test_log.csv")

if __name__ == "__main__":
    main()