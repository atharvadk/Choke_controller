# Real-time Dashboard for Autonomous Choke Control System

This dashboard provides real-time visualization of the choke control simulation with interactive controls for adjusting parameters without needing to restart the application.

## Features

- **Interactive Controls**: Adjust all parameters via GUI elements:
  - Controller type (Fixed, Rule-Based, PID, MPC, RL) using radio buttons
  - Simulation duration, time step, target oil rate, min WHP, and update interval using text boxes
  - Start/Stop buttons to control simulation execution
- **Real-time Visualization**: Live plots of key variables:
  - Choke opening (%)
  - Oil rate (bbl/hr)
  - Wellhead pressure (psi)
  - Bottom hole pressure (psi)
  - Reservoir pressure (bar)
  - Choke command (%)
- **Live Statistics Display**: Current values of all monitored variables
- **Dynamic Axis Scaling**: Plots automatically adjust to show relevant data ranges
- **Support for All Controllers**: Works with all five control strategies implemented in the system

## Usage

Run the dashboard from the project root directory:

```bash
python3 dashboard/dashboard.py
```

### Controls Explained

1. **Controller Selection**: Use the radio buttons to choose between:
   - Fixed: Constant 30% choke opening
   - Rule-Based: Heuristic-based controller
   - PID: Proportional-Integral-Derivative controller with anti-windup
   - MPC: Model Predictive Controller
   - RL: Reinforcement Learning (Actor-Critic) agent

2. **Parameters**:
   - **Duration (s)**: Total simulation time
   - **dt (s)**: Time step for simulation (smaller = more accurate but slower)
   - **Target Oil (bbl/hr)**: Desired oil production rate
   - **Min WHP (psi)**: Minimum allowed wellhead pressure (safety constraint)
   - **Update Interval**: Update plots every N simulation steps (higher = less frequent updates but better performance)

3. **Buttons**:
   - **Start Simulation**: Begin or restart the simulation with current parameters
   - **Stop**: Halt the currently running simulation

## What You'll See

When you start the simulation, six plots will update in real-time:

1. **Choke Opening**: Shows the actual valve position (%)
2. **Oil Rate**: Tracks oil production against your target (dashed line)
3. **Wellhead Pressure**: Monitored against minimum safety limit (dotted line)
4. **Bottom Hole Pressure**: Downhole pressure measurement
5. **Reservoir Pressure**: Pressure in the reservoir
6. **Choke Command**: The controller's output signal to the valve

Below the plots, a text box shows current numerical values for all variables.

## Example Workflow

1. Select "PID" as the controller type
2. Set Target Oil to 120 bbl/hr
3. Set Min WHP to 210 psi (safety constraint)
4. Leave Duration at 3600s (1 hour) and dt at 1.0 second
5. Click "Start Simulation"
6. Watch as the PID controller adjusts the choke to maintain your oil target while respecting the WHP constraint
7. Experiment with different controllers to compare their performance

## Technical Details

- Built with matplotlib for plotting and widgets
- Uses the same simulation engine as the rest of the project
- Updates plots in real-time using matplotlib's animation framework
- Maintains fixed-size data buffers for efficient memory usage
- Blitting disabled for compatibility across environments

## Requirements

- Python 3.10+
- All packages from `requirements.txt` in the project root
- Trained RL model at `models/rl_choke_policy.npz` (for RL controller)

## Suggested Experiments

1. **Controller Comparison**: Run the same scenario with each controller to see differences in response time, overshoot, and steady-state error
2. **Disturbance Response**: While running, imagine disturbances and observe how each controller reacts
3. **Tuning Practice**: Adjust PID gains (would require modifying the code) to see effects on performance
4. **Constraint Handling**: Set a tight Min WHP and see how controllers handle the trade-off between production and safety
5. **Aggressive Targets**: Set very high oil targets to see how controllers behave when goals may be unattainable

## Notes

- The first time you run the RL controller, there may be a short delay while the neural network loads
- For very long simulations or very small time steps, consider increasing the Update Interval for better performance
- All simulations use the same physical model and initial conditions for fair comparison
- Close the window to stop the dashboard completely

---

**Tip**: Start with the PID controller to get familiar with the dynamics, then try the MPC and RL controllers to see how more advanced strategies perform!