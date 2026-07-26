#!/usr/bin/env python3
"""
Real-time dashboard for choke control simulation with interactive controls.
Displays live plots of key variables while simulation runs.
Parameters can be adjusted via GUI controls.
"""

import sys
import os

# Fix the matplotlib callback issue by patching CallbackRegistry.process
import matplotlib.cbook as cbook

# Store the original process method
_original_process = cbook.CallbackRegistry.process

def patched_process(self, signal, *args, **kwargs):
    """Patched version of CallbackRegistry.process that handles missing inattribute"""
    # All of the functions registered to receive callbacks on *s* will be
    # called with ``*args`` and ``**kwargs``.
    if self._signals is not None:
        # Assuming _check_in_list is available or we can skip this check for now
        pass
    if hasattr(self, 'callbacks') and self.callbacks is not None:
        for ref in list(self.callbacks.get(signal, {}).values()):
            func = ref()
            if func is not None:
                try:
                    func(*args, **kwargs)
                # this does not capture KeyboardInterrupt, SystemExit,
                # and GeneratorExit
                except AttributeError as e:
                    # Handle missing inattribute specifically for resize events
                    if "'object' object has no attribute 'inaxes'" in str(args[0] if args else '') or \
                       (len(args) > 0 and hasattr(args[0], 'name') and 'resize' in getattr(args[0], 'name', '')):
                        # Ignore resize events that don't have inattribute - it's safe to ignore
                        pass
                    else:
                        # Re-raise if it's a different AttributeError
                        raise
                except Exception as exc:
                    # this does not capture KeyboardInterrupt, SystemExit,
                    # and GeneratorExit
                    if hasattr(self, 'exception_handler') and self.exception_handler is not None:
                        self.exception_handler(exc)
                    else:
                        raise

# Apply the patch
cbook.CallbackRegistry.process = patched_process

# Now continue with normal imports
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import RadioButtons, TextBox, Button

# Add parent directory to path to import simulator and controllers
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from simulator.simulation import Simulator
from controllers.pid_controller import PIDChokeController
from controllers.rl_controller import RLChokeController
from controllers.mpc_controller import ModelPredictiveChokeController
from controllers.rule_based import RuleBasedChokeController

# Unit conversion constants
PSI_TO_BAR = 0.0689475729
BAR_TO_PSI = 14.5037738
M3S_TO_BBL_HR = 22643.4

CONFIG_PATH = "configs/default.yaml"


class ChokeDashboard:
    def __init__(self):
        # Default parameters
        self.controller_type = "pid"
        self.duration = 3600.0
        self.dt = 1.0
        self.target_oil = 100.0
        self.min_whp = 210.0
        self.update_interval = 1

        # State
        self.sim = None
        self.controller = None
        self.anim = None
        self.running = False

        # Data storage
        self.max_points = 0
        self.times = deque()
        self.choke_openings = deque()
        self.oil_rates = deque()
        self.whp_psi = deque()
        self.bhp_psi = deque()
        self.reservoir_pressure = deque()
        self.choke_commands = deque()

        # Set up the figure and axes
        self.setup_ui()

    def setup_ui(self):
        """Set up the user interface with plots and controls."""
        plt.style.use('seaborn-v0_8')
        self.fig = plt.figure(figsize=(16, 10))
        self.fig.suptitle("Real-time Choke Control Dashboard", fontsize=16)

        # Create grid for plots and controls
        from matplotlib.gridspec import GridSpec
        gs = GridSpec(4, 3, figure=self.fig, hspace=0.3, wspace=0.3)

        # Plot axes (rows 0-2, all columns)
        self.ax_choke = self.fig.add_subplot(gs[0, :])
        self.ax_oil = self.fig.add_subplot(gs[1, :])
        self.ax_whp = self.fig.add_subplot(gs[2, 0])
        self.ax_bhp = self.fig.add_subplot(gs[2, 1])
        self.ax_res = self.fig.add_subplot(gs[2, 2])
        self.ax_cmd = self.fig.add_subplot(gs[3, :])

        # Initialize empty lines
        self.line_choke, = self.ax_choke.plot([], [], 'b-', linewidth=2)
        self.line_oil, = self.ax_oil.plot([], [], 'g-', linewidth=2)
        self.line_whp, = self.ax_whp.plot([], [], 'r-', linewidth=2)
        self.line_bhp, = self.ax_bhp.plot([], [], 'm-', linewidth=2)
        self.line_res, = self.ax_res.plot([], [], 'c-', linewidth=2)
        self.line_cmd, = self.ax_cmd.plot([], [], 'k--', linewidth=2)

        # Set axis labels
        self.ax_choke.set_ylabel("Choke Opening (%)")
        self.ax_choke.grid(True, alpha=0.3)
        self.ax_oil.set_ylabel("Oil Rate (bbl/hr)")
        self.ax_oil.grid(True, alpha=0.3)
        self.ax_whp.set_ylabel("Wellhead Pressure (psi)")
        self.ax_whp.grid(True, alpha=0.3)
        self.ax_bhp.set_ylabel("Bottom Hole Pressure (psi)")
        self.ax_bhp.grid(True, alpha=0.3)
        self.ax_res.set_ylabel("Reservoir Pressure (bar)")
        self.ax_res.set_xlabel("Time (s)")
        self.ax_res.grid(True, alpha=0.3)
        self.ax_cmd.set_ylabel("Choke Command (%)")
        self.ax_cmd.set_xlabel("Time (s)")
        self.ax_cmd.grid(True, alpha=0.3)

        # Add target oil rate line to oil plot
        self.target_line = self.ax_oil.axhline(y=self.target_oil, color='k', linestyle='--', alpha=0.7, label='Target')
        self.ax_oil.legend(loc='upper right')

        # Add min WHP line to whp plot
        self.whp_limit_line = self.ax_whp.axhline(y=self.min_whp, color='r', linestyle=':', alpha=0.7, label='Min WHP')
        self.ax_whp.legend(loc='upper right')

        # Text annotation for current stats
        self.stats_text = self.ax_choke.text(0.02, 0.98, '', transform=self.ax_choke.transAxes,
                               verticalalignment='top', fontsize=10,
                               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        # Create control area at the bottom - SIMPLIFIED VERSION
        self.create_simple_controls()

        # Adjust layout to make room for controls
        plt.subplots_adjust(bottom=0.25, top=0.93, hspace=0.4)

    def create_simple_controls(self):
        """Create simple control widgets to avoid complex positioning issues."""
        # Much simpler approach: put controls in a fixed location at bottom
        # Use a fixed position that's less likely to cause resize issues

        # Controller selection
        controller_ax = self.fig.add_axes([0.05, 0.02, 0.15, 0.15])
        self.controller_radio = RadioButtons(
            controller_ax,
            ('fixed', 'rule_based', 'pid', 'mpc', 'rl'),
            active=2
        )
        self.controller_radio.on_clicked(self.controller_changed)

        # Add label manually
        controller_ax.text(-0.3, 0.5, 'Controller:', transform=controller_ax.transAxes,
                          verticalalignment='center', horizontalalignment='right',
                          fontsize=9)

        # Simple text boxes in a row
        box_width = 0.08
        box_height = 0.03
        box_y = 0.05
        x_start = 0.25
        x_spacing = 0.09

        # Duration
        duration_ax = self.fig.add_axes([x_start, box_y, box_width, box_height])
        self.duration_text = TextBox(duration_ax, 'Duration:', initial=str(self.duration))
        self.duration_text.on_submit(self.duration_submitted)

        # dt
        dt_ax = self.fig.add_axes([x_start + x_spacing, box_y, box_width, box_height])
        self.dt_text = TextBox(dt_ax, 'dt:', initial=str(self.dt))
        self.dt_text.on_submit(self.dt_submitted)

        # Target oil
        target_oil_ax = self.fig.add_axes([x_start + 2*x_spacing, box_y, box_width, box_height])
        self.target_oil_text = TextBox(target_oil_ax, 'Target Oil:', initial=str(self.target_oil))
        self.target_oil_text.on_submit(self.target_oil_submitted)

        # Min WHP
        min_whp_ax = self.fig.add_axes([x_start + 3*x_spacing, box_y, box_width, box_height])
        self.min_whp_text = TextBox(min_whp_ax, 'MWHP:', initial=str(self.min_whp))
        self.min_whp_text.on_submit(self.min_whp_submitted)

        # Update interval
        update_interval_ax = self.fig.add_axes([x_start + 4*x_spacing, box_y, box_width, box_height])
        self.update_interval_text = TextBox(update_interval_ax, 'Update:', initial=str(self.update_interval))
        self.update_interval_text.on_submit(self.update_interval_submitted)

        # Buttons
        button_width = 0.08
        button_height = 0.04
        button_y = 0.01

        start_ax = self.fig.add_axes([x_start, button_y, button_width, button_height])
        self.start_button = Button(start_ax, 'Start')
        self.start_button.on_clicked(self.start_simulation)

        stop_ax = self.fig.add_axes([x_start + x_spacing, button_y, button_width, button_height])
        self.stop_button = Button(stop_ax, 'Stop')
        self.stop_button.on_clicked(self.stop_simulation)

    def controller_changed(self, label):
        """Handle controller selection change."""
        self.controller_type = label

    def duration_submitted(self, text):
        """Handle duration input submission."""
        try:
            self.duration = float(text)
        except ValueError:
            pass  # Keep current value if invalid

    def dt_submitted(self, text):
        """Handle dt input submission."""
        try:
            self.dt = float(text)
        except ValueError:
            pass  # Keep current value if invalid

    def target_oil_submitted(self, text):
        """Handle target oil input submission."""
        try:
            self.target_oil = float(text)
            # Update the target line in the plot
            self.target_line.set_ydata([self.target_oil, self.target_oil])
        except ValueError:
            pass  # Keep current value if invalid

    def min_whp_submitted(self, text):
        """Handle min WHP input submission."""
        try:
            self.min_whp = float(text)
            # Update the WHP limit line in the plot
            self.whp_limit_line.set_ydata([self.min_whp, self.min_whp])
        except ValueError:
            pass  # Keep current value if invalid

    def update_interval_submitted(self, text):
        """Handle update interval input submission."""
        try:
            self.update_interval = max(1, int(float(text)))  # Ensure at least 1
        except ValueError:
            pass  # Keep current value if invalid

    def start_simulation(self, event):
        """Start or restart the simulation."""
        # Stop any existing animation
        if self.anim is not None:
            self.anim.event_source.stop()

        # Reset data storage
        self.times.clear()
        self.choke_openings.clear()
        self.oil_rates.clear()
        self.whp_psi.clear()
        self.bhp_psi.clear()
        self.reservoir_pressure.clear()
        self.choke_commands.clear()

        # Calculate max points
        self.max_points = int(self.duration / self.dt)

        # Initialize simulator
        self.sim = Simulator(CONFIG_PATH)
        self.sim.dt = self.dt
        self.sim.reset()

        # Initialize controller
        if self.controller_type == "fixed":
            self.controller = None
        elif self.controller_type == "rule_based":
            self.controller = RuleBasedChokeController(target_oil_bbl_hr=self.target_oil, min_whp_psi=self.min_whp)
        elif self.controller_type == "pid":
            self.controller = PIDChokeController(kp=1.2, ki=0.08, kd=0.3, target_oil_bbl_hr=self.target_oil, min_whp_psi=self.min_whp, dt=self.dt)
        elif self.controller_type == "mpc":
            self.controller = ModelPredictiveChokeController(config_path=CONFIG_PATH, horizon=6, dt_control=60.0, target_oil_bbl_hr=self.target_oil, min_whp_psi=self.min_whp)
        elif self.controller_type == "rl":
            self.controller = RLChokeController(model_path="models/rl_choke_policy.npz", target_oil_bbl_hr=self.target_oil, min_whp_psi=self.min_whp)

        if self.controller is not None:
            self.controller.reset()

        # Start animation
        self.anim = animation.FuncAnimation(
            self.fig, self.update, interval=100, blit=False, repeat=False, save_count=0
        )
        self.fig.canvas.draw_idle()

    def stop_simulation(self, event):
        """Stop the simulation."""
        if self.anim is not None:
            self.anim.event_source.stop()

    def simulate_step(self, controller_type, controller, choke_command=0.0):
        """Perform one simulation step and return observation and choke command."""
        if controller_type == "fixed":
            choke_cmd = 30.0
            obs = self.sim.step(choke_cmd)
        else:
            obs = self.sim._get_observation()
            choke_cmd = controller.compute_action(obs)
            obs, _ = self.sim.step_with_info(choke_cmd)
        return obs, choke_cmd

    def update(self, frame):
        """Animation update function."""
        # Perform multiple simulation steps if update_interval > 1
        for _ in range(self.update_interval):
            if self.sim.state.time >= self.duration:
                self.anim.event_source.stop()
                print("Simulation finished.")
                break

            obs, choke_cmd = self.simulate_step(self.controller_type, self.controller)
            # Store data
            self.times.append(self.sim.state.time)
            self.choke_openings.append(self.sim.state.opening_actual)
            self.oil_rates.append(self.sim.state.oil_rate * M3S_TO_BBL_HR)
            self.whp_psi.append(self.sim.state.Pwh * BAR_TO_PSI)
            self.bhp_psi.append(self.sim.state.Pwf * BAR_TO_PSI)
            self.reservoir_pressure.append(self.sim.state.Pr)
            self.choke_commands.append(choke_cmd)

        # Update data for lines
        if len(self.times) > 0:
            self.line_choke.set_data(self.times, self.choke_openings)
            self.line_oil.set_data(self.times, self.oil_rates)
            self.line_whp.set_data(self.times, self.whp_psi)
            self.line_bhp.set_data(self.times, self.bhp_psi)
            self.line_res.set_data(self.times, self.reservoir_pressure)
            self.line_cmd.set_data(self.times, self.choke_commands)

            # Update axis limits dynamically
            axes_data = [
                (self.ax_choke, self.choke_openings),
                (self.ax_oil, self.oil_rates),
                (self.ax_whp, self.whp_psi),
                (self.ax_bhp, self.bhp_psi),
                (self.ax_res, self.reservoir_pressure),
                (self.ax_cmd, self.choke_commands)
            ]

            for ax, data_y in axes_data:
                if len(data_y) > 0:
                    y_min = min(data_y)
                    y_max = max(data_y)
                    y_range = y_max - y_min
                    if y_range == 0:
                        y_range = 1
                    ax.set_ylim(y_min - 0.1 * y_range, y_max + 0.1 * y_range)

            # X-axis: time
            if len(self.times) > 0:
                t_min = min(self.times)
                t_max = max(self.times)
                t_range = t_max - t_min
                if t_range == 0:
                    t_range = 1
                for ax in [self.ax_choke, self.ax_oil, self.ax_whp, self.ax_bhp, self.ax_res, self.ax_cmd]:
                    ax.set_xlim(t_min - 0.1 * t_range, t_max + 0.1 * t_range)

            # Update stats text
            latest_oil = self.oil_rates[-1] if self.oil_rates else 0
            latest_whp = self.whp_psi[-1] if self.whp_psi else 0
            latest_bhp = self.bhp_psi[-1] if self.bhp_psi else 0
            latest_choke = self.choke_openings[-1] if self.choke_openings else 0
            latest_res = self.reservoir_pressure[-1] if self.reservoir_pressure else 0
            stats_text = (
                f"Time: {self.sim.state.time:.1f} s\n"
                f"Oil Rate: {latest_oil:.1f} bbl/hr\n"
                f"WHP: {latest_whp:.1f} psi\n"
                f"BHP: {latest_bhp:.1f} psi\n"
                f"Reservoir: {latest_res:.1f} bar\n"
                f"Choke Opening: {latest_choke:.1f}%\n"
                f"Choke Command: {choke_cmd:.1f}%"
            )
            self.stats_text.set_text(stats_text)

        return (self.line_choke, self.line_oil, self.line_whp, self.line_bhp, self.line_res, self.line_cmd, self.stats_text)

    def run(self):
        """Show the dashboard and start the GUI event loop."""
        plt.show()


def main():
    dashboard = ChokeDashboard()
    dashboard.run()


if __name__ == "__main__":
    main()