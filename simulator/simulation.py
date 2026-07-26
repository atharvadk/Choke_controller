"""
Simulation orchestrator: ties together all sub‑models and exposes a simple
step(choke_command) -> observation interface for controllers.
"""

import math
from typing import Dict, Any
from .state import WellState
from .config import Config, load_config
from . import reservoir, perforation, tubing, choke, surface, disturbances, sensors, logger


class Simulator:
    def __init__(self, config_path: str = "configs/default.yaml"):
        self.config: Config = load_config(config_path)
        # Convert productivity index from m3/(day*bar) to m3/(s*bar)
        pi_day_bar = getattr(self.config.reservoir, 'productivity_index', 2.0)
        self.config.reservoir.productivity_index_si = pi_day_bar / 86400.0
        self.state: WellState = self._initialize_state()
        self.time: float = 0.0
        self.dt: float = self.config.simulation.dt  # seconds per step
        self._sensor_state: dict = {}  # internal state for sensor models (delay buffers, etc.)
        self._logger = None  # type: logger.CSVLogger | None

    # -----------------------------------------------------------------
    # Initialization
    # -----------------------------------------------------------------
    def _initialize_state(self) -> WellState:
        """Populate a WellState instance with values from the configuration."""
        cfg = self.config
        # Reservoir
        Pr = getattr(cfg.reservoir, 'pressure', 320.0)          # bar
        Tr = getattr(cfg.reservoir, 'temperature', 90.0)       # °C
        # Bottom hole and tubing start at reservoir pressure (no flow yet)
        Pwf = Pr
        Pth = Pr
        # Choke
        opening_target = getattr(cfg.choke, 'opening_target', 30.0)  # %
        opening_actual = opening_target
        # Effective area will be computed later
        effective_area = 0.0
        pressure_drop = 0.0
        # Surface
        Pwh = getattr(cfg.surface, 'separator_pressure', 20.0)   # bar (approx)
        Twh = getattr(cfg.surface, 'ambient_temperature', 30.0) # °C
        separator_pressure = getattr(cfg.surface, 'separator_pressure', 20.0)
        # Fluid properties
        density = getattr(cfg.fluid, 'density', 850.0)          # kg/m3
        viscosity = getattr(cfg.fluid, 'viscosity', 0.012)     # Pa·s
        water_cut = getattr(cfg.fluid, 'water_cut', 0.20)      # fraction
        gor = getattr(cfg.fluid, 'gor', 120.0)                 # scf/bbl
        # Sand rate
        sand_rate = 0.0
        # Time
        time = 0.0
        # Reward placeholder
        reward = 0.0

        return WellState(
            Pr=Pr, Tr=Tr,
            Pwf=Pwf,
            Pth=Pth,
            opening_target=opening_target,
            opening_actual=opening_actual,
            effective_area=effective_area,
            pressure_drop=pressure_drop,
            Pwh=Pwh,
            Twh=Twh,
            separator_pressure=separator_pressure,
            oil_rate=0.0,
            gas_rate=0.0,
            water_rate=0.0,
            total_flow=0.0,
            density=density,
            viscosity=viscosity,
            water_cut=water_cut,
            gor=gor,
            sand_rate=sand_rate,
            time=time,
            reward=reward,
        )

    # -----------------------------------------------------------------
    # Core step
    # -----------------------------------------------------------------
    def step(self, choke_command: float) -> dict:
        """
        Advance the simulation by one time step.

        Args:
            choke_command: desired choke opening (%).

        Returns:
            Observation dictionary (sensor measurements).
        """
        cfg = self.config
        dt = self.dt
        state = self.state

        # 1. Apply disturbances (e.g., water breakthrough, sand, valve erosion, separator pressure shifts)
        disturbances.apply_disturbances(state, cfg, dt)

        # 2. Actuator dynamics (choke)
        choke.update_actuator(state, choke_command, dt, cfg)

        # 3. Effective area from actual opening
        state.effective_area = choke.effective_area(state, cfg)

        # 4. Solve for equilibrium flow rate Q_eq that satisfies inflow = orifice equation
        Q_eq, solver_info = self._solve_flow_rate_with_info(state, cfg)
        self._last_solver_info = solver_info
        
        # 5. Apply wellbore fluid inventory / momentum dynamics (lag tau_well)
        tau_well = getattr(cfg.well, 'wellbore_time_constant', 30.0)  # seconds
        if tau_well <= 0:
            state.total_flow = Q_eq
        else:
            dt_over_tau = min(1.0, dt / tau_well)
            if state.total_flow <= 0.0 and Q_eq > 0.0:
                state.total_flow = Q_eq
            else:
                state.total_flow += dt_over_tau * (Q_eq - state.total_flow)
                
        Q = state.total_flow

        # 6. Update reservoir pressure using actual produced flow Q
        reservoir.update_reservoir_pressure(state, Q, dt, cfg)

        # 7. Update intermediate pressures (Pwf, Pth) using solved Q (consistent with solution)
        PI = getattr(cfg.reservoir, 'productivity_index_si', 0.0)
        drawdown = (Q / PI) if PI > 0 else 0.0
        # Perforation loss
        C_perf = getattr(cfg, 'perforation', None)
        if C_perf is None:
            C_perf_val = 1e4  # bar/(m3/s)^2
        elif isinstance(C_perf, (int, float)):
            C_perf_val = float(C_perf)
        else:
            C_perf_val = getattr(C_perf, 'coefficient', 1e4)
        state.Pwf = state.Pr - drawdown - C_perf_val * Q * Q
        # Tubing loss
        rho = state.density if state.density > 0 else getattr(cfg.fluid, 'density', 850.0)
        depth = cfg.well.depth
        g = 9.81
        K_tub = getattr(cfg.well, 'friction_coefficient', 1e4)  # bar/(m3/s)^2
        hydrostatic = rho * g * depth / 1e5  # bar
        friction = K_tub * Q * Q
        state.Pth = state.Pwf - hydrostatic - friction

        # 8. Pressure drop across choke (already computed in solver, but recompute for clarity)
        delta_p_bar = state.Pth - state.separator_pressure
        if delta_p_bar < 0.0:
            delta_p_bar = 0.0
        state.pressure_drop = delta_p_bar

        # 9. Surface temperature/pressure (now uses total_flow for flowline loss)
        surface.update_surface(state, dt, cfg)

        # 10. Compute derived flow rates (oil, gas, water) from solved Q
        self._update_derived_quantities(state, Q, cfg)

        # 11. Increment simulation time
        state.time += dt

        # 12. Compute observation (with sensor noise, delay, etc.)
        obs = self._get_observation()

        # 13. Log if logger active
        if self._logger is not None:
            self._logger.log(self._state_to_dict())

        return obs

    def step_with_info(self, choke_command: float) -> tuple:
        """
        Advance simulation by one time step and return (observation, info_dict).
        
        info_dict contains solver convergence, true state, mass balance error, etc.
        """
        obs = self.step(choke_command)
        solver_info = getattr(self, '_last_solver_info', {'iterations': 0, 'converged': True, 'residual': 0.0})
        info = {
            "true_state": self._state_to_dict(),
            "solver_iterations": solver_info['iterations'],
            "mass_balance_error": solver_info['residual'],
            "flow_solver_converged": solver_info['converged'],
            "Q_eq": solver_info.get('Q_eq', self.state.total_flow),
            "time": self.state.time,
        }
        return obs, info

    def _solve_flow_rate(self, state: WellState, cfg: Config) -> float:
        Q, _ = self._solve_flow_rate_with_info(state, cfg)
        return Q

    # -----------------------------------------------------------------
    # Helper: solve for flow rate Q with solver diagnostics
    # -----------------------------------------------------------------
    def _solve_flow_rate_with_info(self, state: WellState, cfg: Config) -> tuple:
        """
        Solve for equilibrium volumetric flow rate Q (m3/s) where:
            Q = Q_choke(Q)
        Returns (Q_sol, solver_info_dict).
        """
        PI = getattr(cfg.reservoir, 'productivity_index_si', 0.0)
        C_perf_raw = getattr(cfg, 'perforation', None)
        if C_perf_raw is None:
            C_perf = 1e4
        elif isinstance(C_perf_raw, (int, float)):
            C_perf = float(C_perf_raw)
        else:
            C_perf = getattr(C_perf_raw, 'coefficient', 1e4)
            
        K_tub = getattr(cfg.well, 'friction_coefficient', 1e4)
        K_flow = getattr(cfg.surface, 'flowline_coefficient', 1e-4)
        rho = state.density if state.density > 0 else getattr(cfg.fluid, 'density', 850.0)
        depth = cfg.well.depth
        g = 9.81
        
        A_max = getattr(cfg.choke, 'max_area', 0.00025)
        n = getattr(cfg.choke, 'exponent', 1.6)
        Cd = getattr(cfg.choke, 'cd', 0.82)
        opening_actual = state.opening_actual
        opening_frac = max(0.0, min(1.0, opening_actual / 100.0))
        A = A_max * (opening_frac ** n)
        
        if A <= 1e-12:
            return 0.0, {'iterations': 0, 'converged': True, 'residual': 0.0, 'Q_eq': 0.0}

        Pr = state.Pr
        Psep = state.separator_pressure
        hydrostatic_bar = rho * g * depth / 1e5
        delta_p0 = Pr - Psep - hydrostatic_bar
        
        if delta_p0 <= 0:
            return 0.0, {'iterations': 0, 'converged': True, 'residual': 0.0, 'Q_eq': 0.0}
            
        alpha = C_perf + K_tub + K_flow  # combined quadratic loss coefficient

        def f(Q_val: float) -> float:
            drawdown = (Q_val / PI) if PI > 0 else 0.0
            delta_p = delta_p0 - drawdown - alpha * Q_val * Q_val
            if delta_p <= 0.0:
                return -Q_val
            delta_p_Pa = delta_p * 1e5
            q_choke = Cd * A * math.sqrt(2.0 * delta_p_Pa / rho)
            return q_choke - Q_val

        hi = 1.0  # max 1 m3/s
        lo = 0.0
        
        if f(lo) < 0:
            return 0.0, {'iterations': 0, 'converged': True, 'residual': abs(f(0.0)), 'Q_eq': 0.0}
            
        iterations = 0
        converged = False
        res = 0.0
        mid = 0.0
        
        for i in range(50):
            iterations += 1
            mid = 0.5 * (lo + hi)
            val = f(mid)
            res = abs(val)
            if res < 1e-9 or (hi - lo) < 1e-12:
                converged = True
                break
            if val > 0:
                lo = mid
            else:
                hi = mid
        Q_sol = max(mid, 0.0)
        info = {
            'iterations': iterations,
            'converged': converged,
            'residual': res,
            'Q_eq': Q_sol
        }
        return Q_sol, info

    # -----------------------------------------------------------------
    # Helper methods
    # -----------------------------------------------------------------
    def _update_derived_quantities(self, state: WellState, q: float, cfg: Config) -> None:
        """
        Derive oil, gas, water rates and total flow from the volumetric flux q.
        Uses water cut and GOR to split phases.
        Assumes q is the total volumetric flow rate at surface conditions (m3/s).
        """
        # Volumetric flow rates (m3/s)
        water_vol_frac = state.water_cut
        oil_vol_frac = 1.0 - water_vol_frac  # oil+gas mixture volume

        water_rate = water_vol_frac * q          # m3/s water
        oil_volume = oil_vol_frac * q            # m3/s oil (liquid rate)
        gas_volume = oil_volume * state.gor      # gas rate proportional to GOR

        state.oil_rate = oil_volume
        state.gas_rate = gas_volume
        state.water_rate = water_rate
        state.total_flow = q

        # Optional: compute a simple reward (e.g., oil rate minus penalties)
        # Penalty for high pressure (example)
        over_pressure = max(0.0, state.Pwh - 50.0)  # bar
        penalty = 0.1 * over_pressure
        state.reward = oil_volume - penalty

    def _get_observation(self) -> dict:
        """
        Apply sensor models to true state values and return observation dict.
        """
        cfg = self.config
        sens_cfg = getattr(cfg, 'sensors', None)
        if sens_cfg is None:
            # No sensor model – return raw state
            return self._state_to_dict()

        obs = {}
        # List of variables we want to sense
        vars_to_sense = [
            'Pr', 'Tr', 'Pwf', 'Pth',
            'opening_actual',
            'Pwh', 'Twh',
            'oil_rate', 'gas_rate', 'water_rate', 'total_flow',
            'water_cut', 'gor',
            'sand_rate',
        ]
        for var in vars_to_sense:
            true_val = getattr(self.state, var, 0.0)
            sen_conf = getattr(sens_cfg, var, None)
            if sen_conf is None:
                # create a dummy config with zero noise/delay/etc.
                class Dummy: pass
                sen_conf = Dummy()
                sen_conf.noise_std = 0.0
                sen_conf.delay_steps = 0
                sen_conf.quant_step = 0.0
                sen_conf.fault_prob = 0.0
                sen_conf.fault_type = 'none'
                sen_conf.fault_param = 0.0
            obs[var] = sensors.apply_sensor(
                true_val,
                sen_conf,
                self._sensor_state,
                var,
            )
        return obs

    def _state_to_dict(self) -> dict:
        """Convert the current WellState to a plain dict (for logging)."""
        return {
            'time': self.state.time,
            'Pr': self.state.Pr,
            'Tr': self.state.Tr,
            'Pwf': self.state.Pwf,
            'Pth': self.state.Pth,
            'opening_target': self.state.opening_target,
            'opening_actual': self.state.opening_actual,
            'effective_area': self.state.effective_area,
            'pressure_drop': self.state.pressure_drop,
            'Pwh': self.state.Pwh,
            'Twh': self.state.Twh,
            'separator_pressure': self.state.separator_pressure,
            'oil_rate': self.state.oil_rate,
            'gas_rate': self.state.gas_rate,
            'water_rate': self.state.water_rate,
            'total_flow': self.state.total_flow,
            'density': self.state.density,
            'viscosity': self.state.viscosity,
            'water_cut': self.state.water_cut,
            'gor': self.state.gor,
            'sand_rate': self.state.sand_rate,
            'reward': self.state.reward,
        }

    # -----------------------------------------------------------------
    # Logging control
    # -----------------------------------------------------------------
    def start_logging(self, filepath: str) -> None:
        """Initialise CSV logger."""
        fieldnames = logger.build_default_fieldnames(self.state)
        self._logger = logger.CSVLogger(filepath, fieldnames)

    def stop_logging(self) -> None:
        if self._logger is not None:
            self._logger.close()
            self._logger = None

    # -----------------------------------------------------------------
    # Reset
    # -----------------------------------------------------------------
    def reset(self) -> None:
        """Reset simulation to initial conditions."""
        pi_day_bar = getattr(self.config.reservoir, 'productivity_index', 50.0)
        self.config.reservoir.productivity_index_si = pi_day_bar / 86400.0
        self.state = self._initialize_state()
        self.time = 0.0
        self._sensor_state.clear()
        disturbances.reset_disturbance_state(self.state)
        # Keep logger open if desired; caller can close/reopen as needed.


# Convenience factory
def make_simulator(config_path: str = "configs/default.yaml") -> Simulator:
    return Simulator(config_path)