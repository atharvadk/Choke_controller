"""
ModelPredictiveChokeController: High-Speed Non-Linear Model Predictive Controller (NMPC).

Uses fast dynamic state propagation over prediction horizon N to achieve real-time optimization.
"""

import numpy as np
from typing import Dict, Any, Optional, Tuple
from scipy.optimize import minimize

from .base_controller import BaseController
from simulator.simulation import Simulator

PSI_TO_BAR = 0.0689475729
BAR_TO_PSI = 14.5037738
M3S_TO_BBL_HR = 22643.4

class ModelPredictiveChokeController(BaseController):
    def __init__(
        self,
        config_path: str = "configs/default.yaml",
        horizon: int = 5,
        dt_control: float = 60.0,
        target_oil_bbl_hr: float = 120.0,
        min_whp_psi: float = 210.0,
        max_slew_rate_pct: float = 2.0,
        w_oil: float = 2.0,
        w_whp: float = 10.0,
        w_smooth: float = 0.2,
        name: str = "Model Predictive Controller (MPC)"
    ):
        super().__init__(name=name)
        self.config_path = config_path
        self.horizon = horizon
        self.dt_control = dt_control
        self.target_oil_bbl_hr = target_oil_bbl_hr
        self.min_whp_psi = min_whp_psi
        self.max_slew_rate_pct = max_slew_rate_pct
        self.w_oil = w_oil
        self.w_whp = w_whp
        self.w_smooth = w_smooth
        
        self.internal_sim = Simulator(config_path)
        self.internal_sim.dt = dt_control
        self.current_choke = 30.0
        self.prev_u_seq = np.full(self.horizon, 30.0)

    def reset(self) -> None:
        self.current_choke = 30.0
        self.prev_u_seq = np.full(self.horizon, 30.0)
        self.internal_sim.reset()

    def _predict_trajectory_fast(self, Pr_0: float, Pwh_0: float, Q_0: float, u_actual_0: float, Psep: float, water_cut: float, u_seq: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Sub-millisecond dynamic state predictor over horizon N.
        """
        cfg = self.internal_sim.config
        dt = self.dt_control
        
        Pr = Pr_0
        Pwh = Pwh_0
        Q = Q_0
        u_act = u_actual_0
        
        oil_rates = []
        whps = []
        
        PI = getattr(cfg.reservoir, 'productivity_index_si', 0.0)
        C_perf = 1e4
        K_tub = getattr(cfg.well, 'friction_coefficient', 1e4)
        K_flow = getattr(cfg.surface, 'flowline_coefficient', 2.5e5)
        rho = getattr(cfg.fluid, 'density', 645.0)
        depth = cfg.well.depth
        g = 9.81
        hydrostatic_bar = rho * g * depth / 1e5
        
        A_max = getattr(cfg.choke, 'max_area', 0.00028)
        n = getattr(cfg.choke, 'exponent', 0.65)
        Cd = getattr(cfg.choke, 'cd', 0.82)
        tau_act = getattr(cfg.choke, 'stroke_time', 15.0)
        tau_well = getattr(cfg.well, 'wellbore_time_constant', 45.0)
        tau_p = getattr(cfg.surface, 'flowline_time_constant', 15.0)
        
        cv = getattr(cfg.reservoir, 'compressibility', 1e-5) * getattr(cfg.reservoir, 'volume', 3.5e7)
        if cv <= 0:
            cv = 350.0

        for u_target in u_seq:
            # 1. Actuator lag
            u_act += min(1.0, dt / tau_act) * (u_target - u_act)
            u_frac = max(0.0, min(1.0, u_act / 100.0))
            A = A_max * (u_frac ** n)
            
            # 2. Flow solver (fast 10-step bisection)
            if A <= 1e-12 or (Pr - Psep - hydrostatic_bar) <= 0:
                Q_eq = 0.0
            else:
                delta_p0 = Pr - Psep - hydrostatic_bar
                alpha = C_perf + K_tub + K_flow
                lo, hi = 0.0, 1.0
                for _ in range(12):
                    mid = 0.5 * (lo + hi)
                    drawdown = (mid / PI) if PI > 0 else 0.0
                    dp = delta_p0 - drawdown - alpha * mid * mid
                    if dp <= 0:
                        hi = mid
                    else:
                        q_c = Cd * A * np.sqrt(2.0 * dp * 1e5 / rho)
                        if q_c > mid:
                            lo = mid
                        else:
                            hi = mid
                Q_eq = max(0.5 * (lo + hi), 0.0)

            # 3. Wellbore storage lag
            Q += min(1.0, dt / tau_well) * (Q_eq - Q)
            
            # 4. Reservoir depletion
            Pr -= (Q * dt) / cv
            
            # 5. WHP surface backpressure lag
            Pwh_target = Psep + K_flow * Q * Q
            Pwh += min(1.0, dt / tau_p) * (Pwh_target - Pwh)
            
            # 6. Derived oil rate
            oil_q = (1.0 - water_cut) * Q
            oil_rates.append(oil_q * M3S_TO_BBL_HR)
            whps.append(Pwh * BAR_TO_PSI)

        return np.array(oil_rates), np.array(whps)

    def _cost_function(self, u_seq: np.ndarray, state_snapshot: tuple, current_choke: float) -> float:
        Pr_0, Pwh_0, Q_0, u_actual_0, Psep, water_cut = state_snapshot
        oil_rates, whps = self._predict_trajectory_fast(Pr_0, Pwh_0, Q_0, u_actual_0, Psep, water_cut, u_seq)
        
        # 1. Oil Rate Target Error Cost
        cost_oil = self.w_oil * np.sum(((oil_rates - self.target_oil_bbl_hr) / 10.0) ** 2)
        
        # 2. WHP Safety Violation Cost
        whp_violations = np.maximum(0.0, self.min_whp_psi - whps)
        cost_whp = self.w_whp * np.sum((whp_violations / 5.0) ** 2)
        
        # 3. Smoothness / Action Slew Penalty
        u_diffs = np.diff(np.insert(u_seq, 0, current_choke))
        cost_smooth = self.w_smooth * np.sum((u_diffs / self.max_slew_rate_pct) ** 2)
        
        return cost_oil + cost_whp + cost_smooth

    def compute_action(self, obs: Dict[str, Any], info: Optional[Dict[str, Any]] = None) -> float:
        choke_actual = obs.get('opening_actual', self.current_choke)
        oil_rate_m3s = obs.get('oil_rate', 0.0)
        oil_bbl_hr = oil_rate_m3s * M3S_TO_BBL_HR
        self.current_choke = choke_actual
        
        st = self.internal_sim.state
        Pr_0 = obs.get('Pr', st.Pr)
        Pwh_0 = obs.get('Pwh', st.Pwh)
        Q_0 = obs.get('total_flow', st.total_flow)
        Psep = obs.get('separator_pressure', 20.0)
        water_cut = obs.get('water_cut', 0.0)
        
        if info and "true_state" in info:
            ts = info["true_state"]
            Psep = ts.get('separator_pressure', Psep)
            water_cut = ts.get('water_cut', water_cut)
            Pr_0 = ts.get('Pr', Pr_0)
            Pwh_0 = ts.get('Pwh', Pwh_0)
            Q_0 = ts.get('total_flow', Q_0)

        state_snapshot = (Pr_0, Pwh_0, Q_0, choke_actual, Psep, water_cut)

        # Smart setpoint error initial guess warm start
        error = self.target_oil_bbl_hr - oil_bbl_hr
        est_target = np.clip(choke_actual + 0.25 * error, 0.0, 100.0)
        u_init = np.full(self.horizon, est_target)
        
        bounds = [(0.0, 100.0) for _ in range(self.horizon)]

        # Optimize trajectory (fast Nelder-Mead / Powell)
        res = minimize(
            self._cost_function,
            u_init,
            args=(state_snapshot, choke_actual),
            method='Powell',
            options={'maxiter': 8, 'ftol': 1e-2}
        )

        u_optimal_seq = np.clip(res.x, 0.0, 100.0)
        self.prev_u_seq = u_optimal_seq
        
        # Receding Horizon: apply first optimal action with slew rate limiting
        raw_u_opt = u_optimal_seq[0]
        delta_u = np.clip(raw_u_opt - choke_actual, -self.max_slew_rate_pct, self.max_slew_rate_pct)
        u_opt = np.clip(choke_actual + delta_u, 0.0, 100.0)
        
        self.current_choke = u_opt
        return u_opt
