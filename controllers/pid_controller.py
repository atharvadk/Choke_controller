"""
PIDChokeController: Industrial Proportional-Integral-Derivative Choke Controller.

Features:
1. Flexible Setpoint Tracking (Oil Production Rate bbl/hr or WHP psi).
2. Anti-Windup Integral Action.
3. Filtered Derivative Action to suppress high-frequency noise.
4. Output Saturation & Slew Rate Limiting.
5. High-Priority Minimum WHP Pressure Safety Guard.
"""

from typing import Dict, Any, Optional
from .base_controller import BaseController

PSI_TO_BAR = 0.0689475729
BAR_TO_PSI = 14.5037738
M3S_TO_BBL_HR = 22643.4

class PIDChokeController(BaseController):
    def __init__(
        self,
        kp: float = 0.8,
        ki: float = 0.05,
        kd: float = 0.2,
        target_oil_bbl_hr: float = 120.0,
        min_whp_psi: float = 210.0,
        max_slew_rate_pct: float = 1.0,
        i_max: float = 30.0,
        dt: float = 1.0,
        name: str = "PID Controller"
    ):
        super().__init__(name=name)
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target_oil_bbl_hr = target_oil_bbl_hr
        self.min_whp_psi = min_whp_psi
        self.max_slew_rate_pct = max_slew_rate_pct
        self.i_max = i_max
        self.dt = dt
        
        self.integral = 0.0
        self.prev_error = 0.0
        self.filtered_d = 0.0
        self.current_choke = 30.0

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = 0.0
        self.filtered_d = 0.0
        self.current_choke = 30.0

    def compute_action(self, obs: Dict[str, Any], info: Optional[Dict[str, Any]] = None) -> float:
        # Measure current state
        oil_rate_m3s = obs.get('oil_rate', 0.0)
        oil_bbl_hr = oil_rate_m3s * M3S_TO_BBL_HR
        whp_bar = obs.get('Pwh', 15.0)
        whp_psi = whp_bar * BAR_TO_PSI
        choke_actual = obs.get('opening_actual', self.current_choke)
        
        # Safety Guard: Check if WHP is below safety limit
        if whp_psi < self.min_whp_psi:
            # Emergency override: trim choke to restore WHP
            whp_deficit = self.min_whp_psi - whp_psi
            delta_u = -min(self.max_slew_rate_pct * 1.5, 0.15 * whp_deficit)
            # Freeze integral during override to avoid windup
            new_choke = choke_actual + delta_u
            new_choke = max(0.0, min(100.0, new_choke))
            self.current_choke = new_choke
            return new_choke

        # Setpoint Error
        error = self.target_oil_bbl_hr - oil_bbl_hr
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with Anti-Windup clamp
        self.integral += self.ki * error * self.dt
        self.integral = max(-self.i_max, min(self.i_max, self.integral))
        
        # Derivative term with low-pass filter
        raw_d = (error - self.prev_error) / self.dt if self.dt > 0 else 0.0
        alpha_filter = 0.2
        self.filtered_d += alpha_filter * (raw_d - self.filtered_d)
        d_term = self.kd * self.filtered_d
        
        self.prev_error = error
        
        # Unclamped PID Output
        u_pid = p_term + self.integral + d_term
        
        # Determine choke step delta relative to current choke
        desired_choke = 30.0 + u_pid
        delta_u = desired_choke - choke_actual
        
        # Apply Slew Rate Limiting
        delta_u = max(-self.max_slew_rate_pct, min(self.max_slew_rate_pct, delta_u))
        
        new_choke = choke_actual + delta_u
        new_choke = max(0.0, min(100.0, new_choke))
        self.current_choke = new_choke
        return new_choke
