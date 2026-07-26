"""
RuleBasedChokeController: Heuristic, rule-based autonomous choke controller.

Features:
1. Target Oil Production Rate Tracking (bbl/hr).
2. Emergency Pressure Protection: trims choke if WHP falls below minimum safety limit.
3. Drawdown Guard: limits maximum pressure drawdown across perforations.
4. Slew-rate Limiting: caps maximum choke adjustment per step to prevent physical wear.
"""

from typing import Dict, Any, Optional
from .base_controller import BaseController

PSI_TO_BAR = 0.0689475729
BAR_TO_PSI = 14.5037738
M3S_TO_BBL_HR = 22643.4

class RuleBasedChokeController(BaseController):
    def __init__(
        self,
        target_oil_bbl_hr: float = 120.0,
        min_whp_psi: float = 210.0,
        max_drawdown_bar: float = 30.0,
        max_slew_rate_pct: float = 0.5,
        deadband_bbl_hr: float = 2.0,
        name: str = "Rule-Based Controller"
    ):
        super().__init__(name=name)
        self.target_oil_bbl_hr = target_oil_bbl_hr
        self.min_whp_psi = min_whp_psi
        self.max_drawdown_bar = max_drawdown_bar
        self.max_slew_rate_pct = max_slew_rate_pct
        self.deadband_bbl_hr = deadband_bbl_hr
        self.current_choke = 30.0

    def reset(self) -> None:
        self.current_choke = 30.0

    def compute_action(self, obs: Dict[str, Any], info: Optional[Dict[str, Any]] = None) -> float:
        # Extract sensor observations
        oil_rate_m3s = obs.get('oil_rate', 0.0)
        oil_bbl_hr = oil_rate_m3s * M3S_TO_BBL_HR
        whp_bar = obs.get('Pwh', 15.0)
        whp_psi = whp_bar * BAR_TO_PSI
        pr_bar = obs.get('Pr', 217.0)
        pwf_bar = obs.get('Pwf', 200.0)
        choke_actual = obs.get('opening_actual', self.current_choke)
        
        drawdown_bar = max(0.0, pr_bar - pwf_bar)
        
        delta_u = 0.0
        
        # Rule 1: Emergency Minimum WHP Protection Constraint (Highest Priority)
        if whp_psi < self.min_whp_psi:
            # WHP is dangerously low, cut choke back to build pressure
            whp_deficit = self.min_whp_psi - whp_psi
            delta_u = -min(self.max_slew_rate_pct * 2.0, 0.2 * whp_deficit)
            
        # Rule 2: Drawdown Protection Constraint
        elif drawdown_bar > self.max_drawdown_bar:
            # Drawdown too high, reduce flow to protect formation/sand inflow
            delta_u = -self.max_slew_rate_pct
            
        # Rule 3: Target Production Rate Tracking
        else:
            oil_error = self.target_oil_bbl_hr - oil_bbl_hr
            if abs(oil_error) > self.deadband_bbl_hr:
                # Proportional adjustment with slew rate cap
                step = 0.02 * oil_error
                delta_u = max(-self.max_slew_rate_pct, min(self.max_slew_rate_pct, step))

        # Update choke target
        new_choke = choke_actual + delta_u
        new_choke = max(0.0, min(100.0, new_choke))
        self.current_choke = new_choke
        return new_choke
