"""
ChokeControlEnv: Standard OpenAI Gym / Gymnasium Compatible Environment for RL Research.

Exposes clean reset() and step(action) methods with rich observation spaces,
reward formulation, and diagnostic info dictionaries.
"""

import numpy as np
from typing import Dict, Any, Tuple, Optional
from simulator.simulation import Simulator

PSI_TO_BAR = 0.0689475729
BAR_TO_PSI = 14.5037738
M3S_TO_BBL_HR = 22643.4

class ChokeControlEnv:
    """
    Autonomous Choke Control Gym Environment.
    """
    def __init__(
        self,
        config_path: str = "configs/default.yaml",
        target_oil_bbl_hr: float = 120.0,
        min_whp_psi: float = 210.0,
        max_steps: int = 3600,
        dt: float = 1.0
    ):
        self.config_path = config_path
        self.target_oil_bbl_hr = target_oil_bbl_hr
        self.min_whp_psi = min_whp_psi
        self.max_steps = max_steps
        self.dt = dt
        
        self.sim = Simulator(config_path)
        self.sim.dt = dt
        self.current_step = 0
        
        # Action space: continuous target choke opening [0, 100]%
        self.action_low = 0.0
        self.action_high = 100.0
        
    def reset(self) -> Tuple[np.ndarray, Dict[str, Any]]:
        self.sim.reset()
        self.current_step = 0
        obs_dict = self.sim._get_observation()
        obs_vec = self._dict_to_vec(obs_dict)
        
        info = {
            "true_state": self.sim._state_to_dict(),
            "solver_iterations": 0,
            "mass_balance_error": 0.0,
            "flow_solver_converged": True,
            "time": 0.0
        }
        return obs_vec, info

    def step(self, action: float) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        choke_command = float(np.clip(action, self.action_low, self.action_high))
        
        prev_choke = self.sim.state.opening_actual
        obs_dict, info = self.sim.step_with_info(choke_command)
        self.current_step += 1
        
        # Extract physical metrics
        oil_rate_m3s = obs_dict.get('oil_rate', 0.0)
        oil_bbl_hr = oil_rate_m3s * M3S_TO_BBL_HR
        whp_bar = obs_dict.get('Pwh', 15.0)
        whp_psi = whp_bar * BAR_TO_PSI
        actual_choke = obs_dict.get('opening_actual', choke_command)
        delta_choke = abs(actual_choke - prev_choke)
        
        # Reward Function Formulation
        # 1. Production Reward
        r_prod = oil_bbl_hr / 100.0
        # 2. Target Error Penalty
        r_err = -0.05 * ((oil_bbl_hr - self.target_oil_bbl_hr) / 10.0) ** 2
        # 3. WHP Safety Violation Penalty
        r_whp = -1.0 * (max(0.0, self.min_whp_psi - whp_psi) / 10.0) ** 2
        # 4. Choke Actuator Wear Penalty
        r_wear = -0.1 * (delta_choke ** 2)
        
        reward = r_prod + r_err + r_whp + r_wear
        
        terminated = False
        truncated = self.current_step >= self.max_steps
        
        obs_vec = self._dict_to_vec(obs_dict)
        return obs_vec, reward, terminated, truncated, info

    def _dict_to_vec(self, obs_dict: Dict[str, Any]) -> np.ndarray:
        return np.array([
            obs_dict.get('Pr', 217.0) / 300.0,
            obs_dict.get('Pwf', 200.0) / 300.0,
            obs_dict.get('Pth', 30.0) / 100.0,
            obs_dict.get('Pwh', 20.0) / 100.0,
            obs_dict.get('opening_actual', 30.0) / 100.0,
            obs_dict.get('oil_rate', 0.0) * M3S_TO_BBL_HR / 200.0,
            self.target_oil_bbl_hr / 200.0,
        ], dtype=np.float32)
