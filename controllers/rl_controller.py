"""
RLChokeController: Reinforcement Learning Policy Controller wrapper.

Loads trained Actor-Critic policy weights from models/rl_choke_policy.npz and computes
optimal choke action commands given observations and target setpoints.
"""

import os
import numpy as np
from typing import Dict, Any, Optional
from .base_controller import BaseController
from train_rl import ActorCriticPolicy

PSI_TO_BAR = 0.0689475729
BAR_TO_PSI = 14.5037738
M3S_TO_BBL_HR = 22643.4

class RLChokeController(BaseController):
    def __init__(
        self,
        model_path: str = "models/rl_choke_policy.npz",
        target_oil_bbl_hr: float = 120.0,
        min_whp_psi: float = 210.0,
        max_slew_rate_pct: float = 1.0,
        name: str = "Reinforcement Learning (RL)"
    ):
        super().__init__(name=name)
        self.model_path = model_path
        self.target_oil_bbl_hr = target_oil_bbl_hr
        self.min_whp_psi = min_whp_psi
        self.max_slew_rate_pct = max_slew_rate_pct
        
        self.policy = ActorCriticPolicy(state_dim=7, hidden_dim=32)
        if os.path.exists(model_path):
            self.policy.load(model_path)
            
        self.current_choke = 30.0

    def reset(self) -> None:
        self.current_choke = 30.0

    def compute_action(self, obs: Dict[str, Any], info: Optional[Dict[str, Any]] = None) -> float:
        choke_actual = obs.get('opening_actual', self.current_choke)
        oil_rate_m3s = obs.get('oil_rate', 0.0)
        
        # Build 7-dim observation vector
        obs_vec = np.array([
            obs.get('Pr', 217.0) / 300.0,
            obs.get('Pwf', 200.0) / 300.0,
            obs.get('Pth', 30.0) / 100.0,
            obs.get('Pwh', 20.0) / 100.0,
            choke_actual / 100.0,
            oil_rate_m3s * M3S_TO_BBL_HR / 200.0,
            self.target_oil_bbl_hr / 200.0,
        ], dtype=np.float32)

        # Policy forward pass (deterministic execution, no exploration noise)
        raw_action = self.policy.select_action(obs_vec, explore=False)
        
        # Apply Slew Rate Limiting
        delta_u = np.clip(raw_action - choke_actual, -self.max_slew_rate_pct, self.max_slew_rate_pct)
        u_opt = np.clip(choke_actual + delta_u, 0.0, 100.0)
        
        # Minimum WHP Safety Guard Override
        whp_psi = obs.get('Pwh', 15.0) * BAR_TO_PSI
        if whp_psi < self.min_whp_psi:
            whp_deficit = self.min_whp_psi - whp_psi
            u_opt = max(0.0, choke_actual - min(self.max_slew_rate_pct, 0.15 * whp_deficit))

        self.current_choke = u_opt
        return u_opt
