"""
BaseController: Abstract base class for all choke control strategies.
"""

from abc import ABC, abstractmethod
from typing import Dict, Any, Optional

class BaseController(ABC):
    def __init__(self, name: str = "BaseController"):
        self.name = name

    @abstractmethod
    def compute_action(self, obs: Dict[str, Any], info: Optional[Dict[str, Any]] = None) -> float:
        """
        Compute the choke command (%) given observation dict and optional info dict.
        
        Returns:
            Choke opening command (%) in range [0, 100].
        """
        pass

    def reset(self) -> None:
        """Reset internal controller state (e.g. integral terms, previous errors)."""
        pass
