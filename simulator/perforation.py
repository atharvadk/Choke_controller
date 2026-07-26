"""
Perforation module: pressure drop from reservoir to bottom hole due to inflow resistance.
"""

from .state import WellState
from .config import Config


def compute_bottom_hole_pressure(state: WellState, q: float, config: Config) -> float:
    """
    Compute bottom-hole flowing pressure (Pwf) considering perforation pressure loss.

    Simple quadratic loss: ΔP = C * q^2
    Pwf = Pr - ΔP

    Args:
        state: current well state (contains Pr)
        q: flow rate from reservoir (m^3/s)
        config: configuration containing perforation coefficient

    Returns:
        Bottom-hole flowing pressure (bar)
    """
    # Perforation coefficient (could be in config; default small)
    C_perf = getattr(config, 'perforation', None)
    if C_perf is None:
        # default: assume small pressure loss
        C_perf_val = 1e-5  # bar/(m^3/s)^2
    else:
        # If config.perforation is a numeric value, use it; else assume it has attribute 'coefficient'
        if isinstance(C_perf, (int, float)):
            C_perf_val = float(C_perf)
        else:
            C_perf_val = getattr(C_perf, 'coefficient', 1e-5)
    delta_p = C_perf_val * q * q
    pwf = state.Pr - delta_p
    # Ensure pressure doesn't go below zero
    if pwf < 0:
        pwf = 0.0
    return pwf