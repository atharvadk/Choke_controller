"""
Tubing module: pressure loss due to gravity and friction in the tubing.
"""

from .state import WellState
from .config import Config


def compute_tubing_head_pressure(state: WellState, q: float, config: Config) -> float:
    """
    Compute tubing head pressure (pressure at top of tubing, just before choke).

    Pressure losses:
        - Hydrostatic (gravity): ΔP_g = ρ * g * h
        - Friction: ΔP_f = K * q^2

    Then: Pth = Pwf - ΔP_g - ΔP_f

    Assumes vertical tubing; deviation angle ignored for simplicity.

    Args:
        state: contains Pwf, fluid density (kg/m^3)
        q: flow rate (m^3/s)
        config: configuration containing well and fluid sections.

    Returns:
        Tubing head pressure (bar)
    """
    # Well depth (m)
    depth = getattr(config.well, 'depth', 3000.0)
    # Tubing inner diameter (m) – not used directly in simple friction term
    tubing_diameter = getattr(config.well, 'tubing_diameter', 0.10)
    # Friction coefficient (lumped) – can be placed in well section or tubing; we look in well
    K_tub = getattr(config.well, 'friction_coefficient', 1e-4)  # bar/(m^3/s)^2 placeholder

    # Fluid density (kg/m^3) – use state if updated, else from config
    rho = state.density if state.density > 0 else getattr(config.fluid, 'density', 850.0)
    # Gravity (m/s^2)
    g = 9.81

    # Hydrostatic pressure loss: ρ * g * h (Pa) → convert to bar (1 bar = 1e5 Pa)
    delta_p_g_pa = rho * g * depth
    delta_p_g_bar = delta_p_g_pa / 1e5

    # Friction loss: K * q^2 (bar)
    delta_p_f_bar = K_tub * q * q

    pth = state.Pwf - delta_p_g_bar - delta_p_f_bar
    # Prevent negative pressure (physical minimum is vacuum, but we set to zero)
    if pth < 0.0:
        pth = 0.0
    return pth