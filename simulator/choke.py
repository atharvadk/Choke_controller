"""
Choke module: actuator dynamics, effective area, orifice flow equation.
"""

import math
from .state import WellState
from .config import Config


def update_actuator(state: WellState, target_open: float, dt: float, config: Config) -> None:
    """
    First‑order actuator dynamics:
        tau * dO/dt = O_target - O_actual
    Euler integration:
        O_actual += (dt / tau) * (O_target - O_actual)
    """
    tau = getattr(config.choke, 'stroke_time', 5.0)  # seconds, default from yaml
    if tau <= 0:
        state.opening_actual = target_open
    else:
        dt_over_tau = dt / tau
        state.opening_actual += dt_over_tau * (target_open - state.opening_actual)
        # Clamp to physical limits
        if state.opening_actual < 0:
            state.opening_actual = 0.0
        if state.opening_actual > 100.0:
            state.opening_actual = 100.0
    state.opening_target = target_open


def effective_area(state: WellState, config: Config) -> float:
    """
    Compute effective flow area based on actual opening.
    A = A_max * (opening/100)^n
    """
    A_max = getattr(config.choke, 'max_area', 0.00025)  # m^2
    n = getattr(config.choke, 'exponent', 1.6)
    opening = state.opening_actual
    if opening < 0:
        opening = 0.0
    if opening > 100:
        opening = 100.0
    area = A_max * ((opening / 100.0) ** n)
    return area


def compute_flow(state: WellState, config: Config) -> float:
    """
    Compute flow rate through choke using orifice equation.
    Q = Cd * A * sqrt(2 * ΔP / ρ)
    with optional critical‑flow limit.

    Returns:
        Flow rate (m^3/s)
    """
    # Pressure drop across choke: upstream = tubing head pressure, downstream = separator pressure
    delta_p_bar = state.Pth - state.separator_pressure  # bar
    if delta_p_bar < 0:
        delta_p_bar = 0.0  # no reverse flow

    # Convert bar to Pa
    delta_p_pa = delta_p_bar * 1e5

    # Fluid density (kg/m^3)
    rho = state.density if state.density > 0 else config.fluid.density
    if rho <= 0:
        rho = 850.0  # fallback

    # Discharge coefficient
    Cd = getattr(config.choke, 'cd', 0.82)

    # Area
    A = effective_area(state, config)

    # Critical pressure drop (optional)
    delta_p_crit_bar = getattr(config.choke, 'critical_delta_p', None)
    if delta_p_crit_bar is not None and delta_p_bar > delta_p_crit_bar:
        delta_p_bar = delta_p_crit_bar
        delta_p_pa = delta_p_bar * 1e5

    # Avoid sqrt of negative
    if delta_p_pa <= 0:
        return 0.0

    q = Cd * A * math.sqrt(2.0 * delta_p_pa / rho)
    return q