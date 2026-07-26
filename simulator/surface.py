"""
Surface module: compute wellhead pressure and temperature.
"""

from .state import WellState
from .config import Config


def update_surface(state: WellState, dt: float, config: Config) -> None:
    """
    Update wellhead pressure and temperature.

    Wellhead pressure is tubing head pressure minus flowline pressure drop.
    Temperature relaxes toward ambient with a time constant.
    """
    # Flowline backpressure drop: Pwh_eq = Psep + K_flow * Q^2
    K_flow = getattr(config.surface, 'flowline_coefficient', 100.0)  # bar/(m3/s)^2
    Psep = state.separator_pressure
    Pwh_target = Psep + K_flow * state.total_flow * state.total_flow
    
    tau_p = getattr(config.surface, 'flowline_time_constant', 10.0)  # seconds
    if tau_p <= 0:
        state.Pwh = Pwh_target
    else:
        dt_over_tau = min(1.0, dt / tau_p)
        if state.Pwh <= 0.0:
            state.Pwh = Pwh_target
        else:
            state.Pwh += dt_over_tau * (Pwh_target - state.Pwh)

    if state.Pwh < 0.0:
        state.Pwh = 0.0

    # Temperature dynamics: first‑order lag toward ambient
    # dT/dt = (1/τ_T) * (T_ambient - T)
    tau_temp = getattr(config.surface, 'temperature_time_constant', 100.0)  # seconds
    if tau_temp <= 0:
        state.Twh = config.surface.ambient_temperature
    else:
        T_amb = config.surface.ambient_temperature
        dt_over_tau = dt / tau_temp
        state.Twh += dt_over_tau * (T_amb - state.Twh)