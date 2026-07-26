"""
Reservoir module: computes inflow from reservoir to wellbore and updates reservoir pressure.
"""

from .state import WellState
from .config import Config


def compute_inflow(state: WellState, config: Config) -> float:
    """
    Compute reservoir inflow (q) using productivity index.

    q = PI * (Pr - Pwf)

    Returns:
        Volumetric flow rate (m^3/s) into the wellbore.
        Positive means flow from reservoir to well.
    """
    # Ensure non-negative flow (if Pwf > Pr, no inflow)
    delta_p = max(state.Pr - state.Pwf, 0.0)
    # Use productivity index in SI units (m^3/(s*bar))
    pi = getattr(config.reservoir, 'productivity_index_si', 2.0 / 86400.0)  # fallback if not set
    q = pi * delta_p
    return q


def update_reservoir_pressure(state: WellState, q: float, dt: float, config: Config) -> None:
    """
    Update reservoir pressure based on produced volume.

    Simple material balance: dP/dt = - (q * B) / (c * V)
    where B is formation volume factor, c is compressibility, V is pore volume.
    For demonstration we use a lumped constant k_res.

    We'll compute:
        dP = - (q * dt) / (c_t * V_p)
    where c_t * V_p is stored as reservoir.compressibility * reservoir.volume.
    If those are not provided, we fall back to a nominal value.
    """
    # Get reservoir compressibility and volume from config if available
    comp = getattr(config.reservoir, 'compressibility', 1e-5)  # 1/bar, typical oil compressibility
    vol = getattr(config.reservoir, 'volume', 1e5)          # m^3 pore volume
    # Product c*V has units: (1/bar) * m^3 = m^3/bar
    cv = comp * vol
    if cv <= 0:
        cv = 1.0  # avoid division by zero; fallback
    # Pressure drop in bar
    delta_p_bar = (q * dt) / cv
    state.Pr -= delta_p_bar
    if state.Pr < 0:
        state.Pr = 0.0