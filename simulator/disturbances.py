"""
Disturbance module: injects random or deterministic faults/alterations
to simulate real‑world issues (water breakthrough, sand, valve erosion, etc.).
"""

import random
from .state import WellState
from .config import Config


def apply_disturbances(state: WellState, config: Config, dt: float) -> None:
    """
    Modify the well state or parameters to reflect disturbances.
    Called each simulation step.
    """
    dist = getattr(config, 'disturbances', None)
    if dist is None:
        return

    # Water breakthrough: increase water cut
    pw_prob = getattr(dist, 'water_breakthrough_prob', 0.0)
    if random.random() < pw_prob:
        inc = getattr(dist, 'water_breakthrough_increment', 0.01)  # fraction per step
        state.water_cut = min(1.0, state.water_cut + inc)
        # optionally adjust oil/gas rates proportionally (handled elsewhere)

    # Gas breakthrough: increase GOR
    pg_prob = getattr(dist, 'gas_breakthrough_prob', 0.0)
    if random.random() < pg_prob:
        inc = getattr(dist, 'gas_breakthrough_increment', 10.0)  # scf/bbl per step
        state.gor += inc

    # Sand spike: increase sand rate temporarily
    ps_prob = getattr(dist, 'sand_spike_prob', 0.0)
    if random.random() < ps_prob:
        spike = getattr(dist, 'sand_spike_magnitude', 5.0)  # kg/hr
        state.sand_rate += spike
        # could decay over time; for simplicity we let it persist until another event reduces it

    # Valve stiction: temporarily reduce actuator responsiveness
    vs_prob = getattr(dist, 'valve_stiction_prob', 0.0)
    if random.random() < vs_prob:
        # Increase effective stroke time (slower response) for a few steps
        # We'll store a temporary multiplier in state if needed; here we just increase stroke_time in config?
        # Simpler: reduce the actual change in opening this step handled elsewhere.
        # We'll add a temporary flag to choke module; for now just increase stroke_time in a local copy.
        pass  # placeholder

    # Valve erosion: slowly increase max_area
    ve_prob = getattr(dist, 'valve_erosion_prob', 0.0)
    if random.random() < ve_prob:
        inc = getattr(dist, 'valve_erosion_increment', 1e-6)  # m^2 per step
        # We'll modify choke config's max_area via a mutable attribute; but config is usually immutable.
        # Instead we store an effective max_area in state.
        if not hasattr(state, 'eff_max_area'):
            state.eff_max_area = getattr(getattr(config, 'choke', None), 'max_area', 0.00025)
        state.eff_max_area += inc

    # Separator pressure change (e.g., ups/downs)
    sp_prob = getattr(dist, 'separator_pressure_shift_prob', 0.0)
    if random.random() < sp_prob:
        delta = getattr(dist, 'separator_pressure_shift_magnitude', 0.5)  # bar
        direction = 1 if random.random() > 0.5 else -1
        state.separator_pressure += direction * delta
        # keep within reasonable bounds
        min_sp = getattr(dist, 'separator_pressure_min', 1.0)
        max_sp = getattr(dist, 'separator_pressure_max', 50.0)
        if state.separator_pressure < min_sp:
            state.separator_pressure = min_sp
        if state.separator_pressure > max_sp:
            state.separator_pressure = max_sp

    # Optional: random seed for reproducibility (handled outside)


def reset_disturbance_state(state: WellState) -> None:
    """
    Clear any disturbance‑specific state variables (called on reset).
    """
    if hasattr(state, 'eff_max_area'):
        delattr(state, 'eff_max_area')
    # sand_rate could be reset to base value; but we keep as part of state.