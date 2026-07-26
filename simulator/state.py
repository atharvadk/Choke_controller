"""
WellState: holds all dynamic state variables of the digital twin.

Each variable represents a physical quantity that evolves over time.
"""

from dataclasses import dataclass


@dataclass
class WellState:
    # Reservoir
    Pr: float = 0.0          # Reservoir Pressure (bar)
    Tr: float = 0.0          # Reservoir Temperature (°C)

    # Bottom Hole
    Pwf: float = 0.0         # Bottom Hole Flowing Pressure (bar)

    # Tubing
    Pth: float = 0.0         # Tubing Head Pressure (bar)

    # Choke
    opening_target: float = 0.0   # Desired choke opening (%)
    opening_actual: float = 0.0   # Actual choke opening (% after actuator lag)
    effective_area: float = 0.0   # Effective flow area (m^2)
    pressure_drop: float = 0.0    # Pressure drop across choke (bar)

    # Surface / Flowline
    Pwh: float = 0.0         # Wellhead Pressure (bar)
    Twh: float = 0.0         # Wellhead Temperature (°C)
    separator_pressure: float = 0.0  # Separator / Flowline Pressure (bar)

    # Production rates
    oil_rate: float = 0.0    # Oil rate (bbl/hr) or (m^3/s) - consistent units
    gas_rate: float = 0.0    # Gas rate (scf/hr) or (m^3/s)
    water_rate: float = 0.0  # Water rate (bbl/hr) or (m^3/s)
    total_flow: float = 0.0  # Total volumetric flow rate (m^3/s)

    # Fluid properties (could be constant, but kept here for possible updates)
    density: float = 0.0     # Fluid density (kg/m^3)
    viscosity: float = 0.0   # Fluid viscosity (Pa·s)
    water_cut: float = 0.0   # Water cut (fraction)
    gor: float = 0.0         # Gas-oil ratio (scf/bbl)

    # Sand and other
    sand_rate: float = 0.0   # Sand rate (kg/hr)

    # Time
    time: float = 0.0        # Simulation elapsed time (s)

    # Optional: for logging / reward
    reward: float = 0.0      # Instantaneous reward (e.g., oil rate - penalties)