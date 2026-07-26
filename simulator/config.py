from dataclasses import dataclass
import yaml


@dataclass
class SimulationConfig:
    dt: float
    duration: float


@dataclass
class ReservoirConfig:
    pressure: float
    temperature: float
    productivity_index: float
    volume: float


@dataclass
class WellConfig:
    depth: float
    tubing_length: float
    tubing_diameter: float
    friction_coefficient: float = 1e4  # bar/(m3/s)^2
    wellbore_time_constant: float = 30.0  # seconds (well fluid inventory lag)


@dataclass
class FluidConfig:
    density: float
    viscosity: float
    water_cut: float
    gor: float


@dataclass
class ChokeConfig:
    cd: float
    max_area: float
    exponent: float
    stroke_time: float


@dataclass
class SurfaceConfig:
    separator_pressure: float
    ambient_temperature: float
    flowline_coefficient: float = 1e-4   # bar/(m3/s)^2
    temperature_time_constant: float = 100.0  # seconds
    flowline_time_constant: float = 10.0  # seconds (wellhead pressure lag)


@dataclass
class Config:
    simulation: SimulationConfig
    reservoir: ReservoirConfig
    well: WellConfig
    fluid: FluidConfig
    choke: ChokeConfig
    surface: SurfaceConfig


def load_config(path: str) -> Config:
    with open(path, "r") as f:
        data = yaml.safe_load(f)

    return Config(
        simulation=SimulationConfig(**data["simulation"]),
        reservoir=ReservoirConfig(**data["reservoir"]),
        well=WellConfig(**data["well"]),
        fluid=FluidConfig(**data["fluid"]),
        choke=ChokeConfig(**data["choke"]),
        surface=SurfaceConfig(**data["surface"]),
    )