# API Documentation: Autonomous Choke Control System

This document provides a detailed API reference for the simulator engine, configuration dataclasses, controller implementations, and Gym environment.

---

## 1. Simulator Core API (`simulator.simulation.Simulator`)

### Class: `Simulator(config_path: str)`

Main simulation orchestrator that couples reservoir inflow, wellbore hydraulics, choke orifice flow, and dynamic state propagation.

#### Methods

- **`reset() -> dict`**
  Resets all physical states to initial configuration values.
  *Returns*: Initial observation dictionary.

- **`step(choke_command: float) -> dict`**
  Advances the physical simulation by one time step `dt` given a target choke opening command.
  *Parameters*:
    - `choke_command` (float): Target choke opening percentage in range `[0, 100]%`.
  *Returns*: Sensed observation dictionary (with noise if configured).

- **`step_with_info(choke_command: float) -> tuple[dict, dict]`**
  Advances simulation by one time step `dt` and returns both the observation and detailed diagnostic metadata.
  *Parameters*:
    - `choke_command` (float): Target choke opening percentage `[0, 100]%`.
  *Returns*: `(observation_dict, info_dict)`
  - `info_dict` contents:
    - `"true_state"` (dict): Uncorrupted internal physical state values.
    - `"solver_iterations"` (int): Bisection root-solver iteration count.
    - `"mass_balance_error"` (float): Residual mass balance error $|f(Q)|$.
    - `"flow_solver_converged"` (bool): Convergence flag (`True`/`False`).
    - `"Q_eq"` (float): Solved static equilibrium flow rate ($\text{m}^3/\text{s}$).
    - `"time"` (float): Elapsed simulation time ($\text{seconds}$).

---

## 2. Dynamic State Dataclass (`simulator.state.WellState`)

Holds all physical state variables of the wellbore system:

| Attribute | Type | Units | Description |
| :--- | :--- | :--- | :--- |
| `Pr` | `float` | `bar` | Reservoir pressure (depletes smoothly via material balance) |
| `Pwf` | `float` | `bar` | Bottom-hole flowing pressure (incorporates PI & perforation loss) |
| `Pth` | `float` | `bar` | Tubing head pressure (upstream of choke valve) |
| `Pwh` | `float` | `bar` | Wellhead / flowline pressure (downstream of choke, dynamic lag $\tau_{\text{flowline}}$) |
| `total_flow` | `float` | $\text{m}^3/\text{s}$ | Total volumetric flow rate (dynamic storage/momentum lag $\tau_{\text{well}}$) |
| `opening_actual`| `float` | `%` | Actual choke valve opening position (actuator stroke lag $\tau_{\text{actuator}}$) |
| `oil_rate` | `float` | $\text{m}^3/\text{s}$ | Produced oil volumetric flow rate |
| `water_rate` | `float` | $\text{m}^3/\text{s}$ | Produced water volumetric flow rate |
| `gas_rate` | `float` | $\text{m}^3/\text{s}$ | Produced gas volumetric flow rate |
| `water_cut` | `float` | fraction | Produced water fraction $0 \le \text{WC} \le 1$ |
| `separator_pressure`| `float` | `bar` | Downstream surface separator pressure |

---

## 3. Controller Package API (`controllers`)

All controllers inherit from abstract base class `BaseController`.

### Abstract Base Class: `BaseController`
- `compute_action(obs: dict, info: dict | None = None) -> float`: Computes choke opening command `%`.
- `reset() -> None`: Resets controller internal memory.

---

### Class: `RuleBasedChokeController`
Heuristic control strategy with safety bounds and deadband setpoint tracking.

```python
RuleBasedChokeController(
    target_oil_bbl_hr: float = 120.0,
    min_whp_psi: float = 210.0,
    max_drawdown_bar: float = 30.0,
    max_slew_rate_pct: float = 0.5,
    deadband_bbl_hr: float = 2.0
)
```

---

### Class: `PIDChokeController`
Industrial Proportional-Integral-Derivative controller with anti-windup clamping and low-pass derivative filter.

```python
PIDChokeController(
    kp: float = 0.8,
    ki: float = 0.05,
    kd: float = 0.2,
    target_oil_bbl_hr: float = 120.0,
    min_whp_psi: float = 210.0,
    max_slew_rate_pct: float = 1.0,
    i_max: float = 30.0,
    dt: float = 1.0
)
```

---

### Class: `ModelPredictiveChokeController`
Non-Linear Model Predictive Controller (NMPC) using vectorised dynamic forward prediction and receding horizon optimization.

```python
ModelPredictiveChokeController(
    config_path: str = "configs/default.yaml",
    horizon: int = 6,
    dt_control: float = 60.0,
    target_oil_bbl_hr: float = 120.0,
    min_whp_psi: float = 210.0,
    max_slew_rate_pct: float = 2.0,
    w_oil: float = 2.0,
    w_whp: float = 10.0,
    w_smooth: float = 0.1
)
```

---

### Class: `RLChokeController`
Reinforcement Learning Policy Controller loading trained Actor-Critic neural network weights.

```python
RLChokeController(
    model_path: str = "models/rl_choke_policy.npz",
    target_oil_bbl_hr: float = 120.0,
    min_whp_psi: float = 210.0,
    max_slew_rate_pct: float = 1.0
)
```

---

## 4. Reinforcement Learning Gym Environment (`controllers.gym_env.ChokeControlEnv`)

Standard OpenAI Gym / Gymnasium environment wrapper:

```python
env = ChokeControlEnv(
    config_path="configs/default.yaml",
    target_oil_bbl_hr=120.0,
    min_whp_psi=210.0,
    max_steps=3600,
    dt=1.0
)

obs, info = env.reset()
obs, reward, terminated, truncated, info = env.step(action)
```

- **Observation Vector (7-dim)**:
  `[Pr / 300, Pwf / 300, Pth / 100, Pwh / 100, opening_actual / 100, oil_rate_bbl_hr / 200, target_oil_bbl_hr / 200]`
- **Action Space**: Continuous scalar choke target `[0, 100]%`.
