# Technical Report & Mathematical Physics Specification: Autonomous Choke Control System

**Author**: Advanced Agentic Control Engineering Team  
**System Version**: 2.0 (Dynamic Twin Architecture)  

---

## 1. Executive Summary

This technical report documents the mathematical physics, dynamic state equations, numerical solver, and control formulations for the **Autonomous Choke Control Reduced-Order Digital Twin**. The simulator models an oil production well equipped with a surface choke valve operating under dynamic reservoir depletion, wellbore fluid inventory lag, surface backpressure dynamics, and choke actuator hysteresis.

The digital twin has been calibrated against field dataset observations, achieving a **2.43% Mean Relative Error on Bottom-Hole Pressure (BHP)**, **11.21% Error on Wellhead Pressure (WHP)**, and **14.45% Error on Produced Oil Rate**.

---

## 2. Mathematical Physics Model

```text
  Reservoir Inflow (Pr, PI)
            │
            ▼
  Perforation Loss (C_perf * Q^2)
            │
            ▼
  Bottom Hole Pressure (Pwf)
            │
            ▼
  Tubing Gravity + Friction (K_tub * Q^2)
            │
            ▼
  Tubing Head Pressure (Pth) ──► Orifice Solver (Cd, A(u), n)
                                        │
                                        ▼
                                  Flowline Backpressure (Pwh_eq = Psep + K_flow * Q^2)
```

### 2.1 Reservoir Inflow Performance Relationship (IPR)
Bottom-hole flowing pressure $P_{\text{wf}}$ is governed by drawdown across the formation and perforation skin resistance:

$$P_{\text{wf}} = P_r - \frac{Q}{\text{PI}_{\text{si}}} - C_{\text{perf}} \cdot Q^2$$

where:
- $P_r$: Reservoir pressure ($\text{bar}$)
- $\text{PI}_{\text{si}}$: Productivity Index ($\text{m}^3/\text{s}/\text{bar}$)
- $C_{\text{perf}}$: Perforation turbulence pressure loss coefficient ($\text{bar}/(\text{m}^3/\text{s})^2$)
- $Q$: Volumetric flow rate ($\text{m}^3/\text{s}$)

### 2.2 Wellbore Hydrostatics & Friction
Tubing Head Pressure $P_{\text{th}}$ (upstream of choke) includes hydrostatic column weight and Darcy-Weisbach tubing friction:

$$P_{\text{th}} = P_{\text{wf}} - \Delta P_{\text{hydrostatic}} - \Delta P_{\text{friction}}$$

$$\Delta P_{\text{hydrostatic}} = \frac{\rho \cdot g \cdot H}{10^5} \quad [\text{bar}]$$

$$\Delta P_{\text{friction}} = K_{\text{tub}} \cdot Q^2 \quad [\text{bar}]$$

where $\rho = 645\text{ kg/m}^3$, $g = 9.81\text{ m/s}^2$, $H = 3000\text{ m}$, and $K_{\text{tub}} = 500\text{ bar}/(\text{m}^3/\text{s})^2$.

### 2.3 Choke Orifice Flow Equation
Volumetric flow through the choke valve is modeled using the non-linear choke orifice equation:

$$Q_{\text{choke}} = C_d \cdot A(u) \cdot \sqrt{\frac{2 \cdot (P_{\text{th}} - P_{\text{wh}}) \cdot 10^5}{\rho}}$$

The effective area $A(u)$ as a function of actual choke opening $u \in [0, 100]\%$ follows a non-linear power-law scaling:

$$A(u) = A_{\text{max}} \cdot \left(\frac{u}{100}\right)^n$$

where $A_{\text{max}} = 2.8 \times 10^{-4}\text{ m}^2$, $C_d = 0.82$, and $n = 0.65$.

### 2.4 Surface Flowline Backpressure
Downstream wellhead pressure $P_{\text{wh}}$ tracks separator pressure $P_{\text{sep}}$ plus flowline quadratic resistance:

$$P_{\text{wh,eq}} = P_{\text{sep}} + K_{\text{flow}} \cdot Q^2$$

where $K_{\text{flow}} = 2.5 \times 10^5\text{ bar}/(\text{m}^3/\text{s})^2$ and $P_{\text{sep}} = 20.0\text{ bar}$ ($290\text{ psi}$).

---

## 3. Dynamic State Variables (Differential Equations)

Rather than assuming instantaneous quasi-steady equilibrium, the simulator models fluid inventory storage, pressure relaxation, and actuator hysteresis using coupled first-order ODEs:

### 3.1 Actuator Lag ($\tau_{\text{actuator}}$)
Choke valve opening position $u_{\text{actual}}$ relaxes toward setpoint command $u_{\text{target}}$:

$$\frac{d u_{\text{actual}}}{d t} = \frac{u_{\text{target}} - u_{\text{actual}}}{\tau_{\text{actuator}}} \quad (\tau_{\text{actuator}} = 15.0\text{ s})$$

### 3.2 Wellbore Fluid Storage & Momentum ($\tau_{\text{well}}$)
Fluid column inventory momentum delays flow rate response:

$$\frac{d Q}{d t} = \frac{Q_{\text{eq}}(u_{\text{actual}}, P_r, P_{\text{sep}}) - Q}{\tau_{\text{well}}} \quad (\tau_{\text{well}} = 45.0\text{ s})$$

### 3.3 Flowline Pressure Relaxation ($\tau_{\text{flowline}}$)
Wellhead pressure $P_{\text{wh}}$ exhibits first-order relaxation:

$$\frac{d P_{\text{wh}}}{d t} = \frac{P_{\text{wh,eq}} - P_{\text{wh}}}{\tau_{\text{flowline}}} \quad (\tau_{\text{flowline}} = 15.0\text{ s})$$

### 3.4 Reservoir Material Balance Depletion
Reservoir pressure declines smoothly as fluids are produced from pore volume $V_{\text{pore}}$:

$$\frac{d P_r}{d t} = -\frac{Q(t)}{c_t \cdot V_{\text{pore}}}$$

where total compliance $C_{\text{res}} = c_t \cdot V_{\text{pore}} = 350.0\text{ m}^3/\text{bar}$ ($V_{\text{pore}} = 3.5 \times 10^7\text{ m}^3$).

---

## 4. Control Formulations

1. **Rule-Based Controller**: Target production tracking with deadband, WHP minimum safety limit ($P_{\text{wh}} \ge 210\text{ psi}$), maximum drawdown guard, and slew rate limiting ($|\Delta u| \le 0.5\%/\text{step}$).
2. **Industrial PID Controller**: Setpoint tracking with anti-windup integral clamping ($I \in [-30, +30]$), low-pass filtered derivative action, output bounds, and WHP override guard.
3. **Non-Linear MPC (NMPC)**: Solves receding horizon trajectory optimization over prediction horizon $N=6$ steps using sub-millisecond dynamic state restoration.
4. **Actor-Critic RL Agent**: Continuous 2-layer MLP neural network policy trained under domain randomization ($Q_{\text{target}} \in [80, 150]\text{ bbl/hr}$, $P_{\text{sep}} \in [16, 24]\text{ bar}$).

---

## 5. Empirical Validation & Benchmark Findings

### 5.1 Dataset Calibration Performance
- **BHP Mean Relative Error**: **2.43%** ($86.4\text{ psi}$ RMSE / $73.2\text{ psi}$ MAE)
- **WHP Mean Relative Error**: **11.21%** ($29.6\text{ psi}$ RMSE / $27.4\text{ psi}$ MAE)
- **Oil Rate Mean Relative Error**: **14.45%** ($20.8\text{ bbl/hr}$ RMSE)

### 5.2 Monte Carlo Statistical Evaluation ($N=20$ Trials)
Under parameter uncertainty ($\text{PI} \pm 20\%$, $P_r \pm 5\text{ bar}$, separator noise):
- **RL Agent**: $1204.3 \pm 122.1\text{ bbl}$ oil, $233.7\text{ bbl}\cdot\text{h}$ IAE Error.
- **NMPC**: $1201.7 \pm 120.9\text{ bbl}$ oil, $236.3\text{ bbl}\cdot\text{h}$ IAE Error.
- **Rule-Based**: $1201.8 \pm 120.9\text{ bbl}$ oil, $236.2\text{ bbl}\cdot\text{h}$ IAE Error.
- **PID**: $1176.8 \pm 102.7\text{ bbl}$ oil, $261.2\text{ bbl}\cdot\text{h}$ IAE Error, $645.8\%$ Wear.
- **Student's $t$-test**: RL, NMPC, and Rule-Based performance difference is $p \approx 0.97$ ($p \ge 0.05$), while all statistically significantly outperform PID ($p = 0.021 < 0.05$).

### 5.3 Computational Decision Latency
- **Rule-Based**: $0.0007\text{ ms/step}$ ($0.7\,\mu\text{s}$)
- **PID**: $0.0010\text{ ms/step}$ ($1.0\,\mu\text{s}$)
- **RL Agent**: $0.0198\text{ ms/step}$ ($19.8\,\mu\text{s}$)
- **NMPC**: $0.1551\text{ ms/step}$ ($155.1\,\mu\text{s}$)
All controllers execute well within real-time industrial PLC constraints ($< 15\text{ ms}$).
