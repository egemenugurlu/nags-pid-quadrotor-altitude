# NAGS-PID: Neural Gain-Scheduled PID for 1D Quadrotor Altitude Control

A pole-placement PID controller for quadrotor altitude, compared against an MLP that schedules the PID gains online within a bounded range. Everything runs in MATLAB/Simulink (block-diagram model) with the MLP trained in Python.

**Status (23 Oct 2026):** the pipeline is complete and validated, but the MLP does **not** show a clear overall advantage over a fixed 1.4x-gain PID. It lowers step overshoot at the price of slower recovery after disturbances (see [Results](#results) and [Limitations](#limitations)). This repository is the frozen state of that experiment (tag `v1-mlp-gain-scheduling`).

## What is in here

| Part | Description |
|---|---|
| Plant | 1D altitude, equivalent single thrust from 4 motors, first-order motor lag, quadratic drag, thrust saturation |
| Baseline | PID by pole placement, derivative on measurement, conditional-integration anti-windup |
| Scheduler | MLP `3-16-16-3` (ReLU, ReLU, tanh), stateless static map, gains bounded to +/-40 % of nominal |
| Training | Evolution Strategies (antithetic sampling), closed-loop cost, 9 training scenarios, held-out test scenario |
| Evaluation | Three modes in one Simulink model: nominal PID, fixed 1.4x PID, MLP |

### Plant

```
m*z'' = U - m*g - c_d*z'*|z'| + d(t)
U = 4*k_f*w^2,    w' = (w_cmd - w) / tau_m
```

`m = 1 kg`, `tau_m = 0.05 s`, `U_max = 2*m*g = 19.62 N`, hover at `w = 700 rad/s`.

### Controller

```
U_cmd = m*g + Kp*e + Ki*Ie - Kd*z'
```

Nominal gains by pole placement (`zeta = 0.707`, `wn = 4 rad/s`, `alpha = 2`): `Kp = 27.312`, `Ki = 32.0`, `Kd = 7.656`.

The MLP takes `(e/0.5, z'/2, Ie/2)` and outputs `K = K0 * (1 + 0.40 * tanh(...))` for each gain. Frozen-gain stability of the whole +/-40 % box was checked with Routh-Hurwitz. This is **not** a stability guarantee for time-varying gains.

### Cost function

```
J = ITAE + 150*step_overshoot + 150*wind_drop + 25*wind_rebound + 25*sine_rms
    + 0.0005*gain_chatter + 60*saturation_penalty + 40*control_effort
```

Each scenario's cost is divided by the nominal PID's cost on that scenario. The controller sees noisy measurements (`sigma_z = 5 mm`, `sigma_v = 5 cm/s`) during training and testing. The overshoot/drop weights (150) were raised after observing a large overshoot in an earlier run; the effort weight was chosen from a fixed-gain sweep. Both are design choices, not optimal values.

### Scenarios

- **Test (never seen in training):** step 0 to 0.2 m at t = 1 s, -6 N wind pulse (5-8 s), 3 N / 2 rad/s sinusoidal wind (from 12 s), T = 20 s.
- **Training (9):** reference 0.15 / 0.20 / 0.30 m, pulses -3 / -4.5 / -8 N, three sinusoids (2 N @ 1, 4 N @ 1.5, 2.5 N @ 3 rad/s).

## Results

Test scenario, Python simulator, no measurement noise (the Simulink model has no noise blocks):

| Metric | Nominal PID (Mode 0) | 1.4x fixed PID (Mode 1) | NAGS-PID / MLP (Mode 2) |
|---|---|---|---|
| Overshoot (%) | 33.30 | 23.74 | 12.79 |
| Wind dip (m) | 0.2065 | 0.1383 | 0.1525 |
| Rebound (m) | 0.2044 | 0.1365 | 0.1357 |
| Settling time (s) | 2.08 | 1.99 | nan (does not settle within 1 cm by t = 12 s) |
| Sine peak-to-peak (m) | 0.2609 | 0.1778 | 0.1754 |
| RMS error (m) | 0.0853 | 0.0581 | 0.0596 |
| ITAE | 12.59 | 8.72 | 10.68 |
| Max U_cmd (N) | 18.55 | 18.63 | 18.85 |

With measurement noise (control-signal jitter, RMS of `dU_cmd`): 0.58 N, 0.81 N and 0.70 N for modes 0, 1 and 2.

The Python simulator reproduces the Simulink nominal-PID results within about 2 % (overshoot 33.3 vs 33.1 %, ITAE 12.5885 vs 12.5967). Simulink responses for all three modes are consistent with the Python results; see `results/`.

## Limitations

- No clear overall win for the MLP: lower overshoot, but slower recovery and higher ITAE than the fixed 1.4x PID.
- Cost weights were adjusted after seeing results; single training seed.
- The fair baseline is missing: a fixed PID with independently optimized `Kp, Ki, Kd` for the same cost.
- Stability analysis covers frozen gains only.
- The Simulink model has no measurement noise; the noisy comparison exists only in Python.
- Mass changes and varying wind regimes were not tested.

## Repository layout

```
quadrotor_1d_v3.slx      Simulink model (3 modes via ctrl_mode)
quad_params_v2.m         physical parameters, scenario settings, MLP weights loader
design_pid_v2.m          pole-placement PID design
nags_pid_fcn.m           MLP forward pass (MATLAB Function block)
run_compare_v2.m         simulation runner, metrics and plots
train_mlp.py             simulator, cost function, ES training, test tables
nags_pid_weights.json    trained MLP weights used for the results above
quad_sim.py              early Python reference simulator / nominal gain calculation
results/                 figures and response plots
archive/                 earlier model and script versions
```

## How to reproduce

**Training (Python, numpy only, about 6 minutes):**

```bash
pip install numpy
python train_mlp.py
```

This writes `nags_pid_weights.json` and prints the test tables (without and with noise).

**Simulink comparison (MATLAB):**

1. Set the solver to variable-step with `Max step size = 0.005` in `quadrotor_1d_v3`.
2. For each mode run `ctrl_mode = 0; run_compare_v2` (then `1`, then `2`).
   - `0` nominal PID, `1` fixed 1.4x PID, `2` MLP.

`run_compare_v2.m` overwrites `baseline_pid.mat` on every run; keep a copy of each mode's output if you need it.

## Next

Moving to a first-principles LQR + Kalman filter (LQG) approach for quadcopter control, built up from altitude to full 3D, with the PID and MLP results here kept as baselines.
