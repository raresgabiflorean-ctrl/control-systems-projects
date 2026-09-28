# Two-Tank Level Control System

A coupled two-tank liquid level control system modeled and controlled using MATLAB and Simulink.

## Overview

This project focuses on the modeling, analysis and control of a liquid level system consisting of two coupled tanks.

The system is modeled as a second-order continuous-time process, and classical P, I, PI and PID controllers are investigated. The controllers are evaluated based on transient response, steady-state error, stability margins and control effort.

The system is implemented in both MATLAB and Simulink, with additional robustness tests involving parameter variations, flow disturbances and actuator saturation.

## System Modeling

The process consists of two tanks connected in series:

- Tank 1 receives the controlled input flow
- Tank 1 drains into Tank 2 through a hydraulic resistance
- Tank 2 drains to the outside through a second hydraulic resistance
- The controlled output is the liquid level of Tank 2

The system is represented using a state-space model with the two tank levels as state variables.

The resulting process is a second-order continuous-time system with two real negative poles and no zeros.

## System Analysis

The open-loop process was analyzed using:

- Pole and zero analysis
- Stability analysis
- Nyquist diagram
- Step response

The system has two real negative poles, confirming stable and overdamped behavior.

## Controller Design

Several classical controllers were investigated:

### P Controller

A proportional controller was initially tested using root locus analysis.

The P controller cannot completely eliminate the steady-state error, regardless of the proportional gain.

### I Controller

A pure integral controller was also investigated.

The integral action eliminates the steady-state error for a step reference, but does not provide the same control over the transient response as a combined PI controller.

### PI Controller

A PI controller was designed using MATLAB's `pidtune` function.

The resulting parameters are:

- `Kp = 0.0281`
- `Ki = 0.0212`

For the closed-loop system, the PI controller achieved:

- Rise time: approximately 1.88 s
- Settling time: approximately 7.42 s
- Overshoot: approximately 11.42%
- Zero steady-state error

### PID Controller

A PID controller was also designed for comparison.

The resulting parameters are:

- `Kp = 0.0346`
- `Ki = 0.0216`
- `Kd = 0.0117`

A filtered derivative was used in the practical implementation.

The PID controller achieved a lower overshoot than the PI controller, but required significantly greater control effort at the initial instant.

## PI vs PID

| Parameter | PI | PID |
|-----------|----|-----|
| Rise Time | 1.8821 s | 2.3308 s |
| Settling Time | 7.4224 s | 8.0663 s |
| Overshoot | 11.42% | 6.90% |
| Steady-State Error | 0 | 0 |

Although the PID controller provides lower overshoot, the control signal exhibits a very large initial peak due to the derivative action.

The PI controller therefore provides a more practical solution for the modeled system, considering both transient performance and actuator limitations.

## MATLAB Analysis

The system was analyzed and simulated using MATLAB and Control System Toolbox.

The analysis includes:

- Step response
- Control signal analysis
- Gain and phase margins
- Linearity verification
- PI vs PID comparison

Both controllers have infinite gain margin and phase margins above 45°.

## Robustness Tests

The robustness of the PI-controlled system was evaluated under several conditions.

### Parameter Variations

The physical parameters `C1`, `C2`, `R1` and `R2` were varied by ±20%.

The closed-loop system remained stable, with moderate changes in transient performance.

### Flow Disturbances

Flow disturbances were introduced to simulate effects such as leakage or changes in the outlet flow.

For moderate disturbances, the integral action allows the system to return to the reference value with zero steady-state error.

A larger disturbance was also tested, producing a significantly larger temporary deviation before the system returned toward the reference.

### Actuator Saturation and Integrator Windup

A physical actuator limitation was introduced by saturating the pump flow at a maximum value.

When the actuator becomes saturated, the integral component continues accumulating error even though the actuator cannot provide the required control action. This produces integrator windup and can result in a permanent deviation from the desired level.

This test highlights the importance of anti-windup strategies in practical control systems.

## Simulink Implementation

The control system was implemented in Simulink using a classical negative-feedback structure.

The model contains:

- Step reference input
- Error calculation
- PI/PID controller
- Transfer function representing the two-tank process
- Feedback loop
- Scope for monitoring the system response

Additional Simulink models were used to investigate:

- Flow disturbances
- Actuator saturation
- Integrator windup
- PI vs PID performance

The Simulink results were consistent with the MATLAB simulations.

## Key Results

| Test | Result |
|------|--------|
| Parameter variation ±20% | System remains stable |
| Moderate flow disturbance | Full recovery to reference |
| Large flow disturbance | Large temporary deviation |
| Actuator saturation | Integrator windup |
| PI controller | Zero steady-state error |
| PID controller | Lower overshoot but significantly higher initial control effort |

## Tools & Technologies

- MATLAB
- Simulink
- Control System Toolbox
- Transfer function modeling
- State-space modeling
- Classical control
- P / I / PI / PID control
- Nyquist analysis
- Root locus
- Robustness analysis
- Disturbance rejection
- Actuator saturation
- Integrator windup

## Files

- `two_tank_control.m` — MATLAB implementation and analysis
- `two_tank_control.slx` — Simulink model
