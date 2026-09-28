# Inverted Pendulum Control

An inverted pendulum control system modeled and controlled using MATLAB and Simulink.

## Overview

This project focuses on the modeling, analysis and control of an inverted pendulum mounted on a cart.

The system is nonlinear and inherently unstable around the upright equilibrium. The project covers the complete control design workflow, from nonlinear mathematical modeling and linearization to controller design, simulation, robustness analysis and nonlinear validation in Simulink.

Two control strategies are investigated:

- PID control using a cascaded control structure
- LQR control using full-state feedback

The two approaches are compared in terms of stabilization, control effort, reference tracking, robustness and disturbance rejection.

## System Modeling

The inverted pendulum consists of a cart moving along a horizontal rail and a pendulum attached to the cart through an ideal hinge.

The control input is the horizontal force applied to the cart.

The state vector contains:

- Cart position `x`
- Cart velocity `ẋ`
- Pendulum angle `θ`
- Pendulum angular velocity `θ̇`

The nonlinear equations of motion were derived and used to describe the complete system dynamics.

## Linearization

The nonlinear model was linearized around the unstable upright equilibrium `θ = 0`.

For small angles:

- `sin(θ) ≈ θ`
- `cos(θ) ≈ 1`

The resulting continuous-time state-space model was used for controller design and system analysis.

The validity of the linearized model was also investigated through simulations with progressively larger initial angles and disturbances.

## Controllability and Observability

The linearized state-space model was analyzed for controllability and observability.

The results confirmed that the system is controllable and observable, allowing the use of full-state feedback techniques such as LQR.

## Controller Design

### PID Control

A cascaded PID control structure was designed:

- Inner loop: PID control of the pendulum angle `θ`
- Outer loop: PD control of the cart position `x`

The inner loop was designed to be significantly faster than the outer loop to reduce interference between the two control loops.

The PID implementation was also evaluated under practical limitations such as actuator saturation and integrator windup.

### LQR Control

An LQR controller was designed using full-state feedback.

The control law is based on the complete state vector:

`F = -Kq`

where `q` contains the four system states.

The weighting matrices `Q` and `R` were selected based on acceptable maximum deviations of the system states and control effort.

The optimal feedback gain was obtained using MATLAB's `lqr` function.

## PID vs LQR

The two controllers were evaluated using stabilization and reference-tracking simulations.

| Metric | PID | LQR |
|-----------|-----------:|-----------:|
| Maximum control effort | 622.72 N | 14.75 N |
| Stabilization time for `θ` | 0.36 s | 2.69 s |
| Angular excursion for `xref = 0.05 m` | 2.03 rad | 0.006 rad |
| Cart position after disturbance | Remains displaced | Returns to zero |

The PID controller can stabilize the pendulum angle quickly, but requires significantly higher control effort and does not regulate the cart position to zero.

The LQR controller uses all four states simultaneously, allowing both the pendulum angle and cart position to be regulated.

## Robustness and Disturbances

Robustness was evaluated through:

- ±20% variations of the system parameters
- Force disturbances of 1 N and 20 N
- Progressively larger initial pendulum angles
- Actuator saturation
- Integrator windup and anti-windup protection

Both controllers remained stable for the tested parameter variations.

For force disturbances, both controllers were able to recover the pendulum angle toward equilibrium.

Larger disturbances were also investigated to determine the limits of the linearized model.

Actuator saturation was analyzed as a practical limitation of the control system. The effect of integrator windup was investigated and anti-windup protection was implemented to improve the behavior of the saturated system.

## Simulink Implementation

The system was implemented in Simulink using the full nonlinear pendulum dynamics.

The nonlinear equations were implemented using a MATLAB Function block, followed by integration of the complete system state.

The Simulink model was used to test and compare:

- PID control with derivative filtering and actuator saturation
- LQR full-state feedback with actuator saturation
- PID and LQR stabilization performance
- System response to disturbances

The nonlinear Simulink implementation was used to validate the main conclusions obtained from the linear MATLAB simulations.

## Key Results

- A nonlinear mathematical model of the inverted pendulum was developed.
- The nonlinear model was linearized around the upright equilibrium.
- Controllability and observability of the linearized system were verified.
- Both PID and LQR controllers were successfully designed and simulated.
- PID provided faster angular stabilization but required significantly higher control effort.
- LQR controlled the complete state vector and provided better cart-position regulation.
- Both controllers remained stable under the tested parameter variations.
- Actuator saturation and integrator windup were investigated as practical control limitations.
- The nonlinear Simulink model was used to validate the control strategies.

## Tools & Technologies

- MATLAB
- Simulink
- Control System Toolbox
- State-space modeling
- Nonlinear system modeling
- Linearization
- PID control
- LQR control
- Controllability and observability analysis
- Robustness analysis
- Disturbance rejection
- Actuator saturation
- Integrator windup

## Files

- MATLAB scripts for mathematical modeling, controller design and simulations
- Simulink model containing the nonlinear plant and controller configurations
- Project documentation
