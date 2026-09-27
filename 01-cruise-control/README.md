# Cruise Control System

A cruise control system modeled and controlled using MATLAB and Simulink.

## Overview

This project focuses on the modeling, analysis and control of a simplified vehicle cruise control system.

The vehicle is modeled using a first-order transfer function, and three classical controllers are investigated: P, PI and PID. Their performance is evaluated based on transient response, steady-state error and disturbance rejection.

The system is also implemented in Simulink to validate the MATLAB-based analysis.

## System Modeling

The vehicle dynamics are modeled considering:

- Motor force as the control input
- Vehicle velocity as the system output
- A resistive force proportional to vehicle velocity

The resulting model is a first-order continuous-time system with a single pole.

## Controllers

Three controllers were analyzed:

### P Controller

The proportional controller was evaluated for different values of the proportional gain.

Increasing the proportional gain improves the response speed and reduces the steady-state error, but does not completely eliminate the steady-state error.

### PI Controller

A PI controller was designed with:

- `Kp = 100`
- `Ki = 10`

The resulting closed-loop system achieved:

- Approximately 8% overshoot
- Approximately 50 s response time
- Zero steady-state error

### PID Controller

A PID controller was also investigated to evaluate the effect of the derivative component on the transient response.

For the simplified first-order vehicle model, the PID controller provided only minor improvements compared with the PI controller.

Based on the obtained results, the PI controller was selected for the final system.

## Disturbance Rejection

The robustness of the controlled system was evaluated by introducing an external disturbance representing a road slope.

The disturbance is applied at `t = 30 s`, increasing the resistive force acting on the vehicle.

The PI controller compensates for the disturbance by increasing the motor force and bringing the vehicle velocity back to the reference value.

This demonstrates the ability of the closed-loop system to reject external disturbances and maintain the desired speed.

## MATLAB Implementation

The system was implemented using MATLAB and Control System Toolbox.

Main functions used:

- `tf()` — transfer function modeling
- `feedback()` — closed-loop system construction
- `pid()` — controller definition
- `step()` — step response analysis
- `lsim()` — simulation with custom input signals

The P, PI and PID controllers were analyzed and their responses were compared.

## Simulink Implementation

The system was also implemented in Simulink using a classical negative-feedback control structure.

The model contains:

- Reference speed input
- Error calculation
- PI controller
- Disturbance input
- Vehicle transfer function
- Negative feedback loop
- Scope for monitoring the system response

The disturbance signal represents a road slope introduced after 30 seconds of operation.

The Simulink results are consistent with the MATLAB analysis.

## Key Results

| Controller | Steady-State Error | Overshoot | Response |
|------------|--------------------|-----------|----------|
| P | Non-zero | — | Faster with increasing `Kp` |
| PI | Zero | ~8% | Good compromise |
| PID | Zero | — | Minor improvement over PI |

For the simplified vehicle model, the PI controller provided the most suitable balance between response speed, stability and steady-state accuracy.

## Tools & Technologies

- MATLAB
- Simulink
- Control System Toolbox
- Transfer function modeling
- Classical control
- P / PI / PID control
- Disturbance rejection

## Files

- `cruise_control.m` — MATLAB implementation and analysis
- `cruise_control.slx` — Simulink model
