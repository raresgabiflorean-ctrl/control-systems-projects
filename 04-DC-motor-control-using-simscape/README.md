# DC Motor Control Using Simscape

A DC motor control system modeled using Simscape and Simulink.

## Overview

This mini-project focuses on the physical modeling and closed-loop control of a DC motor using Simscape.

Instead of representing the plant only through mathematical transfer functions, the electrical and mechanical behavior of the motor is modeled using physical Simscape components.

A PI controller is then integrated into the physical model to control the motor response and reject external disturbances.

## Problem and Model

The DC motor model combines electrical and mechanical dynamics.

The physical model includes:

- Electrical motor components
- Mechanical rotational components
- Motor parameters
- Sensors for feedback
- Physical connections between the electrical and mechanical domains

The model was tested in open loop to verify that the physical system behaves correctly before introducing the controller.

## Control System

A closed-loop feedback structure was implemented using:

- Motor output measurement
- Sensor feedback
- Error calculation
- PI controller
- Motor input source

The PI controller was tuned with:

- `Kp = 0.9`
- `Ki = 8`

The controller provides a fast response with low overshoot and eliminates the steady-state error.

## Disturbance Rejection

An external disturbance was introduced to evaluate the controller's ability to maintain the desired motor response.

The PI controller successfully compensates for the disturbance and brings the system back toward the desired operating condition.

## Robustness Analysis

The robustness of the controlled system was evaluated by varying key physical parameters, including:

- Electrical resistance `R`
- Mechanical inertia `J`
- Load torque

The system remained stable under the tested parameter variations, demonstrating that the controller maintains satisfactory performance despite changes in the physical model.

## Key Results

- A physical DC motor model was built using Simscape.
- Electrical and mechanical domains were integrated into a single physical model.
- The model was validated in open loop before controller implementation.
- A closed-loop PI controller was successfully implemented.
- The PI controller achieved zero steady-state error with a fast response and low overshoot.
- External disturbances were successfully rejected.
- Robustness was verified under variations of resistance, inertia and load.

## Tools & Technologies

- MATLAB
- Simulink
- Simscape
- Physical system modeling
- Electrical and mechanical system modeling
- PI control
- Feedback control
- Disturbance rejection
- Robustness analysis

## Files

- `*.slx` — Simscape/Simulink physical model and control system
