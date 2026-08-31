# 3-Phase Motor Controller

DQ-domain PI control for 3-phase BLDC motor drive, validated against
PLECS simulation.

**Skills: FOC/DQ control · PI compensator design · embedded firmware ·
PCB integration · simulation-to-hardware validation**

## Overview
This is a continuation of the 24-48V boost converter powertrain project. This stage is focused on the control loops for both the boost converter and the 3 phase motor. While the same PCB could have been used, for the purposes of the class a standardized board was assembled from an Altium file reference.

The previous stage, the custom designed synchronous 24V-48V boost converter, is linked here: [Synchronous Boost Converter for Ebike](https://github.com/alexcjang/24V-48V-Synchronous-Boost-Converter-for-Ebike#24v-48v-synchronous-boost-converter-for-ebike)

## Key Values / Deliverables
- Control architecture: DQ-domain PI control for 3-phase current →
  electrical torque
- Boost converter current control: PI controller, 24V→48V
- Validated against PLECS simulation across: back-EMF, 3-phase
  currents, hall sensor outputs, torque
- [Add: switching frequency, current/torque ranges tested, any
  efficiency or response-time numbers if you have them]

## Measurements
[Scope captures or PLECS-vs-experimental comparison plots for:
back-EMF waveform, 3-phase currents, hall sensor outputs, torque —
this is your strongest evidence section, since the sim-to-hardware
validation *is* the headline result of this project. A side-by-side
plot (simulated vs. measured) for even one of these would be very
strong if you have the data.]

## Key Firmware Snippet: DQ-Domain PI Control
```c
// 15–30 lines: e.g., the Park/Clarke transform or the PI current
// control loop
```
[Short explanation: what this block does, why PI (vs. other control
approaches), how gains were tuned]

## What I'd Change
[One honest paragraph — e.g., what you'd instrument better, what
edge cases you didn't test, what you'd optimize with more time
(response time, torque ripple, sensorless operation, etc.)]

## Tools
PLECS · Altium · C (TI C2000) · MATLAB · Oscilloscope
