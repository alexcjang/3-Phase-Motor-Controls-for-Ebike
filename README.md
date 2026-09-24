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
- Successfully controlled a bike motor to spin wheel at constant velocity and respond to changes in throttle input


## System Diagram
<img width="800" height="500" alt="image" src="https://github.com/user-attachments/assets/68cfd3d7-34e3-47b6-80d9-e4441b41ba60" />

## Measurements

Simulation Results
<table>
  <tr>
    <!-- Left Column: Schematic -->
    <td valign="top" width="50%">
      <img src="https://github.com/user-attachments/assets/080b1a91-c163-4870-a0a9-6a96fd52d2ac" alt="PLECS Simulation Setup" width="100%" />
      <p align="center"><b>PLECS Simulation Setup</b></p>
    </td>
    <!-- Right Column: Stacked Graphs with Captions -->
    <td valign="top" width="50%">
      <!-- Top Image + Caption -->
      <img src="https://github.com/user-attachments/assets/924694e7-6165-49c1-a4d8-c7fe531e5469" alt="Output Graph 1" width="100%" />
      <p align="center"><b>Graph 1:</b> Simulated Hall Sensor Outputs Per Phase</p>
      <!-- Bottom Image + Caption -->
      <img src="https://github.com/user-attachments/assets/ca5af2a4-873a-40c8-9cbc-5d31fcbe5fca" alt="Output Graph 2" width="100%" />
      <p align="center"><b>Graph 2:</b> Simulated 3 Phase Currents</p>
      <img width="1919" height="841" alt="image" src="https://github.com/user-attachments/assets/77ccaf95-773b-4d46-a0b7-008fe86f6810" />
      <p align="center"><b>Graph 3:</b> Simulated iq Current Tracking</p>
    </td>
  </tr>
</table>


Experimental Validation
<table>
  <tr>
    <!-- Left Column Image & Caption -->
    <td valign="top" width="50%">
      <img src="https://github.com/user-attachments/assets/f5d43d76-aa65-4ebe-906a-ab29f2e4683c" alt="Image 1" width="100%" />
      <p align="center"><b>Figure 1:</b> Back EMF In Phase With ADC measurements</p>
    </td>
    <!-- Right Column Image & Caption -->
    <td valign="top" width="50%">
      <img src="https://github.com/user-attachments/assets/372bd909-75e2-4951-97a7-367c343ca230" alt="Hall effect output in phase" width="100%" />
      <p align="center"><b>Figure 2:</b> Hall Effect Sensors In Phase With Flux Linkage</p>
    </td>
  </tr>
</table>

The scope captures show that we successfully calibrated our ADC measurements and Hall Effect Sensors to match our simulation. We validated the iq current control when we were able to spin the motor at constant velocity and have it respond to a changing throttle input.

We also calculated different motor parameters experimentally to validate our simulation, such as number of poles, winding DC resistance, winding lower frequency inductance, and flux linkage. Details are in the reports.


## Key Firmware Snippet: DQ-Domain PI Control
```c
// --- abc -> dq (Park Transform) ---
id = (2.0/3.0) * (iLA * cos(theta_e) + iLB * cos(theta_e - OFFSET)
+ iLC * cos(theta_e + OFFSET));
iq = (2.0/3.0) * (-iLA * sin(theta_e) - iLB * sin(theta_e - OFFSET)
- iLC * sin(theta_e + OFFSET));

// --- Current error ---
id_err = id_ref - id; iq_err = iq_ref - iq;
// --- PI controller (trapezoidal integration) ---
d_int = d_int_prev + Ki * (Ts/2.0) * (id_err + id_err_prev);
q_int = q_int_prev + Ki * (Ts/2.0) * (iq_err + iq_err_prev);

// --- dq voltage with decoupling ---
vd = d_int + Kp * id_err + omega_e * Ls * iq;
vq = q_int + Kp * iq_err - omega_e * Ls * id;

// --- Normalize to modulation indices ---
md = vd * (2.0 / Vdc);
mq = vq * (2.0 / Vdc);

// --- dq -> abc (Inverse Park) ---
m_a = md * cos(theta_e) - mq * sin(theta_e);
m_b = md * cos(theta_e - OFFSET) - mq * sin(theta_e - OFFSET);
m_c = md * cos(theta_e + OFFSET) - mq * sin(theta_e + OFFSET);
```
This snippet shows the conversion from 3 phase to dq domain for PI control of iq. In dq domain, we use Pole-Zero cancellation to find the Kp and Ki for a PI controller. Controlling our iq allows us to control the electrical torque of the system.

## What I'd Change
With more time, I would have also implemented outer loop voltage control on our boost converter. This would keep our system safe from transients that come from a spike in current or increase in load. I would also want to refine the closed loop boost gains for better performance at even larger current step amplitudes.

## Media
<img width="509" height="491" alt="image" src="https://github.com/user-attachments/assets/59ae3513-c72d-464e-aa0e-4f3a0a881c39" />

Assembled PCB

## Tools
PLECS · Altium · C (TI C2000) · MATLAB · Oscilloscope
