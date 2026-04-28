# PID Motor Controller - Technical User Manual

## 1. Overview
This system is a high-precision PID motor controller implemented on an STM32G474. It supports dual-loop control (Speed & Position), S-Curve trajectory generation, and advanced tuning features like Feed-Forward Characterization and Ghost Target selection.

---

## 2. Hardware Interface

### 2.1 Communication Protocol
The system uses a **3-character string** protocol via UART:
*   **Format**: `[Command][Emergency_Status][Connection_Status]`
*   **Example**: `LOC` (Rotate Left, Emergency Off, Connected).

---

## 3. Control Logic & Modes

### 3.1 Motor States (`current_mode`)
1.  **STOPPED (0)**: Power cut. Motor is loose.
2.  **SPEED (1)**: Constant velocity rotation (Jogging).
3.  **POSITION (2)**: Closed-loop position holding/tracking.
4.  **AUTOTUNE (3/4)**: Relay-based PID auto-tuning.
5.  **TEST (5)**: Open-loop Feed-Forward characterization (runs 1.0s at 25% PWM).
6.  **GHOST (6)**: Selection mode for S-Curve testing. Motor holds position while target is moved.

---

## 4. Remote Button Mapping

| Button | Action | Behavior Details |
| :---: | :--- | :--- |
| **L** | **Move Positive** | Increments Target Pos or Velocity. |
| **R** | **Move Negative** | Decrements Target Pos or Velocity. |
| **O** | **Release** | Stops movement and locks current position. |
| **P** | **Emergency Stop** | Toggles PID on/off. Forces PWM 0. |
| **A** | **Home Key** | **Click**: Set current pos as 0.0 (Clean reset, no spikes). <br> **3s hold**: Smooth return to 0.0. |
| **M** | **Mode / Test** | **Click**: Toggle COARSE/FINE. <br> **3s hold**: Start 1.0s Feed-Forward Test. |
| **Y** | **Ghost / Execute**| **1s hold**: Toggle Ghost Mode. <br> **Click (in Ghost)**: Execute S-Curve move to target. |

---

## 5. Advanced Features

### 5.1 Feed-Forward Characterization
To eliminate lag and jitter, use the characterization tool:
1. Run `characterize_motor.m` in MATLAB.
2. **Hold M for 3 seconds**. The motor spins for 1s.
3. MATLAB will output a **Suggested Kf**.
4. Enter this value into `tuning.speed_Kf` in Live Expressions.

### 5.2 Ghost Target Selection
To test an S-Curve move before executing:
1. **Hold Y for 1 second** to enter Ghost Mode.
2. Use **L / R** to move the **Magenta Dotted Line** in MATLAB to your goal.
3. **Quick Press Y** to start the move.
4. The motor will follow the **White Dashed Line** (S-Curve) to the goal.

---

## 6. Troubleshooting
1.  **Velocity Spike at Home**: Fixed. Resetting home (Button A) now clears PID integrals.
2.  **Hard Stops**: If the motor stops too aggressively, lower `max_accel` (e.g., to 100-150) or increase `pos_Kd`.
3.  **Inverted Controls**: L/R are now mapped to Positive/Negative direction. To flip, adjust `PWM_Apply` logic.

---
*Updated on April 27, 2026*
