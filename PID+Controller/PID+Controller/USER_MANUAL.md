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

### 3.2 System Control Stages (`control_system_mode`)
*   **JOYSTICK (Default)**: Full control via Bluetooth remote.
*   **BASE SYSTEM**: Motion commands from the remote are **ignored**. This mode is for system-level automation or UART-only control. Emergency Stop and Mode Toggles remain active.

---

## 4. Remote Button Mapping

| Button | STM32 Command | Action | Behavior Details |
| :---: | :---: | :--- | :--- |
| **L / R** | `L` / `R` | **Jog / Step** | Moves robot +/-. Distance depends on Mode. |
| **O** | `O` | **Release** | **Coarse**: Lock Position. **Fine**: Coast to Stop. |
| **X** | `P` | **Emergency** | **Press**: Toggle Kill-Switch. **Reset**: Clears Faults. |
| **A** | `A` | **Home** | **Press**: Set Zero. **3s hold**: Return to 0.0. |
| **M** | `M` | **Mode** | **Press**: Coarse/Fine. **3s hold**: Start FF Test. |
| **Y** | `Y` | **Ghost** | **1s hold**: Toggle Ghost. **Press**: Execute move. |
| **B** | `B` | **System Stage**| **1s hold**: Toggle Joystick / Base System. |

---

## 5. Safety Protocols & Fault Handling

The system features an active watchdog that monitors for hardware and physical faults.

### 5.1 Fault Codes (`fault_code`)
If the system enters **Emergency Stop**, check this variable in Live Expressions:
*   **`FAULT_MOTOR_STALLED`**: Physical obstruction detected (High PWM, No Motion).
*   **`FAULT_ENCODER_ERROR`**: Encoder disconnected, signal loss, or phase inversion (A/B swapped).
*   **`FAULT_JOYSTICK_LOST`**: Bluetooth connection to ESP32 was interrupted.

### 5.2 Safety Override (`safety_enabled`)
For troubleshooting, you can disable automatic shutdowns:
1.  Set `safety_enabled` to `0` in Live Expressions.
2.  The system will still show faults in `fault_code` but will **not** cut power to the motor.
3.  **Use with caution!**

---

## 6. Advanced Features

### 6.1 Feed-Forward Characterization
1. Run `characterize_motor.m` in MATLAB.
2. **Hold M for 3 seconds**. The motor spins for 1s.
3. MATLAB will output a **Suggested Kf**. Enter into `tuning.speed_Kf`.

### 6.2 Ghost Mode Testing
1. **Hold Y for 1s** to enter Ghost Mode.
2. Use **L / R** to move the virtual target (Magenta) in MATLAB.
3. **Quick Press Y** to execute the S-Curve.

---

## 7. Troubleshooting
1.  **Violent Snap in Fine Mode**: Fixed. Fine mode now uses coasting (0 PWM) instead of PID-lock on release.
2.  **Encoder Fault on Start**: Check wires. If `safety_enabled` is 1, it will trip immediately if wires are inverted.
3.  **Base System Not Moving**: Check `control_system_mode`. If set to `BASE_SYSTEM`, it will ignore your remote.

---
*Updated on April 27, 2026*
