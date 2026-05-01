# 1-DOF Robot PID Motor Controller (S-Curve Trajectory)

This project implements a high-precision 1-DOF robot control system using an **STM32G474** for motor control and an **ESP32** for Bluetooth joystick communication. The system features dual-loop PID control (Speed & Position) with S-curve trajectory profiling for smooth movement.

## 🚀 Features
- **Dual-Loop PID Control**: Cascaded speed and position loops for precise tracking.
- **S-Curve Trajectory**: Smooth acceleration/deceleration to minimize vibration and mechanical stress.
- **Enhanced Safety Suite**:
  - **Stall Detection**: Detects physical blocks and cuts power.
  - **Encoder Monitoring**: Detects signal loss or phase inversion.
  - **Over-Rotation Protect**: Software limit (±720°) to prevent wire snap.
  - **Connection Guard**: Auto-halts if Bluetooth joystick disconnects.
- **Dual Control Stages**: Toggle between standard Joystick Control and Base System Mode.
- **Advanced Tuning**:
  - Relay-based Autotune (Position & Speed).
  - Open-loop Feed-Forward (Kf) characterization.
- **Ghost Mode**: Preview target movement in MATLAB before execution.
- **Joystick Interface**: Bluetooth gamepad support via ESP32 (Bluepad32).
- **MATLAB Integration**: Real-time data streaming (50Hz) for analysis and plotting.

---

## 🛠 Hardware Setup

### 1. Pin Configuration (STM32G474RETx)

| Peripheral | Pin(s) | Function |
| :--- | :--- | :--- |
| **TIM3** | PA6, PA7 | Incremental Encoder (2048 PPR) |
| **TIM8** | PC7 (CH2) | Motor PWM Output |
| **GPIO** | PC6 | Motor Direction Control |
| **USART3** | PB10, PB11 | ESP32 Communication (115200 bps) |
| **LPUART1**| PA2, PA3 | PC / MATLAB Communication (921600 bps) |
| **TIM6** | Internal | 100Hz Control Loop Interrupt (10ms) |
| **LED** | PA5 | Status Indicator (LD2) |

### 2. Connection Diagram (ESP32 <-> STM32)
- **ESP32 (TX17)** -> **STM32 (PB11 - USART3_RX)**
- **ESP32 (RX16)** -> **STM32 (PB10 - USART3_TX)**
- **Common GND**

---

## 📂 Project Structure

```text
├── PID+Controller/          # Main Project Folder
│   ├── Core/                # STM32 Firmware
│   │   ├── Inc/
│   │   │   ├── motor_pid_controller.h  # Core Logic Headers
│   │   │   └── params.h                # Tuning & Hardware Constants
│   │   └── Src/
│   │       ├── main.c                  # UART Logic & Initialization
│   │       └── motor_pid_controller.c  # PID & S-Curve Implementation
│   ├── esp32/               # ESP32 Source Code (Arduino)
│   │   └── esp32.ino        # Bluepad32 Joystick Interface
│   ├── PID.ioc              # STM32CubeMX Project File
│   ├── USER_MANUAL.md       # Technical manual and operation guide
│   ├── characterize_motor.m # Feed-Forward (Kf) characterization tool
│   ├── ghost_mode_plotter.m # High-speed PID tuning & performance analysis
│   └── plot_motor_data.m    # Real-time multi-axis telemetry visualizer
└── System_Iden/             # Saved System Identification Data (.mat)
```

---

## 📊 MATLAB Integration

The system supports high-speed telemetry for tuning and validation. **Note:** Ensure the COM port in each script matches your ST-Link/UART adapter.

### 1. Real-time Telemetry (`plot_motor_data.m`)
- **Baudrate**: 115200
- **Features**: Visualizes Actual vs. Reference Position, Velocity, and Acceleration. Includes a magenta line for the "Ghost" target.
- **Usage**: Run while the motor is operating to monitor tracking performance.

### 2. Feed-Forward Characterization (`characterize_motor.m`)
- **Baudrate**: 115200
- **Features**: Automates the calculation of the **Kf** (Feed-Forward) gain.
- **Workflow**:
    1. Run script in MATLAB.
    2. Hold **Button M** for 3s (STM32 runs 1s @ 25% PWM).
    3. Copy the "Suggested Kf" from the command window to `tuning.speed_Kf` in Live Expressions.

### 3. High-Speed Performance Analysis (`ghost_mode_plotter.m`)
- **Baudrate**: 921600 (Uses LPUART1 for maximum bandwidth)
- **Features**: Captures high-resolution data during S-Curve moves. 
- **Analytics**: Automatically calculates **Settling Time** and **Overshoot (%)** for each run, allowing for iterative PID tuning.
- **Workflow**: Use in **Ghost Mode** to evaluate your S-Curve profile before and after execution.

---

## 🎮 Operation & Controls

### Joystick Mapping (Bluetooth Gamepad)
| Button | STM32 Command | Action |
| :---: | :---: | :--- |
| **L / R** | `L` / `R` | Move Positive/Negative (Degrees) |
| **U / D** | `U` / `D` | Manual Velocity Jog (RPM) |
| **O (Release)** | `O` | Release movement (Locks pos in Coarse, Coast in Fine) |
| **X (Emergency)**| `P` | **Emergency Stop / Reset** (Clears Faults) |
| **A (Home)** | `A` | **Press**: Set Zero. **Hold 3s**: Return to Home. |
| **M (Mode)** | `M` | **Press**: Fine/Coarse toggle. **Hold 3s**: Start FF Test. |
| **Y (Select)** | `Y` | **Hold 1s**: Ghost Mode toggle. **Press**: Execute move. |
| **B (Control)** | `B` | **Hold 1s**: Toggle Joystick / Base System Mode. |

### 🔍 Live Expression Monitoring
For real-time debugging in STM32CubeIDE, add these variables to your **Live Expressions** tab to monitor the system's state:

| Expression | Description |
| :--- | :--- |
| `encoder.filtered_rpm` | Current rotational speed (Filtered RPM) |
| `encoder.current_position_deg` | Actual robot arm position (Degrees) |
| `trajectory.target_pos` | Internal S-Curve setpoint position |
| `buffered_target_pos` | The "Ghost" target position |
| `target_position_deg` | Final commanded target position |
| `jog_mode` | `JOG_COARSE` or `JOG_FINE` status |
| `tuning` | PID gains and motion limits structure |
| `rx_buffer` | Raw UART data from ESP32 |
| `autotune_trigger` | Trigger for Relay Autotuning |
| `tuning_progress` | Progress of the active autotune cycle |
| `autotune_status` | Status of autotuning (Idle, Running, Success) |
| `current_mode` | Operational state (SPEED, POSITION, GHOST, etc.) |
| `Emergency_stop` | Global safety lock status |
| `is_joystick_connected` | Bluetooth connection status |
| `fault_code` | Diagnostic status (Stall, Encoder, Connection) |
| `control_system_mode` | `CONTROL_MODE_JOYSTICK` or `CONTROL_MODE_BASE_SYSTEM` |
| `safety_enabled` | Toggle (1/0) to enable/disable auto-emergency |

---

## 📈 Tuning & Analysis
1. **Feed-Forward**: Run `characterize_motor.m`, then hold **M** for 3s. Update `tuning.speed_Kf` with the result.
2. **Ghost Mode**: Hold **Y** for 1s. Use L/R to set target on MATLAB plotter. Click **Y** to execute the S-Curve trajectory.
3. **Data Logging**: Use `plot_motor_pid.m` to capture and visualize transient response for fine-tuning.

---
*Created for FRA223-224 - 1 DOF Robot Project (Group 6)*
