#ifndef MOTOR_PID_CONTROLLER_H
#define MOTOR_PID_CONTROLLER_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* --- Hardware Configuration --- */
#define MOTOR_ENCODER_PPR           2048    /**< จำนวนพัลส์ต่อรอบของ Encoder */
#define MOTOR_GEAR_RATIO            1.0f    /**< อัตราทด (คุณเทสแล้วว่า 1.0 ตรงที่สุด) */
#define MOTOR_CONTROL_FREQ_HZ       100     /**< ความถี่ลูปควบคุม (Hz) */

/* --- Control Structures --- */
typedef enum {
    MOTOR_MODE_STOPPED = 0,
    MOTOR_MODE_SPEED,
    MOTOR_MODE_POSITION,
    MOTOR_MODE_AUTOTUNE,
    MOTOR_MODE_AUTOTUNE_SPEED,
    MOTOR_MODE_TEST,
    MOTOR_MODE_GHOST
} Motor_Control_Mode_t;

typedef enum {
    CONTROL_MODE_BASE_SYSTEM = 0,
    CONTROL_MODE_JOYSTICK = 1
} Control_System_Mode_t;

typedef enum {
    FAULT_NONE = 0x00,
    FAULT_MOTOR_STALLED = 0x01,
    FAULT_ENCODER_ERROR = 0x02,
    FAULT_JOYSTICK_LOST = 0x04
} Fault_Code_t;

typedef enum {
    JOG_COARSE = 0,
    JOG_FINE = 1
} Jog_Mode_t;

typedef enum {
    ATUNE_IDLE = 0,
    ATUNE_POS,    /**< Set to 1 in Live Expression to start Position Autotune */
    ATUNE_SPEED   /**< Set to 2 in Live Expression to start Speed Autotune */
} Autotune_Trigger_t;

typedef enum {
    STATUS_IDLE = 0,
    STATUS_RUNNING_POS,
    STATUS_RUNNING_SPEED,
    STATUS_SUCCESS,
    STATUS_ERROR_LIMIT_EXCEEDED
} Autotune_Status_t;

typedef struct {
    // PID Speed Loop
    float speed_Kp;
    float speed_Ki;
    float speed_Kd;
    float speed_Kf;            // Feed-Forward Gain (PWM per RPM)
    
    // PID Position Loop
    float pos_Kp;
    float pos_Ki;
    float pos_Kd;
    
    // Jog & Step Settings
    float jog_speed_fine;      // RPM for continuous jog
    float move_speed_coarse;   // RPM for step movement
    float step_size_coarse;    // Degrees per click in Coarse mode
    float step_size_fine;      // Step size if ever used in fine mode
    float min_pwm;             // Minimum PWM to overcome friction
    float max_accel;           // Acceleration for position steps
} Tuning_Params_t;

typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float integral_max;
    float error_prev;
    float d_filt;            /**< Low-pass filter state for derivative */
    float output_min, output_max;
} PID_Controller_t;

typedef struct {
    int32_t absolute_counts;
    uint32_t count_prev;
    float current_position_deg;
    float filtered_rpm;
} Encoder_Data_t;

typedef struct {
    float max_velocity;
    float max_acceleration;
    float jerk_smoothing;
} Trajectory_Config_t;

typedef struct {
    float target_pos;
    float target_vel;
    float current_setpoint_pos;
    float current_setpoint_vel;
} Trajectory_State_t;

/* --- Public Variables (For Live Expressions) --- */
extern volatile Motor_Control_Mode_t current_mode;
extern volatile Control_System_Mode_t control_system_mode;
extern volatile Jog_Mode_t jog_mode;
extern volatile Autotune_Trigger_t autotune_trigger;
extern volatile Autotune_Status_t autotune_status;
extern volatile int tuning_progress;
extern volatile Tuning_Params_t tuning;
extern volatile bool is_joystick_connected;
extern volatile bool Emergency_stop;
extern volatile bool safety_enabled;
extern volatile Fault_Code_t fault_code;
extern volatile float target_position_deg;
extern volatile float buffered_target_pos;   /**< The "Ghost" target for S-curve testing */
extern volatile bool ghost_move_active;
extern volatile uint32_t ghost_settle_start_tick;

#define GHOST_BUFFER_MAX 1000
typedef struct {
    uint32_t tick;
    int32_t pos_x100;
    int32_t target_x100;
} Ghost_Buffer_t;

extern Ghost_Buffer_t ghost_buffer[GHOST_BUFFER_MAX];
extern volatile uint32_t ghost_buffer_idx;
extern volatile bool ghost_dump_requested;

extern PID_Controller_t pid_speed;
extern PID_Controller_t pid_position;
extern Encoder_Data_t encoder;
extern Trajectory_State_t trajectory;

/* --- Public Functions --- */
/**
 * @brief เริ่มต้นระบบควบคุมมอเตอร์
 */
void Motor_Init(void);

/**
 * @brief สั่งเคลื่อนที่ไปยังตำแหน่งองศาที่กำหนด
 * @param target_degrees องศาเป้าหมาย
 */
void Motor_MoveToPosition(float target_degrees);

/**
 * @brief ตั้งค่าขีดจำกัดแรงดันไฟฟ้า (PWM Limit)
 * @param max_voltage แรงดันสูงสุดที่อนุญาต (V)
 * @param supply_voltage แรงดันจากแหล่งจ่าย (V)
 */
void Motor_SetVoltageLimit(float max_voltage, float supply_voltage);

/**
 * @brief ตั้งค่าโปรไฟล์การเคลื่อนที่
 * @param max_rpm ความเร็วสูงสุด (RPM)
 * @param max_accel ความเร่งสูงสุด (RPM/s)
 * @param smoothing ค่าความนุ่มนวล S-Curve (0.0 - 0.99)
 */
void Motor_SetMotionProfile(float max_rpm, float max_accel, float smoothing);

/**
 * @brief ตั้งค่าความเร็วสำหรับการ Jog
 * @param rpm ความเร็ว (RPM)
 */
void Motor_SetJogVelocity(float rpm);

/**
 * @brief ประมวลผลคำสั่งจาก UART (เช่น 'L', 'R', 'U', 'D', 'H', 'M', 'E', 'J')
 * @param cmd ตัวอักษรคำสั่ง
 */
void Motor_ProcessCommand(char cmd);

/**
 * @brief อัปเดตสถานะปุ่ม Selection (Y) เพื่อตรวจจับการกดค้าง
 * @param pressed true ถ้าปุ่ม Y ถูกกด
 */
void Motor_UpdateSelectionButton(bool pressed);

/**
 * @brief อัปเดตสถานะปุ่ม Mode (M) เพื่อตรวจจับการกดค้าง
 * @param pressed true ถ้าปุ่ม M ถูกกด
 */
void Motor_UpdateModeButton(bool pressed);

/**
 * @brief อัปเดตสถานะปุ่ม Control Mode (B) เพื่อตรวจจับการกดค้าง
 * @param pressed true ถ้าปุ่ม B ถูกกด
 */
void Motor_UpdateControlModeButton(bool pressed);

/**
 * @brief อัปเดตสถานะการเชื่อมต่อของ Joystick
 * @param connected true: เชื่อมต่อแล้ว (C), false: หลุดการเชื่อมต่อ (N)
 */
void Motor_SetConnectionStatus(bool connected);

/**
 * @brief ส่งข้อมูล Position, Velocity, Acceleration ไปยัง MATLAB ผ่าน UART
 */
void Motor_SendDataToMatlab(void);

/**
 * @brief เริ่มต้นระบบจูน PID อัตโนมัติ (Relay Autotune - Position)
 */
void Motor_StartAutotune(void);

/**
 * @brief เริ่มต้นระบบจูน PID อัตโนมัติ (Relay Autotune - Speed)
 */
void Motor_StartAutotuneSpeed(void);

/**
 * @brief ฟังก์ชันลูปควบคุมหลัก (ต้องเรียกใน Timer Interrupt ทุกๆ 10ms)
 */
void TIM6_Control_Loop_ISR(void);

/**
 * @brief อ่านตำแหน่งปัจจุบัน (องศา)
 */
float Motor_GetPosition(void);

/**
 * @brief อ่านความเร็วปัจจุบัน (RPM)
 */
float Motor_GetSpeed(void);

#endif /* MOTOR_PID_CONTROLLER_H */
