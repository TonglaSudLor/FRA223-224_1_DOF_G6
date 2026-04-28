/**
 * @file params.h
 * @brief Global adjustable parameters for the PID Motor Controller.
 */

#ifndef PARAMS_H
#define PARAMS_H

/* ============================================================================
 * PID GAINS (Tuning)
 * ============================================================================ */

/* Speed Loop (Inner) */
#define DEFAULT_SPEED_KP    1.0f
#define DEFAULT_SPEED_KI    2.0f
#define DEFAULT_SPEED_KD    0.0f
#define DEFAULT_SPEED_KF    1.302f  /**< Your characterized Feed-Forward gain */

/* Position Loop (Outer) */
#define DEFAULT_POS_KP      0.6f
#define DEFAULT_POS_KI      0.1f
#define DEFAULT_POS_KD      0.1f

/* ============================================================================
 * MOTION & JOG SPEEDS
 * ============================================================================ */

#define JOG_SPEED_FINE      20.0f
#define MOVE_SPEED_COARSE   100.0f  /**< As requested from image */

#define STEP_SIZE_COARSE    10.0f
#define STEP_SIZE_FINE      1.0f

/* ============================================================================
 * HARDWARE LIMITS
 * ============================================================================ */

#define MAX_VOLTAGE_LIMIT   24.0f
#define SUPPLY_VOLTAGE      24.0f

#define PID_INTEGRAL_MAX    50.0f
#define POS_INTEGRAL_MAX    200.0f

#define DEFAULT_MIN_PWM     3.0f
#define DEFAULT_MAX_ACCEL   200.0f  /**< Lowered for smoother stopping */

/* ============================================================================
 * SYSTEM TIMING
 * ============================================================================ */

#define HOME_HOLD_TIME_MS   3000

#endif /* PARAMS_H */
