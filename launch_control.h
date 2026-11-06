/*
 * launch_control.h - Controle de largada do kart eletrico
 * FSM de 6 estados, estimativa de carga traseira e malha PI de escorregamento
 */

#ifndef LAUNCH_CONTROL_H
#define LAUNCH_CONTROL_H

#include "vehicle_control.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LC_MAX_RAMP_RATE_NM_PER_S (120.0f)
#define LC_PRELOAD_TORQUE_NM (8.0f)
#define LC_COMPLETION_SPEED_MS (14.0f)
#define LC_MAX_STEERING_RAD (5.0f * DEG_TO_RAD)
#define LC_MAX_STAGING_TIME_S (3.0f)
#define LC_AX_FILTER_TAU_S (0.03f)
#define LC_STAGING_PEDAL_THRESHOLD (0.70f)
#define LC_STAGING_ABORT_PEDAL (0.30f)
#define LC_ARMED_BRAKE_THRESHOLD_BAR (15.0f)
#define LC_BRAKE_DISARM_THRESHOLD_BAR (5.0f)
#define LC_MAX_CHASSIS_SPEED_ARM_MS (0.5f)
#define LC_TARGET_SLIP_DEFAULT (0.12f)
#define LC_KP_SLIP_DEFAULT (65.0f)
#define LC_KI_SLIP_DEFAULT (55.0f)
#define LC_TIMEOUT_LAUNCH_S (4.0f)
#define LC_COMPLETION_BLEND_TIME_S (0.50f)
#define LC_INTEGRAL_SLIP_MAX (0.45f)
#define LC_PRELOAD_RPM (500.0f)
#define LC_YAW_DAMPING_GAIN (8.0f)

typedef enum {
    LC_STATE_DISARMED = 0,
    LC_STATE_ARMED,
    LC_STATE_STAGING,
    LC_STATE_LAUNCHING,
    LC_STATE_COMPLETED,
    LC_STATE_ABORTED
} LaunchControlState_t;

typedef struct LaunchControlContext_s {
    LaunchControlState_t state;
    float target_slip;
    float kp_slip;
    float ki_slip;
    float integral_slip_error;
    float current_torque_output;
    float launch_timer_s;
    float staging_timer_s;
    float ax_filtered_mss;
    float transition_blend;
} LaunchControlContext_t;

void LaunchControl_Init(LaunchControlContext_t *ctx);

bool LaunchControl_Update(LaunchControlContext_t *ctx,
                          const DriverInputs_t *driver,
                          const SensorData_t *sensors,
                          const VehicleSafetyContext_t *safety_ctx,
                          MotorCommands_t *commands,
                          float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* LAUNCH_CONTROL_H */
