/*
 * electronic_differential.h - Diferencial Eletronico e Vetorizacao de Torque
 * Ackermann cinematico com TV proporcional-derivativo e filtro IIR de guinada
 */

#ifndef ELECTRONIC_DIFFERENTIAL_H
#define ELECTRONIC_DIFFERENTIAL_H

#include "vehicle_control.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ED_KP_YAW_DEFAULT (18.5f)
#define ED_KD_YAW_DEFAULT (1.2f)
#define ED_UNDERSTEER_COEFF_DEFAULT (0.0025f)
#define ED_MAX_DELTA_TORQUE_DEFAULT (12.0f)
#define ED_TAU_DERIVATIVE_S_DEFAULT (0.020f)
#define ED_MIN_SPEED_TV_MS_DEFAULT (1.0f)
#define ED_MIN_THROTTLE_TV_DEFAULT (0.05f)
#define ED_ACKERMANN_SLIP_MARGIN_DEFAULT (0.15f)

/* Parametros de calibracao em Flash/ROM */
typedef struct ElectronicDiffParams_s {
    float kp_yaw;
    float kd_yaw;
    float understeer_coeff;
    float max_delta_torque;
    float tau_derivative_s;
    float min_speed_tv_ms;
    float min_throttle_tv;
    float ackermann_slip_margin;
} ElectronicDiffParams_t;

/* Variaveis de estado dinamico alocadas em RAM */
typedef struct ElectronicDiffState_s {
    float prev_yaw_rate;
    float d_term_filtered;
    bool first_run;
} ElectronicDiffState_t;

void ElectronicDiff_InitParams(ElectronicDiffParams_t *params);
void ElectronicDiff_InitState(ElectronicDiffState_t *state);
void ElectronicDiff_Init(ElectronicDiffParams_t *params, ElectronicDiffState_t *state);

void ElectronicDiff_Update(const ElectronicDiffParams_t *params,
                           ElectronicDiffState_t *state,
                           const DriverInputs_t *driver,
                           const SensorData_t *sensors,
                           const VehicleSafetyContext_t *safety_ctx,
                           MotorCommands_t *commands,
                           float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* ELECTRONIC_DIFFERENTIAL_H */
