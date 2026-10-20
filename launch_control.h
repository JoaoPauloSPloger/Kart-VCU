/*
 * launch_control.h - Controle de largada do kart elétrico
 * Máquina de estados, estimativa de carga traseira e malha PI de escorregamento.
 */

#ifndef LAUNCH_CONTROL_H
#define LAUNCH_CONTROL_H

#include "vehicle_control.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LC_MAX_STAGING_TIME_S       (3.0f)
#define LC_AX_FILTER_TAU_S          (0.03f)

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
    float target_slip;           /* Escorregamento alvo (ex: 0.12 para 12%) */
    float kp_slip;               /* Ganho proporcional da malha de slip */
    float ki_slip;               /* Ganho integral */
    float integral_slip_error;   /* Integrador de erro de slip */
    float current_torque_output; /* Comando de torque atual no motor [N.m] */
    float launch_timer_s;        /* Tempo decorrido na arrancada [s] */
    float staging_timer_s;       /* Tempo em pré-carga [s] */
    float ax_filtered_mss;       /* Aceleração longitudinal filtrada [m/s^2] */
    float transition_blend;      /* Fator de interpolação para saída (0 a 1) */
} LaunchControlContext_t;

void LaunchControl_Init(LaunchControlContext_t *ctx);

bool LaunchControl_Update(LaunchControlContext_t *ctx,
                          const DriverInputs_t *driver,
                          const SensorData_t *sensors,
                          MotorCommands_t *commands,
                          float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* LAUNCH_CONTROL_H */
