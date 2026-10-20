/*
 * electronic_differential.h - Diferencial Eletrônico e Vetorização de Torque
 * Cinemática de Ackermann com torque vectoring PD e filtro derivativo IIR.
 */

#ifndef ELECTRONIC_DIFFERENTIAL_H
#define ELECTRONIC_DIFFERENTIAL_H

#include "vehicle_control.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ElectronicDiffConfig_s {
    float kp_yaw;           /* Ganho proporcional de guinada [N.m / (rad/s)] */
    float kd_yaw;           /* Ganho derivativo de amortecimento de guinada [N.m / (rad/s^2)] */
    float understeer_coeff; /* Gradiente de subesterço K_us [rad/(m/s^2)] */
    float max_delta_torque; /* Limite de transferência diferencial [N.m] */
    float prev_yaw_rate;    /* Última taxa de guinada medida pela IMU [rad/s] */
    float d_term_filtered;  /* Estado do filtro passa-baixas derivativo [N.m] */
    float tau_derivative_s; /* Constante de tempo do filtro derivativo [s] */
    bool  first_run;        /* Flag de inicialização do filtro derivativo */
} ElectronicDiffConfig_t;

void ElectronicDiff_Init(ElectronicDiffConfig_t *config);

void ElectronicDiff_Update(ElectronicDiffConfig_t *config,
                           const DriverInputs_t *driver,
                           const SensorData_t *sensors,
                           MotorCommands_t *commands,
                           float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* ELECTRONIC_DIFFERENTIAL_H */
