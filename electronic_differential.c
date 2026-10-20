/*
 * electronic_differential.c - Diferencial Eletrônico e Vetorização de Torque
 * Distribuição de rotação por Ackermann e correção de guinada por torque vectoring.
 */

#include "electronic_differential.h"

void ElectronicDiff_Init(ElectronicDiffConfig_t *config) {
    if (!config) return;

    config->kp_yaw           = 18.5f;
    config->kd_yaw           = 1.2f;
    config->understeer_coeff = 0.0025f;
    config->max_delta_torque = 12.0f;
    config->prev_yaw_rate    = 0.0f;
    config->d_term_filtered  = 0.0f;
    config->tau_derivative_s = 0.020f;
    config->first_run        = true;
}

void ElectronicDiff_Update(ElectronicDiffConfig_t *config,
                           const DriverInputs_t *driver,
                           const SensorData_t *sensors,
                           MotorCommands_t *commands,
                           float dt_s) {
    if (!config || !driver || !sensors || !commands) return;

    if (!isfinite(dt_s) || dt_s <= 0.0f || !driver->safety_plausibility_ok) {
        config->d_term_filtered         = 0.0f;
        config->first_run               = true;
        commands->torque_left_nm        = 0.0f;
        commands->torque_right_nm       = 0.0f;
        commands->target_rpm_left       = 0.0f;
        commands->target_rpm_right      = 0.0f;
        commands->inverter_left_enable  = false;
        commands->inverter_right_enable = false;
        commands->regen_active          = false;
        return;
    }

    commands->inverter_left_enable  = true;
    commands->inverter_right_enable = true;

    float base_throttle = clampf(driver->throttle_demand, 0.0f, 1.0f);
    float t_base = base_throttle * MOTOR_MAX_TORQUE_NM;

    /* Sincronização do ângulo de esterçamento */
    float delta_rad = driver->steering_angle_rad;
    if (fabsf(driver->steering_angle_deg * DEG_TO_RAD - delta_rad) > 1e-3f &&
        fabsf(driver->steering_angle_deg) > 1e-4f && fabsf(delta_rad) < 1e-6f) {
        delta_rad = driver->steering_angle_deg * DEG_TO_RAD;
    }
    delta_rad = clampf(delta_rad, -MAX_STEERING_ANGLE_RAD, MAX_STEERING_ANGLE_RAD);
    float tan_delta = tanf(delta_rad);

    float v_x = isfinite(sensors->vehicle_speed_ms) ? fmaxf(0.0f, sensors->vehicle_speed_ms) : 0.0f;

    /* Velocidades angulares alvo nas rodas traseiras por Ackermann */
    float omega_c = v_x / TIRE_RADIUS_DYNAMIC_M;
    const float geom_factor = REAR_TRACK_M / (2.0f * WHEELBASE_M);
    float ackerman_factor = geom_factor * tan_delta;

    float omega_left  = fmaxf(0.0f, omega_c * (1.0f + ackerman_factor));
    float omega_right = fmaxf(0.0f, omega_c * (1.0f - ackerman_factor));

    float rpm_left  = (omega_left * GEAR_RATIO) * RADS_TO_RPM;
    float rpm_right = (omega_right * GEAR_RATIO) * RADS_TO_RPM;

    commands->target_rpm_left  = clampf(rpm_left, 0.0f, MOTOR_MAX_RPM);
    commands->target_rpm_right = clampf(rpm_right, 0.0f, MOTOR_MAX_RPM);

    /* Taxa de guinada de referência pelo modelo linear de bicicleta */
    float denom = WHEELBASE_M + ((config->understeer_coeff * v_x * v_x) / GRAVITY_MSS);
    if (denom < 0.1f) denom = 0.1f;
    float yaw_rate_des = (v_x / denom) * tan_delta;

    float yaw_error = yaw_rate_des - sensors->yaw_rate_rads;

    /* Derivada aplicada sobre a medição da IMU (-d(yaw_rate)/dt) para eliminar derivative kick */
    if (config->first_run) {
        config->prev_yaw_rate   = sensors->yaw_rate_rads;
        config->d_term_filtered = 0.0f;
        config->first_run       = false;
    } else {
        float tau = (config->tau_derivative_s > 0.001f) ? config->tau_derivative_s : 0.020f;
        float alpha = tau / (tau + dt_s);
        float d_yaw_meas = -(sensors->yaw_rate_rads - config->prev_yaw_rate) / dt_s;
        config->prev_yaw_rate   = sensors->yaw_rate_rads;
        config->d_term_filtered = (alpha * config->d_term_filtered) +
                                  ((1.0f - alpha) * config->kd_yaw * d_yaw_meas);
    }

    float delta_torque = (config->kp_yaw * yaw_error) + config->d_term_filtered;
    delta_torque = clampf(delta_torque, -config->max_delta_torque, config->max_delta_torque);

    if (v_x < 1.0f || base_throttle < 0.05f) {
        delta_torque = 0.0f;
        if (v_x < 0.5f) {
            config->d_term_filtered = 0.0f;
        }
    }

    float t_left  = t_base + delta_torque;
    float t_right = t_base - delta_torque;
    const float t_limit = MOTOR_MAX_TORQUE_NM;

    /* Preserva o momento diferencial 2*delta_torque sob saturação superior ou inferior */
    if (t_left >= t_right && t_left > t_limit) {
        float excess = t_left - t_limit;
        t_left  = t_limit;
        t_right -= excess;
    } else if (t_right > t_left && t_right > t_limit) {
        float excess = t_right - t_limit;
        t_right = t_limit;
        t_left  -= excess;
    }

    if (t_left <= t_right && t_left < 0.0f) {
        float deficit = -t_left;
        t_left  = 0.0f;
        t_right = clampf(t_right + deficit, 0.0f, t_limit);
    } else if (t_right < t_left && t_right < 0.0f) {
        float deficit = -t_right;
        t_right = 0.0f;
        t_left  = clampf(t_left + deficit, 0.0f, t_limit);
    }

    commands->torque_left_nm  = clampf(t_left, 0.0f, t_limit);
    commands->torque_right_nm = clampf(t_right, 0.0f, t_limit);
    commands->regen_active    = false;
}
