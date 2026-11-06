/*
 * electronic_differential.c - Diferencial Eletronico e Vetorizacao de Torque
 * Distribuicao de rotacao por Ackermann e correcao de guinada em malha fechada
 */

#include "electronic_differential.h"

void ElectronicDiff_InitParams(ElectronicDiffParams_t *params) {
    if (!params) {
        return;
    }
    params->kp_yaw = ED_KP_YAW_DEFAULT;
    params->kd_yaw = ED_KD_YAW_DEFAULT;
    params->understeer_coeff = ED_UNDERSTEER_COEFF_DEFAULT;
    params->max_delta_torque = ED_MAX_DELTA_TORQUE_DEFAULT;
    params->tau_derivative_s = ED_TAU_DERIVATIVE_S_DEFAULT;
    params->min_speed_tv_ms = ED_MIN_SPEED_TV_MS_DEFAULT;
    params->min_throttle_tv = ED_MIN_THROTTLE_TV_DEFAULT;
    params->ackermann_slip_margin = ED_ACKERMANN_SLIP_MARGIN_DEFAULT;
}

void ElectronicDiff_InitState(ElectronicDiffState_t *state) {
    if (!state) {
        return;
    }
    state->prev_yaw_rate = 0.0f;
    state->d_term_filtered = 0.0f;
    state->first_run = true;
}

void ElectronicDiff_Init(ElectronicDiffParams_t *params, ElectronicDiffState_t *state) {
    ElectronicDiff_InitParams(params);
    ElectronicDiff_InitState(state);
}

void ElectronicDiff_Update(const ElectronicDiffParams_t *params,
                           ElectronicDiffState_t *state,
                           const DriverInputs_t *driver,
                           const SensorData_t *sensors,
                           const VehicleSafetyContext_t *safety_ctx,
                           MotorCommands_t *commands,
                           float dt_s) {
    if (!params || !state || !driver || !sensors || !safety_ctx || !commands) {
        return;
    }

    if (!isfinite(dt_s) || dt_s <= 0.0f || !safety_ctx->safety_plausibility_ok) {
        state->d_term_filtered = 0.0f;
        state->first_run = true;
        commands->torque_left_nm = 0.0f;
        commands->torque_right_nm = 0.0f;
        commands->target_rpm_left = 0.0f;
        commands->target_rpm_right = 0.0f;
        commands->inverter_left_enable = false;
        commands->inverter_right_enable = false;
        commands->regen_active = false;
        return;
    }

    commands->inverter_left_enable = true;
    commands->inverter_right_enable = true;

    float base_throttle = clampf(safety_ctx->throttle_demand, 0.0f, 1.0f);
    float t_base = base_throttle * MOTOR_MAX_TORQUE_NM;

    float delta_rad = clampf(driver->steering_angle_rad, -MAX_STEERING_ANGLE_RAD, MAX_STEERING_ANGLE_RAD);
    float tan_delta = tanf(delta_rad);
    float v_x = fmaxf(0.0f, safety_ctx->estimated_vx_ms);

    /* Geometria de Ackermann nas rodas traseiras com margem de escorregamento */
    float omega_c = v_x / TIRE_RADIUS_DYNAMIC_M;
    const float geom_factor = REAR_TRACK_M / (2.0f * WHEELBASE_M);
    float ackerman_factor = geom_factor * tan_delta;

    float omega_left = fmaxf(0.0f, omega_c * (1.0f + ackerman_factor));
    float omega_right = fmaxf(0.0f, omega_c * (1.0f - ackerman_factor));

    float slip_margin = 1.0f + params->ackermann_slip_margin;
    float rpm_left = (omega_left * GEAR_RATIO) * RADS_TO_RPM * slip_margin;
    float rpm_right = (omega_right * GEAR_RATIO) * RADS_TO_RPM * slip_margin;

    /* Preserva a razao de velocidades de Ackermann sob saturacao de RPM */
    float max_rpm = fmaxf(rpm_left, rpm_right);
    if (max_rpm > MOTOR_MAX_RPM && max_rpm > 1.0f) {
        float rpm_scale = MOTOR_MAX_RPM / max_rpm;
        rpm_left *= rpm_scale;
        rpm_right *= rpm_scale;
    }
    commands->target_rpm_left = clampf(rpm_left, 0.0f, MOTOR_MAX_RPM);
    commands->target_rpm_right = clampf(rpm_right, 0.0f, MOTOR_MAX_RPM);

    /* Modelo de bicicleta 2-DOF para taxa de guinada de referencia */
    float denom = WHEELBASE_M + ((params->understeer_coeff * v_x * v_x) / GRAVITY_MSS);
    if (denom < 0.1f) {
        denom = 0.1f;
    }
    float yaw_rate_des = (v_x / denom) * tan_delta;
    float yaw_error = yaw_rate_des - sensors->yaw_rate_rads;

    /* Acao derivativa sobre a IMU com filtro IIR de primeira ordem */
    if (state->first_run) {
        state->prev_yaw_rate = sensors->yaw_rate_rads;
        state->d_term_filtered = 0.0f;
        state->first_run = false;
    } else {
        float tau = (params->tau_derivative_s > 0.001f) ? params->tau_derivative_s : ED_TAU_DERIVATIVE_S_DEFAULT;
        float alpha = tau / (tau + dt_s);
        float d_yaw_meas = -(sensors->yaw_rate_rads - state->prev_yaw_rate) / dt_s;
        state->prev_yaw_rate = sensors->yaw_rate_rads;
        state->d_term_filtered = (alpha * state->d_term_filtered) +
                                 ((1.0f - alpha) * params->kd_yaw * d_yaw_meas);
    }

    float delta_torque = (params->kp_yaw * yaw_error) + state->d_term_filtered;
    delta_torque = clampf(delta_torque, -params->max_delta_torque, params->max_delta_torque);

    /* Tapering suave de TV em baixas velocidades e baixo pedal */
    float speed_taper = clampf((v_x - 0.5f) / fmaxf(params->min_speed_tv_ms - 0.5f, 0.05f), 0.0f, 1.0f);
    float throttle_taper = clampf(base_throttle / fmaxf(params->min_throttle_tv, 0.01f), 0.0f, 1.0f);
    delta_torque *= (speed_taper * throttle_taper);

    if (v_x < 0.5f) {
        state->d_term_filtered = 0.0f;
    }

    /* Autoridade de vetorizacao em baixo pedal com margem de transicao */
    float tv_auth = fminf(params->max_delta_torque, t_base + 4.0f);
    delta_torque = clampf(delta_torque, -tv_auth, tv_auth);

    float t_left = t_base + delta_torque;
    float t_right = t_base - delta_torque;
    const float t_limit = MOTOR_MAX_TORQUE_NM;

    /* Saturacao superior preservando momento diferencial */
    if (t_left >= t_right && t_left > t_limit) {
        float excess = t_left - t_limit;
        t_left = t_limit;
        t_right -= excess;
    } else if (t_right > t_left && t_right > t_limit) {
        float excess = t_right - t_limit;
        t_right = t_limit;
        t_left -= excess;
    }

    /* Saturacao inferior sem dumping descontrolado */
    if (t_left < 0.0f) {
        t_left = 0.0f;
        t_right = clampf(t_right, 0.0f, fminf(t_limit, 2.0f * t_base + 4.0f));
    }
    if (t_right < 0.0f) {
        t_right = 0.0f;
        t_left = clampf(t_left, 0.0f, fminf(t_limit, 2.0f * t_base + 4.0f));
    }

    commands->torque_left_nm = clampf(t_left, 0.0f, t_limit);
    commands->torque_right_nm = clampf(t_right, 0.0f, t_limit);
    commands->regen_active = false;
}
