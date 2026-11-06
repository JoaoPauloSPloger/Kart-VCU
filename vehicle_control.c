/*
 * vehicle_control.c - Orquestracao e seguranca funcional da VCU
 * FSAE Electric / ISO 26262 - APPS plausibility, BSPD, limitador I_bus e termico
 */

#include "vehicle_control.h"
#include "electronic_differential.h"
#include "launch_control.h"
#include <stddef.h>

static void SetSafeMotorCommands(MotorCommands_t *commands) {
    if (!commands) {
        return;
    }
    commands->torque_left_nm = 0.0f;
    commands->torque_right_nm = 0.0f;
    commands->target_rpm_left = 0.0f;
    commands->target_rpm_right = 0.0f;
    commands->inverter_left_enable = false;
    commands->inverter_right_enable = false;
    commands->regen_active = false;
}

void VehicleControl_Init(VehicleSafetyContext_t *safety_ctx) {
    if (!safety_ctx) {
        return;
    }
    safety_ctx->implausibility_timer_s = 0.0f;
    safety_ctx->bspd_timer_s = 0.0f;
    safety_ctx->bspd_latch_tripped = false;
    safety_ctx->fault_flags = FAULT_NONE;
    safety_ctx->estimated_vx_ms = 0.0f;
    safety_ctx->vx_filter_initialized = false;
    safety_ctx->last_can_heartbeat = 0U;
    safety_ctx->can_watchdog_timer_s = 0.0f;
    safety_ctx->can_watchdog_armed = false;
    safety_ctx->raw_pedal_demand = 0.0f;
    safety_ctx->throttle_demand = 0.0f;
    safety_ctx->validated_brake_bar = 0.0f;
    safety_ctx->safety_plausibility_ok = false;
    safety_ctx->battery_current_scale = 1.0f;
    safety_ctx->regen_current_scale = 1.0f;
}

bool VehicleControl_ValidateInputs(VehicleSafetyContext_t *safety_ctx,
                                   const DriverInputs_t *inputs,
                                   bool is_staging,
                                   float dt_s) {
    if (!safety_ctx || !inputs || !isfinite(dt_s) || dt_s <= 0.0f) {
        if (safety_ctx) {
            safety_ctx->fault_flags |= FAULT_INVALID_DT;
            safety_ctx->safety_plausibility_ok = false;
            safety_ctx->raw_pedal_demand = 0.0f;
            safety_ctx->throttle_demand = 0.0f;
            safety_ctx->validated_brake_bar = 0.0f;
        }
        return false;
    }

    if (!isfinite(inputs->apps1_percent) ||
        !isfinite(inputs->apps2_percent) ||
        !isfinite(inputs->brake_pressure_bar) ||
        !isfinite(inputs->steering_angle_rad)) {
        safety_ctx->fault_flags |= FAULT_SENSOR_NAN;
        safety_ctx->safety_plausibility_ok = false;
        safety_ctx->raw_pedal_demand = 0.0f;
        safety_ctx->throttle_demand = 0.0f;
        safety_ctx->validated_brake_bar = 0.0f;
        return false;
    }

    safety_ctx->fault_flags &= ~FAULT_SENSOR_NAN;
    safety_ctx->fault_flags &= ~FAULT_INVALID_DT;

    if (fabsf(inputs->steering_angle_rad) > (MAX_STEERING_ANGLE_RAD + 0.01f)) {
        safety_ctx->fault_flags |= FAULT_STEERING_RANGE;
        safety_ctx->safety_plausibility_ok = false;
        safety_ctx->raw_pedal_demand = 0.0f;
        safety_ctx->throttle_demand = 0.0f;
        safety_ctx->validated_brake_bar = 0.0f;
        return false;
    }
    safety_ctx->fault_flags &= ~FAULT_STEERING_RANGE;

    bool range_ok = true;
    if (inputs->apps1_percent < 0.0f || inputs->apps1_percent > 100.0f ||
        inputs->apps2_percent < 0.0f || inputs->apps2_percent > 100.0f) {
        safety_ctx->fault_flags |= FAULT_APPS_OUT_OF_RANGE;
        range_ok = false;
    } else {
        safety_ctx->fault_flags &= ~FAULT_APPS_OUT_OF_RANGE;
    }

    if (inputs->brake_pressure_bar < 0.0f || inputs->brake_pressure_bar > BRAKE_PRESSURE_MAX_BAR) {
        safety_ctx->fault_flags |= FAULT_BRAKE_OUT_OF_RANGE;
        range_ok = false;
    } else {
        safety_ctx->fault_flags &= ~FAULT_BRAKE_OUT_OF_RANGE;
    }

    if (!range_ok) {
        safety_ctx->safety_plausibility_ok = false;
        safety_ctx->raw_pedal_demand = 0.0f;
        safety_ctx->throttle_demand = 0.0f;
        safety_ctx->validated_brake_bar = 0.0f;
        return false;
    }

    safety_ctx->validated_brake_bar = inputs->brake_pressure_bar;

    /* FSAE EV.4.7: Verificacao de discrepancia de 10% entre canais redundantes de APPS */
    float apps_diff = fabsf(inputs->apps1_percent - inputs->apps2_percent);
    if (apps_diff > APPS_DISCREPANCY_MAX_PCT) {
        safety_ctx->implausibility_timer_s += dt_s;
    } else {
        safety_ctx->implausibility_timer_s = 0.0f;
        safety_ctx->fault_flags &= ~FAULT_APPS_DISCREPANCY;
    }

    if (safety_ctx->implausibility_timer_s >= APPS_DISCREPANCY_TIMEOUT_S) {
        safety_ctx->fault_flags |= FAULT_APPS_DISCREPANCY;
        safety_ctx->safety_plausibility_ok = false;
        safety_ctx->raw_pedal_demand = 0.0f;
        safety_ctx->throttle_demand = 0.0f;
        return false;
    }

    float avg_apps = 0.5f * (inputs->apps1_percent + inputs->apps2_percent);
    float raw_demand = (avg_apps < APPS_DEADBAND_PERCENT) ? 0.0f : clampf(avg_apps / 100.0f, 0.0f, 1.0f);
    safety_ctx->raw_pedal_demand = raw_demand;

    bool brake_applied = (inputs->brake_pressure_bar > BSPD_BRAKE_THRESHOLD_BAR) || inputs->brake_switch_active;

    /* FSAE T.4.3: BSPD com debounce de 150 ms para rejeicao de transientes piezoeletricos */
    if (is_staging) {
        safety_ctx->bspd_timer_s = 0.0f;
        safety_ctx->bspd_latch_tripped = false;
        safety_ctx->fault_flags &= ~FAULT_BSPD_TRIPPED;
    } else {
        if (brake_applied && avg_apps > BSPD_APPS_TRIP_PERCENT) {
            safety_ctx->bspd_timer_s += dt_s;
            if (safety_ctx->bspd_timer_s >= BSPD_DEBOUNCE_TIME_S) {
                safety_ctx->bspd_latch_tripped = true;
            }
        } else {
            safety_ctx->bspd_timer_s = 0.0f;
        }

        if (safety_ctx->bspd_latch_tripped) {
            if (avg_apps < BSPD_APPS_RECOVERY_PERCENT) {
                safety_ctx->bspd_latch_tripped = false;
                safety_ctx->bspd_timer_s = 0.0f;
                safety_ctx->fault_flags &= ~FAULT_BSPD_TRIPPED;
            } else {
                safety_ctx->fault_flags |= FAULT_BSPD_TRIPPED;
                safety_ctx->safety_plausibility_ok = true;
                safety_ctx->throttle_demand = 0.0f;
                return true;
            }
        }
    }

    safety_ctx->fault_flags &= ~FAULT_BSPD_TRIPPED;
    safety_ctx->safety_plausibility_ok = true;
    safety_ctx->throttle_demand = raw_demand;
    return true;
}

bool VehicleControl_ValidateSensors(VehicleSafetyContext_t *safety_ctx,
                                    const SensorData_t *sensors,
                                    float dt_s) {
    if (!safety_ctx || !sensors || !isfinite(dt_s) || dt_s <= 0.0f) {
        if (safety_ctx) {
            safety_ctx->fault_flags |= FAULT_INVALID_DT;
        }
        return false;
    }

    if (!isfinite(sensors->wheel_speed_fl_rads) ||
        !isfinite(sensors->wheel_speed_fr_rads) ||
        !isfinite(sensors->wheel_speed_rl_rads) ||
        !isfinite(sensors->wheel_speed_rr_rads) ||
        !isfinite(sensors->yaw_rate_rads) ||
        !isfinite(sensors->ax_mss) ||
        !isfinite(sensors->ay_mss) ||
        !isfinite(sensors->motor_temp_left_c) ||
        !isfinite(sensors->motor_temp_right_c) ||
        !isfinite(sensors->inverter_temp_left_c) ||
        !isfinite(sensors->inverter_temp_right_c) ||
        !isfinite(sensors->battery_voltage_v) ||
        !isfinite(sensors->battery_current_a)) {
        safety_ctx->fault_flags |= FAULT_SENSOR_NAN;
        return false;
    }
    safety_ctx->fault_flags &= ~FAULT_SENSOR_NAN;

    /* Verificacao de faixa operacional dos sensores */
    if (sensors->motor_temp_left_c < -20.0f || sensors->motor_temp_left_c > 180.0f ||
        sensors->motor_temp_right_c < -20.0f || sensors->motor_temp_right_c > 180.0f ||
        sensors->inverter_temp_left_c < -20.0f || sensors->inverter_temp_left_c > 180.0f ||
        sensors->inverter_temp_right_c < -20.0f || sensors->inverter_temp_right_c > 180.0f) {
        safety_ctx->fault_flags |= FAULT_THERMAL_SENSOR;
        return false;
    }
    safety_ctx->fault_flags &= ~FAULT_THERMAL_SENSOR;

    if (sensors->battery_voltage_v < 0.0f || sensors->battery_voltage_v > 120.0f) {
        safety_ctx->fault_flags |= FAULT_BATTERY_VOLTAGE_RANGE;
        return false;
    }
    safety_ctx->fault_flags &= ~FAULT_BATTERY_VOLTAGE_RANGE;

    if (sensors->wheel_speed_fl_rads < -50.0f || sensors->wheel_speed_fl_rads > 500.0f ||
        sensors->wheel_speed_fr_rads < -50.0f || sensors->wheel_speed_fr_rads > 500.0f ||
        sensors->wheel_speed_rl_rads < -50.0f || sensors->wheel_speed_rl_rads > 500.0f ||
        sensors->wheel_speed_rr_rads < -50.0f || sensors->wheel_speed_rr_rads > 500.0f) {
        safety_ctx->fault_flags |= FAULT_WHEEL_SPEED_SENSOR;
        return false;
    }
    safety_ctx->fault_flags &= ~FAULT_WHEEL_SPEED_SENSOR;

    if (fabsf(sensors->ax_mss) > 60.0f || fabsf(sensors->ay_mss) > 60.0f ||
        fabsf(sensors->yaw_rate_rads) > 25.0f) {
        safety_ctx->fault_flags |= FAULT_IMU_RANGE;
        return false;
    }
    safety_ctx->fault_flags &= ~FAULT_IMU_RANGE;

    /* ISO 26262 ASIL-D: Watchdog do barramento CAN (100 ms timeout) */
    if (!safety_ctx->can_watchdog_armed) {
        safety_ctx->can_watchdog_armed = true;
        safety_ctx->last_can_heartbeat = sensors->can_heartbeat_counter;
        safety_ctx->can_watchdog_timer_s = 0.0f;
    } else if (sensors->can_heartbeat_counter != safety_ctx->last_can_heartbeat) {
        safety_ctx->last_can_heartbeat = sensors->can_heartbeat_counter;
        safety_ctx->can_watchdog_timer_s = 0.0f;
        safety_ctx->fault_flags &= ~FAULT_CAN_TIMEOUT;
    } else {
        safety_ctx->can_watchdog_timer_s += dt_s;
        if (safety_ctx->can_watchdog_timer_s >= CAN_WATCHDOG_TIMEOUT_S) {
            safety_ctx->fault_flags |= FAULT_CAN_TIMEOUT;
            return false;
        }
    }

    return true;
}

float VehicleControl_EstimateChassisSpeed(VehicleSafetyContext_t *safety_ctx,
                                          const DriverInputs_t *driver,
                                          const SensorData_t *sensors,
                                          float dt_s) {
    if (!safety_ctx || !sensors || !isfinite(dt_s) || dt_s <= 0.0f) {
        return 0.0f;
    }

    float safe_dt = clampf(dt_s, 0.001f, DT_MAX_SATURATION_S);
    float v_fl = fmaxf(0.0f, sensors->wheel_speed_fl_rads * FRONT_TIRE_RADIUS_DYNAMIC_M);
    float v_fr = fmaxf(0.0f, sensors->wheel_speed_fr_rads * FRONT_TIRE_RADIUS_DYNAMIC_M);
    float v_max_front = fmaxf(v_fl, v_fr);
    float v_avg_front = 0.5f * (v_fl + v_fr);

    bool braking = driver && ((driver->brake_pressure_bar > 5.0f) || driver->brake_switch_active);
    bool wheel_lift_or_lock = (v_max_front > 1.0f) && (fabsf(v_fl - v_fr) > (0.12f * v_max_front));
    float v_wheel_ref = (braking || wheel_lift_or_lock) ? v_max_front : v_avg_front;

    /* Filtro complementar: fusao vel. roda livre dianteira + integrador ax IMU */
    if (!safety_ctx->vx_filter_initialized || (safety_ctx->estimated_vx_ms < 0.01f && v_wheel_ref > 0.5f)) {
        safety_ctx->estimated_vx_ms = v_wheel_ref;
        safety_ctx->vx_filter_initialized = true;
    } else {
        float v_imu_pred = fmaxf(0.0f, safety_ctx->estimated_vx_ms + (sensors->ax_mss * safe_dt));
        bool front_axle_locked = braking && (safety_ctx->estimated_vx_ms > 2.0f) &&
                                 (v_max_front < (0.60f * safety_ctx->estimated_vx_ms));
        float alpha_wheel = front_axle_locked ? 0.15f : 0.88f;

        safety_ctx->estimated_vx_ms = (alpha_wheel * v_wheel_ref) + ((1.0f - alpha_wheel) * v_imu_pred);
        if (v_wheel_ref < 0.05f && fabsf(sensors->ax_mss) < 0.2f && !front_axle_locked) {
            safety_ctx->estimated_vx_ms = 0.0f;
        }
    }

    safety_ctx->estimated_vx_ms = clampf(safety_ctx->estimated_vx_ms, 0.0f, MAX_SPEED_MS * 1.25f);
    return safety_ctx->estimated_vx_ms;
}

void VehicleControl_ApplyPowerAndThermalLimits(VehicleSafetyContext_t *safety_ctx,
                                               const SensorData_t *sensors,
                                               MotorCommands_t *commands,
                                               float dt_s) {
    if (!safety_ctx || !sensors || !commands) {
        return;
    }

    if (!safety_ctx->safety_plausibility_ok ||
        (safety_ctx->fault_flags & FAULT_CRITICAL_DRIVE_MASK) != 0U) {
        SetSafeMotorCommands(commands);
        return;
    }

    commands->target_rpm_left = clampf(commands->target_rpm_left, 0.0f, MOTOR_MAX_RPM);
    commands->target_rpm_right = clampf(commands->target_rpm_right, 0.0f, MOTOR_MAX_RPM);

    /* Derating termico proporcional com corte total em temperatura critica */
    float max_motor_temp = fmaxf(sensors->motor_temp_left_c, sensors->motor_temp_right_c);
    float max_inv_temp = fmaxf(sensors->inverter_temp_left_c, sensors->inverter_temp_right_c);

    float motor_scale = 1.0f;
    if (max_motor_temp >= MOTOR_TEMP_CUTOFF_C) {
        motor_scale = 0.0f;
    } else if (max_motor_temp > MOTOR_TEMP_DERATING_START_C) {
        motor_scale = (MOTOR_TEMP_CUTOFF_C - max_motor_temp) /
                      (MOTOR_TEMP_CUTOFF_C - MOTOR_TEMP_DERATING_START_C);
    }

    float inv_scale = 1.0f;
    if (max_inv_temp >= INVERTER_TEMP_CUTOFF_C) {
        inv_scale = 0.0f;
    } else if (max_inv_temp > INVERTER_TEMP_DERATING_START_C) {
        inv_scale = (INVERTER_TEMP_CUTOFF_C - max_inv_temp) /
                    (INVERTER_TEMP_CUTOFF_C - INVERTER_TEMP_DERATING_START_C);
    }

    float thermal_scale = clampf(fminf(motor_scale, inv_scale), 0.0f, 1.0f);
    if (max_motor_temp > MOTOR_TEMP_DERATING_START_C || max_inv_temp > INVERTER_TEMP_DERATING_START_C) {
        safety_ctx->fault_flags |= FAULT_OVER_TEMP;
    } else {
        safety_ctx->fault_flags &= ~FAULT_OVER_TEMP;
    }

    if (thermal_scale <= 0.0f) {
        SetSafeMotorCommands(commands);
        return;
    }

    /* Derating de subtensao do acumulador LiFePO4 */
    float v_batt = sensors->battery_voltage_v;
    float voltage_scale = 1.0f;
    if (v_batt < BATTERY_UNDERVOLTAGE_START_V) {
        safety_ctx->fault_flags |= FAULT_UNDERVOLTAGE;
        if (v_batt <= BATTERY_UNDERVOLTAGE_CUTOFF_V) {
            voltage_scale = 0.0f;
        } else {
            voltage_scale = (v_batt - BATTERY_UNDERVOLTAGE_CUTOFF_V) /
                            (BATTERY_UNDERVOLTAGE_START_V - BATTERY_UNDERVOLTAGE_CUTOFF_V);
        }
    } else {
        safety_ctx->fault_flags &= ~FAULT_UNDERVOLTAGE;
    }

    if (voltage_scale <= 0.0f) {
        SetSafeMotorCommands(commands);
        return;
    }

    float combined_derating = fminf(thermal_scale, voltage_scale);
    float t_limit = MOTOR_MAX_TORQUE_NM * combined_derating;

    float v_x = fmaxf(0.0f, safety_ctx->estimated_vx_ms);
    float omega_rl = (fabsf(sensors->wheel_speed_rl_rads) > 0.01f) ?
                     fabsf(sensors->wheel_speed_rl_rads) : (v_x / TIRE_RADIUS_DYNAMIC_M);
    float omega_rr = (fabsf(sensors->wheel_speed_rr_rads) > 0.01f) ?
                     fabsf(sensors->wheel_speed_rr_rads) : (v_x / TIRE_RADIUS_DYNAMIC_M);
    float omega_mot_l = omega_rl * GEAR_RATIO;
    float omega_mot_r = omega_rr * GEAR_RATIO;

    const float max_omega_mot_rads = MOTOR_MAX_RPM * RPM_TO_RADS;
    float max_curr_omega_mot = fmaxf(omega_mot_l, omega_mot_r);

    /* Frenagem regenerativa com curva de brake blending e supervisao de slip traseiro */
    bool can_regen = (safety_ctx->throttle_demand <= 0.001f) &&
                     (safety_ctx->raw_pedal_demand <= 0.02f) &&
                     (commands->torque_left_nm <= 0.01f) &&
                     (commands->torque_right_nm <= 0.01f) &&
                     (v_x > REGEN_MIN_SPEED_MS) &&
                     (max_curr_omega_mot < max_omega_mot_rads) &&
                     (v_batt < BATTERY_MAX_REGEN_VOLTAGE_V) &&
                     !safety_ctx->bspd_latch_tripped;

    if (can_regen) {
        float v_rear_avg = 0.5f * (omega_rl + omega_rr) * TIRE_RADIUS_DYNAMIC_M;
        float rear_slip = (v_x - v_rear_avg) / fmaxf(v_x, 1.0f);

        if (rear_slip < 0.15f) {
            float brake_bar = safety_ctx->validated_brake_bar;
            float regen_frac = 0.0f;

            if (brake_bar < 0.5f) {
                regen_frac = 0.20f;
            } else if (brake_bar <= 35.0f) {
                regen_frac = 0.20f + 0.40f * (brake_bar / 35.0f);
            } else if (brake_bar < 65.0f) {
                regen_frac = 0.60f * ((65.0f - brake_bar) / 30.0f);
            } else {
                regen_frac = 0.0f;
            }

            if (regen_frac > 0.0f) {
                float speed_taper = clampf((v_x - REGEN_MIN_SPEED_MS) / 2.0f, 0.0f, 1.0f);
                float t_regen_each = regen_frac * speed_taper * t_limit;

                if (t_regen_each > 0.05f) {
                    float p_regen_max_w = v_batt * BATTERY_MAX_REGEN_A;
                    float p_regen_mec_w = t_regen_each * (omega_mot_l + omega_mot_r);
                    float p_regen_elec_w = p_regen_mec_w * INVERTER_MOTOR_EFFICIENCY;

                    if (p_regen_elec_w > p_regen_max_w && p_regen_elec_w > 1.0f) {
                        t_regen_each *= (p_regen_max_w / p_regen_elec_w);
                    }

                    /* Limitador de corrente de recarga com atualizacao multiplicativa amortecida */
                    if (sensors->battery_current_a < -BATTERY_MAX_REGEN_A) {
                        float ratio = BATTERY_MAX_REGEN_A / fabsf(sensors->battery_current_a);
                        safety_ctx->regen_current_scale = clampf(safety_ctx->regen_current_scale * ratio, 0.05f, 1.0f);
                    } else {
                        safety_ctx->regen_current_scale = fminf(1.0f, safety_ctx->regen_current_scale + (2.0f * dt_s));
                    }
                    t_regen_each *= safety_ctx->regen_current_scale;

                    t_regen_each = clampf(t_regen_each, 0.0f, t_limit);
                    commands->torque_left_nm = -t_regen_each;
                    commands->torque_right_nm = -t_regen_each;
                    commands->target_rpm_left = 0.0f;
                    commands->target_rpm_right = 0.0f;
                    commands->inverter_left_enable = true;
                    commands->inverter_right_enable = true;
                    commands->regen_active = true;
                    return;
                }
            }
        }
    }

    commands->regen_active = false;

    float t_left = commands->torque_left_nm;
    float t_right = commands->torque_right_nm;

    /* Saturação preservando o torque diferencial */
    if (t_left >= t_right && t_left > t_limit) {
        float excess = t_left - t_limit;
        t_left = t_limit;
        t_right = fmaxf(0.0f, t_right - excess);
    } else if (t_right > t_left && t_right > t_limit) {
        float excess = t_right - t_limit;
        t_right = t_limit;
        t_left = fmaxf(0.0f, t_left - excess);
    }

    t_left = clampf(t_left, 0.0f, t_limit);
    t_right = clampf(t_right, 0.0f, t_limit);

    /* Governador de velocidade maxima e sobre-rotacao */
    if (v_x >= MAX_SPEED_MS || max_curr_omega_mot >= max_omega_mot_rads) {
        float speed_over = (v_x - MAX_SPEED_MS) / (0.02f * MAX_SPEED_MS);
        float rpm_over = (max_curr_omega_mot - max_omega_mot_rads) / (0.02f * max_omega_mot_rads);
        float over_frac = fmaxf(speed_over, rpm_over);
        float gov_scale = clampf(1.0f - over_frac, 0.0f, 1.0f);
        t_left *= gov_scale;
        t_right *= gov_scale;
    }

    /* Limitacao de potencia eletrica total */
    float p_mec_w = (fabsf(t_left) * omega_mot_l) + (fabsf(t_right) * omega_mot_r);
    float p_elec_w = p_mec_w / INVERTER_MOTOR_EFFICIENCY;

    float p_max_sys_w = SYSTEM_MAX_POWER_KW * 1000.0f;
    float p_max_batt_w = v_batt * BATTERY_MAX_DISCHARGE_A;
    float p_elec_lim_w = fminf(p_max_sys_w, p_max_batt_w);

    if (p_elec_w > p_elec_lim_w && p_elec_w > 1.0f) {
        float power_scale = p_elec_lim_w / p_elec_w;
        t_left *= power_scale;
        t_right *= power_scale;
    }

    /* Malha de derating I_bus (teto 220 A descarga com controle multiplicativo) */
    if (sensors->battery_current_a > BATTERY_MAX_DISCHARGE_A) {
        float ratio = BATTERY_MAX_DISCHARGE_A / sensors->battery_current_a;
        safety_ctx->battery_current_scale = clampf(safety_ctx->battery_current_scale * ratio, 0.05f, 1.0f);
    } else {
        safety_ctx->battery_current_scale = fminf(1.0f, safety_ctx->battery_current_scale + (2.0f * dt_s));
    }

    t_left *= safety_ctx->battery_current_scale;
    t_right *= safety_ctx->battery_current_scale;

    commands->torque_left_nm = clampf(t_left, 0.0f, t_limit);
    commands->torque_right_nm = clampf(t_right, 0.0f, t_limit);
}

void VehicleControl_Step(VehicleSafetyContext_t *safety_ctx,
                         const ElectronicDiffParams_t *diff_params,
                         ElectronicDiffState_t *diff_state,
                         LaunchControlContext_t *lc_ctx,
                         const DriverInputs_t *driver,
                         const SensorData_t *sensors,
                         MotorCommands_t *commands,
                         float dt_s) {
    if (!safety_ctx || !diff_params || !diff_state || !driver || !sensors || !commands) {
        return;
    }

    if (!isfinite(dt_s) || dt_s <= 0.0f) {
        safety_ctx->fault_flags |= FAULT_INVALID_DT;
        safety_ctx->safety_plausibility_ok = false;
        safety_ctx->raw_pedal_demand = 0.0f;
        safety_ctx->throttle_demand = 0.0f;
        safety_ctx->validated_brake_bar = 0.0f;
        SetSafeMotorCommands(commands);
        return;
    }

    float effective_dt = clampf(dt_s, 0.001f, DT_MAX_SATURATION_S);
    bool is_staging = (lc_ctx != NULL && lc_ctx->state == LC_STATE_STAGING);

    bool inputs_ok = VehicleControl_ValidateInputs(safety_ctx, driver, is_staging, effective_dt);
    bool sensors_ok = VehicleControl_ValidateSensors(safety_ctx, sensors, effective_dt);

    if (!inputs_ok || !sensors_ok) {
        safety_ctx->safety_plausibility_ok = false;
        safety_ctx->throttle_demand = 0.0f;
    }

    if (sensors_ok) {
        VehicleControl_EstimateChassisSpeed(safety_ctx, driver, sensors, effective_dt);
    }

    ElectronicDiff_Update(diff_params, diff_state, driver, sensors, safety_ctx, commands, effective_dt);

    if (lc_ctx) {
        (void)LaunchControl_Update(lc_ctx, driver, sensors, safety_ctx, commands, effective_dt);

        if (lc_ctx->state == LC_STATE_ABORTED) {
            bool brake_applied = (driver->brake_pressure_bar > BSPD_BRAKE_THRESHOLD_BAR) ||
                                 driver->brake_switch_active;
            if (brake_applied && safety_ctx->raw_pedal_demand > (BSPD_APPS_TRIP_PERCENT / 100.0f)) {
                safety_ctx->bspd_latch_tripped = true;
                safety_ctx->fault_flags |= FAULT_BSPD_TRIPPED;
                safety_ctx->throttle_demand = 0.0f;
            }
        }
    }

    VehicleControl_ApplyPowerAndThermalLimits(safety_ctx, sensors, commands, effective_dt);
}
