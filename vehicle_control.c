/*
 * vehicle_control.c - Lógica central da VCU do kart elétrico
 * Validação de segurança (APPS, BSPD), estimação de velocidade,
 * proteção eletrotérmica, frenagem regenerativa e orquestração.
 */

#include "vehicle_control.h"
#include "electronic_differential.h"
#include "launch_control.h"
#include <stddef.h>

#define CAN_WATCHDOG_TIMEOUT_S          (0.100f)
#define APPS_DISCREPANCY_MAX_PCT        (10.0f)
#define APPS_DISCREPANCY_TIMEOUT_S      (0.100f)
#define BSPD_BRAKE_THRESHOLD_BAR        (15.0f)
#define BSPD_APPS_TRIP_PERCENT          (25.0f)
#define BSPD_APPS_RECOVERY_PERCENT      (5.0f)

static void SetSafeMotorCommands(MotorCommands_t *commands) {
    if (!commands) return;
    commands->torque_left_nm        = 0.0f;
    commands->torque_right_nm       = 0.0f;
    commands->target_rpm_left       = 0.0f;
    commands->target_rpm_right      = 0.0f;
    commands->inverter_left_enable  = false;
    commands->inverter_right_enable = false;
    commands->regen_active          = false;
}

void VehicleControl_Init(VehicleSafetyContext_t *safety_ctx) {
    if (!safety_ctx) return;
    safety_ctx->implausibility_timer_s = 0.0f;
    safety_ctx->bspd_latch_tripped     = false;
    safety_ctx->fault_flags            = FAULT_NONE;
    safety_ctx->estimated_vx_ms        = 0.0f;
    safety_ctx->vx_filter_initialized  = false;
    safety_ctx->last_can_heartbeat     = 0U;
    safety_ctx->can_watchdog_timer_s   = 0.0f;
    safety_ctx->can_watchdog_armed     = false;
}

bool VehicleControl_ValidateInputs(VehicleSafetyContext_t *safety_ctx,
                                   DriverInputs_t *inputs,
                                   float dt_s) {
    if (!safety_ctx || !inputs || !isfinite(dt_s) || dt_s <= 0.0f) {
        if (safety_ctx) {
            safety_ctx->fault_flags |= FAULT_SENSOR_NAN;
        }
        if (inputs) {
            inputs->safety_plausibility_ok = false;
            inputs->raw_pedal_demand       = 0.0f;
            inputs->throttle_demand        = 0.0f;
        }
        return false;
    }

    if (!isfinite(inputs->apps1_percent) ||
        !isfinite(inputs->apps2_percent) ||
        !isfinite(inputs->brake_pressure_bar) ||
        !isfinite(inputs->steering_angle_deg) ||
        !isfinite(inputs->steering_angle_rad)) {
        safety_ctx->fault_flags |= FAULT_SENSOR_NAN;
        inputs->safety_plausibility_ok = false;
        inputs->raw_pedal_demand       = 0.0f;
        inputs->throttle_demand        = 0.0f;
        return false;
    }

    /* Sincronização contínua entre radianos e graus do esterçamento */
    if (fabsf(inputs->steering_angle_rad) < 1e-6f && fabsf(inputs->steering_angle_deg) > 1e-4f) {
        inputs->steering_angle_rad = inputs->steering_angle_deg * DEG_TO_RAD;
    } else if (fabsf(inputs->steering_angle_deg) > 1e-4f &&
               fabsf(inputs->steering_angle_deg * DEG_TO_RAD - inputs->steering_angle_rad) > 1e-3f) {
        inputs->steering_angle_rad = inputs->steering_angle_deg * DEG_TO_RAD;
    } else {
        inputs->steering_angle_deg = inputs->steering_angle_rad * RAD_TO_DEG;
    }

    if (fabsf(inputs->steering_angle_rad) > (MAX_STEERING_ANGLE_RAD + 0.01f) ||
        fabsf(inputs->steering_angle_deg) > (MAX_STEERING_ANGLE_DEG + 0.5f)) {
        safety_ctx->fault_flags |= FAULT_SENSOR_NAN;
        inputs->safety_plausibility_ok = false;
        inputs->raw_pedal_demand       = 0.0f;
        inputs->throttle_demand        = 0.0f;
        return false;
    }

    safety_ctx->fault_flags &= ~FAULT_SENSOR_NAN;

    bool range_ok = true;
    if (inputs->apps1_percent < 0.0f || inputs->apps1_percent > 100.0f ||
        inputs->apps2_percent < 0.0f || inputs->apps2_percent > 100.0f) {
        safety_ctx->fault_flags |= FAULT_APPS_OUT_OF_RANGE;
        range_ok = false;
    } else {
        safety_ctx->fault_flags &= ~FAULT_APPS_OUT_OF_RANGE;
    }

    if (inputs->brake_pressure_bar < 0.0f || inputs->brake_pressure_bar > 120.0f) {
        safety_ctx->fault_flags |= FAULT_BRAKE_OUT_OF_RANGE;
        range_ok = false;
    } else {
        safety_ctx->fault_flags &= ~FAULT_BRAKE_OUT_OF_RANGE;
    }

    if (!range_ok) {
        inputs->safety_plausibility_ok = false;
        inputs->raw_pedal_demand       = 0.0f;
        inputs->throttle_demand        = 0.0f;
        return false;
    }

    float apps_diff = fabsf(inputs->apps1_percent - inputs->apps2_percent);
    if (apps_diff > APPS_DISCREPANCY_MAX_PCT) {
        safety_ctx->implausibility_timer_s += dt_s;
    } else {
        safety_ctx->implausibility_timer_s = 0.0f;
        safety_ctx->fault_flags &= ~FAULT_APPS_DISCREPANCY;
    }

    if (safety_ctx->implausibility_timer_s >= APPS_DISCREPANCY_TIMEOUT_S) {
        safety_ctx->fault_flags |= FAULT_APPS_DISCREPANCY;
        inputs->safety_plausibility_ok = false;
        inputs->raw_pedal_demand       = 0.0f;
        inputs->throttle_demand        = 0.0f;
        return false;
    }

    float avg_apps = 0.5f * (inputs->apps1_percent + inputs->apps2_percent);
    float raw_demand = (avg_apps < 2.0f) ? 0.0f : clampf(avg_apps / 100.0f, 0.0f, 1.0f);
    inputs->raw_pedal_demand = raw_demand;

    bool brake_applied = (inputs->brake_pressure_bar > BSPD_BRAKE_THRESHOLD_BAR) ||
                         inputs->brake_switch_active;

    if (brake_applied && avg_apps > BSPD_APPS_TRIP_PERCENT) {
        safety_ctx->bspd_latch_tripped = true;
    }

    if (safety_ctx->bspd_latch_tripped) {
        if (avg_apps < BSPD_APPS_RECOVERY_PERCENT) {
            safety_ctx->bspd_latch_tripped = false;
            safety_ctx->fault_flags &= ~FAULT_BSPD_TRIPPED;
        } else {
            safety_ctx->fault_flags |= FAULT_BSPD_TRIPPED;
            inputs->safety_plausibility_ok = true;
            inputs->throttle_demand        = 0.0f;
            return true;
        }
    }

    safety_ctx->fault_flags &= ~FAULT_BSPD_TRIPPED;
    inputs->safety_plausibility_ok = true;
    inputs->throttle_demand        = raw_demand;
    return true;
}

bool VehicleControl_ValidateSensors(VehicleSafetyContext_t *safety_ctx,
                                    const SensorData_t *sensors,
                                    float dt_s) {
    if (!safety_ctx || !sensors || !isfinite(dt_s) || dt_s <= 0.0f) {
        if (safety_ctx) safety_ctx->fault_flags |= FAULT_SENSOR_NAN;
        return false;
    }

    if (!isfinite(sensors->wheel_speed_fl_rads) ||
        !isfinite(sensors->wheel_speed_fr_rads) ||
        !isfinite(sensors->wheel_speed_rl_rads) ||
        !isfinite(sensors->wheel_speed_rr_rads) ||
        !isfinite(sensors->vehicle_speed_ms)    ||
        !isfinite(sensors->yaw_rate_rads)       ||
        !isfinite(sensors->ax_mss)              ||
        !isfinite(sensors->ay_mss)              ||
        !isfinite(sensors->motor_temp_left_c)   ||
        !isfinite(sensors->motor_temp_right_c)  ||
        !isfinite(sensors->inverter_temp_left_c)||
        !isfinite(sensors->inverter_temp_right_c)||
        !isfinite(sensors->battery_voltage_v)   ||
        !isfinite(sensors->battery_current_a)) {
        safety_ctx->fault_flags |= FAULT_SENSOR_NAN;
        return false;
    }

    if (sensors->motor_temp_left_c < -20.0f     || sensors->motor_temp_left_c > 180.0f ||
        sensors->motor_temp_right_c < -20.0f    || sensors->motor_temp_right_c > 180.0f ||
        sensors->inverter_temp_left_c < -20.0f  || sensors->inverter_temp_left_c > 180.0f ||
        sensors->inverter_temp_right_c < -20.0f || sensors->inverter_temp_right_c > 180.0f ||
        sensors->battery_voltage_v < 0.0f       || sensors->battery_voltage_v > 120.0f ||
        sensors->wheel_speed_fl_rads < -50.0f   || sensors->wheel_speed_fl_rads > 500.0f ||
        sensors->wheel_speed_fr_rads < -50.0f   || sensors->wheel_speed_fr_rads > 500.0f ||
        sensors->wheel_speed_rl_rads < -50.0f   || sensors->wheel_speed_rl_rads > 500.0f ||
        sensors->wheel_speed_rr_rads < -50.0f   || sensors->wheel_speed_rr_rads > 500.0f ||
        fabsf(sensors->ax_mss) > 60.0f          || fabsf(sensors->ay_mss) > 60.0f ||
        fabsf(sensors->yaw_rate_rads) > 25.0f) {
        safety_ctx->fault_flags |= FAULT_SENSOR_NAN;
        return false;
    }

    safety_ctx->fault_flags &= ~FAULT_SENSOR_NAN;

    /* Watchdog do barramento CAN sempre ativo desde o primeiro ciclo */
    if (!safety_ctx->can_watchdog_armed) {
        safety_ctx->can_watchdog_armed   = true;
        safety_ctx->last_can_heartbeat   = sensors->can_heartbeat_counter;
        safety_ctx->can_watchdog_timer_s = 0.0f;
    } else if (sensors->can_heartbeat_counter != safety_ctx->last_can_heartbeat) {
        safety_ctx->last_can_heartbeat   = sensors->can_heartbeat_counter;
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
                                          SensorData_t *sensors,
                                          float dt_s) {
    if (!safety_ctx || !sensors || !isfinite(dt_s) || dt_s <= 0.0f) return 0.0f;

    float v_fl = fmaxf(0.0f, sensors->wheel_speed_fl_rads * FRONT_TIRE_RADIUS_DYNAMIC_M);
    float v_fr = fmaxf(0.0f, sensors->wheel_speed_fr_rads * FRONT_TIRE_RADIUS_DYNAMIC_M);
    float v_max_front = fmaxf(v_fl, v_fr);
    float v_avg_front = 0.5f * (v_fl + v_fr);

    bool braking = driver && ((driver->brake_pressure_bar > 5.0f) || driver->brake_switch_active);
    bool wheel_lift_or_lock = (v_max_front > 1.0f) && (fabsf(v_fl - v_fr) > (0.12f * v_max_front));

    float v_wheel_ref = (braking || wheel_lift_or_lock) ? v_max_front : v_avg_front;

    if (!safety_ctx->vx_filter_initialized || (safety_ctx->estimated_vx_ms < 0.01f && v_wheel_ref > 0.5f)) {
        safety_ctx->estimated_vx_ms       = v_wheel_ref;
        safety_ctx->vx_filter_initialized = true;
    } else {
        float v_imu_pred = fmaxf(0.0f, safety_ctx->estimated_vx_ms + (sensors->ax_mss * dt_s));

        bool front_axle_locked = braking && (safety_ctx->estimated_vx_ms > 2.0f) &&
                                 (v_max_front < (0.60f * safety_ctx->estimated_vx_ms));
        float alpha_wheel = front_axle_locked ? 0.15f : 0.88f;

        safety_ctx->estimated_vx_ms = (alpha_wheel * v_wheel_ref) + ((1.0f - alpha_wheel) * v_imu_pred);
        if (v_wheel_ref < 0.05f && fabsf(sensors->ax_mss) < 0.2f && !front_axle_locked) {
            safety_ctx->estimated_vx_ms = 0.0f;
        }
    }

    safety_ctx->estimated_vx_ms = clampf(safety_ctx->estimated_vx_ms, 0.0f, MAX_SPEED_MS * 1.25f);
    sensors->vehicle_speed_ms   = safety_ctx->estimated_vx_ms;
    return safety_ctx->estimated_vx_ms;
}

void VehicleControl_ApplyPowerAndThermalLimits(VehicleSafetyContext_t *safety_ctx,
                                               const DriverInputs_t *driver,
                                               const SensorData_t *sensors,
                                               MotorCommands_t *commands) {
    if (!safety_ctx || !driver || !sensors || !commands) return;

    if (!driver->safety_plausibility_ok ||
        (safety_ctx->fault_flags & (FAULT_SENSOR_NAN | FAULT_APPS_DISCREPANCY |
                                    FAULT_APPS_OUT_OF_RANGE | FAULT_BRAKE_OUT_OF_RANGE |
                                    FAULT_CAN_TIMEOUT)) != 0U) {
        SetSafeMotorCommands(commands);
        return;
    }

    commands->target_rpm_left  = clampf(commands->target_rpm_left, 0.0f, MOTOR_MAX_RPM);
    commands->target_rpm_right = clampf(commands->target_rpm_right, 0.0f, MOTOR_MAX_RPM);

    /* Derating térmico de motores e inversores */
    float max_motor_temp = fmaxf(sensors->motor_temp_left_c, sensors->motor_temp_right_c);
    float max_inv_temp   = fmaxf(sensors->inverter_temp_left_c, sensors->inverter_temp_right_c);

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

    /* Supervisão estrita de subtensão do barramento DC (sem mascarar 0.0 V) */
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

    float v_x = fmaxf(0.0f, sensors->vehicle_speed_ms);
    float omega_rl = (fabsf(sensors->wheel_speed_rl_rads) > 0.01f) ?
                     fabsf(sensors->wheel_speed_rl_rads) : (v_x / TIRE_RADIUS_DYNAMIC_M);
    float omega_rr = (fabsf(sensors->wheel_speed_rr_rads) > 0.01f) ?
                     fabsf(sensors->wheel_speed_rr_rads) : (v_x / TIRE_RADIUS_DYNAMIC_M);
    float omega_mot_l = omega_rl * GEAR_RATIO;
    float omega_mot_r = omega_rr * GEAR_RATIO;

    const float max_omega_mot_rads = MOTOR_MAX_RPM * RPM_TO_RADS;
    float max_curr_omega_mot = fmaxf(omega_mot_l, omega_mot_r);

    /* Frenagem regenerativa com rampa contínua até zero e verificação de rotação máxima */
    bool can_regen = (driver->throttle_demand <= 0.001f) &&
                     (driver->raw_pedal_demand <= 0.02f) &&
                     (commands->torque_left_nm <= 0.01f) &&
                     (commands->torque_right_nm <= 0.01f) &&
                     (v_x > REGEN_MIN_SPEED_MS) &&
                     (max_curr_omega_mot < max_omega_mot_rads) &&
                     (v_batt < BATTERY_MAX_REGEN_VOLTAGE_V);

    if (can_regen) {
        float v_rear_avg = 0.5f * (omega_rl + omega_rr) * TIRE_RADIUS_DYNAMIC_M;
        float rear_slip = (v_x - v_rear_avg) / fmaxf(v_x, 1.0f);

        if (rear_slip < 0.15f) {
            float brake_bar = driver->brake_pressure_bar;
            float regen_frac = 0.20f;
            if (brake_bar > 0.5f && brake_bar <= 35.0f) {
                regen_frac = 0.20f + (0.40f * (brake_bar / 35.0f));
            } else if (brake_bar > 35.0f && brake_bar < 65.0f) {
                regen_frac = 0.60f * ((65.0f - brake_bar) / 30.0f);
            } else if (brake_bar >= 65.0f) {
                regen_frac = 0.0f;
            }

            float speed_taper = clampf((v_x - REGEN_MIN_SPEED_MS) / 2.0f, 0.0f, 1.0f);
            float t_regen_each = regen_frac * speed_taper * t_limit;

            if (t_regen_each > 0.05f) {
                float p_regen_max_w  = v_batt * BATTERY_MAX_REGEN_A;
                float p_regen_mec_w  = t_regen_each * (omega_mot_l + omega_mot_r);
                float p_regen_elec_w = p_regen_mec_w * INVERTER_MOTOR_EFFICIENCY;

                if (p_regen_elec_w > p_regen_max_w && p_regen_elec_w > 1.0f) {
                    t_regen_each *= (p_regen_max_w / p_regen_elec_w);
                }

                if (sensors->battery_current_a < -BATTERY_MAX_REGEN_A) {
                    float curr_scale = BATTERY_MAX_REGEN_A / fabsf(sensors->battery_current_a);
                    t_regen_each *= curr_scale;
                }

                t_regen_each = clampf(t_regen_each, 0.0f, t_limit);
                commands->torque_left_nm        = -t_regen_each;
                commands->torque_right_nm       = -t_regen_each;
                commands->target_rpm_left       = 0.0f;
                commands->target_rpm_right      = 0.0f;
                commands->inverter_left_enable  = true;
                commands->inverter_right_enable = true;
                commands->regen_active          = true;
                return;
            }
        }
    }

    commands->regen_active = false;

    float t_left  = commands->torque_left_nm;
    float t_right = commands->torque_right_nm;

    /* Saturação preservando o torque diferencial */
    if (t_left >= t_right && t_left > t_limit) {
        float excess = t_left - t_limit;
        t_left  = t_limit;
        t_right = fmaxf(0.0f, t_right - excess);
    } else if (t_right > t_left && t_right > t_limit) {
        float excess = t_right - t_limit;
        t_right = t_limit;
        t_left  = fmaxf(0.0f, t_left - excess);
    }

    t_left  = clampf(t_left, 0.0f, t_limit);
    t_right = clampf(t_right, 0.0f, t_limit);

    /* Governador de velocidade e rotação máxima */
    if (v_x >= MAX_SPEED_MS || max_curr_omega_mot >= max_omega_mot_rads) {
        float speed_over = (v_x - MAX_SPEED_MS) / (0.02f * MAX_SPEED_MS);
        float rpm_over   = (max_curr_omega_mot - max_omega_mot_rads) / (0.02f * max_omega_mot_rads);
        float over_frac  = fmaxf(speed_over, rpm_over);
        float gov_scale  = clampf(1.0f - over_frac, 0.0f, 1.0f);
        t_left  *= gov_scale;
        t_right *= gov_scale;
    }

    /* Limitação de potência elétrica e corrente de descarga */
    float p_mec_w  = (fabsf(t_left) * omega_mot_l) + (fabsf(t_right) * omega_mot_r);
    float p_elec_w = p_mec_w / INVERTER_MOTOR_EFFICIENCY;

    float p_max_sys_w  = SYSTEM_MAX_POWER_KW * 1000.0f;
    float p_max_batt_w = v_batt * BATTERY_MAX_DISCHARGE_A;
    float p_elec_lim_w = fminf(p_max_sys_w, p_max_batt_w);

    if (p_elec_w > p_elec_lim_w && p_elec_w > 1.0f) {
        float power_scale = p_elec_lim_w / p_elec_w;
        t_left  *= power_scale;
        t_right *= power_scale;
    }

    if (sensors->battery_current_a > BATTERY_MAX_DISCHARGE_A && sensors->battery_current_a > 1.0f) {
        float curr_scale = BATTERY_MAX_DISCHARGE_A / sensors->battery_current_a;
        t_left  *= curr_scale;
        t_right *= curr_scale;
    }

    commands->torque_left_nm  = clampf(t_left, 0.0f, t_limit);
    commands->torque_right_nm = clampf(t_right, 0.0f, t_limit);
}

void VehicleControl_Step(VehicleSafetyContext_t *safety_ctx,
                         ElectronicDiffConfig_t *diff_cfg,
                         LaunchControlContext_t *lc_ctx,
                         DriverInputs_t *driver,
                         SensorData_t *sensors,
                         MotorCommands_t *commands,
                         float dt_s) {
    if (!safety_ctx || !diff_cfg || !driver || !sensors || !commands) return;

    if (!isfinite(dt_s) || dt_s <= 0.0f) {
        safety_ctx->fault_flags |= FAULT_SENSOR_NAN;
        driver->safety_plausibility_ok = false;
        driver->raw_pedal_demand       = 0.0f;
        driver->throttle_demand        = 0.0f;
        SetSafeMotorCommands(commands);
        return;
    }

    LaunchControlState_t prev_lc_state = lc_ctx ? lc_ctx->state : LC_STATE_DISARMED;

    bool inputs_ok = VehicleControl_ValidateInputs(safety_ctx, driver, dt_s);
    uint32_t input_nan_fault = safety_ctx->fault_flags & FAULT_SENSOR_NAN;
    bool sensors_ok = VehicleControl_ValidateSensors(safety_ctx, sensors, dt_s);
    safety_ctx->fault_flags |= input_nan_fault;

    if (!inputs_ok || !sensors_ok) {
        driver->safety_plausibility_ok = false;
        driver->throttle_demand        = 0.0f;
    }

    if (sensors_ok) {
        VehicleControl_EstimateChassisSpeed(safety_ctx, driver, sensors, dt_s);
    }

    bool brake_applied = (driver->brake_pressure_bar > BSPD_BRAKE_THRESHOLD_BAR) ||
                         driver->brake_switch_active;

    /* Destrava BSPD durante transição de largada válida */
    if (lc_ctx && driver->safety_plausibility_ok) {
        if ((prev_lc_state == LC_STATE_STAGING && !brake_applied && lc_ctx->staging_timer_s < LC_MAX_STAGING_TIME_S) ||
            prev_lc_state == LC_STATE_LAUNCHING ||
            prev_lc_state == LC_STATE_COMPLETED) {
            if (!brake_applied) {
                safety_ctx->bspd_latch_tripped = false;
                safety_ctx->fault_flags &= ~FAULT_BSPD_TRIPPED;
                driver->throttle_demand = driver->raw_pedal_demand;
            }
        }
    }

    ElectronicDiff_Update(diff_cfg, driver, sensors, commands, dt_s);

    if (lc_ctx) {
        (void)LaunchControl_Update(lc_ctx, driver, sensors, commands, dt_s);

        if (lc_ctx->state == LC_STATE_STAGING) {
            safety_ctx->fault_flags &= ~FAULT_BSPD_TRIPPED;
        } else if (lc_ctx->state == LC_STATE_LAUNCHING && !brake_applied) {
            safety_ctx->bspd_latch_tripped = false;
            safety_ctx->fault_flags &= ~FAULT_BSPD_TRIPPED;
            driver->throttle_demand = driver->raw_pedal_demand;
        } else if (lc_ctx->state == LC_STATE_ABORTED &&
                   (prev_lc_state == LC_STATE_STAGING ||
                    prev_lc_state == LC_STATE_LAUNCHING ||
                    prev_lc_state == LC_STATE_COMPLETED)) {
            if (brake_applied && driver->raw_pedal_demand > 0.25f) {
                safety_ctx->bspd_latch_tripped = true;
                safety_ctx->fault_flags |= FAULT_BSPD_TRIPPED;
                driver->throttle_demand = 0.0f;
            }
        }
    }

    VehicleControl_ApplyPowerAndThermalLimits(safety_ctx, driver, sensors, commands);
}
