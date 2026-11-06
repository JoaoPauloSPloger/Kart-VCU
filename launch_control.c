/*
 * launch_control.c - Controle de largada do kart eletrico
 * FSM de 6 estados, estimativa de carga normal dinamica e malha PI de slip ratio
 */

#include "launch_control.h"

void LaunchControl_Init(LaunchControlContext_t *ctx) {
    if (!ctx) {
        return;
    }
    ctx->state = LC_STATE_DISARMED;
    ctx->target_slip = LC_TARGET_SLIP_DEFAULT;
    ctx->kp_slip = LC_KP_SLIP_DEFAULT;
    ctx->ki_slip = LC_KI_SLIP_DEFAULT;
    ctx->integral_slip_error = 0.0f;
    ctx->current_torque_output = 0.0f;
    ctx->launch_timer_s = 0.0f;
    ctx->staging_timer_s = 0.0f;
    ctx->ax_filtered_mss = 0.0f;
    ctx->transition_blend = 0.0f;
}

bool LaunchControl_Update(LaunchControlContext_t *ctx,
                          const DriverInputs_t *driver,
                          const SensorData_t *sensors,
                          const VehicleSafetyContext_t *safety_ctx,
                          MotorCommands_t *commands,
                          float dt_s) {
    if (!ctx || !driver || !sensors || !safety_ctx || !commands || !isfinite(dt_s) || dt_s <= 0.0f) {
        return false;
    }

    float v_front_l = sensors->wheel_speed_fl_rads * FRONT_TIRE_RADIUS_DYNAMIC_M;
    float v_front_r = sensors->wheel_speed_fr_rads * FRONT_TIRE_RADIUS_DYNAMIC_M;
    float v_front = 0.5f * (v_front_l + v_front_r);
    float v_chassi = (safety_ctx->estimated_vx_ms > 0.01f) ? safety_ctx->estimated_vx_ms : v_front;
    if (v_chassi < 0.0f || !isfinite(v_chassi)) {
        v_chassi = 0.0f;
    }

    float v_rear_l = sensors->wheel_speed_rl_rads * TIRE_RADIUS_DYNAMIC_M;
    float v_rear_r = sensors->wheel_speed_rr_rads * TIRE_RADIUS_DYNAMIC_M;
    float v_rear = 0.5f * (v_rear_l + v_rear_r);
    if (v_rear < 0.0f || !isfinite(v_rear)) {
        v_rear = 0.0f;
    }

    /* Filtro IIR de primeira ordem para aceleracao longitudinal */
    float alpha_ax = LC_AX_FILTER_TAU_S / (LC_AX_FILTER_TAU_S + dt_s);
    if (isfinite(sensors->ax_mss)) {
        ctx->ax_filtered_mss = (alpha_ax * ctx->ax_filtered_mss) + ((1.0f - alpha_ax) * sensors->ax_mss);
    }

    float pedal_demand = (safety_ctx->raw_pedal_demand > safety_ctx->throttle_demand) ?
                         safety_ctx->raw_pedal_demand : safety_ctx->throttle_demand;
    if (!safety_ctx->safety_plausibility_ok) {
        pedal_demand = 0.0f;
    }

    if (!safety_ctx->safety_plausibility_ok || fabsf(driver->steering_angle_rad) > LC_MAX_STEERING_RAD) {
        if (ctx->state != LC_STATE_DISARMED) {
            ctx->state = LC_STATE_ABORTED;
            ctx->current_torque_output = 0.0f;
        }
    }

    bool brake_applied = (driver->brake_pressure_bar > LC_ARMED_BRAKE_THRESHOLD_BAR) || driver->brake_switch_active;

    switch (ctx->state) {
        case LC_STATE_DISARMED:
            if (safety_ctx->safety_plausibility_ok &&
                driver->launch_button_pressed &&
                v_chassi < LC_MAX_CHASSIS_SPEED_ARM_MS &&
                driver->brake_pressure_bar > LC_ARMED_BRAKE_THRESHOLD_BAR &&
                fabsf(driver->steering_angle_rad) <= LC_MAX_STEERING_RAD) {
                ctx->state = LC_STATE_ARMED;
                ctx->integral_slip_error = 0.0f;
                ctx->current_torque_output = 0.0f;
                ctx->launch_timer_s = 0.0f;
                ctx->staging_timer_s = 0.0f;
                ctx->ax_filtered_mss = 0.0f;
                ctx->transition_blend = 0.0f;
            }
            break;

        case LC_STATE_ARMED:
            if (pedal_demand > LC_STAGING_PEDAL_THRESHOLD &&
                driver->brake_pressure_bar > LC_ARMED_BRAKE_THRESHOLD_BAR &&
                v_chassi < LC_MAX_CHASSIS_SPEED_ARM_MS) {
                ctx->state = LC_STATE_STAGING;
                ctx->staging_timer_s = 0.0f;
            } else if ((driver->brake_pressure_bar < LC_BRAKE_DISARM_THRESHOLD_BAR && !driver->brake_switch_active) ||
                       v_chassi >= LC_MAX_CHASSIS_SPEED_ARM_MS) {
                ctx->state = LC_STATE_DISARMED;
            }
            break;

        case LC_STATE_STAGING:
            ctx->staging_timer_s += dt_s;
            ctx->current_torque_output = LC_PRELOAD_TORQUE_NM;

            if (ctx->staging_timer_s >= LC_MAX_STAGING_TIME_S) {
                ctx->state = LC_STATE_ABORTED;
                ctx->current_torque_output = 0.0f;
            } else if (driver->brake_pressure_bar < LC_BRAKE_DISARM_THRESHOLD_BAR && !driver->brake_switch_active) {
                ctx->state = LC_STATE_LAUNCHING;
                ctx->launch_timer_s = 0.0f;
                ctx->integral_slip_error = 0.0f;
            } else if (pedal_demand < LC_STAGING_ABORT_PEDAL) {
                ctx->state = LC_STATE_ABORTED;
                ctx->current_torque_output = 0.0f;
            }
            break;

        case LC_STATE_LAUNCHING: {
            ctx->launch_timer_s += dt_s;

            if (brake_applied) {
                ctx->state = LC_STATE_ABORTED;
                ctx->current_torque_output = 0.0f;
                break;
            }

            if (pedal_demand < 0.05f) {
                ctx->state = LC_STATE_DISARMED;
                ctx->current_torque_output = 0.0f;
                ctx->transition_blend = 1.0f;
                break;
            }

            /* SAE J670: Slip ratio normalizado kappa = (v_wheel - vx) / max(v_wheel, vx, 1.0) */
            float v_ref = fmaxf(fmaxf(v_rear, v_chassi), 1.0f);
            float current_slip = fmaxf(0.0f, (v_rear - v_chassi) / v_ref);

            /* Estimativa de forca normal traseira com transferencia de peso longitudinal */
            float ax_used = fmaxf(0.0f, ctx->ax_filtered_mss);
            const float static_weight = TOTAL_VEHICLE_MASS_KG * GRAVITY_MSS * WEIGHT_DIST_REAR;
            const float transfer_coeff = TOTAL_VEHICLE_MASS_KG * (CG_HEIGHT_M / WHEELBASE_M);
            float n_rear_din = static_weight + (ax_used * transfer_coeff);

            float f_trac_max = TIRE_PEAK_FRICTION_COEFF * n_rear_din;
            float t_wheel_max = f_trac_max * TIRE_RADIUS_DYNAMIC_M;
            const float wheel_to_motor = 1.0f / (2.0f * GEAR_RATIO * TRANSMISSION_EFFICIENCY);
            float t_motor_fric_lim = clampf(t_wheel_max * wheel_to_motor, 5.0f, MOTOR_MAX_TORQUE_NM);

            float slip_error = ctx->target_slip - current_slip;
            float max_step = LC_MAX_RAMP_RATE_NM_PER_S * dt_s;
            bool ramping_up = (ctx->current_torque_output + max_step) < t_motor_fric_lim;

            if (!ramping_up || slip_error < 0.0f) {
                ctx->integral_slip_error += slip_error * dt_s;
                ctx->integral_slip_error = clampf(ctx->integral_slip_error, -LC_INTEGRAL_SLIP_MAX, LC_INTEGRAL_SLIP_MAX);
            }

            float delta_t_pi = (ctx->kp_slip * slip_error) + (ctx->ki_slip * ctx->integral_slip_error);
            float target_torque = clampf(t_motor_fric_lim + delta_t_pi, 0.0f, pedal_demand * MOTOR_MAX_TORQUE_NM);

            if (target_torque > ctx->current_torque_output + max_step) {
                ctx->current_torque_output += max_step;
            } else if (target_torque < ctx->current_torque_output - max_step) {
                ctx->current_torque_output -= max_step;
            } else {
                ctx->current_torque_output = target_torque;
            }

            if (v_chassi >= LC_COMPLETION_SPEED_MS || ctx->launch_timer_s > LC_TIMEOUT_LAUNCH_S || pedal_demand < 0.50f) {
                ctx->state = LC_STATE_COMPLETED;
                ctx->transition_blend = 0.0f;
            }
            break;
        }

        case LC_STATE_COMPLETED:
            if (brake_applied) {
                ctx->state = LC_STATE_ABORTED;
                ctx->current_torque_output = 0.0f;
                break;
            }
            if (pedal_demand < 0.05f) {
                ctx->transition_blend = 1.0f;
                ctx->current_torque_output = 0.0f;
                ctx->state = LC_STATE_DISARMED;
                break;
            }
            ctx->transition_blend += dt_s / LC_COMPLETION_BLEND_TIME_S;
            if (ctx->transition_blend >= 1.0f) {
                ctx->transition_blend = 1.0f;
                ctx->state = LC_STATE_DISARMED;
            }
            break;

        case LC_STATE_ABORTED:
            ctx->current_torque_output = 0.0f;
            if (pedal_demand < 0.05f && !driver->launch_button_pressed && !brake_applied) {
                ctx->state = LC_STATE_DISARMED;
                ctx->staging_timer_s = 0.0f;
            }
            break;

        default:
            ctx->state = LC_STATE_DISARMED;
            break;
    }

    if (ctx->state == LC_STATE_STAGING) {
        commands->torque_left_nm = LC_PRELOAD_TORQUE_NM;
        commands->torque_right_nm = LC_PRELOAD_TORQUE_NM;
        commands->target_rpm_left = LC_PRELOAD_RPM;
        commands->target_rpm_right = LC_PRELOAD_RPM;
        commands->inverter_left_enable = true;
        commands->inverter_right_enable = true;
        commands->regen_active = false;
        return true;
    } else if (ctx->state == LC_STATE_LAUNCHING) {
        /* Amortecimento de guinada baseado no erro em relacao a referencia cinematica */
        float yaw_des = (v_chassi / WHEELBASE_M) * tanf(driver->steering_angle_rad);
        float yaw_error = yaw_des - sensors->yaw_rate_rads;
        float delta_yaw_torque = LC_YAW_DAMPING_GAIN * yaw_error;
        delta_yaw_torque = clampf(delta_yaw_torque, -6.0f, 6.0f);

        float t_cmd_l = ctx->current_torque_output;
        float t_cmd_r = ctx->current_torque_output;

        if (delta_yaw_torque > 0.0f) {
            t_cmd_r = clampf(ctx->current_torque_output - delta_yaw_torque, 0.0f, ctx->current_torque_output);
        } else if (delta_yaw_torque < 0.0f) {
            t_cmd_l = clampf(ctx->current_torque_output + delta_yaw_torque, 0.0f, ctx->current_torque_output);
        }

        commands->torque_left_nm = t_cmd_l;
        commands->torque_right_nm = t_cmd_r;
        commands->target_rpm_left = MOTOR_MAX_RPM;
        commands->target_rpm_right = MOTOR_MAX_RPM;
        commands->inverter_left_enable = true;
        commands->inverter_right_enable = true;
        commands->regen_active = false;
        return true;
    } else if (ctx->state == LC_STATE_COMPLETED && ctx->transition_blend < 1.0f) {
        float max_demand_torque = pedal_demand * MOTOR_MAX_TORQUE_NM;
        float blend = clampf(ctx->transition_blend, 0.0f, 1.0f);
        float blended_t_left = ((1.0f - blend) * ctx->current_torque_output) + (blend * commands->torque_left_nm);
        float blended_t_right = ((1.0f - blend) * ctx->current_torque_output) + (blend * commands->torque_right_nm);

        commands->torque_left_nm = clampf(blended_t_left, 0.0f, max_demand_torque);
        commands->torque_right_nm = clampf(blended_t_right, 0.0f, max_demand_torque);
        commands->regen_active = false;
        return true;
    } else if (ctx->state == LC_STATE_ABORTED) {
        commands->torque_left_nm = 0.0f;
        commands->torque_right_nm = 0.0f;
        commands->target_rpm_left = 0.0f;
        commands->target_rpm_right = 0.0f;
        commands->inverter_left_enable = false;
        commands->inverter_right_enable = false;
        commands->regen_active = false;
        return true;
    }

    return false;
}
