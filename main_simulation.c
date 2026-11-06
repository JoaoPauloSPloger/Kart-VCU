/*
 * main_simulation.c - Bancada HIL e suite de validacao da VCU
 * Formula Tesla UFMG - Testes de bancada, dinamica veicular e Monte Carlo
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>
#include "vehicle_control.h"
#include "electronic_differential.h"
#include "launch_control.h"

#define TEST_ASSERT(cond) do { \
    if (!(cond)) { \
        fprintf(stderr, "\n[FALHA DE ASSERT] %s (%s:%d)\n", #cond, __FILE__, __LINE__); \
        exit(1); \
    } \
} while (0)

static float RandUniform(float min_val, float max_val) {
    float unit = (float)rand() / (float)RAND_MAX;
    return min_val + (unit * (max_val - min_val));
}

static float RandGaussian(float mean, float std_dev) {
    float u1 = ((float)rand() + 1.0f) / ((float)RAND_MAX + 1.0f);
    float u2 = ((float)rand() + 1.0f) / ((float)RAND_MAX + 1.0f);
    float z0 = sqrtf(-2.0f * logf(u1)) * cosf(2.0f * (float)M_PI * u2);
    return mean + (z0 * std_dev);
}

typedef struct {
    float mass_kg;
    float wheelbase_m;
    float rear_track_m;
    float cg_height_m;
    float weight_dist_rear;
    float frontal_area_m2;
    float drag_coeff_cd;
    float rolling_res_crr;
    float mu_peak;
    float j_eq_axle_kgm2;
    float batt_open_circuit_v;
    float batt_internal_res_ohm;
    float motor_temp_init_c;
    float inverter_temp_init_c;
} RandomizedPlantParams_t;

static void SampleRandomPlantParams(RandomizedPlantParams_t *p) {
    p->mass_kg = RandUniform(190.0f, 250.0f);
    p->wheelbase_m = RandUniform(0.935f, 0.975f);
    p->rear_track_m = RandUniform(1.060f, 1.140f);
    p->cg_height_m = RandUniform(0.195f, 0.245f);
    p->weight_dist_rear = RandUniform(0.560f, 0.640f);
    p->frontal_area_m2 = RandUniform(0.390f, 0.460f);
    p->drag_coeff_cd = RandUniform(0.630f, 0.770f);
    p->rolling_res_crr = RandUniform(0.016f, 0.028f);
    p->mu_peak = RandUniform(0.680f, 0.960f);
    p->j_eq_axle_kgm2 = 2.0f * RandUniform(0.095f, 0.130f);
    p->batt_open_circuit_v = RandUniform(68.0f, 79.5f);
    p->batt_internal_res_ohm = RandUniform(0.020f, 0.042f);
    p->motor_temp_init_c = RandUniform(28.0f, 75.0f);
    p->inverter_temp_init_c = RandUniform(25.0f, 52.0f);
}

static float TireFrictionCurve_Mu(float slip_ratio, float mu_peak) {
    if (slip_ratio <= 0.0f) {
        return 0.0f;
    }
    const float lambda_peak = 0.12f;
    float u = slip_ratio / lambda_peak;
    float mu_slide = 0.78f * mu_peak;
    if (u <= 1.0f) {
        return mu_peak * ((2.0f * u) / (1.0f + (u * u)));
    }
    return mu_slide + (mu_peak - mu_slide) * expf(-0.8f * (u - 1.0f));
}

void RunCorneringTest(void) {
    printf("--- Teste 1: Diferencial Eletronico e Vetorizacao de Torque ---\n");

    for (int turn_idx = 0; turn_idx < 2; turn_idx++) {
        float turn_sign = (turn_idx == 0) ? 1.0f : -1.0f;
        const char *turn_name = (turn_sign > 0.0f) ? "Direita" : "Esquerda";

        VehicleSafetyContext_t safety_ctx;
        ElectronicDiffParams_t diff_params;
        ElectronicDiffState_t diff_state;
        LaunchControlContext_t lc_ctx;

        VehicleControl_Init(&safety_ctx);
        ElectronicDiff_InitParams(&diff_params);
        ElectronicDiff_InitState(&diff_state);
        LaunchControl_Init(&lc_ctx);

        DriverInputs_t driver = {0};
        SensorData_t sensors = {0};
        MotorCommands_t commands = {0};

        float dt = RandUniform(0.005f, 0.015f);
        float v_target = RandUniform(7.5f, 16.5f);
        float r_curve = RandUniform(8.0f, 22.0f);
        float throttle_pct = RandUniform(40.0f, 78.0f);
        float apps_offset = RandUniform(-1.5f, 1.5f);
        float understeer_ratio = RandUniform(0.70f, 0.88f);
        float delta_rad = turn_sign * atanf(WHEELBASE_M / r_curve);

        float denom = WHEELBASE_M + ((diff_params.understeer_coeff * v_target * v_target) / GRAVITY_MSS);
        float yaw_rate_des = (v_target / denom) * tanf(delta_rad);

        driver.apps1_percent = throttle_pct;
        driver.apps2_percent = throttle_pct + apps_offset;
        driver.steering_angle_rad = delta_rad;

        sensors.yaw_rate_rads = understeer_ratio * yaw_rate_des;
        sensors.wheel_speed_fl_rads = v_target / FRONT_TIRE_RADIUS_DYNAMIC_M;
        sensors.wheel_speed_fr_rads = v_target / FRONT_TIRE_RADIUS_DYNAMIC_M;
        sensors.wheel_speed_rl_rads = (v_target * (1.0f + (turn_sign * REAR_TRACK_M / (2.0f * r_curve)))) / TIRE_RADIUS_DYNAMIC_M;
        sensors.wheel_speed_rr_rads = (v_target * (1.0f - (turn_sign * REAR_TRACK_M / (2.0f * r_curve)))) / TIRE_RADIUS_DYNAMIC_M;
        sensors.motor_temp_left_c = RandUniform(35.0f, 82.0f);
        sensors.motor_temp_right_c = RandUniform(35.0f, 82.0f);
        sensors.inverter_temp_left_c = RandUniform(30.0f, 58.0f);
        sensors.inverter_temp_right_c = RandUniform(30.0f, 58.0f);
        sensors.battery_voltage_v = RandUniform(66.5f, 79.0f);
        sensors.battery_current_a = RandUniform(45.0f, 130.0f);

        /* Integracao em malha fechada do momento polar de guinada: I_z * d(wz)/dt = M_diff + M_lat - C_psi * wz */
        const float I_z = 45.0f;
        const float C_psi = 85.0f;

        for (int i = 0; i < 40; i++) {
            sensors.can_heartbeat_counter++;
            VehicleControl_Step(&safety_ctx, &diff_params, &diff_state, &lc_ctx, &driver, &sensors, &commands, dt);

            float t_wheel_l = commands.torque_left_nm * GEAR_RATIO * TRANSMISSION_EFFICIENCY;
            float t_wheel_r = commands.torque_right_nm * GEAR_RATIO * TRANSMISSION_EFFICIENCY;
            float delta_f_x = (t_wheel_l - t_wheel_r) / TIRE_RADIUS_DYNAMIC_M;
            float m_diff = delta_f_x * (REAR_TRACK_M * 0.5f);
            float m_lateral = 120.0f * (yaw_rate_des - sensors.yaw_rate_rads);
            float m_z = m_diff + m_lateral - (C_psi * sensors.yaw_rate_rads);
            float yaw_accel = m_z / I_z;

            sensors.yaw_rate_rads += yaw_accel * dt;
        }

        TEST_ASSERT(commands.inverter_left_enable && commands.inverter_right_enable);
        if (turn_sign > 0.0f) {
            TEST_ASSERT(commands.target_rpm_left > commands.target_rpm_right);
            TEST_ASSERT(commands.torque_left_nm > commands.torque_right_nm);
        } else {
            TEST_ASSERT(commands.target_rpm_right > commands.target_rpm_left);
            TEST_ASSERT(commands.torque_right_nm > commands.torque_left_nm);
        }

        printf("Curva para %s: R=%.1fm, v=%.1f km/h, esterco=%.1f deg, delta_T=%.2f N.m\n",
               turn_name, r_curve, safety_ctx.estimated_vx_ms * 3.6f, driver.steering_angle_rad * RAD_TO_DEG,
               commands.torque_left_nm - commands.torque_right_nm);

        /* Verificacao de derating termico em aceleracao plena */
        driver.apps1_percent = 100.0f;
        driver.apps2_percent = 100.0f;
        sensors.wheel_speed_rl_rads = 18.0f;
        sensors.wheel_speed_rr_rads = 18.0f;
        sensors.motor_temp_left_c = RandUniform(98.0f, 112.0f);
        float expected_t_lim = MOTOR_MAX_TORQUE_NM *
                               ((MOTOR_TEMP_CUTOFF_C - fmaxf(sensors.motor_temp_left_c, sensors.motor_temp_right_c)) /
                                (MOTOR_TEMP_CUTOFF_C - MOTOR_TEMP_DERATING_START_C));
        sensors.can_heartbeat_counter++;
        VehicleControl_Step(&safety_ctx, &diff_params, &diff_state, &lc_ctx, &driver, &sensors, &commands, dt);

        if (turn_sign > 0.0f) {
            TEST_ASSERT(commands.torque_left_nm > commands.torque_right_nm);
            TEST_ASSERT(fabsf(commands.torque_left_nm - expected_t_lim) < 1e-2f);
        } else {
            TEST_ASSERT(commands.torque_right_nm > commands.torque_left_nm);
            TEST_ASSERT(fabsf(commands.torque_right_nm - expected_t_lim) < 1e-2f);
        }
    }
    printf("Diferencial e vetorizacao validados.\n\n");
}

void RunLaunchControlSimulation(void) {
    printf("--- Teste 2: Simulacao Dinamica de Largada ---\n");

    RandomizedPlantParams_t plant;
    SampleRandomPlantParams(&plant);

    VehicleSafetyContext_t safety_ctx;
    ElectronicDiffParams_t diff_params;
    ElectronicDiffState_t diff_state;
    LaunchControlContext_t lc;

    VehicleControl_Init(&safety_ctx);
    ElectronicDiff_InitParams(&diff_params);
    ElectronicDiff_InitState(&diff_state);
    LaunchControl_Init(&lc);

    DriverInputs_t driver = {0};
    SensorData_t sensors = {0};
    MotorCommands_t commands = {0};

    float dt = 0.005f;
    float t = 0.0f;
    float v_x = 0.0f;
    float s = 0.0f;

    /* Dinamica independente para roda traseira esquerda e direita (split-mu) */
    float omega_rl = 0.0f;
    float omega_rr = 0.0f;
    float j_wheel_kgm2 = plant.j_eq_axle_kgm2 * 0.5f;
    float mu_peak_l = plant.mu_peak;
    float mu_peak_r = plant.mu_peak * RandUniform(0.94f, 1.0f);

    sensors.motor_temp_left_c = plant.motor_temp_init_c;
    sensors.motor_temp_right_c = plant.motor_temp_init_c + RandUniform(-1.5f, 1.5f);
    sensors.inverter_temp_left_c = plant.inverter_temp_init_c;
    sensors.inverter_temp_right_c = plant.inverter_temp_init_c + RandUniform(-1.0f, 1.0f);
    sensors.battery_voltage_v = plant.batt_open_circuit_v;
    sensors.can_heartbeat_counter = 1U;

    float staging_brake_bar = RandUniform(18.0f, 55.0f);
    float staging_hold_time_s = RandUniform(0.15f, 1.80f);
    int staging_steps = (int)(staging_hold_time_s / dt);

    driver.launch_button_pressed = true;
    driver.brake_pressure_bar = staging_brake_bar;
    driver.brake_switch_active = true;
    driver.apps1_percent = 0.0f;
    driver.apps2_percent = 0.0f;
    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_params, &diff_state, &lc, &driver, &sensors, &commands, dt);
    TEST_ASSERT(lc.state == LC_STATE_ARMED);

    float launch_pedal_pct = RandUniform(88.0f, 100.0f);
    driver.apps1_percent = launch_pedal_pct;
    driver.apps2_percent = launch_pedal_pct;
    for (int st = 0; st < staging_steps; st++) {
        sensors.can_heartbeat_counter++;
        VehicleControl_Step(&safety_ctx, &diff_params, &diff_state, &lc, &driver, &sensors, &commands, dt);
        TEST_ASSERT(lc.state == LC_STATE_STAGING);
        TEST_ASSERT(fabsf(commands.torque_left_nm - LC_PRELOAD_TORQUE_NM) < 1e-3f);
    }

    driver.apps1_percent = 100.0f;
    driver.apps2_percent = 100.0f;
    driver.brake_pressure_bar = 0.0f;
    driver.brake_switch_active = false;
    driver.launch_button_pressed = false;

    printf("Tempo(s) | Vel(km/h) | Dist(m) | Torque(N.m) | V_bat(V) | Slip(%%) | Estado\n");
    printf("-----------------------------------------------------------------------\n");

    const int substeps = 10;
    const float dt_sub = dt / (float)substeps;
    int print_divider = 0;
    float a_x = 0.0f;

    while (v_x < (60.0f / 3.6f) && t < 7.0f) {
        float enc_noise_f = RandGaussian(0.0f, 0.03f);
        float enc_noise_rl = RandGaussian(0.0f, 0.03f);
        float enc_noise_rr = RandGaussian(0.0f, 0.03f);
        float imu_noise = RandGaussian(0.0f, 0.08f);

        sensors.wheel_speed_fl_rads = fmaxf(0.0f, (v_x / FRONT_TIRE_RADIUS_DYNAMIC_M) + enc_noise_f);
        sensors.wheel_speed_fr_rads = fmaxf(0.0f, (v_x / FRONT_TIRE_RADIUS_DYNAMIC_M) - enc_noise_f);
        sensors.wheel_speed_rl_rads = fmaxf(0.0f, omega_rl + enc_noise_rl);
        sensors.wheel_speed_rr_rads = fmaxf(0.0f, omega_rr - enc_noise_rr);
        sensors.ax_mss = a_x + imu_noise;
        sensors.can_heartbeat_counter++;

        VehicleControl_Step(&safety_ctx, &diff_params, &diff_state, &lc, &driver, &sensors, &commands, dt);

        float omega_mot_l = omega_rl * GEAR_RATIO;
        float omega_mot_r = omega_rr * GEAR_RATIO;
        float p_mec_w = (commands.torque_left_nm * omega_mot_l) + (commands.torque_right_nm * omega_mot_r);
        float p_elec_w = (p_mec_w > 0.0f) ? (p_mec_w / INVERTER_MOTOR_EFFICIENCY) : 0.0f;
        float i_batt_a = p_elec_w / fmaxf(sensors.battery_voltage_v, 48.0f);
        sensors.battery_current_a = i_batt_a;
        sensors.battery_voltage_v = clampf(plant.batt_open_circuit_v - (i_batt_a * plant.batt_internal_res_ohm),
                                           64.5f, 81.5f);

        float norm_curr_l = (commands.torque_left_nm / MOTOR_MAX_TORQUE_NM);
        float norm_curr_r = (commands.torque_right_nm / MOTOR_MAX_TORQUE_NM);
        sensors.motor_temp_left_c += (1.8f * norm_curr_l * norm_curr_l) * dt;
        sensors.motor_temp_right_c += (1.8f * norm_curr_r * norm_curr_r) * dt;
        sensors.inverter_temp_left_c += (1.2f * norm_curr_l * norm_curr_l) * dt;
        sensors.inverter_temp_right_c += (1.2f * norm_curr_r * norm_curr_r) * dt;

        float t_axle_l = commands.torque_left_nm * GEAR_RATIO * TRANSMISSION_EFFICIENCY;
        float t_axle_r = commands.torque_right_nm * GEAR_RATIO * TRANSMISSION_EFFICIENCY;

        for (int k = 0; k < substeps; k++) {
            float v_tang_l = omega_rl * TIRE_RADIUS_DYNAMIC_M;
            float v_tang_r = omega_rr * TIRE_RADIUS_DYNAMIC_M;

            float v_ref_l = fmaxf(fmaxf(v_tang_l, v_x), 1.0f);
            float v_ref_r = fmaxf(fmaxf(v_tang_r, v_x), 1.0f);

            float slip_l = fmaxf(0.0f, (v_tang_l - v_x) / v_ref_l);
            float slip_r = fmaxf(0.0f, (v_tang_r - v_x) / v_ref_r);

            float n_rear_total = (plant.mass_kg * GRAVITY_MSS * plant.weight_dist_rear) +
                                 (plant.mass_kg * fmaxf(0.0f, a_x) * (plant.cg_height_m / plant.wheelbase_m));
            float n_rl = n_rear_total * 0.5f;
            float n_rr = n_rear_total * 0.5f;

            float mu_curr_l = TireFrictionCurve_Mu(slip_l, mu_peak_l);
            float mu_curr_r = TireFrictionCurve_Mu(slip_r, mu_peak_r);

            float f_trac_l = mu_curr_l * n_rl;
            float f_trac_r = mu_curr_r * n_rr;

            float d_omega_rl = (t_axle_l - (f_trac_l * TIRE_RADIUS_DYNAMIC_M)) / j_wheel_kgm2;
            float d_omega_rr = (t_axle_r - (f_trac_r * TIRE_RADIUS_DYNAMIC_M)) / j_wheel_kgm2;

            omega_rl += d_omega_rl * dt_sub;
            omega_rr += d_omega_rr * dt_sub;
            if (omega_rl < 0.0f) {
                omega_rl = 0.0f;
            }
            if (omega_rr < 0.0f) {
                omega_rr = 0.0f;
            }

            float f_trac_total = f_trac_l + f_trac_r;
            float f_res = (plant.rolling_res_crr * plant.mass_kg * GRAVITY_MSS) +
                          (0.5f * AIR_DENSITY_KG_M3 * plant.drag_coeff_cd * plant.frontal_area_m2 * v_x * v_x);

            a_x = (f_trac_total - f_res) / plant.mass_kg;
            if (v_x <= 0.0f && a_x < 0.0f) {
                a_x = 0.0f;
            }

            v_x += a_x * dt_sub;
            s += v_x * dt_sub;
        }

        t += dt;

        float omega_rear_avg = 0.5f * (omega_rl + omega_rr);
        float v_ref_print = fmaxf(fmaxf(omega_rear_avg * TIRE_RADIUS_DYNAMIC_M, v_x), 1.0f);
        float slip_pct = ((omega_rear_avg * TIRE_RADIUS_DYNAMIC_M - v_x) / v_ref_print) * 100.0f;
        if (slip_pct < 0.0f) {
            slip_pct = 0.0f;
        }

        if (print_divider++ % 40 == 0) {
            const char *state_str = (lc.state == LC_STATE_LAUNCHING) ? "LAUNCH" :
                                    (lc.state == LC_STATE_COMPLETED) ? "COMPL" : "NORMAL";
            printf(" %5.2f   |   %5.1f   |  %5.1f  |    %5.2f    |   %5.1f  |  %5.1f  | %s\n",
                   t, v_x * 3.6f, s, commands.torque_left_nm, sensors.battery_voltage_v, slip_pct, state_str);
        }
    }

    TEST_ASSERT(v_x >= (60.0f / 3.6f));
    TEST_ASSERT(t < 6.5f);
    TEST_ASSERT(lc.state == LC_STATE_DISARMED || lc.state == LC_STATE_COMPLETED);

    printf("Tempo 0-60 km/h: %.2f s | Distancia: %.1f m\n", t, s);
    printf("Launch Control validado.\n\n");
}

void RunPlausibilitySafetyTest(void) {
    printf("--- Teste 3: Seguranca Funcional e Protecoes ---\n");

    VehicleSafetyContext_t safety_ctx;
    ElectronicDiffParams_t diff_params;
    ElectronicDiffState_t diff_state;
    LaunchControlContext_t lc_ctx;

    VehicleControl_Init(&safety_ctx);
    ElectronicDiff_InitParams(&diff_params);
    ElectronicDiff_InitState(&diff_state);
    LaunchControl_Init(&lc_ctx);

    DriverInputs_t driver = {0};
    SensorData_t sensors = {0};
    MotorCommands_t commands = {0};
    float dt = 0.01f;

    sensors.battery_voltage_v = RandUniform(68.0f, 78.0f);
    sensors.motor_temp_left_c = RandUniform(35.0f, 65.0f);
    sensors.motor_temp_right_c = RandUniform(35.0f, 65.0f);
    sensors.inverter_temp_left_c = RandUniform(30.0f, 50.0f);
    sensors.inverter_temp_right_c = RandUniform(30.0f, 50.0f);
    sensors.can_heartbeat_counter = 1U;

    /* FSAE EV.4.7: Discrepancia APPS > 10% mantida por 100 ms */
    float apps1_fault = RandUniform(35.0f, 95.0f);
    float discrepancy = RandUniform(12.0f, 32.0f);
    float apps2_fault = fmaxf(0.0f, apps1_fault - discrepancy);
    driver.apps1_percent = apps1_fault;
    driver.apps2_percent = apps2_fault;
    for (int i = 0; i < 15; i++) {
        sensors.can_heartbeat_counter++;
        VehicleControl_Step(&safety_ctx, &diff_params, &diff_state, &lc_ctx, &driver, &sensors, &commands, dt);
    }
    TEST_ASSERT(!safety_ctx.safety_plausibility_ok);
    TEST_ASSERT((safety_ctx.fault_flags & FAULT_APPS_DISCREPANCY) != 0U);
    TEST_ASSERT(commands.torque_left_nm == 0.0f && !commands.inverter_left_enable);

    /* Sensor out of range */
    driver.apps1_percent = RandUniform(105.0f, 150.0f);
    driver.apps2_percent = driver.apps1_percent;
    VehicleControl_ValidateInputs(&safety_ctx, &driver, false, dt);
    TEST_ASSERT((safety_ctx.fault_flags & FAULT_APPS_OUT_OF_RANGE) != 0U);
    TEST_ASSERT(!safety_ctx.safety_plausibility_ok);

    /* FSAE T.4.3: BSPD debounce de 150 ms */
    VehicleControl_Init(&safety_ctx);
    float bspd_apps = RandUniform(30.0f, 98.0f);
    float bspd_brake = RandUniform(18.0f, 85.0f);
    driver.apps1_percent = bspd_apps;
    driver.apps2_percent = bspd_apps;
    driver.brake_pressure_bar = bspd_brake;
    driver.brake_switch_active = true;

    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_params, &diff_state, &lc_ctx, &driver, &sensors, &commands, dt);
    TEST_ASSERT(!safety_ctx.bspd_latch_tripped);

    for (int b = 0; b < 16; b++) {
        sensors.can_heartbeat_counter++;
        VehicleControl_Step(&safety_ctx, &diff_params, &diff_state, &lc_ctx, &driver, &sensors, &commands, dt);
    }
    TEST_ASSERT(safety_ctx.bspd_latch_tripped);
    TEST_ASSERT((safety_ctx.fault_flags & FAULT_BSPD_TRIPPED) != 0U);
    TEST_ASSERT(safety_ctx.throttle_demand == 0.0f && commands.torque_left_nm == 0.0f);

    /* Retencao com pedal > 5% */
    float hold_apps = RandUniform(6.0f, 24.0f);
    driver.brake_pressure_bar = 0.0f;
    driver.brake_switch_active = false;
    driver.apps1_percent = hold_apps;
    driver.apps2_percent = hold_apps;
    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_params, &diff_state, &lc_ctx, &driver, &sensors, &commands, dt);
    TEST_ASSERT(safety_ctx.bspd_latch_tripped && safety_ctx.throttle_demand == 0.0f);

    /* Rearme com pedal < 5% */
    float rec_apps = RandUniform(2.2f, 4.8f);
    driver.apps1_percent = rec_apps;
    driver.apps2_percent = rec_apps;
    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_params, &diff_state, &lc_ctx, &driver, &sensors, &commands, dt);
    TEST_ASSERT(!safety_ctx.bspd_latch_tripped);
    TEST_ASSERT((safety_ctx.fault_flags & FAULT_BSPD_TRIPPED) == 0U);
    TEST_ASSERT(safety_ctx.throttle_demand > 0.0f);

    /* Launch Control Staging: frenagem intencional nao deve acionar BSPD */
    VehicleControl_Init(&safety_ctx);
    LaunchControl_Init(&lc_ctx);
    driver.launch_button_pressed = true;
    driver.brake_pressure_bar = RandUniform(20.0f, 50.0f);
    driver.brake_switch_active = true;
    driver.apps1_percent = 0.0f;
    driver.apps2_percent = 0.0f;
    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_params, &diff_state, &lc_ctx, &driver, &sensors, &commands, dt);
    TEST_ASSERT(lc_ctx.state == LC_STATE_ARMED);

    driver.apps1_percent = RandUniform(80.0f, 100.0f);
    driver.apps2_percent = driver.apps1_percent;
    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_params, &diff_state, &lc_ctx, &driver, &sensors, &commands, dt);
    TEST_ASSERT(lc_ctx.state == LC_STATE_STAGING);
    TEST_ASSERT(!safety_ctx.bspd_latch_tripped);

    /* Timeout de staging (> 3.0s) */
    for (int i = 0; i < 305; i++) {
        sensors.can_heartbeat_counter++;
        VehicleControl_Step(&safety_ctx, &diff_params, &diff_state, &lc_ctx, &driver, &sensors, &commands, dt);
    }
    TEST_ASSERT(lc_ctx.state == LC_STATE_ABORTED);
    TEST_ASSERT(commands.torque_left_nm == 0.0f && !commands.inverter_left_enable);
    TEST_ASSERT((safety_ctx.fault_flags & FAULT_BSPD_TRIPPED) != 0U);

    /* Abortamento por freio em launching */
    VehicleControl_Init(&safety_ctx);
    LaunchControl_Init(&lc_ctx);
    driver.launch_button_pressed = true;
    driver.brake_pressure_bar = 25.0f;
    driver.brake_switch_active = true;
    driver.apps1_percent = 0.0f;
    driver.apps2_percent = 0.0f;
    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_params, &diff_state, &lc_ctx, &driver, &sensors, &commands, dt);

    driver.apps1_percent = 100.0f;
    driver.apps2_percent = 100.0f;
    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_params, &diff_state, &lc_ctx, &driver, &sensors, &commands, dt);

    driver.brake_pressure_bar = 0.0f;
    driver.brake_switch_active = false;
    driver.launch_button_pressed = false;
    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_params, &diff_state, &lc_ctx, &driver, &sensors, &commands, dt);
    TEST_ASSERT(lc_ctx.state == LC_STATE_LAUNCHING);

    driver.brake_pressure_bar = RandUniform(22.0f, 70.0f);
    driver.brake_switch_active = true;
    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_params, &diff_state, &lc_ctx, &driver, &sensors, &commands, dt);
    TEST_ASSERT(lc_ctx.state == LC_STATE_ABORTED);
    TEST_ASSERT(commands.torque_left_nm == 0.0f && !commands.inverter_left_enable);
    TEST_ASSERT((safety_ctx.fault_flags & FAULT_BSPD_TRIPPED) != 0U);

    /* Derating termico de motor e inversor */
    VehicleControl_Init(&safety_ctx);
    LaunchControl_Init(&lc_ctx);
    driver.launch_button_pressed = false;
    driver.brake_pressure_bar = 0.0f;
    driver.brake_switch_active = false;
    driver.apps1_percent = 100.0f;
    driver.apps2_percent = 100.0f;

    float v_test = RandUniform(5.0f, 8.2f);
    sensors.wheel_speed_fl_rads = (v_test / FRONT_TIRE_RADIUS_DYNAMIC_M);
    sensors.wheel_speed_fr_rads = (v_test / FRONT_TIRE_RADIUS_DYNAMIC_M);
    sensors.wheel_speed_rl_rads = (v_test / TIRE_RADIUS_DYNAMIC_M);
    sensors.wheel_speed_rr_rads = (v_test / TIRE_RADIUS_DYNAMIC_M);
    sensors.battery_voltage_v = 74.0f;

    float rand_mot_temp = RandUniform(96.5f, 122.5f);
    sensors.motor_temp_left_c = rand_mot_temp;
    sensors.motor_temp_right_c = 50.0f;
    sensors.inverter_temp_left_c = 45.0f;
    sensors.inverter_temp_right_c = 45.0f;
    float expected_mot_scale = (MOTOR_TEMP_CUTOFF_C - rand_mot_temp) /
                               (MOTOR_TEMP_CUTOFF_C - MOTOR_TEMP_DERATING_START_C);
    float expected_mot_torque = MOTOR_MAX_TORQUE_NM * expected_mot_scale;

    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_params, &diff_state, &lc_ctx, &driver, &sensors, &commands, dt);
    TEST_ASSERT((safety_ctx.fault_flags & FAULT_OVER_TEMP) != 0U);
    TEST_ASSERT(fabsf(commands.torque_left_nm - expected_mot_torque) < 0.05f);

    /* Corte total por sobretemperatura do inversor */
    float rand_inv_cutoff = RandUniform(85.1f, 105.0f);
    sensors.inverter_temp_left_c = rand_inv_cutoff;
    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_params, &diff_state, &lc_ctx, &driver, &sensors, &commands, dt);
    TEST_ASSERT(commands.torque_left_nm == 0.0f && !commands.inverter_left_enable);

    /* Derating de subtensao */
    sensors.motor_temp_left_c = 50.0f;
    sensors.inverter_temp_left_c = 42.0f;
    float rand_uv_volt = RandUniform(58.5f, 63.5f);
    sensors.battery_voltage_v = rand_uv_volt;
    float expected_uv_scale = (rand_uv_volt - BATTERY_UNDERVOLTAGE_CUTOFF_V) /
                              (BATTERY_UNDERVOLTAGE_START_V - BATTERY_UNDERVOLTAGE_CUTOFF_V);
    float expected_uv_torque = MOTOR_MAX_TORQUE_NM * expected_uv_scale;
    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_params, &diff_state, &lc_ctx, &driver, &sensors, &commands, dt);
    TEST_ASSERT((safety_ctx.fault_flags & FAULT_UNDERVOLTAGE) != 0U);
    TEST_ASSERT(fabsf(commands.torque_left_nm - expected_uv_torque) < 0.05f);

    /* Corte total em 0.0 V */
    sensors.battery_voltage_v = 0.0f;
    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_params, &diff_state, &lc_ctx, &driver, &sensors, &commands, dt);
    TEST_ASSERT((safety_ctx.fault_flags & FAULT_UNDERVOLTAGE) != 0U);
    TEST_ASSERT(commands.torque_left_nm == 0.0f && !commands.inverter_left_enable);

    /* Validacao explicita do Brake Blending em 3 pontos da curva */
    VehicleControl_Init(&safety_ctx);
    sensors.battery_voltage_v = 74.0f;
    sensors.battery_current_a = 0.0f;
    sensors.wheel_speed_fl_rads = (8.0f / FRONT_TIRE_RADIUS_DYNAMIC_M);
    sensors.wheel_speed_fr_rads = (8.0f / FRONT_TIRE_RADIUS_DYNAMIC_M);
    sensors.wheel_speed_rl_rads = (8.0f / TIRE_RADIUS_DYNAMIC_M);
    sensors.wheel_speed_rr_rads = (8.0f / TIRE_RADIUS_DYNAMIC_M);
    driver.apps1_percent = 0.0f;
    driver.apps2_percent = 0.0f;

    /* Ponto 1: Frenagem leve a 10 bar */
    driver.brake_pressure_bar = 10.0f;
    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_params, &diff_state, &lc_ctx, &driver, &sensors, &commands, dt);
    TEST_ASSERT(commands.regen_active);
    float t_regen_10 = fabsf(commands.torque_left_nm);

    /* Ponto 2: Frenagem moderada a 30 bar (proporcionalidade crescente |T_30| > |T_10|) */
    driver.brake_pressure_bar = 30.0f;
    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_params, &diff_state, &lc_ctx, &driver, &sensors, &commands, dt);
    TEST_ASSERT(commands.regen_active);
    float t_regen_30 = fabsf(commands.torque_left_nm);
    TEST_ASSERT(t_regen_30 > t_regen_10);

    /* Ponto 3: Frenagem de panico a 70 bar (> 65 bar: corte de regeneracao para estabilidade) */
    driver.brake_pressure_bar = 70.0f;
    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_params, &diff_state, &lc_ctx, &driver, &sensors, &commands, dt);
    TEST_ASSERT(!commands.regen_active);
    TEST_ASSERT(fabsf(commands.torque_left_nm) < 1e-4f);

    /* Bloqueio do eixo dianteiro */
    sensors.wheel_speed_fl_rads = 0.0f;
    sensors.wheel_speed_fr_rads = 0.0f;
    sensors.ax_mss = -6.5f;
    driver.brake_pressure_bar = 45.0f;
    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_params, &diff_state, &lc_ctx, &driver, &sensors, &commands, dt);
    TEST_ASSERT(safety_ctx.estimated_vx_ms > (0.75f * v_test));

    /* Watchdog CAN */
    VehicleControl_Init(&safety_ctx);
    sensors.battery_voltage_v = 72.0f;
    sensors.can_heartbeat_counter = 100U;
    driver.apps1_percent = 50.0f;
    driver.apps2_percent = 50.0f;
    driver.brake_pressure_bar = 0.0f;
    driver.brake_switch_active = false;
    VehicleControl_Step(&safety_ctx, &diff_params, &diff_state, &lc_ctx, &driver, &sensors, &commands, dt);
    TEST_ASSERT((safety_ctx.fault_flags & FAULT_CAN_TIMEOUT) == 0U);
    for (int i = 0; i < 11; i++) {
        VehicleControl_Step(&safety_ctx, &diff_params, &diff_state, &lc_ctx, &driver, &sensors, &commands, dt);
    }
    TEST_ASSERT((safety_ctx.fault_flags & FAULT_CAN_TIMEOUT) != 0U);
    TEST_ASSERT(commands.torque_left_nm == 0.0f && !commands.inverter_left_enable);

    printf("Testes de seguranca funcional concluidos com sucesso.\n\n");
}

void RunMonteCarloStressTest(int num_trials) {
    printf("--- Teste 4: Monte Carlo (%d iteracoes) ---\n", num_trials);

    int count_normal_traction = 0;
    int count_regen_active = 0;
    int count_thermal_derating = 0;
    int count_uv_derating = 0;
    int count_safety_cutoffs = 0;

    for (int trial = 0; trial < num_trials; trial++) {
        VehicleSafetyContext_t safety_ctx;
        ElectronicDiffParams_t diff_params;
        ElectronicDiffState_t diff_state;
        LaunchControlContext_t lc_ctx;

        VehicleControl_Init(&safety_ctx);
        ElectronicDiff_InitParams(&diff_params);
        ElectronicDiff_InitState(&diff_state);
        LaunchControl_Init(&lc_ctx);

        DriverInputs_t driver = {0};
        SensorData_t sensors = {0};
        MotorCommands_t commands = {0};

        float dt = RandUniform(0.002f, 0.020f);
        float v_true = RandUniform(0.0f, 24.0f);
        bool inject_fault = (RandUniform(0.0f, 1.0f) < 0.15f);
        float steer_deg = inject_fault ? RandUniform(-45.0f, 45.0f) : RandUniform(-29.5f, 29.5f);

        if (inject_fault) {
            driver.apps1_percent = RandUniform(-10.0f, 115.0f);
            driver.apps2_percent = RandUniform(-10.0f, 115.0f);
        } else {
            float base_apps = (RandUniform(0.0f, 1.0f) < 0.25f) ? 0.0f : RandUniform(0.0f, 100.0f);
            driver.apps1_percent = base_apps;
            driver.apps2_percent = clampf(base_apps + RandUniform(-4.0f, 4.0f), 0.0f, 100.0f);
        }

        driver.brake_pressure_bar = (driver.apps1_percent < 2.0f) ?
                                    RandUniform(0.0f, 70.0f) :
                                    ((RandUniform(0.0f, 1.0f) < 0.08f) ? RandUniform(16.0f, 80.0f) : 0.0f);
        driver.brake_switch_active = (driver.brake_pressure_bar > 5.0f);
        driver.steering_angle_rad = steer_deg * DEG_TO_RAD;
        driver.launch_button_pressed = (v_true < 0.4f && driver.brake_pressure_bar > 16.0f &&
                                        RandUniform(0.0f, 1.0f) < 0.30f);

        sensors.wheel_speed_fl_rads = (v_true / FRONT_TIRE_RADIUS_DYNAMIC_M) * RandUniform(0.95f, 1.05f);
        sensors.wheel_speed_fr_rads = (v_true / FRONT_TIRE_RADIUS_DYNAMIC_M) * RandUniform(0.95f, 1.05f);
        sensors.wheel_speed_rl_rads = (v_true / TIRE_RADIUS_DYNAMIC_M) * RandUniform(0.92f, 1.25f);
        sensors.wheel_speed_rr_rads = (v_true / TIRE_RADIUS_DYNAMIC_M) * RandUniform(0.92f, 1.25f);
        sensors.yaw_rate_rads = RandUniform(-2.5f, 2.5f);
        sensors.ax_mss = RandUniform(-12.0f, 12.0f);
        sensors.ay_mss = RandUniform(-14.0f, 14.0f);
        sensors.motor_temp_left_c = RandUniform(20.0f, 132.0f);
        sensors.motor_temp_right_c = RandUniform(20.0f, 132.0f);
        sensors.inverter_temp_left_c = RandUniform(20.0f, 90.0f);
        sensors.inverter_temp_right_c = RandUniform(20.0f, 90.0f);
        sensors.battery_voltage_v = RandUniform(56.0f, 81.5f);
        sensors.battery_current_a = RandUniform(-55.0f, 240.0f);

        for (int s_idx = 0; s_idx < 12; s_idx++) {
            sensors.can_heartbeat_counter++;
            VehicleControl_Step(&safety_ctx, &diff_params, &diff_state, &lc_ctx, &driver, &sensors, &commands, dt);

            /* Atualizacao em malha fechada da corrente DC para exercitar dinamicamente a malha de derating */
            float omega_l = fabsf(sensors.wheel_speed_rl_rads) * GEAR_RATIO;
            float omega_r = fabsf(sensors.wheel_speed_rr_rads) * GEAR_RATIO;
            float p_mec = (fabsf(commands.torque_left_nm) * omega_l) + (fabsf(commands.torque_right_nm) * omega_r);
            float p_elec = p_mec / INVERTER_MOTOR_EFFICIENCY;
            if (commands.regen_active) {
                sensors.battery_current_a = -(p_elec * INVERTER_MOTOR_EFFICIENCY * INVERTER_MOTOR_EFFICIENCY) /
                                            fmaxf(sensors.battery_voltage_v, 48.0f);
            } else {
                sensors.battery_current_a = p_elec / fmaxf(sensors.battery_voltage_v, 48.0f);
            }

            TEST_ASSERT(isfinite(commands.torque_left_nm));
            TEST_ASSERT(isfinite(commands.torque_right_nm));
            TEST_ASSERT(isfinite(commands.target_rpm_left));
            TEST_ASSERT(isfinite(commands.target_rpm_right));
            TEST_ASSERT(isfinite(safety_ctx.estimated_vx_ms));

            TEST_ASSERT(commands.torque_left_nm >= -MOTOR_MAX_TORQUE_NM - 1e-3f &&
                        commands.torque_left_nm <= MOTOR_MAX_TORQUE_NM + 1e-3f);
            TEST_ASSERT(commands.torque_right_nm >= -MOTOR_MAX_TORQUE_NM - 1e-3f &&
                        commands.torque_right_nm <= MOTOR_MAX_TORQUE_NM + 1e-3f);
            TEST_ASSERT(commands.target_rpm_left <= MOTOR_MAX_RPM + 1e-2f && commands.target_rpm_left >= -1e-2f);
            TEST_ASSERT(commands.target_rpm_right <= MOTOR_MAX_RPM + 1e-2f && commands.target_rpm_right >= -1e-2f);

            if (!commands.inverter_left_enable || !commands.inverter_right_enable || !safety_ctx.safety_plausibility_ok) {
                TEST_ASSERT(fabsf(commands.torque_left_nm) < 1e-5f);
                TEST_ASSERT(fabsf(commands.torque_right_nm) < 1e-5f);
                TEST_ASSERT(!commands.regen_active);
            }

            if (!commands.regen_active) {
                TEST_ASSERT(commands.torque_left_nm >= 0.0f);
                TEST_ASSERT(commands.torque_right_nm >= 0.0f);
            } else {
                TEST_ASSERT(commands.torque_left_nm <= 0.0f);
                TEST_ASSERT(commands.torque_right_nm <= 0.0f);
            }
        }

        if (!commands.inverter_left_enable || safety_ctx.bspd_latch_tripped) {
            count_safety_cutoffs++;
        } else if (commands.regen_active) {
            count_regen_active++;
        } else if (commands.torque_left_nm > 0.0f || commands.torque_right_nm > 0.0f) {
            count_normal_traction++;
        }
        if ((safety_ctx.fault_flags & FAULT_OVER_TEMP) != 0U) {
            count_thermal_derating++;
        }
        if ((safety_ctx.fault_flags & FAULT_UNDERVOLTAGE) != 0U) {
            count_uv_derating++;
        }
    }

    printf("Resumo estatistico:\n");
    printf("  Tracao ativa: %d\n", count_normal_traction);
    printf("  Regeneracao ativa: %d\n", count_regen_active);
    printf("  Derating termico: %d\n", count_thermal_derating);
    printf("  Derating subtensao: %d\n", count_uv_derating);
    printf("  Cortes de seguranca (BSPD/cutoff): %d\n", count_safety_cutoffs);
    printf("Monte Carlo validado com sucesso.\n\n");
}

int main(int argc, char *argv[]) {
    unsigned int seed = (unsigned int)time(NULL);
    if (argc > 1) {
        seed = (unsigned int)strtoul(argv[1], NULL, 10);
    }
    srand(seed);

    printf("Bancada de Testes VCU - Kart Formula Tesla UFMG (Seed: %u)\n\n", seed);

    RunCorneringTest();
    RunLaunchControlSimulation();
    RunPlausibilitySafetyTest();
    RunMonteCarloStressTest(1000);

    printf("Todos os testes passaram com sucesso.\n");
    return 0;
}
