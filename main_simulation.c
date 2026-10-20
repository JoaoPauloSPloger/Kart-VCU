/*
 * main_simulation.c - Bancada de testes e simulação da VCU
 * Validação em malha fechada de Ackermann, Torque Vectoring,
 * Launch Control e segurança funcional com Monte Carlo.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>
#include <assert.h>
#include "vehicle_control.h"
#include "electronic_differential.h"
#include "launch_control.h"

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
    p->mass_kg               = RandUniform(190.0f, 250.0f);
    p->wheelbase_m           = RandUniform(0.935f, 0.975f);
    p->rear_track_m          = RandUniform(1.060f, 1.140f);
    p->cg_height_m           = RandUniform(0.195f, 0.245f);
    p->weight_dist_rear      = RandUniform(0.560f, 0.640f);
    p->frontal_area_m2       = RandUniform(0.390f, 0.460f);
    p->drag_coeff_cd         = RandUniform(0.630f, 0.770f);
    p->rolling_res_crr       = RandUniform(0.016f, 0.028f);
    p->mu_peak               = RandUniform(0.680f, 0.960f);
    p->j_eq_axle_kgm2        = 2.0f * RandUniform(0.095f, 0.130f);
    p->batt_open_circuit_v   = RandUniform(68.0f, 79.5f);
    p->batt_internal_res_ohm = RandUniform(0.020f, 0.042f);
    p->motor_temp_init_c     = RandUniform(28.0f, 75.0f);
    p->inverter_temp_init_c  = RandUniform(25.0f, 52.0f);
}

/* Transição suave do coeficiente de atrito longitudinal (fórmula de Pacejka simplificada) */
static float TireFrictionCurve_Mu(float slip_ratio, float mu_peak) {
    if (slip_ratio <= 0.0f) return 0.0f;
    const float lambda_peak = 0.12f;
    float u = slip_ratio / lambda_peak;
    float mu_slide = 0.78f * mu_peak;
    if (u <= 1.0f) {
        return mu_peak * ((2.0f * u) / (1.0f + (u * u)));
    }
    return mu_slide + (mu_peak - mu_slide) * expf(-0.8f * (u - 1.0f));
}

void RunCorneringTest(void) {
    printf("--- Teste 1: Diferencial Eletrônico e Vetorização de Torque ---\n");

    for (int turn_idx = 0; turn_idx < 2; turn_idx++) {
        float turn_sign = (turn_idx == 0) ? 1.0f : -1.0f;
        const char *turn_name = (turn_sign > 0.0f) ? "Direita" : "Esquerda";

        VehicleSafetyContext_t safety_ctx;
        ElectronicDiffConfig_t diff_config;
        LaunchControlContext_t lc_ctx;

        VehicleControl_Init(&safety_ctx);
        ElectronicDiff_Init(&diff_config);
        LaunchControl_Init(&lc_ctx);

        DriverInputs_t driver = {0};
        SensorData_t sensors = {0};
        MotorCommands_t commands = {0};

        float dt              = RandUniform(0.005f, 0.015f);
        float v_target        = RandUniform(7.5f, 16.5f);
        float r_curve         = RandUniform(8.0f, 22.0f);
        float throttle_pct    = RandUniform(40.0f, 78.0f);
        float apps_offset     = RandUniform(-1.5f, 1.5f);
        float understeer_ratio= RandUniform(0.70f, 0.88f);
        float delta_rad       = turn_sign * atanf(WHEELBASE_M / r_curve);

        float denom = WHEELBASE_M + ((diff_config.understeer_coeff * v_target * v_target) / GRAVITY_MSS);
        float yaw_rate_des = (v_target / denom) * tanf(delta_rad);

        driver.apps1_percent      = throttle_pct;
        driver.apps2_percent      = throttle_pct + apps_offset;
        driver.steering_angle_rad = delta_rad;
        driver.steering_angle_deg = delta_rad * RAD_TO_DEG;

        sensors.yaw_rate_rads         = understeer_ratio * yaw_rate_des;
        sensors.wheel_speed_fl_rads   = v_target / FRONT_TIRE_RADIUS_DYNAMIC_M;
        sensors.wheel_speed_fr_rads   = v_target / FRONT_TIRE_RADIUS_DYNAMIC_M;
        sensors.wheel_speed_rl_rads   = (v_target * (1.0f + (turn_sign * REAR_TRACK_M / (2.0f * r_curve)))) / TIRE_RADIUS_DYNAMIC_M;
        sensors.wheel_speed_rr_rads   = (v_target * (1.0f - (turn_sign * REAR_TRACK_M / (2.0f * r_curve)))) / TIRE_RADIUS_DYNAMIC_M;
        sensors.motor_temp_left_c     = RandUniform(35.0f, 82.0f);
        sensors.motor_temp_right_c    = RandUniform(35.0f, 82.0f);
        sensors.inverter_temp_left_c  = RandUniform(30.0f, 58.0f);
        sensors.inverter_temp_right_c = RandUniform(30.0f, 58.0f);
        sensors.battery_voltage_v     = RandUniform(66.5f, 79.0f);
        sensors.battery_current_a     = RandUniform(45.0f, 130.0f);

        for (int i = 0; i < 20; i++) {
            sensors.can_heartbeat_counter++;
            VehicleControl_Step(&safety_ctx, &diff_config, &lc_ctx, &driver, &sensors, &commands, dt);
        }

        assert(commands.inverter_left_enable && commands.inverter_right_enable);
        if (turn_sign > 0.0f) {
            assert(commands.target_rpm_left > commands.target_rpm_right);
            assert(commands.torque_left_nm > commands.torque_right_nm);
        } else {
            assert(commands.target_rpm_right > commands.target_rpm_left);
            assert(commands.torque_right_nm > commands.torque_left_nm);
        }

        printf("Curva para %s: R=%.1fm, v=%.1f km/h, esterço=%.1f deg, delta_T=%.2f N.m\n",
               turn_name, r_curve, sensors.vehicle_speed_ms * 3.6f, driver.steering_angle_deg,
               commands.torque_left_nm - commands.torque_right_nm);

        /* Saturação com derating térmico */
        driver.apps1_percent = 100.0f;
        driver.apps2_percent = 100.0f;
        sensors.wheel_speed_rl_rads = 18.0f;
        sensors.wheel_speed_rr_rads = 18.0f;
        sensors.motor_temp_left_c   = RandUniform(98.0f, 112.0f);
        float expected_t_lim = MOTOR_MAX_TORQUE_NM *
                               ((MOTOR_TEMP_CUTOFF_C - fmaxf(sensors.motor_temp_left_c, sensors.motor_temp_right_c)) /
                                (MOTOR_TEMP_CUTOFF_C - MOTOR_TEMP_DERATING_START_C));
        sensors.can_heartbeat_counter++;
        VehicleControl_Step(&safety_ctx, &diff_config, &lc_ctx, &driver, &sensors, &commands, dt);

        if (turn_sign > 0.0f) {
            assert(commands.torque_left_nm > commands.torque_right_nm);
            assert(fabsf(commands.torque_left_nm - expected_t_lim) < 1e-2f);
        } else {
            assert(commands.torque_right_nm > commands.torque_left_nm);
            assert(fabsf(commands.torque_right_nm - expected_t_lim) < 1e-2f);
        }
    }
    printf("Diferencial e vetorização validados.\n\n");
}

void RunLaunchControlSimulation(void) {
    printf("--- Teste 2: Simulação Dinâmica de Largada ---\n");

    RandomizedPlantParams_t plant;
    SampleRandomPlantParams(&plant);

    VehicleSafetyContext_t safety_ctx;
    ElectronicDiffConfig_t diff_config;
    LaunchControlContext_t lc;

    VehicleControl_Init(&safety_ctx);
    ElectronicDiff_Init(&diff_config);
    LaunchControl_Init(&lc);

    DriverInputs_t driver = {0};
    SensorData_t sensors = {0};
    MotorCommands_t commands = {0};

    float dt = 0.005f;
    float t = 0.0f;
    float v_x = 0.0f;
    float s = 0.0f;
    float omega_rear = 0.0f;

    sensors.motor_temp_left_c     = plant.motor_temp_init_c;
    sensors.motor_temp_right_c    = plant.motor_temp_init_c + RandUniform(-1.5f, 1.5f);
    sensors.inverter_temp_left_c  = plant.inverter_temp_init_c;
    sensors.inverter_temp_right_c = plant.inverter_temp_init_c + RandUniform(-1.0f, 1.0f);
    sensors.battery_voltage_v     = plant.batt_open_circuit_v;
    sensors.can_heartbeat_counter = 1U;

    float staging_brake_bar   = RandUniform(18.0f, 55.0f);
    float staging_hold_time_s = RandUniform(0.15f, 1.80f);
    int   staging_steps       = (int)(staging_hold_time_s / dt);

    driver.launch_button_pressed = true;
    driver.brake_pressure_bar    = staging_brake_bar;
    driver.brake_switch_active   = true;
    driver.apps1_percent         = 0.0f;
    driver.apps2_percent         = 0.0f;
    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_config, &lc, &driver, &sensors, &commands, dt);
    assert(lc.state == LC_STATE_ARMED);

    float launch_pedal_pct = RandUniform(88.0f, 100.0f);
    driver.apps1_percent = launch_pedal_pct;
    driver.apps2_percent = launch_pedal_pct;
    for (int st = 0; st < staging_steps; st++) {
        sensors.can_heartbeat_counter++;
        VehicleControl_Step(&safety_ctx, &diff_config, &lc, &driver, &sensors, &commands, dt);
        assert(lc.state == LC_STATE_STAGING);
        assert(fabsf(commands.torque_left_nm - 8.0f) < 1e-3f);
    }

    driver.apps1_percent         = 100.0f;
    driver.apps2_percent         = 100.0f;
    driver.brake_pressure_bar    = 0.0f;
    driver.brake_switch_active   = false;
    driver.launch_button_pressed = false;

    printf("Tempo(s) | Vel(km/h) | Dist(m) | Torque(N.m) | V_bat(V) | Slip(%%) | Estado\n");
    printf("-----------------------------------------------------------------------\n");

    const int   substeps = 10;
    const float dt_sub   = dt / (float)substeps;
    int print_divider    = 0;
    float a_x            = 0.0f;

    while (v_x < (60.0f / 3.6f) && t < 7.0f) {
        float enc_noise_f = RandGaussian(0.0f, 0.03f);
        float enc_noise_r = RandGaussian(0.0f, 0.03f);
        float imu_noise   = RandGaussian(0.0f, 0.08f);

        sensors.wheel_speed_fl_rads = fmaxf(0.0f, (v_x / FRONT_TIRE_RADIUS_DYNAMIC_M) + enc_noise_f);
        sensors.wheel_speed_fr_rads = fmaxf(0.0f, (v_x / FRONT_TIRE_RADIUS_DYNAMIC_M) - enc_noise_f);
        sensors.wheel_speed_rl_rads = fmaxf(0.0f, omega_rear + enc_noise_r);
        sensors.wheel_speed_rr_rads = fmaxf(0.0f, omega_rear - enc_noise_r);
        sensors.ax_mss              = a_x + imu_noise;
        sensors.can_heartbeat_counter++;

        VehicleControl_Step(&safety_ctx, &diff_config, &lc, &driver, &sensors, &commands, dt);

        float omega_mot_rads = omega_rear * GEAR_RATIO;
        float p_mec_w        = (commands.torque_left_nm + commands.torque_right_nm) * omega_mot_rads;
        float p_elec_w       = (p_mec_w > 0.0f) ? (p_mec_w / INVERTER_MOTOR_EFFICIENCY) : 0.0f;
        float i_batt_a       = p_elec_w / fmaxf(sensors.battery_voltage_v, 48.0f);
        sensors.battery_current_a = i_batt_a;
        sensors.battery_voltage_v = clampf(plant.batt_open_circuit_v - (i_batt_a * plant.batt_internal_res_ohm),
                                           64.5f, 81.5f);

        float norm_curr = (commands.torque_left_nm / MOTOR_MAX_TORQUE_NM);
        sensors.motor_temp_left_c    += (1.8f * norm_curr * norm_curr) * dt;
        sensors.motor_temp_right_c   += (1.8f * norm_curr * norm_curr) * dt;
        sensors.inverter_temp_left_c += (1.2f * norm_curr * norm_curr) * dt;
        sensors.inverter_temp_right_c+= (1.2f * norm_curr * norm_curr) * dt;

        float t_axle_nm = (commands.torque_left_nm + commands.torque_right_nm) *
                          GEAR_RATIO * TRANSMISSION_EFFICIENCY;

        for (int k = 0; k < substeps; k++) {
            float v_rear_tang = omega_rear * TIRE_RADIUS_DYNAMIC_M;
            float v_ref = (v_x > 1.0f) ? v_x : 1.0f;
            float slip = (v_rear_tang - v_x) / v_ref;
            if (slip < 0.0f) slip = 0.0f;

            float n_rear = (plant.mass_kg * GRAVITY_MSS * plant.weight_dist_rear) +
                           (plant.mass_kg * fmaxf(0.0f, a_x) * (plant.cg_height_m / plant.wheelbase_m));

            float mu_curr = TireFrictionCurve_Mu(slip, plant.mu_peak);
            float f_trac  = mu_curr * n_rear;

            float d_omega_rear = (t_axle_nm - (f_trac * TIRE_RADIUS_DYNAMIC_M)) / plant.j_eq_axle_kgm2;
            omega_rear += d_omega_rear * dt_sub;
            if (omega_rear < 0.0f) omega_rear = 0.0f;

            float f_res = (plant.rolling_res_crr * plant.mass_kg * GRAVITY_MSS) +
                          (0.5f * AIR_DENSITY_KG_M3 * plant.drag_coeff_cd * plant.frontal_area_m2 * v_x * v_x);
            a_x = (f_trac - f_res) / plant.mass_kg;
            if (v_x <= 0.0f && a_x < 0.0f) {
                a_x = 0.0f;
            }

            v_x += a_x * dt_sub;
            s   += v_x * dt_sub;
        }

        t += dt;

        float v_ref_print = (v_x > 1.0f) ? v_x : 1.0f;
        float slip_pct = ((omega_rear * TIRE_RADIUS_DYNAMIC_M - v_x) / v_ref_print) * 100.0f;
        if (slip_pct < 0.0f) slip_pct = 0.0f;

        if (print_divider++ % 40 == 0) {
            const char *state_str = (lc.state == LC_STATE_LAUNCHING) ? "LAUNCH" :
                                    (lc.state == LC_STATE_COMPLETED) ? "COMPL" : "NORMAL";
            printf(" %5.2f   |   %5.1f   |  %5.1f  |    %5.2f    |   %5.1f  |  %5.1f  | %s\n",
                   t, v_x * 3.6f, s, commands.torque_left_nm, sensors.battery_voltage_v, slip_pct, state_str);
        }
    }

    assert(v_x >= (60.0f / 3.6f));
    assert(t < 6.5f);
    assert(lc.state == LC_STATE_DISARMED || lc.state == LC_STATE_COMPLETED);

    printf("Tempo 0-60 km/h: %.2f s | Distância: %.1f m\n", t, s);
    printf("Launch Control validado.\n\n");
}

void RunPlausibilitySafetyTest(void) {
    printf("--- Teste 3: Segurança Funcional e Proteções ---\n");

    VehicleSafetyContext_t safety_ctx;
    ElectronicDiffConfig_t diff_cfg;
    LaunchControlContext_t lc_ctx;

    VehicleControl_Init(&safety_ctx);
    ElectronicDiff_Init(&diff_cfg);
    LaunchControl_Init(&lc_ctx);

    DriverInputs_t driver = {0};
    SensorData_t sensors = {0};
    MotorCommands_t commands = {0};
    float dt = 0.01f;

    sensors.battery_voltage_v     = RandUniform(68.0f, 78.0f);
    sensors.motor_temp_left_c     = RandUniform(35.0f, 65.0f);
    sensors.motor_temp_right_c    = RandUniform(35.0f, 65.0f);
    sensors.inverter_temp_left_c  = RandUniform(30.0f, 50.0f);
    sensors.inverter_temp_right_c = RandUniform(30.0f, 50.0f);
    sensors.can_heartbeat_counter = 1U;

    /* Discrepância APPS > 10% por 100 ms */
    float apps1_fault = RandUniform(35.0f, 95.0f);
    float discrepancy = RandUniform(12.0f, 32.0f);
    float apps2_fault = fmaxf(0.0f, apps1_fault - discrepancy);
    driver.apps1_percent = apps1_fault;
    driver.apps2_percent = apps2_fault;
    for (int i = 0; i < 15; i++) {
        sensors.can_heartbeat_counter++;
        VehicleControl_Step(&safety_ctx, &diff_cfg, &lc_ctx, &driver, &sensors, &commands, dt);
    }
    assert(!driver.safety_plausibility_ok);
    assert((safety_ctx.fault_flags & FAULT_APPS_DISCREPANCY) != 0U);
    assert(commands.torque_left_nm == 0.0f && !commands.inverter_left_enable);

    /* Entrada fora da faixa */
    driver.apps1_percent = RandUniform(105.0f, 150.0f);
    driver.apps2_percent = driver.apps1_percent;
    VehicleControl_ValidateInputs(&safety_ctx, &driver, dt);
    assert((safety_ctx.fault_flags & FAULT_APPS_OUT_OF_RANGE) != 0U);
    assert(!driver.safety_plausibility_ok);

    /* Disparo e rearme do BSPD */
    VehicleControl_Init(&safety_ctx);
    float bspd_apps  = RandUniform(30.0f, 98.0f);
    float bspd_brake = RandUniform(18.0f, 85.0f);
    driver.apps1_percent      = bspd_apps;
    driver.apps2_percent      = bspd_apps;
    driver.brake_pressure_bar = bspd_brake;
    driver.brake_switch_active= true;

    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_cfg, &lc_ctx, &driver, &sensors, &commands, dt);
    assert(safety_ctx.bspd_latch_tripped);
    assert((safety_ctx.fault_flags & FAULT_BSPD_TRIPPED) != 0U);
    assert(driver.throttle_demand == 0.0f && commands.torque_left_nm == 0.0f);

    /* Retenção com pedal > 5% */
    float hold_apps = RandUniform(6.0f, 24.0f);
    driver.brake_pressure_bar = 0.0f;
    driver.brake_switch_active= false;
    driver.apps1_percent      = hold_apps;
    driver.apps2_percent      = hold_apps;
    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_cfg, &lc_ctx, &driver, &sensors, &commands, dt);
    assert(safety_ctx.bspd_latch_tripped && driver.throttle_demand == 0.0f);

    /* Rearme com pedal < 5% */
    float rec_apps = RandUniform(2.2f, 4.8f);
    driver.apps1_percent = rec_apps;
    driver.apps2_percent = rec_apps;
    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_cfg, &lc_ctx, &driver, &sensors, &commands, dt);
    assert(!safety_ctx.bspd_latch_tripped);
    assert((safety_ctx.fault_flags & FAULT_BSPD_TRIPPED) == 0U);
    assert(driver.throttle_demand > 0.0f);

    /* Timeout de staging */
    VehicleControl_Init(&safety_ctx);
    LaunchControl_Init(&lc_ctx);
    driver.launch_button_pressed = true;
    driver.brake_pressure_bar    = RandUniform(20.0f, 50.0f);
    driver.brake_switch_active   = true;
    driver.apps1_percent         = 0.0f;
    driver.apps2_percent         = 0.0f;
    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_cfg, &lc_ctx, &driver, &sensors, &commands, dt);
    assert(lc_ctx.state == LC_STATE_ARMED);

    driver.apps1_percent = RandUniform(80.0f, 100.0f);
    driver.apps2_percent = driver.apps1_percent;
    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_cfg, &lc_ctx, &driver, &sensors, &commands, dt);
    assert(lc_ctx.state == LC_STATE_STAGING);

    for (int i = 0; i < 305; i++) {
        sensors.can_heartbeat_counter++;
        VehicleControl_Step(&safety_ctx, &diff_cfg, &lc_ctx, &driver, &sensors, &commands, dt);
    }
    assert(lc_ctx.state == LC_STATE_ABORTED);
    assert(commands.torque_left_nm == 0.0f && !commands.inverter_left_enable);
    assert((safety_ctx.fault_flags & FAULT_BSPD_TRIPPED) != 0U);

    /* Abortamento por freio em launch */
    VehicleControl_Init(&safety_ctx);
    LaunchControl_Init(&lc_ctx);
    driver.launch_button_pressed = true;
    driver.brake_pressure_bar    = 25.0f;
    driver.brake_switch_active   = true;
    driver.apps1_percent         = 0.0f;
    driver.apps2_percent         = 0.0f;
    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_cfg, &lc_ctx, &driver, &sensors, &commands, dt);
    driver.apps1_percent = 100.0f;
    driver.apps2_percent = 100.0f;
    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_cfg, &lc_ctx, &driver, &sensors, &commands, dt);
    driver.brake_pressure_bar    = 0.0f;
    driver.brake_switch_active   = false;
    driver.launch_button_pressed = false;
    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_cfg, &lc_ctx, &driver, &sensors, &commands, dt);
    assert(lc_ctx.state == LC_STATE_LAUNCHING);

    driver.brake_pressure_bar  = RandUniform(22.0f, 70.0f);
    driver.brake_switch_active = true;
    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_cfg, &lc_ctx, &driver, &sensors, &commands, dt);
    assert(lc_ctx.state == LC_STATE_ABORTED);
    assert(commands.torque_left_nm == 0.0f && !commands.inverter_left_enable);
    assert((safety_ctx.fault_flags & FAULT_BSPD_TRIPPED) != 0U);

    /* Derating térmico e elétrico */
    VehicleControl_Init(&safety_ctx);
    LaunchControl_Init(&lc_ctx);
    driver.launch_button_pressed = false;
    driver.brake_pressure_bar    = 0.0f;
    driver.brake_switch_active   = false;
    driver.apps1_percent         = 100.0f;
    driver.apps2_percent         = 100.0f;

    float v_test = RandUniform(5.0f, 8.2f);
    sensors.wheel_speed_fl_rads = (v_test / FRONT_TIRE_RADIUS_DYNAMIC_M);
    sensors.wheel_speed_fr_rads = (v_test / FRONT_TIRE_RADIUS_DYNAMIC_M);
    sensors.wheel_speed_rl_rads = (v_test / TIRE_RADIUS_DYNAMIC_M);
    sensors.wheel_speed_rr_rads = (v_test / TIRE_RADIUS_DYNAMIC_M);
    sensors.battery_voltage_v   = 74.0f;

    float rand_mot_temp = RandUniform(96.5f, 122.5f);
    sensors.motor_temp_left_c    = rand_mot_temp;
    sensors.motor_temp_right_c   = 50.0f;
    sensors.inverter_temp_left_c = 45.0f;
    sensors.inverter_temp_right_c= 45.0f;
    float expected_mot_scale = (MOTOR_TEMP_CUTOFF_C - rand_mot_temp) /
                               (MOTOR_TEMP_CUTOFF_C - MOTOR_TEMP_DERATING_START_C);
    float expected_mot_torque = MOTOR_MAX_TORQUE_NM * expected_mot_scale;

    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_cfg, &lc_ctx, &driver, &sensors, &commands, dt);
    assert((safety_ctx.fault_flags & FAULT_OVER_TEMP) != 0U);
    assert(fabsf(commands.torque_left_nm - expected_mot_torque) < 0.05f);

    /* Corte total por inversor */
    float rand_inv_cutoff = RandUniform(85.1f, 105.0f);
    sensors.inverter_temp_left_c = rand_inv_cutoff;
    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_cfg, &lc_ctx, &driver, &sensors, &commands, dt);
    assert(commands.torque_left_nm == 0.0f && !commands.inverter_left_enable);

    /* Subtensão (inclusive corte estrito em 0.0 V) */
    sensors.motor_temp_left_c    = 50.0f;
    sensors.inverter_temp_left_c = 42.0f;
    float rand_uv_volt = RandUniform(58.5f, 63.5f);
    sensors.battery_voltage_v = rand_uv_volt;
    float expected_uv_scale = (rand_uv_volt - BATTERY_UNDERVOLTAGE_CUTOFF_V) /
                              (BATTERY_UNDERVOLTAGE_START_V - BATTERY_UNDERVOLTAGE_CUTOFF_V);
    float expected_uv_torque = MOTOR_MAX_TORQUE_NM * expected_uv_scale;
    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_cfg, &lc_ctx, &driver, &sensors, &commands, dt);
    assert((safety_ctx.fault_flags & FAULT_UNDERVOLTAGE) != 0U);
    assert(fabsf(commands.torque_left_nm - expected_uv_torque) < 0.05f);

    /* Falha de 0.0 V corta torque imediatamente */
    sensors.battery_voltage_v = 0.0f;
    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_cfg, &lc_ctx, &driver, &sensors, &commands, dt);
    assert((safety_ctx.fault_flags & FAULT_UNDERVOLTAGE) != 0U);
    assert(commands.torque_left_nm == 0.0f && !commands.inverter_left_enable);

    /* Regeneração */
    sensors.battery_voltage_v = RandUniform(68.0f, 78.0f);
    driver.apps1_percent      = 0.0f;
    driver.apps2_percent      = 0.0f;
    driver.brake_pressure_bar = RandUniform(4.0f, 34.0f);
    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_cfg, &lc_ctx, &driver, &sensors, &commands, dt);
    assert(commands.regen_active);
    assert(commands.torque_left_nm < 0.0f && commands.torque_right_nm < 0.0f);
    assert(commands.target_rpm_left == 0.0f && commands.target_rpm_right == 0.0f);

    /* Bloqueio do eixo dianteiro */
    sensors.wheel_speed_fl_rads = 0.0f;
    sensors.wheel_speed_fr_rads = 0.0f;
    sensors.ax_mss              = -6.5f;
    driver.brake_pressure_bar   = 45.0f;
    sensors.can_heartbeat_counter++;
    VehicleControl_Step(&safety_ctx, &diff_cfg, &lc_ctx, &driver, &sensors, &commands, dt);
    assert(sensors.vehicle_speed_ms > (0.75f * v_test));

    /* Watchdog CAN ativo: contador congelado gera timeout */
    VehicleControl_Init(&safety_ctx);
    sensors.battery_voltage_v = 72.0f;
    sensors.can_heartbeat_counter = 100U;
    driver.apps1_percent = 50.0f;
    driver.apps2_percent = 50.0f;
    driver.brake_pressure_bar = 0.0f;
    driver.brake_switch_active = false;
    VehicleControl_Step(&safety_ctx, &diff_cfg, &lc_ctx, &driver, &sensors, &commands, dt);
    assert((safety_ctx.fault_flags & FAULT_CAN_TIMEOUT) == 0U);
    for (int i = 0; i < 11; i++) {
        VehicleControl_Step(&safety_ctx, &diff_cfg, &lc_ctx, &driver, &sensors, &commands, dt);
    }
    assert((safety_ctx.fault_flags & FAULT_CAN_TIMEOUT) != 0U);
    assert(commands.torque_left_nm == 0.0f && !commands.inverter_left_enable);

    printf("Testes de segurança funcional concluídos com sucesso.\n\n");
}

void RunMonteCarloStressTest(int num_trials) {
    printf("--- Teste 4: Monte Carlo (%d iterações) ---\n", num_trials);

    int count_normal_traction  = 0;
    int count_regen_active     = 0;
    int count_thermal_derating = 0;
    int count_uv_derating      = 0;
    int count_safety_cutoffs   = 0;

    for (int trial = 0; trial < num_trials; trial++) {
        VehicleSafetyContext_t safety_ctx;
        ElectronicDiffConfig_t diff_cfg;
        LaunchControlContext_t lc_ctx;

        VehicleControl_Init(&safety_ctx);
        ElectronicDiff_Init(&diff_cfg);
        LaunchControl_Init(&lc_ctx);

        DriverInputs_t driver = {0};
        SensorData_t sensors = {0};
        MotorCommands_t commands = {0};

        float dt = RandUniform(0.002f, 0.020f);
        float v_true = RandUniform(0.0f, 24.0f);
        float steer_deg = RandUniform(-42.0f, 42.0f);

        bool inject_fault = (RandUniform(0.0f, 1.0f) < 0.15f);
        if (inject_fault) {
            driver.apps1_percent = RandUniform(-10.0f, 115.0f);
            driver.apps2_percent = RandUniform(-10.0f, 115.0f);
        } else {
            float base_apps = (RandUniform(0.0f, 1.0f) < 0.25f) ? 0.0f : RandUniform(0.0f, 100.0f);
            driver.apps1_percent = base_apps;
            driver.apps2_percent = clampf(base_apps + RandUniform(-4.0f, 4.0f), 0.0f, 100.0f);
        }

        driver.brake_pressure_bar    = (driver.apps1_percent < 2.0f) ?
                                       RandUniform(0.0f, 70.0f) :
                                       ((RandUniform(0.0f, 1.0f) < 0.08f) ? RandUniform(16.0f, 80.0f) : 0.0f);
        driver.brake_switch_active   = (driver.brake_pressure_bar > 5.0f);
        driver.steering_angle_deg    = steer_deg;
        driver.steering_angle_rad    = steer_deg * DEG_TO_RAD;
        driver.launch_button_pressed = (v_true < 0.4f && driver.brake_pressure_bar > 16.0f &&
                                        RandUniform(0.0f, 1.0f) < 0.30f);

        sensors.wheel_speed_fl_rads   = (v_true / FRONT_TIRE_RADIUS_DYNAMIC_M) * RandUniform(0.95f, 1.05f);
        sensors.wheel_speed_fr_rads   = (v_true / FRONT_TIRE_RADIUS_DYNAMIC_M) * RandUniform(0.95f, 1.05f);
        sensors.wheel_speed_rl_rads   = (v_true / TIRE_RADIUS_DYNAMIC_M) * RandUniform(0.92f, 1.25f);
        sensors.wheel_speed_rr_rads   = (v_true / TIRE_RADIUS_DYNAMIC_M) * RandUniform(0.92f, 1.25f);
        sensors.yaw_rate_rads         = RandUniform(-2.5f, 2.5f);
        sensors.ax_mss                = RandUniform(-12.0f, 12.0f);
        sensors.ay_mss                = RandUniform(-14.0f, 14.0f);
        sensors.motor_temp_left_c     = RandUniform(20.0f, 132.0f);
        sensors.motor_temp_right_c    = RandUniform(20.0f, 132.0f);
        sensors.inverter_temp_left_c  = RandUniform(20.0f, 90.0f);
        sensors.inverter_temp_right_c = RandUniform(20.0f, 90.0f);
        sensors.battery_voltage_v     = RandUniform(56.0f, 81.5f);
        sensors.battery_current_a     = RandUniform(-55.0f, 240.0f);

        for (int s_idx = 0; s_idx < 12; s_idx++) {
            sensors.can_heartbeat_counter++;
            VehicleControl_Step(&safety_ctx, &diff_cfg, &lc_ctx, &driver, &sensors, &commands, dt);

            assert(isfinite(commands.torque_left_nm));
            assert(isfinite(commands.torque_right_nm));
            assert(isfinite(commands.target_rpm_left));
            assert(isfinite(commands.target_rpm_right));
            assert(isfinite(sensors.vehicle_speed_ms));

            assert(commands.torque_left_nm >= -MOTOR_MAX_TORQUE_NM - 1e-3f &&
                   commands.torque_left_nm <=  MOTOR_MAX_TORQUE_NM + 1e-3f);
            assert(commands.torque_right_nm >= -MOTOR_MAX_TORQUE_NM - 1e-3f &&
                   commands.torque_right_nm <=  MOTOR_MAX_TORQUE_NM + 1e-3f);
            assert(commands.target_rpm_left  <= MOTOR_MAX_RPM + 1e-2f && commands.target_rpm_left >= -1e-2f);
            assert(commands.target_rpm_right <= MOTOR_MAX_RPM + 1e-2f && commands.target_rpm_right >= -1e-2f);

            if (!commands.inverter_left_enable || !commands.inverter_right_enable || !driver.safety_plausibility_ok) {
                assert(fabsf(commands.torque_left_nm) < 1e-5f);
                assert(fabsf(commands.torque_right_nm) < 1e-5f);
                assert(!commands.regen_active);
            }

            if (!commands.regen_active) {
                assert(commands.torque_left_nm >= 0.0f);
                assert(commands.torque_right_nm >= 0.0f);
            } else {
                assert(commands.torque_left_nm <= 0.0f);
                assert(commands.torque_right_nm <= 0.0f);
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

    printf("Resumo estatístico:\n");
    printf("  Tração ativa: %d\n", count_normal_traction);
    printf("  Regeneração ativa: %d\n", count_regen_active);
    printf("  Derating térmico: %d\n", count_thermal_derating);
    printf("  Derating subtensão: %d\n", count_uv_derating);
    printf("  Cortes de segurança (BSPD/cutoff): %d\n", count_safety_cutoffs);
    printf("Monte Carlo validado com sucesso.\n\n");
}

int main(int argc, char *argv[]) {
    unsigned int seed = (unsigned int)time(NULL);
    if (argc > 1) {
        seed = (unsigned int)strtoul(argv[1], NULL, 10);
    }
    srand(seed);

    printf("Bancada de Testes VCU - Kart Fórmula Tesla UFMG (Seed: %u)\n\n", seed);

    RunCorneringTest();
    RunLaunchControlSimulation();
    RunPlausibilitySafetyTest();
    RunMonteCarloStressTest(1000);

    printf("Todos os testes passaram com sucesso.\n");
    return 0;
}
