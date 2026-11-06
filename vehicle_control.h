/*
 * vehicle_control.h - Unidade de Controle Veicular (VCU)
 * Formula Tesla UFMG - Powertrain elétrico e segurança funcional (FSAE EV / ISO 26262)
 */

#ifndef VEHICLE_CONTROL_H
#define VEHICLE_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define DEG_TO_RAD (0.017453292519943295f)
#define RAD_TO_DEG (57.29577951308232f)
#define RPM_TO_RADS (0.10471975511965977f)
#define RADS_TO_RPM (9.549296585513721f)

#define GRAVITY_MSS (9.81f)
#define AIR_DENSITY_KG_M3 (1.225f)

/* Parametros do chassi tubular e distribuicao de massa */
#define TOTAL_VEHICLE_MASS_KG (250.0f)
#define WHEELBASE_M (0.955f)
#define REAR_TRACK_M (1.100f)
#define FRONT_TRACK_M (0.950f)
#define CG_HEIGHT_M (0.220f)
#define WEIGHT_DIST_REAR (0.600f)
#define WEIGHT_DIST_FRONT (0.400f)

#define FRONTAL_AREA_M2 (0.420f)
#define DRAG_COEFFICIENT_CD (0.700f)
#define ROLLING_RESISTANCE_CRR (0.022f)

/* Pneus slick (dianteiro 10x4.50-5, traseiro 11x7.10-5) */
#define FRONT_TIRE_RADIUS_DYNAMIC_M (0.125f)
#define TIRE_RADIUS_DYNAMIC_M (0.140f)
#define TIRE_RADIUS_NOMINAL_M TIRE_RADIUS_DYNAMIC_M
#define TIRE_PEAK_FRICTION_COEFF (1.150f)
#define REAR_WHEEL_EQUIV_INERTIA_KGM2 (0.110f)
#define MAX_STEERING_ANGLE_DEG (29.5f)
#define MAX_STEERING_ANGLE_RAD (MAX_STEERING_ANGLE_DEG * DEG_TO_RAD)

/* Relacao de transmissao por corrente (40T/16T) e limites cinematicos */
#define GEAR_RATIO (2.50f)
#define TRANSMISSION_EFFICIENCY (0.950f)
#define INVERTER_MOTOR_EFFICIENCY (0.902f)
#define MOTOR_MAX_RPM (4222.8f)
#define MAX_SPEED_KMH (83.20f)
#define MAX_SPEED_MS (MAX_SPEED_KMH / 3.6f)

/* Powertrain: 2x Golden Motor HPM03KW-72V + EZkontrol A72400 */
#define NUM_MOTORS (2)
#define MOTOR_RATED_POWER_KW (3.0f)
#define MOTOR_PEAK_POWER_KW (6.8f)
#define SYSTEM_MAX_POWER_KW (13.6f)
#define MOTOR_MAX_TORQUE_NM (25.0f)
#define SYSTEM_MAX_TORQUE_NM (50.0f)

/* Acumulador LiFePO4 e barramento DC */
#define DC_BUS_NOMINAL_VOLTAGE_V (72.0f)
#define BATTERY_MAX_DISCHARGE_A (220.0f)
#define BATTERY_MAX_REGEN_A (60.0f)
#define BATTERY_UNDERVOLTAGE_START_V (64.0f)
#define BATTERY_UNDERVOLTAGE_CUTOFF_V (58.0f)
#define BATTERY_MAX_REGEN_VOLTAGE_V (82.0f)
#define REGEN_MIN_SPEED_MS (1.5f)

/* Limites termicos de operacao */
#define MOTOR_TEMP_DERATING_START_C (95.0f)
#define MOTOR_TEMP_CUTOFF_C (125.0f)
#define INVERTER_TEMP_DERATING_START_C (65.0f)
#define INVERTER_TEMP_CUTOFF_C (85.0f)

/* Temporizacoes e limiares de plausibilidade (FSAE EV.4.7 / T.4.3) */
#define CAN_WATCHDOG_TIMEOUT_S (0.100f)
#define APPS_DISCREPANCY_MAX_PCT (10.0f)
#define APPS_DISCREPANCY_TIMEOUT_S (0.100f)
#define BSPD_BRAKE_THRESHOLD_BAR (15.0f)
#define BSPD_APPS_TRIP_PERCENT (25.0f)
#define BSPD_APPS_RECOVERY_PERCENT (5.0f)
#define BSPD_DEBOUNCE_TIME_S (0.150f)
#define DT_MAX_SATURATION_S (0.050f)
#define BRAKE_PRESSURE_MAX_BAR (120.0f)
#define APPS_DEADBAND_PERCENT (2.0f)

/* Mascaras de diagnostico de falhas (DTC) */
#define FAULT_NONE (0U)
#define FAULT_APPS_DISCREPANCY (1U << 0)
#define FAULT_APPS_OUT_OF_RANGE (1U << 1)
#define FAULT_BRAKE_OUT_OF_RANGE (1U << 2)
#define FAULT_BSPD_TRIPPED (1U << 3)
#define FAULT_OVER_TEMP (1U << 4)
#define FAULT_UNDERVOLTAGE (1U << 5)
#define FAULT_CAN_TIMEOUT (1U << 6)
#define FAULT_INVALID_DT (1U << 7)
#define FAULT_SENSOR_NAN (1U << 8)
#define FAULT_IMU_RANGE (1U << 9)
#define FAULT_WHEEL_SPEED_SENSOR (1U << 10)
#define FAULT_THERMAL_SENSOR (1U << 11)
#define FAULT_BATTERY_VOLTAGE_RANGE (1U << 12)
#define FAULT_STEERING_RANGE (1U << 13)

#define FAULT_CRITICAL_DRIVE_MASK (FAULT_APPS_DISCREPANCY | FAULT_APPS_OUT_OF_RANGE | \
                                   FAULT_BRAKE_OUT_OF_RANGE | FAULT_CAN_TIMEOUT | \
                                   FAULT_INVALID_DT | FAULT_SENSOR_NAN | \
                                   FAULT_IMU_RANGE | FAULT_WHEEL_SPEED_SENSOR | \
                                   FAULT_THERMAL_SENSOR | FAULT_BATTERY_VOLTAGE_RANGE | \
                                   FAULT_STEERING_RANGE)

static inline float clampf(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

/* Estado de seguranca e variaveis internas da VCU */
typedef struct {
    float implausibility_timer_s;
    float bspd_timer_s;
    bool bspd_latch_tripped;
    uint32_t fault_flags;

    float estimated_vx_ms;
    bool vx_filter_initialized;

    uint32_t last_can_heartbeat;
    float can_watchdog_timer_s;
    bool can_watchdog_armed;

    float raw_pedal_demand;
    float throttle_demand;
    float validated_brake_bar;
    bool safety_plausibility_ok;

    float battery_current_scale;
    float regen_current_scale;
} VehicleSafetyContext_t;

/* Entradas fisicas do piloto (sensores de pedal e volante) */
typedef struct {
    float apps1_percent;
    float apps2_percent;
    float brake_pressure_bar;
    bool brake_switch_active;
    float steering_angle_rad;
    bool launch_button_pressed;
} DriverInputs_t;

/* Leituras brutas de telemetria e barramento CAN */
typedef struct {
    float wheel_speed_fl_rads;
    float wheel_speed_fr_rads;
    float wheel_speed_rl_rads;
    float wheel_speed_rr_rads;
    float yaw_rate_rads;
    float ax_mss;
    float ay_mss;
    float motor_temp_left_c;
    float motor_temp_right_c;
    float inverter_temp_left_c;
    float inverter_temp_right_c;
    float battery_voltage_v;
    float battery_current_a;
    uint32_t can_heartbeat_counter;
} SensorData_t;

/* Setpoints para os inversores EZkontrol via CAN */
typedef struct {
    float torque_left_nm;
    float torque_right_nm;
    float target_rpm_left;
    float target_rpm_right;
    bool inverter_left_enable;
    bool inverter_right_enable;
    bool regen_active;
} MotorCommands_t;

struct ElectronicDiffParams_s;
struct ElectronicDiffState_s;
struct LaunchControlContext_s;

void VehicleControl_Init(VehicleSafetyContext_t *safety_ctx);

bool VehicleControl_ValidateInputs(VehicleSafetyContext_t *safety_ctx,
                                   const DriverInputs_t *inputs,
                                   bool is_staging,
                                   float dt_s);

bool VehicleControl_ValidateSensors(VehicleSafetyContext_t *safety_ctx,
                                    const SensorData_t *sensors,
                                    float dt_s);

float VehicleControl_EstimateChassisSpeed(VehicleSafetyContext_t *safety_ctx,
                                          const DriverInputs_t *driver,
                                          const SensorData_t *sensors,
                                          float dt_s);

void VehicleControl_ApplyPowerAndThermalLimits(VehicleSafetyContext_t *safety_ctx,
                                               const SensorData_t *sensors,
                                               MotorCommands_t *commands,
                                               float dt_s);

void VehicleControl_Step(VehicleSafetyContext_t *safety_ctx,
                         const struct ElectronicDiffParams_s *diff_params,
                         struct ElectronicDiffState_s *diff_state,
                         struct LaunchControlContext_s *lc_ctx,
                         const DriverInputs_t *driver,
                         const SensorData_t *sensors,
                         MotorCommands_t *commands,
                         float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* VEHICLE_CONTROL_H */
