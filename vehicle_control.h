/*
 * vehicle_control.h - VCU do Kart Elétrico Fórmula Tesla UFMG
 * Definições de powertrain, segurança funcional (BSPD/APPS) e orquestração.
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

/* Conversões de unidade efetivamente utilizadas nos módulos */
#define DEG_TO_RAD                      ((float)M_PI / 180.0f)
#define RAD_TO_DEG                      (180.0f / (float)M_PI)
#define RPM_TO_RADS                     ((2.0f * (float)M_PI) / 60.0f)
#define RADS_TO_RPM                     (60.0f / (2.0f * (float)M_PI))

/* Constantes físicas */
#define GRAVITY_MSS                     (9.81f)
#define AIR_DENSITY_KG_M3               (1.225f)

/* Geometria e inércia do chassi */
#define TOTAL_VEHICLE_MASS_KG           (250.0f)
#define WHEELBASE_M                     (0.955f)
#define REAR_TRACK_M                    (1.100f)
#define FRONT_TRACK_M                   (0.950f)
#define CG_HEIGHT_M                     (0.220f)
#define WEIGHT_DIST_REAR                (0.600f)
#define WEIGHT_DIST_FRONT               (0.400f)

/* Aerodinâmica e rolamento */
#define FRONTAL_AREA_M2                 (0.420f)
#define DRAG_COEFFICIENT_CD             (0.700f)
#define ROLLING_RESISTANCE_CRR          (0.022f)

/* Pneus (dianteiro 10x4.50-5, traseiro 11x7.10-5) */
#define FRONT_TIRE_RADIUS_DYNAMIC_M     (0.125f)
#define TIRE_RADIUS_NOMINAL_M           (0.135f)
#define TIRE_RADIUS_DYNAMIC_M           (0.132f)
#define TIRE_PEAK_FRICTION_COEFF        (1.150f)
#define REAR_WHEEL_EQUIV_INERTIA_KGM2   (0.110f)
#define MAX_STEERING_ANGLE_DEG          (60.0f)
#define MAX_STEERING_ANGLE_RAD          (MAX_STEERING_ANGLE_DEG * DEG_TO_RAD)

/* Transmissão (Coroa 59T / Pinhão 16T = 3.6875:1) e powertrain */
#define GEAR_RATIO                      (59.0f / 16.0f)
#define TRANSMISSION_EFFICIENCY         (0.950f)
#define INVERTER_MOTOR_EFFICIENCY       (0.880f)
#define MOTOR_MAX_RPM                   (6000.0f)
#define MAX_SPEED_KMH                   (80.0f)
#define MAX_SPEED_MS                    (MAX_SPEED_KMH / 3.6f)

/* Limites elétricos e motores (2x BLDC/PMSM) */
#define NUM_MOTORS                      (2)
#define MOTOR_RATED_POWER_KW            (2.5f)
#define MOTOR_PEAK_POWER_KW             (6.8f)
#define SYSTEM_MAX_POWER_KW             (13.6f)
#define MOTOR_MAX_TORQUE_NM             (25.0f)
#define SYSTEM_MAX_TORQUE_NM            (50.0f)

/* Bateria e barramento DC */
#define DC_BUS_NOMINAL_VOLTAGE_V        (72.0f)
#define BATTERY_MAX_DISCHARGE_A         (220.0f)
#define BATTERY_MAX_REGEN_A             (60.0f)
#define BATTERY_UNDERVOLTAGE_START_V    (64.0f)
#define BATTERY_UNDERVOLTAGE_CUTOFF_V   (58.0f)
#define BATTERY_MAX_REGEN_VOLTAGE_V     (82.0f)
#define REGEN_MIN_SPEED_MS              (1.5f)

/* Limites térmicos */
#define MOTOR_TEMP_DERATING_START_C     (95.0f)
#define MOTOR_TEMP_CUTOFF_C             (125.0f)
#define INVERTER_TEMP_DERATING_START_C  (65.0f)
#define INVERTER_TEMP_CUTOFF_C          (85.0f)

/* Máscaras de diagnóstico de falha */
#define FAULT_NONE                      (0U)
#define FAULT_APPS_DISCREPANCY          (1U << 0)
#define FAULT_APPS_OUT_OF_RANGE         (1U << 1)
#define FAULT_BRAKE_OUT_OF_RANGE        (1U << 2)
#define FAULT_BSPD_TRIPPED              (1U << 3)
#define FAULT_OVER_TEMP                 (1U << 4)
#define FAULT_UNDERVOLTAGE              (1U << 5)
#define FAULT_SENSOR_NAN                (1U << 6)
#define FAULT_CAN_TIMEOUT               (1U << 7)

static inline float clampf(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

typedef struct {
    float    implausibility_timer_s;
    bool     bspd_latch_tripped;
    uint32_t fault_flags;
    float    estimated_vx_ms;
    bool     vx_filter_initialized;
    uint32_t last_can_heartbeat;
    float    can_watchdog_timer_s;
    bool     can_watchdog_armed;
} VehicleSafetyContext_t;

typedef struct {
    float apps1_percent;
    float apps2_percent;
    float raw_pedal_demand;
    float throttle_demand;
    float brake_pressure_bar;
    bool  brake_switch_active;
    float steering_angle_deg;
    float steering_angle_rad;
    bool  launch_button_pressed;
    bool  safety_plausibility_ok;
} DriverInputs_t;

typedef struct {
    float    wheel_speed_fl_rads;
    float    wheel_speed_fr_rads;
    float    wheel_speed_rl_rads;
    float    wheel_speed_rr_rads;
    float    vehicle_speed_ms;
    float    yaw_rate_rads;
    float    ax_mss;
    float    ay_mss;
    float    motor_temp_left_c;
    float    motor_temp_right_c;
    float    inverter_temp_left_c;
    float    inverter_temp_right_c;
    float    battery_voltage_v;
    float    battery_current_a;
    uint32_t can_heartbeat_counter;
} SensorData_t;

typedef struct {
    float torque_left_nm;
    float torque_right_nm;
    float target_rpm_left;
    float target_rpm_right;
    bool  inverter_left_enable;
    bool  inverter_right_enable;
    bool  regen_active;
} MotorCommands_t;

struct ElectronicDiffConfig_s;
struct LaunchControlContext_s;

void VehicleControl_Init(VehicleSafetyContext_t *safety_ctx);

bool VehicleControl_ValidateInputs(VehicleSafetyContext_t *safety_ctx,
                                   DriverInputs_t *inputs,
                                   float dt_s);

bool VehicleControl_ValidateSensors(VehicleSafetyContext_t *safety_ctx,
                                    const SensorData_t *sensors,
                                    float dt_s);

float VehicleControl_EstimateChassisSpeed(VehicleSafetyContext_t *safety_ctx,
                                          const DriverInputs_t *driver,
                                          SensorData_t *sensors,
                                          float dt_s);

void VehicleControl_ApplyPowerAndThermalLimits(VehicleSafetyContext_t *safety_ctx,
                                               const DriverInputs_t *driver,
                                               const SensorData_t *sensors,
                                               MotorCommands_t *commands);

void VehicleControl_Step(VehicleSafetyContext_t *safety_ctx,
                         struct ElectronicDiffConfig_s *diff_cfg,
                         struct LaunchControlContext_s *lc_ctx,
                         DriverInputs_t *driver,
                         SensorData_t *sensors,
                         MotorCommands_t *commands,
                         float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* VEHICLE_CONTROL_H */
