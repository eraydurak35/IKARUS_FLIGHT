#ifndef TYPEDEFS_H
#define TYPEDEFS_H

#include <stdio.h>

#define BUFF_SIZE 130
#define DEG_TO_RAD 0.0174533f
#define RAD_TO_DEG 57.2957795f

typedef struct
{
    int analog_LX;
    int analog_LY;
    int analog_RX;
    int analog_RY;
    int analog_LB;
    int analog_RB;
    int left_trigger;
    int right_trigger;
    int left_shoulder;
    int right_shoulder;
    int button_A;
    int button_B;
    int button_X;
    int button_Y;
} gamepad_t;

typedef struct
{
    int32_t latitude[25];
    int32_t longitude[25];
    uint8_t altitude[25];
    int8_t counter;
    uint8_t is_reached;
} waypoint_t;

typedef struct
{
    int64_t current_time;
    int64_t pulse_lenght;
    int64_t prev_time;
    uint8_t ppm_interrupt_flag;
    uint8_t ppm_start_found;
    uint16_t channel[8];
} radio_t;

typedef struct
{
    float pitch;
    float roll;
    float heading;
    float pitch_degs;
    float roll_degs;
    float yaw_degs;
    float altitude;
    float velocity_x_ms;
    float velocity_y_ms;
    float velocity_z_ms;
    float throttle;
    int32_t latitude;
    int32_t longitude;
} target_t;

typedef struct
{
    uint8_t fix;
    uint8_t satCount;
    int32_t latitude;
    int32_t longitude;
    int32_t altitude_mm;
    int32_t northVel_mms;
    int32_t eastVel_mms;
    int32_t downVel_mms;
    int32_t headingOfMotion;
    uint16_t hdop;
    uint16_t vdop;
    int32_t latitude_origin;
    int32_t longitude_origin;
    int32_t altitude_origin_mm;
    float longitude_scale;
} gps_t;

typedef struct
{
    int16_t range_cm;
} range_finder_t;

typedef struct
{
    float notch_1_freq;
    float notch_1_bandwidth;
    float notch_2_freq;
    float notch_2_bandwidth;
    float ahrs_filter_beta;
    float ahrs_filter_zeta;
    float alt_filter_beta;
    float velz_filter_beta;
    float velz_filter_zeta;
    float velxy_filter_beta;
    float mag_declination_deg;
} nav_config_t;

typedef struct
{
    float pitch_p;
    float pitch_i;
    float pitch_d;

    float roll_p;
    float roll_i;
    float roll_d;

    float yaw_p;
    float yaw_i;

    float position_p;
    float position_i;
    float position_d;

    float altitude_p;
    float altitude_i;
    float altitude_d;

    float max_pitch_angle;
    float max_roll_angle;
    float max_pitch_rate;
    float max_roll_rate;
    float max_yaw_rate;

    float pitch_rate_scale;
    float roll_rate_scale;
    float yaw_rate_scale;
    float max_vertical_velocity;
    float max_horizontal_velocity;

    float v_sens_gain;
    float v_drop_compensation_gain;
    float takeoff_altitude;
    float hover_throttle;
    float notch_1_freq;
    float notch_1_bandwidth;
    float notch_2_freq;
    float notch_2_bandwidth;
    float ahrs_filter_beta;
    float ahrs_filter_zeta;
    float alt_filter_beta;
    float mag_declination_deg;
    //
    float velz_filter_beta;
    float velz_filter_zeta;
    float velxy_filter_beta;
} config_t;

typedef struct
{
    float battery_voltage;
    float pitch;
    float roll;
    float heading;
    float altitude;
    float altitude_calibrated;
    float tof_distance_1;
    float tof_distance_2;
    float velocity_x_ms;
    float velocity_y_ms;
    float velocity_z_ms;
    float flow_x_velocity;
    float flow_y_velocity;
    float flow_quality;
    float flight_mode;
    float arm_status;
    float target_pitch;
    float target_roll;
    float target_heading;
    float target_pitch_dps;
    float target_roll_dps;
    float target_yaw_dps;
    float target_altitude;
    float target_velocity_x_ms;
    float target_velocity_y_ms;
    float target_velocity_z_ms;
    float barometer_pressure;
    float barometer_temperature;
    float imu_temperature;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float acc_x_ms2;
    float acc_y_ms2;
    float acc_z_ms2;
    float mag_x_mgauss;
    float mag_y_mgauss;
    float mag_z_mgauss;

    //==============//
    //     GPS      //
    //==============//
    float gps_fix;
    float gps_satCount;
    float gps_latitude;
    float gps_longitude;
    float gps_altitude_m;
    float gps_northVel_ms;
    float gps_eastVel_ms;
    float gps_downVel_ms;
    float gps_headingOfMotion;
    float gps_hdop;
    float gps_vdop;
    float gps_latitude_origin;
    float gps_longitude_origin;
    float gps_altitude_origin;
    float target_latitude;
    float target_longitude;
    float distance_m_2d;
    float distance_m_3d;
    float velocity_ms_2d;
} telemetry_t;
typedef struct
{
    uint8_t arm_status;
    uint8_t alt_hold_status;
    uint8_t pos_hold_status;
    uint8_t takeoff_status;
    uint8_t waypoint_mission_status;
} flight_t;


typedef struct
{
  float pitch;
  float roll;
  float heading;
  float pitch_dps;
  float roll_dps;
  float yaw_dps;
} data_1_t;

typedef struct
{
    int16_t velocity_x_ms;
    int16_t velocity_y_ms;
    int16_t velocity_z_ms;
    int16_t flow_x_velocity_ms;
    int16_t flow_y_velocity_ms;
    uint8_t flow_quality;
    int16_t altitude;
    int16_t baro_altitude;
    int16_t barometer_pressure;
    uint16_t barometer_temperature;
} data_2_t;

typedef struct
{
    uint16_t imu_temperature;
    int16_t acc_x_ned_ms2;
    int16_t acc_y_ned_ms2;
    int16_t acc_z_ned_ms2;
    int16_t acc_x_ms2;
    int16_t acc_y_ms2;
    int16_t acc_z_ms2;
    int16_t mag_x_gauss;
    int16_t mag_y_gauss;
    int16_t mag_z_gauss;
} data_3_t;

typedef struct
{
    uint8_t size;
    float buffer[5];
    uint8_t index;
} fir_filter_t;

typedef struct
{
    float sample_rate;
    float a0;
    float a1;
    float a2;
    float b0;
    float b1;
    float b2;
    float x1;
    float x2;
    float y1;
    float y2;
} notch_filter_t;

typedef struct
{
    float pitch_deg;
    float roll_deg;
    float heading_deg;
    float pitch_dps;
    float roll_dps;
    float yaw_dps;
    float altitude_m;
    float vel_forward_ms;
    float vel_right_ms;
    float vel_up_ms;
    float acc_forward_ms2;
    float acc_right_ms2;
    float acc_up_ms2;
} states_t;

typedef struct
{
    uint8_t data[BUFF_SIZE];
    uint8_t lenght;
} uart_data_t;

#endif