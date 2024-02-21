#include "control_algorithm.h"
#include "typedefs.h"
#include "math.h"
#include "filters.h"
#include "esc.h"

static ibus_t *radio_p;
static flight_t *flight_p;
static target_t *target_p;
static states_t *state_p;
static telemetry_t *telemetry_p;
static config_t *config_p;
static waypoint_t *waypoint_p;
static gps_t *gps_p;
struct PID
{
    float errPitch, errRoll, errYaw;
    float errPitchPrev, errRollPrev, errYawPrev;
    float pitchDegsPrev, rollDegsPrev, yawDegsPrev;
    float pitchPout, rollPout, yawPout;
    float pitchIout, rollIout, yawIout;
    float pitchDout, rollDout, yawDout;
    float pitchPIDout, rollPIDout, yawPIout;
    float errVel_x, errVel_x_prev, velocity_x_ms_prev;
    float errVel_y, errVel_y_prev, velocity_y_ms_prev;
    float errVel_z, errVel_z_prev, velocity_z_ms_prev;
    float altPout, altIout, altDout;
    float posXPout, posXIout, posXDout;
    float posYPout, posYIout, posYDout;
    float acc_z_prev;
};
static struct PID pid;
struct throttle
{
    float m1, m2, m3, m4;
};

static struct throttle thr;

static fir_filter_t fir_pitch = {5, {0}, 0};
static fir_filter_t fir_roll = {5, {0}, 0};

const float min_throttle = 100, max_throttle = 1000;
static const float maxI = 100.0f, maxPID = 300.0f;
static const float pos_pitch_roll_max_deg = 10.0f;

static int16_t apply_deadband(int16_t input, uint8_t deadband);
static uint8_t outer_control_loop_wp();
static void outer_control_loop_rc();
static void inner_control_loop();
static void arm();
static void disarm();

void control_init(ibus_t *rc, telemetry_t *tlm, flight_t *flt, target_t *trg, states_t *stt, config_t *cfg, waypoint_t *wp, gps_t *g)
{
    radio_p = rc;
    flight_p = flt;
    target_p = trg;
    state_p = stt;
    telemetry_p = tlm;
    config_p = cfg;
    waypoint_p = wp;
    gps_p = g;
}

void flight_mode_control()
{
    // ||==============================================||
    // ||                 RC ARM LOGIC                 ||
    // ||==============================================||
    if (radio_p->ch4 > 1700 && flight_p->arm_status == 0)
    {
        if ((flight_p->alt_hold_status == 1 && (radio_p->ch2 < 1600 && radio_p->ch2 > 1400)) || (flight_p->alt_hold_status == 0 && radio_p->ch2 < 1100))
        {
            arm();
        }
    }
    else if (radio_p->ch4 < 1300 && flight_p->arm_status == 1)
    {
        disarm();
    }
    // ||==============================================||
    // ||              RC ALT HOLD LOGIC               ||
    // ||==============================================||
    if (radio_p->ch7 > 1700 && flight_p->alt_hold_status == 0 && fabs(state_p->vel_up_ms) < 2.0f)
    {
        flight_p->alt_hold_status = 1;
        pid.altIout = target_p->throttle;
        target_p->altitude = state_p->altitude_m;
    }
    else if (radio_p->ch7 < 1300 && flight_p->alt_hold_status == 1)
    {
        flight_p->alt_hold_status = 0;
    }

    // ||==============================================||
    // ||              RC POS HOLD LOGIC               ||
    // ||==============================================||
    if ((radio_p->ch6 > 1400 && radio_p->ch6 < 1600) && flight_p->pos_hold_status == 0 && fabs(state_p->vel_forward_ms) < 2.0f && fabs(state_p->vel_right_ms) < 2.0f)
    {
        flight_p->pos_hold_status = 1;
    }
    else if (radio_p->ch6 < 1300 && flight_p->pos_hold_status == 1)
    {
        flight_p->pos_hold_status = 0;
    }

    // ||==============================================||
    // ||              RC WAYPOINT LOGIC               ||
    // ||==============================================||
    if ((radio_p->ch6 > 1800) && flight_p->waypoint_mission_status == 0 && flight_p->arm_status == 1 && flight_p->pos_hold_status == 1 && flight_p->alt_hold_status == 1 && waypoint_p->latitude[0] != 0 && waypoint_p->longitude[0] != 0 && waypoint_p->altitude[0] != 0 /* && gps.fix == 3 && gps.satCount > 5*/)
    {
        flight_p->waypoint_mission_status = 1;
    }
    else if (flight_p->waypoint_mission_status == 1 && (!(radio_p->ch6 > 1800) || flight_p->pos_hold_status == 0 || flight_p->alt_hold_status == 0))
    {
        flight_p->waypoint_mission_status = 0;
        waypoint_p->is_reached = 1;
        waypoint_p->counter = -1;
    }
}

void flight_control() // 1000Hz
{
    if (flight_p->waypoint_mission_status == 1)
    {
        if (outer_control_loop_wp() == 0)
        {
            outer_control_loop_rc();
        }
    }
    else
    {
        outer_control_loop_rc();
    }

    if (flight_p->arm_status == 1)
    {
        inner_control_loop();
    }
    else
    {
        disarm();
    }
}

static uint8_t outer_control_loop_wp() // 1000 Hz
{
    static uint8_t counter = 0;
    static int32_t distance_north_cm = 0, distance_east_cm = 0, distance_2d_cm = 0;
    static float wp_direction = 0;

    // End of the waypoint list or list is empty
    if (waypoint_p->counter >= 25)
        return 0;
    else if (waypoint_p->counter != -1 && (waypoint_p->latitude[waypoint_p->counter] == 0 || waypoint_p->longitude[waypoint_p->counter] == 0))
        return 0;

    counter++;
    if (counter >= 10) // 100Hz
    {
        counter = 0;

        if (waypoint_p->is_reached == 1)
        {
            waypoint_p->counter++;

            if (waypoint_p->counter >= 25)
                return 0;
            else if (waypoint_p->latitude[waypoint_p->counter] == 0 || waypoint_p->longitude[waypoint_p->counter] == 0)
                return 0;
            target_p->latitude = waypoint_p->latitude[waypoint_p->counter];
            target_p->longitude = waypoint_p->longitude[waypoint_p->counter];
            target_p->altitude = waypoint_p->altitude[waypoint_p->counter] / 10.0f;
            waypoint_p->is_reached = 0;
        }
        else
        {
            distance_north_cm = (target_p->latitude - gps_p->latitude) * 1.1f;
            distance_east_cm = (target_p->longitude - gps_p->longitude) * gps_p->longitude_scale;
            distance_2d_cm = sqrtf((distance_north_cm * distance_north_cm) + (distance_east_cm * distance_east_cm));
            telemetry_p->distance_m_2d = distance_2d_cm / 100.0f;
            if (distance_2d_cm < 300) // closer than 3 meters
            {
                waypoint_p->is_reached = 1;
                return 1;
            }
            else
            {
                wp_direction = atan2f(distance_east_cm, distance_north_cm) * RAD_TO_DEG; /// Lowpass ??
                if (wp_direction < 0)
                    wp_direction += 360.0f;

                float heading_diff = state_p->heading_deg - wp_direction;
                if (heading_diff < -180.0f)
                    heading_diff += 360.0f;
                else if (heading_diff > 180.0f)
                    heading_diff -= 360.0f;
                target_p->heading = wp_direction;
                // Calculate yaw rate from setpoint error
                target_p->yaw_degs = (target_p->heading - state_p->heading_deg) * config_p->yaw_rate_scale;
                // This part ensures the craft turns from closest side to setpoint
                if (target_p->yaw_degs < -180.0f * config_p->yaw_rate_scale)
                    target_p->yaw_degs += 360.0f * config_p->yaw_rate_scale;
                else if (target_p->yaw_degs > 180.0f * config_p->yaw_rate_scale)
                    target_p->yaw_degs -= 360.0f * config_p->yaw_rate_scale;

                // Just limit the yaw rate so it doesnt go nuts
                if (target_p->yaw_degs > config_p->max_yaw_rate)
                    target_p->yaw_degs = config_p->max_yaw_rate;
                else if (target_p->yaw_degs < -config_p->max_yaw_rate)
                    target_p->yaw_degs = -config_p->max_yaw_rate;

                if (fabs(heading_diff) < 20.0f)
                {
                    float vel_from_dist = distance_2d_cm / 600.0f; // 3m distance equals 0.5 m/s
                    if (vel_from_dist > config_p->max_horizontal_velocity)
                        vel_from_dist = config_p->max_horizontal_velocity;
                    target_p->velocity_x_ms = vel_from_dist * cosf(heading_diff * DEG_TO_RAD);
                    target_p->velocity_y_ms = vel_from_dist * cosf((heading_diff + 90.0f) * DEG_TO_RAD);

                    if (target_p->velocity_x_ms > config_p->max_horizontal_velocity)
                        target_p->velocity_x_ms = config_p->max_horizontal_velocity;
                    else if (target_p->velocity_x_ms < -config_p->max_horizontal_velocity)
                        target_p->velocity_x_ms = -config_p->max_horizontal_velocity;

                    if (target_p->velocity_y_ms > config_p->max_horizontal_velocity)
                        target_p->velocity_y_ms = config_p->max_horizontal_velocity;
                    else if (target_p->velocity_y_ms < -config_p->max_horizontal_velocity)
                        target_p->velocity_y_ms = -config_p->max_horizontal_velocity;
                }
                else
                {
                    target_p->velocity_x_ms = 0;
                    target_p->velocity_y_ms = 0;
                }

                pid.errVel_x = -(target_p->velocity_x_ms - state_p->vel_forward_ms);
                pid.errVel_y = target_p->velocity_y_ms - state_p->vel_right_ms;

                pid.posXPout = pid.errVel_x * config_p->position_p;
                pid.posYPout = pid.errVel_y * config_p->position_p;

                // output is saturated. dont wind up.
                if (fabs(target_p->pitch) < pos_pitch_roll_max_deg)
                {
                    pid.posXIout += config_p->position_i * 0.005f * (pid.errVel_x + pid.errVel_x_prev);
                    if (pid.posXIout > 2.0f)
                        pid.posXIout = 2.0f;
                    else if (pid.posXIout < -2.0f)
                        pid.posXIout = -2.0f;
                }

                // output is saturated. dont wind up.
                if (fabs(target_p->roll) < pos_pitch_roll_max_deg)
                {
                    pid.posYIout += config_p->position_i * 0.005f * (pid.errVel_y + pid.errVel_y_prev);
                    if (pid.posYIout > 2.0f)
                        pid.posYIout = 2.0f;
                    else if (pid.posYIout < -2.0f)
                        pid.posYIout = -2.0;
                }

                target_p->pitch = pid.posXPout + pid.posXIout + pid.posXDout;
                if (target_p->pitch > pos_pitch_roll_max_deg)
                    target_p->pitch = pos_pitch_roll_max_deg;
                else if (target_p->pitch < -pos_pitch_roll_max_deg)
                    target_p->pitch = -pos_pitch_roll_max_deg;

                target_p->roll = pid.posYPout + pid.posYIout + pid.posYDout;
                if (target_p->roll > pos_pitch_roll_max_deg)
                    target_p->roll = pos_pitch_roll_max_deg;
                else if (target_p->roll < -pos_pitch_roll_max_deg)
                    target_p->roll = -pos_pitch_roll_max_deg;

                /////////////////////   Pitch Rate Controller   ///////////////////////
                target_p->pitch_degs = (target_p->pitch - state_p->pitch_deg) * config_p->pitch_rate_scale;
                if (target_p->pitch_degs > config_p->max_pitch_rate)
                    target_p->pitch_degs = config_p->max_pitch_rate;
                else if (target_p->pitch_degs < -config_p->max_pitch_rate)
                    target_p->pitch_degs = -config_p->max_pitch_rate;

                /////////////////////   Roll Rate Controller   ///////////////////////
                target_p->roll_degs = (target_p->roll - state_p->roll_deg) * config_p->roll_rate_scale;
                if (target_p->roll_degs > config_p->max_roll_rate)
                    target_p->roll_degs = config_p->max_roll_rate;
                else if (target_p->roll_degs < -config_p->max_roll_rate)
                    target_p->roll_degs = -config_p->max_roll_rate; /////////////////////////////////////////////////////////////

                ////////////////////////    Altitude Velocity Controller    /////////////////////////////////
                // Throttle controls the altitude velocity
                if (radio_p->ch2 < 1600 && radio_p->ch2 > 1400) // Throttle stick is centered, velocity calculated from setpoint error
                {
                    // Target velocity is calculated from dividing the difference between set altitude and actual altitude with a constant value
                    target_p->velocity_z_ms = (target_p->altitude - state_p->altitude_m) / 2.0f;
                    if (target_p->velocity_z_ms > config_p->max_vertical_velocity)
                        target_p->velocity_z_ms = config_p->max_vertical_velocity;
                    else if (target_p->velocity_z_ms < -config_p->max_vertical_velocity)
                        target_p->velocity_z_ms = -config_p->max_vertical_velocity;
                }
                else // Throttle stick not centered, velocity calculated from stick input
                {
                    // Calculate the desired altitude velocity from raw stick input
                    target_p->velocity_z_ms = ((radio_p->ch2 - 1500.0f) / 500.0f) * config_p->max_vertical_velocity;
                    // we dont use altitude setpoint if the stick is not centered
                    // but we want to set our setpoint when the stick is in middle
                    // so that when we let go of the stick, craft stays at the altitude we let go
                    target_p->altitude = state_p->altitude_m;
                }
            }
        }
    }
    return 1;
}

static void outer_control_loop_rc()
{
    static uint8_t counter_set_point = 0;
    counter_set_point++;

    if (counter_set_point >= 10) // 100Hz
    {
        counter_set_point = 0;

        if (flight_p->pos_hold_status == 1)
        {

            target_p->velocity_x_ms = (apply_deadband(radio_p->ch1 - 1500, 20) / 500.0f) * config_p->max_horizontal_velocity;
            target_p->velocity_y_ms = (apply_deadband(radio_p->ch0 - 1500, 20) / 500.0f) * config_p->max_horizontal_velocity;

            pid.errVel_x = -(target_p->velocity_x_ms - state_p->vel_forward_ms);
            pid.errVel_y = target_p->velocity_y_ms - state_p->vel_right_ms;

            pid.posXPout = pid.errVel_x * config_p->position_p;
            pid.posYPout = pid.errVel_y * config_p->position_p;

            // output is saturated. dont wind up.
            if (fabs(target_p->pitch) < pos_pitch_roll_max_deg)
            {
                pid.posXIout += config_p->position_i * 0.005f * (pid.errVel_x + pid.errVel_x_prev);
                if (pid.posXIout > 2.0f)
                    pid.posXIout = 2.0f;
                else if (pid.posXIout < -2.0f)
                    pid.posXIout = -2.0f;
            }

            // output is saturated. dont wind up.
            if (fabs(target_p->roll) < pos_pitch_roll_max_deg)
            {
                pid.posYIout += config_p->position_i * 0.005f * (pid.errVel_y + pid.errVel_y_prev);
                if (pid.posYIout > 2.0f)
                    pid.posYIout = 2.0f;
                else if (pid.posYIout < -2.0f)
                    pid.posYIout = -2.0f;
            }

            target_p->pitch = pid.posXPout + pid.posXIout + pid.posXDout;
            if (target_p->pitch > pos_pitch_roll_max_deg)
                target_p->pitch = pos_pitch_roll_max_deg;
            else if (target_p->pitch < -pos_pitch_roll_max_deg)
                target_p->pitch = -pos_pitch_roll_max_deg;

            target_p->roll = pid.posYPout + pid.posYIout + pid.posYDout;
            if (target_p->roll > pos_pitch_roll_max_deg)
                target_p->roll = pos_pitch_roll_max_deg;
            else if (target_p->roll < -pos_pitch_roll_max_deg)
                target_p->roll = -pos_pitch_roll_max_deg;
        }
        else
        {
            target_p->pitch = (apply_deadband(radio_p->ch1 - 1500, 20) / -500.0f) * config_p->max_pitch_angle;
            target_p->roll = (apply_deadband(radio_p->ch0 - 1500, 20) / 500.0f) * config_p->max_roll_angle;
        }

        /////////////////////   Pitch Rate Controller   ///////////////////////
        target_p->pitch_degs = (target_p->pitch - state_p->pitch_deg) * config_p->pitch_rate_scale;
        if (target_p->pitch_degs > config_p->max_pitch_rate)
            target_p->pitch_degs = config_p->max_pitch_rate;
        else if (target_p->pitch_degs < -config_p->max_pitch_rate)
            target_p->pitch_degs = -config_p->max_pitch_rate;

        /////////////////////   Roll Rate Controller   ///////////////////////
        target_p->roll_degs = (target_p->roll - state_p->roll_deg) * config_p->roll_rate_scale;
        if (target_p->roll_degs > config_p->max_roll_rate)
            target_p->roll_degs = config_p->max_roll_rate;
        else if (target_p->roll_degs < -config_p->max_roll_rate)
            target_p->roll_degs = -config_p->max_roll_rate;

        /////////////////////   Yaw Rate Controller   ///////////////////////
        // Yaw stick controls yaw rate
        if (radio_p->ch3 < 1550 && radio_p->ch3 > 1450) // Yaw stick is centered
        {
            // Calculate yaw rate from setpoint error
            target_p->yaw_degs = (target_p->heading - state_p->heading_deg) * config_p->yaw_rate_scale;

            // This part ensures the craft turns from closest side to setpoint
            // Say the setpoint is 5 deg and craft is at 270, logical thing is craft turns clockwise 95 deg
            // If we dont do this craft will attempt to turn counter clockwise 265deg
            if (target_p->yaw_degs < -180.0f * config_p->yaw_rate_scale)
                target_p->yaw_degs += 360.0f * config_p->yaw_rate_scale;
            else if (target_p->yaw_degs > 180.0f * config_p->yaw_rate_scale)
                target_p->yaw_degs -= 360.0f * config_p->yaw_rate_scale;
        }
        else // Yaw stick is not centered
        {
            target_p->yaw_degs = ((radio_p->ch3 - 1500.0f) / 500.0f) * config_p->max_yaw_rate;
            target_p->heading = state_p->heading_deg;
        }
        // Just limit the yaw rate so it doesnt go nuts
        if (target_p->yaw_degs > config_p->max_yaw_rate)
            target_p->yaw_degs = config_p->max_yaw_rate;
        else if (target_p->yaw_degs < -config_p->max_yaw_rate)
            target_p->yaw_degs = -config_p->max_yaw_rate;

        if (flight_p->alt_hold_status == 1)
        {
            if (flight_p->takeoff_status == 1)
            {
                pid.altIout += (config_p->hover_throttle - IDLE_THROTTLE) / 100.0f;

                if (pid.altIout >= config_p->hover_throttle)
                {
                    target_p->altitude = config_p->takeoff_altitude;
                    flight_p->takeoff_status = 0;
                }
            }
            ////////////////////////    Altitude Velocity Controller    /////////////////////////////////
            // Throttle controls the altitude velocity
            if (radio_p->ch2 < 1600 && radio_p->ch2 > 1400) // Throttle stick is centered, velocity calculated from setpoint error
            {
                // Target velocity is calculated from dividing the difference between set altitude and actual altitude with a constant value
                // we always aim 0.1m higher to make sure target is always reached
                target_p->velocity_z_ms = (target_p->altitude - state_p->altitude_m) / 2.0f;
                if (target_p->velocity_z_ms > config_p->max_vertical_velocity)
                    target_p->velocity_z_ms = config_p->max_vertical_velocity;
                else if (target_p->velocity_z_ms < -config_p->max_vertical_velocity)
                    target_p->velocity_z_ms = -config_p->max_vertical_velocity;
            }
            else // Throttle stick not centered, velocity calculated from stick input
            {
                // Calculate the desired altitude velocity from raw stick input
                target_p->velocity_z_ms = ((radio_p->ch2 - 1500.0f) / 500.0f) * config_p->max_vertical_velocity;
                // we dont use altitude setpoint if the stick is not centered
                // but we want to set our setpoint when the stick is in middle
                // so that when we let go of the stick, craft stays at the altitude we let go
                target_p->altitude = state_p->altitude_m;
            }
        }
        else
        {
            // (MAX_TARGET_THROTTLE - IDLE_THROTTLE) / 1000.0 = (800 - 300) / 1000.0 = 0.5
            target_p->throttle = (radio_p->ch2 - 1000.0f) * 0.5f + IDLE_THROTTLE;

            if (target_p->throttle > MAX_TARGET_THROTTLE)
                target_p->throttle = MAX_TARGET_THROTTLE;
            else if (target_p->throttle < IDLE_THROTTLE)
                target_p->throttle = IDLE_THROTTLE;
        }
    }
}

static void inner_control_loop() // 1000Hz
{
    // ↓↓↓↓↓↓↓↓↓↓   CALCULATE CURRENT ERROR   ↓↓↓↓↓↓↓↓↓↓
    pid.errPitch = target_p->pitch_degs - state_p->pitch_dps;
    pid.errRoll = target_p->roll_degs - state_p->roll_dps;
    pid.errYaw = target_p->yaw_degs - state_p->yaw_dps;
    // ↑↑↑↑↑↑↑↑↑↑   CALCULATE CURRENT ERROR   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   PITCH P CALCULATION   ↓↓↓↓↓↓↓↓↓↓
    pid.pitchPout = pid.errPitch * config_p->pitch_p;
    // ↑↑↑↑↑↑↑↑↑↑   PITCH P CALCULATION   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   ROLL P CALCULATION   ↓↓↓↓↓↓↓↓↓↓
    pid.rollPout = pid.errRoll * config_p->roll_p;
    // ↑↑↑↑↑↑↑↑↑↑   ROLL P CALCULATION   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   YAW P CALCULATION   ↓↓↓↓↓↓↓↓↓↓
    pid.yawPout = pid.errYaw * config_p->yaw_p;
    // ↑↑↑↑↑↑↑↑↑↑   YAW P CALCULATION   ↑↑↑↑↑↑↑↑↑↑

    if (target_p->throttle > 320)
    {
        if (fabs(state_p->pitch_dps) < 25.0f)
        {
            // ↓↓↓↓↓↓↓↓↓↓  PITCH I CALCULATION   ↓↓↓↓↓↓↓↓↓↓
            pid.pitchIout += config_p->pitch_i * 0.000625f * (pid.errPitch + pid.errPitchPrev); //  0.000625 = 0.5 * sampleTime
            if (pid.pitchIout > maxI)
                pid.pitchIout = maxI;
            else if (pid.pitchIout < -maxI)
                pid.pitchIout = -maxI;
            // ↑↑↑↑↑↑↑↑↑↑   PITCH I CALCULATION   ↑↑↑↑↑↑↑↑↑↑
        }

        if (fabs(state_p->roll_dps) < 25.0f)
        {
            // ↓↓↓↓↓↓↓↓↓↓   ROLL I CALCULATION   ↓↓↓↓↓↓↓↓↓↓
            pid.rollIout += config_p->roll_i * 0.000625f * (pid.errRoll + pid.errRollPrev);
            if (pid.rollIout > maxI)
                pid.rollIout = maxI;
            else if (pid.rollIout < -maxI)
                pid.rollIout = -maxI;
            // ↑↑↑↑↑↑↑↑↑↑   ROLL I CALCULATION   ↑↑↑↑↑↑↑↑↑↑
        }

        // ↓↓↓↓↓↓↓↓↓↓   YAW I CALCULATION   ↓↓↓↓↓↓↓↓↓↓
        pid.yawIout += config_p->yaw_i * 0.000625f * (pid.errYaw + pid.errYawPrev);
        if (pid.yawIout > maxI)
            pid.yawIout = maxI;
        else if (pid.yawIout < -maxI)
            pid.yawIout = -maxI;
        // ↑↑↑↑↑↑↑↑↑↑   YAW I CALCULATION   ↑↑↑↑↑↑↑↑↑↑
    }
    else
    {
        pid.pitchIout = 0.0f;
        pid.rollIout = 0.0f;
        pid.yawIout = 0.0f;
    }

    // ↓↓↓↓↓↓↓↓↓↓   PITCH D CALCULATION   ↓↓↓↓↓↓↓↓↓↓
    // pid.pitchDout = -config.pitch_d * (states.pitch_degs - pid.pitchDegsPrev);
    pid.pitchDout = config_p->pitch_d * (pid.errPitch - pid.errPitchPrev);
    // ↑↑↑↑↑↑↑↑↑↑   PITCH D CALCULATION   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   ROLL D CALCULATION   ↓↓↓↓↓↓↓↓↓↓
    // pid.rollDout = -config.roll_d * (states.roll_degs - pid.rollDegsPrev);
    pid.rollDout = config_p->roll_d * (pid.errRoll - pid.errRollPrev);
    // ↑↑↑↑↑↑↑↑↑↑   ROLL D CALCULATION   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   PITCH PID OUT   ↓↓↓↓↓↓↓↓↓↓
    fir_filter(&fir_pitch, &pid.pitchDout);
    pid.pitchPIDout = pid.pitchPout + pid.pitchIout + pid.pitchDout;
    if (pid.pitchPIDout > maxPID)
        pid.pitchPIDout = maxPID;
    else if (pid.pitchPIDout < -maxPID)
        pid.pitchPIDout = -maxPID;
    // ↑↑↑↑↑↑↑↑↑↑   PITCH PID OUT   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   ROLL PID OUT   ↓↓↓↓↓↓↓↓↓↓
    fir_filter(&fir_roll, &pid.rollDout);
    pid.rollPIDout = pid.rollPout + pid.rollIout + pid.rollDout;
    if (pid.rollPIDout > maxPID)
        pid.rollPIDout = maxPID;
    else if (pid.rollPIDout < -maxPID)
        pid.rollPIDout = -maxPID;
    // ↑↑↑↑↑↑↑↑↑↑   ROLL PID OUT   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   YAW PID OUT   ↓↓↓↓↓↓↓↓↓↓
    pid.yawPIout = pid.yawPout + pid.yawIout;
    if (pid.yawPIout > maxPID)
        pid.yawPIout = maxPID;
    else if (pid.yawPIout < -maxPID)
        pid.yawPIout = -maxPID;
    // ↑↑↑↑↑↑↑↑↑↑   YAW PID OUT   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   HOLD LAST ERROR FOR NEXT CALCULATION   ↓↓↓↓↓↓↓↓↓↓
    pid.errPitchPrev = pid.errPitch;
    pid.errRollPrev = pid.errRoll;
    pid.errYawPrev = pid.errYaw;
    pid.pitchDegsPrev = state_p->pitch_dps;
    pid.rollDegsPrev = state_p->roll_dps;
    pid.yawDegsPrev = state_p->yaw_dps;
    // ↑↑↑↑↑↑↑↑↑↑   HOLD LAST ERROR FOR NEXT CALCULATION   ↑↑↑↑↑↑↑↑↑↑

    //=============================================================================//
    //                              Altitude Controller
    //=============================================================================//

    if (flight_p->alt_hold_status == 1)
    {
        static uint8_t counter_alt_controller = 0;
        counter_alt_controller++;
        if (counter_alt_controller >= 20) // 50 Hz
        {
            counter_alt_controller = 0;

            // ↓↓↓↓↓↓↓↓↓↓  CALCULATE CURRENT ERROR   ↓↓↓↓↓↓↓↓↓↓
            pid.errVel_z = target_p->velocity_z_ms - state_p->vel_up_ms;
            // ↑↑↑↑↑↑↑↑↑↑   CALCULATE CURRENT ERROR   ↑↑↑↑↑↑↑↑↑↑

            // ↓↓↓↓↓↓↓↓↓↓  ALTITUDE P CALCULATION  ↓↓↓↓↓↓↓↓↓↓
            pid.altPout = pid.errVel_z * config_p->altitude_p;
            // ↑↑↑↑↑↑↑↑↑↑  ALTITUDE P CALCULATION   ↑↑↑↑↑↑↑↑↑↑

            // ↓↓↓↓↓↓↓↓↓↓  ALTITUDE I CALCULATION   ↓↓↓↓↓↓↓↓↓↓
            if (flight_p->takeoff_status == 0)
            {
                pid.altIout += config_p->altitude_i * 0.01f * (pid.errVel_z + pid.errVel_z_prev); //  0.01 = 0.5 * sampleTime
                if (pid.altIout > MAX_TARGET_THROTTLE)
                    pid.altIout = MAX_TARGET_THROTTLE;
                else if (pid.altIout < IDLE_THROTTLE)
                    pid.altIout = IDLE_THROTTLE;
            }
            // ↑↑↑↑↑↑↑↑↑↑   ALTITUDE I CALCULATION   ↑↑↑↑↑↑↑↑↑↑

            // ↓↓↓↓↓↓↓↓↓↓   ALTITUDE D CALCULATION   ↓↓↓↓↓↓↓↓↓↓
            // pid.altDout = -config.altitude_d * (states.velocity_z_ms - pid.velocity_z_ms_prev);
            pid.altDout = -config_p->altitude_d * (state_p->acc_up_ms2 + pid.acc_z_prev);
            pid.acc_z_prev = state_p->acc_up_ms2;
            // ↑↑↑↑↑↑↑↑↑↑   ALTITUDE D CALCULATION   ↑↑↑↑↑↑↑↑↑↑

            // Altitude_D_Term_Filter();

            // ↓↓↓↓↓↓↓↓↓↓   ALTITUDE PID OUT   ↓↓↓↓↓↓↓↓↓↓
            target_p->throttle = pid.altPout + pid.altIout + pid.altDout;
            if (target_p->throttle > MAX_TARGET_THROTTLE)
                target_p->throttle = MAX_TARGET_THROTTLE;
            else if (target_p->throttle < IDLE_THROTTLE)
                target_p->throttle = IDLE_THROTTLE;
            // ↑↑↑↑↑↑↑↑↑↑   ALTITUDE PID OUT   ↑↑↑↑↑↑↑↑↑↑

            // ↓↓↓↓↓↓↓↓↓↓   HOLD LAST ERROR FOR NEXT CALCULATION   ↓↓↓↓↓↓↓↓↓↓
            pid.errVel_z_prev = pid.errVel_z;
            pid.velocity_z_ms_prev = state_p->vel_up_ms;
            // ↑↑↑↑↑↑↑↑↑↑   HOLD LAST ERROR FOR NEXT CALCULATION   ↑↑↑↑↑↑↑↑↑↑
        }
    }

    float comp_target_thr = target_p->throttle;
    // polinomial is specific to this drone
    // collect some data while hovering
    // substract initial throttle value from all data it should start from 0
    // remove begining and end of the data
    // katsayilar = polyfit(batt_v, thr_zero, 2); this is the function for matlab (2 is for second order)
    if (telemetry_p->battery_voltage < 11.5f)
        comp_target_thr += telemetry_p->battery_voltage * -48.8911436f + 564.0f;

    float cosAngAbs = cosf(fabs(state_p->pitch_deg) * DEG_TO_RAD) * cosf(fabs(state_p->roll_deg) * DEG_TO_RAD);
    if (cosAngAbs != 0)
        comp_target_thr += ((1.0f / cosAngAbs) - 1.0f) * comp_target_thr;

    // ↓↓↓↓↓↓↓↓↓↓   MOTOR 1 (LEFT BOTTOM)   ↓↓↓↓↓↓↓↓↓↓
    thr.m1 = comp_target_thr - pid.pitchPIDout + pid.rollPIDout + pid.yawPIout;
    if (thr.m1 < min_throttle)
        thr.m1 = min_throttle;
    else if (thr.m1 > max_throttle)
        thr.m1 = max_throttle;
    // ↑↑↑↑↑↑↑↑↑↑   MOTOR 1 (LEFT BOTTOM)   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   MOTOR 2 (LEFT TOP)   ↓↓↓↓↓↓↓↓↓↓
    thr.m2 = comp_target_thr + pid.pitchPIDout + pid.rollPIDout - pid.yawPIout;
    if (thr.m2 < min_throttle)
        thr.m2 = min_throttle;
    else if (thr.m2 > max_throttle)
        thr.m2 = max_throttle;
    // ↑↑↑↑↑↑↑↑↑↑   MOTOR 2 (LEFT TOP)   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   MOTOR 3 (RIGHT BOTTOM)   ↓↓↓↓↓↓↓↓↓↓
    thr.m3 = comp_target_thr - pid.pitchPIDout - pid.rollPIDout - pid.yawPIout;
    if (thr.m3 < min_throttle)
        thr.m3 = min_throttle;
    else if (thr.m3 > max_throttle)
        thr.m3 = max_throttle;
    // ↑↑↑↑↑↑↑↑↑↑   MOTOR 3 (RIGHT BOTTOM)   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   MOTOR 4 (RIGHT TOP)   ↓↓↓↓↓↓↓↓↓↓
    thr.m4 = comp_target_thr + pid.pitchPIDout - pid.rollPIDout + pid.yawPIout;
    if (thr.m4 < min_throttle)
        thr.m4 = min_throttle;
    else if (thr.m4 > max_throttle)
        thr.m4 = max_throttle;
    // ↑↑↑↑↑↑↑↑↑↑   MOTOR 4 (RIGHT TOP)   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   OUTPUT TO THE MOTORS   ↓↓↓↓↓↓↓↓↓↓
    //write_throttle(0, 0, 0, 0);
    write_throttle((uint16_t)(thr.m1 * 2.0f), (uint16_t)(thr.m2 * 2.0f), (uint16_t)(thr.m3 * 2.0f), (uint16_t)(thr.m4 * 2.0f));
    // ↑↑↑↑↑↑↑↑↑↑   OUTPUT TO THE MOTORS   ↑↑↑↑↑↑↑↑↑↑
}

static void arm()
{
    target_p->throttle = IDLE_THROTTLE;
    pid.pitchDegsPrev = state_p->pitch_dps;
    pid.rollDegsPrev = state_p->roll_dps;
    pid.yawDegsPrev = state_p->yaw_dps;

    pid.velocity_x_ms_prev = state_p->vel_forward_ms;
    pid.velocity_y_ms_prev = state_p->vel_right_ms;
    pid.velocity_z_ms_prev = state_p->vel_up_ms;

    target_p->pitch = 0;
    target_p->pitch_degs = 0;

    target_p->roll = 0;
    target_p->roll_degs = 0;

    target_p->heading = state_p->heading_deg;
    target_p->yaw_degs = 0;

    target_p->altitude = 0;
    target_p->velocity_x_ms = 0;
    target_p->velocity_y_ms = 0;
    target_p->velocity_z_ms = 0;

    pid.pitchIout = 0;
    pid.rollIout = 0;
    pid.yawIout = 0;
    pid.altIout = 0;
    pid.posXIout = 0;
    pid.posYIout = 0;

    pid.errPitchPrev = 0;
    pid.errRollPrev = 0;
    pid.errYawPrev = 0;

    gps_p->latitude_origin = gps_p->latitude;
    gps_p->longitude_origin = gps_p->longitude;
    gps_p->altitude_origin_mm = gps_p->altitude_mm;
    gps_p->longitude_scale = fabs(cosf(gps_p->longitude * DEG_TO_RAD) * 1.1f);

    // if altitude hold mode enabled before arming
    // then when armed, automatically takeoff pre determined altitude
    flight_p->takeoff_status = 0;
    if (flight_p->alt_hold_status == 1)
    {
        flight_p->takeoff_status = 1;
        pid.altIout = IDLE_THROTTLE;
    }

    telemetry_p->arm_status = 1;
    flight_p->arm_status = 1;
}

static void disarm()
{
    telemetry_p->arm_status = 0;
    flight_p->arm_status = 0;

    write_throttle(0, 0, 0, 0);
}

static int16_t apply_deadband(int16_t input, uint8_t deadband)
{
    if (input > deadband || input < -deadband)
    {
        return (input > 0) ? (input - deadband) : (input + deadband);
    }
    else
    {
        return 0;
    }
}
