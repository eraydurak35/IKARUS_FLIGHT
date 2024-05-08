#include "guidance_control.h"
#include "esc.h"
#include "filters.h"
#include "math.h"
#include "typedefs.h"

#define BENCH_MODE 0

static ibus_t *radio_p;
static flight_t *flight_p;
static target_t *target_p;
static states_t *state_p;
static telemetry_t *telemetry_p;
static config_t *config_p;
static waypoint_t *waypoint_p;
static gps_t *gps_p;
static home_point_t home;
static pid_bb_record_t *pid_bb;

struct PID
{
    float errPitch, errRoll, errYaw;
    float errPitchPrev, errRollPrev, errYawPrev;
    float pitchDegsPrev, rollDegsPrev, yawDegsPrev;
    float pitchPout, rollPout, yawPout;
    float pitchIout, rollIout, yawIout;
    float pitchDout, rollDout, yawDout;
    float pitch_ff_out, roll_ff_out, yaw_ff_out;
    float pitchPIDout, rollPIDout, yawPIout;
    float errVel_x, errVel_x_prev, velocity_x_ms_prev;
    float errVel_y, errVel_y_prev, velocity_y_ms_prev;
    float errVel_z, errVel_z_prev, velocity_z_ms_prev;
    float altPout, altIout, altDout;
    float posXPout, posXIout;
    float posYPout, posYIout;
};
static struct PID pid;
struct throttle
{
    float m1, m2, m3, m4;
};

static struct throttle thr;
static biquad_lpf_t lpf_pitch_d_term;
static biquad_lpf_t lpf_roll_d_term;
static biquad_lpf_t lpf_yaw_p_term;
static target_location_t target_location;

static int16_t deriv_rc_ch0 = 0;
static int16_t prev_rc_ch0 = 0;
static int16_t deriv_rc_ch1 = 0;
static int16_t prev_rc_ch1 = 0;
static int16_t deriv_rc_ch3 = 0;
static int16_t prev_rc_ch3 = 0;

static uint8_t use_gps_hold = 0;
static uint8_t is_hold_location_set = 0;
static uint8_t disarmed_by_landing = 0;

static int16_t apply_deadband(int16_t input, uint16_t deadband);
static uint8_t navigation_controller();
static uint8_t outer_control_loop_rth();
static uint8_t outer_control_loop_wp();
static void outer_control_loop_rc(uint8_t land_flag);
static void inner_control_loop();
static void arm();
static void disarm();
static void get_distance_bearing(target_location_t *tar_loc, int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2);
static uint8_t landing_detector(uint8_t need_reset);
static void limit_symmetric(float *value, float limit);
static float calculate_target_yaw_degs_from_target_heading_deg();
static float calculate_target_pitch_degs_from_target_pitch_deg();
static float calculate_target_roll_degs_from_target_roll_deg();

void control_init(ibus_t *rc, telemetry_t *tlm, flight_t *flt, target_t *trg, states_t *stt, config_t *cfg, waypoint_t *wp, gps_t *g, pid_bb_record_t *bb)
{
    radio_p = rc;
    flight_p = flt;
    target_p = trg;
    state_p = stt;
    telemetry_p = tlm;
    config_p = cfg;
    waypoint_p = wp;
    gps_p = g;
    pid_bb = bb;

    biquad_lpf_configure(D_TERM_CUTOFF_FREQ, &lpf_pitch_d_term);
    biquad_lpf_configure(D_TERM_CUTOFF_FREQ, &lpf_roll_d_term);
    biquad_lpf_configure(P_YAW_CUTOFF_FREQ, &lpf_yaw_p_term);
}

uint8_t flight_mode_control()
{
    uint8_t value = 0;
    // ||==============================================||
    // ||                 RC ARM LOGIC                 ||
    // ||==============================================||
    if (radio_p->ch4 > 1700 && flight_p->arm_status == 0 && disarmed_by_landing == 0)
    {
        // if alt hold is on we want throttle stick in the middle else zero
        if ((flight_p->alt_hold_status == 1 && (radio_p->ch2 < 1600 && radio_p->ch2 > 1400)) || (flight_p->alt_hold_status == 0 && radio_p->ch2 < 1100))
        {
            // if RTH stick is not active
            if (!(radio_p->ch5 > 1400 && radio_p->ch5 < 1600))
            {
                arm();
                value = 1;
            }
        }
    }
    else if (radio_p->ch4 < 1300 && (flight_p->arm_status == 1 || disarmed_by_landing == 1))
    {
        disarm();
        disarmed_by_landing = 0;
        value = 2;
        /////////////////////////////////
        //    RESET OTHER FUNCTIONS    // 
        flight_p->alt_hold_status = 0;
        flight_p->pos_hold_status = 0;
        use_gps_hold = 0;
        is_hold_location_set = 0;
        flight_p->waypoint_mission_status = 0;
        waypoint_p->is_reached = 1;
        waypoint_p->counter = 0;
        target_p->latitude = gps_p->latitude;
        target_p->longitude = gps_p->longitude;
        target_p->altitude = state_p->altitude_m;
        /////////////////////////////////
    }
    // ||==============================================||
    // ||              RC ALT HOLD LOGIC               ||
    // ||==============================================||
    if ((radio_p->ch7 > 1700 && flight_p->alt_hold_status == 0) && flight_p->rth_status == 0)
    {
        flight_p->alt_hold_status = 1;
        pid.altIout = target_p->throttle;
        target_p->altitude = state_p->altitude_m;
    }
    else if ((radio_p->ch7 < 1300 && flight_p->alt_hold_status == 1) && flight_p->rth_status == 0)
    {
        flight_p->alt_hold_status = 0;
    }

    // ||==============================================||
    // ||              RC POS HOLD LOGIC               ||
    // ||==============================================||
    if ((radio_p->ch6 > 1400 &&
         flight_p->pos_hold_status == 0) && flight_p->rth_status == 0)
    {
        flight_p->pos_hold_status = 1;
        if (telemetry_p->is_gnss_sanity_check_ok == 1)
        {
            target_p->latitude = gps_p->latitude;
            target_p->longitude = gps_p->longitude;
            use_gps_hold = 1;
            is_hold_location_set = 1;
        }
    }
    else if ((radio_p->ch6 < 1300 && flight_p->pos_hold_status == 1) && flight_p->rth_status == 0)
    {
        flight_p->pos_hold_status = 0;
        use_gps_hold = 0;
        is_hold_location_set = 0;
    }

    // ||==============================================||
    // ||              RC WAYPOINT LOGIC               ||
    // ||==============================================||
    if ((radio_p->ch6 > 1800) &&
        flight_p->waypoint_mission_status == 0 &&
        flight_p->arm_status == 1 &&
        flight_p->pos_hold_status == 1 &&
        flight_p->alt_hold_status == 1 &&
        flight_p->is_takeoff_done == 1 &&
        waypoint_p->latitude[0] != 0 &&
        waypoint_p->longitude[0] != 0 &&
        waypoint_p->altitude[0] != 0 &&
        flight_p->rth_status == 0 &&
        telemetry_p->is_gnss_sanity_check_ok == 1)
    {
        flight_p->waypoint_mission_status = 1;
        flight_p->is_rth_done = 0;
    }
    else if ((flight_p->waypoint_mission_status == 1 &&
              (!(radio_p->ch6 > 1800) || flight_p->pos_hold_status == 0 || flight_p->alt_hold_status == 0)) &&
             flight_p->rth_status == 0)
    {
        flight_p->waypoint_mission_status = 0;
        waypoint_p->is_reached = 1;
        waypoint_p->counter = 0;
        target_p->latitude = gps_p->latitude;
        target_p->longitude = gps_p->longitude;
        target_p->altitude = state_p->altitude_m;
    }
    // ||==============================================||
    // ||                RC RTH LOGIC                  ||
    // ||==============================================||
    if ((radio_p->ch5 > 1400 && radio_p->ch5 < 1600) &&
        flight_p->rth_status == 0 &&
        flight_p->arm_status == 1)
    {
        flight_p->rth_status = 1;
        flight_p->is_rth_done = 0;
        flight_p->alt_hold_status = 1;
        pid.altIout = target_p->throttle;

        if (home.latitude != 0 && home.longitude != 0)
        {
            target_p->latitude = home.latitude;
            target_p->longitude = home.longitude;
        }
        else if (telemetry_p->is_gnss_sanity_check_ok == 1)
        {
            target_p->latitude = gps_p->latitude;
            target_p->longitude = gps_p->longitude;
            use_gps_hold = 1;
            is_hold_location_set = 1;
        }
        target_p->altitude = state_p->altitude_m;
        flight_p->pos_hold_status = 1;
    }
    else if (!(radio_p->ch5 > 1400 && radio_p->ch5 < 1600) && flight_p->rth_status == 1)
    {
        flight_p->rth_status = 0;

        if (flight_p->waypoint_mission_status == 1 && !(waypoint_p->is_reached == 1 && (waypoint_p->counter >= 25 || (waypoint_p->latitude[waypoint_p->counter] == 0 || waypoint_p->longitude[waypoint_p->counter] == 0))))
        {
            target_p->latitude = waypoint_p->latitude[waypoint_p->counter - 1];
            target_p->longitude = waypoint_p->longitude[waypoint_p->counter - 1];
            target_p->altitude = waypoint_p->altitude[waypoint_p->counter - 1] / 10.0f;
        }
        else if ((radio_p->ch6 > 1400 && radio_p->ch6 < 1600) && telemetry_p->is_gnss_sanity_check_ok == 1)
        {
            target_p->latitude = gps_p->latitude;
            target_p->longitude = gps_p->longitude;
            use_gps_hold = 1;
            is_hold_location_set = 1;
        }
        target_p->altitude = state_p->altitude_m;
    }

    return value;
}

void flight_control() // 1000Hz
{
    if (flight_p->arm_status == 1)
    {
        // return to home function has highest priority
        if (flight_p->rth_status == 1)
        {
            // if home locaiton not set
            if (home.latitude == 0 || home.longitude == 0)
            {
                // regular control, autoland flag set
                outer_control_loop_rc(1);
            }
            else if (outer_control_loop_rth() == 0)
            {
                // home reached or gps not reliable,
                // switch to regular control, autoland flag set
                outer_control_loop_rc(1);
            }
        }
        else if (flight_p->waypoint_mission_status == 1)
        {
            static uint8_t prev_ret = 1;
            // ret = 0 --> gnss err or fall to RC control after end of mission
            // ret = 1 --> wp mission is ongoing
            // ret = 2 --> wp mission ended. rth without autolanding (origin is set)
            // ret = 3 --> wp mission ended. rth with autolanding (origin is set)
            uint8_t ret = outer_control_loop_wp();

            // check if end of mission behaviour is RTH 
            if ((prev_ret == 1 && ret == 2) || (prev_ret == 1 && ret == 3) || (prev_ret == 0 && ret == 2) || (prev_ret == 0 && ret == 3))
            {
                target_p->latitude = home.latitude;
                target_p->longitude = home.longitude;
            }
            prev_ret = ret;

            if (ret == 0)
            {
                // wp list ended or gps not reliable,
                // switch to regular control, autoland flag not set
                outer_control_loop_rc(0);
            }
            else if (ret == 2)
            {
                if (outer_control_loop_rth() == 0)
                {
                    outer_control_loop_rc(0);
                }
            }
            else if (ret == 3)
            {
                if (outer_control_loop_rth() == 0)
                {
                    outer_control_loop_rc(1);
                }
            }
        }
        else
        {
            // regular control, autoland not set
            outer_control_loop_rc(0);
        }

        inner_control_loop();
    }
    else
    {
        disarm();
    }
}

static uint8_t outer_control_loop_rth() // 1000 Hz
{
    if (flight_p->is_rth_done == 1 || telemetry_p->is_gnss_sanity_check_ok == 0)
        return 0;

    if (navigation_controller() == 1)
    {
        flight_p->is_rth_done = 1;
        return 0;
    }
    return 1;
}

static uint8_t outer_control_loop_wp() // 1000 Hz
{
    // gnss not available. Return early. Fall to manual RC control
    if (telemetry_p->is_gnss_sanity_check_ok == 0) return 0;
    // end of mission detected. select what to do next
    else if (waypoint_p->is_reached == 1 && (waypoint_p->counter >= 25 || (waypoint_p->latitude[waypoint_p->counter] == 0 || waypoint_p->longitude[waypoint_p->counter] == 0)))
    {
        if (waypoint_p->end_of_mission_behaviour == 0) return 0;                                                     // fall to RC manual control (passive position hold)
        else if (waypoint_p->end_of_mission_behaviour == 1 && home.latitude != 0 && home.longitude != 0) return 2;   // activate rth without autoland
        else if (waypoint_p->end_of_mission_behaviour == 2 && home.latitude != 0 && home.longitude != 0) return 3;   // activate rth with autoland
        else return 0;
    }

    // set initial or next wp location as target
    if (waypoint_p->is_reached == 1)
    {
        target_p->latitude = waypoint_p->latitude[waypoint_p->counter];
        target_p->longitude = waypoint_p->longitude[waypoint_p->counter];
        target_p->altitude = waypoint_p->altitude[waypoint_p->counter] / 10.0f;
        waypoint_p->is_reached = 0;
        waypoint_p->counter++;
    }
    else
    {
        if (navigation_controller() == 1)
        {
            waypoint_p->is_reached = 1;
            return 1;
        }
    }
    return 1;
}


// This function should only be called if gnss reliable
static uint8_t navigation_controller() // 100 Hz
{
    static float calculated_heading_correction = 0;
    static float gps_heading_wp_heading_diff = 0;
    static uint8_t counter = 0;

    counter++;
    if (counter >= 10)
    {
        counter = 0;

        get_distance_bearing(&target_location, target_p->latitude, target_p->longitude, gps_p->latitude, gps_p->longitude);
        telemetry_p->distance_m_2d = target_location.distance_cm / 100.0f;

        // we are away from wp threshold
        if (target_location.distance_cm > config_p->wp_threshold_cm)
        {
            // if velocity bigger than 1m/s gnss valocity and head_of_motion becomes reliable
            // we can thrust gnss to calculate wind and declination corrected heading.
            if (sqrtf(gps_p->northVel_mms * gps_p->northVel_mms + gps_p->eastVel_mms * gps_p->eastVel_mms) > 1000.0f)
            {
                gps_heading_wp_heading_diff = target_location.bearing_deg - (gps_p->headingOfMotion / 100000.0f);

                if (gps_heading_wp_heading_diff < -180.0f) gps_heading_wp_heading_diff += 360.0f;
                else if (gps_heading_wp_heading_diff > 180.0f) gps_heading_wp_heading_diff -= 360.0f;

                if (fabs(gps_heading_wp_heading_diff) < 30.0f)
                    calculated_heading_correction += (gps_heading_wp_heading_diff - calculated_heading_correction) * config_p->wp_heading_correct_gain;
            }

            target_p->heading = target_location.bearing_deg + calculated_heading_correction;

            if (target_p->heading >= 360.0f) target_p->heading -= 360.0f;
            else if (target_p->heading < 0.0f) target_p->heading += 360.0f;

            target_p->yaw_degs = calculate_target_yaw_degs_from_target_heading_deg();

/*             // Calculate yaw rate from setpoint error
            target_p->yaw_degs = (target_p->heading - state_p->heading_deg) * config_p->yaw_rate_scale;
            // This part ensures the craft turns from closest side to setpoint
            if (target_p->yaw_degs < -180.0f * config_p->yaw_rate_scale) target_p->yaw_degs += 360.0f * config_p->yaw_rate_scale;
            else if (target_p->yaw_degs > 180.0f * config_p->yaw_rate_scale) target_p->yaw_degs -= 360.0f * config_p->yaw_rate_scale; */

            // Just limit the yaw rate so it doesnt go nuts
            limit_symmetric(&target_p->yaw_degs, config_p->max_yaw_rate);

            float heading_diff = state_p->heading_deg - target_p->heading;

            if (heading_diff < -180.0f) heading_diff += 360.0f;
            else if (heading_diff > 180.0f) heading_diff -= 360.0f;

            if (fabs(heading_diff) < 10.0f) // increase this value to not stop every big heading change. > 180 disables it
            {
                float vel_from_dist = target_location.distance_cm / config_p->wp_dist_to_vel_gain; // gain = 300 -> 3m distance equals 1.0 m/s
                if (vel_from_dist > config_p->max_horizontal_velocity) vel_from_dist = config_p->max_horizontal_velocity;
                
                target_p->velocity_x_ms = vel_from_dist * cosf(heading_diff * DEG_TO_RAD);
                target_p->velocity_y_ms = vel_from_dist * cosf((heading_diff + 90.0f) * DEG_TO_RAD);

                limit_symmetric(&target_p->velocity_x_ms, config_p->max_horizontal_velocity);
                limit_symmetric(&target_p->velocity_y_ms, config_p->max_horizontal_velocity);
            }
            else
            {
                target_p->velocity_x_ms = 0;
                target_p->velocity_y_ms = 0;
            }
        }
        else
        {
            // return 1 indicates that the craft did reach to target location
            return 1;
        }

        pid.errVel_x = -(target_p->velocity_x_ms - state_p->vel_forward_ms);
        pid.errVel_y = target_p->velocity_y_ms - state_p->vel_right_ms;

        pid.posXPout = pid.errVel_x * config_p->position_p;
        pid.posYPout = pid.errVel_y * config_p->position_p;

        // output is saturated. dont wind up.
        if (fabs(target_p->pitch) < POS_CTRL_MAX_PITCH_ROLL_DEG)
        {
            pid.posXIout += config_p->position_i * 0.005f * (pid.errVel_x + pid.errVel_x_prev);
            limit_symmetric(&pid.posXIout, POS_I_CTRL_MAX);
        }

        // output is saturated. dont wind up.
        if (fabs(target_p->roll) < POS_CTRL_MAX_PITCH_ROLL_DEG)
        {
            pid.posYIout += config_p->position_i * 0.005f * (pid.errVel_y + pid.errVel_y_prev);
            limit_symmetric(&pid.posYIout, POS_I_CTRL_MAX);
        }

        pid.posXIout -= pid.posYIout * sinf(state_p->yaw_dps * DEG_TO_RAD * 0.01f);
        pid.posYIout += pid.posXIout * sinf(state_p->yaw_dps * DEG_TO_RAD * 0.01f);

        // smooth out requested pitch from autopilot
        target_p->pitch += ((pid.posXPout + pid.posXIout) - target_p->pitch) * 0.2f;
        limit_symmetric(&target_p->pitch, POS_CTRL_MAX_PITCH_ROLL_DEG);

        // smooth out requested roll from autopilot
        target_p->roll += ((pid.posYPout + pid.posYIout) - target_p->roll) * 0.2f;
        limit_symmetric(&target_p->roll, POS_CTRL_MAX_PITCH_ROLL_DEG);

        target_p->pitch_degs = calculate_target_pitch_degs_from_target_pitch_deg();
/*         /////////////////////   Pitch Rate Controller   ///////////////////////
        target_p->pitch_degs = (target_p->pitch - state_p->pitch_deg) * config_p->pitch_rate_scale;
        limit_symmetric(&target_p->pitch_degs, config_p->max_pitch_rate); */

        target_p->roll_degs = calculate_target_roll_degs_from_target_roll_deg();
/*         /////////////////////   Roll Rate Controller   ///////////////////////
        target_p->roll_degs = (target_p->roll - state_p->roll_deg) * config_p->roll_rate_scale;
        limit_symmetric(&target_p->roll_degs, config_p->max_roll_rate); */

        ////////////////////////    Altitude Velocity Controller    /////////////////////////////////
        // Throttle controls the altitude velocity
        if (radio_p->ch2 < 1550 && radio_p->ch2 > 1450) // Throttle stick is centered, velocity calculated from setpoint error
        {
            // Target velocity is calculated from dividing the difference between set altitude and actual altitude with a constant value
            target_p->velocity_z_ms = (target_p->altitude - state_p->altitude_m) * config_p->alt_to_vel_gain;
            limit_symmetric(&target_p->velocity_z_ms, config_p->max_vertical_velocity);
        }
        else // Throttle stick not centered, velocity calculated from stick input
        {
            // Calculate the desired altitude velocity from raw stick input
            target_p->velocity_z_ms = (apply_deadband(radio_p->ch2 - 1500.0f, 50) / 450.0f) * config_p->max_vertical_velocity;
            // we dont use altitude setpoint if the stick is not centered
            // but we want to set our setpoint when the stick is in middle
            // so that when we let go of the stick, craft stays at the altitude we let go
            target_p->altitude = state_p->altitude_m;
        }
    }

    // return 0 indicates navigation is in progress. craft did not reach the target location
    return 0;
}

static void outer_control_loop_rc(uint8_t land_flag)
{
    static uint8_t counter_set_point = 0;
    static uint8_t prev_land_flag = 0;

    counter_set_point++;
    if (counter_set_point >= 10) // 100Hz
    {
        counter_set_point = 0;

        // stick feed forward calculation
        deriv_rc_ch0 = (radio_p->ch0 - prev_rc_ch0);
        prev_rc_ch0 = radio_p->ch0;
        deriv_rc_ch1 = (radio_p->ch1 - prev_rc_ch1);
        prev_rc_ch1 = radio_p->ch1;
        deriv_rc_ch3 = (radio_p->ch3 - prev_rc_ch3);
        prev_rc_ch3 = radio_p->ch3;

        pid.pitch_ff_out += ((deriv_rc_ch1 * config_p->ff_gain) - pid.pitch_ff_out) * 0.4f;
        pid.roll_ff_out += ((deriv_rc_ch0 * config_p->ff_gain) - pid.roll_ff_out) * 0.4f;
        pid.yaw_ff_out += ((deriv_rc_ch3 * config_p->ff_gain) - pid.yaw_ff_out) * 0.4f;

        // detect start of land command
        if (prev_land_flag == 0 && land_flag == 1)
        {
            // set heading as takeoff heading (can be overwritten by yaw stick)
            target_p->heading = home.heading_deg;
            // reset landing detector
            landing_detector(1);
        }
        prev_land_flag = land_flag;

        if (flight_p->pos_hold_status == 1)
        {
            // pitch / roll stick centered
            if ((radio_p->ch1 > 1450 && radio_p->ch1 < 1550) && (radio_p->ch0 > 1450 && radio_p->ch0 < 1550))
            {
                // check if active gps hold can be used
                if (use_gps_hold == 1 && telemetry_p->is_gnss_sanity_check_ok == 1)
                {
                    // we hold the current location if hold location is selected
                    if (is_hold_location_set == 1)
                    {
                        get_distance_bearing(&target_location, target_p->latitude, target_p->longitude, gps_p->latitude, gps_p->longitude);
                        telemetry_p->distance_m_2d = target_location.distance_cm / 100.0f;

                        // we are outside hold threshold 
                        if (target_location.distance_cm > config_p->wp_threshold_cm / 1.5f)
                        {
                            target_p->velocity_x_ms = (cosf(state_p->heading_deg * DEG_TO_RAD) * target_location.distance_north_cm + cosf((state_p->heading_deg - 90.0f) * DEG_TO_RAD) * target_location.distance_east_cm) / (config_p->wp_dist_to_vel_gain * 2.0f);
                            target_p->velocity_y_ms = (cosf(state_p->heading_deg * DEG_TO_RAD) * target_location.distance_east_cm + cosf((state_p->heading_deg + 90.0f) * DEG_TO_RAD) * target_location.distance_north_cm) / (config_p->wp_dist_to_vel_gain * 2.0f);

                            limit_symmetric(&target_p->velocity_x_ms, config_p->max_horizontal_velocity);
                            limit_symmetric(&target_p->velocity_y_ms, config_p->max_horizontal_velocity);
                        }
                        else
                        {
                            target_p->velocity_x_ms = 0;
                            target_p->velocity_y_ms = 0;
                        }
                    }
                    else
                    {
                        target_p->velocity_x_ms = 0;
                        target_p->velocity_y_ms = 0;
                        // we want to select a hold location if we are not moving fast
                        // if we choose a hold location while moving fast, we owershoot
                        // to prevent this first we have to slow down
                        if (fabs(state_p->vel_forward_ms) < 0.5 && fabs(state_p->vel_right_ms) < 0.5)
                        {
                            is_hold_location_set = 1;
                            target_p->latitude = gps_p->latitude;
                            target_p->longitude = gps_p->longitude;
                        }
                    }
                }
                else
                {   
                    // we dont have a good gps use passive hold
                    is_hold_location_set = 0;
                    target_p->velocity_x_ms = 0;
                    target_p->velocity_y_ms = 0;
                }
            }
            else    // pitch / roll stick not centered
            {
                is_hold_location_set = 0;
                // manual valocity control
                target_p->velocity_x_ms = ((radio_p->ch1 - 1500) / 500.0f) * config_p->max_horizontal_velocity;
                target_p->velocity_y_ms = ((radio_p->ch0 - 1500) / 500.0f) * config_p->max_horizontal_velocity;

                if (use_gps_hold == 1 && telemetry_p->is_gnss_sanity_check_ok == 1)
                {
                    target_p->latitude = gps_p->latitude;
                    target_p->longitude = gps_p->longitude;
                }
            }

            pid.errVel_x = -(target_p->velocity_x_ms - state_p->vel_forward_ms);
            pid.errVel_y = target_p->velocity_y_ms - state_p->vel_right_ms;

            pid.posXPout = pid.errVel_x * config_p->position_p;
            pid.posYPout = pid.errVel_y * config_p->position_p;

            // output is saturated. dont wind up.
            if (fabs(target_p->pitch) < POS_CTRL_MAX_PITCH_ROLL_DEG)
            {
                pid.posXIout += config_p->position_i * 0.005f * (pid.errVel_x + pid.errVel_x_prev);
                limit_symmetric(&pid.posXIout, POS_I_CTRL_MAX);
            }

            // output is saturated. dont wind up.
            if (fabs(target_p->roll) < POS_CTRL_MAX_PITCH_ROLL_DEG)
            {
                pid.posYIout += config_p->position_i * 0.005f * (pid.errVel_y + pid.errVel_y_prev);
                limit_symmetric(&pid.posYIout, POS_I_CTRL_MAX);
            }

            pid.posXIout -= pid.posYIout * sinf(state_p->yaw_dps * DEG_TO_RAD * 0.01f);
            pid.posYIout += pid.posXIout * sinf(state_p->yaw_dps * DEG_TO_RAD * 0.01f);

            // smooth out requested pitch from autopilot
            target_p->pitch += ((pid.posXPout + pid.posXIout) - target_p->pitch) * 0.2f;
            limit_symmetric(&target_p->pitch, POS_CTRL_MAX_PITCH_ROLL_DEG);

            // smooth out requested roll from autopilot
            target_p->roll += ((pid.posYPout + pid.posYIout) - target_p->roll) * 0.2f;
            limit_symmetric(&target_p->roll, POS_CTRL_MAX_PITCH_ROLL_DEG);
        }
        else
        {
            target_p->pitch = (apply_deadband(radio_p->ch1 - 1500, 20) / -480.0f) * config_p->max_pitch_angle;
            target_p->roll = (apply_deadband(radio_p->ch0 - 1500, 20) / 480.0f) * config_p->max_roll_angle;
        }

        target_p->pitch_degs = calculate_target_pitch_degs_from_target_pitch_deg();
/*         /////////////////////   Pitch Rate Controller   ///////////////////////
        target_p->pitch_degs = (target_p->pitch - state_p->pitch_deg) * config_p->pitch_rate_scale;
        limit_symmetric(&target_p->pitch_degs, config_p->max_pitch_rate); */

        target_p->roll_degs = calculate_target_roll_degs_from_target_roll_deg();
/*         /////////////////////   Roll Rate Controller   ///////////////////////
        target_p->roll_degs = (target_p->roll - state_p->roll_deg) * config_p->roll_rate_scale;
        limit_symmetric(&target_p->roll_degs, config_p->max_roll_rate); */

        /////////////////////   Yaw Rate Controller   ///////////////////////
        // Yaw stick controls yaw rate
        if (radio_p->ch3 < 1550 && radio_p->ch3 > 1450) // Yaw stick is centered
        {
            target_p->yaw_degs = calculate_target_yaw_degs_from_target_heading_deg();
            /*          
            // Calculate yaw rate from setpoint error
            target_p->yaw_degs = (target_p->heading - state_p->heading_deg) * config_p->yaw_rate_scale;

            // This part ensures the craft turns from closest side to setpoint
            // Say the setpoint is 5 deg and craft is at 270, logical thing is craft turns clockwise 95 deg
            // If we dont do this craft will attempt to turn counter clockwise 265deg
            if (target_p->yaw_degs < -180.0f * config_p->yaw_rate_scale) target_p->yaw_degs += 360.0f * config_p->yaw_rate_scale;
            else if (target_p->yaw_degs > 180.0f * config_p->yaw_rate_scale) target_p->yaw_degs -= 360.0f * config_p->yaw_rate_scale; */
        }
        else // Yaw stick is not centered
        {
            target_p->yaw_degs = ((radio_p->ch3 - 1500.0f) / 500.0f) * config_p->max_yaw_rate;
            target_p->heading = state_p->heading_deg;
        }
        // limit the yaw rate so it doesn't go crazy
        limit_symmetric(&target_p->yaw_degs, config_p->max_yaw_rate);

        if (flight_p->alt_hold_status == 1)
        {
            if (flight_p->takeoff_status == 1)
            {   
                // ramp up throttle to hover value slowly for takeoff
                pid.altIout += (config_p->hover_throttle - IDLE_THROTTLE) / 100.0f;

                // when i value bigger than hover, set target altitude and let altitude controller take over
                if (pid.altIout >= config_p->hover_throttle)
                {
                    target_p->altitude = config_p->takeoff_altitude;
                    flight_p->takeoff_status = 0;
                }
            }
            else if (land_flag == 0) // autoland off / Normal Z velocity control
            {
                ////////////////////////    Altitude Velocity Controller    /////////////////////////////////
                // Throttle controls the altitude velocity
                if (radio_p->ch2 < 1550 && radio_p->ch2 > 1450) // Throttle stick is centered, velocity calculated from setpoint error
                {
                    // Target velocity is calculated from dividing the difference between set altitude and actual altitude with a constant value
                    target_p->velocity_z_ms = (target_p->altitude - state_p->altitude_m) * config_p->alt_to_vel_gain;
                    limit_symmetric(&target_p->velocity_z_ms, config_p->max_vertical_velocity);

                    if (flight_p->is_takeoff_done == 0 && (target_p->altitude - state_p->altitude_m) < 0.4f)
                    {
                        flight_p->is_takeoff_done = 1;
                    }
                }
                else // Throttle stick not centered, velocity calculated from stick input
                {
                    flight_p->is_takeoff_done = 1;
                    // 450 = 500 - deadband{50}
                    // Calculate the desired altitude velocity from raw stick input
                    target_p->velocity_z_ms = (apply_deadband(radio_p->ch2 - 1500.0f, 50) / 450.0f) * config_p->max_vertical_velocity;
                    // we dont use altitude setpoint if the stick is not centered
                    // but we want to set our setpoint when the stick is in middle
                    // so that when we let go of the stick, craft stays at the altitude we let go
                    target_p->altitude = state_p->altitude_m;
                }
            }
            else
            {
                // when landing is detected, disarm
                if (landing_detector(0) == 1)
                {
                    disarmed_by_landing = 1;
                    disarm();
                }
                // set target altitude to -1.0f to indicate landing
                target_p->altitude = -1.0f;
                // altitude > 0.3f we use normal descent
                if (state_p->altitude_m > 0.3f)
                {
                    target_p->velocity_z_ms += ((-state_p->altitude_m * config_p->alt_to_vel_gain) - target_p->velocity_z_ms) * 0.02f;
                }
                else
                {
                    // when we are close enough to ground we start killing the motors by slowly decreasing target velocity
                    target_p->velocity_z_ms -= 0.005f;
                }
                limit_symmetric(&target_p->velocity_z_ms, config_p->max_vertical_velocity);
            }

        }
        else
        {
            // (MAX_TARGET_THROTTLE - IDLE_THROTTLE) / 1000.0 = (800 - 300) / 1000.0 = 0.5
            target_p->throttle = (radio_p->ch2 - 1000.0f) * 0.5f + IDLE_THROTTLE;

            if (target_p->throttle > MAX_TARGET_THROTTLE) target_p->throttle = MAX_TARGET_THROTTLE;
            else if (target_p->throttle < IDLE_THROTTLE) target_p->throttle = IDLE_THROTTLE;
        }
    }
}

static void inner_control_loop() // 1000Hz
{
    static float filt_target_pitch_dps;
    static float filt_target_roll_dps;
    static float filt_target_yaw_dps;
    
    // coordinate yaw turn when pitch & roll not zero
    float target_pitch_dps_corrected = sinf(state_p->roll_deg * DEG_TO_RAD) * target_p->yaw_degs + target_p->pitch_degs;
    float target_roll_dps_corrected = sinf(-state_p->pitch_deg * DEG_TO_RAD) * target_p->yaw_degs + target_p->roll_degs;
    float target_yaw_dps_corrected = fabs(cosf(state_p->roll_deg * DEG_TO_RAD)) * fabs(cosf(state_p->pitch_deg * DEG_TO_RAD)) * target_p->yaw_degs;

    // limit angular acceleration
    float pitch_requested_angular_accel = ((target_pitch_dps_corrected - filt_target_pitch_dps) * 0.1f) * 1000.0f;
    float roll_requested_angular_accel = ((target_roll_dps_corrected - filt_target_roll_dps) * 0.1f) * 1000.0f;
    float yaw_requested_angular_accel = ((target_yaw_dps_corrected - filt_target_yaw_dps) * 0.1f) * 1000.0f;

    if (pitch_requested_angular_accel > MAX_ANGULAR_ACCEL) filt_target_pitch_dps += MAX_ANGULAR_ACCEL * 0.001f;
    else if (pitch_requested_angular_accel < -MAX_ANGULAR_ACCEL) filt_target_pitch_dps -= MAX_ANGULAR_ACCEL * 0.001f;
    else filt_target_pitch_dps += (target_pitch_dps_corrected - filt_target_pitch_dps) * 0.1f;

    if (roll_requested_angular_accel > MAX_ANGULAR_ACCEL) filt_target_roll_dps += MAX_ANGULAR_ACCEL * 0.001f;
    else if (roll_requested_angular_accel < -MAX_ANGULAR_ACCEL) filt_target_roll_dps -= MAX_ANGULAR_ACCEL * 0.001f;
    else filt_target_roll_dps += (target_roll_dps_corrected - filt_target_roll_dps) * 0.1f;

    if (yaw_requested_angular_accel > MAX_ANGULAR_ACCEL) filt_target_yaw_dps += MAX_ANGULAR_ACCEL * 0.001f;
    else if (yaw_requested_angular_accel < -MAX_ANGULAR_ACCEL) filt_target_yaw_dps -= MAX_ANGULAR_ACCEL * 0.001f;
    else filt_target_yaw_dps += (target_yaw_dps_corrected - filt_target_yaw_dps) * 0.1f;


    // ↓↓↓↓↓↓↓↓↓↓   CALCULATE CURRENT ERROR   ↓↓↓↓↓↓↓↓↓↓
    pid.errPitch = filt_target_pitch_dps - state_p->pitch_dps;
    pid.errRoll = filt_target_roll_dps - state_p->roll_dps;
    pid.errYaw = filt_target_yaw_dps - state_p->yaw_dps;
    // ↑↑↑↑↑↑↑↑↑↑   CALCULATE CURRENT ERROR   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   PITCH P CALCULATION   ↓↓↓↓↓↓↓↓↓↓
    pid.pitchPout = pid.errPitch * config_p->pitch_p;
    pid_bb->pitch_p_out = pid.pitchPout;
    // ↑↑↑↑↑↑↑↑↑↑   PITCH P CALCULATION   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   ROLL P CALCULATION   ↓↓↓↓↓↓↓↓↓↓
    pid.rollPout = pid.errRoll * config_p->roll_p;
    pid_bb->roll_p_out = pid.rollPout;
    // ↑↑↑↑↑↑↑↑↑↑   ROLL P CALCULATION   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   YAW P CALCULATION   ↓↓↓↓↓↓↓↓↓↓
    pid.yawPout = pid.errYaw * config_p->yaw_p;
    biquad_lpf(&lpf_yaw_p_term, &pid.yawPout);
    pid_bb->yaw_p_out = pid.yawPout;
    // ↑↑↑↑↑↑↑↑↑↑   YAW P CALCULATION   ↑↑↑↑↑↑↑↑↑↑

    if (target_p->throttle > 320)
    {
        if (fabs(target_pitch_dps_corrected) < 25.0f)
        {
            // ↓↓↓↓↓↓↓↓↓↓  PITCH I CALCULATION   ↓↓↓↓↓↓↓↓↓↓
            pid.pitchIout += config_p->pitch_i * 0.000625f * (pid.errPitch + pid.errPitchPrev); //  0.000625 = 0.5 * sampleTime
            limit_symmetric(&pid.pitchIout, MAX_I);
            // ↑↑↑↑↑↑↑↑↑↑   PITCH I CALCULATION   ↑↑↑↑↑↑↑↑↑↑
        }

        if (fabs(target_roll_dps_corrected) < 25.0f)
        {
            // ↓↓↓↓↓↓↓↓↓↓   ROLL I CALCULATION   ↓↓↓↓↓↓↓↓↓↓
            pid.rollIout += config_p->roll_i * 0.000625f * (pid.errRoll + pid.errRollPrev);
            limit_symmetric(&pid.rollIout, MAX_I);
            // ↑↑↑↑↑↑↑↑↑↑   ROLL I CALCULATION   ↑↑↑↑↑↑↑↑↑↑
        }

        // ↓↓↓↓↓↓↓↓↓↓   YAW I CALCULATION   ↓↓↓↓↓↓↓↓↓↓
        pid.yawIout += config_p->yaw_i * 0.000625f * (pid.errYaw + pid.errYawPrev);
        limit_symmetric(&pid.yawIout, MAX_I);
        // ↑↑↑↑↑↑↑↑↑↑   YAW I CALCULATION   ↑↑↑↑↑↑↑↑↑↑
    }
    else
    {
        pid.pitchIout = 0.0f;
        pid.rollIout = 0.0f;
        pid.yawIout = 0.0f;
    }
    pid_bb->pitch_i_out = pid.pitchIout;
    pid_bb->roll_i_out = pid.rollIout;
    pid_bb->yaw_i_out = pid.yawIout;

    // ↓↓↓↓↓↓↓↓↓↓   PITCH D CALCULATION   ↓↓↓↓↓↓↓↓↓↓
    pid.pitchDout = config_p->pitch_d * (pid.errPitch - pid.errPitchPrev);
    biquad_lpf(&lpf_pitch_d_term, &pid.pitchDout);
    pid_bb->pitch_d_out = pid.pitchDout;
    // ↑↑↑↑↑↑↑↑↑↑   PITCH D CALCULATION   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   ROLL D CALCULATION   ↓↓↓↓↓↓↓↓↓↓
    pid.rollDout = config_p->roll_d * (pid.errRoll - pid.errRollPrev);
    biquad_lpf(&lpf_roll_d_term, &pid.rollDout);
    pid_bb->roll_d_out = pid.rollDout;
    // ↑↑↑↑↑↑↑↑↑↑   ROLL D CALCULATION   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   PITCH PID OUT   ↓↓↓↓↓↓↓↓↓↓
    pid.pitchPIDout = pid.pitchPout + pid.pitchIout + pid.pitchDout;
    limit_symmetric(&pid.pitchPIDout, MAX_PID);
    // ↑↑↑↑↑↑↑↑↑↑   PITCH PID OUT   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   ROLL PID OUT   ↓↓↓↓↓↓↓↓↓↓
    pid.rollPIDout = pid.rollPout + pid.rollIout + pid.rollDout;
    limit_symmetric(&pid.rollPIDout, MAX_PID);
    // ↑↑↑↑↑↑↑↑↑↑   ROLL PID OUT   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   YAW PID OUT   ↓↓↓↓↓↓↓↓↓↓
    pid.yawPIout = pid.yawPout + pid.yawIout;
    limit_symmetric(&pid.yawPIout, MAX_PID);
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
    //                              Altitude Controller                            //
    //=============================================================================//
    static float accel_limited_target_z_velocity = 0;
    if (flight_p->alt_hold_status == 1)
    {
        static uint8_t counter = 0;
        counter++;
        if (counter >= 10) // 100 Hz
        {
            counter = 0;

            // calculate acceleration limited target velocity z
            float target_accel_z = (target_p->velocity_z_ms - accel_limited_target_z_velocity) * 100.0f;
            if (target_accel_z > MAX_VEL_Z_ACCEL) accel_limited_target_z_velocity += MAX_VEL_Z_ACCEL * 0.01f;
            else if (target_accel_z < -MAX_VEL_Z_ACCEL) accel_limited_target_z_velocity -= MAX_VEL_Z_ACCEL * 0.01f;
            else accel_limited_target_z_velocity += (target_p->velocity_z_ms - accel_limited_target_z_velocity);

            // ↓↓↓↓↓↓↓↓↓↓  CALCULATE CURRENT ERROR   ↓↓↓↓↓↓↓↓↓↓
            pid.errVel_z = accel_limited_target_z_velocity - state_p->vel_up_ms;
            // ↑↑↑↑↑↑↑↑↑↑   CALCULATE CURRENT ERROR   ↑↑↑↑↑↑↑↑↑↑

            // ↓↓↓↓↓↓↓↓↓↓  ALTITUDE P CALCULATION  ↓↓↓↓↓↓↓↓↓↓
            pid.altPout = pid.errVel_z * config_p->altitude_p;
            // ↑↑↑↑↑↑↑↑↑↑  ALTITUDE P CALCULATION   ↑↑↑↑↑↑↑↑↑↑

            // ↓↓↓↓↓↓↓↓↓↓  ALTITUDE I CALCULATION   ↓↓↓↓↓↓↓↓↓↓
            if (flight_p->takeoff_status == 0)
            {
                pid.altIout += config_p->altitude_i * 0.005f * (pid.errVel_z + pid.errVel_z_prev); //  0.005 = 0.5 * sampleTime
                if (pid.altIout > MAX_TARGET_THROTTLE) pid.altIout = MAX_TARGET_THROTTLE;
                else if (pid.altIout < IDLE_THROTTLE) pid.altIout = IDLE_THROTTLE;
            }
            // ↑↑↑↑↑↑↑↑↑↑   ALTITUDE I CALCULATION   ↑↑↑↑↑↑↑↑↑↑

            // ↓↓↓↓↓↓↓↓↓↓   ALTITUDE D CALCULATION   ↓↓↓↓↓↓↓↓↓↓
            pid.altDout = -config_p->altitude_d * state_p->acc_up_ms2;
            // ↑↑↑↑↑↑↑↑↑↑   ALTITUDE D CALCULATION   ↑↑↑↑↑↑↑↑↑↑

            // ↓↓↓↓↓↓↓↓↓↓   ALTITUDE PID OUT   ↓↓↓↓↓↓↓↓↓↓
            target_p->throttle = pid.altPout + pid.altIout + pid.altDout;
            if (target_p->throttle > MAX_TARGET_THROTTLE) target_p->throttle = MAX_TARGET_THROTTLE;
            else if (target_p->throttle < IDLE_THROTTLE) target_p->throttle = IDLE_THROTTLE;
            // ↑↑↑↑↑↑↑↑↑↑   ALTITUDE PID OUT   ↑↑↑↑↑↑↑↑↑↑

            // ↓↓↓↓↓↓↓↓↓↓   HOLD LAST ERROR FOR NEXT CALCULATION   ↓↓↓↓↓↓↓↓↓↓
            pid.errVel_z_prev = pid.errVel_z;
            pid.velocity_z_ms_prev = state_p->vel_up_ms;
            // ↑↑↑↑↑↑↑↑↑↑   HOLD LAST ERROR FOR NEXT CALCULATION   ↑↑↑↑↑↑↑↑↑↑
        }
    }
    else
    {
        accel_limited_target_z_velocity = 0;
    }

    float comp_target_thr = target_p->throttle;
    // polinomial is specific to this drone
    // collect some data while hovering
    // substract initial throttle value from all data it should start from 0
    // remove begining and end of the data
    // gains = polyfit(batt_v, thr_zero, 2); this is the function for matlab (2 is for second order)
    if (telemetry_p->battery_voltage < 11.5f)
        comp_target_thr += telemetry_p->battery_voltage * -48.8911436f + 564.0f;

    float cosAngAbs = cosf(fabs(state_p->pitch_deg) * DEG_TO_RAD) * cosf(fabs(state_p->roll_deg) * DEG_TO_RAD);
    if (cosAngAbs != 0)
        comp_target_thr += ((1.0f / cosAngAbs) - 1.0f) * comp_target_thr;

    // feed forward control should only work on manual pitch roll control
    if (telemetry_p->flight_mode > 1)
    {
        pid.pitch_ff_out = 0;
        pid.roll_ff_out = 0;
        pid.yaw_ff_out = 0;
        prev_rc_ch0 = radio_p->ch0;
        prev_rc_ch1 = radio_p->ch1;
        prev_rc_ch3 = radio_p->ch3;
    }

    // ↓↓↓↓↓↓↓↓↓↓   MOTOR 1 (LEFT BOTTOM)   ↓↓↓↓↓↓↓↓↓↓
    thr.m1 = comp_target_thr - pid.pitchPIDout + pid.rollPIDout + pid.yawPIout + pid.pitch_ff_out + pid.roll_ff_out + pid.yaw_ff_out;
    if (thr.m1 < MIN_THROTTLE) thr.m1 = MIN_THROTTLE;
    else if (thr.m1 > MAX_THROTTLE) thr.m1 = MAX_THROTTLE;
    // ↑↑↑↑↑↑↑↑↑↑   MOTOR 1 (LEFT BOTTOM)   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   MOTOR 2 (LEFT TOP)   ↓↓↓↓↓↓↓↓↓↓
    thr.m2 = comp_target_thr + pid.pitchPIDout + pid.rollPIDout - pid.yawPIout - pid.pitch_ff_out + pid.roll_ff_out - pid.yaw_ff_out;
    if (thr.m2 < MIN_THROTTLE) thr.m2 = MIN_THROTTLE;
    else if (thr.m2 > MAX_THROTTLE) thr.m2 = MAX_THROTTLE;
    // ↑↑↑↑↑↑↑↑↑↑   MOTOR 2 (LEFT TOP)   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   MOTOR 3 (RIGHT BOTTOM)   ↓↓↓↓↓↓↓↓↓↓
    thr.m3 = comp_target_thr - pid.pitchPIDout - pid.rollPIDout - pid.yawPIout + pid.pitch_ff_out - pid.roll_ff_out - pid.yaw_ff_out;
    if (thr.m3 < MIN_THROTTLE) thr.m3 = MIN_THROTTLE;
    else if (thr.m3 > MAX_THROTTLE) thr.m3 = MAX_THROTTLE;
    // ↑↑↑↑↑↑↑↑↑↑   MOTOR 3 (RIGHT BOTTOM)   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   MOTOR 4 (RIGHT TOP)   ↓↓↓↓↓↓↓↓↓↓
    thr.m4 = comp_target_thr + pid.pitchPIDout - pid.rollPIDout + pid.yawPIout - pid.pitch_ff_out - pid.roll_ff_out + pid.yaw_ff_out;
    if (thr.m4 < MIN_THROTTLE) thr.m4 = MIN_THROTTLE;
    else if (thr.m4 > MAX_THROTTLE) thr.m4 = MAX_THROTTLE;
    // ↑↑↑↑↑↑↑↑↑↑   MOTOR 4 (RIGHT TOP)   ↑↑↑↑↑↑↑↑↑↑

    // ↓↓↓↓↓↓↓↓↓↓   OUTPUT TO THE MOTORS   ↓↓↓↓↓↓↓↓↓↓
    #if BENCH_MODE == 1
        write_throttle(0, 0, 0, 0);
    #else
        write_throttle((uint16_t)(thr.m1 * 2.0f), (uint16_t)(thr.m2 * 2.0f), (uint16_t)(thr.m3 * 2.0f), (uint16_t)(thr.m4 * 2.0f));
    #endif
    //  ↑↑↑↑↑↑↑↑↑↑   OUTPUT TO THE MOTORS   ↑↑↑↑↑↑↑↑↑↑
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

    if (telemetry_p->is_gnss_sanity_check_ok == 1)
    {
        home.latitude = gps_p->latitude;
        home.longitude = gps_p->longitude;
        home.altitude_mm = gps_p->altitude_mm;
    }
    else
    {
        home.latitude = 0;
        home.longitude = 0;
        home.altitude_mm = 0;
    }

    telemetry_p->gps_latitude_origin = home.latitude;
    telemetry_p->gps_longitude_origin = home.longitude;
    telemetry_p->gps_altitude_origin = home.altitude_mm;

    home.heading_deg = state_p->heading_deg;

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


uint8_t gnss_sanity_check()
{   
    // manually trigger gnss not safe for testing purposes
    if (radio_p->ch8 <= 1300) return 0;
    // manually trigger gnss safe for testing purposes
    else if ((radio_p->ch8 >= 1700) || (gps_p->satCount > 5 && gps_p->hdop < 3.0f && gps_p->fix == 3)) return 1;
    return 0;
}



static float calculate_target_yaw_degs_from_target_heading_deg()
{
    // Calculate yaw rate from setpoint error
    static float degs = 0.0f;
    degs = (target_p->heading - state_p->heading_deg) * config_p->yaw_rate_scale;
    // This part ensures the craft turns from closest side to setpoint
    // Say the setpoint is 5 deg and craft is at 270, logical thing is craft turns clockwise 95 deg
    // If we dont do this craft will attempt to turn counter clockwise 265deg
    if (degs < -180.0f * config_p->yaw_rate_scale) degs += 360.0f * config_p->yaw_rate_scale;
    else if (degs > 180.0f * config_p->yaw_rate_scale) degs -= 360.0f * config_p->yaw_rate_scale;
    return degs;
}

static float calculate_target_pitch_degs_from_target_pitch_deg()
{
    /////////////////////   Pitch Rate Controller   ///////////////////////
    static float degs = 0.0f;
    degs = (target_p->pitch - state_p->pitch_deg) * config_p->pitch_rate_scale;
    limit_symmetric(&degs, config_p->max_pitch_rate);
    return degs;
}

static float calculate_target_roll_degs_from_target_roll_deg()
{
    /////////////////////   Roll Rate Controller   ///////////////////////
    static float degs = 0.0f;
    degs = (target_p->roll - state_p->roll_deg) * config_p->roll_rate_scale;
    limit_symmetric(&degs, config_p->max_roll_rate);
    return degs;
}

static uint8_t landing_detector(uint8_t need_reset)
{
    static float observed_throttle_value = 0;

    if (need_reset == 0)
    {   
        // lowpass throttle value to decide when to say landed
        observed_throttle_value += (target_p->throttle - observed_throttle_value) * 0.01f;

        if (state_p->altitude_m < 0.3f && observed_throttle_value < IDLE_THROTTLE + 100.0f)
        {
            return 1;
        }
    }
    else
    {
        observed_throttle_value = config_p->hover_throttle;
    }
    return 0;
}

static void limit_symmetric(float *value, float limit)
{
    if (*value > limit) *value = limit;
    else if (*value < -limit) *value = -limit;
}

static void disarm()
{
    telemetry_p->arm_status = 0;
    flight_p->arm_status = 0;
    flight_p->is_takeoff_done = 0;

    write_throttle(0, 0, 0, 0);
}

static int16_t apply_deadband(int16_t input, uint16_t deadband)
{
    if (input > deadband || input < -deadband)
        return (input > 0) ? (input - deadband) : (input + deadband);
    else
        return 0;
}

static void get_distance_bearing(target_location_t *tar_loc, int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2)
{
    // integer degree to float radian ((PI / 180) / 10000000)
    float lat1_rad = lat1 * 1.745329252e-9f;
    float lat2_rad = lat2 * 1.745329252e-9f;
    float lon1_rad = lon1 * 1.745329252e-9f;
    float lon2_rad = lon2 * 1.745329252e-9f;
    
    // calculate lat lon difference
    float dLat = lat2_rad - lat1_rad;
    float dLon = lon2_rad - lon1_rad;
    
    // calculate haversine distance 
    float sin_dlat = sinf(dLat / 2.0f);
    float sin_dlon = sinf(dLon / 2.0f);
    float cos_lat1 = cosf(lat1_rad);
    
    float a = sin_dlat * sin_dlat;
    tar_loc->distance_north_cm = atan2f(sqrtf(a), sqrtf(1.0f - a)) * EARTH_2_RADIUS_CM;

    if (dLat > 0)
        tar_loc->distance_north_cm = -tar_loc->distance_north_cm;
    
    a = cos_lat1 * cos_lat1 * sin_dlon * sin_dlon;
    tar_loc->distance_east_cm = atan2f(sqrtf(a), sqrtf(1.0f - a)) * EARTH_2_RADIUS_CM;
    
    if (dLon > 0)
        tar_loc->distance_east_cm = -tar_loc->distance_east_cm;
    
    tar_loc->distance_cm = sqrtf(tar_loc->distance_north_cm * tar_loc->distance_north_cm + tar_loc->distance_east_cm * tar_loc->distance_east_cm);
    tar_loc->bearing_deg = atan2f(tar_loc->distance_east_cm, tar_loc->distance_north_cm) * RAD_TO_DEG;
    
    if (tar_loc->bearing_deg < 0) tar_loc->bearing_deg += 360.0f;
}