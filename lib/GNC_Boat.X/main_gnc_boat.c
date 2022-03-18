/*
 * File:   main_gnc_boat.c
 * Author: Pavlo Vlastos
 * Brief:  Guidance, navigation, and control (GNC) library for a boat
 * Created on September 30, 2021, 4:09 PM
 */

/******************************************************************************
 * #INCLUDES                                                                  *
 *****************************************************************************/
#include "xc.h"
#include "Board.h"
#include "System_timer.h"
#include "common/mavlink.h"
#include "MavSerial.h"
#include "ICM_20948.h"
#include "linear_trajectory.h"
#include "pid_controller.h"
#include "RC_RX.h"
#include "RC_servo.h"
#include "Publisher.h"
#include "nmea0183v4.h"

#ifdef CF_AHRS
#include "cf_ahrs.h"
#endif
#ifdef TRIAD_AHRS
#include "triad_ahrs.h"
#endif
#include "Lin_alg_float.h"


/******************************************************************************
 * #DEFINES                                                                   *
 *****************************************************************************/
#define HEARTBEAT_PERIOD 1000 //1 sec interval for hearbeat update
#define MAV_SERIAL_READ_PERIOD 10 
#define GPS_FIX_TIME 4000
#define WP_STATE_MACHINE_PERIOD 250
#define WP_CONFIRM_PERIOD 1000
//#define GPS_PERIOD 1000 //1 Hz update rate (For the time being)
#define NUM_MSG_SEND_CONTROL_PERIOD 6
#define CONTROL_PERIOD 20 //Period for control loop in msec
#define SAMPLE_TIME (((float) CONTROL_PERIOD)*(0.001))
#define RAW 1
#define SCALED 2

#define K_ACT ((float) 0.01)
#define UPPER_ACT_BOUND ((float) 75.0) * K_ACT //((float) 0.8) // The maximum rudder actuator limit in radians
#define LOWER_ACT_BOUND ((float) -75.0) * K_ACT //((float)-0.8) // The minimum rudder actuator limit in radians
#define RANGE_ACT (UPPER_ACT_BOUND - LOWER_ACT_BOUND) // Range of actuator
#define SERVO_PAD 0.0
#define SERVO_DELTA ((float) (RC_SERVO_CENTER_PULSE - RC_SERVO_MIN_PULSE))

#define TRANSMIT_PRD 500 // Time to wait to check reference waypoint in milliseconds
#define STATE_MACHINE_PRD 2 // Time to wait to check reference waypoint in milliseconds
#define TOL 0.0001
#define MAX_ACCEPTABLE_CTE ((float) 10.0)

#define COG_YAW_MAX_DIFF (5.0 / 180.0 * M_PI)
#define RAD_2_DEG (180.0 / M_PI)
#define DEG_2_RAD (M_PI / 180.0) 
#define GYRO_SCALE DEG_2_RAD
#define GYRO_BUF_LEN 1000
#define GYRO_N ((float) GYRO_BUF_LEN)

#define LTP_X0 ((float) 10.0)
#define LTP_Y0 ((float) -10.0)
#define LTP_BOUND_X ((float) 60.0)/* meters */
#define LTP_BOUND_Y ((float) 80.0) /* meters */

#define HEADING_ANGLE_BUF 10

/******************************************************************************
 * MAIN                                                                       *
 *****************************************************************************/
int main(void) {
    uint32_t cur_time = 0;
    uint32_t startup_wait_time = 0;
    uint32_t last_prev_wp_en_time = 0;
    uint32_t last_next_wp_en_time = 0;
    uint32_t control_start_time = 0;
    uint32_t heartbeat_start_time = 0;
    uint32_t mav_serial_start_time = 0;
    uint32_t sm_start_time = 0; /* Waypoint state machine time count */

    int8_t IMU_state = ERROR;
    int8_t IMU_retry = 5;
    uint32_t IMU_error = 0;
    uint8_t error_report = 50;

    //    uint32_t ha_buf_i = 0;

    /**************************************************************************
     * Initialization routines                                                *
     *************************************************************************/
    Board_init(); //board configuration
    Sys_timer_init(); //start the system timer

    TRISAbits.TRISA3 = 0; /* Set pin as output. This is also LED4 on Max32 */
    TRISCbits.TRISC1 = 0; /* LED5 */

    LATCbits.LATC1 = 1; /* Set LED5 */
    LATAbits.LATA3 = 1; /* Set LED4 */

    MavSerial_Init();

    publisher_init(MAV_TYPE_SURFACE_BOAT);

    nmea_serial_init();

    RCRX_init(); //initialize the radio control system
    RC_channels_init(); //set channels to midpoint of RC system
    RC_servo_init(); // start the servo subsystem

#ifndef HIL
    IMU_state = IMU_init(IMU_SPI_MODE);

    if (IMU_state == ERROR && IMU_retry > 0) {
        IMU_state = IMU_init(IMU_SPI_MODE);

#ifdef USB_DEBUG
        printf("IMU failed init, retrying %f \r\n", IMU_retry);
#endif
        IMU_retry--;
    }
#endif
    cur_time = Sys_timer_get_msec();
    startup_wait_time = cur_time;
    //    while ((cur_time - startup_wait_time) < 5000) {
    //        cur_time = Sys_timer_get_msec();
    //    }

    LATCbits.LATC1 = 0; /* Set LED5 low */
    LATAbits.LATA3 = 0; /* Set LED4 low */

    float wp_received_en[DIM]; // [EAST (meters), NORTH (meters)]

    // Trajectory 
    float wp_prev_en[DIM]; // [EAST (meters), NORTH (meters)]
    //    float wp_prev_en_last[DIM]; // [EAST (meters), NORTH (meters)]
    float vehi_pt_en[DIM]; // [EAST (meters), NORTH (meters)]

    float wp_next_en[DIM]; // [EAST (meters), NORTH (meters)]
    float ref_ll[DIM]; // [latitude, longitude]
    float ref_lla[DIM + 1]; // [latitude, longitude, altitude]
    float vehi_pt_ll[DIM]; // [latitude, longitude]
    float vehi_pt_lla[DIM + 1] = {0.0}; // [latitude, longitude, altitude]
    float vehi_pt_lla_last[DIM + 1] = {0.0}; // [latitude, longitude, altitude]
    float vehi_pt_ned[DIM + 1]; // [NORTH, EAST, DOWN] (METERS)


    float cross_track_error = 0.0;
    float cross_track_error_last = 0.0;
    float path_angle = 0.0;
    float heading_angle = 0.0; /* Heading angle between North unit vector and 
                                * heading vector within the local tangent plane
                                * (LTP) */
    float angle_correction = 0.0; /* To correct the heading angle from the CF 
                                   * AHRS */
    float cog = 0.0; /* Course over ground angle from GPS */
    //    float cog_last = 0.0; /* Course over ground angle from GPS */
    float ha_report = 0.0;
    //    float heading_angle_buffer[HEADING_ANGLE_BUF];
    float heading_angle_diff = 0.0;
    float u = 0.0; // Resulting control effort
    float delta_angle = 0.0; // Resulting control effort
    float u_sign = 1.0;
    float acc_cmd = 0.0;
    float rtemp = 0.0;
    float speed = 1.0; /* 'measured' speed in meters per second */
    int u_intermediate = 0; // Pulse from converted control effort
    uint16_t u_pulse = 0; // Pulse from converted control effort

    // Attitude
    struct IMU_output imu;
    imu.acc.x = 0.0;
    imu.acc.y = 0.0;
    imu.acc.z = 0.0;

    imu.mag.x = 0.0;
    imu.mag.y = 0.0;
    imu.mag.z = 0.0;

    imu.gyro.x = 0.0;
    imu.gyro.y = 0.0;
    imu.gyro.z = 0.0;

    float acc[MSZ] = {0.1};
    float mag[MSZ] = {0.1};

#ifdef CF_AHRS
    float gyro_bias[MSZ] = {0.0};
    float gyro[MSZ] = {0.0};
#endif

#ifdef TRIAD_AHRS
    triad_ahrs_init(SAMPLE_TIME);
#endif

    float roll = 0.0;
    float pitch = 0.0;
    float yaw = 0.0;
    //    float roll_rate = 0.0;
    //    float pitch_rate = 0.0;
    //    float yaw_rate = 0.0;

    // Controller
    pid_controller_t trajectory_tracker;
    pid_controller_init(&trajectory_tracker,
            SAMPLE_TIME, // dt The sample time
            0.0,//0.5, // kp The initial proportional gain
            0.0, // ki The initial integral gain
            10.0, // kd The initial derivative gain [HIL:0.1]
            1000.0, // The maximum 
            -1000.0); // The minimum 

#ifdef USB_DEBUG
    printf("\r\nMinimal Mavlink application %s, %s \r\n", __DATE__, __TIME__);
#endif

    // MAVLink and State Machine
    uint8_t current_mode = MAV_MODE_MANUAL_ARMED;
    uint8_t last_mode = current_mode;
    uint32_t msg_id = 0;
    uint16_t cmd = 0;
    float wp_type = -1.0;
    float wp_yaw = 0.0;
    //    float m[3] = {0.0};

    enum waypoints_state {
        ERROR_WP = 0,
        FINDING_REF_WP, /* Find the reference point for the LTP calculation */
        SENDING_PREV,
        SENDING_NEXT,
        WAITING_FOR_NEXT_WP, /* Get rid of this */
        TRACKING
    };
    typedef enum waypoints_state waypoints_state_t;

    waypoints_state_t current_wp_state;
    char is_new_imu = FALSE;
    char is_new_msg = FALSE;
    char is_new_prev_wp = FALSE;
    char is_new_next_wp = FALSE;
    char is_far_out = FALSE;
    char is_new_gps = FALSE;

    current_wp_state = FINDING_REF_WP;

    publisher_set_mode(MAV_MODE_MANUAL_ARMED);
    publisher_set_state(MAV_STATE_ACTIVE);

    unsigned int control_loop_count = 0;

    uint8_t i_tx_mav_msg = 0;


    /**************************************************************************
     * Primary Loop                                                           *
     *************************************************************************/
    while (1) {
        cur_time = Sys_timer_get_msec();

        /**********************************************************************
         * Check for all events:                                              *
         * - HIL                                                              *
         * - Real sensors                                                     *
         *********************************************************************/

        is_new_gps = check_GPS_events(); //check and process incoming GPS messages

        if (cur_time - mav_serial_start_time >= MAV_SERIAL_READ_PERIOD) {
            mav_serial_start_time = cur_time; //reset the timer
            is_new_msg = check_mavlink_serial_events(wp_received_en, &msg_id,
                    &cmd, &wp_type, &wp_yaw);

            if ((is_new_msg == TRUE) && (cmd == MAV_CMD_NAV_WAYPOINT)) {

                if ((cur_time - last_prev_wp_en_time) >=
                        WP_CONFIRM_PERIOD) {
                    if (fabs(wp_type - WP_PREV) <= TOL) {

                        /* Update the new 'prev' waypoint */
                        wp_prev_en[0] = wp_received_en[0];
                        wp_prev_en[1] = wp_received_en[1];

                        //                        wp_prev_en_last[0] = wp_prev_en[0];
                        //                        wp_prev_en_last[1] = wp_prev_en[1];

                        last_prev_wp_en_time = cur_time;

                        is_new_prev_wp = TRUE;
                    }
                }

                if ((cur_time - last_next_wp_en_time) >=
                        WP_CONFIRM_PERIOD) {
                    if (fabs(wp_type - WP_NEXT) <= TOL) {

                        /* Update the new 'next' waypoint */
                        wp_next_en[0] = wp_received_en[0];
                        wp_next_en[1] = wp_received_en[1];

                        last_next_wp_en_time = cur_time;

                        is_new_next_wp = TRUE;
                    }
                }
            }

#ifdef HIL
            if ((is_new_msg == TRUE) && (cmd == MAV_CMD_NAV_WAYPOINT)) {
                is_new_gps = TRUE;
                if (fabs(wp_type - VEHI_PT) <= TOL) {
                    vehi_pt_en[0] = wp_received_en[0];
                    vehi_pt_en[1] = wp_received_en[1];

                    yaw = wp_yaw;
                }
            }

            /* Receive IMU Events from computer */
            if (msg_id == MAVLINK_MSG_ID_HIL_SENSOR) {
                check_HIL_IMU_events(&imu);
            }
#else
            is_new_imu = check_IMU_events(SCALED, &imu);
            is_new_msg = check_mavlink_serial_events(wp_received_en, &msg_id,
                    &cmd, &wp_type, &wp_yaw);
            check_RC_events(); //check incoming RC commands

            /* Update aiding vectors and gyro angular rates */
            if (is_new_imu == TRUE) {
                lin_alg_set_v(imu.acc.x, imu.acc.y, imu.acc.z, acc);
                lin_alg_set_v(imu.mag.x, -imu.mag.y, -imu.mag.z, mag);
                //                m[0] = imu.mag.x;
                //                m[1] = imu.mag.y;
                //                m[2] = imu.mag.z;
#ifdef CF_AHRS
                lin_alg_set_v(-imu.gyro.x, -imu.gyro.y, -imu.gyro.z, gyro);
                lin_alg_v_scale(GYRO_SCALE, gyro);
#endif
            }

            //            if (is_new_gps == TRUE) {
            publisher_get_gps_rmc_position(vehi_pt_ll);

            if (current_wp_state == FINDING_REF_WP) {
                ref_ll[0] = vehi_pt_ll[0];
                ref_ll[1] = vehi_pt_ll[1];

                ref_lla[0] = vehi_pt_ll[0];
                ref_lla[1] = vehi_pt_ll[1];
                ref_lla[2] = 0.0;
            }

            vehi_pt_lla[0] = vehi_pt_ll[0];
            vehi_pt_lla[1] = vehi_pt_ll[1];
            vehi_pt_lla[2] = 0.0;

            /* Convert the gps position to the Local Tangent Plane (LTP) */
            if ((vehi_pt_lla[0] != vehi_pt_lla_last[0]) &&
                    (vehi_pt_lla[1] != vehi_pt_lla_last[1])) {

                vehi_pt_lla_last[0] = vehi_pt_lla[0];
                vehi_pt_lla_last[1] = vehi_pt_lla[1];

                lin_tra_lla_to_ned(vehi_pt_lla, ref_lla, vehi_pt_ned);
            }

            vehi_pt_en[0] = vehi_pt_ned[1]; // EAST 
            vehi_pt_en[1] = vehi_pt_ned[0]; // NORTH
            //            }
#endif
        }

        // Check if mode switch event occurred
        current_mode = check_mavlink_mode();

#ifdef AUTONOMOUS_MODE_TEST
        current_mode = MAV_MODE_AUTO_ARMED;
#endif

        if (current_mode != last_mode) {
            last_mode = current_mode;
        }

        /**********************************************************************
         * Wayopint State Machine Logic                                       *
         *********************************************************************/
        if ((cur_time - sm_start_time) > WP_STATE_MACHINE_PERIOD) {
            sm_start_time = cur_time;
            switch (current_wp_state) {
                case ERROR_WP:
                    break;

                    /* Wait until a GPS fix is achieved. The starting  GPS position is
                     * reference waypoint for the LTP. This may take up to about 20 
                     * minutes for the first try after power on. */
                case FINDING_REF_WP:
                    /**********************************************************/

#ifdef HIL
                    vehi_pt_en[0] = 0.0;
                    vehi_pt_en[1] = 0.0;
#endif
                    wp_prev_en[0] = 0.0;
                    wp_prev_en[1] = 0.0;

                    /* Complementary Filter Attitude and Heading Reference 
                     * System (AHRS) Initialization. For now use the starting 
                     * rates assuming the vehicle is stationary */
#ifdef CF_AHRS
                    /* Z-axis Found experimentally, others are greatly 
                     * corrected by gravity */
                    lin_alg_set_v(0.0, 0.0, -0.0062, gyro_bias); /* Control period: 20ms*/
                    cf_ahrs_set_gyro_biases(gyro_bias);

                    //                    cf_ahrs_set_mag_vi(m);

                    cf_ahrs_set_kp_acc(10.0);
                    //                    cf_ahrs_set_kp_mag(0.075);
                    cf_ahrs_set_kp_mag(0.01); /* Control period: 20ms*/
                    cf_ahrs_set_ki_gyro(0.0);

                    cf_ahrs_init(SAMPLE_TIME, gyro_bias);
#endif

#ifdef HIL
                    current_wp_state = SENDING_NEXT;
#else


                    //                        publisher_get_gps_rmc_position(vehi_pt_ll);


                    // State exit case
                    if ((cur_time - startup_wait_time) >= GPS_FIX_TIME) {
                        if ((ref_ll[0] != 0.0) && (ref_ll[1] != 0.0)) {
                            current_wp_state = SENDING_PREV;
                        }
                    }
#endif


                    break;

                case SENDING_PREV:
                    /**********************************************************/
                    // Send what the vehicle calculated as its 'prev' waypoint
                    publish_waypoint_en(wp_prev_en, WP_PREV);

                    if ((cur_time - last_prev_wp_en_time) >=
                            WP_CONFIRM_PERIOD) {

                        lin_tra_set_prev_wp(wp_prev_en);

                        current_wp_state = TRACKING;
                    }

                    break;

                case SENDING_NEXT:
                    /**********************************************************/
                    // Send what the vehicle calculated as its 'next' waypoint
                    publish_waypoint_en(wp_next_en, WP_NEXT);

                    if ((cur_time - last_next_wp_en_time) >=
                            WP_CONFIRM_PERIOD) {

                        lin_tra_set_next_wp(wp_next_en);

                        current_wp_state = TRACKING;
                    }
                    break;

                case TRACKING:
                    /**********************************************************/

                    /* Edge case if too far out */
                    if ((fabs(cross_track_error) > MAX_ACCEPTABLE_CTE) &&
                            (is_far_out == FALSE)) {
                        wp_prev_en[0] = vehi_pt_en[0];
                        wp_prev_en[1] = vehi_pt_en[1];
                        cross_track_error_last = cross_track_error;
                        is_far_out = TRUE;
                    }

                    if ((fabs(cross_track_error_last + cross_track_error) <
                            MAX_ACCEPTABLE_CTE) && (is_far_out == TRUE)) {
                        //                        wp_prev_en_last[0] = wp_prev_en[0];
                        //                        wp_prev_en_last[1] = wp_prev_en[1];
                        is_far_out = FALSE;
                    }

                    /* Exit cases: */
                    if (is_new_prev_wp == TRUE) {
                        current_wp_state = SENDING_PREV;
                        is_new_prev_wp = FALSE;
                    }

                    if (is_new_next_wp == TRUE) {
                        current_wp_state = SENDING_NEXT;
                        is_new_next_wp = FALSE;
                    }

                    lin_tra_init(wp_prev_en, wp_next_en, vehi_pt_en);
                    break;
            }
        }


        /**********************************************************************
         * CONTROL: Control and publish data                                  *
         *********************************************************************/
        if ((cur_time - control_start_time) >= CONTROL_PERIOD) {
            control_start_time = cur_time; //reset control loop timer

            set_control_output(); // set actuator outputs
#ifndef HIL
            IMU_state = IMU_start_data_acq(); //initiate IMU measurement with SPI

            if (IMU_state == ERROR) {
                IMU_error++;
                if (IMU_error % error_report == 0) {

#ifdef USB_DEBUG
                    printf("IMU error count %d\r\n", IMU_error);
#endif

                }
            }
#endif

            /******************************************************************
             * Complementary filter attitude and heading reference system     *
             *****************************************************************/
            /* Update */
#ifndef HIL
#ifdef CF_AHRS
            cf_ahrs_update(acc, mag, gyro, &yaw, &pitch, &roll);

            heading_angle = yaw;

            // Using cog for now, but still publishing the ATTITUDE message
            cog = (nmea_get_rmc_cog() * DEG_2_RAD);

            //            if (fabs(cog - cog_last) > M_PI_2) {
            //                cog = cog_last;
            //            }
            //
            //            cog_last = cog;

#endif
#ifdef TRIAD_AHRS
            mag[2] = 0.0; /* Not using z component of magnetometer with TRIAD */
            triad_ahrs_update(acc, mag, &yaw, &pitch, &roll);

            /* Average the TRIAD yaw angle */
            heading_angle_buffer[control_loop_count % HEADING_ANGLE_BUF] = yaw;
            heading_angle = 0;
            for (ha_buf_i = 0; ha_buf_i < HEADING_ANGLE_BUF; ha_buf_i++) {
                heading_angle += heading_angle_buffer[ha_buf_i];
            }

            heading_angle /= ((float) HEADING_ANGLE_BUF);

            /* Correction offset */
            //            heading_angle += (M_PI);
            //            heading_angle = fmod(
            //                    (heading_angle + M_PI), 2.0 * M_PI) - M_PI;
#endif
#endif

            /******************************************************************
             * Control                                                        *
             *****************************************************************/
            if ((current_mode == MAV_MODE_AUTO_ARMED) &&
                    (lin_tra_is_initialized() == TRUE)) {
                //            if (lin_tra_is_initialized() == TRUE) {

                if (lin_tra_update(vehi_pt_en) == SUCCESS) {
                    cross_track_error = lin_tra_get_cte();

                    path_angle = lin_tra_get_path_angle();

                    ha_report = (heading_angle * RAD_2_DEG);
                    //                    ha_report = cog * RAD_2_DEG;


                    heading_angle_diff = heading_angle - path_angle;
                    //                    heading_angle_diff = cog - path_angle;
                    heading_angle_diff = fmod(
                            (heading_angle_diff + M_PI), 2.0 * M_PI) - M_PI;
                }

                /* Trajectory tracking controller */
                acc_cmd = pid_controller_update(
                        &trajectory_tracker,
                        0.0, // Commanded reference
                        cross_track_error,
                        heading_angle_diff); // Change in heading angle over time

                /* Convert acceleration command to rudder angle */
                if (acc_cmd != 0.0) {
                    if (acc_cmd < 0.0) {
                        u_sign = -1.0;
                    }
                    if (acc_cmd > 0.0) {
                        u_sign = 1.0;
                    }

                    rtemp = (speed * speed) / fabs(acc_cmd);

                    if (rtemp != 0.0) {
                        delta_angle = u_sign * (sqrt(fabs(acc_cmd) / rtemp) *
                                SAMPLE_TIME);
                    }
                }

                u = delta_angle;

                // Scale resulting control input
                u *= (SERVO_DELTA - SERVO_PAD);
                u /= UPPER_ACT_BOUND;
                u += ((float) RC_SERVO_CENTER_PULSE);
                u_intermediate = (int)u;
                u_pulse = ((uint16_t) u_intermediate);

                RC_servo_set_pulse(u_pulse, RC_STEERING);

                delta_angle = (float) u_pulse;

            }


            // Information from trajectory


            /******************************************************************
             * Publish data                                                   *
             *****************************************************************/
            switch (i_tx_mav_msg) {
                case 0:
                    publish_attitude(roll,
                            pitch,
                            ha_report, //heading_angle, /* Using differently on purpose */
                            path_angle, // roll-rate, /* Using differently on purpose */
                            delta_angle, // pitch-rate, /* Using differently on purpose */
                            (float) current_wp_state); // yaw-rate /* Using differently on purpose */ /* @TODO: add rates */

                    LATCbits.LATC1 ^= 1; /* Toggle LED5 */
                    break;
                case 1:
                    publish_RC_signals_raw();
                    break;
                case 2:
                    publish_IMU_data(SCALED);
                    break;
                case 3:
                    publisher_set_mode(current_mode); // Sets mode in heartbeat
                    break;
                case 4:
#ifndef HIL
                    publish_waypoint_en(vehi_pt_en, VEHI_PT);
#endif 
                    break;
                case 5:
#ifndef HIL
                    if (current_wp_state != FINDING_REF_WP) {
                        publish_GPS();
                    }
#endif 
                    break;
            }
            control_loop_count++;
            i_tx_mav_msg++;
            i_tx_mav_msg %= NUM_MSG_SEND_CONTROL_PERIOD;
        }

        //#endif
        // Publish heartbeat
        if (cur_time - heartbeat_start_time >= HEARTBEAT_PERIOD) {
            heartbeat_start_time = cur_time; //reset the timer
            publish_heartbeat(); // TODO: add argument to update the mode

#ifdef HIL
            publish_HIL_servo_output_raw(u_pulse);
#endif 

            LATAbits.LATA3 ^= 1; /* Set LED4 */

        }
    }

    return 0;
}