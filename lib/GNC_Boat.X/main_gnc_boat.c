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
#include "cf_ahrs.h"
#include "Lin_alg_float.h"


/******************************************************************************
 * #DEFINES                                                                   *
 *****************************************************************************/
#define HEARTBEAT_PERIOD 1000 //1 sec interval for hearbeat update
#define MAV_SERIAL_READ_PERIOD 10 
#define WP_STATE_MACHINE_PERIOD 250
#define GPS_PERIOD 1000 //1 Hz update rate (For the time being)
#define CONTROL_PERIOD 10 //Period for control loop in msec
#define SAMPLE_TIME (1.0 / ((float) CONTROL_PERIOD))
#define RAW 1
#define SCALED 2

#define UPPER_ACT_BOUND ((float) 0.8) // The maximum rudder actuator limit in radians
#define LOWER_ACT_BOUND ((float)-0.8) // The minimum rudder actuator limit in radians
#define RANGE_ACT (UPPER_ACT_BOUND - LOWER_ACT_BOUND) // Range of actuator
#define SERVO_PAD 0.0
#define SERVO_DELTA ((float) (RC_SERVO_CENTER_PULSE - RC_SERVO_MIN_PULSE))

#define TRANSMIT_PRD 500 // Time to wait to check reference waypoint in milliseconds
#define STATE_MACHINE_PRD 2 // Time to wait to check reference waypoint in milliseconds
#define TOL 0.0001

#define GYRO_SCALE ((M_PI / 180.0) * 0.6)
#define GYRO_BUF_LEN 1000
#define GYRO_N ((float) GYRO_BUF_LEN)

#define LTP_X0 ((float) 10.0)
#define LTP_Y0 ((float) -10.0)
#define LTP_BOUND_X ((float) 60.0)/* meters */
#define LTP_BOUND_Y ((float) 80.0) /* meters */

/******************************************************************************
 * MAIN                                                                       *
 *****************************************************************************/
int main(void) {
    uint32_t cur_time = 0;
    uint32_t transmit_time = 0;
    uint32_t control_start_time = 0;
    uint32_t heartbeat_start_time = 0;
    uint32_t mav_serial_start_time = 0;
    uint32_t sm_start_time = 0; /* Waypoint state machine time count */

    int8_t IMU_state = ERROR;
    int8_t IMU_retry = 5;
    uint32_t IMU_error = 0;
    uint8_t error_report = 50;

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

    LATCbits.LATC1 = 0; /* Set LED5 low */
    LATAbits.LATA3 = 0; /* Set LED4 low */

    // Trajectory 
    float wp_a[DIM]; // [lat, lon]
    float wp_b[DIM];

    float wp_prev_lla[DIM + 1]; // [lat, lon, alt]

    float wp_received_en[DIM]; // [EAST (meters), NORTH (meters)]
    float last_wp_received_en[DIM]; // [EAST (meters), NORTH (meters)]

    float wp_prev_ned[DIM + 1]; // [EAST (meters), NORTH (meters), UP (meters)]

    float wp_prev_en[DIM]; // [EAST (meters), NORTH (meters)]
    float vehi_pt_en[DIM]; // [EAST (meters), NORTH (meters)]

    float wp_next_en[DIM]; // [EAST (meters), NORTH (meters)]
    float last_wp_next_en[DIM]; // [EAST (meters), NORTH (meters)]
    float ref_lla[DIM + 1]; // [latitude, longitude, altitude]

    float cross_track_error = 0.0;
    float path_angle = 0.0;
    float heading_angle = 0.0; /* Heading angle between North unit vector and 
                                * heading vector within the local tangent plane
                                * (LTP) */
    float heading_angle_diff = 0.0;
    float u = 0.0; // Resulting control effort
    float u_sign = 1.0;
    float acc_cmd = 0.0;
    float rtemp = 0.0;
    float speed = 1.0; /* 'measured' speed in meters per second */
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

    float acc[MSZ] = {0.0};
    float mag[MSZ] = {0.0};
    float gyro_bias[MSZ] = {0.0};
    float gyro[MSZ] = {0.0};
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
            1.0, // kp The initial proportional gain
            0.0, // ki The initial integral gain
            0.01, // kd The initial derivative gain [HIL:0.1]
            UPPER_ACT_BOUND, // The maximum rudder actuator limit in radians
            LOWER_ACT_BOUND); // The minimum rudder actuator limit in radians

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

    enum waypoints_state {
        ERROR_WP = 0,
        FINDING_REF_WP, /* Find the reference point for the LTP calculation */
        SENDING_REF_WP, /* Send the reference point for the LTP calculation */
        CHECKING_REF_WP, /* Check the reference point sending and comparing 
                          * echo from Companion Computer */
        WAITING_FOR_NEXT_WP, /* Wait for the next waypoint from the 
                              * Companion Computer */
    };
    typedef enum waypoints_state waypoints_state_t;

    waypoints_state_t current_wp_state;
    char is_new_imu = FALSE;
    char is_new_msg = FALSE;
    char is_new_wp = FALSE;
    char is_new_gps = FALSE;
    char found_ref_point = FALSE;

    current_wp_state = FINDING_REF_WP;

    publisher_set_mode(MAV_MODE_MANUAL_ARMED);
    publisher_set_state(MAV_STATE_ACTIVE);

    unsigned int control_loop_count = 0;

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
        if (cur_time - mav_serial_start_time >= MAV_SERIAL_READ_PERIOD) {
            mav_serial_start_time = cur_time; //reset the timer
#ifdef HIL
            /* Hardware in the loop (HIL, or sometimes HITL)*/
            is_new_msg = check_mavlink_serial_events(wp_received_en, &msg_id,
                    &cmd, &wp_type, &wp_yaw);

            if ((last_wp_received_en[0] != wp_received_en[0]) &&
                    (last_wp_received_en[1] != wp_received_en[1])) {

                wp_received_en[0] = last_wp_received_en[0]; /* Lat */
                wp_received_en[1] = last_wp_received_en[1]; /* Lon */

                is_new_wp = TRUE;
            }

            /* Receive IMU Events from computer */
            if (msg_id == MAVLINK_MSG_ID_HIL_SENSOR) {
                check_HIL_IMU_events(&imu);
            }

            /* The waypoint type determins if it is LTP waypoint or vehicle 
             * East, North*/
            if (wp_type == 2.0) {
                vehi_pt_en[0] = wp_received_en[0];
                vehi_pt_en[1] = wp_received_en[1];
                yaw = wp_yaw;

                is_new_gps = TRUE;
            } 
#else
            is_new_imu = check_IMU_events(SCALED, &imu);
            is_new_msg = check_mavlink_serial_events(wp_received_en, &msg_id,
                    &cmd, &wp_type, &wp_yaw);
            check_RC_events(); //check incoming RC commands
            is_new_gps = check_GPS_events(); //check and process incoming GPS messages

            /* Update aiding vectors and gyro angular rates */
            if (is_new_imu == TRUE) {
                lin_alg_set_v(imu.acc.x, imu.acc.y, imu.acc.z, acc);
                lin_alg_set_v(imu.mag.x, imu.mag.y, imu.mag.z, mag);
                lin_alg_set_v(imu.gyro.x, imu.gyro.y, imu.gyro.z, gyro);
                lin_alg_v_scale(GYRO_SCALE, gyro);
            }
#endif
        }

#ifndef HIL
        /* Convert the gps position to the Local Tangent Plane (LTP) if a 
         * reference point has been established*/
        if ((is_new_gps == TRUE) && (found_ref_point == TRUE)) {
            vehi_pt_lla[0] = wp_received_en[0]; /* latitude */
            vehi_pt_lla[1] = wp_received_en[1]; /* longitude */
            vehi_pt_lla[2] = 0.0; /* altitude */

            /* Convert the gps position to the Local Tangent Plane (LTP) */
            lin_tra_lla_to_ned(vehi_pt_lla, ref_lla, vehi_pt_ned);

            vehi_pt_en[0] = vehi_pt_ned[1]; // EAST 
            vehi_pt_en[1] = vehi_pt_ned[0]; // NORTH
        }
#endif

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

                    // State exit case
                    if ((is_new_gps == TRUE) &&
                            (current_mode == MAV_MODE_AUTO_ARMED)) {
#ifdef HIL
                        wp_a[0] = 0.0;
                        wp_a[1] = 0.0;
#else
                        publisher_get_gps_rmc_position(wp_a);
#endif

                        /* Complementary Filter Attitude and Heading Reference 
                         * System (AHRS) Initialization. For now use the starting 
                         * rates assuming the vehicle is stationary */
                        lin_alg_set_v(0.0, 0.0, 0.0, gyro_bias);
                        cf_ahrs_set_gyro_biases(gyro_bias);

                        cf_ahrs_set_kp_acc(5.0);
                        cf_ahrs_set_kp_mag(1.0);

                        cf_ahrs_init(SAMPLE_TIME, gyro_bias);

                        current_wp_state = SENDING_REF_WP;
                        
                        is_new_gps = FALSE;
                    }
                    break;

                    /* Send the reference waypoint */
                case SENDING_REF_WP:
                    /**********************************************************/
                    LATCbits.LATC1 ^= 1; // Toggle LED 5

#ifdef HIL
                    publish_waypoint(wp_a);
#else
                    if ((wp_a[0] != 0.0) && (wp_a[1] != 0.0)) {
                        publish_waypoint(wp_a);
                    }
#endif

                    // State exit case
                    if ((is_new_msg == TRUE) && (cmd == MAV_CMD_NAV_WAYPOINT)&&
                            (is_new_wp == TRUE)) {
                        wp_a[0] = wp_received_en[0];
                        wp_a[1] = wp_received_en[1];

                        current_wp_state = CHECKING_REF_WP;
                        
                        /* Set the new message as FALSE after the state machine 
                         * has run */
                        is_new_msg = FALSE; 
                        
                        /* Reset new waypoint flag after using the newest */
                        is_new_wp = FALSE;
                        
                        /* Reset command after reacting to it */
                        cmd = 0;
                    }
                    break;

                case CHECKING_REF_WP:
                    /**********************************************************/

                    // State exit case
                    if ((is_new_msg == TRUE) && (cmd == MAV_CMD_NAV_WAYPOINT)&&
                            (is_new_wp == TRUE)) {
                        wp_b[0] = wp_received_en[0];
                        wp_b[1] = wp_received_en[1];

                        if (lin_tra_calc_dist(wp_a, wp_b) < TOL) {

                            ref_lla[0] = wp_a[0]; // latitude
                            ref_lla[1] = wp_a[1]; // longitude
                            ref_lla[2] = 0.0; // Altitude

                            wp_prev_lla[0] = ref_lla[0]; // latitude
                            wp_prev_lla[1] = ref_lla[1]; // longitude
                            wp_prev_lla[2] = ref_lla[2]; // Altitude

                            lin_tra_lla_to_ned(wp_prev_lla, ref_lla,
                                    wp_prev_ned);

                            wp_prev_en[0] = wp_prev_ned[1]; // East
                            wp_prev_en[1] = wp_prev_ned[0]; // North

                            found_ref_point = TRUE;

                            /* The ack result is the current state */
                            publish_ack(CHECKING_REF_WP);

                            current_wp_state = WAITING_FOR_NEXT_WP;
                        } else {
                            publish_ack(ERROR_WP);
                            current_wp_state = FINDING_REF_WP;
                        }
                        
                        /* Set the new message as FALSE after the state machine 
                         * has run */
                        is_new_msg = FALSE; 
                        
                        /* Reset new waypoint flag after using the newest */
                        is_new_wp = FALSE;
                        
                        /* Reset command after reacting to it */
                        cmd = 0;
                    }
                    break;

                case WAITING_FOR_NEXT_WP:
                    /**********************************************************/
                    if ((is_new_msg == TRUE) && (cmd == MAV_CMD_NAV_WAYPOINT)&&
                            (is_new_wp == TRUE)) {
                        wp_next_en[0] = wp_received_en[0];
                        wp_next_en[1] = wp_received_en[1];

                        if (lin_tra_calc_dist(wp_next_en, last_wp_next_en) > TOL) {
                            wp_prev_en[0] = last_wp_next_en[0];
                            wp_prev_en[1] = last_wp_next_en[1];
                        }

                        lin_tra_init(wp_prev_en, wp_next_en, vehi_pt_en);

                        /* Set the new message as FALSE after the state machine 
                         * has run */
                        is_new_msg = FALSE; 
                        
                        /* Reset new waypoint flag after using the newest */
                        is_new_wp = FALSE;
                        
                        /* Reset command after reacting to it */
                        cmd = 0;
                    }
                    break;
            }

            last_wp_received_en[0] = wp_received_en[0];
            last_wp_received_en[1] = wp_received_en[1];
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
            cf_ahrs_update(acc, mag, gyro, &yaw, &pitch, &roll);
#endif

            /******************************************************************
             * Control                                                        *
             *****************************************************************/
            if ((current_mode == MAV_MODE_AUTO_ARMED) &&
                    (lin_tra_is_initialized() == TRUE)) {

                if (lin_tra_update(vehi_pt_en) == SUCCESS) {
                    cross_track_error = lin_tra_get_cte();
                    path_angle = lin_tra_get_path_angle();

                    /* Path Angle with angle offset for now */
                    heading_angle = yaw;
#ifdef HIL
                    heading_angle += M_PI;
#else
                    heading_angle += (M_PI / 2.0);
#endif
                    heading_angle = fmod(
                            (heading_angle + M_PI), 2.0 * M_PI) - M_PI;

                    heading_angle_diff = heading_angle - path_angle;

                    heading_angle_diff = fmod(
                            (heading_angle_diff + M_PI), 2.0 * M_PI) - M_PI;
                }

                /* Trajectory tracking controller */
                acc_cmd = pid_controller_update(
                        &trajectory_tracker,
                        0.0, // Commanded reference
                        0.0, //cross_track_error,
                        heading_angle_diff); // Change in heading angle over time

                //                /* Convert acceleration command to rudder angle */
                //                if (acc_cmd != 0.0) {
                //                    if (acc_cmd < 0.0) {
                //                        u_sign = -1.0;
                //                    }
                //                    if (acc_cmd >= 0.0) {
                //                        u_sign = 1.0;
                //                    }
                //                    
                //                    rtemp = (speed*speed)/fabs(acc_cmd);
                //                    
                //                    if (rtemp != 0.0) {
                //                        u
                //                    }
                //                }

                u = acc_cmd;

                // Scale resulting control input
                u /= UPPER_ACT_BOUND;
                u *= (SERVO_DELTA - SERVO_PAD);
                u += ((float) RC_SERVO_CENTER_PULSE);
                u_pulse = ((uint16_t) u);

                RC_servo_set_pulse(u_pulse, RC_STEERING);

            }


            // Information from trajectory


            /******************************************************************
             * Publish data                                                   *
             *****************************************************************/
            publish_RC_signals_raw();
            publish_IMU_data(SCALED);
            publish_attitude(roll,
                    pitch,
                    heading_angle, /* Using differently on purpose */
                    path_angle, // roll-rate, /* Using differently on purpose */
                    heading_angle_diff, // pitch-rate, /* Using differently on purpose */
                    (float) current_wp_state); // yaw-rate /* Using differently on purpose */ /* @TODO: add rates */
            publisher_set_mode(current_mode); // Sets mode in heartbeat
#ifndef HIL
            publish_GPS();
#endif 
            control_loop_count++;
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