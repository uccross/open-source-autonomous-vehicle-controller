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
#define GPS_PERIOD 1000 //1 Hz update rate (For the time being)
#define CONTROL_PERIOD 20 //Period for control loop in msec
#define SAMPLE_TIME (1.0 / ((float) CONTROL_PERIOD))
#define RAW 1
#define SCALED 2

#define UPPER_ACT_BOUND ((float) 0.8) // The maximum rudder actuator limit in radians
#define LOWER_ACT_BOUND ((float)-0.8) // The minimum rudder actuator limit in radians
#define RANGE_ACT (UPPER_ACT_BOUND - LOWER_ACT_BOUND) // Range of actuator
#define SERVO_PAD 0.0
#define SERVO_DELTA ((float) (RC_RX_MID_COUNTS - RC_RX_MIN_COUNTS)) - SERVO_PAD

#define TRANSMIT_PRD 500 // Time to wait to check reference waypoint in milliseconds
#define STATE_MACHINE_PRD 2 // Time to wait to check reference waypoint in milliseconds
#define TOL 0.00001

#define GYRO_SCALE ((M_PI / 180.0) * 0.6)
#define GYRO_BUF_LEN 1000
#define GYRO_N ((float) GYRO_BUF_LEN)

/******************************************************************************
 * MAIN                                                                       *
 *****************************************************************************/
int main(void) {
    uint32_t cur_time = 0;
    uint32_t transmit_time = 0;
    uint32_t gps_start_time = 0;
    uint32_t control_start_time = 0;
    uint32_t heartbeat_start_time = 0;
    uint32_t sm_start_time = 0;
    //    uint8_t index;
    int8_t IMU_state = ERROR;
    int8_t IMU_retry = 5;
    uint32_t IMU_error = 0;
    uint8_t error_report = 50;

    uint32_t i = 0;
    uint32_t i_gyro = 0;
    uint32_t j = 0;



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


    IMU_state = IMU_init(IMU_SPI_MODE);

    if (IMU_state == ERROR && IMU_retry > 0) {
        IMU_state = IMU_init(IMU_SPI_MODE);

#ifdef USB_DEBUG
        printf("IMU failed init, retrying %f \r\n", IMU_retry);
#endif
        IMU_retry--;
    }

    LATCbits.LATC1 = 0; /* Set LED5 low */
    LATAbits.LATA3 = 0; /* Set LED4 low */

    // Trajectory 
    float wp_a_lat_lon[DIM]; // [lat, lon]
    float wp_b_lat_lon[DIM];
    float wp_received_ll[DIM]; // [lat, lon]

    float wp_prev_lla[DIM + 1]; // [lat, lon, alt]
    float vehi_pt_lla[DIM + 1]; // [lat, lon, alt]
    float wp_next_lla[DIM + 1]; // [lat, lon, alt]

    float wp_prev_enu[DIM + 1]; // [EAST (meters), NORTH (meters), UP (meters)]
    float vehi_pt_enu[DIM + 1]; // [EAST (meters), NORTH (meters), UP (meters)]
    float wp_next_enu[DIM + 1]; // [EAST (meters), NORTH (meters), UP (meters)]

    float wp_prev_en[DIM]; // [EAST (meters), NORTH (meters)]
    float vehi_pt_en[DIM]; // [EAST (meters), NORTH (meters)]

    float wp_next_en[DIM]; // [EAST (meters), NORTH (meters)]
    float ref_lla[DIM + 1]; // [latitude, longitude, altitude]

    float cross_track_error = 0.0;
    float path_angle = 0.0;
    float heading_angle = 0.0; /* Heading angle between North unit vector and 
                                * heading vector within the local tangent plane
                                * (LTP) */
    float heading_angle_diff = 0.0;
    float u = 0.0; // Resulting control effort
    uint16_t u_pulse = 0; // Pulse from converted control effort

    // Attitude
    struct IMU_output data;
    data.acc.x = 0.0;
    data.acc.y = 0.0;
    data.acc.z = 0.0;

    data.mag.x = 0.0;
    data.mag.y = 0.0;
    data.mag.z = 0.0;

    data.gyro.x = 0.0;
    data.gyro.y = 0.0;
    data.gyro.z = 0.0;
    float acc[MSZ] = {0.0};
    float mag[MSZ] = {0.0};
    float gyro_bias[MSZ] = {0.0};
    float gyro[MSZ] = {0.0};
    float gyro_x_buf[GYRO_BUF_LEN] = {0.0};
    float gyro_y_buf[GYRO_BUF_LEN] = {0.0};
    float gyro_z_buf[GYRO_BUF_LEN] = {0.0};\
    char gyro_init_avg = FALSE;
    float roll = 0.0;
    float pitch = 0.0;
    float yaw = 0.0;
    float roll_rate = 0.0;
    float pitch_rate = 0.0;
    float yaw_rate = 0.0;

    // Controller
    pid_controller_t trajectory_tracker;
    pid_controller_init(&trajectory_tracker,
            SAMPLE_TIME, // dt The sample time
            1.0, // kp The initial proportional gain
            0.0, // ki The initial integral gain
            1.0, // kd The initial derivative gain
            UPPER_ACT_BOUND, // The maximum rudder actuator limit in radians
            LOWER_ACT_BOUND); // The minimum rudder actuator limit in radians

#ifdef USB_DEBUG
    printf("\r\nMinimal Mavlink application %s, %s \r\n", __DATE__, __TIME__);
#endif

    // MAVLink and State Machine
    uint8_t current_mode = MAV_MODE_MANUAL_ARMED;
    uint8_t last_mode = current_mode;
    uint16_t cmd = 0;

    enum waypoints_state {
        ERROR_WP = 0,
        FINDING_REF_WP, /* Find the reference point for the LTP calculation */
        SENDING_REF_WP, /* Send the reference point for the LTP calculation */
        CHECKING_REF_WP, /* Check the reference point sending and comparing 
                          * echo from Companion Computer */
        WAITING_FOR_PREV_WP, /* Wait for the previous waypoint from the 
                              * Companion Computer */
        CHECKING_PREV_WP, /* Echo test the previous waypoint */
        WAITING_FOR_NEXT_WP, /* Wait for the next waypoint from the 
                              * Companion Computer */
        CHECKING_NEXT_WP, /* Echo test the next waypoint */
        TRACKING_WP /* Track the line connecting the previous and next waypoint */
    };
    typedef enum waypoints_state waypoints_state_t;

    waypoints_state_t current_wp_state = FINDING_REF_WP;
    waypoints_state_t last_wp_state = FINDING_REF_WP;
    char new_imu = FALSE;
    char new_msg = FALSE;
    char new_gps = FALSE;
    char found_ref_point = FALSE;


    publisher_set_mode(MAV_MODE_MANUAL_ARMED);
    publisher_set_state(MAV_STATE_ACTIVE);

    unsigned int control_loop_count = 0;

    /**************************************************************************
     * Primary Loop                                                           *
     *************************************************************************/
    while (1) {
        cur_time = Sys_timer_get_msec();

        /**********************************************************************
         * Check for all events                                               *
         *********************************************************************/
        new_imu = check_IMU_events(SCALED, &data);
        new_msg = check_mavlink_serial_events(wp_received_ll, &cmd);
        check_RC_events(); //check incoming RC commands
        new_gps = check_GPS_events(); //check and process incoming GPS messages

        /* Update aiding vectors and gyro angular rates */
        if (new_imu == TRUE) {
            lin_alg_set_v(data.acc.x, data.acc.y, data.acc.z, acc);
            lin_alg_set_v(data.mag.x, data.mag.y, data.mag.z, mag);
            lin_alg_set_v(data.gyro.x, data.gyro.y, data.gyro.z, gyro);
            lin_alg_v_scale(GYRO_SCALE, gyro);
        }

        /* Convert the gps position to the Local Tangent Plane (LTP) if a 
         * reference point has been established*/
        if ((new_gps == TRUE) && (found_ref_point == TRUE)) {
            vehi_pt_lla[0] = wp_received_ll[0]; /* latitude */
            vehi_pt_lla[1] = wp_received_ll[1]; /* longitude */
            vehi_pt_lla[2] = 0.0; /* altitude */

            /* Convert the gps position to the Local Tangent Plane (LTP) */
            lin_tra_lla_to_enu(vehi_pt_lla, ref_lla, vehi_pt_enu);

            vehi_pt_en[0] = vehi_pt_enu[1]; // EAST /* @TODO: FIX ENU->NED*/
            vehi_pt_en[1] = vehi_pt_enu[0]; // NORTH
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
        switch (current_wp_state) {
            case ERROR_WP:
                break;

            case FINDING_REF_WP:
                // State exit case
                if ((new_gps == TRUE) &&
                        (current_mode == MAV_MODE_AUTO_ARMED)) {
                    publisher_get_gps_rmc_position(wp_a_lat_lon);

                    /* Complementary Filter Attitude and Heading Reference 
                     * System (AHRS) Initialization. For now use the starting 
                     * rates assuming the vehicle is stationary */
                    lin_alg_set_v(0.0, 0.0, 0.0, gyro_bias);
                    cf_ahrs_set_gyro_biases(gyro_bias);

                    cf_ahrs_set_kp_acc(5.0);
                    cf_ahrs_set_kp_mag(1.0);

                    cf_ahrs_init(SAMPLE_TIME, gyro_bias);

                    current_wp_state = SENDING_REF_WP;
                }
                break;

            case SENDING_REF_WP:

                LATCbits.LATC1 ^= 1; // Toggle LED 5
                // Transmit periodically, until the exit condition is met
                if ((cur_time - transmit_time) >= TRANSMIT_PRD) {
                    transmit_time = cur_time;
                    publish_waypoint(wp_a_lat_lon);
                }

                // State exit case
                if ((new_msg == TRUE) && (cmd == MAV_CMD_NAV_WAYPOINT)) {
                    wp_b_lat_lon[0] = wp_received_ll[0];
                    wp_b_lat_lon[1] = wp_received_ll[1];

                    current_wp_state = CHECKING_REF_WP;
                }
                break;

            case CHECKING_REF_WP:
                // State exit case
                if (lin_tra_calc_dist(wp_a_lat_lon, wp_b_lat_lon) < TOL) {

                    ref_lla[0] = wp_a_lat_lon[0]; // latitude
                    ref_lla[1] = wp_a_lat_lon[1]; // longitude
                    ref_lla[2] = 0.0; // Altitude

                    found_ref_point = TRUE;

                    /* The ack result is the current state */
                    publish_ack(CHECKING_REF_WP);

                    current_wp_state = WAITING_FOR_PREV_WP;
                } else {
                    publish_ack(ERROR_WP);

                    current_wp_state = FINDING_REF_WP;
                }
                break;

            case WAITING_FOR_PREV_WP:

                if ((new_msg == TRUE) && (cmd == MAV_CMD_NAV_WAYPOINT)) {
                    wp_a_lat_lon[0] = wp_received_ll[0];
                    wp_a_lat_lon[1] = wp_received_ll[1];

                    publish_waypoint(wp_a_lat_lon); /* Echo back the received 
                                                     * waypoint */

                    current_wp_state = CHECKING_PREV_WP;
                } else {
                    /* Repeat the ACK SUCCESS message in case the Companion 
                     * Computer missed it */
                    publish_ack(CHECKING_REF_WP);
                }
                break;

            case CHECKING_PREV_WP:
                /* Check for a second previous waypoint and compare them to 
                 * make sure that they match */
                if ((new_msg == TRUE) && (cmd == MAV_CMD_NAV_WAYPOINT)) {
                    wp_b_lat_lon[0] = wp_received_ll[0];
                    wp_b_lat_lon[1] = wp_received_ll[1];

                    if (lin_tra_calc_dist(wp_a_lat_lon, wp_b_lat_lon) < TOL) {
                        wp_prev_lla[0] = wp_b_lat_lon[0]; /* latitude */
                        wp_prev_lla[1] = wp_b_lat_lon[1]; /* longitude */
                        wp_prev_lla[2] = 0.0; /* altitude */

                        /* Convert to ENU */
                        lin_tra_lla_to_enu(wp_prev_lla, ref_lla, wp_prev_enu);

                        /* The next waypoint becomes the previous waypoint*/
                        wp_prev_en[0] = wp_prev_enu[1];
                        wp_prev_en[1] = wp_prev_enu[0]; /* @TODO: FIX ENU->NED*/
                        lin_tra_set_prev_wp(wp_prev_en);

                        publish_ack(CHECKING_PREV_WP);

                        current_wp_state = WAITING_FOR_NEXT_WP;
                    } else {
                        publish_ack(ERROR_WP);
                        current_wp_state = WAITING_FOR_PREV_WP;
                    }
                }
                break;

            case WAITING_FOR_NEXT_WP:
                if ((new_msg == TRUE) && (cmd == MAV_CMD_NAV_WAYPOINT)) {
                    wp_a_lat_lon[0] = wp_received_ll[0];
                    wp_a_lat_lon[1] = wp_received_ll[1];

                    publish_waypoint(wp_a_lat_lon); /* Echo back the received 
                                                     * waypoint */

                    current_wp_state = CHECKING_NEXT_WP;
                } else {
                    /* Repeat the ACK CHECKING_PREV_WP message in case the 
                     * Companion Computer missed it */
                    publish_ack(CHECKING_PREV_WP);
                }
                break;

            case CHECKING_NEXT_WP:
                /* Check for a second next waypoint and compare them to make
                 * sure that they match */
                if ((new_msg == TRUE) && (cmd == MAV_CMD_NAV_WAYPOINT)) {
                    wp_b_lat_lon[0] = wp_received_ll[0];
                    wp_b_lat_lon[1] = wp_received_ll[1];

                    if (lin_tra_calc_dist(wp_a_lat_lon, wp_b_lat_lon) < TOL) {
                        wp_next_lla[0] = wp_b_lat_lon[0]; /* latitude */
                        wp_next_lla[1] = wp_b_lat_lon[1]; /* longitude */
                        wp_next_lla[2] = 0.0; /* altitude */

                        if (last_wp_state == TRACKING_WP) {
                            wp_prev_en[0] = wp_next_en[0];
                            wp_prev_en[1] = wp_next_en[1];
                            lin_tra_set_prev_wp(wp_prev_en);
                        }

                        /* Convert to ENU */
                        lin_tra_lla_to_enu(wp_next_lla, ref_lla, wp_next_enu);

                        /* Get a new next waypoint */
                        wp_next_en[0] = wp_next_enu[1];
                        wp_next_en[1] = wp_next_enu[0];
                        lin_tra_set_next_wp(wp_next_en);

                        /* Now we can initialize the linear trajectory */
                        lin_tra_init(wp_prev_en, wp_next_en, vehi_pt_en);

                        publish_ack(CHECKING_NEXT_WP); // SUCCESS

                        current_wp_state = TRACKING_WP;
                    } else {
                        publish_ack(ERROR_WP); // FAILURE

                        current_wp_state = WAITING_FOR_NEXT_WP;
                    }
                }
                break;

            case TRACKING_WP:
                last_wp_state = current_wp_state;

                /* Check for new next waypoint */
                if ((new_msg == TRUE) && (cmd == MAV_CMD_NAV_WAYPOINT)) {
                    wp_a_lat_lon[0] = wp_received_ll[0];
                    wp_a_lat_lon[1] = wp_received_ll[1];

                    current_wp_state = CHECKING_NEXT_WP;
                }
                break;
        }
        new_msg = FALSE; /* Set the new message as FALSE after the state 
                          * machine has run*/

        /**********************************************************************
         * CONTROL: Control and publish data                                  *
         *********************************************************************/
        if ((cur_time - control_start_time) >= CONTROL_PERIOD) {
            control_start_time = cur_time; //reset control loop timer

            set_control_output(); // set actuator outputs

            IMU_state = IMU_start_data_acq(); //initiate IMU measurement with SPI

            if (IMU_state == ERROR) {
                IMU_error++;
                if (IMU_error % error_report == 0) {

#ifdef USB_DEBUG
                    printf("IMU error count %d\r\n", IMU_error);
#endif

                }
            }
            /******************************************************************
             * Estimate the gyro drift with a moving average                  *
             *****************************************************************/
            //            gyro_x_buf[i_gyro] = gyro[0] / GYRO_N;
            //            gyro_y_buf[i_gyro] = gyro[1] / GYRO_N;
            //            gyro_z_buf[i_gyro] = gyro[2] / GYRO_N;
            //
            //            i_gyro = i % GYRO_BUF_LEN;
            //
            //            /* Update the average gyro bias */
            //            if (i_gyro == 0) {
            //
            //                gyro_bias[0] = 0.0;
            //                gyro_bias[2] = 0.0;
            //                gyro_bias[3] = 0.0;
            //
            //                for (j = 0; j < GYRO_BUF_LEN; j++) {
            //                    gyro_bias[0] += gyro_x_buf[j];
            //                    gyro_bias[1] += gyro_y_buf[j];
            //                    gyro_bias[2] += gyro_z_buf[j];
            //                }
            //
            //                gyro_init_avg = TRUE;
            //            }
            //
            //            i++;

            /******************************************************************
             * Control                                                        *
             *****************************************************************/
            if ((current_mode == MAV_MODE_AUTO_ARMED) &&
                    (lin_tra_is_initialized() == TRUE)) {

                if (lin_tra_update(vehi_pt_en) == SUCCESS) {
                    cross_track_error = lin_tra_get_cte();
                    path_angle = lin_tra_get_path_angle();
                    heading_angle = yaw;
                    heading_angle_diff = heading_angle - path_angle;

                    heading_angle_diff = fmod(
                            (heading_angle_diff + M_PI),
                            (2.0 * M_PI)) - M_PI;
                }


                u = pid_controller_update(
                        &trajectory_tracker,
                        0.0, // Commanded reference
                        0.0, //cross_track_error,
                        heading_angle_diff); // Change in heading angle over time

                // Scale resulting control input
                u /= UPPER_ACT_BOUND;
                u *= (SERVO_DELTA);
                u += ((float) RC_RX_MID_COUNTS);
                u_pulse = ((uint16_t) u);

                RC_servo_set_pulse(u_pulse, RC_STEERING);

            }

            /******************************************************************
             * Complementary filter attitude and heading reference system     *
             *****************************************************************/
            /* Update */
            cf_ahrs_update(acc, mag, gyro, &yaw, &pitch, &roll);

            // Information from trajectory


            /******************************************************************
             * Publish data                                                   *
             *****************************************************************/
            publish_RC_signals_raw();
            publish_IMU_data(SCALED);
            publish_attitude(roll, pitch, yaw,
                    path_angle, /* Using differently on purpose */
                    heading_angle_diff, /* Using differently on purpose */
                    (float) u_pulse); /* Using differently on purpose */ /* @TODO: add rates */
            publisher_set_mode(current_mode); // Sets mode in heartbeat

            control_loop_count++;
        }

        // Publish GPS
        //#ifdef USING_GPS
        if (cur_time - gps_start_time >= GPS_PERIOD) {
            gps_start_time = cur_time; //reset GPS timer
            publish_GPS();
        }
        //#endif
        // Publish heartbeat
        if (cur_time - heartbeat_start_time >= HEARTBEAT_PERIOD) {
            heartbeat_start_time = cur_time; //reset the timer
            publish_heartbeat(); // TODO: add argument to update the mode 

            LATAbits.LATA3 ^= 1; /* Set LED4 */

        }
    }

    return 0;
}