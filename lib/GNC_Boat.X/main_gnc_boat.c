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
#define SERVO_PAD 30
#define ACTUATOR_SATURATION (M_PI/20.0)/SAMPLE_TIME

#define FINDING_REF_WP_PRD 250 // Time to wait to check reference waypoint in milliseconds
#define STATE_MACHINE_PRD 2 // Time to wait to check reference waypoint in milliseconds
#define TOL 0.00001

/******************************************************************************
 * MAIN                                                                       *
 *****************************************************************************/
int main(void) {
    uint32_t cur_time = 0;
    uint32_t gps_start_time = 0;
    uint32_t control_start_time = 0;
    uint32_t heartbeat_start_time = 0;
    uint32_t sm_start_time = 0;
    //    uint8_t index;
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
    //    float wp_a_en[DIM]; // [EAST (lon), NORTH (lat)]
    float wp_b_lat_lon[DIM];
    float wp_received[DIM];
    float wp_prev[DIM];
    float wp_next[DIM];

    float ref_lla[DIM + 1];

    float cross_track_error = 0.0;
    float closest_point[DIM];
    float path_angle = 0.0;
    float yaw_rate = 0.0;
    float u = 0.0; // Resulting control effort
    uint16_t u_pulse = 0; // Pulse from converted control effort

    // Controller
    pid_controller_t trajectory_tracker;
    pid_controller_init(&trajectory_tracker,
            SAMPLE_TIME, // dt The sample time
            10.0, // kp The initial proportional gain
            0.001, // ki The initial integral gain
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
    uint32_t find_wp_ref_time = 0;
    uint8_t waypoints_ready = FALSE;

    enum waypoints_state {
        FINDING_REF_WP, /* Find the reference point for the LTP calculation */
        CHECKING_REF_WP, /* Check the reference point sending and comparing 
                          * echo from Companion Computer */
        WAITING_FOR_PREV_WP, /* Wait for the previous waypoint from the 
                              * Companion Computer */
        CHECKING_PREV_WP, /* Echo test the previous waypoint */
        WAITING_FOR_NEXT_WP, /* Wait for the next waypoint from the 
                              * Companion Computer */
        CHECKING_NEXT_WP /* Echo test the next waypoint */
    };
    typedef enum waypoints_state waypoints_state_t;

    waypoints_state_t current_wp_state = FINDING_REF_WP;
    char new_msg = FALSE;
    char new_gps = FALSE;


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
        check_IMU_events(SCALED);
        new_msg = check_mavlink_serial_events(wp_received, &cmd);
        check_RC_events(); //check incoming RC commands
        new_gps = check_GPS_events(); //check and process incoming GPS messages

        // Check if mode switch event occurred
        current_mode = check_mavlink_mode();

        //        if (current_mode != last_mode) {
        //            last_mode = current_mode;
        //            publisher_get_gps_rmc_position(point);
        //            //            lin_tra_init(position,);
        //        }

        /**********************************************************************
         * Wayopint State Machine Logic                                       *
         *********************************************************************/
        //        if ((cur_time - sm_start_time) > STATE_MACHINE_PRD) {
        //            sm_start_time = cur_time;
        switch (current_wp_state) {
            case FINDING_REF_WP:
                if (new_gps == TRUE) {
                    publisher_get_gps_rmc_position(wp_a_lat_lon);
                    //                    wp_a_en[0] = wp_a_lat_lon[1];
                    //                    wp_a_en[1] = wp_a_lat_lon[0];
                    publish_waypoint(wp_a_lat_lon);
                    find_wp_ref_time = cur_time;
                }

                // State exit case
                if ((new_msg == TRUE) && (cmd == MAV_CMD_NAV_WAYPOINT)) {
                    wp_b_lat_lon[0] = wp_received[0];
                    wp_b_lat_lon[1] = wp_received[1];
                    current_wp_state = CHECKING_REF_WP;
                }

                break;

            case CHECKING_REF_WP:

                LATCbits.LATC1 ^= 1; // Toggle LED 5

                // State exit case
                if (lin_tra_calc_dist(wp_a_lat_lon, wp_b_lat_lon) < TOL) {
                    ref_lla[0] = wp_a_lat_lon[0]; // latitude
                    ref_lla[1] = wp_a_lat_lon[1]; // longitude
                    ref_lla[2] = 0.0; // Altitude

                    publish_ack(1); // SUCCESS

                    current_wp_state = WAITING_FOR_PREV_WP;
                } else {
                    publish_ack(0); // FAILURE

                    current_wp_state = FINDING_REF_WP;
                }

                break;

            case WAITING_FOR_PREV_WP:

                if ((new_msg == TRUE) && (cmd == MAV_CMD_NAV_WAYPOINT)) {
                    wp_a_lat_lon[0] = wp_received[0];
                    wp_a_lat_lon[1] = wp_received[1];

                    publish_waypoint(wp_a_lat_lon); /* Echo back the received 
                                                 * waypoint */
                    current_wp_state = CHECKING_PREV_WP;
                } else {
                    publish_ack(1); // SUCCESS
                }
                break;

            case CHECKING_PREV_WP:
                if (new_msg == TRUE) {
                    wp_b_lat_lon[0] = wp_received[0];
                    wp_b_lat_lon[1] = wp_received[1];

                    if (lin_tra_calc_dist(wp_a_lat_lon, wp_b_lat_lon) < TOL) {
                        wp_prev[0] = wp_a_lat_lon[0];
                        wp_prev[1] = wp_a_lat_lon[1];
                        wp_prev[2] = 0.0;

                        publish_ack(1); // SUCCESS

                        current_wp_state = WAITING_FOR_NEXT_WP;
                    } else {
                        publish_ack(0); // FAILURE

                        current_wp_state = WAITING_FOR_PREV_WP;
                    }
                }
                break;

            case WAITING_FOR_NEXT_WP:
                break;

            case CHECKING_NEXT_WP:
                break;
        }
        new_msg = FALSE; /* Set the new message as FALSE after the state 
                              * machine has run*/
        //        }

        /**********************************************************************
         * CONTROL: Control and publish data                                  *
         *********************************************************************/
        if ((cur_time - control_start_time) > CONTROL_PERIOD) {
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
             * Control                                                        *
             *****************************************************************/
            //            if ((current_mode == MAV_MODE_AUTO_ARMED) &&
            //              (waypoints_ready == TRUE)) {

            //            cross_track_error = 
            //            
            //            u = pid_controller_update(
            //                    &trajectory_tracker,
            //                    0.0, // Commanded reference
            //                    cross_track_error,
            //                    yaw_rate); // Change in heading angle over time
            //
            //            // Scale resulting control input
            //            u_pulse = (float) (RC_RX_MID_COUNTS);
            //            u_pulse -= (((u * (float) ((((float) RC_RX_MID_COUNTS)
            //                    - ((float) (RC_RX_MIN_COUNTS + SERVO_PAD)))))
            //                    / ((float) ACTUATOR_SATURATION)));

            //            RC_servo_set_pulse(u_pulse, RC_STEERING);

            //            }

            // Information from trajectory


            /******************************************************************
             * Publish data                                                   *
             *****************************************************************/
            publish_RC_signals_raw();
            publish_IMU_data(SCALED);
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