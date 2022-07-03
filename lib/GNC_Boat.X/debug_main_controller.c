/*
 * File:   debug_main_controller.c
 * Author: Pavlo Vlastos
 * Brief:  A small helper program to help debug models and controllers. 
 * Created on September 30, 2021, 4:09 PM
 */

/******************************************************************************
 * #INCLUDES                                                                  *
 *****************************************************************************/
#include "xc.h"
#include "Board.h"
#include "System_timer.h"
#include "linear_trajectory.h"
#include "pid_controller.h"
#include "common/mavlink.h"
#include "RC_servo.h"
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
#define SIM_PERIOD 2 //Period for control loop in msec
#define CONTROL_PERIOD 20 //Period for control loop in msec
#define PRINT_PERIOD 200
#define SAMPLE_TIME (((float) CONTROL_PERIOD)*(0.001))
#define RAW 1
#define SCALED 2

#define K_ACT ((float) 0.01)
#define UPPER_ACT_BOUND ((float) 50.0) * K_ACT //((float) 0.8) // The maximum rudder actuator limit in radians
#define LOWER_ACT_BOUND ((float) -50.0) * K_ACT //((float)-0.8) // The minimum rudder actuator limit in radians
#define RANGE_ACT (UPPER_ACT_BOUND - LOWER_ACT_BOUND) // Range of actuator
#define SERVO_PAD 0.0
#define SERVO_DELTA ((float) (RC_SERVO_CENTER_PULSE - RC_SERVO_MIN_PULSE))

#define TRANSMIT_PRD 500 // Time to wait to check reference waypoint in milliseconds
#define STATE_MACHINE_PRD 2 // Time to wait to check reference waypoint in milliseconds
#define TOL 0.0001
#define MAX_ACCEPTABLE_CTE ((float) 3.0)

#define RAD_2_DEG (180.0 / M_PI)
#define DEG_2_RAD (M_PI / 180.0) 

#define WP_THRESHOLD 2.0
#define MIN_TURNING_RADIUS 2.5


/******************************************************************************
 * MAIN                                                                       *
 *****************************************************************************/
int main(void) {
    uint32_t cur_time = 0;
    uint32_t sim_start_time = 0;
    uint32_t control_start_time = 0;
    uint32_t print_start_time = 0;

    /**************************************************************************
     * Initialization routines                                                *
     *************************************************************************/
    Board_init(); //board configuration
    Sys_timer_init(); //start the system timer

    TRISAbits.TRISA3 = 0; /* Set pin as output. This is also LED4 on Max32 */
    TRISCbits.TRISC1 = 0; /* LED5 */

    LATCbits.LATC1 = 1; /* Set LED5 */
    LATAbits.LATA3 = 1; /* Set LED4 */

    cur_time = Sys_timer_get_msec();

    LATCbits.LATC1 = 0; /* Set LED5 low */
    LATAbits.LATA3 = 0; /* Set LED4 low */
    
    // Trajectory 
    float wp_prev_en[DIM]; // [EAST (meters), NORTH (meters)]
    float wp_next_en[DIM]; // [EAST (meters), NORTH (meters)]
    float vehi_pt_en[DIM]; // [EAST (meters), NORTH (meters)]

    float w_mat[5][DIM]; // Matrix of waypoints

    // Zig-zag
    w_mat[0][0] = 0.0;
    w_mat[0][1] = 5.0;

    w_mat[1][0] = 20.0;
    w_mat[1][1] = 5.0;

    w_mat[2][0] = 0.0;
    w_mat[2][1] = 25.0;

    w_mat[3][0] = 10.0;
    w_mat[3][1] = 25.0;

    w_mat[3][0] = 20.0;
    w_mat[3][1] = 15.0;
    
    int i_w = 0;
    
    /* Set initial vehicle position */
    vehi_pt_en[0] = w_mat[i_w][0];
    vehi_pt_en[1] = w_mat[i_w][1];
    
    /* Set initial previous and next waypoints*/
    wp_prev_en[0] = w_mat[i_w][0];
    wp_prev_en[1] = w_mat[i_w][1];
    
    i_w += 1;
    
    wp_next_en[0] = w_mat[i_w][0];
    wp_next_en[1] = w_mat[i_w][1];

    float wp_next_en[DIM]; // [EAST (meters), NORTH (meters)]

    // State
    float state_out[SSZ] = {0.0};
    float state_in[SSZ] = {0.0};
    state_in[2] = wp_prev_en[0];
    state_in[3] = wp_prev_en[1];
    state_in[4] = 0.67;
    
    lin_tra_init(wp_prev_en, wp_next_en, vehi_pt_en);
            
    float cross_track_error = 0.0;
    float path_angle = 0.0;
    float heading_angle = 0.0; /* Heading angle between North unit vector and 
                                * heading vector within the local tangent plane
                                * (LTP) */
    //    float cog_last = 0.0; /* Course over ground angle from GPS */
    //    float heading_angle_buffer[HEADING_ANGLE_BUF];
    float heading_angle_diff = 0.0;
    float ha_report = 0.0;
    float u = 0.0; // Resulting control effort
    float delta_angle = 0.0; // Resulting control effort
    float u_sign = 1.0;
    float acc_cmd = 0.0;
    float rtemp = 0.0;
    float speed = 1.0; /* 'measured' speed in meters per second */
    int u_intermediate = 0; // Pulse from converted control effort
    uint16_t u_pulse = 0; // Pulse from converted control effort



#ifdef USB_DEBUG
    printf("\r\nMinimal Mavlink application %s, %s \r\n", __DATE__, __TIME__);
#endif

    // MAVLink and State Machine
    uint8_t current_mode = MAV_MODE_MANUAL_ARMED;
    float kp_track = 1.0;
    float kd_track = 10.0;

    // Controller
    pid_controller_t trajectory_tracker;
    pid_controller_init(&trajectory_tracker,
            SAMPLE_TIME, // dt The sample time
            kp_track, //0.5, // kp The initial proportional gain
            0.0, // ki The initial integral gain
            kd_track, // kd The initial derivative gain [HIL:0.1]
            1000.0, // The maximum 
            -1000.0); // The minimum 

    enum waypoints_state {
        ERROR_WP = 0,
        FINDING_REF_WP, /* Find the reference point for the LTP calculation */
        SENDING_PREV,
        SENDING_NEXT,
        WAITING_FOR_NEXT_WP, /* Get rid of this */
        TRACKING
    };
    typedef enum waypoints_state waypoints_state_t;


    current_wp_state = FINDING_REF_WP;

    unsigned int control_loop_count = 0;

    uint8_t i_tx_mav_msg = 0;

    /* Data to print */
    float t_msec = 0.0;
    print("%%t_msec, psi, r, x, y, v\r\n");
    
    int i_state = 0;
    
    // Controller
    pid_controller_init(&trajectory_tracker,
                            SAMPLE_TIME, // dt The sample time
                            kp_track, //0.5, // kp The initial proportional gain
                            0.0, // ki The initial integral gain
                            kd_track, // kd The initial derivative gain [HIL:0.1]
                            1000.0, // The maximum 
                            -1000.0); // The minimum 
    
    /**************************************************************************
     * Primary Loop                                                           *
     *************************************************************************/
    while (1) {
        cur_time = Sys_timer_get_msec();

        // Check if mode switch event occurred
        current_mode = MAV_MODE_AUTO_ARMED;

        /* Waypoint transition logic that can otherwise be sent by the 
         * Raspberry Pi (or other comparable companion computer) */
        if (lin_tra_get_dist(vehi_pt_en, wp_next_en) <= WP_THRESHOLD) {
            
            /* Update previous */
            wp_prev_en[0] = wp_next_en[0];
            wp_prev_en[1] = wp_next_en[1];
            
            i_w += 1;
            
            /* Update next */
            wp_next_en[0] = w_mat[i_w][0];
            wp_next_en[1] = w_mat[i_w][1];
            
            lin_tra_init(wp_prev_en, wp_next_en, vehi_pt_en);
        }

        /**********************************************************************
         * SIM: Update model                                                  *
         *********************************************************************/
        if ((cur_time - sim_start_time) >= SIM_PERIOD) {
            sim_start_time = cur_time; //reset control loop timer
            
            for (i_state = 0; i_state < SSZ; i_state++) {
                state_in[i_state] = state_out[i_state];
            }
            
            nomoto_update(state_in, u, state_out);
        }
        
        /**********************************************************************
         * CONTROL: Control and publish data                                  *
         *********************************************************************/
        if ((cur_time - control_start_time) >= CONTROL_PERIOD) {
            control_start_time = cur_time; //reset control loop timer

            /******************************************************************
             * Control                                                        *
             *****************************************************************/
            if ((current_mode == MAV_MODE_AUTO_ARMED) &&
                    (lin_tra_is_initialized() == TRUE)) {
                
                heading_angle = state_out[0]; /* psi */
                vehi_pt_en[0] = state_out[2]; /* x (East) */
                vehi_pt_en[1] = state_out[3]; /* y (North) */

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

                u = -delta_angle;

                // Scale resulting control input
                u *= (SERVO_DELTA - SERVO_PAD);
                u /= UPPER_ACT_BOUND;
                u += ((float) RC_SERVO_CENTER_PULSE);
                u_intermediate = (int) u;
                u_pulse = ((uint16_t) u_intermediate);

                delta_angle = (float) u_pulse;
            }


            // Information from trajectory

            control_loop_count++;
            i_tx_mav_msg++;
            i_tx_mav_msg %= NUM_MSG_SEND_CONTROL_PERIOD;

            LATAbits.LATA3 ^= 1; /* Toggle LED4 */
        }
        
        /**********************************************************************
         * PRINT                                                              *
         *********************************************************************/
        if ((cur_time - print_start_time) >= PRINT_PERIOD) {
            print_start_time = cur_time;
            
            t_msec = ((float) cur_time) / 1000.0;
            
            /* t_msec, psi, r, x, y, v */
            print("%f, %f, %f, %f, %f, %f\r\n", 
                    cur_time, 
                    state_out[0], 
                    state_out[1], 
                    state_out[2], 
                    state_out[3], 
                    state_out[4]);
            
            LATCbits.LATC1 ^= 1; /* Toggle  LED5 */
        }
    }

    return 0;
}