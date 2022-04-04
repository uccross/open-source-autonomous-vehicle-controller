/*
 * File:   Publisher.h
 * Author: Aaron Hunter (modified by Pavlo Vlastos)
 * Brief:  Publish events after checking
 * 
 * Updated from Mavlink_minimal.X/main.c on September 3, 2021 at 2:24 pm
 */

#ifndef PUBLISHER_H // Header guard
#define	PUBLISHER_H //

/******************************************************************************
 * PUBLIC #INCLUDES                                                           *
 *****************************************************************************/
#include "RC_RX.h"
#include "RC_servo.h"
#include "ICM_20948.h"
#include "MavSerial.h"
#include "System_timer.h"
#include "nmea0183v4.h"
#include "linear_trajectory.h"

/******************************************************************************
 * PUBLIC  #DEFINES                                                           *
 *****************************************************************************/
#define LLA_DIM 3 // DO NOT CHANGE THIS

#define WP_PREV ((float) 1.0)
#define WP_NEXT ((float) 1.5)
#define VEHI_PT ((float) 2.0)


/******************************************************************************
 * PUBLIC DATATYPES                                                           *
 *****************************************************************************/


/******************************************************************************
 * FUNCTION PROTOTYPES                                                        *
 *****************************************************************************/

/**
 * @function publisher_init(uint8_t desired_autopilot)
 * @param desired_autopilot Example: MAV_TYPE_GROUND_ROVER, or
 * MAV_TYPE_SURFACE_BOAT
 * @brief Set the autopilot, mode, and state of the vehicle
 * @author Pavlo Vlastos
 */
void publisher_init(uint8_t desired_autopilot);

/**
 * @function publisher_set_autopilot(uint8_t desired_autopilot)
 * @param desired_autopilot Example: MAV_TYPE_GROUND_ROVER, or
 * MAV_TYPE_SURFACE_BOAT
 * @brief Set the autopilot
 * @author Pavlo Vlastos
 */
void publisher_set_autopilot(uint8_t desired_autopilot);

/**
 * @function publisher_set_mode(uint8_t desired_mode)
 * @param desired_mode Example: MAV_MODE_MANUAL_DISARMED, 
 * MAV_MODE_MANUAL_ARMED, MAV_MODE_AUTO_ARMED, ...
 * @brief Set the mode
 * @author Pavlo Vlastos
 */
void publisher_set_mode(uint8_t desired_mode);

/**
 * @function publisher_set_state(uint8_t desired_state)
 * @param desired_state Example: MAV_STATE_STANDBY, MAV_STATE_ACTIVE, ...
 * @brief Set the state
 * @author Pavlo Vlastos
 */
void publisher_set_state(uint8_t desired_state);

/**
 * @function check_mavlink_mode(void)
 * @param none
 * @brief Reads RC_channel (Channel 4) to determine if manual or autonomous. 
 * This function returns the MAVLink mode (uint8_t) such as 
 * MAV_MODE_MANUAL_ARMED
 * @author Pavlo Vlastos
 */
uint8_t check_mavlink_mode(void);

/**
 * @function publisher_get_gps_rmc_position(float position[DIM])
 * @param position[] Latitude and longitude 
 * @brief Wrapper function to get the gpc position
 * @author Pavlo Vlastos
 */
void publisher_get_gps_rmc_position(float position[DIM]);

/**
 * @function check_HIL_IMU_events(struct IMU_output *data)
 * @param data An IMU_output struct pointer
 * @return TRUE or FALSE if a new IMU event received from the Companion 
 * Computer
 */
char check_HIL_IMU_events(struct IMU_output *data);

/**
 * @function check_IMU_events(uint8_t data_type, struct IMU_output *data)
 * @brief detects when IMU SPI transaction completes and then publishes data 
 * over Mavlink
 * @param data_type RAW or SCALED
 * @param data An IMU_output struct pointer
 * @return TRUE or FALSE if a new IMU event was detected
 * @author Aaron Hunter (modified by Pavlo Vlastos)
 */
char check_IMU_events(uint8_t data_type, struct IMU_output *data);

/**
 * @function RC_channels_init(void)
 * @param none
 * @brief set all RC channels to RC_RX_MID_COUNTS
 * @author Aaron Hunter
 */
void RC_channels_init(void);

/**
 * @function check_RC_events(void)
 * @param none
 * @brief checks for RC messages and stores data in RC channel buffer
 * @author Aaron Hunter
 */
void check_RC_events();

/**
 * @function check_GPS_events(void)
 * @brief checks for GPS messages, parses, and stores data in module gps 
 * variable
 * @return TRUE or FALSE if a GPS NMEA frame was available
 * @author aaron hunter, modified by Pavlo Vlastos
 */
char check_GPS_events(void);

/**
 * @function publish_IMU_data()
 * @param data_type RAW or SCALED
 * @brief reads module level IMU data and publishes over radio or USB serial in 
 * Mavlink
 * @author Aaron Hunter
 */
void publish_IMU_data(uint8_t data_type);

/**
 * @function publish_attitude(float roll, float pitch, float yaw, 
 *      float roll_rate, float pitch_rate, float yaw_rate)
 * @breif send an ATTITUDE MAVLink message to the companion computer
 * @param roll Angle in radians
 * @param pitch Angle in radians
 * @param yaw Angle in radians
 * @param roll_rate Angular velocity in radians per second
 * @param pitch_rate Angular velocity in radians per second
 * @param yaw_rate Angular velocity in radians per second
 * @author Pavlo Vlastos
 */
void publish_attitude(float roll, float pitch, float yaw, float roll_rate,
        float pitch_rate, float yaw_rate);

/**
 * @function publish_HIL_servo_output_raw(uint16_t servo4_raw)
 * @param servo4_raw
 * @author Pavlo Vlastos
 */
void publish_HIL_servo_output_raw(uint16_t servo4_raw);

/**
 * @function publish_RC_signals_raw(void)
 * @param none
 * @brief scales raw RC signals
 * @author Aaron Hunter
 */
void publish_RC_signals_raw(void);

/**
 * @function check_mavlink_serial_events(void)
 * @brief Check mavlink serial events/messages and populate the waypoint 
 * parameter as necessary
 * @param wp A waypoint with longitude and latitude in that order
 * @param *msgid A pointer for getting the MAVLink message ID
 * @param *command A pointer for getting the MAVLink command
 * @param *wp_type A pointer for getting what type of waypoint: previous or 
 * next? (0.0 is previous, 1.0 is next) This is a temporary use of the altitude 
 * @param *yaw A pointer for getting a yaw angle associated with the waypoint
 * parameter for the MAV_CMD_NAV_WAYPOINT message.
 * @return TRUE or FALSE if an message was received
 * @author Pavlo Vlastos
 */
char check_mavlink_serial_events(float wp[DIM], uint32_t *msgid,
        uint16_t *command, float *wp_type, float *yaw, float *kp, float *kd);

/**
 * @function publish_GPS(void)
 * @brief invokes mavlink helper function to generate GPS message and sends to
 * radio or USB serial port
 * @author aaron hunter
 */
void publish_GPS(void);

/**
 * @Function publish_heartbeat(void)
 * @param none
 * @brief invokes mavlink helper to generate heartbeat and sends out via the 
 * USB
 * @author aaron hunter
 */
void publish_heartbeat(void);

/**
 * @Function publish_parameter(uint8_t param_id[16])
 * @param parameter ID
 * @brief invokes mavlink helper to send out stored parameter 
 * @author aaron hunter
 */
void publish_parameter(const char *param_id);

/**
 * @Function publish_waypoint_ll(float wp_lat_lon[DIM])
 * @brief Send waypoint with North and East elements to the companion 
 * @param wp_lat_lon[] A waypoint with North and East elements within the Local 
 * Tangent Plane (LTP) in meters. 
 * NOTE: Order matters here; [North (lat), East (lon)]
 * @return SUCCESS or FAILURE
 * @author Pavlo Vlastos
 */
int publish_waypoint_ll(float wp_lat_lon[DIM]);

/**
 * @Function publish_waypoint_en(float wp_en[DIM])
 * @brief Send waypoint with East and North elements to the companion 
 * @param wp_en[] A waypoint with North and East elements within the Local 
 * @param prev_next_vehi A value to indicate if the waypoint is prev: 1.0, 
 * next: 1.5, or vehicle location: 2.0
 * Tangent Plane (LTP) in meters. 
 * NOTE: Order matters here; [North (meters), East (meters)]
 * @return SUCCESS or FAILURE
 * @author Pavlo Vlastos
 */
int publish_waypoint_en(float wp_en[DIM], float prev_next_vehi);

/**
 * @Function publish_ack(uint8_t result)
 * @brief Send an acknowledgment to the Companion Computer
 * @param result Information associated with the acknowledgment
 * @author Pavlo Vlastos
 */
int publish_ack(uint8_t result);

/**
 * @Function calc_pw(uint16_t raw_counts)
 * @param raw counts from the radio transmitter or USB (11 bit unsigned int)
 * @return pulse width in microseconds
 * @brief converts the RC input into the equivalent pulsewidth output for servo
 * and ESC control
 * @author aahunter
 * @modified <Your Name>, <year>.<month>.<day> <hour> <pm/am> */
uint16_t calc_pw(uint16_t raw_counts);

/**
 * @Function set_control_output(void)
 * @param none
 * @return none
 * @brief converts RC input signals to pulsewidth values and sets the actuators
 * (servos and ESCs) to those values
 * @author Aaron Hunter
 */
void set_control_output(void);

#endif	/* PUBLISHER_H */ // End of header guard