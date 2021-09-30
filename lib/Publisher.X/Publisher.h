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

/******************************************************************************
 * FUNCTION PROTOTYPES                                                        *
 *****************************************************************************/
/**
 * @function check_IMU_events(void)
 * @param data_type RAW or SCALED
 * @brief detects when IMU SPI transaction completes and then publishes data 
 * over Mavlink
 * @author Aaron Hunter (modified by Pavlo Vlastos)
 */
void check_IMU_events(uint8_t data_type);

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
 * @param none
 * @brief checks for GPS messages, parses, and stores data in module gps 
 * variable
 * @author aaron hunter
 */
void check_GPS_events(void);

/**
 * @function publish_IMU_data()
 * @param data_type RAW or SCALED
 * @brief reads module level IMU data and publishes over radio or USB serial in 
 * Mavlink
 * @author Aaron Hunter
 */
void publish_IMU_data(uint8_t data_type);

/**
 * @function publish_RC_signals_raw(void)
 * @param none
 * @brief scales raw RC signals
 * @author Aaron Hunter
 */
void publish_RC_signals_raw(void);

/**
 * @function check_mavlink_serial_events(void)
 * @param none
 * @brief scales raw RC signals
 * @author Pavlo Vlastos
 */
void check_mavlink_serial_events(void);

/**
 * @Function publish_encoder_data()
 * @param none
 * @brief looks for messages sent over the radio serial port to OSAVC, parses
 * them and provides responses, if needed
 * @author Aaron Hunter
 */
void publish_encoder_data(void);

/**
 * @function publish_GPS(void)
 * @param none
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