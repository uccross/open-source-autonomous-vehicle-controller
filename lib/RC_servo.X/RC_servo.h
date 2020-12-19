/* 
 * File:   RC_servo.h
 * Author: Aaron Hunter
 * Brief: Library driver for up to four servos for the Max32 dev board
 * Created on 12/18/2020 3;30 pm
 * Modified 
 */


#ifndef RCSERVO_H
#define	RCSERVO_H

/*******************************************************************************
 * PUBLIC #INCLUDES                                                            *
 ******************************************************************************/

#include <stdint.h>

/*******************************************************************************
 * PUBLIC #DEFINES                                                            *
 ******************************************************************************/

#define RC_SERVO_MIN_PULSE 900
#define RC_SERVO_CENTER_PULSE 1500
#define RC_SERVO_MAX_PULSE 2000

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                           *
 ******************************************************************************/

/**
 * @Function RC_servo_init(void)
 * @param None
 * @return SUCCESS or ERROR
 * @brief initializes hardware required and set it to the CENTER PULSE */
uint8_t RC_servo_init(void);

/**
 * @Function int RC_servo_set_pulse(unsigned int inPulse)
 * @param inPulse, integer representing number of microseconds
 * @return SUCCESS or ERROR
 * @brief takes in microsecond count, converts to ticks and updates the internal variables
 * @warning This will update the timing for the next pulse, not the current one */
uint8_t RC_servo_set_pulse(uint16_t in_pulse);

/**
 * @Function int RC_servo_get_pulse(void)
 * @param None
 * @return Pulse in microseconds currently set */
uint16_t RC_servo_get_pulse(void);

/**
 * @Function int RC_servo_get_raw_ticks(void)
 * @param None
 * @return raw timer ticks required to generate current pulse. */
uint16_t RC_servo_get_raw_ticks(void);

#endif	/* RCSERVO_H */

