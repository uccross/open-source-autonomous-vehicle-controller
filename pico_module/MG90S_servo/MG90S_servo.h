/*
 * File: MG90S_servo.c
 * Author: Bhuml Depani
 * Brief: This file is a motor driver module for the MG90S Servo Motor. It
 * contains multiple functions which can be used to control the rotation of the
 * servo motor.
 * Created on 06/22/2022 04:28 pm
*/

#ifndef MG90SSERVO_H
#define MG90SSERVO_H

/*******************************************************************************
 * PUBLIC #INCLUDES                                                            *
 ******************************************************************************/

#include "pico/stdlib.h"                // Pico Standard Library

/*******************************************************************************
 * PUBLIC #DEFINES                                                            *
 ******************************************************************************/

#define MHz 1000000
#define SYSTEM_CLOCK_FREQ 125 * MHz     /* Pico System Clock frequency in Hz
* from Datasheet */

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                           *
 ******************************************************************************/

/*
 * @Function void MG90S_servo_init(void)
 * @param None
 * @return None
 * @brief Initializes PWM with the required PWM Period for motor at pin
 * MOTOR_PIN
 * @author Bhumil Depani
 */
void MG90S_servo_init(void);

/*
 * @Function void MG90S_servo_set_angle(float angle_degree)
 * @param angle_degree, an angle, in degree, where we want servo to rotate, for
 * MG90S Servo Motor angle should be between -90 and 90 degree and can even 
 * give float angle as an input
 * @return None
 * @brief takes an angle in degree, finds PWM duty cycle required, set PWM duty
 * cycle to the MOTOR_PIN 
 * @author Bhumil Depani
 */
void MG90S_servo_set_angle(float angle_degree);

/*
 * @Function uint16_t microsecond_to_pwm_count(float microsecond)
 * @param microsecond, microsecond will be converted into appropriate PWM Count
 * @return PWM Count associated with output microsecond
 * @brief takes microsecond as an input and according to values of 
 * PWM_PERIOD_HZ and pwm_wrap_count, the function will calculate the pwm count*
 * @author Bhumil Depani
 */
uint16_t microsecond_to_pwm_count(float microsecond);


/*
 * @Function float MG90S_servo_get_angle(void)
 * @param None
 * @return angle of the servo motor in degree
 * @brief function can be called to know at which angle servo is remained
 * @author Bhumil Depani
 */
float MG90S_servo_get_angle(void);

#endif  /*MG90SSERVO_H */