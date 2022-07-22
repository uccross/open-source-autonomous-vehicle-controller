/*
 * File: range_at_angle.h
 * Author: Bhuml Depani
 * Brief: This is one of the mode of sensor driver module. In this file, there 
 * are functions to find the range (distance) at a particular angle.
 * Created on 07/12/2022 07:35 pm
*/

#ifndef RANGEATANGLE_H
#define RANGEATANGLE_H

/*******************************************************************************
 * PUBLIC #INCLUDES                                                            *
 ******************************************************************************/

#include "pico/stdlib.h"                            // Pico Standard Library

/*******************************************************************************
 * PUBLIC #DEFINES                                                            *
 ******************************************************************************/

typedef struct angle_an_range
{
    int16_t angle;
    uint16_t range;
}angle_and_range;

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                           *
 ******************************************************************************/

/*
 * @Function uint16_t initialize_system_components(void)
 * @param None
 * @return an initial angle of magnet, read by rotary encoder
 * @brief Initializes PWM in Pico for MG90S servo motor, initializes I2C 
 * communication for V3HP LiDAR and initialized SPI communication with required
 * speed and on particular GPIO pins of Pico.
 * @author Bhumil Depani
 */
uint16_t initialize_system_components(void);

/*
 * @Function angle_and_range get_angle_and_range(uint16_t initial_angle)
 * @param initial_angle, an initial angle measured by encoder, while booting up
 * system
 * @return a structure angle_and_range
 * @brief measures a distnace using LiDAR, measures an angle using an encoder 
 * at current position of the servo motor, and combine both measurements in the
 * angle_and_range structure
 * @author Bhumil Depani
 */
angle_and_range get_angle_and_range(uint16_t initial_angle);

/*
 * @Function void range_at_angle_mode(int16_t angle, uint16_t initial_angle)
 * @param angle, the angle at which want to measure a distance
 * @param initial_angle, an initial angle measured by encoder, while booting up
 * system
 * @return None
 * @brief this function implements one of the mode of a sensor driver module. A 
 * function will first send command to motor, 
 * @author Bhumil Depani
 */
void range_at_angle_mode(int16_t angle, uint16_t initial_angle);

#endif      // RANGEATANGLE_H
