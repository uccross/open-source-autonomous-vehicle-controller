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

struct range_and_angle
{
    int16_t angle;
    uint16_t range;
};

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
 * @Function struct range_and_angle get_range_at_angle(int16_t angle, uint16_t
 * initial_angle)
 * @param angle, an angle whre motor will rotate LiDAR to and LiDAR will measure
 * a range
 * @param initial_angle, initial magnet angle, read by rotary encoder
 * @return a structure variable, which contains angle and range information
 * @brief this function rotates servo motor at an angle, given by input, 
 * measures distance using LiDAR, measures angle using encoder and return 
 * distnace and angle
 * @author Bhumil Depani
 */
struct range_and_angle get_range_at_angle(int16_t angle, uint16_t 
initial_angle);

#endif      // RANGEATANGLE_H
