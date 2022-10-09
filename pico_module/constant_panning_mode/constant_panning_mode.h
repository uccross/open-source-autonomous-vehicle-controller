/*
 * File: constant_panning_mode.h
 * Author: Bhuml Depani
 * Brief: This is one of the modes of sensor driver module. In this file, there 
 * are functions to scan across the field of view.
 * Created on 08/15/2022 07:27 am
*/

#ifndef CONSTANTPANNINGMODE_H
#define CONSTANTPANNINGMODE_H

/*******************************************************************************
 * PUBLIC #INCLUDES                                                            *
 ******************************************************************************/

#include "../range_at_angle/range_at_angle.h"

/*******************************************************************************
 * PUBLIC #DEFINES                                                            *
 ******************************************************************************/

#define ANGLE_INCREMENT 1000        /* the angle increment in centidegree; the measurement will be taken on 0, 10, 20... degrees

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                           *
 ******************************************************************************/

/*
 * @Function int16_t start_constant_panning_mode(uint16_t angle)
 * @param angle, an initial angle used by the encoder
 * @return offset angle value
 * @brief this is a function to initiate the constant panning mode; this 
 * function should be called when we first start constant panning mode
 * @author Bhumil Depani
 */
int16_t start_constant_panning_mode(uint16_t angle);

/*
 * @Function angle_and_range continue_constant_panning_mode(void)
 * @param None
 * @return None
 * @brief this is a function to drive the constant panning mode; this 
 * function should be called continuously after entering in the constant panning
 * mode
 * @author Bhumil Depani
 */
angle_and_range continue_constant_panning_mode(void);

#endif  // CONSTANTPANNINGMODE_H
