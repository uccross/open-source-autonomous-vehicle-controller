/*
 * File:   triad_ahrs.h
 * Author: Pavlo Vlastos
 *
 * Created on January 28, 2020, 12:47 PM
 */

#ifndef TRIAD_AHRS_H
#define TRIAD_AHRS_H

/******************************************************************************
 * PUBLIC #include                                                            *
 *****************************************************************************/
#include "Lin_alg_float.h"

/******************************************************************************
 * PUBLIC #DEFINES                                                            *
 *****************************************************************************/

/******************************************************************************
 * PUBLIC VARIABLES                                                           *
 *****************************************************************************/

/******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES                                                 *
 *****************************************************************************/

/**
 * @function cf_ahrs_init()
 * Initialize the private varibles used in cf_ahrs_update()
 * @TODO: Do calibration here?
 * @param desired_dt The desired sample time in seconds
 */
void triad_ahrs_init(float desired_dt);

/**
 * @function triad_ahrs_update()
 * @param w1 Accelerometer body-fixed aiding reference vector measurement 
 * @param w2 Magnetometer body-fixed aiding reference vector measurement 
 * @param yaw Output yaw angle in radians from -pi to pi, passed by reference
 * @param pitch Output pitch angle in radians from -pi to pi, passed by reference
 * @param roll Output yaw angle in radians from -pi to pi, passed by reference
 */
void triad_ahrs_update(float w1[MSZ], float w2[MSZ], float *yaw, 
        float *pitch, float *roll);


#endif  /* TRIAD_AHRS_H */