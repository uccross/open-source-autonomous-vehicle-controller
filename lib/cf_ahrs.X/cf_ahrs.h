/*
 * File:   cf_ahrs.h
 * Author: Pavlo Vlastos
 *
 * Created on January 28, 2020, 12:47 PM
 */

#ifndef CF_AHRS_H
#define CF_AHRS_H

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
 * @param exp_gyro_bias Initial gryo biases found experimentally by some means 
 */
void cf_ahrs_init(float desired_dt, const float exp_gyro_bias[MSZ]);

/**
 * @function cf_ahrs_set_dt()
 * @param desired_dt The desired sample time in seconds
 */
void cf_ahrs_set_dt(float desired_dt);

/**
 * @function cf_ahrs_set_gyro_biases()
 * @param exp_gyro_bias Initial gryo biases found experimentally by some means 
 */
void cf_ahrs_set_gyro_biases(const float exp_gyro_bias[MSZ]);

/**
 * @function cf_ahrs_set_kp_acc()
 * @param desired_kp_acc Desired gains on accelerometer correction contribution 
 */
void cf_ahrs_set_kp_acc(float desired_kp_acc);

/**
 * @function desired_kp_mag()
 * @param desired_kp_mag Desired gains on magnetometer correction contribution 
 */
void cf_ahrs_set_kp_mag(float desired_kp_mag);

/**
 * @function cf_ahrs_update()
 * Mahony Complementary Filter update function, uses a combination of 
 * quaternions and direction cosine matrices (DCMs)
 * @TODO: improve to use ONLY quaternions -_'
 * @param acc_vb Accelerometer body-fixed aiding reference vector measurement 
 * @param mag_vb Magnetometer body-fixed aiding reference vector measurement 
 * @param gyro_vb Gyroscope body-fixed angular rates in radians per second 
 *          NOTE: this is the raw gyro reading. The gyro biases are subtracted
 *              internally. You do not need to subtract yourself. You can 
 *              update the gyro baises with cf_ahrs_set_gyro_biases()
 * @param yaw Output yaw angle in radians from -pi to pi, passed by reference
 * @param pitch Output pitch angle in radians from -pi to pi, passed by reference
 * @param roll Output yaw angle in radians from -pi to pi, passed by reference
 */
void cf_ahrs_update(float acc_vb[MSZ], float mag_vb[MSZ],
        const float gyro_vb[MSZ], float *yaw, float *pitch, float *roll);


#endif  /* CF_AHRS_H */