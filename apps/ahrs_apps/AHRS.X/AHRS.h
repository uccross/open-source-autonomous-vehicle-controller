/* ************************************************************************** */
/** Attitude Heading Reference System
 * File:   AHRS.h
 * Author: Aaron Hunter
 * Brief: Implements the complementary filter described by Mahoney 2008
 * Created on 08/05/2022
 * Modified on 
 */
/* ************************************************************************** */

#ifndef AHRS_H    /* Guard against multiple inclusion */
#define AHRS_H

/*******************************************************************************
 * PUBLIC #INCLUDES                                                            *
 ******************************************************************************/

#include <stdint.h>
#include "lin_alg_float.h"

/*******************************************************************************
 * PUBLIC #DEFINES                                                             *
 ******************************************************************************/


/*******************************************************************************
 * PUBLIC TYPEDEFS                                                             *
 ******************************************************************************/


/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES                                                  *
 ******************************************************************************/


/**
 * @Function 
 * @param none
 * @return current magnetic aiding vector
 * @brief 
 * @note 
 * @author Aaron Hunter, 08/05/2022
 * @modified */
void AHRS_get_mag_inertial(float mag_i[MSZ]);

/**
 * @Function 
 * @param none
 * @return current magnetic aiding vector
 * @brief 
 * @note 
 * @author Aaron Hunter, 08/05/2022
 * @modified */
void AHRS_set_mag_inertial(float mag_i[MSZ]);

/**
 * @Function 
 * @param none
 * @return current filter gains
 * @brief 
 * @note 
 * @author Aaron Hunter, 08/05/2022
 * @modified */
void AHRS_get_filter_gains(float *kp_a_get, float *ki_a_get, float *kp_m, float *ki_m_get);

/**
 * @Function 
 * @param none
 * @return none
 * @brief sets CF filter gains
 * @note 
 * @author Aaron Hunter, 08/05/2022
 * @modified */
void AHRS_set_filter_gains(float kp_a_set, float ki_a_set, float kp_m_set, float ki_m_set);

/**
 * @Function AHRS_update
 * @param IMU data in the form of three axis
 * normalized magnetometer and acceleromter arrays, gyro data in rad/sec
 * @param dt, the integration time in seconds
 * @param mag_i normalized inertial aiding vector of magnetic field at location
 * @params kp_a, kp_i, km_p, km_i filter gains
 * @return attitude quaternion and gyro biases vector (x,y,z)
 * @brief implements the complementary filter update step 
 * @note 
 * @author Aaron Hunter, 08/05/2022
 * @modified  11/01/22 to return quaternion attitude and bias values rather
 * than Euler angles*/
void AHRS_update(float accels[MSZ], float mags[MSZ], float gyros[MSZ], 
        float dt, float q[QSZ], float bias[MSZ]);

#endif /* AHRS_H */

/* *****************************************************************************
 End of File
 */
