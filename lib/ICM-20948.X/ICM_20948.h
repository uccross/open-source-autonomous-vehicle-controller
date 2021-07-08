/* 
 * File:   ICM_20948.h
 * Author: Aaron Hunter
 * Brief: Library for the ICM-20948 IMU
 * Created on Nov 13, 2020 9:46 am
 */

#ifndef ICM_20948_H // Header guard
#define	ICM_20948_H //

/*******************************************************************************
 * PUBLIC #INCLUDES                                                            *
 ******************************************************************************/
#include <sys/types.h>
#include <stdfix.h>
#include "ICM_20948_registers.h"

/*******************************************************************************
 * PUBLIC #DEFINES                                                             *
 ******************************************************************************/
#define IMU_SPI_MODE 0
#define IMU_I2C_MODE 1
/*lin alg constants*/
#define MSZ 3 //matrix/vector size per dimension

/*******************************************************************************
 * PUBLIC TYPEDEFS                                                             *
 ******************************************************************************/
struct IMU_device {
    accum x;
    accum y;
    accum z;
};

struct IMU_output {
    struct IMU_device acc;
    struct IMU_device gyro;
    struct IMU_device mag;
    accum temp;
    uint16_t mag_status;
};

/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES                                                  *
 ******************************************************************************/

/**
 * @Function IMU_init(void)
 * @return SUCCESS or ERROR
 * @brief initializes the I2C system for IMU operation
 * @note 
 * @author Aaron Hunter
 **/
uint8_t IMU_init(char interface_mode);
/**
 * @Function IMU_start_data_acq(void);
 * @return none
 * @param none
 * @brief this function starts the SPI data read
 * @author Aaron Hunter
 **/
int8_t IMU_start_data_acq(void);

/**
 * @Function IMU_is_data_ready(void)
 * @return TRUE or FALSE
 * @brief TRUE if unread data is available
 * @note 
 * @author Aaron Hunter,
 **/
uint8_t IMU_is_data_ready(void);

/**
 * @Function IMU_get_raw_data(void)
 * @return pointer to IMU_output struct 
 * @brief returns most current (raw) data from the IMU
 * @note 
 * @author Aaron Hunter,
 **/
uint8_t IMU_get_raw_data(struct IMU_output* IMU_data);

/**
 * @Function IMU_get_scaled_data(void)
 * @return pointer to IMU_output struct 
 * @brief returns most current scaled data from the IMU
 * @note 
 * @author Aaron Hunter,
 **/
uint8_t IMU_get_scaled_data(struct IMU_output* IMU_data);

/**
 * @Function IMU_set_mag_cal(accum A[MSZ][MSZ], accum b[MSZ])
 * @param cal contains the A scaling matrix and b bias vector for the mag
 * @brief sets scaling matrix and offset vector for magnetometer 
 * @note bias vector is assumed to be normalized to one, so it gets scaled
 * to the expected magnitude of magnetic field, i.e., 475 mGauss
 * @return SUCCESS or ERROR
 * @author Aaron Hunter,
 **/
int8_t IMU_set_mag_cal(accum A[MSZ][MSZ], accum b[MSZ]);

/**
 * @Function IMU_set_acc_cal(accum A[MSZ][MSZ], accum b[MSZ])
 * @param A source matrix
 * @param b source offset
 * @brief sets scaling matrix and offset vector for accelerometer 
 * @note calibration is assumed to be normalized to one, so bias is scaled
 * by 1000mG, the expected value of the earth's gravitational field. 
 * @return SUCCESS or ERROR
 * @author Aaron Hunter,
 **/
int8_t IMU_set_acc_cal(accum A[MSZ][MSZ], accum b[MSZ]);

/**
 * @Function IMU_set_gyro_cal(accum A[MSZ][MSZ], accum b[MSZ])
 * @param A source matrix
 * @param b source offset
 * @brief sets scaling matrix and offset vector for gyroscope 
 * @note Scaling parameters on diagonal (x,y,z) if cross terms unknown. Also,
 * bias values are assumed in raw counts and get scaled to engineering units
 * @return SUCCESS or ERROR
 * @author Aaron Hunter,
 **/
int8_t IMU_set_gyro_cal(accum A[MSZ][MSZ], accum b[MSZ]);

/**
 * @Function IMU_get_mag_cal(accum A[MSZ][MSZ], accum b[MSZ])
 * @param A destination matrix
 * @param b destination offset
 * @brief gets scaling matrix and offset vector for magnetometer 
 * @note bias is scaled to mGauss
 * @return SUCCESS or ERROR
 * @author Aaron Hunter,
 **/
int8_t IMU_get_mag_cal(accum A[MSZ][MSZ], accum b[MSZ]);

/**
 * @Function IMU_get_acc_cal(accum A[MSZ][MSZ], accum b[MSZ])
 * @param A destination matrix
 * @param b destination offset
 * @brief gets scaling matrix and offset vector for accelerometer 
 * @note bias is scaled by 1000mG, the expected value of the earth's 
 * gravitational field. 
 * @return SUCCESS or ERROR
 * @author Aaron Hunter,
 **/
int8_t IMU_get_acc_cal(accum A[MSZ][MSZ], accum b[MSZ]);

/**
 * @Function IMU_get_gyro_cal(accum A[MSZ][MSZ], accum b[MSZ])
 * @param A destination matrix
 * @param b destination offset
 * @brief gets scaling matrix and offset vector for gyros 
 * @note bias is scaled to dps, not counts
 * @return SUCCESS or ERROR
 * @author Aaron Hunter,
 **/
int8_t IMU_get_gyro_cal(accum A[MSZ][MSZ], accum b[MSZ]);




#endif	/* ICM_20948_H */ // End of header guard

