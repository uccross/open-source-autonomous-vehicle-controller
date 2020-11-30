/* 
 * File:   ICM_20948.h
 * Author: Aaron Hunter
 * Brief: Library for the ICM-20948 IMU
 * Created on Nov 13, 2020 9:46 am
 * Modified on 
 */

#ifndef ICM_20948_H // Header guard
#define	ICM_20948_H //

/*******************************************************************************
 * PUBLIC #INCLUDES                                                            *
 ******************************************************************************/
#include <sys/types.h>

/*******************************************************************************
 * PUBLIC #DEFINES                                                             *
 ******************************************************************************/
#define IMU_SPI_MODE 0
#define IMU_I2C_MODE 1

/*******************************************************************************
 * PUBLIC TYPEDEFS                                                             *
 ******************************************************************************/
struct IMU_device {
    int16_t x;
    int16_t y;
    int16_t z;
};

struct IMU_output {
    struct IMU_device acc;
    struct IMU_device gyro;
    struct IMU_device mag;
    int16_t temp;
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
 * @author Aaron Hunter,
 * @modified  */
uint8_t IMU_init(char interface_mode);

/**
 * @Function IMU_is_data_ready(void)
 * @return TRUE or FALSE
 * @brief TRUE if unread data is available
 * @note 
 * @author Aaron Hunter,
 * @modified  */
uint8_t IMU_is_data_ready(void);

/**
 * @Function IMU_get_data(void)
 * @return pointer to IMU_output struct 
 * @brief returns most current data from the IMU
 * @note 
 * @author Aaron Hunter,
 * @modified  */
struct IMU_output * IMU_get_data(void);



#endif	/* ICM_20948_H */ // End of header guard

