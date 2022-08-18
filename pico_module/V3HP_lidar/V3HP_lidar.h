/*
 * File: V3HP_lidar.c
 * Author: Bhuml Depani
 * Brief: This file is a sensor driver module for the V3HP LiDAR module. It
 * contains multiple functions which can be used to measure the distance in 
 * different modes.
 * Created on 06/27/2022 04:08 pm
*/

#ifndef V3HPLIDAR_H
#define V3HPLIDAR_H

/*******************************************************************************
 * PUBLIC #INCLUDES                                                            *
 ******************************************************************************/

#include "pico/stdlib.h"                            // Pico Standard Library

/*******************************************************************************
 * PUBLIC #DEFINES                                                            *
 ******************************************************************************/

#define KHZ 1000

enum {                              // defining error codes
    LIDAR_WRITE_SUCCESS = 0,
    LIDAR_WRITE_ERROR_TIMEOUT = -1,
    LIDAR_WRITE_ERROR_GENERIC = -2,
    LIDAR_READ_SUCCESS = 1,
    LIDAR_READ_ERROR_TIMEOUT = -3,
    LIDAR_READ_ERROR_GENERIC = -4,
};

enum {
    TRIGGER_SUCCESS = 0,
};

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                           *
 ******************************************************************************/

/*
 * @Function void i2c_initialization(void)
 * @param None
 * @return None
 * @brief Initializes I2C communication with required speed and on particular 
 * GPIO pins of Pico
 * @author Bhumil Depani
 */
void i2c_initialize(void);

/*
 * @Function short trigger_measurement(void)
 * @param None
 * @return a constant indicating the success or error.Output could be one of
 * the following: TRIGGER_SUCCESS, LIDAR_WRITE_ERROR_TIMEOUT,
 * LIDAR_WRITE_ERROR_GENERIC
 * @brief to configure required LiDAR internal registers to initiate measurement
 * @author Bhumil Depani
 */
short trigger_measurement(void);

/*
 * @Function int32_t get_distance(void)
 * @param None, 
 * @return distance in centimeters or error code
 * @brief reads the distnace from the internal registers of the LiDAR
 * @author Bhumil Depani
 */
int32_t get_distance(void);

/*
 * @Function short get_busy_flag(void)
 * @param None,
 * @return a byte with the LSB as busy_flag or return an error code
 * @brief this function reads STATUS register and extracts it's LSB
 * @author Bhumil Depani
 */
short get_busy_flag(void);

/*
 * @Function short lidar_write(uint8_t register_address, uint8_t write_data[],
 * uint8_t no_of_byte_to_write, bool nostop)
 * @param register_address, internal register of LiDAR
 * @param write_data[], in case of a single byte data write, this parameter 
 * could be a reference to the variable or an array of a single element. In case
 * of multiple data write, this parameter should be an array only.
 * @param no_of_byte_to_write, 
 * @param nostop, if true, master retains control of the bus at the end of the
 *  transfer (no Stop is issued), and the next transfer will begin with a 
 * Restart rather than a Start. If false, master will let go the control of
 * SDA. 
 * @return no of byte wrote successfully, or PICO_ERROR_TIMEOUT or 
 * PICO_ERROR_GENERIC
 * @brief low level function to write a single or multiple bytes to a paticular
 * internal register address(es)
 * @author Bhumil Depani
 */
short lidar_write(uint8_t register_address, uint8_t write_data[], uint8_t 
no_of_byte_to_write, bool nostop);

/*
 * @Function short lidar_write_wrapper(uint8_t register_address, uint8_t 
 * write_data[], uint8_t no_of_byte_to_write, bool nostop)
 * @param register_address, internal register of LiDAR
 * @param write_data[], in case of a single byte data write, this parameter 
 * could be a reference to the variable or an array of a single element. In case
 * of multiple data write, this parameter should be an array only.
 * @param no_of_byte_to_write, 
 * @param nostop, if true, master retains control of the bus at the end of the
 * transfer (no Stop is issued), and the next transfer will begin with a 
 * Restart rather than a Start. If false, master will let go the control of
 * SDA. 
 * @return LIDAR_WRITE_SUCCESS, LIDAR_WRITE_ERROR_TIMEOUT, 
 * LIDAR_WRITE_ERROR_GENERIC
 * @brief this is a wrapper function to lidar_write() function. It maximum calls
 * lidar_write() function 3 times, and if still error persists then sends the 
 * error code to the caller function.
 * @author Bhumil Depani
*/
short lidar_write_wrapper(uint8_t register_address, uint8_t write_data[], 
uint8_t no_of_byte_to_write, bool nostop);

/*
 * @Function short lidar_read(uint8_t register_address, uint8_t read_data[],
 * uint8_t no_of_byte_to_read)
 * @param register_address, internal register of LiDAR
 * @param read_data[], in case of a single byte data read, this parameter 
 * could be a reference to the variable or an array of a single element. In case
 * of multiple data read, this parameter should be an array only.
 * @param no_of_byte_to_read, 
 * @return LIDAR_READ_SUCCESS, LIDAR_READ_ERROR_TIMEOUT,
 * LIDAR_READ_ERROR_GENERIC, LIDAR_WRITE_ERROR_TIMEOUT, 
 * LIDAR_WRITE_ERROR_GENERIC
 * @brief low level function to read a single or multiple bytes from a paticular
 * internal register address(es)
 * @author Bhumil Depani
 */
short lidar_read(uint8_t register_address, uint8_t read_data[], uint8_t 
no_of_byte_to_read);

/*
 * @Function short lidar_read_wrapper(uint8_t register_address, uint8_t
 * read_data[], uint8_t no_of_byte_to_read)
 * @param register_address, internal register of LiDAR
 * @param read_data[], in case of a single byte data read, this parameter 
 * could be a reference to the variable or an array of a single element. In case
 * of multiple data read, this parameter should be an array only.
 * @param no_of_byte_to_read, 
 * @return LIDAR_READ_SUCCESS, LIDAR_READ_ERROR_TIMEOUT, 
 * LIDAR_READ_ERROR_GENERIC, LIDAR_WRITE_ERROR_TIMEOUT, 
 * LIDAR_WRITE_ERROR_GENERIC
 * @brief a wrapper function to lidar_read() function. It maximum calls
 * lidar_read() function 3 times, and if still error persists then sends the 
 * error code to the caller function.
 * @author Bhumil Depani
*/
short lidar_read_wrapper(uint8_t register_address, uint8_t read_data[], uint8_t 
no_of_byte_to_read);

/*
 * @Function short lidar_advanced_settings(uint8_t max_acqisition, bool
 * quick_termination, uint8_t sensitivity)
 * @param max_acqisition, its value can range from 0 to 255, the default value 
 * is 255, max_acqisition presents the number of times the LiDAR will integrate 
 * acquisitions to measure distance for one time, The unit-less relationship is 
 * roughly as follows: rate = 1/max_acquisition and range = max_acquisition^(1/
 * 4)  
 * @param quick_termination, its value could be 0 or 1, the default value is 0,
 * if input is true, device will terminate a distance measurement early, this 
 * allows for faster and slightly less accurate operation without sacrificing 
 * long-range performance.
 * @param sensitivity, its value can range from 0 to 255, the default value is 
 * 0, recommended values are 0x20 for higher sensitivity with more frequent
 * erroneous measurements, and 0x60 for reduced sensitivity and fewer erroneous 
 * measurements.
 * @return error codes, LIDAR_WRITE_ERROR_TIMEOUT, LIDAR_WRITE_ERROR_GENERIC,
 * LIDAR_READ_ERROR_TIMEOUT, LIDAR_READ_ERROR_GENERIC
 * @brief a function to set advanced setting in LiDAR to change speed, range, 
 * and sensitivity of the measurement of distance.
 * @author Bhumil Depani
 */
short lidar_advanced_settings(uint8_t max_acqisition, bool quick_termination, 
uint8_t sensitivity);

/*
 * @Function void send_lidar_error_on_mavlink(short message)
 * @param message, an error code
 * @return None
 * @brief a function just takes an error code and wrote the appropriate error 
 * description on the serial port.
 * @author Bhumil Depani
 */
void send_lidar_error_on_mavlink(short message);

#endif  //V3HPLIDAR_H