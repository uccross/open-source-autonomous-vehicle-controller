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
 * @Function void trigger_measurement(void)
 * @param None
 * @return None
 * @brief to configure required LiDAR internal registers to initiate measurement
 * @author Bhumil Depani
 */
void trigger_measurement(void);

/*
 * @Function uint16_t get_distance(void)
 * @param None, 
 * @return measured distnace in centimeter
 * @brief low level function to read a single or multiple bytes from a paticular
 * internal register address(es)
 * @author Bhumil Depani
 */
uint16_t get_distance(void);

/*
 * @Function uint8_t get_busy_flag(void)
 * @param None,
 * @return a byte with the LSB as busy_flag
 * @brief this function reads STATUS register and isolate it's LSB
 * @author Bhumil Depani
 */
uint8_t get_busy_flag(void);

/*
 * @Function void lidar_write(uint8_t register_address, uint8_t write_data[],
 * uint8_t no_of_byte_to_write)
 * @param register_address, internal register of LiDAR
 * @param write_data[], in case of a single byte data write, this parameter 
 * could be a reference to the variable or an array of a single element. In case
 * of multiple data write, this parameter should be an array only.
 * @param no_of_byte_to_write, 
 * @return None
 * @brief low level function to write a single or multiple bytes to a paticular
 * internal register address(es)
 * @author Bhumil Depani
 */
void lidar_write(uint8_t register_address, uint8_t write_data[], uint8_t 
no_of_byte_to_write);

/*
 * @Function void lidar_read(uint8_t register_address, uint8_t read_data[],
 * uint8_t no_of_byte_to_read)
 * @param register_address, internal register of LiDAR
 * @param read_data[], in case of a single byte data read, this parameter 
 * could be a reference to the variable or an array of a single element. In case
 * of multiple data read, this parameter should be an array only.
 * @param no_of_byte_to_read, 
 * @return None
 * @brief low level function to read a single or multiple bytes from a paticular
 * internal register address(es)
 * @author Bhumil Depani
 */
void lidar_read(uint8_t register_address, uint8_t read_data[], uint8_t 
no_of_byte_to_read);

#endif  //V3HPLIDAR_H