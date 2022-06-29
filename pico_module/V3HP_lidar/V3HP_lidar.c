/*
 * File: V3HP_lidar.c
 * Author: Bhuml Depani
 * Brief: This file is a sensor driver module for the V3HP LiDAR module. It
 * contains multiple functions which can be used to measure the distance in 
 * different modes.
 * Created on 06/27/2022 04:08 pm
*/

/*******************************************************************************
 * #INCLUDES                                                                   *
 ******************************************************************************/

#include <stdio.h>                      // C standard Input Output Library
#include "V3HP_lidar.h"                 // Header file for V3HP_lidar.c
#include "hardware/i2c.h"               // Pico I2C Library

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/

#define I2C_PORT i2c0                   /* to use default I2C0 pins on Pico, 
* In default I2C0, GPIO4 will be SDA and GPIO5 will be SCL */
#define I2C_SPEED 400 * KHZ             // data transfer speed for fast I2C
#define ACQ_COMMAND 0x00                // Internal LiDAR register
#define STATUS 0x01                     // Internal LiDAR register
#define FULL_DELAY 0x0f                 // Internal LiDAR register

/*******************************************************************************
 * PRIVATE VARIABLES                                                            
 ******************************************************************************/

static int lidar_address = 0x62;            // LiDAR I2C address from datasheet
static int sda_gpio_pin = 4;                // default I2C0 SDA pin
static int scl_gpio_pin = 5;                // default I2C0 SCL pin

/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                 
 ******************************************************************************/

/*******************************************************************************
 * PUBLIC FUNCTION IMPLEMENTATIONS                                             *
 ******************************************************************************/

/*
 * @Function void i2c_initialization(void)
 * @param None
 * @return None
 * @brief Initializes I2C communication with required speed and on particular 
 * GPIO pins of Pico
 * @author Bhumil Depani
 */
void i2c_initialize(void)
{
    stdio_init_all();                   // initializes Pico I/O pins
    
    i2c_init(I2C_PORT, I2C_SPEED);      // initializes Pico I2C

    gpio_set_function(sda_gpio_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_gpio_pin, GPIO_FUNC_I2C);

    gpio_pull_up(sda_gpio_pin);         // To enable the GPIO pull up function
    gpio_pull_up(scl_gpio_pin);
}

/*
 * @Function void trigger_measurement(void)
 * @param None
 * @return None
 * @brief to configure required LiDAR internal registers to initiate measurement
 * @author Bhumil Depani
 */
void trigger_measurement(void)
{
    uint8_t data = 0x04;        /* need to write 0x04 to ACQ_COMMAND to 
    * initiate distance measurement */
    lidar_write(ACQ_COMMAND, &data, 1);

    uint8_t busy_flag;
    do
    {
        busy_flag = get_busy_flag();
    } while (busy_flag == 0x01);    /* come out of the loop if busy_flag is not
    * 0x01; only LSB of busy_flag is useful. if that LSB is 0, then 
    * LiDAR is ideal and can be used for distance measurement, otherwise LiDAR
    * is busy */
}

/*
 * @Function uint16_t get_distance(void)
 * @param None, 
 * @return measured distnace in centimeter
 * @brief low level function to read a single or multiple bytes from a paticular
 * internal register address(es)
 * @author Bhumil Depani
 */
uint16_t get_distance(void)
{
    uint8_t raw_distance[2];
    uint8_t register_address = FULL_DELAY;  /* measured distance will be stored 
    * in the FULL_DELAY_HIGH and FULL_DELAY_LOW register of the LiDAR */
    
    lidar_read(register_address, raw_distance, 2); /* read distance in 
    * raw_distance array */
    uint16_t distance = (raw_distance[0] << 8) | raw_distance[1];   /* rotate 
    * higher byte to 8 step left and OR (combine) it with lower byte */
    
    return distance;
}

/*
 * @Function uint8_t get_busy_flag(void)
 * @param None,
 * @return a byte with the LSB as busy_flag
 * @brief this function reads STATUS register and isolate it's LSB
 * @author Bhumil Depani
 */
uint8_t get_busy_flag(void)
{
    uint8_t register_address = STATUS;
    
    uint8_t status_read;
    lidar_read(register_address, &status_read, 1);

    uint8_t busy_flag = status_read & 0x01;     // extracting LSB of status_flag
    return busy_flag;
}

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
no_of_byte_to_write)
{
    uint8_t no_of_byte_for_i2c = no_of_byte_to_write + 1;
    
    uint8_t protocol_data[no_of_byte_for_i2c];
    protocol_data[0] = register_address;     /* as per I2C protocol, register
    * address should be the first byte after device address. this register  
    * address will be followed by data byte(s). */

    for(int i = 0; i < no_of_byte_to_write; i++)
        protocol_data[i + 1] = write_data[i];

    i2c_write_blocking(I2C_PORT, lidar_address, protocol_data, 
    no_of_byte_for_i2c, false);     /* If last argument is true, master retains
    * control of the bus at the end of the transfer (no Stop is issued), and
    * the next transfer will begin with a Restart rather than a Start. If 
    * false, master will let go the control of SDA. In this case, we just want
    * to write required byte(s) and let go the control. */
}

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
no_of_byte_to_read)
{
    /* as per I2C protocol, needs to write register address first 
    * before reading data bytes from the register */
    i2c_write_blocking(I2C_PORT, lidar_address, &register_address, 1, true);
    /* If last argument is true, master retains control of the bus at the end
    * of the transfer (no Stop is issued), and the next transfer will begin 
    * with a Restart rather than a Start. If false, master will let go the 
    * control of SDA. In this case, we want to read required byte(s) after 
    * writing register_address. So, to retail control of SDA, using "true" */
    
    i2c_read_blocking(I2C_PORT, lidar_address, read_data, no_of_byte_to_read,
    false);
}

#ifdef V3HPLIDAR_TESTING

int main()
{
    sleep_ms(5000);

    i2c_initialize();

    while (true)
    {
        trigger_measurement();
        uint16_t distance = get_distance();
        printf("\nCurrent distace: %5d cm", distance);
        sleep_ms(1000);
    }
}

#endif      // V3HPLIDAR_TESTING