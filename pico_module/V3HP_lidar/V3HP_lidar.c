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
#define SDA_PIN 4       // GPIO pin number where SDA line of I2C is connected
#define SCL_PIN 5       // GPIO pin number where SCL line of I2C is connected

#define LIDAR_ADDRESS 0x62              // LiDAR I2C address from datasheet
#define ACQ_COMMAND 0x00                // Internal LiDAR register
#define STATUS 0x01                     // Internal LiDAR register
#define FULL_DELAY 0x0f                 // Internal LiDAR register
#define SIG_COUNT_VAL 0x02              // Internal LiDAR register
#define ACQ_CONFIG_REG 0x04             // Internal LiDAR register
#define THRESHOLD_BYPASS 0x1c           // Internal LiDAR register

#define WRITE_TIMEOUT 300               /* Write timeout of 300 microseconds.
* As i2c_write function is taking around 90 to 100 microseconds time for
* writing into the specific internal register, taking 3 times more then it for 
* timeout. */
#define READ_TIMEOUT 300                /* Read timeout of 300 microseconds. As
* i2c_read function is taking around 70 microseconds for reading 1 byte and 90 
* microseconds for reading 2 bytes. So defining a timeout more then 3 times. */

/*******************************************************************************
 * PRIVATE VARIABLES                                                            
 ******************************************************************************/

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
    i2c_init(I2C_PORT, I2C_SPEED);      // initializes Pico I2C

    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(SDA_PIN);         // To enable the GPIO pull up function
    gpio_pull_up(SCL_PIN);
}

/*
 * @Function short trigger_measurement(void)
 * @param None
 * @return a constant indicating the success or error.Output could be one of
 * the following: TRIGGER_SUCCESS, LIDAR_WRITE_ERROR_TIMEOUT,
 * LIDAR_WRITE_ERROR_GENERIC
 * @brief to configure required LiDAR internal registers to initiate measurement
 * @author Bhumil Depani
 */

short trigger_measurement(void)
{
    uint8_t data = 0x04;        /* need to write 0x04 to ACQ_COMMAND to 
    * initiate distance measurement */
    
    short write_output = lidar_write_wrapper(ACQ_COMMAND, &data, 1, false);

    if(write_output != LIDAR_WRITE_SUCCESS)
        return write_output;
    
    short busy_flag;
    do
    {
        busy_flag = get_busy_flag();
        
        if(busy_flag == LIDAR_READ_ERROR_GENERIC | busy_flag == 
        LIDAR_READ_ERROR_TIMEOUT)
        {
            return busy_flag;       // escalate error to caller function. 
        }
    } while (busy_flag != 0x00);    /* come out of the loop if busy_flag is not
    * 0x01; only LSB of busy_flag is useful. if that LSB is 0, then 
    * LiDAR is ideal and can be used for distance measurement, otherwise LiDAR
    * is busy */
    return TRIGGER_SUCCESS;
}

/*
 * @Function int32_t get_distance(void)
 * @param None, 
 * @return distance in centimeters or error code
 * @brief reads the distnace from the internal registers of the LiDAR
 * @author Bhumil Depani
 */
int32_t get_distance(void)
{
    uint8_t raw_distance[2];
    uint8_t register_address = FULL_DELAY;  /* measured distance will be stored 
    * in the FULL_DELAY_HIGH and FULL_DELAY_LOW register of the LiDAR */
    
    short read_output = lidar_read_wrapper(register_address, raw_distance, 2); 
    /* read distance in raw_distance array */

    int32_t distance;
    
    if(read_output == LIDAR_READ_SUCCESS)
    {
        distance = (raw_distance[0] << 8) | raw_distance[1];   
        /* rotate higher byte to 8 step left and OR (combine) it with lower 
        byte */
    }
    else
    {
        distance = read_output;         // sending error code
    }
    return distance;
}

/*
 * @Function short get_busy_flag(void)
 * @param None,
 * @return a byte with the LSB as busy_flag or return an error code
 * @brief this function reads STATUS register and extracts it's LSB
 * @author Bhumil Depani
 */
short get_busy_flag(void)
{
    uint8_t register_address = STATUS;
    uint8_t status_read;
    
    short read_output = lidar_read_wrapper(register_address, &status_read, 1);

    short busy_flag;
    if(read_output == LIDAR_READ_SUCCESS)
    {
        busy_flag = status_read & 0x01; // extracting LSB of status_flag
    }
    else
    {
        busy_flag = read_output;        // sending error code
    }
    return busy_flag;
}

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
no_of_byte_to_write, bool nostop)
{
    uint8_t no_of_byte_for_i2c = no_of_byte_to_write + 1;
    
    uint8_t protocol_data[no_of_byte_for_i2c];
    protocol_data[0] = register_address;     /* as per I2C protocol, register
    * address should be the first byte after device address. this register  
    * address will be followed by data byte(s). */

    for(int i = 0; i < no_of_byte_to_write; i++)
        protocol_data[i + 1] = write_data[i];

    uint64_t base = time_us_64();
    base += WRITE_TIMEOUT;

    absolute_time_t absolute_delay;
    update_us_since_boot(&absolute_delay, base);

    short bytes = i2c_write_blocking_until(I2C_PORT, LIDAR_ADDRESS, 
    protocol_data, no_of_byte_for_i2c, nostop, absolute_delay);
    
    
    return bytes;
}

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
uint8_t no_of_byte_to_write, bool nostop)
{
    int bytes = lidar_write(register_address, write_data, no_of_byte_to_write, 
    nostop);

    if(bytes == (no_of_byte_to_write + 1))
    {
        return LIDAR_WRITE_SUCCESS;
    }
    else                    // if else executes then error has been occurred
    {
        for(short i = 0; i < 2; i++)        // try again for 2 times
        {
            bytes = lidar_write(register_address, write_data, 
            no_of_byte_to_write, nostop);
            if(bytes == (no_of_byte_to_write + 1))
                return LIDAR_WRITE_SUCCESS;
        }
        if(bytes == PICO_ERROR_TIMEOUT)         // sends appropriate error code
            return LIDAR_WRITE_ERROR_TIMEOUT;
        else
            return LIDAR_WRITE_ERROR_GENERIC;
    }
    
}

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
no_of_byte_to_read)
{
    /* as per I2C protocol, needs to write register address first 
    * before reading data bytes from the register */
    
    uint64_t base = time_us_64();
    base += WRITE_TIMEOUT;

    absolute_time_t absolute_delay;
    update_us_since_boot(&absolute_delay, base);

    short write_output = i2c_write_blocking_until(I2C_PORT, LIDAR_ADDRESS, &
    register_address, 1, true, absolute_delay);
    /* If second last argument is true, master retains
    * control of the bus at the end of the transfer (no Stop is issued), and
    * the next transfer will begin with a Restart rather than a Start. In this 
    * case, we want to read required byte(s) after writing register_address. 
    * So, to retail control of SDA, using "true"*/

    // as we are just writing one byte, write_output should be 1.
    if(write_output == 1)
    {
        uint64_t base = time_us_64();
        base += READ_TIMEOUT;

        absolute_time_t absolute_delay;
        update_us_since_boot(&absolute_delay, base);

        short bytes = i2c_read_blocking_until(I2C_PORT, LIDAR_ADDRESS, 
        read_data, no_of_byte_to_read, false, absolute_delay);

        if(bytes == no_of_byte_to_read)
        {
            return LIDAR_READ_SUCCESS;
        }
        else                        // sending an error code
        {
            if(bytes == PICO_ERROR_TIMEOUT)
                return LIDAR_READ_ERROR_TIMEOUT;
            else
                return LIDAR_READ_ERROR_GENERIC;
        }
    }
    else                            // sending error code
    {
        if(write_output == PICO_ERROR_TIMEOUT)
            return LIDAR_WRITE_ERROR_TIMEOUT;
        else
            return LIDAR_WRITE_ERROR_GENERIC;
    }
}

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
no_of_byte_to_read)
{
    int read_output = lidar_read(register_address, read_data,
    no_of_byte_to_read);

    if(read_output == LIDAR_READ_SUCCESS)
    {
        return LIDAR_READ_SUCCESS;
    }
    else                        // if else executes then error has been occurred
    {
        for(short i = 0; i < 2; i++)        // try again for 2 times
        {
            read_output = lidar_read(register_address, read_data,
            no_of_byte_to_read);
            if(read_output == LIDAR_READ_SUCCESS)
            {
                return LIDAR_READ_SUCCESS;
            }
        }
        return read_output;         // sends appropriate error code
    }
}

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
uint8_t sensitivity)
{
    short output = lidar_write_wrapper(SIG_COUNT_VAL, &max_acqisition, 1, 
    false);
    if(output != LIDAR_WRITE_SUCCESS)
    {
        return output;
    }
    
    if(quick_termination)
    {
        uint8_t acquisition_mode_control;
        output = lidar_read_wrapper(ACQ_CONFIG_REG, &acquisition_mode_control, 
        1);
        if(output == LIDAR_READ_SUCCESS)
        {
            acquisition_mode_control = acquisition_mode_control & 0b11110111;
            output = lidar_write_wrapper(ACQ_CONFIG_REG, &
            acquisition_mode_control, 1, false);
            if(output != LIDAR_WRITE_SUCCESS)
            {
                return output;
            }
        }
        else
        {
            return output;
        }
    }

    output = lidar_write_wrapper(THRESHOLD_BYPASS, &sensitivity, 1, false);
    if(output != LIDAR_WRITE_SUCCESS)
    {
        return output;
    }
}

/*
 * @Function void send_lidar_error_on_mavlink(short message)
 * @param message, an error code
 * @return None
 * @brief a function just takes an error code and wrote the appropriate error 
 * description on the serial port.
 * @author Bhumil Depani
 */
void send_lidar_error_on_mavlink(short message)
{
    if(message == LIDAR_WRITE_ERROR_TIMEOUT)
    {
        printf("\nLiDAR I2C Write Timeout.");
    }
    else if(message == LIDAR_WRITE_ERROR_GENERIC)
    {
        printf("\nLiDAR I2C Write Generic Error.");
    }
    else if(message == LIDAR_READ_ERROR_TIMEOUT)
    {
        printf("\nLiDAR I2C Read Timeout.");
    }
    else if(message == LIDAR_READ_ERROR_GENERIC)
    {
        printf("\nLiDAR I2C Read Generic Error.");
    }
    else
    {
        printf("\nUnknown Error from LiDAR driver module.");
    }
}

#ifdef V3HPLIDAR_TESTING

int main()
{
    stdio_init_all();                   // initializes Pico I/O pins

    i2c_initialize();
    short trigger_output;
    uint32_t start, stop, middle;
    while (true)
    {
        start = time_us_32();
        trigger_output = trigger_measurement();
        middle = time_us_32();
        if(trigger_output != TRIGGER_SUCCESS)
        {
            send_lidar_error_on_mavlink(trigger_output);
        }
        
        int32_t distance = get_distance();
        stop = time_us_32();
        if(distance >= 0)
        {

            printf("\nCurrent distace: %5d cm", distance);
            printf("    trigger: %10lu, read: %10lu, all: %10lu", (middle - start), (stop - middle), (stop - start));
        }
        if(distance < 0)
        {
            send_lidar_error_on_mavlink(distance);
        }
        sleep_ms(1000);
    }
}

#endif      // V3HPLIDAR_TESTING