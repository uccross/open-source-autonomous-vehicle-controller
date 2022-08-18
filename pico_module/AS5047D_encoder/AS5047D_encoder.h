/*
 * File: AS5047D_encoder.h
 * Author: Bhuml Depani
 * Brief: This file is a sensor driver module for the AS5047D Magnetic Rotary
 * Encoder. It contains multiple functions which can be used to measure the
 * absolute angle of any rotating motor. There are special functions written in
 * this driver to read angel specially for MG90S servo motor for rotation from 
 * -90 to +90 degree.
 * Created on 07/10/2022 11:07 am
*/
#ifndef AS5047DENCODER_H
#define AS5047DENCODER_H

/*******************************************************************************
 * PUBLIC #INCLUDES                                                            *
 ******************************************************************************/

#include "pico/stdlib.h"                            // Pico Standard Library

/*******************************************************************************
 * PUBLIC #DEFINES                                                            *
 ******************************************************************************/

#define SPI_SPEED 5 * MHZ           // SPI communication baud rate
#define KHZ 1000
#define MHZ 1000000

enum {                              // error code definitions
    SPI_READ_SUCCESS = 1,
    SPI_WRITE_ERROR_GENERIC = -1,
    SPI_WRITE_COMMAND_ERROR = -2,
    SPI_READ_ERROR_GENERIC = -3,
    SPI_READ_ERROR_PARITY = -4,
};

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                           *
 ******************************************************************************/

/*
 * @Function void spi_initialize(int data_speed_hz)
 * @param data_speed_hz, SPI daud rate
 * @return None
 * @brief Initializes SPI communication with required speed and on particular 
 * GPIO pins of Pico
 * @author Bhumil Depani
 */
void spi_initialize(int data_speed_hz);

/*
 * @Function uint16_t get_initial_angle(void)
 * @param None
 * @return an initial angle stored in an internal register of encoder. The 
 * return value will be between 0 and 36000, including 0. If error occurs, then
 * error code will be sent for SPI_WRITE_ERROR_GENERIC, SPI_WRITE_COMMAND_ERROR,
 * SPI_READ_ERROR_GENERIC and SPI_READ_ERROR_PARITY.
 * @brief This function should be called just after powering up controller and 
 * initializing GPIO and SPI communication. An output initial angle will be used
 * to calculate the absolute angle afterwards.
 * @author Bhumil Depani
 */
uint16_t get_initial_angle(void);

/*
 * @Function int16_t get_angle(uint16_t initial_angle)
 * @param initial_angle, an initial angle when microcontroller has powered up.
 * @return a current angle of servo motor in centidegree (-18000 to +18000). If 
 * error occurs, then error code will be sent for SPI_WRITE_ERROR_GENERIC, 
 * SPI_WRITE_COMMAND_ERROR, SPI_READ_ERROR_GENERIC and SPI_READ_ERROR_PARITY.
 * @brief This function should be called to get a current angle of a servo 
 * motor. This current angle will be calculated with current reading from
 * encoder and an initial_angle.
 * @author Bhumil Depani
 */
int16_t get_angle(uint16_t initial_angle);

/*
 * @Function void generate_command_frame(uint16_t register_address, bool 
 * read_write, uint8_t output_command_frame[])
 * @param register_address, internal register address of 16 bit
 * @param read_write, for Write this bit should be 0, for Read this bit should
 * be 1.
 * @param output_command_frame[], output command array to be use while reading 
 * from SPI slave. output_command_frame[0] contains lower bits of address of
 * internal register. 0:5 bits of output_command_frame[1] contains higher bits
 * of address of internal register. 6th bit contains read_write bit and 7th bit
 * contains parity bit.
 * @return None
 * @brief AS5047D requires special command frame with register address, read/
 * write bit and parity bit. This function will design a frame and gives output 
 * in output_command_frame array.
 * @author Bhumil Depani
 */
void generate_command_frame(uint16_t register_address, bool read_write, uint8_t
output_command_frame[]);

/*
 * @Function uint16_t attach_parity_bit(uint16_t data)
 * @param data, input 16 bits data. Out of 16 bits, only lower 15 bits contain 
 * data and MSB bit is reserved for a parity.
 * @return 16 bit output with lower 15 bits same as input data and MSB with 
 * even parity.
 * @brief AS5047D requires special command frame with even parity bit in MSB of 
 * command frame. This function will generate an even parity bit and will attach
 * that bit in MSB.
 * @author Bhumil Depani
 */
uint16_t attach_parity_bit(uint16_t data);

/*
 * @Function bool parity_checker(uint8_t data[])
 * @param data, data array with 2 uint8_t elements. Which includes even parity
 * bit.
 * @return true if parity matches, false if not.
 * @brief to check the even parity bit.
 * @author Bhumil Depani
 */
bool parity_checker(uint8_t data[]);


/*
 * @Function void spi_read(uint16_t register_address, uint8_t read_data[])
 * @param register_address, an internal register address from where to read
 * @param read_data[], output read_data array. read_data[0] with higher 8 bits 
 * of register_address content and read_data[1] with lower 8 bits of 
 * register_address content.
 * @return success or error code, SPI_READ_SUCCESS, SPI_WRITE_ERROR_GENERIC, 
 * SPI_WRITE_COMMAND_ERROR, SPI_READ_ERROR_GENERIC, SPI_READ_ERROR_PARITY 
 * @brief This function will manage the lower level signals for reading data
 * out of SPI slave.
 * @author Bhumil Depani
 */
short spi_read(uint16_t register_address, uint8_t read_data[]);

/*
 * @Function void spi_read_wrapper(uint16_t register_address, uint8_t read_data
 * [])
 * @param register_address, an internal register address from where to read
 * @param read_data[], output read_data array. read_data[0] with higher 8 bits 
 * of register_address content and read_data[1] with lower 8 bits of 
 * register_address content.
 * @return success or error code, SPI_READ_SUCCESS, SPI_WRITE_ERROR_GENERIC, 
 * SPI_WRITE_COMMAND_ERROR, SPI_READ_ERROR_GENERIC, SPI_READ_ERROR_PARITY
 * @brief This function will act as a wrapper function of spi_read() function, 
 * the function will call spi_read() function at the max 3 times, and if still
 * error persist, it will send an error code.
 * @author Bhumil Depani
 */
short spi_read_wrapper(uint16_t register_address, uint8_t read_data[]);

/*
 * @Function uint16_t extract_angle(uint8_t raw_sensor_data_array[])
 * @param raw_sensor_data_array[], raw 16 bit dara read from encoder internal
 * register.
 * @return raw angle converted into centidegree (from 0 to 36000).
 * @brief this function accepts raw data read from encoder internal register,
 * extracts error bit, parity bit and 14 bit raw angle value. Even sends back 
 * raw angle value converted in centidegree (from 0 to 36000).
 * @author Bhumil Depani
 */
uint16_t extract_angle(uint8_t raw_sensor_data_array[]);

/*
 * @Function int16_t angle_correction(uint16_t raw_angle, uint16_t
 * initial_angle)
 * @param raw_angle, current raw data read from internal encoder register in 
 * centidegree (0 to 36000).
 * @param initial_angle, initial angle read just after powering up controller
 * and initializing GPIO and SPI communication.
 * @return final angle in degree (between -18000 to +18000).
 * @brief this function accepts raw angle read from encoder internal register
 * and an initial angle. This function will first compare the raw_angle with
 * initial_angle and then converts the corrected angle in the range of -18000 to
 * +18000.
 * @author Bhumil Depani
 */
int16_t angle_correction(uint16_t raw_angle, uint16_t initial_angle);

/*
 * @Function void send_initial_angle_encoder_error_on_mavlink(uint16_t message)
 * @param message, encoded error code to be sent out on MAVLink
 * @return None
 * @brief this function accepts error code and sent the appropriate message on 
 * the mavlink. The function can only decode error code, sent from 
 * get_initial_angle() function.
 * @author Bhumil Depani
 */
void send_initial_angle_encoder_error_on_mavlink(uint16_t message);

/*
 * @Function void send_angle_encoder_error_on_mavlink(uint16_t message)
 * @param message, encoded error code to be sent out on MAVLink
 * @return None
 * @brief this function accepts error code and sent the appropriate message on 
 * the mavlink. The function can only decode error code, sent from 
 * get_angle() function.
 * @author Bhumil Depani
 */
void send_angle_encoder_error_on_mavlink(int16_t message);


#endif      // AS5047DENCODER_H