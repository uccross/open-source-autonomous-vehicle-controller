/*
 * File: AS5047D_encoder.c
 * Author: Bhuml Depani
 * Brief: This file is a sensor driver module for the AS5047D Magnetic Rotary
 * Encoder. It contains multiple functions which can be used to measure the
 * absolute angle of any rotating motor. There are special functions written in
 * this driver to read angel specially for MG90S servo motor for rotation from 
 * -9000 to +9000 centidegree.
 * Created on 07/10/2022 11:00 am
*/

/*******************************************************************************
 * #INCLUDES                                                                   *
 ******************************************************************************/

#include <stdio.h>                      // C standard Input Output Library
#include "hardware/spi.h"               // Pico SPI Library
#include "AS5047D_encoder.h"            // Header file for AS5047D_encoder.c

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/

#define SPI_PORT spi0   /* to use default SPI0 pins on Pico, In default SPI0, 
* GPIO16 will be MISO, GPIO17 will be CSn, GPIO18 will be SCLK and GPIO19 will 
* be MOSI. */
#define MISO_PIN 16     // GPIO pin number where MISO line of SPI is connected
#define CS_PIN 17       // GPIO pin number where CSn line of SPI is connected
#define SCLK_PIN 18     // GPIO pin number where SCLK line of SPI is connected 
#define MOSI_PIN 19     // GPIO pin number where MOSI line of SPI is connected

#define ANGLEUNC_ADDRESS 0x3FFE // Internal AS5047D Encoder register
#define ANGLECOM_ADDRESS 0x3FFF // Internal AS5047D Encoder register
#define DIAAGC_ADDRESS 0x3FFC   // Internal AS5047D Encoder register

#define ENCODER_RESOLUTION 16383    /* For AS5047D encoder, the encoded angle
* value will be available in 14 bits. So, highest value a 14 bits can contain 
* is 2^14 - 1 = 16383. */

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
 * @Function void spi_initialize(int data_speed_hz)
 * @param data_speed_hz, SPI daud rate
 * @return None
 * @brief Initializes SPI communication with required speed and on particular 
 * GPIO pins of Pico
 * @author Bhumil Depani
 */
void spi_initialize(int data_speed_hz)
{
    spi_init(SPI_PORT, data_speed_hz);          // initializes Pico SPI

    gpio_set_function(MISO_PIN, GPIO_FUNC_SPI); 
    gpio_set_function(SCLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MOSI_PIN, GPIO_FUNC_SPI);

    gpio_init(CS_PIN);
    gpio_set_dir(CS_PIN, GPIO_OUT);
    gpio_put(CS_PIN, 1);                // In ideal state CS pin would be high

    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST); /* As
    * per datasheet of AS5047D encoder Clock Polarity should be 0 and Clock 
    * phase should be 1. So, AS5047D encoder works on SPI mode 1. In SPI 
    * communication MSB bit should be sent first. */
}

/*
 * @Function uint16_t get_initial_angle(void)
 * @param None
 * @return an initial angle stored in an internal register of encoder. The 
 * return value will be between 0 and 36000, including 0.
 * @brief This function should be called just after powering up controller and 
 * initializing GPIO and SPI communication. An output initial angle will be used
 * to calculate the absolute angle afterwards.
 * @author Bhumil Depani
 */
uint16_t get_initial_angle(void)
{
    uint8_t read_data[2];
    uint16_t initial_angle;

    spi_read(ANGLEUNC_ADDRESS, read_data);  /* spi_read function will send back 
    * data in read_data array. */

    initial_angle = extract_angle(read_data);
    return initial_angle;
}

/*
 * @Function int16_t get_angle(uint16_t initial_angle)
 * @param initial_angle, an initial angle when microcontroller has powered up.
 * @return a current angle of servo motor in centidegree (-18000 to +18000).
 * @brief This function should be called to get a current angle of a servo 
 * motor. This current angle will be calculated with current reading from
 * encoder and an initial_angle.
 * @author Bhumil Depani
 */
int16_t get_angle(uint16_t initial_angle)
{
    uint8_t read_data[2];
    int16_t angle_degree;

    spi_read(ANGLEUNC_ADDRESS, read_data);  /* spi_read function will send back 
    * data in read_data array. */

    uint16_t raw_angle_degree = extract_angle(read_data);  /* this 
    raw_angle_degree will be raw angle (from 0 to 36000 centidegree) reading, 
    residing in the encoder. */

    angle_degree = angle_correction(raw_angle_degree, initial_angle);   /* to 
    * get absolute angle between -18000 to +18000 centidegree. */
    return angle_degree;
}


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
output_command_frame[])
{
    uint16_t command_frame = register_address | (read_write << 14);
    command_frame = attach_parity_bit(command_frame);
    output_command_frame[0] = (command_frame & 0xFF00) >> 8;
    output_command_frame[1] = (command_frame & 0x00FF);
}

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
uint16_t attach_parity_bit(uint16_t data)
{
    uint16_t data_copy = data;
    unsigned int count = 0;     //to count number of ones in the "data"
    while (data_copy) {
        count += data_copy & 1;
        data_copy >>= 1;
    }
    if(count % 2 != 0)          // if number of ones is odd, make parity bit 1
        data |= 0x8000;
    return data;
}

/*
 * @Function void spi_read(uint16_t register_address, uint8_t read_data[])
 * @param register_address, an internal register address from where to read
 * @param read_data[], output read_data array. read_data[0] with higher 8 bits 
 * of register_address content and read_data[1] with lower 8 bits of 
 * register_address content.
 * @return None
 * @brief This function will manage the lower level signals for reading data
 * out of SPI slave.
 * @author Bhumil Depani
 */
void spi_read(uint16_t register_address, uint8_t read_data[])
{
    uint8_t output_command[2];
    generate_command_frame(register_address, 1, output_command);    /* to 
    * generate appropriate command frame for AS5047D SPI slave. */

    gpio_put(CS_PIN, 0);
    spi_write_blocking(SPI_PORT, output_command, 2);
    gpio_put(CS_PIN, 1);

    sleep_us(1);
    
    gpio_put(CS_PIN, 0);
    spi_read_blocking(SPI_PORT, 0, read_data, 2);
    gpio_put(CS_PIN, 1);
}

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
uint16_t extract_angle(uint8_t raw_sensor_data_array[])
{
    uint16_t raw_sensor_data = raw_sensor_data_array[0] << 8 | 
    raw_sensor_data_array[1];
    
    bool error_bit = (raw_sensor_data_array[0] & 0b01000000) >> 6;
    bool parity_bit = (raw_sensor_data_array[0] & 0b10000000) >> 7;
            
    uint16_t raw_sensor_angle = raw_sensor_data & 0x3FFF;
    uint16_t raw_angle_degree = ((raw_sensor_angle * 360.0) / ENCODER_RESOLUTION) * 100;              //calculating centidegrees

    return raw_angle_degree;
}

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
int16_t angle_correction(uint16_t raw_angle, uint16_t initial_angle)
{
    float raw_angle_float = raw_angle / 100.0;
    float initial_angle_float = initial_angle / 100.0;
    
    float corrected_angle = initial_angle_float - raw_angle_float ; /* Here, we
    * substracted initial_angle_float from raw_angle_float, because in internal
    * register of encoder, angle count got decreased on clockwise rotation.*/
   
    if (corrected_angle < 0)            // to convert angle into 0 to 360
        corrected_angle += 360;
    
    int16_t final_angle = 0;
    
    if(corrected_angle >= 0 && corrected_angle <= 180)
        final_angle = -corrected_angle;
    else if(corrected_angle > 180 && corrected_angle <= 360)
        final_angle = 360 - corrected_angle;
    else
        final_angle = corrected_angle;  /* always corrected_angle should be 
        * between 0 to 360. So, code will never enter in "else" condition. */
    
    final_angle = final_angle * 100;        // to convert into centidegree
    
    return final_angle;
}

#ifdef AS5047DENCODER_TESTING

int main()
{
    stdio_init_all();
    spi_initialize(SPI_SPEED);
    
    float initial_angle;
    initial_angle = get_initial_angle();

    while(1)
    {
        for(int i = 0; i < 10; i++)
        {
            int16_t angle_degree = get_angle(initial_angle);
            printf("\nAngle is %d", angle_degree);

            sleep_ms(1000);
        }
    }
}

#endif      // AS5047DENCODER_TESTING