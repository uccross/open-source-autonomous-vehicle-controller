/*
 * File: range_at_angle.c
 * Author: Bhuml Depani
 * Brief: This is one of the mode of sensor driver module. In this file, there 
 * are functions to find the range (distance) at a particular angle.
 * Created on 07/12/2022 07:12 pm
*/

/*******************************************************************************
 * #INCLUDES                                                                   *
 ******************************************************************************/
#include <stdio.h>                          // C standard Input Output Library

#include "range_at_angle.h"                 // header file for range_at_angle.c
#include "../MG90S_servo/MG90S_servo.h"                    // header file for servo motor MG90S
#include "../V3HP_lidar/V3HP_lidar.h"                     // header file for V3HP LiDAR
#include "../AS5047D_encoder/AS5047D_encoder.h"                // header file for AS5047D encoder

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/

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
 * @Function uint16_t initialize_system_components(void)
 * @param None
 * @return an initial angle of magnet, read by rotary encoder
 * @brief Initializes PWM in Pico for MG90S servo motor, initializes I2C 
 * communication for V3HP LiDAR and initialized SPI communication with required
 * speed and on particular GPIO pins of Pico. 
 * @author Bhumil Depani
 */
uint16_t initialize_system_components(void)
{
    MG90S_servo_init();
    i2c_initialize();

    spi_initialize(SPI_SPEED);
    uint16_t initial_angle;
    initial_angle = get_initial_angle();

    return initial_angle;
}

/*
 * @Function struct range_and_angle get_range_at_angle(int16_t angle, uint16_t
 * initial_angle)
 * @param angle, an angle whre motor will rotate LiDAR to and LiDAR will measure
 * a range
 * @param initial_angle, initial magnet angle, read by rotary encoder
 * @return a structure variable, which contains angle and range information
 * @brief this function rotates servo motor at an angle, given by input, 
 * measures distance using LiDAR, measures angle using encoder and return 
 * distnace and angle
 * @author Bhumil Depani
 */
struct range_and_angle get_range_at_angle(int16_t angle, uint16_t initial_angle)
{
    MG90S_servo_set_angle(angle);
    
    sleep_ms(500);                  // What should be the delay here?
    
    trigger_measurement();
    uint16_t distance = get_distance();
    
    int16_t angle_from_encoder = get_angle(initial_angle);
    
    struct range_and_angle output = {angle_from_encoder, distance};
    return output;
}

#ifdef RANGEATANGLE_TESTING

int main(void)
{
    stdio_init_all();                   // initializes Pico I/O pins

    uint16_t initial_angle = initialize_system_components();

    uint16_t distance;
    int16_t angle;
    struct range_and_angle output;

    while(1)
    {
        output = get_range_at_angle(0, initial_angle);
        printf("\nDistance at %d angle is %10d; Real Angle: 0", output.angle, output.range);
        sleep_ms(5000);

        output = get_range_at_angle(9000, initial_angle);
        printf("\nDistance at %d angle is %10d; Real Angle: 9000", output.angle, output.range);
        sleep_ms(5000);

        output = get_range_at_angle(0, initial_angle);
        printf("\nDistance at %d angle is %10d; Real Angle: 0", output.angle, output.range);
        sleep_ms(5000);

        output = get_range_at_angle(-9000, initial_angle);
        printf("\nDistance at %d angle is %10d; Real Angle: -9000", output.angle, output.range);
        sleep_ms(5000);
    }
}

#endif      // RANGEATANGLE_TESTING