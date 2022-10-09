/*
 * File: range_at_angle.c
 * Author: Bhuml Depani
 * Brief: This is one of the modes of sensor driver module. In this file, there 
 * are functions to find the range (distance) at a particular angle.
 * Created on 07/12/2022 07:12 pm
*/

/*******************************************************************************
 * #INCLUDES                                                                   *
 ******************************************************************************/
#include <stdio.h>                          // C standard Input Output Library

#include "range_at_angle.h"                 // header file for range_at_angle.c
#include "../MG90S_servo/MG90S_servo.h"     // header file for servo motor MG90S
#include "../V3HP_lidar/V3HP_lidar.h"       // header file for V3HP LiDAR
#include "../AS5047D_encoder/AS5047D_encoder.h" /* header file for AS5047D 
* encoder */
#include "hardware/watchdog.h"

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/

#define LIDAR_PERIOD 2000000  // read LiDAR at every 20 ms
#define ENCODER_PERIOD 100000 // read encoder at every 1 ms
#define WATCHDOG_COUNT  8000    /* Watchdog reset count in milliseconds, maximum
* value could be 8388 milliseconds. */

/*******************************************************************************
 * PRIVATE VARIABLES                                                            
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                 
 ******************************************************************************/

/*
 * @Function void watchdog_disable(void)
 * @param None
 * @return None
 * @brief As Pico C/C++ SDK is not giving any function to disable the Watchdog
 * timer, we are implementing it by accessing interal register of Pico.
 * @author Bhumil Depani
 */
void watchdog_disable(void)
{
    hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
}

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
 * @Function angle_and_range get_angle_and_range(uint16_t initial_angle)
 * @param initial_angle, an initial angle measured by encoder, while booting up
 * system
 * @return a structure angle_and_range
 * @brief measures a distnace using LiDAR, measures an angle using an encoder 
 * at current position of the servo motor, and combine both measurements in the
 * angle_and_range structure
 * @author Bhumil Depani
 */
angle_and_range get_angle_and_range(uint16_t initial_angle)
{
    trigger_measurement();
    uint16_t distance = get_distance();
    
    int16_t angle_from_encoder = get_angle(initial_angle);
    
    angle_and_range output = {angle_from_encoder, distance};
    return output;
}

/*
 * @Function angle_and_range range_at_angle_mode(int16_t angle, uint16_t       * initial_angle)
 * @param angle, the angle at which want to measure a distance
 * @param initial_angle, an initial angle measured by encoder, while booting up
 * system
 * @return Final range and angle
 * @brief this function implements one of the mode of a sensor driver module. A 
 * function will first send command to motor, 
 * @author Bhumil Depani
 */
angle_and_range range_at_angle_mode(int16_t angle, uint16_t initial_angle)
{
    MG90S_servo_set_angle(angle);
    
    int16_t lower_limit = angle - TOLERANCE;    // define an acceptable margine 
    int16_t upper_limit = angle + TOLERANCE;
    
    uint32_t lidar_start_time = 0, encoder_start_time = 0;
    uint32_t lidar_current_time, encoder_current_time;

    watchdog_enable(WATCHDOG_COUNT, 0);     /* to enable Watchdog timer for range at angle mode. */
    
    int16_t angle_from_encoder;
    angle_and_range output;
    
    /* this won't be infinite loop. We are using break statement in on of the if
    * condition. So, whenever motor angle would be within acceptable range of 
    * angle, this break statement will be executed. */
    while(1)
    {
        lidar_current_time = encoder_current_time = time_us_32();
        /* this if condition will only be true after a duration mentioned in
        * LIDAR_PERIOD. */
        if((lidar_current_time - lidar_start_time) >= LIDAR_PERIOD)
        {
            output = get_angle_and_range(initial_angle);
            if(output.angle <= lower_limit || output.angle >= upper_limit)
            {
                printf("\nAngle: %10d, Distnace: %10d", output.angle, 
                output.range);            
            }
            angle_from_encoder = output.angle;
            lidar_start_time = lidar_current_time;
        }
        /* this if condition will only be true after a duration mentioned in
        * ENCODER_PERIOD. */
        if((encoder_current_time - encoder_start_time) >= ENCODER_PERIOD)
        {
            angle_from_encoder = get_angle(initial_angle);
            encoder_start_time = encoder_current_time;
        }
        /* this if condition will only be true after a motor will be within
        * defined margine of angle. This margine will be define by TOLERANCE. */
        if((angle_from_encoder > lower_limit) && (angle_from_encoder < 
        upper_limit))
        {
            watchdog_disable();     /* As we are done with the work for 
            * commanded angle, now disable the Watchdog timer. */

            printf("\n%10d,  %10d,  %10d",angle_from_encoder, lower_limit, upper_limit);
            output = get_angle_and_range(initial_angle);
            printf("\nFinal Angle: %10d, Distnace: %10d", output.angle, output.
            range);
            break;  // this break statement will be used to come out of while(1)
        }
    }
    return output;
}

#ifdef RANGEATANGLE_TESTING

int main(void)
{
    stdio_init_all();                   // initializes Pico I/O pins

    uint16_t initial_angle = initialize_system_components();

    lidar_advanced_settings(0xFF, 0, 0x00);     /*defualt LiDAR settings, can 
    * change according to requirements, see the comments before function 
    * implementation. */

    int16_t input_angle;
    angle_and_range output;

    for(int i = 0; i< 10; i++)
    {
        input_angle = i*1000;
        printf("\n\n\nAn angle where want a distance: %d", input_angle);
        
        range_at_angle_mode(input_angle, initial_angle);

        for(int j=0; j<2; j++)
        {
            printf("\nIdeal time");
            sleep_ms(2000);
        }
    }
}

#endif      // RANGEATANGLE_TESTING