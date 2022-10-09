/*
 * File: constant_panning_mode.c
 * Author: Bhuml Depani
 * Brief: This is one of the modes of sensor driver module. In this file, there 
 * are functions to scan across the field of view.
 * Created on 08/09/2022 10:03 am
*/

/*******************************************************************************
 * #INCLUDES                                                                   *
 ******************************************************************************/
#include <stdio.h>                          // C standard Input Output Library

#include "constant_panning_mode.h"           // header file for constant_panning_mode.c
#include "../MG90S_servo/MG90S_servo.h"     // header file for servo motor MG90S
#include "../V3HP_lidar/V3HP_lidar.h"       // header file for V3HP LiDAR
#include "../AS5047D_encoder/AS5047D_encoder.h" /* header file for AS5047D 
* encoder */
#include "hardware/watchdog.h"
#include "../range_at_angle/range_at_angle.h"   /* header file for range at angle mode. */

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/



/*******************************************************************************
 * PRIVATE VARIABLES                                                            
 ******************************************************************************/

uint16_t initial_angle = 0;     // initial angle for encoder measurement
int16_t stopping_angle[200];    /* Array of angles, to stop and measure the 
* distance. */
uint8_t no_of_stop = 0;         // Total number of stoppages
uint8_t next_stop = 0;          // index of the stopping_angle array to stop
bool direction = 1;     // 1 --> -90 to 90; 0 --> 90 to -90

/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                 
 ******************************************************************************/

/*******************************************************************************
 * PUBLIC FUNCTION IMPLEMENTATIONS                                             *
 ******************************************************************************/

/*
 * @Function int16_t start_constant_panning_mode(uint16_t angle)
 * @param angle, an initial angle used by the encoder
 * @return offset angle value
 * @brief this is a function to initiate the constant panning mode; this 
 * function should be called when we first start constant panning mode
 * @author Bhumil Depani
 */
int16_t start_constant_panning_mode(uint16_t angle)
{
    angle_and_range output = {0, 0xFFFF};       // dummy data
    initial_angle = angle;
    direction = 1;          // initial direction will be from -90 to 90 degree
    no_of_stop = 0;         // initialize no_of_stop

    // creating an array with the angle for stop, in centidegree
    for(int16_t i = -9000; i <= 9000; i += ANGLE_INCREMENT)
    {
        stopping_angle[no_of_stop] = i;
        no_of_stop++;
    }
    int16_t current_angle = get_angle(initial_angle);

    for(uint8_t i = 0; i < no_of_stop; i++)
    {
        if(stopping_angle[i] >= current_angle)
        {
            next_stop = i;
            break;
        }
    }
    MG90S_servo_set_angle(9000 + 1000);     // Giving 1000 centidegree extra
    return stopping_angle[next_stop];
}

/*
 * @Function angle_and_range continue_constant_panning_mode(void)
 * @param None
 * @return None
 * @brief this is a function to drive the constant panning mode; this 
 * function should be called continuously after entering in the constant panning
 * mode
 * @author Bhumil Depani
 */
angle_and_range continue_constant_panning_mode(void)
{
    int16_t current_angle = get_angle(initial_angle);
    angle_and_range output = {0, 0xFFFF};

    if(direction)       // if going from -90 to 90
    {
        if(current_angle > (stopping_angle[next_stop] - TOLERANCE))
        {
            output = get_angle_and_range(initial_angle);
            printf("\nDistance at angle %10d (actual angle is %10d) is: %10d", 
            stopping_angle[next_stop], output.angle, output.range);
            next_stop++;
            if(next_stop == no_of_stop)     // At 90 degree
            {
                direction = 0;      /* reverse the direction of motor rotation; 
                * that is from 90 to -90. */
                next_stop = no_of_stop - 2;
                MG90S_servo_set_angle(-9000 - 1000);
            }
        }
    }
    else                    // if going from 90 to -90
    {
        if(current_angle < (stopping_angle[next_stop] + TOLERANCE))
        {
            output = get_angle_and_range(initial_angle);
            printf("\nDistance at angle %10d (actual angle is %10d) is: %10d", stopping_angle[next_stop], output.angle, output.range);
            if(next_stop == 0)      // at -90 degree
            {
                direction = 1;      /* reverse the direction of motor rotation; 
                * that is from -90 to 90. */
                next_stop = 2;
                MG90S_servo_set_angle(9000 + 1000);
            }
            next_stop--;
        }
    }
    return output;
}


