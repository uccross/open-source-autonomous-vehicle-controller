/*
 * File: main.c
 * Author: Bhuml Depani
 * Brief: This is a main function file for sensor/pico module. Contains 
 * functions to display menu and switch between different modes for distance and
 * angle measurement.
 * Created on 07/12/2022 07:12 pm
*/
#include <stdio.h>                          // C standard Input Output Library
#include "pico/stdlib.h"
#include "constant_panning_mode/constant_panning_mode.h"
#include "range_at_angle/range_at_angle.h"
#include "MG90S_servo/MG90S_servo.h"

typedef enum
{
    CONSTANT_PANNING,
    RANGE_AT_ANGLE,
    IDLE
}lidar_mode;

void print_menu(void);


void main(void)
{
    stdio_init_all();                   // initializes Pico I/O pins
    sleep_ms(1000);

    uint16_t initial_angle = initialize_system_components();

    print_menu();
    int choice;
    int angle, range;
    lidar_mode current_mode = IDLE;
    lidar_mode previous_mode = IDLE;
    while(1)
    {
        choice = getchar_timeout_us(0);
        if(choice != PICO_ERROR_TIMEOUT)
        {
            switch(choice)
            {
                case 'S':
                case 's':
                    current_mode = CONSTANT_PANNING;
                    break;
                case 'X':
                case 'x':
                    current_mode = IDLE;
                    break;
                case 'A':
                case 'a':
                    previous_mode = current_mode;
                    current_mode = RANGE_AT_ANGLE;
                default:
                    printf("\nPlease enter appropriate data.");
                    print_menu();
            }
            sleep_ms(1000);
        }
        
        switch(current_mode)
        {
            case CONSTANT_PANNING:
                if(previous_mode == CONSTANT_PANNING)
                {
                    continue_constant_panning_mode();
                }
                else
                {
                    start_constant_panning_mode(initial_angle);
                    previous_mode = CONSTANT_PANNING;
                }
                break;
            case RANGE_AT_ANGLE:
                printf("\nEnter Angle: ");
                scanf("%d", &angle);
                printf("\nEnter Range: ");
                scanf("%d", &range);
                printf("\nAngle: %d, Range: %d", angle, range);
                range_at_angle_mode(angle, initial_angle);
                current_mode = previous_mode;
                previous_mode = RANGE_AT_ANGLE;
                break;
            case IDLE:
                if(previous_mode == IDLE)
                {
                    angle_and_range output = get_angle_and_range(initial_angle);
                    printf("\nDistance at angle %10d is: %10d", output.angle, 
                    output.range);
                }
                else
                {
                    MG90S_servo_set_angle(0);
                    previous_mode = IDLE;
                }
                break;
        }
    }
}

void print_menu(void)
{
    printf("\n=====================================");
    printf("\n== Type a single character command ==");
    printf("\n=====================================");
    printf("\n S - Start continuous scanning mode");
    printf("\n X - Stop continuous scanning mode");
    printf("\n A, <angle>, <range> - A scan at angle (degrees) over range (degrees)");
}
