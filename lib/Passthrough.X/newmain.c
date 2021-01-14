/*
 * File:   newmain.c
 * Author: Aaron Hunter
 * Brief:  main file for the radio-control passthrough mode for the OSAVC
 * project
 * Created on January 13, 2021, 9:41 AM
 */

/*******************************************************************************
 * #INCLUDES                                                                   *
 ******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "xc.h"
#include "Board.h"
#include "SerialM32.h"
#include "RC_RX.h"
#include "RC_servo.h"



/*******************************************************************************
 * #DEFINES                                                                    *
 ******************************************************************************/

/*******************************************************************************
 * VARIABLES                                                                   *
 ******************************************************************************/

/*******************************************************************************
 * TYPEDEFS                                                                    *
 ******************************************************************************/


/*******************************************************************************
 * FUNCTION PROTOTYPES                                                         *
 ******************************************************************************/
uint16_t calc_pw(uint16_t raw_counts);

/*******************************************************************************
 * FUNCTIONS                                                                   *
 ******************************************************************************/
uint16_t calc_pw(uint16_t raw_counts) {
    int16_t normalized_pulse; //converted to microseconds and centered at 0
    uint16_t pulse_width; //servo output in microseconds
    normalized_pulse = (raw_counts - RC_RX_MID_COUNTS) >> 1;
    pulse_width = normalized_pulse + RC_SERVO_CENTER_PULSE;
    return pulse_width;
}

int main(void) {

    enum RC_channels {
        ACCELERATOR = 2,
        STEERING,
        SWITCH_D,
    }; //map to the car controls from the RC receiver
    RCRX_channel_buffer channels[CHANNELS];


    Board_init(); //board configuration
    Serial_init(); //start debug terminal (USB)
    RCRX_init(); // start the RCRX subsystem
    RC_servo_init(); // start the servo subsystem
    printf("RC Passthrough Mode application %s, %s\r\n", __DATE__, __TIME__);
    while (1) {
        if (RCRX_new_cmd_avail() == TRUE) {
            //update channel data
            RCRX_get_cmd(channels);
            //            printf("T %d S %d M %d \r", channels[2], channels[3], channels[4]);
            // update pulsewidths for each servo output

            RC_servo_set_pulse(calc_pw(channels[2]), RC_LEFT_WHEEL);
            RC_servo_set_pulse(calc_pw(channels[2]), RC_RIGHT_WHEEL);
            RC_servo_set_pulse(calc_pw(channels[3]), RC_STEERING);
//            printf("T:%d\r", RC_servo_get_pulse(RC_STEERING));
        }
    }
    return 0;
}
