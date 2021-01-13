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

/*******************************************************************************
 * FUNCTIONS                                                                   *
 ******************************************************************************/


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
    printf("RC Passthrough Mode application %s, %s", __DATE__,__TIME__);
    while(1){
        if (RCRX_new_cmd_avail){
            //update channel data
            RCRX_get_cmd(&channels[0]);
            printf("T %d S %d M %d \r", channels[2], channels[3], channels[4]);
            // update pulsewidths for each servo output
        }
    }
    return 0;
}
