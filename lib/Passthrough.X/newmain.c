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
#define CTS_2_USEC 1  //divide by 2 scaling of counts to usec

/*******************************************************************************
 * VARIABLES                                                                   *
 ******************************************************************************/

/*******************************************************************************
 * TYPEDEFS                                                                    *
 ******************************************************************************/


/*******************************************************************************
 * FUNCTION PROTOTYPES                                                         *
 ******************************************************************************/
/**
 * @Function calc_pw(uint16_t raw_counts)
 * @param raw counts from the radio transmitter (11 bit unsigned int)
 * @return pulse width in microseconds
 * @brief converts the RC input into the equivalent pulsewidth output for servo
 * and ESC control
 * @author aahunter
 * @modified <Your Name>, <year>.<month>.<day> <hour> <pm/am> */
static uint16_t calc_pw(uint16_t raw_counts);

/*******************************************************************************
 * FUNCTIONS                                                                   *
 ******************************************************************************/

/**
 * @Function calc_pw(uint16_t raw_counts)
 * @param raw counts from the radio transmitter (11 bit unsigned int)
 * @return pulse width in microseconds
 * @brief converts the RC input into the equivalent pulsewidth output for servo
 * and ESC control
 * @author aahunter
 * @modified <Your Name>, <year>.<month>.<day> <hour> <pm/am> */
static uint16_t calc_pw(uint16_t raw_counts) {
    int16_t normalized_pulse; //converted to microseconds and centered at 0
    uint16_t pulse_width; //servo output in microseconds
    normalized_pulse = (raw_counts - RC_RX_MID_COUNTS) >> CTS_2_USEC;
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
            RC_servo_set_pulse(calc_pw(channels[ACCELERATOR]), RC_LEFT_WHEEL);
            RC_servo_set_pulse(calc_pw(channels[ACCELERATOR]), RC_RIGHT_WHEEL);
            RC_servo_set_pulse(calc_pw(channels[STEERING]), RC_STEERING);
//            printf("F:%d, T: %d\r", RC_servo_get_pulse(RC_STEERING), RC_servo_get_pulse(RC_LEFT_WHEEL));
        }
    }
    return 0;
}
