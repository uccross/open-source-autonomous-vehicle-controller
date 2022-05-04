/* 
 * File:   ARX.c
 * Author: Aaron Hunter
 * Brief: app to develop motor models. A white noise sequence generates pseudo-
 * random outputs which are converted into step commands for the motors
 * Angle and velocity data are captured with the AS5047D angular encoders. The
 * data are streamed out the serial port as mavlink messages
 * Created on May 3, 2022, 10:58 AM
 */

/*******************************************************************************
 * #INCLUDES                                                                   *
 ******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include "xc.h"
#include "Board.h"
#include "SerialM32.h"
#include "System_timer.h"
#include "mavlink.h"
#include "RC_servo.h"
#include "AS5047D.h"

/*******************************************************************************
 * #DEFINES                                                                    *
 ******************************************************************************/
#define HEARTBEAT_PERIOD 1000 //1 sec interval for hearbeat update
#define CONTROL_PERIOD 10 //Period for control loop in msec
#define ENCODER_MAX_CTS 16384.0 //2^14 counts/rev for the encoder
#define ENCODER_TWO_PI 0.000383495197  //counts to 2 pi conversion = 2pi/0x4fff
#define WHITESEQ_LENGTH 255
#define TESTPULSE 100


/*******************************************************************************
 * VARIABLES                                                                   *
 ******************************************************************************/
mavlink_system_t mavlink_system = {
    1, // System ID (1-255)
    MAV_COMP_ID_AUTOPILOT1 // Component ID (a MAV_COMPONENT value)
};

encoder_t encoder_data[NUM_ENCODERS];
static uint8_t pub_RC_servo = TRUE;
static uint8_t pub_Encoder = TRUE;
static float cts_to_deg = 360.0 / ENCODER_MAX_CTS;
static uint8_t white_seq[WHITESEQ_LENGTH];
uint16_t seq_index = 0;

/*******************************************************************************
 * FUNCTION PROTOTYPES                                                         *
 ******************************************************************************/
void Gen_white_sequence(void) {
    int32_t new_val;
    uint8_t num_digits = 32;
    // seed random number generator
    srand(time(NULL));
    new_val = rand();
    /*Fill the array*/
    //    printf("printing values\r\n");
    for (seq_index = 0; seq_index < WHITESEQ_LENGTH; seq_index++) {
        white_seq[seq_index] = (new_val & 0x1); //next element in sequence is LSB of new_val
        //        printf("%d", new_val & 0x1);
        new_val = new_val >> 1; //shift one bit right
        if (new_val == 0) { // if current value is zero, get a new random int
            new_val = rand();
            //            printf("new value %d \r\n", new_val);
        }
    }
    printf("\r\n");
}

/*
 * 
 */
int main(int argc, char** argv) {
    uint32_t cur_time = 0;
    uint32_t warmup_time = 5000; //time in ms to allow subsystems to stabilize (IMU))
    uint32_t control_start_time = 0;
    uint16_t index = 0;
    uint16_t test_idx = 10;
    uint16_t pulse_width = RC_SERVO_CENTER_PULSE;
    char testing = TRUE;

    Board_init();
    Serial_init();
    Sys_timer_init();
    RC_servo_init();
    printf("Motor ARX app %s, %s \r\n", __DATE__, __TIME__);
    printf("Connecting to ESCs\r\n");
    RC_servo_set_pulse(RC_SERVO_CENTER_PULSE + 10, RC_LEFT_WHEEL);
    RC_servo_set_pulse(RC_SERVO_CENTER_PULSE + 10, RC_LEFT_WHEEL);
    control_start_time = Sys_timer_get_msec();
    while ((cur_time - control_start_time) < warmup_time) {
        cur_time = Sys_timer_get_msec();
    }

    Gen_white_sequence();

    index = 0;
    control_start_time = Sys_timer_get_msec();
    printf("testing...\r\n");
    while (testing == TRUE) {
        cur_time = Sys_timer_get_msec(); //check the time
        /*if the control period is reached */
        if (cur_time - control_start_time >= CONTROL_PERIOD) {
            control_start_time = Sys_timer_get_msec(); //reset control timer
            /*update the motors*/
            if (white_seq[index] == 0) {
                pulse_width = RC_SERVO_CENTER_PULSE - TESTPULSE;
                RC_servo_set_pulse(pulse_width, RC_STEERING);
                RC_servo_set_pulse(pulse_width, RC_LEFT_WHEEL);
                //                printf("-");
            } else {
                pulse_width = RC_SERVO_CENTER_PULSE + TESTPULSE;
                RC_servo_set_pulse(pulse_width, RC_STEERING);
                RC_servo_set_pulse(pulse_width, RC_LEFT_WHEEL);
                //                printf("+");
            }
            index++;
            //            if (index % 80 == 0) { //wrap screen
            //                printf("\r\n");
            //            }
            if (index == WHITESEQ_LENGTH) {
                testing = FALSE;
            }
        }
    }
    RC_servo_set_pulse(RC_SERVO_CENTER_PULSE, RC_STEERING);
    RC_servo_set_pulse(RC_SERVO_CENTER_PULSE - 20, RC_LEFT_WHEEL);
    printf("\r\nTest complete.\r\n");
    while (1);
    return (EXIT_SUCCESS);
}

