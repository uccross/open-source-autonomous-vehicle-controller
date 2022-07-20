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
#define CONTROL_PERIOD 50 //Period for control loop in msec
#define ENCODER_PERIOD 10 //period for the encoder upate rate
#define ONE_SEC 1000 //one second in msec
#define ENCODER_MAX_CTS 16384.0 //2^14 counts/rev for the encoder
#define ENCODER_TWO_PI 0.000383495197  //counts to 2 pi conversion = 2pi/0x4fff
#define WHITESEQ_LENGTH 255
#define TESTPULSE 100
#define RC_ESC_TRIM -10 //microsecond offset for zero rotation
#define STEP_TEST
// #define ARX


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

/**
 * @function Gen_white_sequence(void)
 * @brief: seeds the random number generator and creates a sequence of pseudo-
 * random numbers. Sequence is stored in module level variable white_seq[]
 * @author Aaron Hunter
 * @date May 9, 2022 11:00 am
 */
void Gen_white_sequence(void) {
    int32_t new_val;
    uint8_t num_digits = 32;
    // seed random number generator
    srand(time(NULL));
    new_val = rand();
    /*Fill the array*/
    for (seq_index = 0; seq_index < WHITESEQ_LENGTH; seq_index++) {
        white_seq[seq_index] = (new_val & 0x1); //next element in sequence is LSB of new_val
        new_val = new_val >> 1; //shift one bit right
        if (new_val == 0) { // if current value is zero, get a new random int
            new_val = rand();
        }
    }
}

/*
 * 
 */
int main(int argc, char** argv) {
    uint32_t cur_time = 0;
    uint32_t warmup_time = 5000;
    uint32_t control_start_time = 0;
    uint32_t encoder_start_time = 0;
    uint16_t index = 0;
    uint16_t test_idx = 10;
    int8_t pulse_direction;
    uint16_t pulse_width = RC_SERVO_CENTER_PULSE;
    int16_t delta_pw = 100;
    char testing = TRUE;
    encoder_t encoder_data[NUM_ENCODERS];

    Board_init();
    Serial_init();
    Sys_timer_init();
    RC_servo_init();
    Encoder_init();
    //initialize encoder data
    for (index = 0; index < NUM_ENCODERS; index++) {
        Encoder_init_encoder_data(&encoder_data[index]);
    }

    printf("Motor ARX app %s, %s \r\n", __DATE__, __TIME__);
    printf("Starting ESC \r\n");

    control_start_time = Sys_timer_get_msec();
    while ((cur_time - control_start_time) < warmup_time) { //give ESC time to start up
        cur_time = Sys_timer_get_msec();
    }
    RC_servo_set_pulse(pulse_width + RC_ESC_TRIM, RC_LEFT_WHEEL);
    RC_servo_set_pulse(pulse_width + RC_ESC_TRIM, RC_RIGHT_WHEEL);
    printf("ESC pulsewidth set\r\n");

    control_start_time = Sys_timer_get_msec();
    while ((cur_time - control_start_time) < warmup_time) { //give ESC time to start up
        cur_time = Sys_timer_get_msec();
    }
    printf("ESC startup complete.\r\n");

#ifdef ARX
    printf("testing...\r\n");
    Gen_white_sequence();

    index = 0;
    control_start_time = Sys_timer_get_msec();
    encoder_start_time = control_start_time;
    while (testing == TRUE) {
        cur_time = Sys_timer_get_msec(); //check the time
        /*if the control period is reached */
        if (cur_time - control_start_time >= CONTROL_PERIOD) {
            control_start_time = Sys_timer_get_msec(); //reset control timer
            /*update the motors*/
            if (white_seq[index] == 0) {
                pulse_direction = -1;
            } else {
                pulse_direction = 1;
            }
            RC_servo_set_pulse(RC_SERVO_CENTER_PULSE + (pulse_direction * TESTPULSE), RC_STEERING);
            RC_servo_set_pulse((RC_SERVO_CENTER_PULSE + RC_ESC_TRIM) + (pulse_direction * TESTPULSE), RC_LEFT_WHEEL);
            RC_servo_set_pulse((RC_SERVO_CENTER_PULSE + RC_ESC_TRIM) + (pulse_direction * TESTPULSE), RC_RIGHT_WHEEL);

            /*publish RC_servo data*/
            index++;
            if (index >= WHITESEQ_LENGTH) {
                testing = FALSE;
            }
        }
        /*initiate encoder data acquisition */
        if (cur_time - encoder_start_time >= ENCODER_PERIOD) {
            encoder_start_time = cur_time; //reset encoder timer
            Encoder_start_data_acq();
        }
        /*check for new data and publish */
        if (Encoder_is_data_ready() == TRUE) {
            Encoder_get_data(encoder_data);
            /*publish encoder data*/
            printf("%d,%d,%d,%d,%d\r\n", cur_time, pulse_direction, encoder_data[RC_LEFT_WHEEL].next_theta, encoder_data[RC_RIGHT_WHEEL].next_theta, encoder_data[RC_STEERING].next_theta);
        }
    }
    /*set all controls to zero*/
    RC_servo_set_pulse(RC_SERVO_CENTER_PULSE, RC_STEERING);
    RC_servo_set_pulse(RC_SERVO_CENTER_PULSE + RC_ESC_TRIM, RC_LEFT_WHEEL);
    RC_servo_set_pulse((RC_SERVO_CENTER_PULSE + RC_ESC_TRIM), RC_RIGHT_WHEEL);
    printf("ARX Test complete.\r\n");
#endif //ARX
    
#ifdef STEP_TEST
    printf("Starting step test\r\n");
    /* Allow motors time to stop*/
    control_start_time = Sys_timer_get_msec();
    testing = TRUE;
    while (testing) {
        cur_time = Sys_timer_get_msec();
        if (cur_time - control_start_time >= ONE_SEC) testing = FALSE;
    }
    testing = TRUE;
    control_start_time = Sys_timer_get_msec();
    encoder_start_time = control_start_time;
    RC_servo_set_pulse(RC_SERVO_CENTER_PULSE + RC_ESC_TRIM + TESTPULSE, RC_LEFT_WHEEL);
    RC_servo_set_pulse((RC_SERVO_CENTER_PULSE + RC_ESC_TRIM + TESTPULSE), RC_RIGHT_WHEEL);
    while (testing) {
        cur_time = Sys_timer_get_msec(); //check the time
        /*initiate encoder data acquisition */
        if (cur_time - encoder_start_time >= ENCODER_PERIOD) {
            encoder_start_time = cur_time; //reset encoder timer
            Encoder_start_data_acq();
        }
        /*check for new data and publish */
        if (Encoder_is_data_ready() == TRUE) {
            Encoder_get_data(encoder_data);
            /*publish encoder data*/
            printf("%d,%d,%d,%d,%d\r\n", cur_time, pulse_direction, encoder_data[RC_LEFT_WHEEL].next_theta, encoder_data[RC_RIGHT_WHEEL].next_theta, encoder_data[RC_STEERING].next_theta);
        }
        if (cur_time - control_start_time > 2 * ONE_SEC) testing = FALSE;
    }
    /*stop the motors*/
    RC_servo_set_pulse(RC_SERVO_CENTER_PULSE + RC_ESC_TRIM, RC_LEFT_WHEEL);
    RC_servo_set_pulse((RC_SERVO_CENTER_PULSE + RC_ESC_TRIM), RC_RIGHT_WHEEL);
    printf("Testing complete\r\n");
#endif //STEP_TEST
    
    while (1);
    return (EXIT_SUCCESS);
}

