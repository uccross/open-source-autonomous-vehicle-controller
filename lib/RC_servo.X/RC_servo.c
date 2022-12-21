/* 
 * File:   RC_servo.c
 * Author: Aaron Hunter
 * Brief: Library driver for up to four servos for the Max32 dev board
 * Created on 12/18/2020 3;30 pm
 * Modified 
 */

/*******************************************************************************
 * #INCLUDES                                                                   *
 ******************************************************************************/

#include "RC_servo.h" // The header file for this source file.
#include "Board.h"   //Max32 setup
#include "SerialM32.h"
#include "System_timer.h"
#include <xc.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/attribs.h>  //for ISR definitions
#include <proc/p32mx795f512l.h>



/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/
#define USEC_NUM 5  //numerator multiplier to avoid divides
#define USEC_LOG_DEN 1  //right shift to divide by 2^x

/*******************************************************************************
 * PRIVATE VARIABLES                                                            *
 ******************************************************************************/
static uint16_t pulse_width[RC_SERVO_NUM_OUTPUTS]; //PW in microseconds
static uint16_t raw_ticks[RC_SERVO_NUM_OUTPUTS]; // raw ticks corresponding to pulse width
static int8_t RC_SET_NEW_CMD = FALSE; //flag to indicate when the new command can be loaded 
/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                 *
 ******************************************************************************/

/**
 * @Function delay(int cycles)
 * @param cycles, number of cycles to delay
 * @brief simple delay loop
 * @note ~500nsec for one delay, then +12.5 nsec for every increment higher
 * @author ahunter
 */
void RC_servo_delay(int cycles);
/*******************************************************************************
 * PUBLIC FUNCTION IMPLEMENTATIONS                                             *
 ******************************************************************************/

/**
 * @Function RC_servo_init(void)
 * @param None
 * @return SUCCESS or ERROR
 * @brief initializes hardware required and set it to the correct initial setting
 *  */
int8_t RC_servo_init(uint8_t output_type, uint8_t output_channel) {
    uint8_t i;
    uint16_t ticks_center = (RC_SERVO_CENTER_PULSE * USEC_NUM) >> USEC_LOG_DEN;
    uint16_t ticks_min = (RC_SERVO_MIN_PULSE * USEC_NUM) >> USEC_LOG_DEN;
    printf("min ticks %d \r\n", ticks_min);

    __builtin_disable_interrupts();
    /* timer 3 settings */
    T3CON = 0;
    TMR3 = 0x0;
    T3CONbits.TCKPS = 0b101; // prescaler set to 1:32
    PR3 = 49999; //20 msec <=> 50 Hz
    /*setup timer3 interrupt on period rollover*/
    IPC3bits.T3IP = 0b110; //priority 6
    IPC3bits.T3IS = 0b01; // subpriority 1
    IFS0bits.T3IF = 0; // clear interrupt flag
    IEC0bits.T3IE = 1; //enable interrupt

    /* Output Capture configuration*/
    switch (output_channel) {
        case SERVO_PWM_1:
            if (output_type == RC_SERVO_TYPE || output_type == ESC_BIDIRECTIONAL_TYPE) {
                pulse_width[SERVO_PWM_1] = RC_SERVO_CENTER_PULSE;
                raw_ticks[SERVO_PWM_1] = ticks_center;
            } else {
                pulse_width[SERVO_PWM_1] = RC_SERVO_MIN_PULSE;
                raw_ticks[SERVO_PWM_1] = ticks_min;
            }
            OC2CON = 0x0000; //Turn off OC3 while doing setup.
            OC2R = 0x0000; // Initialize primary Compare Register
            OC2RS = 0x0000; // Initialize secondary Compare Register
            OC2CONbits.OC32 = 0; //16 bit mode
            OC2CONbits.OCTSEL = 1; //use timer 3
            OC2CONbits.OCM = 0b110; // PWM mode, no fault detection
            OC2R = raw_ticks[SERVO_PWM_1]; // need load this register initially
            OC2RS = raw_ticks[SERVO_PWM_1]; // OCxRS -> OCxR at timer rollover
            printf("OC2RS: %d\r\n", OC2RS);
            OC2CONbits.ON = 1; //turn on the peripheral
        case SERVO_PWM_2:
            if (output_type == RC_SERVO_TYPE || output_type == ESC_BIDIRECTIONAL_TYPE) {
                pulse_width[SERVO_PWM_2] = RC_SERVO_CENTER_PULSE;
                raw_ticks[SERVO_PWM_2] = ticks_center;
            } else {
                pulse_width[SERVO_PWM_2] = RC_SERVO_MIN_PULSE;
                raw_ticks[SERVO_PWM_2] = ticks_min;
            }
            OC3CON = 0x0;
            OC3R = 0x0000; // Initialize primary Compare Register
            OC3RS = 0x0000; // Initialize secondary Compare Register
            OC3CONbits.OC32 = 0; //16 bit mode
            OC3CONbits.OCTSEL = 1; //use timer 3
            OC3CONbits.OCM = 0b110; // PWM mode, no fault detection
            OC3R = raw_ticks[SERVO_PWM_2]; // need load this register initially 
            OC3RS = raw_ticks[SERVO_PWM_2]; // OCxRS -> OCxR at timer rollover
            OC3CONbits.ON = 1; //turn on the peripheral
        case SERVO_PWM_3:
            if (output_type == RC_SERVO_TYPE || output_type == ESC_BIDIRECTIONAL_TYPE) {
                pulse_width[SERVO_PWM_3] = RC_SERVO_CENTER_PULSE;
                raw_ticks[SERVO_PWM_3] = ticks_center;
            } else {
                pulse_width[SERVO_PWM_3] = RC_SERVO_MIN_PULSE;
                raw_ticks[SERVO_PWM_3] = ticks_min;
            }
            OC4CON = 0x0;
            OC4R = 0x0000; // Initialize primary Compare Register
            OC4RS = 0x0000; // Initialize secondary Compare Register
            OC4CONbits.OC32 = 0; //16 bit mode
            OC4CONbits.OCTSEL = 1; //use timer 3
            OC4CONbits.OCM = 0b110; // PWM mode, no fault detection
            OC4R = raw_ticks[SERVO_PWM_3]; // need load this register initially 
            OC4RS = raw_ticks[SERVO_PWM_3]; // OCxRS -> OCxR at timer rollover
            OC4CONbits.ON = 1; //turn on the peripheral

        case SERVO_PWM_4:
            if (output_type == RC_SERVO_TYPE || output_type == ESC_BIDIRECTIONAL_TYPE) {
                pulse_width[SERVO_PWM_4] = RC_SERVO_CENTER_PULSE;
                raw_ticks[SERVO_PWM_4] = ticks_center;
            } else {
                pulse_width[SERVO_PWM_4] = RC_SERVO_MIN_PULSE;
                raw_ticks[SERVO_PWM_4] = ticks_min;
            }
            OC5CON = 0x0;
            OC5R = 0x0000; // Initialize primary Compare Register
            OC5RS = 0x0000; // Initialize secondary Compare Register
            OC5CONbits.OC32 = 0; //16 bit mode
            OC5CONbits.OCTSEL = 1; //use timer 3
            OC5CONbits.OCM = 0b110; // PWM mode, no fault detection
            OC5R = raw_ticks[SERVO_PWM_4]; // need load this register initially 
            OC5RS = raw_ticks[SERVO_PWM_4]; // OCxRS -> OCxR at timer rollover
            OC5CONbits.ON = 1; //turn on the peripheral
        default:
            break;
    }
    /* turn on the timer */
    T3CONbits.ON = 1;
    __builtin_enable_interrupts();
    return SUCCESS;
}

/**
 * @Function int RC_servo_set_pulse(uint16_t in_pulse, uint8_t which_servo)
 * @param in_pulse, integer representing PWM width in microseconds
 * @param which_servo, servo number to set
 * @return SUCCESS or ERROR
 * @brief takes in microsecond count, converts to ticks and updates the internal variables
 * @warning This will update the timing for the next pulse, not the current one */
int8_t RC_servo_set_pulse(uint16_t in_pulse, uint8_t which_servo) {
    if (in_pulse < RC_SERVO_MIN_PULSE) { //prevent servos from exceeding limits
        in_pulse = RC_SERVO_MIN_PULSE;
    }
    if (in_pulse > RC_SERVO_MAX_PULSE) {
        in_pulse = RC_SERVO_MAX_PULSE;
    }
    if (in_pulse != pulse_width[which_servo]) { //only update struct and OCxRS register if new value
        pulse_width[which_servo] = in_pulse;
        raw_ticks[which_servo] = (in_pulse * USEC_NUM) >> USEC_LOG_DEN; //only update PW register if the value has changed
        switch (which_servo) {
            case SERVO_PWM_1:
                OC2RS = raw_ticks[which_servo]; //load new PWM value into OCxRS
                break;
            case SERVO_PWM_2:
                OC3RS = raw_ticks[which_servo]; //load new PWM value into OCxRS
                break;
            case SERVO_PWM_3:
                OC4RS = raw_ticks[which_servo]; //load new PWM value into OCxRS
                break;
            case SERVO_PWM_4:
                OC5RS = raw_ticks[which_servo]; //load new PWM value into OCxRS
                break;
            default:
                break;
        }
    }
    RC_SET_NEW_CMD = FALSE; //reset flag after new command is set
    return SUCCESS;
}

/**
 * @Function RC_servo_get_pulse(uint8_t which_servo)
 * @param which_servo, servo number to retrieve PWM value from 
 * @return Pulse in microseconds currently set */
uint16_t RC_servo_get_pulse(uint8_t which_servo) {
    return pulse_width[which_servo];
}

/**
 * @Function RC_servo_get_raw_ticks(void)
 * @param which_servo, servo number to retrieve raw timer compare value from 
 * @return raw timer ticks required to generate current pulse. */
uint16_t RC_servo_get_raw_ticks(uint8_t which_servo) {
    return raw_ticks[which_servo];
}

/**
 * @Function RC_servo_cmd_needed(void)
 * @brief returns TRUE when the RC servo period register is ready for a new
 * pulsewidth 
 * @return TRUR or FALSE
 * @author Aaron Hunter
 */
uint8_t RC_servo_cmd_needed(void) {
    return RC_SET_NEW_CMD;
}
/*******************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATIONS                                            *
 ******************************************************************************/

/**
 * @Function delay(int cycles)
 * @param cycles, number of cycles to delay
 * @brief simple delay loop
 * @note ~500nsec for one delay, then +12.5 nsec for every increment higher
 * @author ahunter
 */
void RC_servo_delay(int cycles) {
    int i;
    for (i = 0; i < cycles; i++) {
        Nop();
    }
}

/**
 * @Function __ISR(_Timer_3_Vector, ipl6) RC_T3_handler(void)
 * @brief sets a flag when the period register rolls over
 * @author ahunter
 */
void __ISR(_TIMER_3_VECTOR, ipl6auto) RC_T3_handler(void) {
    RC_SET_NEW_CMD = TRUE; //set new command needed boolean
    // printf("ISR\r\n");
    IFS0bits.T3IF = 0; //clear interrupt flag
}

/*Test harness*/
#ifdef RCSERVO_TESTING

void main(void) {
    uint16_t test_pulse = RC_SERVO_MIN_PULSE;
    uint16_t start_time;
    uint16_t cur_time;
    uint16_t ESC_time = 5000;
    uint16_t test_time = 250;
    int direction = 1;
    int DONE_TESTING = FALSE;
    Board_init();
    Serial_init();
    Sys_timer_init();
    printf("\r\nRC_servo Test Harness %s %s\r\n", __DATE__, __TIME__);

    RC_servo_init(ESC_UNIDIRECTIONAL_TYPE, SERVO_PWM_1);
    RC_servo_init(ESC_UNIDIRECTIONAL_TYPE, SERVO_PWM_2);
    RC_servo_init(ESC_UNIDIRECTIONAL_TYPE, SERVO_PWM_3);
    RC_servo_init(ESC_UNIDIRECTIONAL_TYPE, SERVO_PWM_4);
    printf("Exiting init() functions\r\n");
    start_time = Sys_timer_get_msec();
    cur_time = start_time;
    while ((cur_time - start_time) <= ESC_time) {
        cur_time = Sys_timer_get_msec();
    }

    printf("ESC delay over\r\n");

    while (DONE_TESTING == FALSE) {
        if (RC_servo_cmd_needed() == TRUE) {
            if (direction == 1) {
                RC_servo_set_pulse(test_pulse, SERVO_PWM_1);
                RC_servo_set_pulse(test_pulse, SERVO_PWM_2);
                RC_servo_set_pulse(test_pulse, SERVO_PWM_3);
                RC_servo_set_pulse(test_pulse, SERVO_PWM_4);
                test_pulse += 10;
                if (test_pulse > RC_SERVO_CENTER_PULSE) {
                    direction = -1;
                }
            }
            if (direction == -1) {
                RC_servo_set_pulse(test_pulse, SERVO_PWM_1);
                RC_servo_set_pulse(test_pulse, SERVO_PWM_2);
                RC_servo_set_pulse(test_pulse, SERVO_PWM_3);
                RC_servo_set_pulse(test_pulse, SERVO_PWM_4);
                test_pulse -= 10;
                if (test_pulse < RC_SERVO_MIN_PULSE) {
                    direction = 1;
                    DONE_TESTING = TRUE;
                }
            }
        }
        printf("SERVO_PWM_1: %d, current ticks: %d \r\n", RC_servo_get_pulse(SERVO_PWM_1), RC_servo_get_raw_ticks(SERVO_PWM_1));
        printf("SERVO_PWM_2: %d, current ticks: %d \r\n", RC_servo_get_pulse(SERVO_PWM_2), RC_servo_get_raw_ticks(SERVO_PWM_2));
        printf("SERVO_PWM_3: %d, current ticks: %d \r\n", RC_servo_get_pulse(SERVO_PWM_3), RC_servo_get_raw_ticks(SERVO_PWM_3));
        printf("SERVO_PWM_4: %d, current ticks: %d \r\n", RC_servo_get_pulse(SERVO_PWM_4), RC_servo_get_raw_ticks(SERVO_PWM_4));
        start_time = Sys_timer_get_msec();
        cur_time = start_time;
        while ((cur_time - start_time) <= test_time) {
            cur_time = Sys_timer_get_msec();
        }
    }
}

#endif //RCSERVO_TESTING