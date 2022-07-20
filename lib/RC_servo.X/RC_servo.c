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
static uint16_t pulse_width[RC_NUM_SERVOS]; //PW in microseconds
static uint16_t raw_ticks[RC_NUM_SERVOS]; // raw ticks corresponding to pulse width
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
 * @brief initializes hardware and sets all PWM periods to to RC_SERVO_CENTER_PULSE */
int8_t RC_servo_init(void) {
    uint8_t i;
    uint16_t ticks_center = (RC_SERVO_CENTER_PULSE * USEC_NUM) >> USEC_LOG_DEN;
    for (i = 0; i < RC_NUM_SERVOS; i++) {
        pulse_width[i] = RC_SERVO_CENTER_PULSE;
        raw_ticks[i] = ticks_center;
    }
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
    OC2CON = 0x0000; //Turn off OC3 while doing setup.
    OC2R = 0x0000; // Initialize primary Compare Register
    OC2RS = 0x0000; // Initialize secondary Compare Register
    OC2CONbits.OC32 = 0; //16 bit mode
    OC2CONbits.OCTSEL = 1; //use timer 3
    OC2CONbits.OCM = 0b110; // PWM mode, no fault detection
    OC2R = raw_ticks[RC_LEFT_WHEEL]; // need load this register initially
    OC2RS = raw_ticks[RC_LEFT_WHEEL]; // OCxRS -> OCxR at timer rollover
    OC2CONbits.ON = 1; //turn on the peripheral

    if (RC_NUM_SERVOS > 1) { //second servo = RIGHT_WHEEL
        OC3CON = 0x0;
        OC3R = 0x0000; // Initialize primary Compare Register
        OC3RS = 0x0000; // Initialize secondary Compare Register
        OC3CONbits.OC32 = 0; //16 bit mode
        OC3CONbits.OCTSEL = 1; //use timer 3
        OC3CONbits.OCM = 0b110; // PWM mode, no fault detection
        OC3R = raw_ticks[RC_RIGHT_WHEEL]; // need load this register initially 
        OC3RS = raw_ticks[RC_RIGHT_WHEEL]; // OCxRS -> OCxR at timer rollover
        OC3CONbits.ON = 1; //turn on the peripheral
    }
    if (RC_NUM_SERVOS > 2) { //third servo = STEERING servo
        OC4CON = 0x0;
        OC4R = 0x0000; // Initialize primary Compare Register
        OC4RS = 0x0000; // Initialize secondary Compare Register
        OC4CONbits.OC32 = 0; //16 bit mode
        OC4CONbits.OCTSEL = 1; //use timer 3
        OC4CONbits.OCM = 0b110; // PWM mode, no fault detection
        OC4R = raw_ticks[RC_STEERING]; // need load this register initially 
        OC4RS = raw_ticks[RC_STEERING]; // OCxRS -> OCxR at timer rollover
        OC4CONbits.ON = 1; //turn on the peripheral
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
            case RC_LEFT_WHEEL:
                OC2RS = raw_ticks[which_servo]; //load new PWM value into OCxRS
                break;
            case RC_RIGHT_WHEEL:
                OC3RS = raw_ticks[which_servo]; //load new PWM value into OCxRS
                break;
            case RC_STEERING:
                OC4RS = raw_ticks[which_servo]; //load new PWM value into OCxRS
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
    uint16_t test_pulse = RC_SERVO_CENTER_PULSE;
    int direction = 1;
    Board_init();
    Serial_init();
    RC_servo_init();

    printf("\r\nRC_servo Test Harness %s %s\r\n", __DATE__, __TIME__);
    RC_servo_set_pulse(test_pulse + RC_ESC_TRIM, RC_LEFT_WHEEL);
    RC_servo_set_pulse(test_pulse + RC_ESC_TRIM, RC_RIGHT_WHEEL);
    RC_servo_delay(320000);

    while (1) {
        if (RC_servo_cmd_needed() == TRUE) {
            if (direction == 1) {
                RC_servo_set_pulse(test_pulse + RC_ESC_TRIM, RC_LEFT_WHEEL);
                RC_servo_set_pulse(test_pulse + RC_ESC_TRIM, RC_RIGHT_WHEEL);
                RC_servo_set_pulse(test_pulse, RC_STEERING);
                test_pulse += 10;
                if (test_pulse > RC_SERVO_MAX_PULSE) {
                    direction = -1;
                }
            }
            if (direction == -1) {
                RC_servo_set_pulse(test_pulse + RC_ESC_TRIM, RC_LEFT_WHEEL);
                RC_servo_set_pulse(test_pulse + RC_ESC_TRIM, RC_RIGHT_WHEEL);
                RC_servo_set_pulse(test_pulse, RC_STEERING);
                test_pulse -= 10;
                if (test_pulse < RC_SERVO_MIN_PULSE) {
                    direction = 1;
                }
            }
        }
        printf("LEFT PWM: %d, current ticks: %d \r\n", RC_servo_get_pulse(RC_LEFT_WHEEL), RC_servo_get_raw_ticks(RC_LEFT_WHEEL));
        printf("RIGHT PWM: %d, current ticks: %d \r\n", RC_servo_get_pulse(RC_RIGHT_WHEEL), RC_servo_get_raw_ticks(RC_RIGHT_WHEEL));
        printf("STEERING PWM: %d, current ticks: %d \r\n", RC_servo_get_pulse(RC_STEERING), RC_servo_get_raw_ticks(RC_STEERING));
        RC_servo_delay(320000);

    }
    // RC_servo_set_pulse(RC_SERVO_MIN_PULSE, RC_LEFT_WHEEL);
}

#endif //RCSERVO_TESTING