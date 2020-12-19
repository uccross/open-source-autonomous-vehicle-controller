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
#include "xc.h"
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/attribs.h>
#include <proc/p32mx795f512l.h>



/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/
#define USEC_NUM 5
#define USEC_LOG_DEN 1

/*******************************************************************************
 * PRIVATE VARIABLES                                                            *
 ******************************************************************************/
static uint16_t pulse_width = 0; //PW in microseconds
static uint16_t raw_ticks = 0; // raw ticks corresponding to pulse width
/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                 *
 ******************************************************************************/
/**
 * @Function __ISR(_OUTPUT_COMPARE_2_VECTOR, IPL4AUTO) OC2_ISR_handler(void);
 * @ brief handles output compare IRQ, updates duty cycle of PWM if different
 * @ author Aaron Hunter
 */
void __ISR(_OUTPUT_COMPARE_2_VECTOR, IPL4AUTO) OC2_ISR_handler(void);

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
 * @brief initializes hardware required and set it to the CENTER PULSE */
uint8_t RC_servo_init(void) {
    pulse_width = RC_SERVO_CENTER_PULSE;
    raw_ticks = (pulse_width * USEC_NUM) >> USEC_LOG_DEN;
    __builtin_disable_interrupts();
    /* timer 3 settings */
    T3CON = 0;
    T3CONbits.TCKPS = 0b101; // prescaler set to 1:32
    PR3 = 49999; //20 msec <=> 50 Hz

    /* Output Capture configuration*/
    OC2CON = 0x0000; //Turn off OC3 while doing setup.
    OC2R = 0x0000; // Initialize primary Compare Register
    OC2RS = 0x0000; // Initialize secondary Compare Register
    OC2CONbits.OC32 = 0; //16 bit mode
    OC2CONbits.OCTSEL = 1; //use timer 3
    OC2CONbits.OCM = 0b101; // Continuous pulses
    OC2R = 0; // start of pulse
    OC2RS = raw_ticks; //this parameter can be changed at any time by the user
    /*Configure interrupt*/
    IFS0bits.OC2IF = 0; //clear interrupt
    IPC2bits.OC2IP = 0b100; //priority 4
    IEC0bits.OC2IE = 1; //enable interrupt

    /* turn on the peripherals */
    T3CONbits.ON = 1;
    OC2CONbits.ON = 1;
    __builtin_enable_interrupts();
    return SUCCESS;
}

/**
 * @Function int RC_servo_set_pulse(uint16_t in_pulse)
 * @param in_pulse, integer representing number of microseconds
 * @return SUCCESS or ERROR
 * @brief takes in microsecond count, converts to ticks and updates the internal variables
 * @warning This will update the timing for the next pulse, not the current one */
uint8_t RC_servo_set_pulse(uint16_t in_pulse) {
    /*if inPulse < min pulse, inPulse = min pulse*/
    /*if inPulse > max pulse, inPulse = max pulse*/
    if (in_pulse < RC_SERVO_MIN_PULSE) {
        in_pulse = RC_SERVO_MIN_PULSE;
    }
    if (in_pulse > RC_SERVO_MAX_PULSE) {
        in_pulse = RC_SERVO_MAX_PULSE;
    }
    if (in_pulse != pulse_width) {
        pulse_width = in_pulse;
        raw_ticks = (pulse_width * USEC_NUM) >> USEC_LOG_DEN; //only update PW register if the value has changed
    }
    return SUCCESS;
}

/**
 * @Function int RCServo_GetPulse(void)
 * @param None
 * @return Pulse in microseconds currently set */
uint16_t RC_servo_get_pulse(void) {
    return (uint16_t) (RC_servo_get_raw_ticks() << USEC_LOG_DEN) / USEC_NUM;
}

/**
 * @Function int RCServo_GetRawTicks(void)
 * @param None
 * @return raw timer ticks required to generate current pulse. */
uint16_t RC_servo_get_raw_ticks(void) {
    return raw_ticks;
}
/*******************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATIONS                                            *
 ******************************************************************************/

/**
 * @Function __ISR(_OUTPUT_COMPARE_3_VECTOR, IPL6SOFT) OutputCompareISR(void)
 * @ brief handles output compare IRQ, updates duty cycle of PWM if different
 * @ author Aaron Hunter
 */

void __ISR(_OUTPUT_COMPARE_2_VECTOR, IPL4AUTO) OC2_ISR_handler(void) {
    /*update compare register if it's changed*/
    OC2RS = raw_ticks;
    IFS0bits.OC2IF = 0; //clear interrupt flag
}

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

/*Test harness*/
#ifdef RCSERVO_TESTING

void main(void) {
    uint16_t test_pulse = RC_SERVO_CENTER_PULSE;
    int direction = 1;
    Board_init();
    Serial_init();
    RC_servo_init();

    printf("\r\nRC_servo Test Harness %s %s\r\n", __DATE__, __TIME__);


    while (1) {
        if (direction == 1) {
            RC_servo_set_pulse(test_pulse);
            test_pulse += 10;
            if (test_pulse > RC_SERVO_MAX_PULSE) {
                direction = -1;
            }
        }
        if (direction == -1) {
            RC_servo_set_pulse(test_pulse);
            test_pulse -= 10;
            if (test_pulse < RC_SERVO_MIN_PULSE) {
                direction = 1;
            }
        }
        RC_servo_delay(80000);
        printf("Current pulse width: %d, current ticks: %d \r\n", RC_servo_get_pulse(), RC_servo_get_raw_ticks());
    }
}

#endif //RCSERVO_TESTING