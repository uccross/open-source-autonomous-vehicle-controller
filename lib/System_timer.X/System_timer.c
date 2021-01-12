/* 
 * File:   System_time.c
 * Author: Aaron Hunter
 * Brief: 
 * Created on Jan 12 2021 10:13 am
 * Modified on <month> <day>, <year>, <hour> <pm/am>
 */

/*******************************************************************************
 * #INCLUDES                                                                   *
 ******************************************************************************/

#include "System_timer.h" // The header file for this source file. 
#include "SerialM32.h" // The header file for this source file. 
#include "Board.h"   //Max32 setup      
#include <xc.h>
#include <stdio.h>
#include <sys/attribs.h>  //for ISR definitions
#include <proc/p32mx795f512l.h>

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/
#define PERIOD 9999 //  80 MHz sysclock/8 = 10000 pulses/msec
#define TICKS_PER_USEC 10

/*******************************************************************************
 * PRIVATE VARIABLES                                                           *
 ******************************************************************************/
static uint32_t msec_counter = 0;
static uint32_t usec_counter = 0;
/*******************************************************************************
 * PRIVATE TYPEDEFS                                                            *
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                *
 ******************************************************************************/

/*******************************************************************************
 * PUBLIC FUNCTION IMPLEMENTATIONS                                             *
 ******************************************************************************/

/**
 * @Function void Sys_timer_init(void)
 * @param none
 * @return None.
 * @brief  Initializes the timer module 
 * @author Aaron Hunter*/
void Sys_timer_init(void) {
    __builtin_disable_interrupts(); // disable interrupt at CPU
    /*set up peripheral*/
    T5CON = 0; //set to default
    T5CONbits.TCKPS = 0b011; // prescalar of 1:8
    PR5 = PERIOD; // number of ticks in one cycle (rollover)
    T5CONbits.ON = 1; //turn on the timer
    IFS0bits.T5IF = 0; //clear interrupt flag
    IPC5bits.T5IP = 4; //priority level 4
    IEC0bits.T5IE = 1; //enable interrupt
    __builtin_enable_interrupts(); //enable interrupts
}

/**
 * Function: Sys_timer_get_msec(void)
 * @param None
 * @return the current millisecond counter value
 */
uint32_t Sys_timer_get_msec(void) {
    return msec_counter;
}

/**
 * Function: Sys_timer_get_usec(void)
 * @param None
 * @return the current microsecond counter value
 * @author Aaron Hunter
 */
uint32_t Sys_timer_get_usec(void) {
    // get the timer5 count
    // divide by scalar to convert to usecs 
    // Add to usec_counter
    // return sum
    return (TMR5 / TICKS_PER_USEC + usec_counter);
}

/*******************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATIONS                                            *
 ******************************************************************************/


void __ISR(_TIMER_5_VECTOR, ipl4auto) Timer5_handler(void) {
    // increment millisecond counter
    msec_counter++;
    // increment microsecond counter by 1000
    usec_counter += 1000;
    IFS0bits.T5IF = 0; //clear interrupt flag
}

//Testing harness
#ifdef SYSTEM_TIMER_TESTING

void main(void) {
    uint32_t cur_time = 0;
    uint32_t prev_time = 0;
    uint32_t period = 1000; // 1 sec
    Board_init();
    Serial_init();
    Sys_timer_init();
    printf("System Timer testing, %s, %s \r\n", __DATE__, __TIME__);
    prev_time = Sys_timer_get_msec();
    while (1) {
        cur_time = Sys_timer_get_msec();
        if (cur_time - prev_time >= period) {
            prev_time = cur_time;
            printf("msec count: %d, usec count: %d \r\n", Sys_timer_get_msec(), Sys_timer_get_usec());
        }
    }


}
#endif


