/* 
 * File: System_timer.h
 * Author: Aaron Hunter
 * Brief: 
 * Created on Jan 12 2021 10:13 am
 * Modified on <month> <day>, <year>, <hour> <pm/am>
 */

#ifndef SYSTEM_TIMER_H // Header guard
#define	SYSTEM_TIMER_H//

/*******************************************************************************
 * PUBLIC #INCLUDES                                                            *
 ******************************************************************************/

#include <stdint.h> 

/*******************************************************************************
 * PUBLIC #DEFINES                                                             *
 ******************************************************************************/


/*******************************************************************************
 * PUBLIC TYPEDEFS                                                             *
 ******************************************************************************/


/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES                                                  *
 ******************************************************************************/


/**
 * @Function void Sys_timer_init(void)
 * @param none
 * @return None.
 * @brief  Initializes the timer module 
 * @author Aaron Hunter*/
void Sys_timer_init(void);

/**
 * Function: Sys_timer_get_msec(void)
 * @param None
 * @return the current millisecond counter value
   */
uint32_t Sys_timer_get_msec(void);

/**
 * Function: Sys_timer_get_usec(void)
 * @param None
 * @return the current microsecond counter value
 * @author Aaron Hunter
   */
uint32_t Sys_timer_get_usec(void);

#endif	/* SYSTEM_TIMER_H */ // End of header guard

