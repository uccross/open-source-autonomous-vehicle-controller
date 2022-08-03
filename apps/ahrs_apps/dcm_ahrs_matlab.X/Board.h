/* 
 * File:   Board.h
 * Author: Aaron Hunter
 * Brief: Max32 configuration
 * Created on 11/9/2020
 * Modified on 
 */

#ifndef BOARD_H // Header guard
#define	BOARD_H //

/*******************************************************************************
 * PUBLIC #INCLUDES                                                            *
 ******************************************************************************/

#include <stdint.h>

/*******************************************************************************
 * PUBLIC #DEFINES                                                             *
 ******************************************************************************/
// Boolean defines for TRUE, FALSE, SUCCESS and ERROR

#define FALSE ((int8_t) 0)
#define TRUE ((int8_t) 1)
#define ERROR ((int8_t) -1)
#define SUCCESS ((int8_t) 1)


/*******************************************************************************
 * PUBLIC TYPEDEFS                                                             *
 ******************************************************************************/


/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES                                                  *
 ******************************************************************************/

/**
 * @Function Board_init()
 * @param none
 * @return none
 * @brief initializes the board configuration 
 * @note 
 * @author Aaron Hunter,
 * @modified */
void Board_init(void);

/**
 * @Function Board_get_sys_clock();
 * @brief returns the peripheral clock speed
 * @author Aaron Hunter
 */
uint32_t Board_get_sys_clock(void);

/**
 * @Function Board_get_PB_clock();
 * @brief returns the peripheral clock speed
 * @author Aaron Hunter
 */
uint32_t Board_get_PB_clock(void);



#endif	/* BOARD_H */ // End of header guard

