/* 
 * File:   EEPROM.h
 * Author: Aaron Hunter
 * Brief: interface to EEPROM storage
 * Created on May 6, 2021, 11:48 am
 * Modified on <month> <day>, <year>, <hour> <pm/am>
 */

#ifndef EEPROM_H // Header guard
#define	EEPROM_H //

/*******************************************************************************
 * PUBLIC #INCLUDES                                                            *
 ******************************************************************************/
#include <sys/types.h>

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
 * @Function EEPROM_init(void)
 * @param none
 * @return SUCCESS or ERROR
 * @brief initializes I2C2 peripheral to 100 KHz and sets up interrupt
 * @note 
 * @author Aaron Hunter,
 * @modified 
 */
uint8_t EEPROM_init(void);



#endif	/* EEPROM_H */ // End of header guard

