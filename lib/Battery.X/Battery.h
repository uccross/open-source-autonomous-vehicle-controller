/* 
 * File:   Battery.h
 * Author: Aaron Hunter
 * Brief: 
 * Created on May 2, 2022, 10:36 am
 * Modified on <month> <day>, <year>, <hour> <pm/am>
 */

#ifndef BATTERY_H // Header guard
#define	BATTERY_H //

/*******************************************************************************
 * PUBLIC #INCLUDES                                                            *
 ******************************************************************************/


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
 * @Function Battery_init(void);
 * @return ERROR or SUCCESS
 * @brief Initialize the AD system for battery operation
 * @note 
 * @author Aaron Hunter,
 * @modified <Your Name>, <year>.<month>.<day> <hour> <pm/am> */
int8_t Battery_init(void);



#endif	/* BATTERY_H */ // End of header guard

