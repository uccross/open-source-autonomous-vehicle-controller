/* 
 * File:   garmin_v3hp.h
 * Author: Aaron Hunter
 * Brief: Garmin Lidar-lite v3hp driver header file
 * Created on April, 27, 2021, 11:23 am
 * Modified on <month> <day>, <year>, <hour> <pm/am>
 */

#ifndef GARMINV3HP_H // Header guard
#define	GARMINV3HP_H //

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
 * @Function uint16_t Lidar_get_range(void);
 * @return 16 bit range measurement 
 * @brief initiates new measurement and returns the most recent one
 */
uint16_t Lidar_get_range(void);

/**
 * @Function Lidar_Init(void);
 * @brief sets up I2C communication and configures LIDAR
 * @return SUCCESS or ERROR
 * @author Aaron Hunter */
int8_t Lidar_Init(void);



#endif	/* GARMINV3HP_H */ // End of header guard

