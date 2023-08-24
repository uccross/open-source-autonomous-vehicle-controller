/* 
 * File:   NEO_M8N.h
 * Author: Aaron Hunter
 * Brief: Library driver for ublox NEO-M8N GPS receiver
 * Created on 12/09/2020
 * Modified 
 */

#ifndef NEO_M8N_H // Header guard
#define	NEO_M8N_H //

/*******************************************************************************
 * PUBLIC #INCLUDES                                                            *
 ******************************************************************************/



/*******************************************************************************
 * PUBLIC #DEFINES                                                             *
 ******************************************************************************/


/*******************************************************************************
 * PUBLIC TYPEDEFS                                                             *
 ******************************************************************************/
struct GPS_data{
    double time; // UTC time in seconds
    double lat; //latitude, deg 
    double lon; //longitude, deg 
    double spd; //GPS speed in knots
    double cog; //GPS heading in deg
};


/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES                                                  *
 ******************************************************************************/

/**
 * @Function GPS_init(void)
 * @param None
 * @return SUCCESS or ERROR
 * @brief sets up UART2 for communication 
 * @author Aaron Hunter */
int GPS_init(void);

/**
 * @Function char GPS_parse_stream(void)
 * @return SUCCESS or ERROR
 * @brief sends received GPS message to parser
 * @author Aaron Hunter */
char GPS_parse_stream(void);

/**
 * @Function GPS_is_msg_avail(void)
 * @param None
 * @return TRUE if queue is not empty
 * @brief 
 * @author Aaron Hunter */
char GPS_is_msg_avail(void);

/**
 * @Function GPS_is_data_avail(void)
 * @param None
 * @return TRUE if GPS data table is not empty
 * @brief 
 * @author Aaron Hunter */
char GPS_is_data_avail(void);


/*
 *Function char GPS_has_fix(void);
 * return TRUE if GPS data is valid
 * author:  Aaron Hunter
 */
char GPS_has_fix(void);

/**
 * Function GPS_get_data(struct * GPS_data)
 * param *GPS  pointer to GPS_data struct
 * return SUCCESS or  ERROR
 * brief:  grabs data at the read index of the data table and stores the values
 * in GPS
 * author: Aaron Hunter
 */
char GPS_get_data(struct GPS_data* data);





#endif	/* NEO_M8N_H */ // End of header guard

