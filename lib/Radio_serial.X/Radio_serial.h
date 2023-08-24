/* 
 * File:   Radio_serial.h
 * Author: Aaron Hunter
 * Brief: 
 * Created on 1/22/2021 12:41 pm
 * Modified 
 */

#ifndef RADIOSERIAL_H // Header guard
#define	RADIOSERIAL_H //

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
 * @Function Radio_serial_init(void)
 * @param none
 * @return none
 * @brief  Initializes the UART subsystem for the SiK radio link
 * @author Aaron Hunter*/
void Radio_serial_init(void);

/**
 * @Function Radio_put_char(char c)
 * @param c, the char to be sent to serial port
 * @return SUCCESS or ERROR
 * @brief  adds the char to the tail of the transmit buffer
 * @author Aaron Hunter*/
char Radio_put_char(unsigned char c);

/**
 * @Function Radio_get_char(void)
 * @return character read from receive buffer
 * @brief  returns the value in the receive buffer
 * @author Aaron Hunter*/
unsigned char Radio_get_char(void);

/**
 * @Function Radio_data_available();
 * @return TRUE or FALSE
 * @brief responds with TRUE if the buffer is not empty
 * @author Aaron Hunter*/
unsigned char Radio_data_available(void);



#endif	/* RADIOSERIAL_H */ // End of header guard

