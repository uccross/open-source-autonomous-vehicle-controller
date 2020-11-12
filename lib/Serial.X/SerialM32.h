/* 
 * File:   SerialM32.h
 * Author: Aaron Hunter
 * Brief: Serial library for Max32 processor
 * Created on November 10, 2020, 9:52 am
 * Modified on <month> <day>, <year>, <hour> <pm/am>
 */

#ifndef SERIAL_M32_H // Header guard
#define	SERIAL_M32_H //

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
 * @Function serial_init(void)
 * @param none
 * @return none
 * @brief  Initializes the UART subsystem to 115200 and sets up the circular buffer
 * @author Aaron Hunter*/
void Serial_init(void);

/**
 * @Function put_char(char c)
 * @param c, the char to be sent to serial port
 * @return SUCCESS or ERROR
 * @brief  adds the char to the tail of the transmit buffer
 * @author Aaron Hunter*/
char put_char(unsigned char c);

/**
 * @Function get_char(void)
 * @return character read from receive buffer
 * @brief  returns the value in the receive buffer
 * @author Aaron Hunter*/
unsigned char get_char(void);

/**
 * @Function _mon_putc(char c)
 * @param c - char to be sent
 * @return None.
 * @brief  overwrites weakly define extern to use circular buffer instead of Microchip 
 * functions
 * @author Max Dunne, 2011.11.10 */
void _mon_putc(char c);



#endif	/* SERIAL_M32_H */ // End of header guard

