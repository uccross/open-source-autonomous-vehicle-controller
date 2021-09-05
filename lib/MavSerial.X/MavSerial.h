/* 
 * File:   mav_serial.h
 * Author: Pavlo Vlastos
 *
 * Created on March 13, 2021, 5:19 PM, based on code from Max Dunne
 */

#ifndef MAVSERIAL_H
#define	MAVSERIAL_H

/*******************************************************************************
 * PUBLIC #INCLUDES                                                            *
 ******************************************************************************/
#include "board.h"
#include "mavlink.h"
#include "common.h"
#include "mavlink_helpers.h"

/*******************************************************************************
 * PUBLIC #DEFINES                                                             *
 ******************************************************************************/
#define MAV_SERIAL_BAUD_RATE 115200
#define MAV_SERIAL_RX_BUF_SIZE 5

/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES                                                  *
 ******************************************************************************/

/**
 * @Function serial_init(void)
 * @param none
 * @return none
 * @brief  Initializes the UART subsystem to 115200 and sets up the circular 
 * buffer */
void MavSerial_Init(void);

/**
 * @Function MavSerial_PutChar(char ch)
 * @param ch - the char to be sent out the serial port
 * @return None.
 * @brief  adds char to the end of the circular buffer and forces the interrupt flag 
 * high if nothing is currently transmitting
 * @author Max Dunne, 2011.11.10 modified by Pavlo Vlastos 2019.7.23 */
void MavSerial_PutChar(char ch);

/**
 * @Function MavSerial_GetChar(void)
 * @param None.
 * @return ch - char from the serial port
 * @brief  reads first character from buffer or returns 0 if no chars available
 * @author Max Dunne, 2011.11.10 modified by Pavlo Vlastos 2019.7.23 */
char MavSerial_GetChar(void);

/**
 * @Function MavSerial_IsTxEmpty(void)
 * @param None.
 * @return TRUE or FALSE
 * @brief  returns the state of the receive buffer */
char MavSerial_IsTxEmpty(void);

/**
 * @Function MavSerial_IsRxEmpty(void)
 * @param None.
 * @return TRUE or FALSE
 * @brief  returns the state of the receive buffer */
char MavSerial_IsRxEmpty(void);

/**
 * @function MavSerial_IsMavMsgAvailable(void)
 * @return TRUE or FALSE depending on if a message is available
 */
char MavSerial_IsMavMsgAvailable(void);


/**
 * @function MavSerial_IsMavMsgBufFull(void)
 * @return TRUE or FALSE depending on if the rx MAVLink message buffer is full 
 * or not.
 */
char MavSerial_IsMavMsgBufFull(void);

/**
 * @function MavSerial_IsSpaceLeft(void)
 * @return The number of spaces left in the mav_serial message buffer
 */
uint8_t MavSerial_IsSpaceLeft(void);

/**
 * @function MavSerial_getMavMsg(mavlink_message_t* r_message)
 * @brief Grabs the latest
 * @param r_message Pointer to mavlink message for pass by reference
 * @return TRUE or FALSE if there is a new message
 */
char MavSerial_getMavMsg(mavlink_message_t* r_message);

/**
 * @function MavSerial_sendMavPacket(mavlink_message_t* msg)
 * @param msg A pointer to a mavlink_message_t
 */
void MavSerial_sendMavPacket(mavlink_message_t* msg);

/**
 * @function MavSerial_ParseWrapper(void)
 * @brief Calls mavlink byte parsing state machine. Should be called in main
 * while loop.
 */
void MavSerial_ParseWrapper(void);

/**
 * @function MavSerial_send_msg()
 * @param sys_id The system ID for the vehicle
 * @param t The current time in milliseconds (100 = 100ms)
 * @param wp A waypoint to send to the companion computer
 * @param msg A pointer to a mavlink_message_t
 * @return SUCCESS or ERROR
 */
//int MavSerial_send_msg(uint8_t sys_id, unsigned int t, float wp[MSZ], 
//        mavlink_message_t *msg);

/**
 * @function MavSerial_SendAck()
 * @param sys_id The system ID for the vehicle
 * @param result Result of for the acknowledgment, 1, 0, etc.
 * @param msg A pointer to a mavlink_message_t
 * @return SUCCESS or ERROR
 */
int MavSerial_SendAck(uint8_t sys_id, uint8_t result, 
        mavlink_message_t *msg);

#endif	/* MAVSERIAL_H */

