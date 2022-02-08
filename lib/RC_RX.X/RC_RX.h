/* 
 * File:   RC_RX.h
 * Author: Aaron Hunter
 * Brief: Header file for the radio control receiver used for input to the 
 * vehicle.  
 * Created on Dec 22,2020 9:42 am
 * Modified on 
 */

#ifndef RC_RX_H // Header guard
#define	RC_RX_H //

/*******************************************************************************
 * PUBLIC #INCLUDES                                                            *
 ******************************************************************************/

#include <stdint.h> 

/*******************************************************************************
 * PUBLIC #DEFINES                                                             *
 ******************************************************************************/
#define CHANNELS 16
#define RC_RX_MID_COUNTS 992
#define RC_RX_MAX_COUNTS 1811
#define RC_RX_MIN_COUNTS 172
#define RC_RAW_TO_FS 10000/(RC_RX_MAX_COUNTS - RC_RX_MID_COUNTS)

/*******************************************************************************
 * PUBLIC TYPEDEFS                                                             *
 ******************************************************************************/
typedef uint16_t RCRX_channel_buffer; //servo data array


/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES                                                  *
 ******************************************************************************/

/**
 * @Function RCRX_init()
 * @param none
 * @return SUCCESS or ERROR
 * @brief initializes the UART, interrupts and data structs for radio control 
 * receiver inputs
 * @note 
 * @author aahunter
 * @modified <Your Name>, <year>.<month>.<day> <hour> <pm/am> */
uint8_t RCRX_init(void);

/**
 * @Function RCRX_new_cmd_avail()
 * @param none
 * @return TRUE or FALSE
 * @brief returns TRUE if a new RC RX packet is available
 * @note 
 * @author aahunter
 * @modified <Your Name>, <year>.<month>.<day> <hour> <pm/am> */
uint8_t RCRX_new_cmd_avail(void);

/**
 * @Function RCRX_get_cmd(RCRX_channel_buffer*)
 * @param pointer to servo data buffer
 * @return SUCCESS or ERROR
 * @brief processes most current message, stores data and returns the pointer
 * to the data
 * @note 
 * @author aahunter
 * @modified <Your Name>, <year>.<month>.<day> <hour> <pm/am> */
uint8_t RCRX_get_cmd(RCRX_channel_buffer *channels);

/**
 * @Function RCRX_get_err()
 * @param pointer to servo data buffer
 * @return number of rc errors detected since init
 * @brief 
 * @note 
 * @author aahunter
 * @modified <Your Name>, <year>.<month>.<day> <hour> <pm/am> */
unsigned int RCRX_get_err(void);





#endif	/* RC_RX_H */ // End of header guard

