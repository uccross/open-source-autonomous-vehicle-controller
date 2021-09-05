/* 
 * File:   nmea0183v4.h
 * Author: Pavlo Vlastos
 *
 * Created on January 15, 2020, 12:43 PM
 */

#ifndef NMEA0183V4_H
#define	NMEA0183V4_H

/*******************************************************************************
 * PUBLIC #include                                                             *
 ******************************************************************************/
#include <stdint.h>

/*******************************************************************************
 * PUBLIC #DEFINES                                                             *
 ******************************************************************************/
#define BAUD_RATE 9600
#define TALK_ID_LEN 2
#define S_FRMTTR_LEN 3
#define MAX_MSG_LEN 512
#define CHECKSUM_LEN 2
#define QUEUESIZE 1024
#define FRAME_BUFFER_SIZE 16

/*******************************************************************************
 * PUBLIC VARIABLES                                                   *
 ******************************************************************************/
typedef struct {
    unsigned char talk_id[TALK_ID_LEN+1];
    unsigned char s_frmtr[S_FRMTTR_LEN+1];
    char data[MAX_MSG_LEN+1];
    int dlen;
} nmea_frame_t;

/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES                                                  *
 ******************************************************************************/

/**
 * @Function nmea_serial_init(void)
 * @brief  Initializes UART2 module for reading nmea messages*/
void nmea_serial_init(void);

///**
// * @Function int nmea_send_msg(unsigned char len, void *Payload)
// * @param data, pointer to data, will be copied in during the function
// * @return SUCCESS or ERROR*/
//int nmea_send_msg(nmea_frame_t address, void *data);

/**
 * @Function nmea_checksum_iterate(char xp, char xm)
 * @param xp The next byte of the NMEA packet to be used in checksum calculation
 * @param xm The last byte of the NMEA packet used in checksum calculation
 * @return The result byte of the checksum thus far
 * @brief  Calculates Checksum */
unsigned char nmea_checksum_iterate(unsigned char xp, unsigned char xm);

/**
 * @Function unsigned char nmea_read_head_address(void)
 * @return nmea frame with only the talker ID and sentence formatter. If no
 * message is available the talker ID member will be ERROR 
 * @note reading the address does not remove the nmea message from the buffer. 
 * only nmea_get_data will remove the message at the head of the buffer. */
nmea_frame_t nmea_read_head_address(void);

/**
 * @Function int nmea_get_data(void* data)
 * @param data, Memory location to put data
 * @return SUCCESS or ERROR */
char nmea_get_data(void* data);

/**
 * @Function char nmea_is_msg_available(void)
 * @param None
 * @return TRUE if Queue is not Empty */
char nmea_is_msg_available(void);

/**
 * @Function char nmea_is_cb_full(void)
 * @param None
 * @return TRUE is QUEUE is Full */
char nmea_is_cb_full(void);

/**
 * @Function int nmea_ascii_to_hex(unsigned char ascii_c[CHECKSUM_LEN])
 * @param ascii_c An array containing the ascii character representation of the
 *      two-character hex checksum 
 * @return converted_check The checksum converted from ascii to hex */
unsigned char nmea_ascii_to_hex(unsigned char ascii_c[CHECKSUM_LEN]);

/**
 * @Function nmea_sm_store_msg(unsigned char c)
 * @param c Character read over serial from GPS
 * @brief  runs a state machine to store the incoming characters from GPS and
 * if the characters form a valid message, the checksum is validated, then the
 * message is stored in an nmear message buffer to be parsed later.
 * @return SUCCESS or ERROR */
int nmea_sm_store_msg(unsigned char c);

/**
 * @Function nmea_is_tx_empty(void)
 * @param None.
 * @return TRUE or FALSE
 * @brief  returns the state of the receive buffer */
char nmea_is_tx_empty(void);

/**
 * @Function nmea_rmc_parse(void)
 * @param nmea_frame_t *msg An already populated nmea message frame pointer. 
 * @note nmea_get_data() should be called before using this function. The 
 * format for XXRMC message is as follows:
 * $xxRMC,time,status,lat,NS,long,EW,spd,cog,date,mv,mvEW,posMode,navStatus*cs<CR><LF>
 * @ return SUCCESS or ERROR depending on if navStatus indicates 'A' or 'V' 
 * respectively */
char nmea_rmc_parse(nmea_frame_t *msg);

/**
 * @Function nmea_rmc_parse(void)
 * @param nmea_frame_t *msg An already populated nmea message frame pointer. 
 * @note nmea_get_data() should be called before using this function. The 
 * format for XXRMC message is as follows:
 * $xxRMC,time,status,lat,NS,long,EW,spd,cog,date,mv,mvEW,posMode,navStatus*cs<CR><LF>
 * @ return SUCCESS or ERROR depending on if navStatus indicates 'A' or 'V' 
 * respectively */
char nmea_rmc_parse2(nmea_frame_t *msg);

/**
 * @Function nmea_get_rmc_time(void)
 * @note Must call nmea_rmc_parse continually to get this data
 * @return time */
long double nmea_get_rmc_time(void);

/**
 * @Function nmea_get_rmc_status(void)
 * @note Must call nmea_rmc_parse continually to get this data
 * @return status An 'A' means valid data, and a 'V' means data is INVALID */
char nmea_get_rmc_status(void) ;

/**
 * @Function nmea_get_rmc_lat_int(void)
 * @note Must call nmea_rmc_parse continually to get this data
 * @return lat */
int32_t nmea_get_rmc_lat_int(void);

/**
 * @Function nmea_get_rmc_lat(void)
 * @note Must call nmea_rmc_parse continually to get this data
 * @return lat */
long double nmea_get_rmc_lat(void);

/**
 * @Function nmea_get_rmc_NS(void)
 * @note Must call nmea_rmc_parse continually to get this data
 * @return NS */
char nmea_get_rmc_NS(void);

/**
 * @Function nmea_get_rmc_long_int(void)
 * @note Must call nmea_rmc_parse continually to get this data
 * @return lat */
int32_t nmea_get_rmc_long_int(void);

/**
 * @Function nmea_get_rmc_long(void)
 * @note Must call nmea_rmc_parse continually to get this data
 * @return long */
long double nmea_get_rmc_long(void);

/**
 * @Function nmea_get_rmc_EW(void)
 * @note Must call nmea_rmc_parse continually to get this data
 * @return EW */
char nmea_get_rmc_EW(void);

/**
 * @Function nmea_get_rmc_spd(void)
 * @note Must call nmea_rmc_parse continually to get this data
 * @return spd */
long double nmea_get_rmc_spd(void);

/**
 * @Function nmea_get_rmc_cog(void)
 * @note Must call nmea_rmc_parse continually to get this data
 * @return cog in Degrees*/
long double nmea_get_rmc_cog(void);

/**
 * @Function nmea_get_rmc_date(void)
 * @note Must call nmea_rmc_parse continually to get this data
 * @return date */
long double nmea_get_rmc_date(void);

/**
 * @Function nmea_get_rmc_mv(void)
 * @note Must call nmea_rmc_parse continually to get this data
 * @return mv */
long double nmea_get_rmc_mv(void);

/**
 * @Function nmea_get_rmc_mvEW(void)
 * @note Must call nmea_rmc_parse continually to get this data
 * @return mvEW */
char nmea_get_rmc_mvEW(void);

/**
 * @Function nmea_get_rmc_posMode(void)
 * @note Must call nmea_rmc_parse continually to get this data
 * @return posMode */
char nmea_get_rmc_posMode(void);

/**
 * @Function nmea_get_rmc_navStatus(void)
 * @note Must call nmea_rmc_parse continually to get this data
 * @return navStatus An 'A' means equipment is providing navigational status 
 * and that data is valid, a 'V' means it is NOT */
char nmea_get_rmc_navStatus(void);

/**
 * @Function nmea_get_vtg_cogt(void)
 * @param nmea_frame_t *msg An already populated nmea message frame pointer.
 * Should call nmea_get_data() before using this function. 
 * @return course over ground in degrees 
 * @note 
 * $xxVTG,cogt,T,cogm,M,knots,N,kph,K,posMode*cs<CR><LF>*/
long double nmea_get_vtg_cogt(nmea_frame_t *msg); // MAKE PARSER TOO IF NEEDED

/**
 * @Function nmea_get_gsa_hdop(void)
 * @param nmea_frame_t *msg An already populated nmea message frame pointer.
 * Should call nmea_get_data() before using this function. 
 * @return Horizontal dilution of precision
 * @note 
 * $xxGSA,opMode,navMode{,sv},PDOP,HDOP,VDOP,systemId*cs<CR><LF>*/
long double nmea_get_gsa_hdop(nmea_frame_t *msg);

/**
 * @Function nmea_get_all_data()
 * @param cur_msg A pointer to an NMEA frame/message
 * @param lat_i Latitude as an integer pointer
 * @param lat_f Latitude as an float pointer
 * @param long_i Longitude as an integer pointer
 * @param long_f Longitude as an float pointer
 * @param cog Course over ground float pointer
 * @param vel Velocity float pointer
 * @param hdop Horizontal dilution of precision float pointer
 */
void nmea_get_all_data(nmea_frame_t *cur_msg, int32_t *lat_i, 
        long double *lat_f, int32_t *long_i, long double *long_f, 
        long double *cog, long double *vel, long double *hdop);

#endif	/* NMEA0183V4_H */

