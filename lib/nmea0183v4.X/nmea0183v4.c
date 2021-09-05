/*
 * File:   nmea0183v4.c
 * Author: Pavlo Vlastos
 *
 * Created on January 15, 2020, 12:42 PM based on Protocol.h by Max Dunne
 */

/******************************************************************************/
/* #INCLUDES */
/******************************************************************************/
#include "nmea0183v4.h"
#include "Board.h"
#include "xc.h"
#include <sys/attribs.h> //needed to use an interrupt
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <proc/p32mx795f512l.h>

/*******************************************************************************
 * #DEFINES
 ******************************************************************************/
#define CRX_BUFF_SIZE 1024
#define HDOP_N 15
#define MAX_FIELD_LEN 30
#define SIG_FIGS 10
#define DEG_FIGS 3
#define MIN_FIGS 9 // 7 + 2, including '.' and ending NULL
#define MINS2DEC_SEC ((long double) 60.0)
#define BASE 10

/*******************************************************************************
 * PRIVATE DATATYPES                                                           *
 ******************************************************************************/
typedef struct CircBuffer {
    unsigned char buffer[QUEUESIZE];
    int head;
    int tail;
    unsigned int size;
    unsigned char overflowCount;
} CircBuffer;
typedef struct CircBuffer* CBRef;

enum rmc_fields {
    RMC_F_TIME = 1,
    RMC_F_STATUS,
    RMC_F_LAT,
    RMC_F_NS,
    RMC_F_LONG,
    RMC_F_EW,
    RMC_F_SPD,
    RMC_F_COG,
    RMC_F_DATE,
    RMC_F_MV,
    RMC_F_MVEW,
    RMC_F_POSMODE,
    RMC_F_NAVSTATUS
};

typedef enum rmc_fields rmc_field_t;

/*******************************************************************************
 * PRIVATE VARIABLES 
 ******************************************************************************/
static long double rmc_time = 0.0;
static char rmc_status = 0;
static int32_t rmc_lat_int = 0;
static long double rmc_lat = 0.0;
static char rmc_NS = 0;
static int32_t rmc_long_int = 0;
static long double rmc_long = 0.0;
static char rmc_EW = 0;
static long double rmc_spd = 0.0;
static long double rmc_cog = 0.0;
static long double rmc_date = 0.0;
static long double rmc_mv = 0.0;
static char rmc_mvEW = 0;
static char rmc_posMode = 0;
static char rmc_navStatus = 0;

enum nmea_rx_states {
    WAITING_FOR_START = 0,
    READING_TALKER_ID,
    READING_SENTENCE_FORMAT,
    READING_DATA,
    READING_CHECKSUM
};

typedef enum nmea_rx_states rx_states_t;
static rx_states_t rx_state = WAITING_FOR_START;

static nmea_frame_t frame_cb[FRAME_BUFFER_SIZE]; // circular buffer
static int frame_cb_head = 0;
static int frame_cb_tail = 0;
static nmea_frame_t pmsg; // possible message

static unsigned char crx2 = 0;
static unsigned char crx3 = 0; // To debug checksum

// UART2
struct CircBuffer outgoingUart2;
CBRef transmitBuffer2;
struct CircBuffer incomingUart2;
CBRef receiveBuffer2;
static uint8_t AddingToTransmit2 = FALSE;
static uint8_t TransmitCollisionOccured2 = FALSE;
/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES                                                         *
 ******************************************************************************/
static void newCircBuffer(CBRef cB);
static unsigned int getLength(CBRef cB);
static unsigned char readFront(CBRef cB);


/*******************************************************************************
 * PUBLIC FUNCTIONS                                                           *
 ******************************************************************************/

/**
 * @Function nmea_serial_init(void)
 * @brief  Initializes UART2 module for reading nmea messages*/
void nmea_serial_init(void) {
    // UART2
    transmitBuffer2 = (struct CircBuffer*) &outgoingUart2; //set up buffer for receive
    newCircBuffer(transmitBuffer2);

    /* 1. Initialize the UxBRG register for the appropriate baud rate */
    U2BRG = (Board_get_PB_clock() / (16 * (BAUD_RATE))) - 1;
    U2MODE = 0; /* 8-bit data, no parity bit */
    U2MODEbits.STSEL = 0; /* Stop Selection bit: 0 -> 1 Stop bit */
    IEC1bits.U2TXIE = 1; // Enabled transmit interrupts 
    IPC8bits.U2IP = 2; // Priority. 1 lowest, 7 highest
    IPC8bits.U2IS = 1; // Sub-priority 0 lowest, 7 highest
    U2STAbits.UTXISEL = 1; /* Interrupt is generated and asserted when all 
                             * characters have been transmitted *//* For receive */
    IEC1bits.U2RXIE = 1; // Enable receive interrupts as well
    U2STAbits.URXISEL = 0; /* Interrupt flag bit is asserted while receive 
                            * buffer is not empty (i.e., has at least 1 data
                            * character). */
    U2STAbits.UTXEN = 1;
    U2STAbits.URXEN = 1;
    U2MODEbits.ON = 1;
}

///**
// * @Function int nmea_send_msg(unsigned char len, void *Payload)
// * @param data, pointer to data, will be copied in during the function
// * @return SUCCESS or ERROR*/
//int nmea_send_msg(nmea_frame_t address, void *data);

/**
 * @Function nmea_checksum_iterate(unsigned char xp, unsigned char xm)
 * @param xp The next byte of the NMEA packet to be used in checksum calculation
 * @param xm The last byte of the NMEA packet used in checksum calculation
 * @return The result byte of the checksum thus far
 * @brief  Calculates Checksum */
unsigned char nmea_checksum_iterate(unsigned char xp, unsigned char xm) {
    return (xp ^= xm);
}

/**
 * @Function unsigned char nmea_read_head_address(void)
 * @return nmea frame with only the talker ID and sentence formatter. If no
 * message is available the talker ID member will be ERROR 
 * @note reading the address does not remove the nmea message from the buffer. 
 * only nmea_get_data will remove the message at the head of the buffer. */
nmea_frame_t nmea_read_head_address(void) {
    nmea_frame_t address;
    memset(&address, 0, sizeof (address));
    address.talk_id[0] = ((unsigned char) ERROR);
    address.talk_id[1] = ((unsigned char) ERROR);
    int i = 0;
    if (nmea_is_msg_available() == TRUE) {
        for (i = 0; i < TALK_ID_LEN; i++) {
            address.talk_id[i] = frame_cb[frame_cb_head].talk_id[i];
        }

        for (i = 0; i < S_FRMTTR_LEN; i++) {
            address.s_frmtr[i] = frame_cb[frame_cb_head].s_frmtr[i];
        }
    }
    return address;
}

/**
 * @Function int nmea_get_data(void* data)
 * @param data, Memory location to put data
 * @return SUCCESS or ERROR */
char nmea_get_data(void* data) {
    int status = ERROR;
    int i = 0;
    if (nmea_is_msg_available() == TRUE) {
        for (i = 0; i < frame_cb[frame_cb_head].dlen; i++) {
            ((char *) data)[i] = frame_cb[frame_cb_head].data[i];
        }
        frame_cb_head++;
        frame_cb_head %= FRAME_BUFFER_SIZE;
        status = SUCCESS;
    }
    return status;
}

/**
 * @Function char nmea_is_msg_available(void)
 * @param None
 * @return TRUE if Queue is not Empty */
char nmea_is_msg_available(void) {
    char status = FALSE;
    int calc_size = 0;
    calc_size = ((frame_cb_head - (frame_cb_tail + 1)) % FRAME_BUFFER_SIZE);
    if (calc_size < 0) {
        calc_size = calc_size * (-1);
    }
    if (calc_size > 1) {
        status = TRUE;
    }
    return status;
}

/**
 * @Function char nmea_is_cb_full(void)
 * @param None
 * @return TRUE is QUEUE is Full */
char nmea_is_cb_full(void) {
    char status = TRUE;
    int calc_size = 0;
    calc_size = ((frame_cb_head - (frame_cb_tail + 1)) % FRAME_BUFFER_SIZE);
    if (calc_size < 0) {
        calc_size = calc_size * (-1);
    }
    if (calc_size < FRAME_BUFFER_SIZE) {
        status = FALSE;
    }
    return status;
}

/**
 * @Function int nmea_ascii_to_hex(char ascii_c[CHECKSUM_LEN])
 * @param ascii_c An array containing the ascii character representation of the
 *      two-character hex checksum 
 * @return converted_check The checksum converted from ascii to hex */
unsigned char nmea_ascii_to_hex(unsigned char ascii_c[CHECKSUM_LEN]) {
    unsigned char converted_check = 0;
    unsigned char temp_c = 0;
    int i = 0;
    for (i = 0; i < CHECKSUM_LEN; i++) {
        converted_check = (converted_check << 4);
        if ((ascii_c[i] >= '0') && (ascii_c[i] <= '9')) {
            temp_c = (ascii_c[i] - 48); // See ASCII table
        } else if ((ascii_c[i] >= 'A') && (ascii_c[i] <= 'F')) {
            temp_c = (ascii_c[i] - 55); // See ASCII table
        }
        converted_check |= temp_c;
    }
    return converted_check;
}

/**
 * @Function nmea_sm_store_msg(unsigned char c)
 * @param c Character read over serial from GPS
 * @brief  runs a state machine to store the incoming characters from GPS and
 * if the characters form a valid message, the checksum is validated, then the
 * message is stored in an nmear message buffer to be parsed later.
 * @return SUCCESS or ERROR */
int nmea_sm_store_msg(unsigned char c) {
    static int i = 0;
    static int data_len = 0;
    static unsigned char temp_checksum = 0;
    static unsigned char rx_checksum[CHECKSUM_LEN];

    if (c == '$') {
        rx_state = WAITING_FOR_START;
    }

    switch (rx_state) {
        case WAITING_FOR_START:
            if (c == '$') {
                memset(&pmsg, 0, sizeof (pmsg));
                memset(&rx_checksum, 0, sizeof (rx_checksum));
                i = 0;
                data_len = 0;
                temp_checksum = 0;
                rx_state = READING_TALKER_ID;
            }
            break;

        case READING_TALKER_ID:
            if ((c != 'G') && (i == 0)) {
                rx_state = WAITING_FOR_START;
            } else {
                pmsg.talk_id[i] = c;
                i++;
                temp_checksum = nmea_checksum_iterate(c, temp_checksum);

                if (i == TALK_ID_LEN) {
                    i = 0;
                    rx_state = READING_SENTENCE_FORMAT;
                }
            }
            break;

        case READING_SENTENCE_FORMAT:
            pmsg.s_frmtr[i] = c;
            i++;
            temp_checksum = nmea_checksum_iterate(c, temp_checksum);

            if (i == S_FRMTTR_LEN) {
                i = 0;
                rx_state = READING_DATA;
            }
            break;

        case READING_DATA:
            if ((i < MAX_MSG_LEN) && (c != '*')) {
                pmsg.data[i] = c;
                temp_checksum = nmea_checksum_iterate(c, temp_checksum);
                data_len++;
                i++;
            } else if (c == '*') {
                i = 0;
                pmsg.dlen = data_len;
                rx_state = READING_CHECKSUM;
            } else if (i >= MAX_MSG_LEN) {
                rx_state = WAITING_FOR_START;
            }
            break;

        case READING_CHECKSUM:
            if (i == 0) {
                rx_checksum[i] = c;
                i++;
            } else if (i == 1) {
                rx_checksum[i] = c;
                crx2 = nmea_ascii_to_hex(rx_checksum);
                crx3 = temp_checksum;
                if (temp_checksum == nmea_ascii_to_hex(rx_checksum)) {
                    if (nmea_is_cb_full() == FALSE) {

                        frame_cb[frame_cb_tail].talk_id[0] = pmsg.talk_id[0];
                        frame_cb[frame_cb_tail].talk_id[1] = pmsg.talk_id[1];
                        frame_cb[frame_cb_tail].s_frmtr[0] = pmsg.s_frmtr[0];
                        frame_cb[frame_cb_tail].s_frmtr[1] = pmsg.s_frmtr[1];
                        frame_cb[frame_cb_tail].s_frmtr[2] = pmsg.s_frmtr[2];

                        for (i = 0; (i < MAX_MSG_LEN) && (i < pmsg.dlen); i++) {
                            frame_cb[frame_cb_tail].data[i] = pmsg.data[i];
                        }

                        frame_cb[frame_cb_tail].dlen = pmsg.dlen;
                        frame_cb_tail++;
                        frame_cb_tail %= FRAME_BUFFER_SIZE;

                    }
                }
                temp_checksum = 0;
                rx_checksum[0] = 0;
                rx_checksum[1] = 0;
                rx_state = WAITING_FOR_START;
            }
            break;
    }
    return SUCCESS;
}

/**
 * @Function nmea_is_tx_empty(void)
 * @param None.
 * @return TRUE or FALSE
 * @brief  returns the state of the receive buffer */
char nmea_is_tx_empty(void) {
    if (getLength(transmitBuffer2) == 0)
        return TRUE;
    return FALSE;
}

/**
 * @Function nmea_rmc_parse(void)
 * @param nmea_frame_t *msg An already populated nmea message frame pointer. 
 * @note nmea_get_data() should be called before using this function. The 
 * format for XXRMC message is as follows:
 * $xxRMC,time,status,lat,NS,long,EW,spd,cog,date,mv,mvEW,posMode,navStatus*cs<CR><LF>
 * @ return SUCCESS or ERROR depending on if navStatus indicates 'A' or 'V' 
 * respectively */
char nmea_rmc_parse(nmea_frame_t *msg) {
    char status = ERROR;
    char f_str[MAX_FIELD_LEN]; // Field string
    char degrees[DEG_FIGS] = {0};
    char minutes[MIN_FIGS] = {0};
    int d = 0;
    int i = 0;
    int j = 0;
    int m = 0;
    rmc_field_t field_num = 0;

    for (i = 0; i < sizeof (msg->data); i++) {
        if (j < MAX_FIELD_LEN) {
            f_str[j] = msg->data[i];
            j++;
        }
        if ((msg->data[i] == ',') || (msg->data[i] == 0)) {
            switch (field_num) {
                case RMC_F_TIME:
                    rmc_time = (atof(f_str));
                    break;
                case RMC_F_STATUS:
                    rmc_status = f_str[0];
                    break;
                case RMC_F_LAT:
                    /* Loop to decimal, the two left characters are minutes 
                     * Example: 4717.112671 -> 47 degrees, 17.112671 minutes 
                     * Divide minutes by 60 to get decimal degrees.
                     * Example: 47.28521118*/
                    for (d = 0; (d < DEG_FIGS) && (f_str[d + 2] != '.'); d++) {
                        degrees[d] = f_str[d];
                    }
                    for (m = 0; (m < MIN_FIGS); d++, m++) {
                        minutes[m] = f_str[d];
                    }
                    rmc_lat = (atof(degrees) + (atof(minutes) / MINS2DEC_SEC));
                    break;
                case RMC_F_NS:
                    rmc_NS = f_str[0];
                    //                    if (rmc_NS == 'S') {
                    //                        rmc_lat = rmc_lat * (-1.0);
                    //                    }
                    break;
                case RMC_F_LONG:
                    for (d = 0; (d < DEG_FIGS) && (f_str[d + 2] != '.'); d++) {
                        degrees[d] = f_str[d];
                    }
                    for (m = 0; (m < MIN_FIGS); d++, m++) {
                        minutes[m] = f_str[d];
                    }
                    rmc_long = (atof(degrees) + (atof(minutes) / MINS2DEC_SEC));
                    break;
                case RMC_F_EW:
                    rmc_EW = f_str[0];
                    //                    if (rmc_NS == 'W') {
                    //                        rmc_long = rmc_long * (-1.0);
                    //                    }
                    break;
                case RMC_F_SPD:
                    rmc_spd = (atof(f_str));
                    break;
                case RMC_F_COG:
                    rmc_cog = (atof(f_str));
                    break;
                case RMC_F_DATE:
                    rmc_date = (atof(f_str));
                    break;
                    /* *********************************************************
                     * Only available with Automotive Dead Reckoning (ADR) 4.10 
                     * and above. */
                case RMC_F_MV:
                    //rmc_mv = (atof(f_str));
                    break;
                case RMC_F_MVEW:
                    //rmc_mvEW = f_str[0];
                    break;
                    /**********************************************************/
                case RMC_F_POSMODE:
                    rmc_posMode = f_str[0];
                    if (rmc_navStatus != 'N') { // If there is a fix: success
                        status = SUCCESS;
                    }
                    break;
                case RMC_F_NAVSTATUS:
                    rmc_navStatus = f_str[0];
                    if (rmc_navStatus == 'A') {
                        status = SUCCESS;
                    }
                    break;
            }
            field_num++;
            j = 0;
            memset(&f_str, 0, sizeof (f_str));
            memset(&degrees, 0, sizeof (degrees));
            memset(&minutes, 0, sizeof (minutes));
        }
    }
    return status;
}

/**
 * @Function nmea_rmc_parse2(void)
 * @param nmea_frame_t *msg An already populated nmea message frame pointer. 
 * @note nmea_get_data() should be called before using this function. The 
 * format for XXRMC message is as follows:
 * $xxRMC,time,status,lat,NS,long,EW,spd,cog,date,mv,mvEW,posMode,navStatus*cs<CR><LF>
 * @ return SUCCESS or ERROR depending on if navStatus indicates 'A' or 'V' 
 * respectively */
char nmea_rmc_parse2(nmea_frame_t *msg) {
    char status = ERROR;
    char temp[MAX_FIELD_LEN] = {0};
    const char s[2] = ",";
    char *token;
    char *p;
    int i = 0;
    int j = 0;
    long int temp_full = 0;
    long int temp_deg = 0;
    long int temp_min = 0;
    rmc_field_t field_num = 1;
    for (token = strtok(msg->data, s); field_num < (MAX_MSG_LEN + 1);
            token = strtok(NULL, s), field_num++) {

        if (token != NULL) {
            switch (field_num) {
                case RMC_F_TIME:
                    rmc_time = (atof(token));
                    break;
                case RMC_F_STATUS:
                    //rmc_status = *token;
                    break;
                case RMC_F_LAT:
                    //temp_full = atof(token);
                    sprintf(temp, "%s", token);
                    /* Find index of the period and shift everything up one decimal 
                     * place */
                    for (i = 0; (temp[i] != '.') && (i < MAX_FIELD_LEN); i++);
                    for (j = 0; (j + 1) < MAX_FIELD_LEN; j++) {
                        if (j >= i) {
                            temp[j] = temp[j + 1];
                        }
                    }
                    // @TODO: make this a helper function
                    // Convert to decimal degrees
                    temp_full = strtol(temp, &p, BASE);
                    temp_deg = temp_full;
                    temp_deg /= 10000000;
                    temp_deg *= 10000000;
                    temp_min = temp_full - temp_deg;
                    temp_min *= 100;
                    temp_min /= 60;
                    temp_full = temp_deg + temp_min;
                    rmc_lat_int = ((int32_t) temp_full);
                    rmc_lat = ((long double) temp_full
                            / ((long double) 10000000.0));
                    break;
                case RMC_F_NS:
                    //rmc_NS = *token;
                    //                    if (rmc_NS == 'S') {
                    //                        rmc_lat = rmc_lat * (-1.0);
                    //                    }
                    break;
                case RMC_F_LONG:
                    sprintf(temp, "%s", token);
                    /* Find index of the period and shift everything up one decimal 
                     * place */
                    for (i = 0; (temp[i] != '.') && (i < MAX_FIELD_LEN); i++);
                    for (j = 0; (j + 1) < MAX_FIELD_LEN; j++) {
                        if (j >= i) {
                            temp[j] = temp[j + 1];
                        }
                    }
                    // @TODO: make this a helper function
                    // Convert to decimal degrees
                    temp_full = strtol(temp, &p, BASE);
                    temp_deg = temp_full;
                    temp_deg /= 10000000;
                    temp_deg *= 10000000;
                    temp_min = temp_full - temp_deg;
                    temp_min *= 100;
                    temp_min /= 60;
                    temp_full = temp_deg + temp_min;
                    rmc_long_int = ((int32_t) temp_full);
                    rmc_long = ((long double) temp_full
                            / ((long double) 10000000.0));
                    break;
                case RMC_F_EW:
                    //rmc_EW = *token;
                    //                    if (rmc_NS == 'W') {
                    //                        rmc_long = rmc_long * (-1.0);
                    //                    }
                    break;
                case RMC_F_SPD:
                    sprintf(temp, "%s", token);
                    rmc_spd = (atof(temp));
                    break;
                case RMC_F_COG:
                    sprintf(temp, "%s", token);
                    rmc_cog = (atof(temp));
                    if (rmc_cog > 360.0) {
                        rmc_cog = 0.0;
                    }
                    break;
                case RMC_F_DATE:
                    rmc_date = (atof(token));
                    break;
                    /* *********************************************************
                     * Only available with Automotive Dead Reckoning (ADR) 4.10 
                     * and above. */
                case RMC_F_MV:
                    //rmc_mv = (atof(f_str));
                    break;
                case RMC_F_MVEW:
                    //rmc_mvEW = f_str[0];
                    break;
                    /**********************************************************/
                case RMC_F_POSMODE:
                    //rmc_posMode = *token;
                    if (rmc_navStatus != 'N') { // If there is a fix: success
                        status = SUCCESS;
                    }
                    break;
                case RMC_F_NAVSTATUS:
                    //rmc_navStatus = *token;
                    if (rmc_navStatus == 'A') {
                        status = SUCCESS;
                    }
                    break;
            }

            temp_full = 0.0;
            temp_deg = 0;
            temp_min = 0.0;
        }
    }
    return status;
}

/**
 * @Function nmea_get_rmc_time(void)
 * @note Must call nmea_rmc_parse continually to get this data
 * @return time */
long double nmea_get_rmc_time(void) {
    return rmc_time;
}

/**
 * @Function nmea_get_rmc_status(void)
 * @note Must call nmea_rmc_parse continually to get this data
 * @return status An 'A' means valid data, and a 'V' means data is INVALID */
char nmea_get_rmc_status(void) {
    return rmc_status;
}

/**
 * @Function nmea_get_rmc_lat_int(void)
 * @note Must call nmea_rmc_parse continually to get this data
 * @return lat */
int32_t nmea_get_rmc_lat_int(void) {
    return rmc_lat_int;
}

/**
 * @Function nmea_get_rmc_lat(void)
 * @note Must call nmea_rmc_parse continually to get this data
 * @return lat */
long double nmea_get_rmc_lat(void) {
    return rmc_lat;
}

/**
 * @Function nmea_get_rmc_NS(void)
 * @note Must call nmea_rmc_parse continually to get this data
 * @return NS */
char nmea_get_rmc_NS(void) {
    return rmc_NS;
}

/**
 * @Function nmea_get_rmc_long_int(void)
 * @note Must call nmea_rmc_parse continually to get this data
 * @return lat */
int32_t nmea_get_rmc_long_int(void) {
    return rmc_long_int;
}

/**
 * @Function nmea_get_rmc_long(void)
 * @note Must call nmea_rmc_parse continually to get this data
 * @return long */
long double nmea_get_rmc_long(void) {
    return rmc_long;
}

/**
 * @Function nmea_get_rmc_EW(void)
 * @note Must call nmea_rmc_parse continually to get this data
 * @return EW */
char nmea_get_rmc_EW(void) {
    return rmc_EW;
}

/**
 * @Function nmea_get_rmc_spd(void)
 * @note Must call nmea_rmc_parse continually to get this data
 * @return spd */
long double nmea_get_rmc_spd(void) {
    return rmc_spd;
}

/**
 * @Function nmea_get_rmc_cog(void)
 * @note Must call nmea_rmc_parse continually to get this data
 * @return cog in Degrees*/
long double nmea_get_rmc_cog(void) {

    return rmc_cog;
}

/**
 * @Function nmea_get_rmc_date(void)
 * @note Must call nmea_rmc_parse continually to get this data
 * @return date */
long double nmea_get_rmc_date(void) {
    return rmc_date;
}

/**
 * @Function nmea_get_rmc_mv(void)
 * @note Must call nmea_rmc_parse continually to get this data
 * @return mv */
long double nmea_get_rmc_mv(void) {
    return rmc_mv;
}

/**
 * @Function nmea_get_rmc_mvEW(void)
 * @note Must call nmea_rmc_parse continually to get this data
 * @return mvEW */
char nmea_get_rmc_mvEW(void) {
    return rmc_mvEW;
}

/**
 * @Function nmea_get_rmc_posMode(void)
 * @note Must call nmea_rmc_parse continually to get this data
 * @return posMode */
char nmea_get_rmc_posMode(void) {
    return rmc_posMode;
}

/**
 * @Function nmea_get_rmc_navStatus(void)
 * @note Must call nmea_rmc_parse continually to get this data
 * @return navStatus An 'A' means equipment is providing navigational status 
 * and that data is valid, a 'V' means it is NOT */
char nmea_get_rmc_navStatus(void) {
    return rmc_navStatus;
}

/**
 * @Function nmea_get_vtg_cogt(void)
 * @param nmea_frame_t *msg An already populated nmea message frame pointer.
 * Should call nmea_get_data() before using this function. 
 * @return course over ground in degrees 
 * @note 
 * $xxVTG,cogt,T,cogm,M,knots,N,kph,K,posMode*cs<CR><LF>*/
long double nmea_get_vtg_cogt(nmea_frame_t *msg) {
    const char delim = ',';
    char *token;
    token = strtok(msg->data, &delim);

    return (atof(token));
}

/**
 * @Function nmea_get_gsa_hdop(void)
 * @param nmea_frame_t *msg An already populated nmea message frame pointer.
 * Should call nmea_get_data() before using this function. 
 * @return Horizontal dilution of precision
 * @note 
 * $xxGSA,opMode,navMode{,sv},PDOP,HDOP,VDOP,systemId*cs<CR><LF>*/
long double nmea_get_gsa_hdop(nmea_frame_t *msg) {
    const char delim = ',';
    char *token;
    int i = 0;
    int j = 0;
    int comma_count = 0;
    long double result = 0.0;
    char hdop[MAX_FIELD_LEN];

    for (i = 0; i < sizeof (msg->data); i++) {
        if (comma_count == HDOP_N) {
            hdop[j] = msg->data[i];
            j++;
        }
        if (msg->data[i] == ',') {
            comma_count++;
        }
    }
    token = strtok(msg->data, &delim);
    token = strtok(NULL, &delim);
    if (*token != '1') { // If there is a fix (2 for 2D, 3 for 3D)
        result = atof(hdop);
    }
    return result;
}

void nmea_get_all_data(nmea_frame_t *cur_msg, int32_t *lat_i,
        long double *lat_f, int32_t *long_i, long double *long_f,
        long double *cog, long double *vel, long double *hdop) {

    // CHECK FOR XXRMC
    if ((nmea_read_head_address().talk_id[0] == 'G') &&
            (nmea_read_head_address().talk_id[1] == 'N') &&
            (nmea_read_head_address().s_frmtr[0] == 'R') &&
            (nmea_read_head_address().s_frmtr[1] == 'M') &&
            (nmea_read_head_address().s_frmtr[2] == 'C')) {
        if (nmea_get_data(&(cur_msg->data)) == SUCCESS) {

            nmea_rmc_parse2(cur_msg);

            *lat_i = nmea_get_rmc_lat_int();
            *lat_f = nmea_get_rmc_lat();
            *long_i = -nmea_get_rmc_long_int(); //@TODO: Fix sign
            *long_f = -nmea_get_rmc_long(); //@TODO: Fix sign
            *cog = nmea_get_rmc_cog();
            *vel = nmea_get_rmc_spd();
        }
    } else if ((nmea_read_head_address().talk_id[0] == 'G') &&
            (nmea_read_head_address().talk_id[1] == 'N') &&
            (nmea_read_head_address().s_frmtr[0] == 'G') &&
            (nmea_read_head_address().s_frmtr[1] == 'S') &&
            (nmea_read_head_address().s_frmtr[2] == 'A')) {
        if (nmea_get_data(&(cur_msg->data)) == SUCCESS) {
            *hdop = nmea_get_gsa_hdop(cur_msg);
        }
    }else {
        nmea_get_data(&(cur_msg->data)); // Call to remove msgs from buf
    }
}

/*******************************************************************************
 * PRIVATE FUNCTIONS                                                           *
 ******************************************************************************/

/**
 * @Function __ISR(_UART_2_VECTOR) uart_2_int_handler(void)
 * @param _UART_2_VECTOR Interrupt vector
 * @param IPL1SOFT Interrupt priority level and register selection 
 *  (SOFT, AUTO, SRS)
 * @return SUCCESS or ERROR */
void __ISR(_UART_2_VECTOR) uart_2_int_handler(void) {
    /* Check if the receive interrupt flag is high. If it is, clear the 
     * receive interrupt flag. */
    if (IFS1bits.U2RXIF == 1) {
        nmea_sm_store_msg(U2RXREG);
        IFS1bits.U2RXIF = 0; /* Not sure why, but putting this here works. 
                              * Normally, the flag should be cleared first */
    }
    /* Check if the transmit interrupt flag is high. If it is, clear the 
     * transmit interrupt flag, then check if the Transmit Shift Register 
     * is empty. */
    if (IFS1bits.U2TXIF == 1) {
        IFS1bits.U2TXIF = 0;
        if (getLength(transmitBuffer2) > 0) {
            if (!AddingToTransmit2) {
                U2TXREG = readFront(transmitBuffer2);
            } else { // There may have been a collision
                TransmitCollisionOccured2 = TRUE;
                IEC1bits.U2TXIE = 0; // Disable the TX Interrupt until next putchar()
                //LATDbits.LATD1 ^= 1; // Toggle IO pin for o-scope test
            }
        }
        if (getLength(transmitBuffer2) == 0) { //

            IEC1bits.U2TXIE = 0; // Disable the TX Interrupt until next putchar()
        }
    }
}

static void newCircBuffer(CBRef cB) {

    // initialize to zero
    int i;
    for (i = 0; i < QUEUESIZE; i++) {

        cB->buffer[i] = 0;
    }

    // initialize the data members
    cB->head = 0;
    cB->tail = 0;
    cB->size = QUEUESIZE;
    cB->overflowCount = 0;

}

// Accesor Methods
// ===============

// returns the amount of unread bytes in the circular buffer

static unsigned int getLength(CBRef cB) {
    // if the circular buffer is not null
    if (cB != NULL) {
        if (cB->head <= cB->tail) {
            return (cB->tail - cB->head);
        } else {
            return (cB->size + cB->tail - cB->head);
        }
    } else {

        return 0;
    }


}

// Manipulation Procedures
// ======================
// returns the front of the circular buffer and marks the byte as read

static unsigned char readFront(CBRef cB) {
    // if the circular buffer is not null
    if (cB != NULL) {
        char retVal;
        // if there are bytes in the buffer
        if (getLength(cB) > 0) {
            retVal = cB->buffer[cB->head];
            cB->head = cB->head < (cB->size - 1) ? cB->head + 1 : 0;

            return retVal;
        }
        return 128;
    }
    return 254;
}

/*******************************************************************************
 * MODULE UNIT TESTS                                                           *
 ******************************************************************************/

#ifdef CHECKSUM_TEST
#include "xc.h"
#include "board.h"
#include "serial.h"
#include <stdio.h>
#include <stdlib.h>

int main(void) {
    board_init();
    serial_init(); // UART1 

    char gps_message[] = "$GNRMC,,V,,,,,,,,,,N*4D";
    char check_result = 0;

    printf("Checksum Test\r\n");
    printf("Example NMEA 0183 Version 4.0 message:\r\n");
    printf("%s\r\n", gps_message);

    int i = 0;
    for (i = 1; gps_message[i] != '*'; i++) {
        check_result = nmea_checksum_iterate(gps_message[i], check_result);
    }

    printf("Checksum = %x\r\n", check_result);

    while (1);

    return 1;
}
#endif

#ifdef NMEA_STATE_MACHINE
#include "xc.h"
#include "board.h"
#include "serial.h"
#include <stdio.h>

int main(void) {
    board_init();
    serial_init(); // UART1 

    TRISCbits.TRISC1 = 0; /* LED5 */
    TRISAbits.TRISA3 = 0; /* Set pin as output. This is also LED4 on Max32 */
    LATCbits.LATC1 = 0; /* Set LED5 low */
    LATAbits.LATA3 = 0; /* Set LED4 low */

    char gps_message[] = "$GNRMC,,V,,,,,,,,,,N*4D";
    //char gps_message[] = "$GNGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99*2E";
    //char gps_message[] = "$GPZDA,141644.00,22,03,2002,00,00*67";
    unsigned char check_result = 0;
    unsigned char gps_ascii_checksum[CHECKSUM_LEN];

    int i = 0;

    for (i = 0; gps_message[i] != '*'; i++);
    gps_ascii_checksum[0] = gps_message[i + 1];
    gps_ascii_checksum[1] = gps_message[i + 2];


    printf("Checksum Test\r\n");
    printf("Example NMEA 0183 Version 4.0 message:\r\n");
    printf("%s\r\n", gps_message);

    for (i = 0; i < sizeof (gps_message); i++) {
        nmea_sm_store_msg(gps_message[i]);
    }

    nmea_frame_t cur_msg;
    memset(&cur_msg, 0, sizeof (cur_msg));

    if (nmea_is_msg_available() == TRUE) {

        printf("\r\n");
        printf("\r\n");
        printf("%s", nmea_read_head_address().talk_id);
        printf("\r\n");
        printf("%s", nmea_read_head_address().s_frmtr);
        printf("\r\n");

        if (nmea_get_data(&(cur_msg.data)) == SUCCESS) {
            printf("%s", cur_msg.data);
            printf(" %d ", crx2);
            printf(" %d ", crx3);
            printf("\r\n");
            printf("\r\n");
            memset(&(cur_msg.data), 0, sizeof (cur_msg.data));
        }

    }

    check_result = nmea_ascii_to_hex(gps_ascii_checksum);

    printf("ASCII to HEX Convert checksum:  %x\r\n", check_result);

    while (1);

    return 1;
}
#endif

#ifdef READ_NMEA_STREAM
#include "xc.h"
#include "board.h"
#include "serial.h"
#include "nmea0183v4.h"
#include <stdio.h>
#include <string.h>

int main(void) {
    board_init();
    serial_init(); // UART1 

    printf("TEST\r\n");

    nmea_serial_init();

    TRISCbits.TRISC1 = 0; /* LED5 */
    TRISAbits.TRISA3 = 0; /* Set pin as output. This is also LED4 on Max32 */
    LATCbits.LATC1 = 0; /* Set LED5 low */
    LATAbits.LATA3 = 0; /* Set LED4 low */

    printf("READ NMEA STREAM TEST\r\n");

    nmea_frame_t cur_msg;
    memset(&cur_msg, 0, sizeof (cur_msg));

    while (1) {
        if (nmea_is_msg_available() == TRUE) {
            printf("\r\n");
            printf("%c%c",
                    nmea_read_head_address().talk_id[0],
                    nmea_read_head_address().talk_id[1]);
            printf("%c%c%c",
                    nmea_read_head_address().s_frmtr[0],
                    nmea_read_head_address().s_frmtr[1],
                    nmea_read_head_address().s_frmtr[2]);
            if (nmea_get_data(&(cur_msg.data)) == SUCCESS) {
                printf("%s", cur_msg.data);
                printf(" %x ", crx2);
                printf(" %x ", crx3);
                printf("\r\n");
                memset(&(cur_msg.data), 0, sizeof (cur_msg.data));
            }

        }
    }

    while (1);

    return 1;
}
#endif

#ifdef READ_STANDARD_MSGS
#include "xc.h"
#include "board.h"
#include "serial.h"
#include "nmea0183v4.h"
#include <stdio.h>
#include <string.h>

int main(void) {
    board_init();
    serial_init(); // UART1 

    printf("TEST\r\n");

    nmea_serial_init();

    printf("READ GNVTG DATA\r\n");

    nmea_frame_t cur_msg;
    memset(&cur_msg, 0, sizeof (cur_msg));

    int i = 0;

    // XXRMC
    // Form:
    // $xxGGA,time,lat,NS,long,EW,quality,numSV,HDOP,alt,M,sep,M,diffAge,diffStation*cs<CR><LF>
    // Ex:
    // $GPRMC,083559.00,A,4717.11437,N,00833.91522,E,0.004,77.52,091202,,,A,V*57
    char example_data1[] = ",083559.00,A,4717.11437,N,00833.91522,E,0.004,77.52,091202,,,A,V*57";
    //char example_data1[] = ",213154.00,A,3658.56019,N,12201.72543,W,0.020,,190120,,,D306,M,,0000";
    for (i = 0; i < sizeof (example_data1); i++) {
        cur_msg.data[i] = example_data1[i];
    }
    printf("Example data: %s\r\n", cur_msg.data);
    nmea_rmc_parse(&cur_msg);
    printf("After RMC parsing:\r\n");
    printf("TIME: %lf\r\n", nmea_get_rmc_time());
    printf("status: %c\r\n", nmea_get_rmc_status());
    printf("lat: %lf\r\n", nmea_get_rmc_lat());
    printf("NS: %c\r\n", nmea_get_rmc_NS());
    printf("long: %lf\r\n", nmea_get_rmc_long());
    printf("EW: %c\r\n", nmea_get_rmc_EW());
    printf("spd: %lf\r\n", nmea_get_rmc_spd());
    printf("cog: %lf\r\n", nmea_get_rmc_cog());
    printf("date: %lf\r\n", nmea_get_rmc_date());
    // printf("mv: %.5f\r\n", nmea_get_rmc_mv());
    // printf("mvEW: %c\r\n", nmea_get_rmc_mvEW());
    printf("posMode: %c\r\n", nmea_get_rmc_posMode());
    printf("navStatus: %c\r\n\r\n", nmea_get_rmc_navStatus());
    memset(&cur_msg, 0, sizeof (cur_msg));

    // XXVTG
    // Form:
    // $xxVTG,cogt,T,cogm,M,knots,N,kph,K,posMode*cs<CR><LF>
    // Ex:
    // $GPVTG,77.52,T,,M,0.004,N,0.008,K,A*06
    char example_data2[] = "77.52,T,,M,0.004,N,0.008,K,A*06";
    for (i = 0; i < sizeof (example_data2); i++) {
        cur_msg.data[i] = example_data2[i];
    }
    printf("Example data: %s\r\n", cur_msg.data);
    printf("XXVTG COGT: %lf\r\n", nmea_get_vtg_cogt(&cur_msg));
    memset(&cur_msg, 0, sizeof (cur_msg));

    // XXGSA
    // Form:
    // $xxGSA,opMode,navMode{,sv},PDOP,HDOP,VDOP,systemId*cs<CR><LF>
    // Ex:
    // $GPGSA,A,3,23,29,07,08,09,18,26,28,,,,,1.94,1.18,1.54,1*0D
    char example_data3[] = "A,3,23,29,07,08,09,18,26,28,,,,,1.94,1.18,1.54,1*0D";
    for (i = 0; i < sizeof (example_data3); i++) {
        cur_msg.data[i] = example_data3[i];
    }
    printf("Example data: %s\r\n", cur_msg.data);
    printf("XXGSA HDOP: %lf\r\n", nmea_get_gsa_hdop(&cur_msg));
    memset(&cur_msg, 0, sizeof (cur_msg));

    while (1) {
        if (nmea_is_msg_available() == TRUE) {

            // CHECK FOR XXRMC
            if ((nmea_read_head_address().talk_id[0] == 'G') &&
                    (nmea_read_head_address().talk_id[1] == 'N') &&
                    (nmea_read_head_address().s_frmtr[0] == 'R') &&
                    (nmea_read_head_address().s_frmtr[1] == 'M') &&
                    (nmea_read_head_address().s_frmtr[2] == 'C')) {
                if (nmea_get_data(&(cur_msg.data)) == SUCCESS) {
                    printf("\r\n");
                    printf("Msg data: %s\r\n", cur_msg.data);
                    nmea_rmc_parse(&cur_msg);
                    printf("After RMC parsing:\r\n");
                    printf("TIME: %lf\r\n", nmea_get_rmc_time());
                    printf("status: %c\r\n", nmea_get_rmc_status());
                    printf("lat: %lf\r\n", nmea_get_rmc_lat());
                    printf("NS: %c\r\n", nmea_get_rmc_NS());
                    printf("long: %lf\r\n", nmea_get_rmc_long());
                    printf("EW: %c\r\n", nmea_get_rmc_EW());
                    printf("spd: %lf\r\n", nmea_get_rmc_spd());
                    printf("cog: %lf\r\n", nmea_get_rmc_cog());
                    printf("date: %lf\r\n", nmea_get_rmc_date());
                    // printf("mv: %.5f\r\n", nmea_get_rmc_mv());
                    // printf("mvEW: %c\r\n", nmea_get_rmc_mvEW());
                    printf("posMode: %c\r\n", nmea_get_rmc_posMode());
                    printf("navStatus: %c\r\n", nmea_get_rmc_navStatus());
                }

                // CHECK FOR XXVTG
            } else if ((nmea_read_head_address().talk_id[0] == 'G') &&
                    (nmea_read_head_address().talk_id[1] == 'N') &&
                    (nmea_read_head_address().s_frmtr[0] == 'V') &&
                    (nmea_read_head_address().s_frmtr[1] == 'T') &&
                    (nmea_read_head_address().s_frmtr[2] == 'G')) {
                if (nmea_get_data(&(cur_msg.data)) == SUCCESS) {
                    printf("\r\n");
                    printf("COGT: %lf\r\n", nmea_get_vtg_cogt(&cur_msg));
                }
                // CHECK FOR XXGSA
            } else if ((nmea_read_head_address().talk_id[0] == 'G') &&
                    (nmea_read_head_address().talk_id[1] == 'N') &&
                    (nmea_read_head_address().s_frmtr[0] == 'G') &&
                    (nmea_read_head_address().s_frmtr[1] == 'S') &&
                    (nmea_read_head_address().s_frmtr[2] == 'A')) {
                if (nmea_get_data(&(cur_msg.data)) == SUCCESS) {
                    printf("\r\n");
                    printf("HDOP: %lf\r\n", nmea_get_gsa_hdop(&cur_msg));
                }
            } else {
                nmea_get_data(&(cur_msg.data)); // Call to remove msgs from buf
            }
        }
    }

    while (1);

    return 1;
}
#endif

#ifdef READ_RMC_CSV
#include "xc.h"
#include "board.h"
#include "serial.h"
#include "nmea0183v4.h"
#include <stdio.h>
#include <string.h>

int main(void) {
    board_init();
    serial_init(); // UART1 

    printf("%% TEST\r\n");

    nmea_serial_init();

    TRISCbits.TRISC1 = 0; /* LED5 */
    TRISAbits.TRISA3 = 0; /* Set pin as output. This is also LED4 on Max32 */
    LATCbits.LATC1 = 0; /* Set LED5 low */
    LATAbits.LATA3 = 0; /* Set LED4 low */

    printf("%% READ NMEA STREAM TEST\r\n");

    nmea_frame_t cur_msg;
    memset(&cur_msg, 0, sizeof (cur_msg));

    while (1) {
        if (nmea_is_msg_available() == TRUE) {

            // CHECK FOR XXRMC
            if ((nmea_read_head_address().talk_id[0] == 'G') &&
                    (nmea_read_head_address().talk_id[1] == 'N') &&
                    (nmea_read_head_address().s_frmtr[0] == 'R') &&
                    (nmea_read_head_address().s_frmtr[1] == 'M') &&
                    (nmea_read_head_address().s_frmtr[2] == 'C')) {
                if (nmea_get_data(&(cur_msg.data)) == SUCCESS) {
                    printf("%s\r\n", cur_msg.data);
                    memset(&(cur_msg.data), 0, sizeof (cur_msg.data));
                }
            } else {
                nmea_get_data(&(cur_msg.data)); // Call to remove msgs from buf
            }
        }
    }

    while (1);

    return 1;
}
#endif