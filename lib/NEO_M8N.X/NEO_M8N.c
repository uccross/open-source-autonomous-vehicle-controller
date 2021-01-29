/* 
 * File:   NEO_M8N.c
 * Author: Aaron Hunter
 * Brief: Library driver for ublox NEO-M8N GPS receiver
 * Created on 12/09/2020
 * Modified 
 */

/*******************************************************************************
 * #INCLUDES                                                                   *
 ******************************************************************************/

#include "NEO_M8N.h" // The header file for this source file. 
#include "Board.h"   //Max32 setup
#include "SerialM32.h"
#include "xc.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/attribs.h>
#include <proc/p32mx795f512l.h>

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/
#define BUFFER_LENGTH 512
#define GPS_BUFFERSIZE 5  // how many NMEA msgs the buffer stores
#define DEG2RAD M_PI/180.0
#define RMC_ID "RMC"
#define HOURS2SEC 3600
#define MIN2SEC 60
#define SHIFT4  10000
#define SHIFT2  100
#define GPS_ENTRIES 4
#define GPS_PAYLOADLENGTH 128 
#define GPS_HEAD 0X24   //ASCII $
#define GPS_TAIL 0X2A   //ASCII *
//#define GPS_BAUD_RATE 9600
#define GPS_BAUD_RATE 115200

/*******************************************************************************
 * PRIVATE TYPEDEFS                                                            *
 ******************************************************************************/
typedef struct circular_buffer {
    int read_index;
    int write_index;
    int size;
    unsigned char data[BUFFER_LENGTH];
} circular_buffer; /*circular  buffer */

/*Buffer to place received messages*/
typedef struct GPS_msg_buffer {
    unsigned char payload[GPS_BUFFERSIZE][GPS_PAYLOADLENGTH];
    unsigned char length[GPS_BUFFERSIZE];
    unsigned char checksum[GPS_BUFFERSIZE];
    int write_index;
    int read_index;
} GPS_msg_buffer;

typedef enum {
    WAITING_FOR_HEAD,
    GET_PAYLOAD,
    GET_CHECKSUM,
    GETCRLF,
} state_t;

struct RMC_frame {
    char messageID[7];
    double time; //stored in seconds, UTC time
    char status[2];
    double lat;
    char NS[2];
    double lon;
    char EW[2];
    float speed;
    float cog;
    char date[7];
    float mv;
    char mvEW[2];
    char posMode[2];
    char cs[3];
} RMC = {
    .messageID = "$GNRMC",
    .time = 0.0,
    .status = "V",
    .lat = 0.0,
    .NS = "N",
    .lon = 0.0,
    .EW = "E",
    .speed = 0.0,
    .date = "010120",
    .mv = 0.0,
    .mvEW = "E",
    .posMode = "V",
    .cs = "00"
};

typedef enum {
    ID, GPSTIME, STATUS, LAT, NS, LON, EW, SPD, COG, DATE, MV, MVEW, POSMODE, CS
} RMC_frame_t;

/*******************************************************************************
 * PRIVATE VARIABLES                                                            *
 ******************************************************************************/
static struct circular_buffer U2_RX_buffer; //circular buffer for GPS strings
static struct circular_buffer *rxp = &U2_RX_buffer; // pointer to U2_RX_buffer
static struct GPS_msg_buffer incoming_msg_buffer;
static struct GPS_msg_buffer *msg_buffer_p = &incoming_msg_buffer;
static unsigned char reading_from_RX_buffer = FALSE;
static unsigned char RX_collision = FALSE;
static double NMEA_latitude = 0.0;
static double NMEA_longitude = 0.0;
static double NMEA_time = 0.0;
struct GPS_data RMC_data = {
    .time = 0.0,
    .lat = 0.0,
    .lon = 0.0,
    .spd = 0.0,
    .cog = 0.0
};
static uint8_t is_data_valid = FALSE;
static uint8_t is_data_new = FALSE;

/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                 *
 ******************************************************************************/
/**
 * @Function UART2_interrupt_handler(void)
 * @brief Handles the UART interrupts
 * @note 
 * @author Aaron Hunter
 * @modified  */
static void __ISR(_UART_2_VECTOR, IPL3SOFT) UART2_interrupt_handler(void);
/**
 * @Function void init_buffer(struct circular_buffer *buf);
 * @brief Initializes a new circular buffer
 * @note 
 * @author Aaron Hunter
 * @modified  */
static void init_buffer(struct circular_buffer *buf);

/**
 * @Function void initPacketBuffer(struct packetBuffer *buf)
 * @brief Initializes a new packet buffer
 * @note 
 * @author Aaron Hunter
 * @modified  */
static void init_msg_buffer(struct GPS_msg_buffer *buf);

/**
 * @Function void GPS_run_RX_state_machine(unsigned char charIn)
 * @param charIn, next character to process
 * @return None
 * @brief Runs the protocol state machine for receiving characters, it should be called from 
 * within the interrupt and process the current character
 * @author Hunter */
static void GPS_run_RX_state_machine(unsigned char charIn);

/**
 * @Function char GPS_is_queue_full(void)
 * @param None
 * @return TRUE is QUEUE is Full
 * @brief 
 * @author Aaron Hunter */
char GPS_is_queue_full(void);

/**
 * @Function char GPS_IsQueueEmpty(void)
 * @param None
 * @return TRUE is data table is empty
 * @brief 
 * @author Aaron Hunter */
char GPS_IsQueueEmpty(void);

/**
 * @Function void GPS_store_msg(&payload);
 * @return 
 */
static int GPS_store_msg(unsigned char *payload, int length, unsigned char checksum);

/**
 * @Function unsigned char ascii2hex(char c);
 * converts a single char into its numeric representation
 * @return 
 */
static unsigned char ascii2hex(char c);

/**
 * @Function NMEA_parse(char* sentence)
 * @param sentence, NMEA sentence string
 * @return SUCCESS or ERROR
 * @brief 
 * @note 
 * @author Aaron Hunter,
 * @modified */
static int NMEA_parse(char* sentence);

/**
 * @Function int RMC_parse(char* messageID, char* sentence)
 * @param RMC sentence, message ID (RMC)
 * @return SUCCESS or ERROR
 * @brief 
 * @note only called if messageID = xxxRMC
 * @author Aaron Hunter,
 * @modified */
static int RMC_parse(char* messageID, char* sentence);

/*
 * Function char RMC_storeData(void);
 * brief stores RMC data into GPS_dataTable
 * returns SUCCESS or ERROR
 * author Aaron Hunter
 */
static char RMC_storeData(void);

/*******************************************************************************
 * PUBLIC FUNCTION IMPLEMENTATIONS                                             *
 ******************************************************************************/

/**
 * @Function GPS_Init(void)
 * @param None
 * @return SUCCESS or ERROR
 * @brief sets up UART2 for communication and initializes GPS buffer
 * @author Aaron Hunter */
int GPS_init(void) {
    /*Initialize the circular buffers*/
    init_buffer(rxp);
    init_msg_buffer(msg_buffer_p);

    __builtin_disable_interrupts();
    U2MODEbits.UEN = 0; // TX/RX enabled, configure using software flow control
    U2MODEbits.PDSEL = 0; //8 bits no parity
    U2MODEbits.STSEL = 0; //1 stop bit
    /* configure the interrupts*/
    U2STAbits.UTXISEL = 1; // int when all characters are sent (TRMT == TRUE)
    U2STAbits.URXISEL = 0; // interrupt while buffer is not empty

    U2BRG = ((Board_get_PB_clock() / GPS_BAUD_RATE) / 16) - 1; //set baud rate

    IFS1bits.U2RXIF = 0; //clear interrupt flags
    IFS1bits.U2TXIF = 0;
    IEC1bits.U2RXIE = 1; //enable interrupt on RX
    IEC1bits.U2TXIE = 1; //enable interrupts on TX

    IPC8bits.U2IP = 3; //set interrupt priority to 3
    U2STAbits.UTXEN = 0; // TX disabled--at this point we're not writing to GPS
    U2STAbits.URXEN = 1; // RX enabled
    //turn on UART
    U2MODEbits.ON = 1;
    __builtin_enable_interrupts();

    return SUCCESS;
}

/**
 * @Function GPS_IsMessageAvailable(void)
 * @param None
 * @return TRUE if Queue is not Empty
 * @brief 
 * @author Aaron Hunter */
char GPS_is_msg_avail(void) {
    {
        struct GPS_msg_buffer *buf = msg_buffer_p;
        if (buf->read_index == buf->write_index) { //if read = write then the buffer is empty
            return FALSE;
        }
        return TRUE;
    }
}


/**
 * @Function char GPS_parseStream(void)
 * @return SUCCESS or ERROR
 * @brief 
 * @author Aaron Hunter */
char GPS_parse_stream(void) {
    struct GPS_msg_buffer *buf = msg_buffer_p;

    if (GPS_is_msg_avail() == TRUE) {
        NMEA_parse(buf->payload[buf->read_index]);
        /*increment and wrap read index*/
        buf->read_index = (buf->read_index + 1) % GPS_BUFFERSIZE;
        return SUCCESS;
    } else return ERROR;
}

/**
 * @Function GPS_is_data_available(void)
 * @param None
 * @return TRUE if GPS data has not been read
 * @brief helper function to signal when there is unread data
 * @author Aaron Hunter */
char GPS_is_data_avail(void) {
    return is_data_new; // status of data if false then no new data
}

/*
 *Function char GPS_has_fix(void);
 * return TRUE if GPS data is valid
 * author:  Aaron Hunter
 */
char GPS_has_fix(void) {
    return is_data_valid; //TRUE if status field != "V"
}

/**
 * Function GPS_getData(struct * GPS_data)
 * param *GPS  pointer to GPS_data struct
 * return SUCCESS or  ERROR
 * brief:  grabs data at the read index of the data table and stores the values
 * in GPS
 * author: Aaron Hunter
 */
char GPS_get_data(struct GPS_data * data) {

    data->time = RMC_data.time;
    data->lat = RMC_data.lat;
    data->lon = RMC_data.lon;
    data->spd = RMC_data.spd;
    data->cog = RMC_data.cog;
    is_data_new = FALSE; // change flag to indicate that data has been read
    return SUCCESS;
}

/*******************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATIONS                                            *
 ******************************************************************************/

/**
 * @Function IntUart2Handler(void)
 * @brief Handles the UART interrupts
 * @note 
 * @author Aaron Hunter
 * @modified  */
void __ISR(_UART_2_VECTOR, IPL3AUTO) UART2_interrupt_handler(void) {
    unsigned char charIn;
    if (IFS1bits.U2RXIF) { //check for received data flag
        IFS1bits.U2RXIF = 0; //clear the flag
        unsigned char charIn = U2RXREG;
        /*run the state machine with the new character from the RX buffer*/
        if (reading_from_RX_buffer == FALSE) {
            GPS_run_RX_state_machine(U2RXREG);
        } else {
            RX_collision = TRUE;
        }
    }
    if (IFS1bits.U2TXIF) { /*check for transmission flag*/
        IFS1bits.U2TXIF = 0; /*clear the flag*/
        /*handler from Protocol lib for reference*/
        //        if (bufEmpty(txp) == FALSE) {
        //            if (writingToTXBuffer == FALSE) { /*If not being accessed */
        //                U1TXREG = readBuffer(txp);
        //            } else {
        //                txCollision = TRUE;
        //            }
        //        }
    }
    if (IFS1bits.U2EIF) { //error flag--can check U1STA for the reason
        IFS1bits.U2EIF = 0; /* unhandled, just clear */
    }
}

/**
 * @Function void initBuffer(struct oBuffer *buf)
 * @brief Initializes a new circular buffer
 * @note 
 * @author Aaron Hunter
 * @modified  */
void init_buffer(struct circular_buffer * buf) {
    int i;
    buf->read_index = 0; /*initialize read index to 0 */
    buf->write_index = 0; /*initialize write index to 0 */
    buf->size = BUFFER_LENGTH; /*Set size to buffer length const*/
    for (i = 0; i < BUFFER_LENGTH; i++) { /*initialize data to zero*/
        buf->data[i] = 0;
    } /*end for */
}

/**
 * @Function void initPacketBuffer(struct packetBuffer *buf)
 * @brief Initializes a new packet buffer
 * @note 
 * @author Aaron Hunter
 * @modified  */
void init_msg_buffer(struct GPS_msg_buffer * buf) {
    int i;
    int j;
    buf->read_index = 0; /*initialize read index to 0 */
    buf->write_index = 0; /*initialize write index to 0 */

    for (i = 0; i < GPS_BUFFERSIZE; i++) { /*initialize data to zero*/
        buf->length[j] = 0;
        buf->checksum[j] = 0;
        for (j = 0; j < GPS_PAYLOADLENGTH; j++) {
            buf->payload[i][j] = 0;
        } /*end for j */
    } /*end for i */
}

/**
 * @Function void GPS_run_RX_state_machine(unsigned char char_in)
 * @param char_in, next character to process
 * @return None
 * @brief Runs the protocol state machine for receiving characters, it should be called from 
 * within the interrupt and process the current character
 * @author aaron Hunter */
void GPS_run_RX_state_machine(unsigned char char_in) {
    static state_t current_state = WAITING_FOR_HEAD;
    static state_t next_state = WAITING_FOR_HEAD;
    static unsigned char index = 0; //payload index
    static unsigned char cksum_index = 0; //index of checksum
    static unsigned char payload[GPS_PAYLOADLENGTH];
    static unsigned char checksum = 0;
    static unsigned char cksum_string[2]; //string to compare against received checksum value
    int cksum_calc = 0; //calculated checksum derived from cksum_string above

    switch (current_state) {
        case WAITING_FOR_HEAD:
        {
            index = 0;
            if (char_in == GPS_HEAD) {
                next_state = GET_PAYLOAD;
                payload[index] = char_in;
                checksum = 0;
            } else {
                next_state = WAITING_FOR_HEAD;
            }
            break;
        }

        case GET_PAYLOAD:
        {
            index++;
            payload[index] = char_in;
            if (char_in == GPS_TAIL) {
                next_state = GET_CHECKSUM;
                cksum_index = 0;
            } else {
                next_state = GET_PAYLOAD;
                checksum ^= char_in;
            }
            break;
        }
        case GET_CHECKSUM:
        {
            index++;
            payload[index] = char_in;
            cksum_string[cksum_index] = char_in;
            if (cksum_index == 1) {
                next_state = WAITING_FOR_HEAD;
                /*convert checksum string to hex*/
                cksum_calc = ((int) ascii2hex(cksum_string[0]) << 4) + ascii2hex(cksum_string[1]);
                cksum_index++;
                cksum_string[cksum_index] = '\0'; //terminate  string
                /*compare to checksum*/
                if (checksum == cksum_calc) {
                    /*store GPS message only if checksum matches*/
                    GPS_store_msg(payload, index + 1, checksum); //TODO remove checksum storage from message buffer
                }
            } else next_state = GET_CHECKSUM;
            cksum_index++;
            break;
        }
    }
    current_state = next_state; /* update the state for the next event */
}

/**
 * @Function GPS_store_msg(unsigned char *payload, int length, unsigned char checksum)
 * @param *payload, pointer to a char array with the payload  data
 * @param length, length of payload
 * @return SUCCESS or ERROR
 * @brief stores the payload of an incoming message into the packet buffer
 * @author Aaron Hunter */
int GPS_store_msg(unsigned char *payload, int length, unsigned char checksum) {
    int i;
    struct GPS_msg_buffer *buf = msg_buffer_p;

    if (GPS_is_queue_full() == FALSE) { /*If the packet buffer is not full */
        reading_from_RX_buffer = TRUE;
        buf->length[buf->write_index] = length; /*write length to the bufferLength at the writeIndex */
        for (i = 0; i < length; i++) {
            buf->payload[buf->write_index][i] = payload[i]; //write payload data
        }
        /*NULL terminate the string*/
        buf->payload[buf->write_index][length] = '\0';
        reading_from_RX_buffer = FALSE;
        buf->checksum[buf->write_index] = checksum;
        buf->write_index = (buf->write_index + 1) % GPS_BUFFERSIZE; //increment and wrap
        if (RX_collision == TRUE) {
            IFS1bits.U2RXIF = 1; /*reset interrupt for RX*/
            RX_collision = FALSE; /* reset collision flag */
        }
        return SUCCESS;
    } else return ERROR;
}

/**
 * @Function char GPS_is_queue_full(void)
 * @param None
 * @return TRUE is QUEUE is Full
 * @brief 
 * @author Aaron Hunter */
char GPS_is_queue_full(void) {
    struct GPS_msg_buffer *buf = msg_buffer_p;
    if ((buf->write_index + 1) % GPS_BUFFERSIZE == buf->read_index) {
        return TRUE;
    }
    return FALSE;
}

/**
 * @Function unsigned char ascii2hex(char c);
 * converts a single char into its numeric representation
 * @return 
 */
static unsigned char ascii2hex(char c) {
    if (c >= '0' & c <= '9') c = c - '0';
    else if (c >= 'A' & c <= 'F') c = c - 'A' + 10;
    else if (c >= 'a' & c <= 'f') c = c - 'a' + 10;
    else c = 0; //error
    return c;
}

static int NMEA_parse(char* sentence) {
    char* message_ID;
    char* nextField;

    message_ID = sentence;
    /*Sentence ID is the first field and delimited with a ',' */
    nextField = strchr(message_ID, ',');
    *nextField = '\0';
    nextField++;
    /*parse RMC message*/
    if (strcmp(message_ID + 3, RMC_ID) == 0) {
        if (RMC_parse(message_ID, nextField) == SUCCESS) {
            return SUCCESS;
        } else {
            return ERROR;
        }
    }
    /*otherwise don't parse the message */
    //    printf("Message ID: %s not handled", message_ID);
    return SUCCESS;
}

/**
 * @Function RMC_(char* sentence)
 * @param sentence, RMC sentence string without message ID
 * @return SUCCESS or ERROR
 * @brief 
 * @note 
 * @author Aaron Hunter,
 * @modified */
static int RMC_parse(char* messageID, char* sentence) {
    char* field;
    char* nextField;
    double deg; // degrees
    double fdeg; //fractional degrees
    double float_minutes;
    double time;
    int val;
    RMC_frame_t index = GPSTIME;

    /*msgID is RMC, first 3 chars show which satellite network in use*/
    strcpy(RMC.messageID, messageID);
    field = sentence;
    /*$GNRMC,212713.00,A,3657.62313,N,12201.97543,W,0.038,,100820,,,D*73*/
    /*note the while conditions imply a single use of an asterisk as a field delimeter*/
    while ((nextField = strchr(field, ',')) != NULL || (nextField = strchr(field, '*')) != NULL) {
        /*null terminate the field to use string functionality*/
        *nextField = '\0';
        nextField++;

        switch (index) {
            case GPSTIME:
                if (strlen(field) >= 9) {
                    val = (atoi(field) / SHIFT4) * HOURS2SEC; //get hours->seconds
                    val += (atoi(field + 2) / SHIFT2) * MIN2SEC; //get minutes->seconds
                    time = (double) val + atof(field + 4); // calculate total seconds
                    RMC.time = time;
                    NMEA_time = time;
                }
                break;
            case STATUS:
                strncpy(RMC.status, field, sizeof (RMC.status));
                if (RMC.status == "V") {
                    is_data_valid = FALSE;
                } else {
                    is_data_valid = TRUE;
                }
                break;
            case LAT:
                /*lat has following format: ddmm.mmmmm*/
                deg = (double) (atoi(field) / 100); //get degrees -  minutes
                float_minutes = atof(field + 2); //minutes are the remainder of the string
                fdeg = float_minutes / 60.0; //scale to degrees (fractional part of degrees)
                RMC.lat = deg + fdeg;
                break;
            case NS:
                strncpy(RMC.NS, field, sizeof (RMC.NS));
                if (RMC.NS[0] == 'S') {
                    NMEA_latitude = -RMC.lat;
                } else {
                    NMEA_latitude = RMC.lat;
                }
                break;
            case LON:
                // lon has following format: ddddmm.mmmmm
                deg = (double) (atoi(field) / 100); //get degrees - minutes
                float_minutes = atof(field + 3); //minutes are the remainder of the string
                fdeg = float_minutes / 60.0; //scale to degrees (fractional part of degrees)
                RMC.lon = deg + fdeg;
                break;
            case EW:
                strncpy(RMC.EW, field, sizeof (RMC.EW));
                if (RMC.EW[0] == 'W') {
                    NMEA_longitude = -RMC.lon;
                } else {
                    NMEA_longitude = RMC.lon;
                }
                break;
            case SPD:
                RMC.speed = atof(field);
                break;
            case COG:
                RMC.cog = atof(field);
                break;
            case DATE:
                strncpy(RMC.date, field, sizeof (RMC.date));
                break;
            case MV:
                RMC.mv = atof(field);
                break;
            case MVEW:
                strncpy(RMC.mvEW, field, sizeof (RMC.mvEW));
                break;
            case POSMODE:
                strncpy(RMC.posMode, field, sizeof (RMC.posMode));
                break;
            default:
                break;
        }
        field = nextField;
        index++;
    }
    /*last field is always the checksum*/
    strncpy(RMC.cs, field, sizeof (RMC.cs)); //field is not NULL terminated, so need to copy only 2 chars
    RMC_storeData(); //update GPS_dataTable
    return SUCCESS;
}

/*
 * Function char RMC_storeData(void);
 * brief stores RMC data into GPS_dataTable
 * returns SUCCESS or ERROR
 * author Aaron Hunter
 */
static char RMC_storeData(void) {

    RMC_data.time = RMC.time;
    RMC_data.lat = NMEA_latitude; //these two account for sign (N,E = +)
    RMC_data.lon = NMEA_longitude;
    RMC_data.cog = RMC.cog;
    RMC_data.spd = RMC.speed;
    is_data_new = TRUE; //set flag to indicate that there is unread data
    return SUCCESS;
}




#ifdef GPS_TESTING

void main(void) {
    struct GPS_data data;
    Board_init();
    Serial_init();
    GPS_init();

    printf("GPS Test Harness, %s, %s", __DATE__, __TIME__);

    while (1) {

        if (GPS_is_msg_avail() == TRUE) {
            GPS_parse_stream();
        }

        if (GPS_is_data_avail() == TRUE) {
            if (is_data_valid == TRUE) {
                GPS_get_data(&data);
                if (GPS_is_data_avail() == TRUE) { //this should now be false
                    printf("Error in setting is_data_new flag");
                }
                /*print current data in degrees to output*/
                printf("time: %.2f, location %0.6f, %0.6f, speed %3.3f, dir"
                        " %0.6f\r", data.time, data.lat,
                        data.lon, data.spd, data.cog);
            } else {
                printf("Data is not valid");
            }
        }
    }

}
#endif //GPS_TESTING

