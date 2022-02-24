/* 
 * File:   RC_RX.h
 * Author: Aaron Hunter
 * Brief: Source file for the radio control receiver used for input to the 
 * vehicle.  
 * Created on Dec 22,2020 9:42 am
 * Modified on 
 */

/*******************************************************************************
 * #INCLUDES                                                                   *
 ******************************************************************************/

#include "RC_RX.h" // The header file for this source file. 
#include "SerialM32.h" // The header file for this source file. 
#include "Board.H"   //Max32 setup      
#include <xc.h>
#include <stdio.h>
#include <sys/attribs.h>  //for ISR definitions
#include <xc.h>

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/
#define RX_BAUD 100000 //100 kBaud
#define SBUS_BUFFER_LENGTH 25 //bytes 0-24
#define RX_BUFFER_LENGTH 128
#define RX_NUM_MSGS 2
#define START_BYTE 0x0F
#define END_BYTE 0x00

/*******************************************************************************
 * PRIVATE TYPEDEFS                                                            *
 ******************************************************************************/

struct RCRX_msg_buffer {
    uint8_t read_index;
    uint8_t write_index;
    uint8_t sbus_buffer[RX_NUM_MSGS][SBUS_BUFFER_LENGTH]; //raw data
};
struct RCRX_msg_buffer RCRX_msgs;
struct RCRX_msg_buffer* RCRX_buf_p = &RCRX_msgs;

static int8_t parsing_RX = FALSE;
static int8_t RX_collision = FALSE;
static volatile int8_t new_data_avail = FALSE;
static int8_t parse_error = FALSE;
static unsigned int parse_error_counter = 0;

//typedef enum {
//    WAIT_FOR_START,
//    GET_DATA,
//    GET_END,
//} RCRX_state_t;

typedef enum {
    WAIT_FIRST,
    GET_SECOND,
    WAIT_SYNC,
    GET_START,
    GET_DATA,
} RCRX_state_t;

/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                 *
 ******************************************************************************/

/**
 * @Function RCRX_init_msg_buffer(struct RCRX_msg_buffer* buf)
 * @param pointer to a RC receiver buffer struct
 * @return none
 * @brief initializes the buffer
 * @note 
 * @author Aaron Hunter,
 * @modified */
static void RCRX_init_msg_buffer(struct RCRX_msg_buffer* buf);

/**
 * @Function  RCRX_UART_interrupt_handler(void)
 * @brief Handles the RC receiver UART interrupts
 * @note 
 * @author Aaron Hunter
 * @modified  */
static void __ISR(_UART_5_VECTOR, IPL6SOFT) RCRX_UART_interrupt_handler(void);

/**
 * @Function void RCRX_run_RX_state_machine(uint8_t char_in);
 * @param char_in, next character to process
 * @return None
 * @brief Runs the RX state machine for receiving data, it is called from 
 * within the interrupt and reads the current character
 * @author Aaron Hunter */
static void RCRX_run_RX_state_machine(uint8_t char_in);

/**
 * @Function uint8_t RCRX_calc_cmd(RCRX_channel_buffer* channels)
 * @param pointer to channel data array
 * @return SUCCESS or ERROR
 * @brief converts sbus raw data into 11 bit channel data
 * @author Aaron Hunter 
 */
static uint8_t RCRX_calc_cmd(RCRX_channel_buffer *channels);


/*******************************************************************************
 * PUBLIC FUNCTION IMPLEMENTATIONS                                             *
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
uint8_t RCRX_init(void) {
    //initialize the message buffers
    RCRX_init_msg_buffer(RCRX_buf_p);
    /* turn off UART while configuring */
    U5MODEbits.ON = 0;
    __builtin_disable_interrupts();
    U5BRG = ((Board_get_PB_clock() / RX_BAUD) / 16) - 1; //set baud rate
    // configure the mode register
    U5MODEbits.RXINV = 1; //RX polarity is inverted
    U5MODEbits.PDSEL = 1; //8 bits, even parity
    U5MODEbits.STSEL = 1; //2 stop bits
    //config the status register
    U5STAbits.URXEN = 1; // enable RX pin
    U5STAbits.URXISEL = 0x0; //interrupt when character is received
    //configure UART interrupts
    IPC12bits.U5IP = 0b110; //Interrupt priority of 6
    IPC12bits.U5IS = 0; //sub-priority 0
    IEC2bits.U5RXIE = 1; //enable interrupt on RX
    IFS2bits.U5RXIF = 0; //clear interrupt flags
    // turn on UART
    U5MODEbits.ON = 1;
    __builtin_enable_interrupts();
    return SUCCESS;

}

/**
 * @Function RCRX_new_cmd_avail()
 * @param none
 * @return TRUE or FALSE
 * @brief returns TRUE if a new RC RX packet is available
 * @note 
 * @author aahunter
 * @modified <Your Name>, <year>.<month>.<day> <hour> <pm/am> */
uint8_t RCRX_new_cmd_avail(void) {
    return new_data_avail;
}

/**
 * @Function RCRX_get_cmd(RCRX_channel_buffer*)
 * @param pointer to servo data buffer
 * @return SUCCESS or ERROR
 * @brief processes most current message, stores data at pointer to data array
 * to the data
 * @note 
 * @author aahunter
 * @modified <Your Name>, <year>.<month>.<day> <hour> <pm/am> */
uint8_t RCRX_get_cmd(RCRX_channel_buffer *channels) {
    parsing_RX = TRUE;
    RCRX_calc_cmd(channels);
    parsing_RX = FALSE;
    //if a collision occured, clear the flag and re-enable interrupt to get data
    if (RX_collision == TRUE) {
        RX_collision = FALSE;
        IEC0bits.U1RXIE = 1;
    }
    new_data_avail = FALSE;

    return SUCCESS;
}

/**
 * @Function RCRX_get_err()
 * @param pointer to servo data buffer
 * @return number of rc errors detected since init
 * @brief 
 * @note 
 * @author aahunter
 * @modified <Your Name>, <year>.<month>.<day> <hour> <pm/am> */
unsigned int RCRX_get_err(void) {
    return parse_error_counter;
}

/*******************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATIONS                                            *
 ******************************************************************************/

/**
 * @Function RCRX_init_msg_buffer(struct RCRX_msg_buffer* buf)
 * @param pointer to a RC receiver buffer struct
 * @return none
 * @brief initializes the buffer
 * @note 
 * @author Aaron Hunter,
 * @modified */
static void RCRX_init_msg_buffer(struct RCRX_msg_buffer* buf) {
    int i;
    int j;
    buf->read_index = 0;
    buf->write_index = 0;
    for (i = 0; i < RX_NUM_MSGS; i++) {
        for (j = 0; j < SBUS_BUFFER_LENGTH; j++) {
            buf->sbus_buffer[i][j] = 0;
        }
    }
}

/**
 * @Function  RCRX_UART_interrupt_handler(void)
 * @brief Handles the RC receiver UART interrupts
 * @note 
 * @author Aaron Hunter
 * @modified  */
static void __ISR(_UART_5_VECTOR, IPL6SOFT) RCRX_UART_interrupt_handler(void) {
    if (IFS2bits.U5RXIF) { //check for received data flag
        //run the state machine with the new character from the RX buffer
        if (parsing_RX == FALSE) {
            RCRX_run_RX_state_machine(U5RXREG);
        } else {
            /*a collision occurred need to disable interrupts to exit ISR*/
            /*it will be re-enabled in parsing_RX */
            RX_collision = TRUE;
            IEC0bits.U1RXIE = 0;
        }
        IFS2bits.U5RXIF = 0; //clear the flag
    }
}

///**
// * @Function void RCRX_run_RX_state_machine(uint8_t char_in);
// * @param char_in, next character to process
// * @return None
// * @brief Runs the RX state machine for receiving data, it is called from 
// * within the interrupt and reads the current character
// * @author Aaron Hunter */
//static void RCRX_run_RX_state_machine(uint8_t char_in) {
//    static RCRX_state_t current_state = WAIT_FOR_START;
//    RCRX_state_t next_state = WAIT_FOR_START;
//
//    static uint8_t byte_counter = 0;
//    switch (current_state) {
//        case WAIT_FOR_START:
//            if (char_in == START_BYTE) {
//                byte_counter = 0;
//                parsing_RX = TRUE; //set parsing flag
//                //store the start byte 
//                RCRX_msgs.sbus_buffer[RCRX_msgs.write_index][byte_counter] = char_in;
//                byte_counter++;
//                next_state = GET_DATA;
//            } else {
//                next_state = WAIT_FOR_START;
//            }
//            break;
//        case GET_DATA:
//            RCRX_msgs.sbus_buffer[RCRX_msgs.write_index][byte_counter] = char_in;
//            byte_counter++;
//            if (byte_counter == (SBUS_BUFFER_LENGTH - 1)) {
//                next_state = GET_END;
//            } else {
//                next_state = GET_DATA;
//            }
//            break;
//        case GET_END:
//            if (char_in == END_BYTE) {
//                RCRX_msgs.sbus_buffer[RCRX_msgs.write_index][byte_counter] = END_BYTE;
//                // set read index to most recent messsage
//                RCRX_msgs.read_index = RCRX_msgs.write_index;
//                //advance write index and wrap
//                RCRX_msgs.write_index = (RCRX_msgs.write_index + 1) % RX_NUM_MSGS;
//                new_data_avail = TRUE;
//                parse_error = FALSE; //clear previous parsing error
//            } else {
//                parse_error = TRUE; //error occurred
//                parse_error_counter++;
//            }
//            parsing_RX = FALSE; //clear the parsing flag to let app know write data is most current
//            next_state = WAIT_FOR_START;
//            break;
//        default:
//            next_state = WAIT_FOR_START;
//            break;
//    }
//    current_state = next_state;
//}

///**
// * @Function void RCRX_run_RX_state_machine(uint8_t char_in);
// * @param char_in, next character to process
// * @return None
// * @brief Runs the RX state machine for receiving data, it is called from 
// * within the interrupt and reads the current character
// * @author Aaron Hunter */
//static void RCRX_run_RX_state_machine(uint8_t char_in) {
//    static RCRX_state_t current_state = WAIT_FIRST;
//    RCRX_state_t next_state;
//    static uint8_t byte_counter = 0;
//    switch (current_state) {
//        case WAIT_FIRST:
////            printf("%c", char_in);
//            if (char_in == END_BYTE) {
//                next_state = GET_SECOND;
//            } else {
//                next_state = WAIT_FIRST;
//            }
//            break;
//        case GET_SECOND:
//            if (char_in == END_BYTE) {
//                next_state = GET_START;
//            } else {
//                next_state = WAIT_FIRST;
//            }
//            break;
//        case GET_START:
//            if (char_in == START_BYTE) {
//                byte_counter = 0;
//                parsing_RX = TRUE; //set parsing flag
//                //store the start byte 
//                RCRX_msgs.sbus_buffer[RCRX_msgs.write_index][byte_counter] = char_in;
//                byte_counter++;
//                next_state = GET_DATA;
//            } else {
//                next_state = WAIT_FIRST;
//            }
//            break;
//        case GET_DATA:
//            RCRX_msgs.sbus_buffer[RCRX_msgs.write_index][byte_counter] = char_in;
//            byte_counter++;
//            if (byte_counter == (SBUS_BUFFER_LENGTH)) {
//                // set read index to most recent messsage
//                RCRX_msgs.read_index = RCRX_msgs.write_index;
//                //advance write index and wrap
//                RCRX_msgs.write_index = (RCRX_msgs.write_index + 1) % RX_NUM_MSGS;
//                new_data_avail = TRUE;
//                parsing_RX = FALSE; //clear the parsing flag to let app know write data is most current
//                next_state = WAIT_FIRST;
//
//            } else {
//                next_state = GET_DATA;
//            }
//            break;
//        default:
//            next_state = WAIT_FIRST;
//            break;
//    }
//    current_state = next_state;
//}

/**
 * @Function void RCRX_run_RX_state_machine(uint8_t curr_byte);
 * @param curr_byte, next character to process
 * @return None
 * @brief Runs the RX state machine for receiving data, it is called from 
 * within the interrupt and reads the current character
 * @author Aaron Hunter */
static void RCRX_run_RX_state_machine(uint8_t curr_byte) {
    static RCRX_state_t current_state = GET_START;
    static uint8_t prev_byte = START_BYTE; //initialize the previous byte to something other than END_BYTE
    static uint8_t byte_counter = 0;
    RCRX_state_t next_state;
    switch (current_state) {
        case WAIT_SYNC:
            /* data stream shows STATUS BYTE = 0x00 = END_BYTE so we look for 
             * two instances of END_BYTE in a row to synchronize the parsing
             * sequence. 
             */
            if (curr_byte == END_BYTE && prev_byte == END_BYTE) {
                next_state = GET_START;
            }
            break;
        case GET_START:
            if (curr_byte == START_BYTE && prev_byte == END_BYTE) {
                byte_counter = 0;
                //store the start byte 
                RCRX_msgs.sbus_buffer[RCRX_msgs.write_index][byte_counter] = curr_byte;
                byte_counter++;
                next_state = GET_DATA;
            } else {
                next_state = WAIT_SYNC;
            }
            break;
        case GET_DATA:
            RCRX_msgs.sbus_buffer[RCRX_msgs.write_index][byte_counter] = curr_byte;
            byte_counter++;
            if (byte_counter == (SBUS_BUFFER_LENGTH)) {
                if (curr_byte == END_BYTE) {
                    new_data_avail = TRUE;
                    // data is good, so set the read index to this array
                    RCRX_msgs.read_index = RCRX_msgs.write_index;
                    //advance write index and wrap
                    RCRX_msgs.write_index = (RCRX_msgs.write_index + 1) % RX_NUM_MSGS;
                    next_state = GET_START; //return to the beginning
                } else { //otherwise the data is corrupt so don't store it
                    parse_error == TRUE;
                    parse_error_counter++;
                    next_state = WAIT_SYNC; // need to re-sync the signal
                }
                byte_counter = 0;
            } else {
                next_state = GET_DATA;
            }
            break;
        default:
            next_state = WAIT_SYNC;
            break;
    }
    current_state = next_state;
    prev_byte = curr_byte;
}

/**
 * @Function delay(int cycles)
 * @param cycles, number of cycles to delay
 * @brief simple delay loop
 * @note ~500nsec for one delay, then +12.5 nsec for every increment higher
 * @author ahunter
 */
void RCRX_delay(int cycles) {
    int i;
    for (i = 0; i < cycles; i++) {
        Nop();
    }
}

/**
 * @Function uint8_t RCRX_calc_cmd(void)
 * @param none
 * @return SUCCESS or ERROR
 * @brief converts sbus raw data into 11 bit channel data
 * @author Aaron Hunter 
 */
static uint8_t RCRX_calc_cmd(RCRX_channel_buffer *channels) {
    channels[0] = (uint16_t) ((RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][1]\
            | RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][2] << 8) &0x7ff);
    channels[1] = (uint16_t) ((RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][2] >> 3 \
            | RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][3] << 5) &0x7ff);
    channels[2] = (uint16_t) ((RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][3] >> 6 \
            | RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][4] << 2 \
            | RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][5] << 10) &0x7ff);
    channels[3] = (uint16_t) ((RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][5] >> 1 \
            | RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][6] << 7) & 0x7ff);
    channels[4] = (uint16_t) ((RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][6] >> 4 \
            | RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][7] << 4) & 0x7ff);
    channels[5] = (uint16_t) ((RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][7] >> 7 \
            | RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][8] << 1 \
            | RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][9] << 9) & 0x7ff);
    channels[6] = (uint16_t) ((RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][9] >> 2 \
            | RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][10] << 6) & 0x7ff);
    channels[7] = (uint16_t) ((RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][10] >> 5 \
            | RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][11] << 3) & 0x7ff);
    // this pattern repeats for the second 8 channels
    channels[8] = (uint16_t) ((RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][12]\
            | RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][13] << 8) &0x7ff);
    channels[9] = (uint16_t) ((RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][13] >> 3 \
            | RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][14] << 5) &0x7ff);
    channels[10] = (uint16_t) ((RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][14] >> 6 \
            | RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][15] << 2 \
            | RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][16] << 10) &0x7ff);
    channels[11] = (uint16_t) ((RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][16] >> 1 \
            | RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][17] << 7) & 0x7ff);
    channels[12] = (uint16_t) ((RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][17] >> 4 \
            | RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][18] << 4) & 0x7ff);
    channels[13] = (uint16_t) ((RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][18] >> 7 \
            | RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][19] << 1 \
            | RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][20] << 9) & 0x7ff);
    channels[14] = (uint16_t) ((RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][20] >> 2 \
            | RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][21] << 6) & 0x7ff);
    channels[15] = (uint16_t) ((RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][21] >> 5 \
            | RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][22] << 3) & 0x7ff);
    return SUCCESS;
}


/*Test harness*/
#ifdef RC_RX_TESTING
RCRX_channel_buffer servo_data[CHANNELS];

void main(void) {
    uint8_t i;
    Board_init();
    Serial_init();
    printf("\r\nRC Receiver Test Harness %s %s\r\n", __DATE__, __TIME__);
    RCRX_init();
    printf("Radio control receiver initialized.\r\n");
    while (1) {
        if (RCRX_new_cmd_avail() == TRUE) {
            RCRX_get_cmd(servo_data);
            ///*Throttle is assigned to elevator channel to center at midpoint for ESCs unlike
            // how an airplane motor is configured.  We need reverse drive in other words.
            // Steering servo is assigned to rudder channel, may be easier to drive on aileron
            // Switch D is for passthrough mode and assigned to channel .  Low is passthrough, High is autonomous*/
            printf("T %d S %d M %d stat %x ERR %d \r", servo_data[2], servo_data[3], \
                    servo_data[7], RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][23], parse_error_counter);
//            printf("stat %x ERR %d \r\n", RCRX_msgs.sbus_buffer[RCRX_msgs.read_index][23], parse_error_counter);
        }
    }
}

#endif //RC_RX_TESTING
