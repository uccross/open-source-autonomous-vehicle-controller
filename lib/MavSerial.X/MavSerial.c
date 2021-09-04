/* 
 * File:   MavSerial_.c
 * Author: Pavlo Vlastos
 *
 * Created on March 13, 2021, 5:19 PM, based on code from Max Dunne
 * Modified on August 29, 2021, 2:01 PM
 */

#include <xc.h>
#include "MavSerial.h"

#include "Board.h"
#include <sys/attribs.h> //needed to use an interrupt
#include <stdint.h>
#include <proc/p32mx795f512l.h>


/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/

#define F_PB (board_get_pb_clock())
#define QUEUESIZE 512
#define PARSE_CALL_COUNT_LIMIT QUEUESIZE
#define TX_BUFFER_LENTH 2041

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


/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                *
 ******************************************************************************/
void newCircBuffer(CBRef cB);
void freeCircBuffer(CBRef* cB);
unsigned int getLength(CBRef cB);
int readHead(CBRef cB);
int readTail(CBRef cB);
unsigned char peak(CBRef cB);
unsigned char readFront(CBRef cB);
unsigned char writeBack(CBRef cB, unsigned char data);
static uint8_t rx_buf_head_index = 0;
static uint8_t rx_buf_tail_index = 0;
static mavlink_message_t rx_buffer[MAV_SERIAL_RX_BUF_SIZE];
static uint8_t tx_buf[TX_BUFFER_LENTH];

/*******************************************************************************
 * PRIVATE VARIABLES                                                           *
 ******************************************************************************/
struct CircBuffer outgoingUart;
CBRef transmitBuffer;
struct CircBuffer incomingUart;
CBRef receiveBuffer;
static uint8_t AddingToTransmit = FALSE;
static uint8_t GettingFromReceive = FALSE;
static uint8_t TransmitCollisionOccured = FALSE;
static uint8_t ReceiveCollisionOccured = FALSE;
static mavlink_status_t status;

// Debugging:

static enum {
    EXCEP_IRQ = 0, // interrupt
    EXCEP_AdEL = 4, // address error exception (load or ifetch)
    EXCEP_AdES, // address error exception (store)
    EXCEP_IBE, // bus error (ifetch)
    EXCEP_DBE, // bus error (load/store)
    EXCEP_Sys, // syscall
    EXCEP_Bp, // breakpoint
    EXCEP_RI, // reserved instruction
    EXCEP_CpU, // coprocessor unusable
    EXCEP_Overflow, // arithmetic overflow
    EXCEP_Trap, // trap (possible divide by zero)
    EXCEP_IS1 = 16, // implementation specfic 1
    EXCEP_CEU, // CorExtend Unuseable
    EXCEP_C2E // coprocessor 2
} _excep_code;
//static unsigned int _epc_code; // Microchips example from online...
static unsigned int _excep_addr;

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                           *
 ******************************************************************************/
// Debugging:
// this function overrides the normal _weak_ generic handler

void _general_exception_handler(void) {
    asm volatile("mfc0 %0,$13" : "=r" (_excep_code));
    asm volatile("mfc0 %0,$14" : "=r" (_excep_addr));
    _excep_code = (_excep_code & 0x0000007C) >> 2;
    while (1) {
        // Examine _excep_code to identify the type of exception
        // Examine _excep_addr to find the address that caused the exception
    }
}

void MavSerial_Init(void) {
    transmitBuffer = (struct CircBuffer*) &outgoingUart; //set up buffer for receive
    newCircBuffer(transmitBuffer);

    receiveBuffer = (struct CircBuffer*) &incomingUart; //set up buffer for transmit
    newCircBuffer(receiveBuffer);


    /**************************************************************************/
    /* Setup for UART Transmit and Receive 
     * For PIC32MX795F512L, both TX and RX buffers are 8-levels, FIFO
     * Requires call to Board_init() first. */
    /**************************************************************************/

    /* 1. Initialize the UxBRG register for the appropriate baud rate */
    int sourceClock = Board_get_PB_clock() >> 3;
    int brg = sourceClock / 115200;
    brg++;
    brg >>= 1;
    brg--;
    U1BRG = brg;

    /* 2. Set the number of data nd Stop bits, and parity selectino by writing 
     * to the PDSEL <1:0> bits (UxMODE<2:1>) and STSEL bit (UxMODE<0>). */
    U1MODE = 0; /* 8-bit data, no parity bit */
    U1MODEbits.STSEL = 0; /* Stop Selection bit: 0 -> 1 Stop bit */


    /* 3. If transmit interrupts are desired, set the UxTXIE control bit in the 
     * corresponding interrupt Enable Control register (IEC). Specify the 
     * interrupt priority and sub-priority for the transmit interrupt using the
     * UxIP<2:0> and UxIS<1:0> control bits in the corresponding Interrupt
     * Priority Control register (IPC). Also, select the Transmit Interrupt mod
     * by writing to the UTXISEL bits (UxSTA<15:14>).*/
    IEC0bits.U1TXIE = 1; // Enabled transmit interrupts 
    IPC6bits.U1IP = 1; // Priority 1 (the lowest) 
    IPC6bits.U1IS = 1; // Sub-priority 1 (the second lowest) 
    U1STAbits.UTXISEL = 1; /* Interrupt is generated and asserted when all 
                             * characters have been transmitted */
    /* For receive */
    IEC0bits.U1RXIE = 1; // Enable receive interrupts as well
    U1STAbits.URXISEL = 0; /* Interrupt flag bit is asserted while receive 
                            * buffer is not empty (i.e., has at least 1 data
                            * character). */


    /* 4. Enable the transmission by setting the UTXEN bit (UxSTA<10>), which
     * also sets the UxTXIF bit. The UxTXIF bit should be cleared in the 
     * software routine that services the UART transmit interrupt. The operation
     * of the UxTXIF bit is controlled by the UTXISEL control bits. */
    U1STAbits.UTXEN = 1;

    /* Enabled receive */
    U1STAbits.URXEN = 1;

    /* 5. Enable the UART module by setting the ON bit (UxMODE<15>). */
    U1MODEbits.ON = 1;

    /* 6. Load data to the UxTXREG register (start transmission). 
     * This step is not necessary to do here specifically, if there is an 
     * interrupt UART ISR, which in this case is true. Loading the Transmit
     * buffer register (8-level deep for PIC32MX795F512L) can happen in the 
     * interrupt. */
    //U1TXREG = 'x'; //Example of loading the transmit buffer register.
}

void MavSerial_PutChar(char ch) {

    if (getLength(transmitBuffer) < QUEUESIZE) {
        AddingToTransmit = TRUE;
        writeBack(transmitBuffer, ch);
        AddingToTransmit = FALSE;
        if (U1STAbits.TRMT) { // If the shift register is empty, TRMT bit high
            IFS0bits.U1TXIF = 1; // Set transmit interrupt flag high
            IEC0bits.U1TXIE = 1; // Enable Transmit Interrupt again, now that TX needed
        }
        //re-enter the interrupt if we removed a character while getting another one
        if (TransmitCollisionOccured) {
            IFS0bits.U1TXIF = 1;
            TransmitCollisionOccured = FALSE;
            IEC0bits.U1TXIE = 1; // Enable Transmit Interrupt again, now that TX needed
        }
    }
}

char MavSerial_GetChar(void) {
    char ch;
    if (getLength(receiveBuffer) == 0) {
        ch = 0;
    } else {
        GettingFromReceive = TRUE;
        ch = readFront(receiveBuffer);
        GettingFromReceive = FALSE;
    }
    //re-enter the interrupt if we added a character while transmitting another one
    if (ReceiveCollisionOccured) {
        IFS0bits.U1RXIF = 1;
        ReceiveCollisionOccured = FALSE;
    }
    return ch;
}

void _mon_putc(char c) {
    MavSerial_PutChar(c);
}

void _mon_puts(const char* s) {
    int i;
    for (i = 0; i<sizeof (s); i++)
        MavSerial_PutChar(s[i]);
}

char MavSerial_IsRxEmpty(void) {
    if (getLength(receiveBuffer) == 0)
        return TRUE;
    return FALSE;
}

char MavSerial_IsTxEmpty(void) {
    if (getLength(transmitBuffer) == 0)
        return TRUE;
    return FALSE;
}

char MavSerial_IsMavMsgAvailable(void) {
    char status = FALSE;
    if ((rx_buf_tail_index - rx_buf_head_index) != 0) {
        status = TRUE;
    }
    return status;
}

char MavSerial_IsMavMsgBufFull(void) {
    char status = TRUE;

    if (MAV_SERIAL_RX_BUF_SIZE - (rx_buf_tail_index - rx_buf_head_index) > 0) {
        status = FALSE;
    }

    return status;
}

uint8_t MavSerial_IsSpaceLeft(void) {
    uint8_t n = 0;
    n = MAV_SERIAL_RX_BUF_SIZE - (rx_buf_tail_index - rx_buf_head_index);
    return n;
}

char MavSerial_getMavMsg(mavlink_message_t* r_message) {
    unsigned int i = 0;
    char is_new_msg = FALSE;

    if (rx_buf_head_index != rx_buf_tail_index) { // Is the rx_buffer empty?

        r_message->checksum = rx_buffer[rx_buf_head_index].checksum;
        r_message->compat_flags = rx_buffer[rx_buf_head_index].compat_flags;
        r_message->compid = rx_buffer[rx_buf_head_index].compid;
        r_message->incompat_flags = rx_buffer[rx_buf_head_index].incompat_flags;
        r_message->len = rx_buffer[rx_buf_head_index].len;
        r_message->magic = rx_buffer[rx_buf_head_index].magic;
        r_message->msgid = rx_buffer[rx_buf_head_index].msgid;

        for (i = 0; i < r_message->len; i++) {
            r_message->payload64[i] = rx_buffer[rx_buf_head_index].payload64[i];
        }

        r_message->seq = rx_buffer[rx_buf_head_index].seq;
        r_message->sysid = rx_buffer[rx_buf_head_index].sysid;

        rx_buf_head_index++;
        rx_buf_head_index %= MAV_SERIAL_RX_BUF_SIZE;

        is_new_msg = TRUE;
    }

    return is_new_msg;
}

void MavSerial_send_mav_packet(mavlink_message_t *msg) {
    int bytes_sent = 0;
    uint8_t num_to_send = 0;
    num_to_send = mavlink_msg_to_send_buffer(tx_buf, msg);
    for (bytes_sent = 0; bytes_sent < num_to_send; bytes_sent++) {
        MavSerial_PutChar(tx_buf[bytes_sent % TX_BUFFER_LENTH]);
    }
}

void MavSerial_ParseWrapper(void) {

    if (((rx_buf_tail_index + 1) % MAV_SERIAL_RX_BUF_SIZE) !=
            rx_buf_head_index) {
        if (mavlink_parse_char(MAVLINK_COMM_0, (uint8_t) MavSerial_GetChar(),
                &(rx_buffer[rx_buf_tail_index]), &status) == 1) {
        }
    }
}

int MavSerial_SendAck(uint8_t sys_id, uint8_t result, 
        mavlink_message_t *msg) {
    /* Send a command acknowledgment */
    mavlink_msg_command_ack_pack(sys_id, 1, msg, MAV_CMD_ACK_OK, result, 0, 0, 0, 0);

    MavSerial_send_mav_packet(msg);
    return SUCCESS;
}

/*******************************************************************************
 * PRIVATE FUNCTIONS                                                          *
 ******************************************************************************/

/**
 * @Function IntUart1Handler(void)
 * @param None.
 * @return none
 * @brief  
 * @author Max Dunne 2011.12.15, modified by Pavlo Vlastos 2019.7.16 */
void __ISR(_UART_1_VECTOR, IPL1SOFT) uart_1_int_handler(void) {
    if (IFS0bits.U1RXIF) {
        if (mavlink_parse_char(MAVLINK_COMM_0, (uint8_t) U1RXREG,
                &(rx_buffer[rx_buf_tail_index]), &status) == 1) {

            if (((rx_buf_tail_index + 1) % MAV_SERIAL_RX_BUF_SIZE) !=
                    rx_buf_head_index) {
                rx_buf_tail_index++;
                rx_buf_tail_index %= MAV_SERIAL_RX_BUF_SIZE;
            }
        }
        IFS0bits.U1RXIF = 0; // Clearing RX flag AFTER potential collision resolution
    }

    if (IFS0bits.U1TXIF == 1) { // Is Transmit Interrupt, specifically, thrown?
        IFS0bits.U1TXIF = 0; // Clear the Transmit interrupt flag status bit
        if (getLength(transmitBuffer) > 0) {
            if (!AddingToTransmit) {
                U1TXREG = readFront(transmitBuffer);
            } else { // There may have been a collision
                TransmitCollisionOccured = TRUE;
                IEC0bits.U1TXIE = 0; // Disable the TX Interrupt until next putchar()
                //LATDbits.LATD1 ^= 1; // Toggle IO pin for o-scope test
            }
        }
        if (getLength(transmitBuffer) == 0) { //
            IEC0bits.U1TXIE = 0; // Disable the TX Interrupt until next putchar()
        }
    }
}

void newCircBuffer(CBRef cB) {

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

// this function frees the Circular Buffer CB Ref

void freeCircBuffer(CBRef* cB) {
    // if it is already null, nothing to free
    if (cB == NULL || *cB == NULL) {

        return;
    }

    // free and nil the pointer
    //free(*cB);
    *cB = NULL;
}




// Accesor Methods
// ===============

// returns the amount of unread bytes in the circular buffer

unsigned int getLength(CBRef cB) {
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

// returns the actual index of the head

int readHead(CBRef cB) {
    // if the circular buffer is not null
    if (cB != NULL) {
        return (cB->head);
    } else {

        return 0;
    }

}

// returns the actual index of the tail

int readTail(CBRef cB) {
    // if the circular buffer is not null
    if (cB != NULL) {
        return (cB->tail);
    } else {

        return 0;
    }

}

// returns the byte (actual value) that the head points to. this
// does not mark the byte as read, so succesive calls to peak will
// always return the same value

unsigned char peak(CBRef cB) {
    // if the circular buffer is not null
    if (cB != NULL) {
        // if there are bytes in the buffer
        if (getLength(cB) > 0) {

            return cB->buffer[cB->head];
        }
    }
    return 0;
}


// Manipulation Procedures
// ======================
// returns the front of the circular buffer and marks the byte as read

unsigned char readFront(CBRef cB) {
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

// writes one byte at the end of the circular buffer,
// increments overflow count if overflow occurs

unsigned char writeBack(CBRef cB, unsigned char data) {
    // if the circular buffer is not null
    if (cB != NULL) {
        if (getLength(cB) == (cB->size - 1)) {
            cB->overflowCount++;
            //return 1;
        } else {
            cB->buffer[cB->tail] = data;
            cB->tail = cB->tail < (cB->size - 1) ? cB->tail + 1 : 0;
            //return 0;
        }
        //return 0;
    } else {

        return 1;
    }
    return 0;
}

// empties the circular buffer. It does not change the size. use with caution!!

void makeEmpty(CBRef cB) {
    if (cB != NULL) {
        int i;
        for (i = 0; i < cB->size; ++i) {

            cB->buffer[i] = 0;
        }
        cB->head = 0;
        cB->tail = 0;
        cB->overflowCount = 0;
    }
}

// returns the amount of times the CB has overflown;

unsigned char getOverflow(CBRef cB) {
    if (cB != NULL) {

        return cB->overflowCount;
    }
    return 0;
}

/******************************************************************************
 * UNIT TESTS
 *****************************************************************************/
#ifdef MAV_SERIAL_TEST

/******************************************************************************
 * MAIN
 *****************************************************************************/

//#include "radio.h"
#include "System_timer.h"

#define RATE_1HZ 1000
#define SYS_ID 1

int main(void) {
    Board_init();
    MavSerial_Init(); // UART 1 serial for debugging
    Sys_timer_init(); //start the system timer
//    radio_init();
//    timer_init(); // Placed here to observe any possible influence

    TRISAbits.TRISA3 = 0; /* Set pin as output. This is also LED4 on Max32 */
    TRISCbits.TRISC1 = 0; /* LED5 */

    LATCbits.LATC1 = 0; /* Set LED5 low */
    LATAbits.LATA3 = 0; /* Set LED4 low */
    
#ifdef ECHO
    char x = 'x';
    while (1) {
        if (x != 0) {
            MavSerial_PutChar(x);
//            radio_put_char(x);
            x = 0;
        }
        x = MavSerial_GetChar();
    }
#endif


#ifdef TRANSMIT_ONLY
    unsigned int t_new = 0;
    unsigned int t_old = 0;
    
    int i = 0;
    for (i = 0; i < 10000; i++) {
        MavSerial_PutChar(0x22);
    }

    LATCbits.LATC1 = 0; /* Set LED5 low */
    LATAbits.LATA3 = 0; /* Set LED4 low */

    mavlink_message_t msg;

    int vehicle_mode = MAV_MODE_PREFLIGHT;
    int vehicle_state = MAV_STATE_STANDBY;
    /* Send Heartbeat */
    mavlink_msg_heartbeat_pack(SYS_ID, 1, &msg,
            MAV_TYPE_SURFACE_BOAT,
            MAV_AUTOPILOT_GENERIC,
            vehicle_mode,
            1, vehicle_state);

    while (1) {
        t_new = Sys_timer_get_msec();

        if ((t_new - t_old) >= RATE_1HZ) {
            t_old = t_new;
            MavSerial_send_mav_packet(&msg);
            LATAbits.LATA3 ^= 1; /* Set LED4 */
        }
    }
#endif

#ifdef RECEIVE_ONLY

    LATCbits.LATC1 = 0; /* Set LED5 low */
    LATAbits.LATA3 = 0; /* Set LED4 low */

    mavlink_message_t rec_msg;

    while (1) {
        if (MavSerial_getMavMsg(&rec_msg) == TRUE) {
            LATCbits.LATC1 ^= 1; /* Set LED5 */
            LATAbits.LATA3 ^= 1; /* Set LED4 */
        }
    }
#endif

#ifdef SEND_AND_RECEIVE_MAVLINK

    // Timing
    unsigned int t_new = 0;
    unsigned int t_old = 0;

    // Small 5 second delay so that if this test fails, the reset is obvious
    while ((t_new - t_old) < 5000) {
        t_new = Sys_timer_get_msec();

        if ((((t_new >> 2) << 2) % 100) == 0) {
            LATAbits.LATA3 ^= 1; /* Set LED4 */
            LATCbits.LATC1 ^= 1; /* Set LED5 */
        }
    }
    LATAbits.LATA3 = 0; // LED4

    mavlink_message_t msg;
    mavlink_message_t rec_msg;

    int vehicle_mode = MAV_MODE_PREFLIGHT;
    int vehicle_state = MAV_STATE_STANDBY;

    int need_to_ack = FALSE;

    /**************************************************************************
     * MAIN LOOP
     *************************************************************************/
    while (1) {
        t_new = Sys_timer_get_msec();

        /**********************************************************************
         * MAVLink transmission to companion computer
         *********************************************************************/
        if ((t_new - t_old) >= RATE_1HZ) {
            t_old = t_new;

            /* Send Heartbeat */
            mavlink_msg_heartbeat_pack(SYS_ID, 1, &msg,
                    MAV_TYPE_SURFACE_BOAT,
                    MAV_AUTOPILOT_GENERIC,
                    vehicle_mode,
                    1, vehicle_state);

            MavSerial_send_mav_packet(&msg);

            /* Send System Status */
            mavlink_msg_sys_status_pack(SYS_ID, 1, &msg, 0, 0, 0, 500, 11000, -1,
                    -1, 0, 0, 0, 0, 0, 0);
            MavSerial_send_mav_packet(&msg);

            if (need_to_ack == TRUE) {
                /* Send a command acknowledgment */
                mavlink_msg_command_ack_pack(SYS_ID, 1, &msg,
                        MAV_CMD_ACK_OK,
                        1, /* 1 for armed - @TODO: double check this */
                        0,
                        0,
                        0, /* @TODO: add TARGET SYSTEM */
                        0); /* @TODO: add TARGET COMPONENT */
                MavSerial_send_mav_packet(&msg);
                need_to_ack = FALSE;
            }

            LATAbits.LATA3 ^= 1; // LED4
        }

        /**********************************************************************
         * MAVLink reception from companion computer
         *********************************************************************/

        /* To test this, you must have something like QGC or pymavlink to 
         * send a message with MAVLINK_MSG_ID_COMMAND_LONG */

        if (MavSerial_getMavMsg(&rec_msg) == TRUE) {
            if (mavlink_msg_command_long_get_command(&rec_msg) ==
                    MAV_CMD_COMPONENT_ARM_DISARM) {
                need_to_ack = TRUE;
            }
            if (mavlink_msg_command_long_get_command(&rec_msg) ==
                    MAV_CMD_NAV_WAYPOINT) {
                need_to_ack = TRUE;
                LATCbits.LATC1 ^= 1; /* Set LED5 */
            }
        }
    }
#endif

    return 1;
}

#endif
