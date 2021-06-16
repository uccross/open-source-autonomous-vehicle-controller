/* 
 * File:   SerialM32.c
 * Author: Aaron Hunter
 * Brief: 
 * Created on November 10, 2020, 9:52 am
 * Modified on November 11/2020 9:27 am
 */

/*******************************************************************************
 * #INCLUDES                                                                   *
 ******************************************************************************/

#include "SerialM32.h" // The header file for this source file. 
#include "Board.H"   //Max32 setup      
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/attribs.h>  //for ISR definitions
#include <proc/p32mx795f512l.h>

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/
#define BUFFER_LENGTH 2048
#define MESSAGE_LENGTH 128
#define BAUD_RATE 115200

/*******************************************************************************
 * PRIVATE TYPEDEFS                                                            *
 ******************************************************************************/
struct circular_buffer {
    int read_index;
    int write_index;
    int size;
    unsigned char data[BUFFER_LENGTH];
}; /*circular  buffer */

struct circular_buffer tx_buffer; /*Circular UART TX buffer */
struct circular_buffer rx_buffer; /*Circular UART RX buffer */

/*pointers to the buffers so we don't pass the entire buffer to the functions*/
struct circular_buffer *txp = &tx_buffer;
struct circular_buffer *rxp = &rx_buffer;

static int8_t tx_collision = FALSE;
static int8_t rx_collision = FALSE;
static int8_t writing_tx_buffer = FALSE;
static int8_t reading_rx_buffer = FALSE;
static int8_t new_char_avail = FALSE;

/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                 *
 ******************************************************************************/
static void init_buffer(struct circular_buffer *buf);
static int8_t is_buffer_empty(struct circular_buffer *buf);
static int8_t is_buffer_full(struct circular_buffer *buf);
static int8_t write_buffer(struct circular_buffer *buf, unsigned char c);
static unsigned char read_from_buffer(struct circular_buffer *buf);
static int get_num_elements(struct circular_buffer *buf);

/*******************************************************************************
 * PUBLIC FUNCTION IMPLEMENTATIONS                                             *
 ******************************************************************************/

/**
 * @Function Protocol_Init(void)
 * @param None
 * @return SUCCESS or ERROR
 * @brief enables UART and interrupts
 * @author ahunter*/
void Serial_init(void) {
    /*Initialize the circular buffers*/
    init_buffer(rxp);
    init_buffer(txp);
    /* turn off UART while configuring */
    U1MODEbits.ON = 0;
    __builtin_disable_interrupts();
    U1MODEbits.BRGH = 0; //M = 16
    U1BRG = ((Board_get_PB_clock() / BAUD_RATE) / 16) - 1; //set baud rate
    /* configure the RX and TX pins */
    U1STAbits.UTXEN = 1;
    U1STAbits.URXEN = 1;
    /*clear any overflow*/
    if (U1STAbits.OERR == 1) {
        U1STAbits.OERR = 0;
    }
    /* configure using software flow control, if set to 2 it would be CTS/RTS */
    U1MODEbits.UEN = 0;
    /*configure UART interrupts */
    U1STAbits.URXISEL = 0x0; //interrupt when buffer is not empty
    U1STAbits.UTXISEL = 0x01; // int when all characters are sent (TRMT == TRUE)
    IFS0bits.U1RXIF = 0; //clear interrupt flags
    IFS0bits.U1TXIF = 0;
    IPC6bits.U1IP = 1; //set interrupt priority to 1 
    IEC0bits.U1RXIE = 1; //enable interrupt on RX
    IEC0bits.U1TXIE = 1; //enable interrupts on TX
    /* turn on UART*/
    U1MODEbits.ON = 1;
    __builtin_enable_interrupts();
}

/**
 * @Function IntUart1Handler(void)
 * @brief Handles the UART interrupts
 * @note 
 * @author Aaron Hunter
 * @modified  */
void __ISR(_UART_1_VECTOR, IPL1SOFT) IntUart1Handler(void) {
    if (IFS0bits.U1RXIF) { //check for received data flag
        if (is_buffer_full(rxp) == FALSE) {
            if (reading_rx_buffer == FALSE) {
                write_buffer(rxp, U1RXREG);
            } else {
                /*a collision occurred need to disable interrupts to exit ISR*/
                /*it will be re-enabled in get_char() */
                rx_collision = TRUE;
                IEC0bits.U1RXIE = 0;
            }
        }
        IFS0bits.U1RXIF = 0; // clear the flag
    } else if (IFS0bits.U1TXIF) { /*check for transmission flag*/
        if (is_buffer_empty(txp) == TRUE) { //if tx circular buffer is empty
            IEC0bits.U1TXIE = 0; //disable interrupts on TX
        } else if (writing_tx_buffer == FALSE) { /*If not being accessed */
            U1TXREG = read_from_buffer(txp);
        } else {
            /*a collision occurred, turn off interrupt to exit ISR*/
            /*it will be re-enabled in put_char()*/
            tx_collision = TRUE;
            IEC0bits.U1TXIE = 0;
        }
        IFS0bits.U1TXIF = 0; /*clear the interrupt flag*/
    } else if (IFS0bits.U1EIF) { //error flag--can check U1STA for the reason
        if (U1STAbits.OERR == 1) {
            U1STAbits.OERR = 0; //over run error is the only one we can write to clear
        }
        IFS0bits.U1EIF = 0;
    }
}

/**
 * @Function get_char(void)
 * @return character read from receive buffer
 * @brief  returns the value in the receive buffer
 * @author Aaron Hunter*/
unsigned char get_char(void) {
    unsigned char c;
    if (is_buffer_empty(rxp) == FALSE) {
        reading_rx_buffer = TRUE; /*set buffer access flag*/
        c = read_from_buffer(rxp);
        reading_rx_buffer = FALSE;
        //if a collision occurred we need to re-enable interrupts
        if (rx_collision == TRUE) {
            rx_collision = FALSE;
            IEC0bits.U1RXIE = 1;
        }
        return c;
    }
    return 0; /*no data available*/
}

/**
 * @Function int8_t put_char(unsigned char c) 
 * @param c, unsigned char to put into transmit buffer
 * @return SUCCESS or ERROR
 * @brief 
 * @author Aaron Hunter */
char put_char(unsigned char c) {
    if (is_buffer_full(txp) == FALSE) { //if the buffer isn't full
        writing_tx_buffer = TRUE; // set flag so that interrupt can't access the buffer
        write_buffer(txp, c); // write to buffer 
        writing_tx_buffer = FALSE; // allow buffer access
        /*if interrupt isn't set, it needs to be enabled.  Note this is slightly
         * different from get_char because the module starts with no interrupts
         * so there are two reasons for disabled interrupts:  empty transmit
         * buffer or a collision.  Either way we need to re-enable them */
        if (IEC0bits.U1TXIE == 0) {
            tx_collision = FALSE;
            IEC0bits.U1TXIE = 1;
        }
        return SUCCESS;
    } else {
        return ERROR; //buffer is full!
    }
}

/**
 * @Function _mon_putc(char c)
 * @param c - char to be sent
 * @return None.
 * @brief  overwrites weakly define extern to use circular buffer instead of Microchip 
 * functions
 * @author Max Dunne, 2011.11.10 */
void _mon_putc(char c) {
    put_char(c);
}

/*******************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATIONS                                            *
 ******************************************************************************/

static void init_buffer(struct circular_buffer *buf) {
    int i;
    buf->read_index = 0; /*initialize read index to 0 */
    buf->write_index = 0; /*initialize write index to 0 */
    buf->size = BUFFER_LENGTH; /*Set size to buffer length const*/
    for (i = 0; i < BUFFER_LENGTH; i++) { /*initialize data to zero*/
        buf->data[i] = 0;
    } /*end for */
}

/* function int is_buffer_empty(struct circular_buffer *buf)
 * takes a pointer to a circular buffer and compares the read and write indices
 * if they are equal then the buffer is empty
 */
static int8_t is_buffer_empty(struct circular_buffer *buf) {
    if (buf->read_index == buf->write_index) { //if read = write then the buffer is empty
        return TRUE;
    }
    return FALSE;
}

/*  bufFull(struct oBuffer *buf)
 * takes a pointer to a circular buffer and compares the read and write indices
 * if write+1 = read, then the buffer is full. 
 */
static int8_t is_buffer_full(struct circular_buffer *buf) {
    /* write index +1 == read index is full,  the mod provides wrap around*/
    if ((buf->write_index + 1) % BUFFER_LENGTH == buf->read_index) {
        return TRUE;
    }
    return FALSE;
}

/* writeBuffer( (struct oBuffer *buf, unsigned char c)
 * takes a pointer to a circular buffer and a char to be written
 * returns SUCCESS or ERROR
 */
static int8_t write_buffer(struct circular_buffer *buf, unsigned char c) {
    if (is_buffer_full(buf) == FALSE) {
        buf->data[buf->write_index] = c;
        /*increment the write index and wrap using modulus arithmetic */
        buf->write_index = (buf->write_index + 1) % BUFFER_LENGTH;
        return SUCCESS;
    }
    return ERROR; /*no data written*/
}

/*int readBuffer(struct oBuffer *buf)
 * takes a pointer to a circular buffer   
 * returns the value from the buffer
 * the read index is incremented and wrapped using modulus arithmetic
 * Returns 0 if the buffer is empty or the pointer is invalid
 */
static unsigned char read_from_buffer(struct circular_buffer *buf) {
    unsigned char val;
    if (is_buffer_empty(buf) == FALSE) {
        val = buf->data[buf->read_index]; //get the char from the buffer
        /*increment the read index and wrap using modulus arithmetic*/
        buf->read_index = (buf->read_index + 1) % BUFFER_LENGTH;
        return val;
    }
    return 0;
}

static int get_num_elements(struct circular_buffer *buf) {
    if (buf != NULL) {
        if (buf->write_index < buf->read_index) { /*test for wrap around*/
            return (buf->write_index + BUFFER_LENGTH - buf->read_index);
        } else {
            return (buf->write_index - buf->read_index);
        }
    }
    return 0;
}

#ifdef SERIAL_TESTING

int main(void) {
    char msg[MESSAGE_LENGTH];
    char c;
    int i;
    int lines = 100;
    Board_init();
    //set LEDs for troubleshooting
    //    TRISAbits.TRISA4 = 0; //pin 72 Max32
    //    TRISAbits.TRISA3 = 0; //built-in LED, pin 13
    //    LATACLR = 0x18;
    Serial_init();
    printf("\r\nSerial test harness %s, %s \r\n", __DATE__, __TIME__);
    printf("System clock freq: %d (Hz)\r\n", Board_get_sys_clock());
    printf("Peripheral clock freq: %d (Hz)\r\n", Board_get_PB_clock());
    for(i=0;i<lines;i++){
        printf("Printing line %d \r", i);
    }
    printf("\nPrinted %d lines \r\n", i);
    printf("Everything after this line will be echoed: \r\n");
    printf("**********************************************\r\n");
    while (1) {
        if (c = get_char()) {
            put_char(c);
        }
    }
    return 0;
}
#endif //SERIAL_TESTING



