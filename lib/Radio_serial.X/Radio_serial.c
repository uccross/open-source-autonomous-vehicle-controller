/* 
 * File:   Radio_serial.c
 * Author: Aaron Hunter
 * Brief: 
 * Created on 01/21/2021 12:42 pm
 * Modified on 
 */

/*******************************************************************************
 * #INCLUDES                                                                   *
 ******************************************************************************/

#include "Radio_serial.h" // The header file for this source file. 
#include "SerialM32.h" //debug serial
#include "Board.H"   //Max32 setup      
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/attribs.h>  //for ISR definitions
#include <xc.h>

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/
#define BUFFER_LENGTH 2048
#define RADIO_BAUD_RATE 115200

/*******************************************************************************
 * PRIVATE TYPEDEFS                                                            *
 ******************************************************************************/

struct circular_buffer {
    int16_t read_index;
    int16_t write_index;
    int16_t size;
    unsigned char data[BUFFER_LENGTH];
}; /*circular  buffer */

static struct circular_buffer tx_buffer; /*Circular UART TX buffer */
static struct circular_buffer rx_buffer; /*Circular UART RX buffer */

/*pointers to the buffers so we don't pass the entire buffer to the functions*/
static struct circular_buffer *txp = &tx_buffer;
static struct circular_buffer *rxp = &rx_buffer;

static int8_t tx_collision = FALSE;
static int8_t rx_collision = FALSE;
static int8_t writing_tx_buffer = FALSE;
static int8_t reading_rx_buffer = FALSE;
static int8_t new_char_avail = FALSE;
/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                 *
 ******************************************************************************/
/**
 * @Function init_buffer(struct circular_buffer *buf)
 * @param buf, pointer to a circular buffer
 * @return none
 * @brief  initializes the circular buffer struct
 * @author Aaron Hunter
 */
static void init_buffer(struct circular_buffer *buf);

/**
 * @Function is_buffer_empty(struct circular_buffer *buf);
 * @param buf, circular buffer pointer
 * @brief checks if the buffer is empty
 * @returns TRUE or FALSE
 */
static int8_t is_buffer_empty(struct circular_buffer *buf);

/**  
 * @Function is_buffer_full(struct oBuffer *buf)
 * @param circular buffer pointer
 * @brief takes a pointer to a circular buffer and checks if it is full 
 * @return TRUE or FALSE
 */
static int8_t is_buffer_full(struct circular_buffer *buf);

/** 
 * @Function write_buffer( (struct oBuffer *buf, unsigned char c)
 * @param circular buffer pointer
 * @param unsigned char 
 * @brief writes the char to the circular buffer reference
 * returns SUCCESS or ERROR
 **/
static int8_t write_buffer(struct circular_buffer *buf, unsigned char c);

/**
 * @Function int read_from_buffer(struct oBuffer *buf)
 * @param circular buffer pointer   
 * @brief returns char at read index from the buffer
 * @return char at index or  0 if the buffer is empty or the pointer is invalid
 */
static unsigned char read_from_buffer(struct circular_buffer *buf);

/**
 * @Function get_num_elements(struct circular buffer *buf)
 * @param buf, pointer to circular buffer
 * @brief returns the number of elements in buf
 * @return returns the number of elements in buf
 */
static int16_t get_num_elements(struct circular_buffer *buf);


/*******************************************************************************
 * PUBLIC FUNCTION IMPLEMENTATIONS                                             *
 ******************************************************************************/

/**
 * @Function Radio_serial_init(void)
 * @param none
 * @return none
 * @brief  Initializes the UART subsystem for the SiK radio link
 * @author Aaron Hunter*/
void Radio_serial_init(void) {
    /*Initialize the circular buffers*/
    init_buffer(rxp);
    init_buffer(txp);
    /* turn off UART while configuring */
    U4MODEbits.ON = 0;
    __builtin_disable_interrupts();
    U4MODEbits.BRGH = 0; //M = 16
    U4BRG = ((Board_get_PB_clock() / RADIO_BAUD_RATE) / 16) - 1; //set baud rate
    /* configure the RX and TX pins */
    U4STAbits.UTXEN = 1;
    U4STAbits.URXEN = 1;
    /*clear any overflow*/
    if (U4STAbits.OERR == 1) {
        U4STAbits.OERR = 0;
    }
    /* configure using software flow control, if set to 2 it would be CTS/RTS */
    // JU4,U5,U6 only have software flow control (no DTR, CTS pins)
    /*configure UART interrupts */
    U4STAbits.URXISEL = 0x0; //interrupt when buffer is not empty
    U4STAbits.UTXISEL = 0x01; // int when all characters are sent (TRMT == TRUE)
    IFS2bits.U4RXIF = 0; //clear interrupt flags
    IFS2bits.U4TXIF = 0;
    IPC12bits.U4IP= 2; //set interrupt priority to 1 
    IEC2bits.U4RXIE = 1; //enable interrupt on RX
    IEC2bits.U4TXIE = 1; //enable interrupts on TX
    /* turn on UART*/
    U4MODEbits.ON = 1;
    __builtin_enable_interrupts();
}

/**
 * @Function Radio_put_char(char c)
 * @param c, the char to be sent to serial port
 * @return SUCCESS or ERROR
 * @brief  adds the char to the tail of the transmit buffer
 * @author Aaron Hunter*/
char Radio_put_char(unsigned char c){
    if (is_buffer_full(txp) == FALSE) { //if the buffer isn't full
        writing_tx_buffer = TRUE; // set flag so that interrupt can't access the buffer
        write_buffer(txp, c); // write to buffer 
        writing_tx_buffer = FALSE; // allow buffer access
        /*if interrupt isn't set, it needs to be enabled.  Note this is slightly
         * different from get_char because the module starts with no interrupts
         * so there are two reasons for disabled interrupts:  empty transmit
         * buffer or a collision.  Either way we need to re-enable them */
        if (IEC2bits.U4TXIE == 0) {
            tx_collision = FALSE;
            IEC2bits.U4TXIE = 1; //enable interrupts on TX
        }
        return SUCCESS;
    } else {
        return ERROR; //buffer is full!
    }
}

/**
 * @Function Radio_get_char(void)
 * @return character read from receive buffer
 * @brief  returns the value in the receive buffer
 * @author Aaron Hunter*/
unsigned char Radio_get_char(void){
    unsigned char c;
    if (is_buffer_empty(rxp) == FALSE) {
        reading_rx_buffer = TRUE; /*set buffer access flag*/
        c = read_from_buffer(rxp);
        reading_rx_buffer = FALSE;
        //if a collision occurred we need to re-enable interrupts
        if (rx_collision == TRUE) {
            rx_collision = FALSE;
            IEC2bits.U4RXIE = 1;
        }
        return c;
    }
    return 0; /*no data available*/
}

/**
 * @Function Radio_data_available();
 * @return TRUE or FALSE
 * @brief responds with TRUE if the buffer is not empty, FALSE otherwise
 * @author Aaron Hunter*/
unsigned char Radio_data_available(void){
    return(!is_buffer_empty(rxp));
}

/*******************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATIONS                                            *
 ******************************************************************************/

void __ISR(_UART_4_VECTOR, IPL2AUTO) U4_interrupt_handler(void) {
    if (IFS2bits.U4RXIF) { //check for received data flag
        if (is_buffer_full(rxp) == FALSE) {
            if (reading_rx_buffer == FALSE) {
                write_buffer(rxp, U4RXREG);
            } else {
                /*a collision occurred need to disable interrupts to exit ISR*/
                /*it will be re-enabled in get_char() */
                rx_collision = TRUE;
                IEC2bits.U4RXIE = 0;
            }
        }
        IFS2bits.U4RXIF = 0; // clear the flag
    } else if (IFS2bits.U4TXIF) { /*check for transmission flag*/
        if (is_buffer_empty(txp) == TRUE) { //if tx circular buffer is empty
            IEC2bits.U4TXIE = 0; //disable interrupts on TX
        } else if (writing_tx_buffer == FALSE) { /*If not being accessed */
            U4TXREG = read_from_buffer(txp);
        } else {
            /*a collision occurred, turn off interrupt to exit ISR*/
            /*it will be re-enabled in put_char()*/
            tx_collision = TRUE;
            IEC2bits.U4TXIE = 0;
        }
        IFS2bits.U4TXIF = 0; /*clear the interrupt flag*/
    } else if (IFS2bits.U4EIF) { //error flag--can check U1STA for the reason
        if (U4STAbits.OERR == 1) {
            U4STAbits.OERR = 0; //over run error is the only one we can write to clear
        }
        IFS2bits.U4EIF = 0;
    }
}

/**
 * @Function init_buffer(struct circular_buffer *buf)
 * @param buf, pointer to a circular buffer
 * @return none
 * @brief  initializes the circular buffer struct
 * @author Aaron Hunter
 */
static void init_buffer(struct circular_buffer *buf) {
    int i;
    buf->read_index = 0; /*initialize read index to 0 */
    buf->write_index = 0; /*initialize write index to 0 */
    buf->size = BUFFER_LENGTH; /*Set size to buffer length const*/
    for (i = 0; i < BUFFER_LENGTH; i++) { /*initialize data to zero*/
        buf->data[i] = 0;
    } /*end for */
}

/**
 * @Function is_buffer_empty(struct circular_buffer *buf);
 * @param buf, circular buffer pointer
 * @brief checks if the buffer is empty
 * @returns TRUE or FALSE
 */
static int8_t is_buffer_empty(struct circular_buffer *buf) {
    if (buf->read_index == buf->write_index) { //if read = write then the buffer is empty
        return TRUE;
    }
    return FALSE;
}

/**  
 * @Function is_buffer_full(struct oBuffer *buf)
 * @param circular buffer pointer
 * @brief takes a pointer to a circular buffer and checks if it is full 
 * @return TRUE or FALSE
 */
static int8_t is_buffer_full(struct circular_buffer *buf) {
    /* write index +1 == read index is full,  the mod provides wrap around*/
    if ((buf->write_index + 1) % BUFFER_LENGTH == buf->read_index) {
        return TRUE;
    }
    return FALSE;
}

/** 
 * @Function write_buffer( (struct oBuffer *buf, unsigned char c)
 * @param circular buffer pointer
 * @param unsigned char 
 * @brief writes the char to the circular buffer reference
 * returns SUCCESS or ERROR
 **/
static int8_t write_buffer(struct circular_buffer *buf, unsigned char c) {
    if (is_buffer_full(buf) == FALSE) {
        buf->data[buf->write_index] = c;
        /*increment the write index and wrap using modulus arithmetic */
        buf->write_index = (buf->write_index + 1) % BUFFER_LENGTH;
        return SUCCESS;
    }
    return ERROR; /*no data written*/
}

/**
 * @Function int read_from_buffer(struct oBuffer *buf)
 * @param circular buffer pointer   
 * @brief returns char at read index from the buffer
 * @return char at index or  0 if the buffer is empty or the pointer is invalid
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

/**
 * @Function get_num_elements(struct circular buffer *buf)
 * @param buf, pointer to circular buffer
 * @brief returns the number of elements in buf
 * @return returns the number of elements in buf
 */
static int16_t get_num_elements(struct circular_buffer *buf) {
    if (buf != NULL) {
        if (buf->write_index < buf->read_index) { /*test for wrap around*/
            return (buf->write_index + BUFFER_LENGTH - buf->read_index);
        } else {
            return (buf->write_index - buf->read_index);
        }
    }
    return 0;
}



#ifdef RADIO_TESTING

int main(void) {
    unsigned char msg[BUFFER_LENGTH];
    char c;
    int i;
    Board_init();
    Serial_init();
    Radio_serial_init();

    printf("\r\nRadio serial test harness %s, %s \r\n", __DATE__, __TIME__);
    printf("System clock freq: %d (Hz)\r\n", Board_get_sys_clock());
    printf("Peripheral clock freq: %d (Hz)\r\n", Board_get_PB_clock());
    printf("Everything in Radio serial port window will be echoed here: \r\n");
    printf("**********************************************\r\n");
    sprintf(msg, "Testing Radio Serial Port\r\n");
    for(i = 0;i < strlen(msg); i++ ){
        Radio_put_char(msg[i]);
    }
    printf("elements in TX circular buffer %d\r\n", get_num_elements(txp));
    printf("Is TX buffer empty? %d \r\n", is_buffer_empty(txp));
    printf("Is TX buffer full? %d \r\n", is_buffer_full(txp));
    
    while (1) {
        if(Radio_data_available() == TRUE){
        c = Radio_get_char();
            Radio_put_char(c); //send via the radio
            put_char(c); //send over USB/debug also
        }
    }
    return 0;
}



#endif