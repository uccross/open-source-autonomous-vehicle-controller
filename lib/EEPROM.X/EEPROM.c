/* 
 * File:   EEPROM.c
 * Author: Aaron Hunter
 * Brief: EEPROM module functions
 * Created on May 6, 2021, 11:48 am
 * Modified on <month> <day>, <year>, <hour> <pm/am>
 */

/*******************************************************************************
 * #INCLUDES                                                                   *
 ******************************************************************************/
#include "EEPROM.h" // The header file for this source file. 
#include "Board.h"  //Max 32 dev board 
#include "SerialM32.h" // serial communications via USB
#include "System_timer.h" // Millisecond and micrsecond hardware timer
#include "xc.h" // compiler
#include <stdio.h>
#include <string.h>
#include <sys/attribs.h>  //for ISR definitions
#include <proc/p32mx795f512l.h> //Max 32 specific info

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/
#define EEPROM_I2C_ADDR 0x50  //device address
#define ACK 0
#define NACK 1
#define READ 1
#define WRITE 0
#define PAGESIZE 64  //number of bytes per page
#define NUMPAGES 512  //number of pages
#define LOG64 6 //number of bits to shift page number to get to its address
#define NUMSHORTS PAGESIZE >> 2
#define NUMINTS PAGESIZE >> 4
#define NUMLONGS PAGESIZE >> 8
#define NUMFLOATS PAGESIZE >> 4
#define NUMDOUBLES PAGESIZE >> 8
#define SHORT2BYTE 1
#define INT2BYTE 2

/*******************************************************************************
 * PRIVATE TYPEDEFS                                                            *
 ******************************************************************************/
typedef enum {
    EEPROM_START,
    EEPROM_SEND_DEV_ADDR_W,
    EEPROM_SEND_MEM_ADDR_H,
    EEPROM_SEND_MEM_ADDR_L,
    EEPROM_SEND_DATA,
    EEPROM_RESTART,
    EEPROM_SEND_DEV_ADDR_R,
    EEPROM_READ_DATA,
    EEPROM_ACK_DATA,
    EEPROM_NACK_DATA,
    EEPROM_STOP,
} EEPROM_I2C_states_t; //state machine states during I2C

typedef struct {
    uint8_t mem_high_byte; //high byte memory address
    uint8_t mem_low_byte; //low byte memory address
    uint8_t mode; //read or write
    uint8_t* source_data;
    uint8_t length;
    uint8_t index;
} EEPROM_settings_t;

static EEPROM_settings_t EEPROM_settings;

/*EEPROM_error set to TRUE when chip fails to ACK, this usually means the device
 is in a write cycle, but can indicate a device or address failure*/
static uint8_t EEPROM_error = FALSE;

/*we don't want to access EEPROM if in a write or read cycle cleared by state
 machine*/
static uint8_t EEPROM_busy = FALSE;

/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                 *
 ******************************************************************************/
/**
 * @Function static void __ISR(_I2C1_VECTOR, IPL2AUTO) EEPROM_interrupt_handler(void) 
 * @brief Interrupt handler for I2C1 master
 * @author Aaron Hunter
 */
static void __ISR(_I2C1_VECTOR, IPL2AUTO) EEPROM_interrupt_handler(void);

/**
 * @Function EEPROM_run_I2C_state_machine(void)
 * @param none
 * @return none
 * @brief Calls state machine for I2C1 transaction and clears interrupt
 * @note 
 * @author Aaron Hunter
 * @modified
 */
static void EEPROM_run_I2C_state_machine(void);

/**
 * @Function 
 * @param uint8_t data[], pointer to data
 * @param uint8_t length, number of bytes to write <= PAGESIZE
 * @param, uint32_t page < NUMPAGES, which page to write the data into
 * @param, uint32_t offset <PAGESIZE, local address within the page
 * @return SUCCESS or ERROR
 * @brief writes 1 to PAGESIZE bytes to EEPROM
 * @author Aaron Hunter
 */
static int8_t EEPROM_write_data(uint8_t data[], uint8_t length, uint32_t page, uint32_t offset);

/**
 * @Function 
 * @param uint8_t data[], where to store retrieved data
 * @param uint8_t length, number of bytes to read <= PAGESIZE
 * @param, uint32_t page < NUMPAGES, which page to read the data from
 * @param, uint32_t offset <PAGESIZE, local address within the page
 * @return SUCCESS or ERROR
 * @brief reads 1 to PAGESIZE bytes from EEPROM at specified location
 * @note 
 * @author Aaron Hunter
 */
static int8_t EEPROM_read_data(uint8_t data[], uint8_t length, uint32_t page, uint32_t offset);


/*******************************************************************************
 * PUBLIC FUNCTION IMPLEMENTATIONS                                             *
 ******************************************************************************/

/**
 * @Function EEPROM_init(void)
 * @param none
 * @return SUCCESS or ERROR
 * @brief initializes I2C1 peripheral to 100 KHz and sets up interrupt
 * @note 
 * @author Aaron Hunter,
 * @modified 
 */
uint8_t EEPROM_init(void) {
    __builtin_disable_interrupts();
    /*config EERPROM I2C interrupt*/
    /*config priority and sub-priority--must match IPL level*/
    IPC6bits.I2C1IP = 2; //priority 2
    IPC6bits.I2C1IS = 1; //sub-priority 1
    /*clear interrupt flag*/
    IFS0bits.I2C1MIF = 0; //clear flag
    /*enable I2C master interrupt*/
    IEC0bits.I2C1MIE = 1;
    __builtin_enable_interrupts();
    I2C1CON = 0; //reset config
    I2C1BRG = 98; //set baud rate to 400KHz
    I2C1CONbits.ON = 1; //enable I2C
    return SUCCESS;
}

/**
 * @Function EEPROM_is_busy(void)
 * @param none
 * @return TRUE or FALSE
 * @brief if EEPROM in write or read state returns TRUE
 * @note 
 * @author Aaron Hunter,
 * @modified 
 */
uint8_t EEPROM_is_busy(void) {
    return EEPROM_busy;
}

/**
 * @Function EEPROM_is_error(void)
 * @param none
 * @return TRUE or FALSE
 * @brief if EEPROM fails to ACK its address returns TRUE
 * @note 
 * @author Aaron Hunter,
 * @modified 
 */
uint8_t EEPROM_is_error(void) {
    return EEPROM_error;
}

/**
 * @Function 
 * @param uint8_t data[], pointer to data
 * @param uint8_t length, number of bytes to write <= PAGESIZE (64 bytes)
 * @param, uint32_t page < NUMPAGES (512), which page to write the data into
 * @param, uint32_t offset <PAGESIZE, local address within the page
 * @return SUCCESS or ERROR
 * @brief writes 1 to PAGESIZE bytes to EEPROM
 * @author Aaron Hunter
 */
int8_t EEPROM_write_byte_array(uint8_t data[], uint8_t length, uint32_t page, uint32_t offset) {
    return EEPROM_write_data(data, length, page, offset);
}

/**
 * @Function  EEPROM_write_short_array(uint16_t data[], uint8_t length, uint32_t page, uint32_t offset)
 * @param uint16_t data[], pointer to data
 * @param uint8_t array length)
 * @param, uint32_t page < NUMPAGES (512), which page to write the data into
 * @param, uint32_t offset <PAGESIZE, local address within the page
 * @return SUCCESS or ERROR
 * @brief casts the 16 bit data into 8 bit and passes to the internal method for storage
 * @author Aaron Hunter
 */
int8_t EEPROM_write_short_array(int16_t data[], uint8_t length, uint32_t page, uint32_t offset) {
    length = length << SHORT2BYTE; //increase length variable for short type
    /*cast the data into the correct type and send to the EEPROM*/
    return EEPROM_write_data((uint8_t*) data, length, page, offset);
}

/**
 * @Function  EEPROM_write_int_array(int32_t data[], uint8_t length, uint32_t page, uint32_t offset)
 * @param uint16_t data[], pointer to data
 * @param uint8_t length, number of 32 bit ints to write
 * @param, uint32_t page < NUMPAGES (512), which page to write the data into
 * @param, uint32_t offset <PAGESIZE, local address within the page
 * @return SUCCESS or ERROR
 * @brief writes 1 to PAGESIZE bytes to EEPROM
 * @author Aaron Hunter
 */
int8_t EEPROM_write_int_array(int32_t data[], uint8_t length, uint32_t page, uint32_t offset) {
    length = length << INT2BYTE; //increase length variable for short type
    /*cast the data into the correct type and send to the EEPROM*/
    return EEPROM_write_data((uint8_t*) data, length, page, offset);
}

/*******************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATIONS                                            *
 ******************************************************************************/

/**
 * @Function 
 * @param uint8_t data[], pointer to data
 * @param uint8_t length, number of bytes to write <= PAGESIZE
 * @param, uint32_t page < NUMPAGES, which page to write the data into
 * @param, uint32_t offset <PAGESIZE, local address within the page
 * @return SUCCESS or ERROR (see note)
 * @brief writes 1 to PAGESIZE bytes to EEPROM
 * @note ERROR means the size of the data array is too large or the address is 
 * outside the address space of the EEPROM
 * @author Aaron Hunter
 */
static int8_t EEPROM_write_data(uint8_t data[], uint8_t length, uint32_t page, uint32_t offset) {
    uint32_t address;
    EEPROM_busy = TRUE;
    if ((length <= PAGESIZE)&& (page < NUMPAGES) && (offset + length < PAGESIZE)) {
        address = (page << LOG64 | offset); //set the address of data
        EEPROM_settings.mem_high_byte = (uint8_t) (address >> 8); //mask off high byte
        EEPROM_settings.mem_low_byte = (uint8_t) address; //mask off low byte
        EEPROM_settings.mode = WRITE;
        EEPROM_settings.source_data = data;
        EEPROM_settings.length = length;
        EEPROM_settings.index = 0;
    } else {
        printf("Bad address or array too large\r\n");
        return ERROR;
    }
    /*no errors, so kick of the transfer*/
    I2C1CONbits.SEN = 1; // send start command
    return SUCCESS;
}

/**
 * @Function 
 * @param uint8_t data[], where to store retrieved data
 * @param uint8_t length, number of bytes to read <= PAGESIZE
 * @param, uint32_t page < NUMPAGES, which page to read the data from
 * @param, uint32_t offset <PAGESIZE, local address within the page
 * @return SUCCESS or ERROR
 * @brief reads 1 to PAGESIZE bytes from EEPROM at specified location
 * @note 
 * @author Aaron Hunter
 */
static int8_t EEPROM_read_data(uint8_t data[], uint8_t length, uint32_t page, uint32_t offset) {
    uint32_t address;
    EEPROM_busy = TRUE;
    if ((length <= PAGESIZE)&& (page < NUMPAGES) && (offset + length < PAGESIZE)) {
        address = (page << LOG64 | offset); //set the address of data
        EEPROM_settings.mem_high_byte = (uint8_t) (address >> 8); //mask off high byte
        EEPROM_settings.mem_low_byte = (uint8_t) address; //mask off low byte
        EEPROM_settings.mode = READ;
        EEPROM_settings.source_data = data;
        EEPROM_settings.length = length;
        EEPROM_settings.index = 0;
    } else {
        return ERROR;
    }
    I2C1CONbits.SEN = 1; // send start command
    return SUCCESS;
}

/**
 * @Function static void __ISR(_I2C1_VECTOR, IPL2AUTO) EEPROM_interrupt_handler(void) 
 * @brief Interrupt handler for I2C1 master
 * @author Aaron Hunter
 */
static void __ISR(_I2C1_VECTOR, IPL2AUTO) EEPROM_interrupt_handler(void) {
    EEPROM_run_I2C_state_machine();
    IFS0bits.I2C1MIF = 0; //clear flag
}

/**
 * @Function EEPROM_run_I2C_state_machine(void)
 * @param none
 * @return none
 * @brief Calls state machine for I2C1 transaction and clears interrupt
 * @note 
 * @author Aaron Hunter
 * @modified
 */
static void EEPROM_run_I2C_state_machine(void) {
    static EEPROM_I2C_states_t current_state = EEPROM_START;
    EEPROM_I2C_states_t next_state = EEPROM_START;

    switch (current_state) {
        case(EEPROM_START): /*we get here after the start condition is sent*/
            if (EEPROM_error == TRUE) {
                printf("No ACK from device!\r\n");
            }
            next_state = EEPROM_SEND_DEV_ADDR_W;
            /*load device address into I2C transmit buffer with write bit set*/
            I2C1TRN = (EEPROM_I2C_ADDR << 1 | WRITE);
            break;
        case(EEPROM_SEND_DEV_ADDR_W):
            if (I2C1STATbits.ACKSTAT == NACK) { // check for ack
                EEPROM_error = TRUE;
                I2C1CONbits.PEN = 1; //send stop condition 
                next_state = EEPROM_STOP;
            } else {
                EEPROM_error = FALSE; // device has ACKED, so I2C working
                /*load high memory address byte into I2C transmit buffer*/
                I2C1TRN = EEPROM_settings.mem_high_byte;
                next_state = EEPROM_SEND_MEM_ADDR_H;
            }
            break;
        case(EEPROM_SEND_MEM_ADDR_H):
            if (I2C1STATbits.ACKSTAT == NACK) { // check for ack
                EEPROM_error = TRUE;
                I2C1CONbits.PEN = 1; //send stop condition 
                next_state = EEPROM_STOP;
            } else {
                /*load low memory address byte into I2C transmit buffer*/
                I2C1TRN = EEPROM_settings.mem_low_byte;
                next_state = EEPROM_SEND_MEM_ADDR_L;
            }
            break;
        case(EEPROM_SEND_MEM_ADDR_L):
            if (I2C1STATbits.ACKSTAT == NACK) { // check for ack
                EEPROM_error = TRUE;
                I2C1CONbits.PEN = 1; //send stop condition 
                next_state = EEPROM_STOP;
            } else {
                /*Branch to write or read data*/
                if (EEPROM_settings.mode == WRITE) { /*write branch*/
                    I2C1TRN = EEPROM_settings.source_data[EEPROM_settings.index]; //send first byte
                    EEPROM_settings.index++; //increment the data index
                    next_state = EEPROM_SEND_DATA;
                } else { /*read branch */
                    I2C1CONbits.RSEN = 1; /*send restart condition*/
                    next_state = EEPROM_RESTART;
                }
            }
            break;
        case(EEPROM_SEND_DATA): //write branch 
            if (I2C1STATbits.ACKSTAT == NACK) { // check for ack
                EEPROM_error = TRUE;
                I2C1CONbits.PEN = 1; //send stop condition 
                next_state = EEPROM_STOP;
            } else { /*data was ACKD so proceed with transaction*/
                if (EEPROM_settings.index < EEPROM_settings.length) { //if more data to be written
                    I2C1TRN = EEPROM_settings.source_data[EEPROM_settings.index]; //send next byte
                    EEPROM_settings.index++; //increment the data index
                    next_state = EEPROM_SEND_DATA; /*next state is EEPROM_SEND_DATA*/
                } else { // else stop 
                    next_state = EEPROM_STOP;
                    I2C1CONbits.PEN = 1; //send stop condition 
                }
            }
            break;
        case(EEPROM_RESTART): //read branch 
            I2C1TRN = (EEPROM_I2C_ADDR << 1 | READ); //send device address with read bit set
            next_state = EEPROM_SEND_DEV_ADDR_R;
            break;
        case(EEPROM_SEND_DEV_ADDR_R):
            if (I2C1STATbits.ACKSTAT == NACK) { // check for ack
                EEPROM_error = TRUE;
                I2C1CONbits.PEN = 1; //send stop condition 
                next_state = EEPROM_STOP;
            } else {
                /*ACK received, EEPROM will send one byte*/
                I2C1CONbits.RCEN = 1; //setting this bit initiates a receive.
                next_state = EEPROM_READ_DATA;
            }
            break;
        case(EEPROM_READ_DATA): //we always enter here after a receive interrupt
            EEPROM_settings.source_data[EEPROM_settings.index] = I2C1RCV;
            EEPROM_settings.index++; /*increment byte index*/
            if (EEPROM_settings.index < EEPROM_settings.length) {
                I2C1CONbits.ACKDT = 0; /*byte received set ACK bit*/
                I2C1CONbits.ACKEN = 1; /*send ACK to EEPROM*/
                next_state = EEPROM_ACK_DATA;
            } else { /*last byte received so NACK byte*/
                I2C1CONbits.ACKDT = 1; /*set NACK bit*/
                I2C1CONbits.ACKEN = 1; /*send NACK to EEPROM*/
                next_state = EEPROM_NACK_DATA;
            }
            break;
        case(EEPROM_ACK_DATA):
            I2C1CONbits.RCEN = 1; //setting this bit initiates a receive.
            next_state = EEPROM_READ_DATA;
            break;
        case(EEPROM_NACK_DATA):
            I2C1CONbits.PEN = 1; //send stop condition 
            next_state = EEPROM_STOP;
            break;

        case(EEPROM_STOP):
            /*stop sent so return to START state*/
            next_state = EEPROM_START;
            EEPROM_busy = FALSE;
            break;
        default:
            EEPROM_error = TRUE;
            next_state = EEPROM_START;
            break;
    }
    current_state = next_state;
}


#ifdef EEPROM_TESTING

void main(void) {
    int startTime;
    int write_time = 6000; //microsecs
    Board_init();
    Serial_init();
    Sys_timer_init();
    EEPROM_init();

    uint8_t test_data[] = {0xde, 0xad, 0xbe, 0xef};
    uint8_t test2_data[] = {1, 2, 4, 8};
    int16_t shorts[] = {0xaacc, 0xff00, 0xbbdd, 0xff00, 0x2112, 0xff00, 0x0202, 0x4884};
    int32_t ints[] = {0xaaccff00, 0xbbddff00, 0x2112ff00, 0x02024884};
    uint8_t bytes[] = {0xff, 0xca, 0x00, 0xff, 0x02, 0x02, 0x08, 0x08, 0xbb, 0xdd, 0xff, 0x00};

    uint8_t recv_data[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t index = 0;

    printf("EEPROM Test Harness,  %s, %s\r\n", __DATE__, __TIME__);
    startTime = Sys_timer_get_usec();
    EEPROM_write_data(test_data, 4, 510, 0);
    while (EEPROM_is_busy() == TRUE) {
        ;
    }
    printf("ISR write time (usec): %d\r\n", Sys_timer_get_usec() - startTime);
    startTime = Sys_timer_get_usec();
    while (Sys_timer_get_usec() - startTime < write_time) {
        ;
    }
    printf("u-secs busy: %d\r\n", Sys_timer_get_usec() - startTime);
    EEPROM_write_data(test2_data, 4, 510, 4);
    startTime = Sys_timer_get_usec();
    while (Sys_timer_get_usec() - startTime < write_time) {
        ;
    }
    EEPROM_read_data(recv_data, 8, 510, 0);
    while (EEPROM_is_busy() == TRUE) {
        ;
    }
    printf("byte array test:\t");
    for (index = 0; index < EEPROM_settings.length; index++) {
        printf("0x%x \t", recv_data[index]);
    }
    printf("\r\n");
    printf("return: %d\r\n", EEPROM_write_short_array(shorts, 8, 128, 0));
    //    printf("return: %d\r\n",EEPROM_write_byte_array((uint8_t*)shorts, 12, 128, 0));
    startTime = Sys_timer_get_usec();
    while (Sys_timer_get_usec() - startTime < write_time) {
        ;
    }
    EEPROM_read_data(recv_data, 16, 128, 0);
    startTime = Sys_timer_get_usec();
    while (Sys_timer_get_usec() - startTime < write_time) {
        ;
    }
    printf("EEPROM error = %d\r\n", EEPROM_error);
    printf("post data:\t");
    for (index = 0; index < EEPROM_settings.length; index++) {
        printf("0x%x \t", recv_data[index]);
    }
    printf("\r\n");
    printf("return: %d\r\n", EEPROM_write_int_array(ints, 4, 129, 0));
    //    printf("return: %d\r\n",EEPROM_write_byte_array((uint8_t*)shorts, 12, 128, 0));
    startTime = Sys_timer_get_usec();
    while (Sys_timer_get_usec() - startTime < write_time) {
        ;
    }
    EEPROM_read_data(recv_data, 16, 129, 0);
    startTime = Sys_timer_get_usec();
    while (Sys_timer_get_usec() - startTime < write_time) {
        ;
    }
    printf("EEPROM error = %d\r\n", EEPROM_error);
    printf("post data:\t");
    for (index = 0; index < EEPROM_settings.length; index++) {
        printf("0x%x \t", recv_data[index]);
    }
    printf("\r\n");
    while (1) {
        ;
    }

}
#endif


