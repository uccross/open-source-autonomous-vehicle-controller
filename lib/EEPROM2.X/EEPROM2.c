/* 
 * File:   EEPROM2.c
 * Author: Aaron Hunter
 * Brief: Blocking version of EEPROM library
 * Created on 06/15/2021 7:41 am
 * Modified on 
 */

/*******************************************************************************
 * #INCLUDES                                                                   *
 ******************************************************************************/

#include "EEPROM2.h" // The header file for this source file. 
#include "Board.h"  //Max 32 dev board 
#include "SerialM32.h" // serial communications via USB
#include "xc.h"
#include "System_timer.h" // compiler
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
#define SHORT2BYTE 1
#define INT2BYTE 2
#define LONG2BYTE 3
#define FLOAT2BYTE 2
#define DOUBLE2BYTE 3

/*******************************************************************************
 * PRIVATE TYPEDEFS                                                            *
 ******************************************************************************/
typedef struct {
    uint8_t mem_high_byte; //high byte memory address
    uint8_t mem_low_byte; //low byte memory address
    uint8_t mode; //read or write
    uint8_t* source_data;
    uint8_t length;
    uint8_t index;
} EEPROM_settings_t;

static EEPROM_settings_t EEPROM_settings;
/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                 *
 ******************************************************************************/

/**
 * @Function EEPROM_start(void)
 * @return none
 * @brief sends start bit to I2C bus
 * @note 
 * @author Aaron Hunter
 */
static void EEPROM_start(void);

/**
 * @Function EEPROM_stop(void)
 * @return none
 * @brief sends stop bit to I2C bus
 * @note 
 * @author Aaron Hunter
 */
static void EEPROM_stop(void);

/**
 * @Function EEPROM_restart(void)
 * @return none
 * @brief sends restart bit to I2C bus
 * @note 
 * @author Aaron Hunter
 */
static void EEPROM_restart(void);

/**
 * @Function EEPROM_send_byte(uint8_t byte)
 * @return SUCCESS or ERROR
 * @brief sends one byte to I2C bus
 * @note if NACK from device returns ERROR
 * @author Aaron Hunter
 */
static int8_t EEPROM_send_byte(uint8_t byte);

/**
 * @Function EEPROM_read_byte(void)
 * @return SUCCESS or ERROR
 * @brief sends one byte to I2C bus
 * @note if NACK from device returns ERROR
 * @author Aaron Hunter
 */
static int8_t EEPROM_read_byte(void);

/**
 * @Function EEPROM_delay(void)
 * @return none
 * @brief delay loop for write cycle
 * @author Aaron Hunter
 */
static void EEPROM_delay(void);

/**
 * @Function EEPROM_read_data(uint8_t data[], uint8_t length, uint32_t page, uint32_t offset)
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

/**
 * @Function EEPROM_write_data(uint8_t data[], uint8_t length, uint32_t page, uint32_t offset)
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
static int8_t EEPROM_write_data(uint8_t data[], uint8_t length, uint32_t page, uint32_t offset);

/*******************************************************************************
 * PUBLIC FUNCTION IMPLEMENTATIONS                                             *
 ******************************************************************************/

/**
 * @Function EEPROM_init(void)
 * @param none
 * @return SUCCESS or ERROR
 * @brief initializes I2C1 peripheral to 100 KHz
 * @note 
 * @author Aaron Hunter,
 * @modified 
 */
uint8_t EEPROM_init(void) {
    I2C1CON = 0; //reset config
    I2C1BRG = 390; //set baud rate to 100KHz
    I2C1CONbits.ON = 1; //enable I2C
    return SUCCESS;
}

/**
 * @Function EEPROM_write_byte_array(uint8_t data[], uint8_t length, uint32_t page, uint32_t offset)
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
 * @Function EEPROM_read_byte_array(uint8_t data[], uint8_t length, uint32_t page, uint32_t offset)
 * @param uint8_t data[], pointer to data storage
 * @param uint8_t length, number of bytes to read <= PAGESIZE (64 bytes)
 * @param, uint32_t page < NUMPAGES (512), which page to write the data into
 * @param, uint32_t offset <PAGESIZE, local address within the page
 * @return SUCCESS or ERROR
 * @brief reads 1 to PAGESIZE bytes to EEPROM of datatype. returns ERROR if 
 * address is incorrect or PAGESIZE is violated (array wraps around page boundary)
 * @author Aaron Hunter
 */
int8_t EEPROM_read_byte_array(uint8_t data[], uint8_t length, uint32_t page, uint32_t offset) {
    return EEPROM_read_data(data, length, page, offset);
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
    length = length << SHORT2BYTE; //calculate the number of bytes to write
    /*cast the data into the correct type and send to the EEPROM*/
    return EEPROM_write_data((uint8_t*) data, length, page, offset);
}

/**
 * @Function EEPROM_read_short_array(int16_t data[], uint8_t length, uint32_t page, uint32_t offset)
 * @param int16_t data[], pointer to data storage
 * @param uint8_t length, number of shorts to read <= PAGESIZE/2
 * @param, uint32_t page < NUMPAGES (512), which page to write the data into
 * @param, uint32_t offset <PAGESIZE, local address within the page
 * @return SUCCESS or ERROR
 * @brief reads 1 to PAGESIZE bytes to EEPROM of datatype. returns ERROR if 
 * address is incorrect or PAGESIZE is violated (array wraps around page boundary)
 * @author Aaron Hunter
 */
int8_t EEPROM_read_short_array(int16_t data[], uint8_t length, uint32_t page, uint32_t offset) {
    length = length << SHORT2BYTE; //calculate the number of bytes to write
    /*cast the data into the correct type and send to the EEPROM*/
    return EEPROM_read_data((uint8_t *) data, length, page, offset);
}

/**
 * @Function  EEPROM_write_int_array(int32_t data[], uint8_t length, uint32_t page, uint32_t offset)
 * @param int32_t data[], pointer to data
 * @param uint8_t length, number of 32 bit ints to write <= PAGESIZE/4
 * @param, uint32_t page < NUMPAGES (512), which page to write the data into
 * @param, uint32_t offset <PAGESIZE, local address within the page
 * @return SUCCESS or ERROR
 * @brief writes 1 to PAGESIZE bytes to EEPROM
 * @author Aaron Hunter
 */
int8_t EEPROM_write_int_array(int32_t data[], uint8_t length, uint32_t page, uint32_t offset) {
    length = length << INT2BYTE; //calculate the number of bytes to write
    /*cast the data into the correct type and send to the EEPROM*/
    return EEPROM_write_data((uint8_t*) data, length, page, offset);
}

/**
 * @Function EEPROM_read_int_array(int32_t data[], uint8_t length, uint32_t page, uint32_t offset)
 * @param int32_t data[], pointer to data storage
 * @param uint8_t length, number of ints to read <= PAGESIZE/4
 * @param, uint32_t page < NUMPAGES (512), which page to write the data into
 * @param, uint32_t offset <PAGESIZE, local address within the page
 * @return SUCCESS or ERROR
 * @brief reads 1 to PAGESIZE bytes to EEPROM of datatype. returns ERROR if 
 * address is incorrect or PAGESIZE is violated (array wraps around page boundary)
 * @author Aaron Hunter
 */
int8_t EEPROM_read_int_array(int32_t data[], uint8_t length, uint32_t page, uint32_t offset) {
    length = length << INT2BYTE; //calculate the number of bytes to write
    /*cast the data into the correct type and read from the EEPROM*/
    return EEPROM_read_data((uint8_t *) data, length, page, offset);
}

/**
 * @Function  EEPROM_write_long_array(int64_t data[], uint8_t length, uint32_t page, uint32_t offset)
 * @param int64_t data[], pointer to data
 * @param uint8_t length, number of 64 bit ints to write <= PAGESIZE/8
 * @param, uint32_t page < NUMPAGES (512), which page to write the data into
 * @param, uint32_t offset <PAGESIZE, local address within the page
 * @return SUCCESS or ERROR
 * @brief writes 1 to PAGESIZE bytes to EEPROM of datatype. returns ERROR if 
 * address is incorrect or PAGESIZE is violated (array wraps around page boundary)
 * @author Aaron Hunter
 */
int8_t EEPROM_write_long_array(int64_t data[], uint8_t length, uint32_t page, uint32_t offset) {
    length = length << LONG2BYTE; //calculate the number of bytes to write
    /*cast the data into the correct type and send to the EEPROM*/
    return EEPROM_write_data((uint8_t*) data, length, page, offset);
}

/**
 * @Function EEPROM_read_long_array(int64_t data[], uint8_t length, uint32_t page, uint32_t offset)
 * @param int64_t data[], pointer to data storage
 * @param uint8_t length, number of longs to read <= PAGESIZE/8
 * @param, uint32_t page < NUMPAGES (512), which page to write the data into
 * @param, uint32_t offset <PAGESIZE, local address within the page
 * @return SUCCESS or ERROR
 * @brief reads 1 to PAGESIZE bytes to EEPROM of datatype. returns ERROR if 
 * address is incorrect or PAGESIZE is violated (array wraps around page boundary)
 * @author Aaron Hunter
 */
int8_t EEPROM_read_long_array(int64_t data[], uint8_t length, uint32_t page, uint32_t offset) {
    length = length << LONG2BYTE; //calculate the number of bytes to read
    /*cast the data pointer into the correct type and read from the EEPROM*/
    return EEPROM_read_data((uint8_t *) data, length, page, offset);
}
/**
 * @Function EEPROM_write_float_array(float data[], uint8_t length, uint32_t page, uint32_t offset)
 * @param float data[], pointer to data
 * @param uint8_t length, number of floats to write <= PAGESIZE/4
 * @param, uint32_t page < NUMPAGES (512), which page to write the data into
 * @param, uint32_t offset <PAGESIZE, local address within the page
 * @return SUCCESS or ERROR
 * @brief writes 1 to PAGESIZE bytes to EEPROM of datatype. returns ERROR if 
 * address is incorrect or PAGESIZE is violated (array wraps around page boundary)
 * @author Aaron Hunter
 */
int8_t EEPROM_write_float_array(float data[], uint8_t length, uint32_t page, uint32_t offset) {
    length = length << FLOAT2BYTE; //calculate the number of bytes to write
    /*cast the data into the correct type and send to the EEPROM*/
    return EEPROM_write_data((uint8_t*) data, length, page, offset);
}

/**
 * @Function EEPROM_read_float_array(float data[], uint8_t length, uint32_t page, uint32_t offset)
 * @param float data[], pointer to data storage
 * @param uint8_t length, number of floats to read <= PAGESIZE/4
 * @param, uint32_t page < NUMPAGES (512), which page to write the data into
 * @param, uint32_t offset <PAGESIZE, local address within the page
 * @return SUCCESS or ERROR
 * @brief reads 1 to PAGESIZE bytes to EEPROM of datatype. returns ERROR if 
 * address is incorrect or PAGESIZE is violated (array wraps around page boundary)
 * @author Aaron Hunter
 */
int8_t EEPROM_read_float_array(float data[], uint8_t length, uint32_t page, uint32_t offset) {
    length = length << FLOAT2BYTE; //calculate the number of bytes to read
    /*cast the data pointer into the correct type and read from the EEPROM*/
    return EEPROM_read_data((uint8_t *) data, length, page, offset);
}
/**
 * @Function EEPROM_write_double_array(double data[], uint8_t length, uint32_t page, uint32_t offset)
 * @param double data[], pointer to data
 * @param uint8_t length, number of doubles to write <= PAGESIZE/8
 * @param, uint32_t page < NUMPAGES (512), which page to write the data into
 * @param, uint32_t offset <PAGESIZE, local address within the page
 * @return SUCCESS or ERROR
 * @brief writes 1 to PAGESIZE bytes to EEPROM of datatype. returns ERROR if 
 * address is incorrect or PAGESIZE is violated (array wraps around page boundary)
 * @author Aaron Hunter
 */
int8_t EEPROM_write_double_array(double data[], uint8_t length, uint32_t page, uint32_t offset) {
    length = length << DOUBLE2BYTE; //calculate the number of bytes to write
    /*cast the data into the correct type and send to the EEPROM*/
    return EEPROM_write_data((uint8_t*) data, length, page, offset);
}

/**
 * @Function EEPROM_read_double_array(double data[], uint8_t length, uint32_t page, uint32_t offset)
 * @param double data[], pointer to data storage 
 * @param uint8_t length, number of doubles to read <= PAGESIZE/8
 * @param, uint32_t page < NUMPAGES (512), which page to write the data into
 * @param, uint32_t offset <PAGESIZE, local address within the page
 * @return SUCCESS or ERROR
 * @brief reads 1 to PAGESIZE bytes to EEPROM of datatype. returns ERROR if 
 * address is incorrect or PAGESIZE is violated (array wraps around page boundary)
 * @author Aaron Hunter
 */
int8_t EEPROM_read_double_array(double data[], uint8_t length, uint32_t page, uint32_t offset) {
    length = length << DOUBLE2BYTE; //calculate the number of bytes to read
    /*cast the data pointer into the correct type and read from the EEPROM*/
    return EEPROM_read_data((uint8_t *) data, length, page, offset);
}
/*******************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATIONS                                            *
 ******************************************************************************/

/**
 * @Function EEPROM_start(void)
 * @return none
 * @brief sends start bit to I2C bus
 * @note 
 * @author Aaron Hunter
 */
static void EEPROM_start(void) {
    I2C1CONbits.SEN = 1;
    while (I2C1CONbits.SEN) {
        ;
    } //wait for start bit to be cleared
}

/**
 * @Function EEPROM_stop(void)
 * @return none
 * @brief sends stop bit to I2C bus
 * @note 
 * @author Aaron Hunter
 */
static void EEPROM_stop(void) {
    I2C1CONbits.PEN = 1;
    while (I2C1CONbits.PEN) {
        ;
    } //wait for stop bit to be cleared
}

/**
 * @Function EEPROM_restart(void)
 * @return none
 * @brief sends restart bit to I2C bus
 * @note 
 * @author Aaron Hunter
 */
static void EEPROM_restart(void) {
    I2C1CONbits.RSEN = 1;
    while (I2C1CONbits.RSEN) {
        ;
    } //wait for restart bit to be cleared
}

/**
 * @Function EEPROM_send_byte(uint8_t byte)
 * @return SUCCESS or ERROR
 * @brief sends one byte to I2C bus
 * @note if NACK from device returns ERROR
 * @author Aaron Hunter
 */
static int8_t EEPROM_send_byte(uint8_t byte) {
    I2C1TRN = byte; //if an address bit0 = 0 for write, 1 for read
    while (I2C1STATbits.TRSTAT) { //wait for byte to be transmitted
        ;
    }
    if (I2C1STATbits.ACKSTAT == 1) { //not acknowledged, transmission failed
        return ERROR;
    }
    return SUCCESS;
}

/**
 * @Function EEPROM_read_byte(void)
 * @return SUCCESS or ERROR
 * @brief sends one byte to I2C bus
 * @note if NACK from device returns ERROR
 * @author Aaron Hunter
 */
static int8_t EEPROM_read_byte(void) {
    I2C1CONbits.RCEN = 1; //setting this bit initiates a receive.  Hardware clears after the received has finished
    while (I2C1CONbits.RCEN) { //wait for byte to be received
        ;
    }
    return I2C1RCV; //return the value in receive buffer
}

/**
 * @Function EEPROM_delay()
 * @return none
 * @brief delay loop for write cycle
 * @author Aaron Hunter
 */
static void EEPROM_delay(void) {
    uint8_t EEPROM_busy = TRUE;
    while (EEPROM_busy == TRUE) {
        EEPROM_start();
        if (EEPROM_send_byte(EEPROM_I2C_ADDR << 1 | WRITE) == SUCCESS) {
            EEPROM_busy = FALSE;
        }
        EEPROM_stop();
    }
    EEPROM_stop();
}

/**
 * @Function EEPROM_read_data(uint8_t data[], uint8_t length, uint32_t page, uint32_t offset)
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
    //    EEPROM_busy = TRUE;
    if ((length <= PAGESIZE)&& (page < NUMPAGES) && (offset + length < PAGESIZE)) {
        address = (page << LOG64 | offset); //set the address of data
        EEPROM_settings.mem_high_byte = (uint8_t) (address >> 8); //mask off high byte
        EEPROM_settings.mem_low_byte = (uint8_t) address; //mask off low byte
        EEPROM_settings.mode = READ;
        EEPROM_settings.source_data = data;
        EEPROM_settings.length = length;
        EEPROM_settings.index = 0;
    } else {
        //        printf("Bad address or array too large\r\n");
        return ERROR;
    }
    // send start command
    EEPROM_start();
    // send device address
    if (EEPROM_send_byte(EEPROM_I2C_ADDR << 1 | WRITE) == ERROR) {
        return ERROR;
    }
    // if ACK, send mem address high byte
    if (EEPROM_send_byte(EEPROM_settings.mem_high_byte) == ERROR) {
        return ERROR;
    }
    // if ACK send mem address low byte
    if (EEPROM_send_byte(EEPROM_settings.mem_low_byte) == ERROR) {
        return ERROR;
    }
    // send restart bit to initiate data read
    EEPROM_restart();
    if (EEPROM_send_byte(EEPROM_I2C_ADDR << 1 | READ) == ERROR) {
        return ERROR;
    }
    // read all data
    for (EEPROM_settings.index = 0; EEPROM_settings.index < EEPROM_settings.length; EEPROM_settings.index++) {
        EEPROM_settings.source_data[EEPROM_settings.index] = EEPROM_read_byte();
        if (EEPROM_settings.index < EEPROM_settings.length - 1) { // don't ACK final byte
            I2C1CONbits.ACKEN = 1; //ACK the byte to signal the EEPROM to continue
            while (I2C1CONbits.ACKEN) { //wait for ack bit to be cleared
                ;
            }
        }
    }
    // send stop condition
    EEPROM_stop();
    // wait for write to complete
    EEPROM_delay();
    return SUCCESS;
}

/**
 * @Function EEPROM_write_data(uint8_t data[], uint8_t length, uint32_t page, uint32_t offset)
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
    if ((length <= PAGESIZE)&& (page < NUMPAGES) && (offset + length < PAGESIZE)) {
        address = (page << LOG64 | offset); //set the address of data
        EEPROM_settings.mem_high_byte = (uint8_t) (address >> 8); //mask off high byte
        EEPROM_settings.mem_low_byte = (uint8_t) address; //mask off low byte
        EEPROM_settings.mode = WRITE;
        EEPROM_settings.source_data = data;
        EEPROM_settings.length = length;
        EEPROM_settings.index = 0;
    } else {
        //        printf("Bad address or array too large\r\n");
        return ERROR;
    }
    // send start command
    EEPROM_start();
    // send device address
    if (EEPROM_send_byte(EEPROM_I2C_ADDR << 1 | WRITE) == ERROR) {
        return ERROR;
    }
    // if ACK, send mem address high byte
    if (EEPROM_send_byte(EEPROM_settings.mem_high_byte) == ERROR) {
        return ERROR;
    }
    // if ACK send mem address low byte
    if (EEPROM_send_byte(EEPROM_settings.mem_low_byte) == ERROR) {
        return ERROR;
    }
    // send all the data 
    for (EEPROM_settings.index = 0; EEPROM_settings.index < EEPROM_settings.length; EEPROM_settings.index++) {
        if (EEPROM_send_byte(EEPROM_settings.source_data[EEPROM_settings.index]) == ERROR) {
            return ERROR;
        }
    }
    // stop transaction
    EEPROM_stop();
    // delay until page write completes
    EEPROM_delay();
    return SUCCESS;
}

#ifdef EEPROM_TESTING

void main(void) {
    int8_t compare = TRUE;
    uint8_t byte;
    uint8_t i;
    uint32_t page;
    uint8_t offset;
    uint8_t msg[PAGESIZE];
    uint8_t recv_msg[PAGESIZE];
    uint8_t length;
    int32_t start_time;
    int32_t end_time;
    int16_t shorts[] = {0xaacc, 0xff00, 0xbbdd, 0xff00, 0x2112, 0xff00, 0x0202, 0x4884};
    int16_t recv_shorts[PAGESIZE >> SHORT2BYTE];
    int32_t ints[] = {0xaaccff00, 0xbbddff00, 0x2112ff00, 0x02024884};
    int32_t recv_ints[PAGESIZE >> INT2BYTE];
    int64_t longs[] = {0xbbddff00aaccff00, 0x020248842112ff00};
    int64_t recv_longs[PAGESIZE >> LONG2BYTE];
    float pi_float[] = {3.1415926535897932384626433832795F};
    float recv_floats[PAGESIZE >> FLOAT2BYTE];
    float * val;
    double pi_double[] = {3.1415926535897932384626433832795L};
    double recv_doubles[PAGESIZE>>DOUBLE2BYTE];


    Board_init();
    Serial_init();
    Sys_timer_init();
    EEPROM_init();

    printf("EEPROM2 Test Harness,  %s, %s\r\n", __DATE__, __TIME__);
    /**********************************************/
    //read first byte in memory
    length = 1;
    EEPROM_read_data(&byte, length, 0, 0);
    printf("Byte read is %c \r\n", byte);
    //read first 13 bytes in memory
    length = 13;
    printf("data is: ");
    EEPROM_read_data(msg, length, 0, 0);
    for (i = 0; i < length; i++) {
        printf("%c", msg[i]);
    }
    printf("\r\n");
    /**********************************************/
    // test a write on page 1
    page = 1;
    offset = 0;
    sprintf(msg, "Developed by Aaron Hunter");
    length = strlen(msg);
    start_time = Sys_timer_get_usec();
    EEPROM_write_data(msg, length, page, offset);
    // test page write time
    end_time = Sys_timer_get_usec();
    printf("busy time is %d microseconds\r\n", end_time - start_time);
    EEPROM_read_data(msg, length, page, offset);
    for (i = 0; i < length; i++) {
        printf("%c", msg[i]);
    }
    printf("\r\n");
    /**********************************************/
    // test wrapper for byte array
    if (EEPROM_read_byte_array(recv_msg, length, page, offset) == SUCCESS) {
        for (i = 0; i < length; i++) {
            printf("%c", recv_msg[i]);
        }
        printf("\r\n");
    } else {
        printf("ERROR in read byte array \r\n");
    }
    if (strcmp(msg, recv_msg) == 0) {
        printf("byte array read success!\r\n");
    } else {
        printf("byte array read fail!\r\n");
    }
    /**********************************************/
    // test write and read short array
    length = sizeof (shorts) / sizeof (shorts[0]);
    page = NUMPAGES - 1;
    EEPROM_write_short_array(shorts, length, page, offset);
    EEPROM_read_short_array(recv_shorts, length, page, offset);
    compare = TRUE;
    for (i = 0; i < length; i++) {
        if (shorts[i] != recv_shorts[i]) {
            compare = FALSE;
        }
    }
    if (compare == FALSE) {
        printf("Short array read/write error\r\n");
    } else {
        printf("Short array read/write success!\r\n");
    }
    /**********************************************/
    // test write and read int array
    length = sizeof (ints) / sizeof (ints[0]);
    page = NUMPAGES - 2;
    EEPROM_write_int_array(ints, length, page, offset);
    EEPROM_read_int_array(recv_ints, length, page, offset);
    compare = TRUE;
    for (i = 0; i < length; i++) {
        if (ints[i] != recv_ints[i]) {
            compare = FALSE;
        }
    }
    if (compare == FALSE) {
        printf("Int array read/write error\r\n");
    } else {
        printf("Int array read/write success!\r\n");
    }
    /**********************************************/
    // test write and read long array
    length = sizeof (longs) / sizeof (longs[0]);
    page = NUMPAGES - 3;
    EEPROM_write_long_array(longs, length, page, offset);
    EEPROM_read_long_array(recv_longs, length, page, offset);
    compare = TRUE;
    for (i = 0; i < length; i++) {
        if (longs[i] != recv_longs[i]) {
            compare = FALSE;
        }
    }
    if (compare == FALSE) {
        printf("Long array read/write error\r\n");
    } else {
        printf("Long array read/write success!\r\n");
    }
    /**********************************************/
    // test write and read float array
    length = sizeof (pi_float) / sizeof (pi_float[0]);
    page = NUMPAGES - 4;
    EEPROM_write_float_array(pi_float, length, page, offset);
    EEPROM_read_float_array(recv_floats, length, page, offset);
    compare = TRUE;
    for (i = 0; i < length; i++) {
        if (pi_float[i] != recv_floats[i]) {
            compare = FALSE;
        }
    }
    if (compare == FALSE) {
        printf("Float array read/write error\r\n");
    } else {
        printf("Float array read/write success!\r\n");
    }
        /**********************************************/
    // test write and read double array
    length = sizeof (pi_double) / sizeof (pi_double[0]);
    page = NUMPAGES - 5;
    EEPROM_write_double_array(pi_double, length, page, offset);
    EEPROM_read_double_array(recv_doubles, length, page, offset);
    compare = TRUE;
    for (i = 0; i < length; i++) {
        if (pi_double[i] != recv_doubles[i]) {
            compare = FALSE;
        }
    }
    if (compare == FALSE) {
        printf("Double array read/write error\r\n");
    } else {
        printf("Double array read/write success!\r\n");
    }
    printf("Receive Pi as a double:  %.34f \r\n", recv_doubles[0]);
}
#endif


