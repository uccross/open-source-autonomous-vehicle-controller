/* 
 * File:   EEPROM.h
 * Author: Aaron Hunter
 * Brief: interface to EEPROM storage
 * Created on May 6, 2021, 11:48 am
 * Modified on <month> <day>, <year>, <hour> <pm/am>
 */

#ifndef EEPROM_H // Header guard
#define	EEPROM_H //

/*******************************************************************************
 * PUBLIC #INCLUDES                                                            *
 ******************************************************************************/
#include <sys/types.h>

/*******************************************************************************
 * PUBLIC #DEFINES                                                             *
 ******************************************************************************/


/*******************************************************************************
 * PUBLIC TYPEDEFS                                                             *
 ******************************************************************************/


/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES                                                  *
 ******************************************************************************/

/**
 * @Function EEPROM_init(void)
 * @param none
 * @return SUCCESS or ERROR
 * @brief initializes I2C1 peripheral to 400 KHz and sets up interrupt
 * @note EEPROM requires pullup resistors ~4.7K on SDA and SCL
 * @author Aaron Hunter,
 * @modified 
 */
uint8_t EEPROM_init(void);

/**
 * @Function EEPROM_is_busy(void)
 * @param none
 * @return TRUE or FALSE
 * @brief if EEPROM in write or read state returns TRUE
 * @note WRITEs require up to 5msec after TRUE before next communication, 
 * device will NACK any attempts to access device resulting in ERROR (see below)
 * @author Aaron Hunter,
 * @modified 
 */
uint8_t EEPROM_is_busy(void);

/**
 * @Function EEPROM_is_error(void)
 * @param none
 * @return TRUE or FALSE
 * @brief if EEPROM fails to ACK its address returns TRUE
 * @note 
 * @author Aaron Hunter,
 * @modified 
 */
uint8_t EEPROM_is_error(void);

/**
 * @Function 
 * @param uint8_t data[], pointer to data
 * @param uint8_t length, number of bytes to write <= PAGESIZE (64 bytes)
 * @param, uint32_t page < NUMPAGES (512), which page to write the data into
 * @param, uint32_t offset <PAGESIZE, local address within the page
 * @return SUCCESS or ERROR
 * @brief writes 1 to PAGESIZE bytes to EEPROM of datatype. returns ERROR if 
 * address is incorrect or PAGESIZE is violated (array wraps around page boundary)
 * @author Aaron Hunter
 */
int8_t EEPROM_write_byte_array(uint8_t data[], uint8_t length, uint32_t page, uint32_t offset);

/**
 * @Function EEPROM_read_byte_array(uint8_t data[], uint8_t length, uint32_t page, uint32_t offset)
 * @param uint8_t data[], pointer to data storage
 * @param uint8_t length, number of bytes to write <= PAGESIZE (64 bytes)
 * @param, uint32_t page < NUMPAGES (512), which page to write the data into
 * @param, uint32_t offset <PAGESIZE, local address within the page
 * @return SUCCESS or ERROR
 * @brief reads 1 to PAGESIZE bytes to EEPROM of datatype. returns ERROR if 
 * address is incorrect or PAGESIZE is violated (array wraps around page boundary)
 * @author Aaron Hunter
 */
int8_t EEPROM_read_byte_array(uint8_t data[], uint8_t length, uint32_t page, uint32_t offset);

/**
 * @Function  EEPROM_write_short_array(uint16_t data[], uint8_t length, uint32_t page, uint32_t offset)
 * @param uint16_t data[], pointer to data
 * @param uint8_t length, number of 16 bit words to write
 * @param, uint32_t page < NUMPAGES (512), which page to write the data into
 * @param, uint32_t offset <PAGESIZE, local address within the page
 * @return SUCCESS or ERROR
 * @brief writes 1 to PAGESIZE bytes to EEPROM of datatype. returns ERROR if 
 * address is incorrect or PAGESIZE is violated (array wraps around page boundary)
 * @author Aaron Hunter
 */
int8_t EEPROM_write_short_array(int16_t data[], uint8_t length, uint32_t page, uint32_t offset);

/**
 * @Function  EEPROM_write_int_array(int32_t data[], uint8_t length, uint32_t page, uint32_t offset)
 * @param uint16_t data[], pointer to data
 * @param uint8_t length, number of 32 bit ints to write
 * @param, uint32_t page < NUMPAGES (512), which page to write the data into
 * @param, uint32_t offset <PAGESIZE, local address within the page
 * @return SUCCESS or ERROR
 * @brief writes 1 to PAGESIZE bytes to EEPROM of datatype. returns ERROR if 
 * address is incorrect or PAGESIZE is violated (array wraps around page boundary)
 * @author Aaron Hunter
 */
int8_t EEPROM_write_int_array(int32_t data[], uint8_t length, uint32_t page, uint32_t offset);

/**
 * @Function  EEPROM_write_long_array(int64_t data[], uint8_t length, uint32_t page, uint32_t offset)
 * @param uint64_t data[], pointer to data
 * @param uint8_t length, number of 64 bit ints to write
 * @param, uint32_t page < NUMPAGES (512), which page to write the data into
 * @param, uint32_t offset <PAGESIZE, local address within the page
 * @return SUCCESS or ERROR
 * @brief writes 1 to PAGESIZE bytes to EEPROM of datatype. returns ERROR if 
 * address is incorrect or PAGESIZE is violated (array wraps around page boundary)
 * @author Aaron Hunter
 */
int8_t EEPROM_write_long_array(int64_t data[], uint8_t length, uint32_t page, uint32_t offset);

/**
 * @Function EEPROM_write_float_array(float data[], uint8_t length, uint32_t page, uint32_t offset)
 * @param float data[], pointer to data
 * @param uint8_t length, number of floats to write
 * @param, uint32_t page < NUMPAGES (512), which page to write the data into
 * @param, uint32_t offset <PAGESIZE, local address within the page
 * @return SUCCESS or ERROR
 * @brief writes 1 to PAGESIZE bytes to EEPROM of datatype. returns ERROR if 
 * address is incorrect or PAGESIZE is violated (array wraps around page boundary)
 * @author Aaron Hunter
 */
int8_t EEPROM_write_float_array(float data[], uint8_t length, uint32_t page, uint32_t offset);

/**
 * @Function EEPROM_write_double_array(double data[], uint8_t length, uint32_t page, uint32_t offset)
 * @param double data[], pointer to data
 * @param uint8_t length, number of doubles to write
 * @param, uint32_t page < NUMPAGES (512), which page to write the data into
 * @param, uint32_t offset <PAGESIZE, local address within the page
 * @return SUCCESS or ERROR
 * @brief writes 1 to PAGESIZE bytes to EEPROM of datatype. returns ERROR if 
 * address is incorrect or PAGESIZE is violated (array wraps around page boundary)
 * @author Aaron Hunter
 */
int8_t EEPROM_write_double_array(double data[], uint8_t length, uint32_t page, uint32_t offset);



#endif	/* EEPROM_H */ // End of header guard

