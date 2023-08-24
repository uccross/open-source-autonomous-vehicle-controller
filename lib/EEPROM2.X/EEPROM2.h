/* 
 * File:   EEPROM2.h
 * Author: Aaron Hunter
 * Brief: Blocking version of EEPROM library
 * Created on 06/15/2021 7:41 am
 * Modified on 
 */

#ifndef EEPROM2_H // Header guard
#define	EEPROM2_H //

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
 * @brief initializes I2C1 peripheral to 400 KHz
 * @note EEPROM requires pullup resistors ~4.7K on SDA and SCL
 * @author Aaron Hunter,
 * @modified 
 */
uint8_t EEPROM_init(void);

/**
 * @Function EEPROM_write_byte_array(uint8_t data[], uint8_t length, uint32_t page, uint32_t offset)
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
 * @param uint8_t array length)
 * @param, uint32_t page < NUMPAGES (512), which page to write the data into
 * @param, uint32_t offset <PAGESIZE, local address within the page
 * @return SUCCESS or ERROR
 * @brief casts the 16 bit data into 8 bit and passes to the internal method for storage
 * @author Aaron Hunter
 */
int8_t EEPROM_write_short_array(int16_t data[], uint8_t length, uint32_t page, uint32_t offset);

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
int8_t EEPROM_read_short_array(int16_t data[], uint8_t length, uint32_t page, uint32_t offset);

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
int8_t EEPROM_write_int_array(int32_t data[], uint8_t length, uint32_t page, uint32_t offset);

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
int8_t EEPROM_read_int_array(int32_t data[], uint8_t length, uint32_t page, uint32_t offset);

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
int8_t EEPROM_write_long_array(int64_t data[], uint8_t length, uint32_t page, uint32_t offset);

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
int8_t EEPROM_read_long_array(int64_t data[], uint8_t length, uint32_t page, uint32_t offset);
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
int8_t EEPROM_write_float_array(float data[], uint8_t length, uint32_t page, uint32_t offset);

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
int8_t EEPROM_read_float_array(float data[], uint8_t length, uint32_t page, uint32_t offset);

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
int8_t EEPROM_write_double_array(double data[], uint8_t length, uint32_t page, uint32_t offset);

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
int8_t EEPROM_read_double_array(double data[], uint8_t length, uint32_t page, uint32_t offset);
#endif	/* EEPROM2_H  */ // End of header guard

