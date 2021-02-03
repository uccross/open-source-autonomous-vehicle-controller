/* 
 * File:   AS5047D.h
 * Author: Aaron Hunter
 * Brief: Driver for the rotary encoders 
 * Created on Dec 3, 2020 3:46 pm
 * Modified on 
 */

#ifndef AS5047D_H // Header guard
#define	AS5047D_H //

/*******************************************************************************
 * PUBLIC #INCLUDES                                                            *
 ******************************************************************************/

#include <sys/types.h>

/*******************************************************************************
 * PUBLIC #DEFINES                                                             *
 ******************************************************************************/
#define NUM_ENCODERS 3

/*******************************************************************************
 * PUBLIC TYPEDEFS                                                             *
 ******************************************************************************/
// All the potential encoders in the system

typedef enum {
    LEFT_MOTOR,
    RIGHT_MOTOR,
    HEADING,
    PAN,
} encoder_enum_t;

typedef struct encoder {
    int16_t last_theta; //old angle
    int16_t next_theta; //new angle
    int16_t omega; //angular velocity
} encoder_t;

typedef struct encoder* encoder_ptr_t; //pointer to encoder struct

/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES                                                  *
 ******************************************************************************/

/**
 * @Function Encoder_Init(void)
 * @param freq, frequency of SPI rate
 * @return SUCCESS or ERROR
 * @brief initializes hardware in appropriate mode along with the needed interrupts */
uint8_t Encoder_init(void);

/**
 * @Function int16_Encoder_get_angle(encoder_enum_t encoder_num);
 * @param encoder number
 * @return 14-bit number representing the raw encoder angle (0-16384)
 * @author Aaron Hunter */
int16_t Encoder_get_angle(encoder_enum_t encoder_num);

/**
 * @Function Encoder_start_data_acq(void);
 * @return none
 * @param none
 * @brief this function starts the SPI data read
 * @author Aaron Hunter
 **/
void Encoder_start_data_acq(void);

/**
 * @Function Encoder_is_data_ready(void)
 * @param none
 * @brief returns TRUE if there is unread (from Encoder_get_data()) data available
 * @return TRUE or FALSE
 * @author Aaron Hunter
 */
int8_t Encoder_is_data_ready(void);

/**
 * @Function int16_t Encoder_get_velocity(encoder_enum_t encoder_num);
 * @param encoder number 
 * @return angular velocity measurement
 */
int16_t Encoder_get_velocity(encoder_enum_t encoder_num);

/**
 * @Function void init_encoder_data(void);
 * @brief initializes encoder data struct(s)
 * @author Aaron Hunter
 */
void Encoder_init_encoder_data(encoder_ptr_t enc);

/**
 * @Function Encoder_get_data(encoder_t * data)
 * @param encoder_t data--to receive private encoder data 
 * @brief copies internal encoder data to data
 * @author Aaron Hunter
 */
int8_t Encoder_get_data(encoder_t * data);


#endif	/* AS5047D_H */ // End of header guard

