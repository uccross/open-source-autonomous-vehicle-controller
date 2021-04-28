/* 
 * File:   Garmin_v3hp.c
 * Author: Aaron Hunter
 * Brief: Sensor driver for the Garmin Lidar-lite V3HP
 * Created on April, 27, 2021, 11:23 am
 * Modified on <month> <day>, <year>, <hour> <pm/am>
 */

/*******************************************************************************
 * #INCLUDES                                                                   *
 ******************************************************************************/

#include "Garmin_v3hp.h" // The header file for this source file. 
#include "Board.h"  //Max 32 dev board 
#include "System_timer.h" // Millisecond and micrsecond hardware timer
#include "SerialM32.h" // serial communications via USB
#include "xc.h" // compiler
#include <stdio.h>
#include <string.h>
#include <sys/attribs.h>  //for ISR definitions
#include <proc/p32mx795f512l.h>  //Max 32 specific info

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/
#define LIDAR_I2C_ADDR 0x62  //device address
/********LiDAR register defines*************/
#define ACQ_COMMAND 0x00    //Device Command. 0x04 takes measurement with bias correction
#define STATUS 0x01
#define SIG_COUNT_VAL 0x02  //Maximum acquisition count, default 0x80
#define ACQ_CONFIG_REG 0x04  //Acquisition mode control, default 0x08 
#define FULL_DELAY_HIGH 0x0f    //current high byte
#define FULL_DELAY_LOW 0x10     //current low byte
/*the commented out registers don't appear for V3HP*/
//#define OUTER_LOOP_COUNT 0x11   //number of measurements per trigger
//#define LAST_DELAY_HIGH 0x14    //prev high byte
//#define LAST_DELAY_LOW 0x15     //prev low byte
#define THRESHOLD_BYPASS 0x1c   //peak detection threshold bypass--allows for simple thresholding instead of algorithm
#define MEASURE_DELAY 0x45  //delay between automatic measurements 0xc8 ~ 10 Hz, 0x14 ~ 100 Hz
#define POWER_CONTROL 0x65  //setting bit0 disables receiver circuit to save 40mA
#define DIST_REG 0x8f
/******************************************/
#define TEN_HZ 100000
#define HUNDRED_HZ 1000000
#define ACK 0
#define NACK 1
#define READ 1
#define WRITE 0

/*******************************************************************************
 * PRIVATE TYPEDEFS                                                            *
 ******************************************************************************/
typedef enum {
    LIDAR_IDLE,
    TX_ADDRESS_SENT,
    TX_REGISTER_SENT,
    TX_SETTINGS_SENT,
    TX_STOP,
    RX_START_SET_REGISTER,
    RX_SEND_ADDRESS,
    RX_SEND_REGISTER,
    RX_STOP_SET_REGISTER,
    RX_START_READ,
    RX_READ_SEND_ADDRESS,
    RX_READ_HIGH_BYTE,
    RX_ACK_HIGH_BYTE,
    RX_READ_LOW_BYTE,
    RX_NACK_LOW_BYTE,
    RX_STOP_READ,
} LIDAR_I2C_states_t; //state machine states during I2C

typedef struct {
    uint8_t LIDAR_register; //register to write to
    uint8_t setting; //value to write to the register
    uint8_t mode; //read or write
} LIDAR_packet_t;

LIDAR_packet_t LIDAR_settings = {
    .LIDAR_register = ACQ_CONFIG_REG,
    .setting = 0x09,
    .mode = WRITE
}; //struct for I2C state machine settings

LIDAR_packet_t* settings_p = &LIDAR_settings;

static int8_t I2C_error = FALSE;
static uint16_t LIDAR_distance = 0; //current range measurement

/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                 *
 ******************************************************************************/
static void __ISR(_I2C2_VECTOR, IPL3AUTO) LIDAR_interrupt_handler(void);
static void LIDAR_set_values(uint8_t LIDAR_register, uint8_t setting, uint8_t mode);
static void LIDAR_run_I2C_state_machine(void);
/*******************************************************************************
 * PUBLIC FUNCTION IMPLEMENTATIONS                                             *
 ******************************************************************************/

/**
 * @Function Lidar_Init(void);
 * @brief sets up I2C communication and configures LIDAR
 * @return SUCCESS or ERROR
 * @author Aaron Hunter */
int8_t Lidar_Init(void) {
    __builtin_disable_interrupts();
    /*config LIDAR I2C interrupt*/
    /*config priority and subpriority--must match IPL level*/
    IPC8bits.I2C2IP = 3;
    IPC8bits.I2C2IS = 0;
    /*clear interrupt flag*/
    IFS1bits.I2C2MIF = 0; //clear flag
    /*enable I2C master interrupt*/
    IEC1bits.I2C2MIE = 1;
    __builtin_enable_interrupts();

    I2C2CON = 0; //reset config; turn off I2C
    I2C2BRG = 398; //set baud rate to 400KHz-->verify with Oscope for correct timing
    I2C2CONbits.ON = 1; //enable I2C
    //need 22msec delay before first measurement
    TRISEbits.TRISE9 = 0;
    LATEbits.LATE9 = 1; //set power enable pin high (low to disable)
    LIDAR_set_values(ACQ_CONFIG_REG, 0x09, WRITE);
    I2C2CONbits.SEN = 1; // send start command to write settings to device
    return SUCCESS;
}

/**
 * @Function uint16_t Lidar_get_range(void);
 * @return most recent 16 bit range measurement
 * @brief returns the most recent range measurement and starts the next acquisition
 * @author:  Aaron Hunter
 */
uint16_t Lidar_get_range(void) {
    /*set ACQ_COMMAND to any non-zero value to initiate new measurement*/
    LIDAR_set_values(ACQ_COMMAND, 0x04, READ); 
    I2C2CONbits.SEN = 1; // send start condition to start state machine
    return LIDAR_distance;
}

/*******************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATIONS                                            *
 ******************************************************************************/
/**
 * @Function static void __ISR(_I2C2_VECTOR, IPL3AUTO) LIDAR_interrupt_handler(void) 
 * @brief Interrupt handler for I2C2 master
 * @author Aaron Hunter
 */
static void __ISR(_I2C2_VECTOR, IPL3AUTO) LIDAR_interrupt_handler(void) {
    LIDAR_run_I2C_state_machine();
    IFS1bits.I2C2MIF = 0; //clear flag
}

/**
 * @Function void LIDAR_set_values(uint8_t LIDAR_register, uint8_t setting, uint8_t mode)
 * @param LIDAR_register, from defined values
 * @param setting, byte specific to the register
 * @param mode READ or WRITE
 * @brief:  loads register, setting and mode into the LIDAR_settings packet
 * @author Aaron Hunter
 **/
static void LIDAR_set_values(uint8_t LIDAR_register, uint8_t setting, uint8_t mode) {
    LIDAR_settings.LIDAR_register = LIDAR_register;
    LIDAR_settings.setting = setting;
    LIDAR_settings.mode = mode;
}

/**
 * @Function:  LIDAR_run_I2C_state_machine
 * @param none
 * @brief: implements the LIDAR state machine for setting registers and reading distance data
 * @author:  Aaron Hunter
 **/
static void LIDAR_run_I2C_state_machine(void) {
    static LIDAR_I2C_states_t current_state = LIDAR_IDLE;
    static LIDAR_I2C_states_t next_state = LIDAR_IDLE;
    static uint8_t error = FALSE;

    switch (current_state) {
        case(LIDAR_IDLE):
            //            LATESET = 0x01;
            //set module flag if we entered this state in error
            if (error == TRUE) {
                I2C_error = TRUE;
            } else {
                I2C_error = FALSE;
            }
            //load device address into I2C transmit buffer write bit set
            I2C2TRN = LIDAR_I2C_ADDR << 1;
            next_state = TX_ADDRESS_SENT;
            break;
        case(TX_ADDRESS_SENT):
            //            LATESET = 0x03;
            if (I2C2STATbits.ACKSTAT == NACK) { // check for ack
                error = TRUE;
                I2C2CONbits.PEN = 1; //send stop condition 
                next_state = TX_STOP;
            } else {
                error = FALSE; // device has ACKED, so I2C working
                I2C2TRN = LIDAR_settings.LIDAR_register; //load device register into I2C transmit buffer
                next_state = TX_REGISTER_SENT;
            }
            break;
        case(TX_REGISTER_SENT):
            //            LATESET = 0x07;
            if (I2C2STATbits.ACKSTAT == NACK) { // check for ack
                error = TRUE;
                I2C2CONbits.PEN = 1; //send stop condition 
                next_state = TX_STOP;
            } else {
                I2C2TRN = LIDAR_settings.setting; //load register setting into I2C transmit buffer
                next_state = TX_SETTINGS_SENT;
            }
            break;
        case(TX_SETTINGS_SENT):
            //            LATESET = 0x0F;
            if (I2C2STATbits.ACKSTAT == NACK) { // check for ack
                error = TRUE;
                I2C2CONbits.PEN = 1; //send stop condition 
                next_state = TX_STOP;
            } else if (LIDAR_settings.mode == WRITE) {
                next_state = TX_STOP;
            } else {
                next_state = RX_START_SET_REGISTER;
            }
            I2C2CONbits.PEN = 1; //send stop condition
            break;
        case(TX_STOP):
            next_state = LIDAR_IDLE;
            break;
        case(RX_START_SET_REGISTER):
            //            LATESET = 0x10;
            I2C2CONbits.SEN = 1; //send start condition
            next_state = RX_SEND_ADDRESS;
            break;
        case(RX_SEND_ADDRESS):
            //load device address into I2C transmit buffer write bit set
            I2C2TRN = LIDAR_I2C_ADDR << 1;
            next_state = RX_SEND_REGISTER;
            break;
        case(RX_SEND_REGISTER):
            if (I2C2STATbits.ACKSTAT == NACK) { // check for ack
                error = TRUE;
                I2C2CONbits.PEN = 1; //send stop condition 
                next_state = TX_STOP;
            } else {
                I2C2TRN = (0x80 | FULL_DELAY_HIGH); // initiates auto-increment read
                next_state = RX_STOP_SET_REGISTER;
            }
            break;
        case(RX_STOP_SET_REGISTER):
            if (I2C2STATbits.ACKSTAT == NACK) { // check for ack
                error = TRUE;
                next_state = TX_STOP;
            } else {
                next_state = RX_START_READ;
            }
            I2C2CONbits.PEN = 1; //send stop condition 
            break;
        case(RX_START_READ):
            I2C2CONbits.SEN = 1; //send start condition
            next_state = RX_READ_SEND_ADDRESS;
            break;
        case(RX_READ_SEND_ADDRESS):
            //load device address into I2C transmit buffer with read bit set
            I2C2TRN = (LIDAR_I2C_ADDR << 1 | READ);
            next_state = RX_READ_HIGH_BYTE;
            break;
        case(RX_READ_HIGH_BYTE):
            if (I2C2STATbits.ACKSTAT == NACK) { // check for ack
                error = TRUE;
                I2C2CONbits.PEN = 1; //send stop condition 
                next_state = TX_STOP;
            } else {
                I2C2CONbits.RCEN = 1; //setting this bit initiates a receive.
                next_state = RX_ACK_HIGH_BYTE;
            }
            break;
        case(RX_ACK_HIGH_BYTE):
            LIDAR_distance = I2C2RCV << 8; //high byte;
            I2C2CONbits.ACKDT = 0; // ACK the byte
            I2C2CONbits.ACKEN = 1;
            next_state = RX_READ_LOW_BYTE;
            break;
        case(RX_READ_LOW_BYTE):
            I2C2CONbits.RCEN = 1; //setting this bit initiates a receive.
            next_state = RX_NACK_LOW_BYTE;
            break;
        case(RX_NACK_LOW_BYTE):
            LIDAR_distance = LIDAR_distance | I2C2RCV;
            I2C2CONbits.ACKDT = 1; // NACK the byte
            I2C2CONbits.ACKEN = 1;
            next_state = RX_STOP_READ;
            break;
        case(RX_STOP_READ):
            I2C2CONbits.PEN = 1; //send stop condition 
            next_state = TX_STOP;
            break;
        default:
            break;
    }
    current_state = next_state;
}


#ifdef LIDARV3HP_TESTING
void main(void) {
    int startTime;
    int loopTime = TEN_HZ;
    uint16_t range;
    Board_init();
    Serial_init();
    Sys_timer_init();
    Lidar_Init();

    printf("LIDAR-liteV3HP Test Harness,  %s, %s\r\n", __DATE__, __TIME__);
    startTime = Sys_timer_get_msec();
    /*Need to specify a warm-up time of 22msec before taking the first measurement*/
    while (Sys_timer_get_msec() < startTime + 22) {
        ;
    }
    startTime = Sys_timer_get_usec();
    while (1) {
        if (Sys_timer_get_usec() > startTime + loopTime) {
            startTime = Sys_timer_get_usec();
            range = Lidar_get_range();
            printf("Range: %d cm\r\n", range);
            if(I2C_error == TRUE){
                printf("I2C Error \r\n");
            }
        }
    }

}

#endif //LIDARV3HP_TESTING

