/* 
 * File:   ICM-20948.c
 * Author: Aaron Hunter
 * Brief: Library for the ICM-20948 IMU
 * Created on Nov 13, 2020 9:46 am
 * Modified on 
 */

/*******************************************************************************
 * #INCLUDES                                                                   *
 ******************************************************************************/

#include "ICM-20948.h" // The header file for this source file. 
#include "SerialM32.h"
#include "Board.h"
#include <stdio.h>
#include <sys/attribs.h>  //for ISR definitions
#include <proc/p32mx795f512l.h>


/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/
#define IMU_BAUD 400000
#define ICM_I2C_ADDR 0b1101001 //=decimal 69
#define ICM_WHO_AM_I 0 //base register of the I2C device
#define ACK 0
#define NACK 1
#define READ 1
#define WRITE 0

/*******************************************************************************
 * PRIVATE TYPEDEFS                                                            *
 ******************************************************************************/
typedef enum {
    IMU_IDLE,
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
} IMU_config_states_t;

static int8_t I2C_is_configured = FALSE;
/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                 *
 ******************************************************************************/
/*blocking functions, deprecated*/
void I2C_start(void);
void I2C_stop(void);
void I2C_restart(void);
int I2C_sendByte(unsigned char byte);
unsigned char I2C_readByte(void);
/***********************************/
void IMU_cfg_I2C_state_machine(void);
/*******************************************************************************
 * PUBLIC FUNCTION IMPLEMENTATIONS                                             *
 ******************************************************************************/

/**
 * @Function IMU_init(void)
 * @return SUCCESS or ERROR
 * @brief initializes the I2C system for IMU operation
 * @note 
 * @author Aaron Hunter,
 * @modified  */
uint8_t IMU_init(void) {
    uint32_t pb_clk;
    __builtin_disable_interrupts();
    /*config LIDAR I2C interrupt*/
    /*config priority and subpriority--must match IPL level*/
    IPC6bits.I2C1IP = 2;
    IPC6bits.I2C1IS = 0;
    /*enable master interrupt*/
    //    IEC0bits.I2C1MIE = 1;
    /*clear master interrupt flag*/
    IFS0bits.I2C1MIF = 0;
    /*restart interrupts*/
    __builtin_enable_interrupts();

    I2C1CON = 0; //reset config; turn off I2C
    /*calculate the BRG from the baud rate*/
    /*FSCK = (PBCLK) / ((I2CxBRG+2) * 2)
     * I2CBRG = (PBCLK / (2 *FSCK)) - 2*/
    pb_clk = Board_get_PB_clock();
    I2C1BRG = (pb_clk / (2 * IMU_BAUD)) - 2; //set baud rate to 400KHz-->verify with Oscope for correct timing
    //    printf("Board set to BRG %d",I2C1BRG);
    I2C1CONbits.ON = 1; //enable I2C
    printf("I2C1 CON state %X", I2C1CON);
    /* send start command to write settings to device */
    //    I2C1CONbits.SEN = 1;
    /* Note 1: When using the 1:1 PBCLK divisor, the user?s software should not
     * read/write the peripheral?s SFRs in the SYSCLK cycle immediately
     * following the instruction that clears the module?s ON bit.*/

    return SUCCESS;
}

/*******************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATIONS                                            *
 ******************************************************************************/
void I2C_start(void) {
    I2C1CONbits.SEN = 1;
    while (I2C1CONbits.SEN) {
        ;
    } //wait for start bit to be cleared
}

void I2C_stop(void) {
    I2C1CONbits.PEN = 1; //send stop condition
    while (I2C1CONbits.PEN) {
        ;
    } //wait for stop bit to be cleared
}

void I2C_restart(void) {
    I2C1CONbits.RSEN = 1;
    while (I2C1CONbits.RSEN) {
        ;
    } //wait for restart bit to be cleared
}

int I2C_sendByte(unsigned char byte) {
    I2C1TRN = byte; //if an address bit0 = 0 for write, 1 for read
    while (I2C1STATbits.TRSTAT) { //wait for byte to be transmitted
        ;
    }
    if (I2C1STATbits.ACKSTAT == 1) { //not acknowledged, transmission failed
        return ERROR;
    }
    return SUCCESS;
}

unsigned char I2C_readByte(void) {
    I2C1CONbits.RCEN = 1; //setting this bit initiates a receive.  Hardware clears after the received has finished
    while (I2C1CONbits.RCEN) { //wait for byte to be received
        ;
    }
    return I2C1RCV; //return the value in receive buffer
}

void __ISR(_I2C1_VECTOR, IPL2AUTO) IMU_interrupt_handler(void) {
    IFS0bits.I2C1MIF = 0; //clear flag
    IMU_cfg_I2C_state_machine();
    LATEINV = 0x4; //toggle led3 RE2
}

void IMU_cfg_I2C_state_machine(void) {
    static IMU_config_states_t current_state = IMU_IDLE;
    static IMU_config_states_t next_state = IMU_IDLE;
    static uint8_t error = FALSE;

    switch (current_state) {
        case(IMU_IDLE):
            next_state = IMU_IDLE;
            break;
        default:
            break;
    }
    current_state = next_state;
}



/**
 * @Function someFunction(void)
 * @param foo, some value
 * @return TRUE or FALSE
 * @brief 
 * @note 
 * @author <Your Name>
 * @modified <Your Name>, <year>.<month>.<day> <hour> <pm/am> */
uint8_t someFunction(int foo);

#ifdef ICM_TESTING

int main(void) {
    int retval = 0;
    int errstate = 0;
    Board_init();
    Serial_init();
    IMU_init();
    printf("\r\nICM-20948 Test Harness %s, %s\r\n", __DATE__, __TIME__);
    I2C_start();
    I2C_sendByte(ICM_I2C_ADDR << 1 | WRITE); //read from device
    I2C_sendByte(ICM_WHO_AM_I);
    I2C_stop();
    I2C_restart();
    errstate = I2C_sendByte(ICM_I2C_ADDR << 1 | READ);
    if (errstate) {
        printf("Error No ack from device\r\n");
    }
    retval = I2C_readByte();
    I2C_stop();
    printf("Device returned %d \r\n", retval);
    while (1) {
        ;
    }
}
#endif //ICM_TESTING


