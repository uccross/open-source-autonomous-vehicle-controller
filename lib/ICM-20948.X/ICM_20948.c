/* 
 * File:   ICM_20948.c
 * Author: Aaron Hunter
 * Brief: Library for the ICM-20948 IMU
 * Created on Nov 13, 2020 9:46 am
 * Modified on 
 */

/*******************************************************************************
 * #INCLUDES                                                                   *
 ******************************************************************************/

#include "ICM_20948.h" // The header file for this source file. 
#include "ICM_20948_registers.h"  //register definitions for the device
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
#define BYPASS_EN 0X2
#define MAG_NUM_BYTES 9
#define IMU_NUM_BYTES 22
#define MAG_MODE_4 0b01000
#define MAG_MODE_3 0b00110
#define MAG_MODE_2 0b00100
#define MAG_MODE_1 0b00010
#define MAG_MODE_0 0b00001
#define MAG_I2C_ADDR 0b0001100  //hex 0C
#define MAG_WHO_I_AM_1 0
#define MAG_WHO_I_AM_2 1
#define USER_BANK_0 0
#define USER_BANK_1 0b00010000
#define USER_BANK_2 0b00100000
#define USER_BANK_3 0b00110000

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

static uint8_t Mag_data[MAG_NUM_BYTES];
static uint8_t IMU_data[IMU_NUM_BYTES];
/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                 *
 ******************************************************************************/
/*blocking functions, deprecated*/
void I2C_start(void);
void I2C_stop(void);
void I2C_restart(void);
int I2C_sendByte(unsigned char byte);
unsigned char I2C_readByte(void);
uint8_t IMU_get_byte(uint8_t i2c_addr, uint8_t reg_addr);
uint8_t ICM_set_reg(uint8_t i2c_addr, uint8_t reg_addr, uint8_t setting);
void MAG_read_data();
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
    uint8_t value;
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
    printf("I2C1 CON state %X\r\n", I2C1CON);
    /* send start command to write settings to device */
    //    I2C1CONbits.SEN = 1;
    /* Note 1: When using the 1:1 PBCLK divisor, the user?s software should not
     * read/write the peripheral?s SFRs in the SYSCLK cycle immediately
     * following the instruction that clears the module?s ON bit.*/
    /*Config the two devices for operation*/
    //    value = IMU_get_byte(ICM_I2C_ADDR, AGB0_REG_WHO_AM_I);
    //    printf("ICM returned %d \r\n", value);
    /*turn on device by clearing bit 6 and setting bit1*/
    ICM_set_reg(ICM_I2C_ADDR, AGB0_REG_PWR_MGMT_1, 0x2);
    //    value = IMU_get_byte(ICM_I2C_ADDR, AGB0_REG_PWR_MGMT_1);
    //    printf("ICM returned %d \r\n", value);
    /*attempt to use I2C bypass feature, this is only in effect if I2C MSTR disabled*/
    ICM_set_reg(ICM_I2C_ADDR, AGB0_REG_INT_PIN_CONFIG, BYPASS_EN);
    //        value = IMU_get_byte(ICM_I2C_ADDR, AGB0_REG_INT_PIN_CONFIG);
    //        printf("ICM returned %d \r\n", value);
    /*now see if we can contact the mag*/
    //    value = IMU_get_byte(MAG_I2C_ADDR, M_REG_WIA2);
    //    printf("MAG returned 0x%x \r\n", value);
    /*set magnetometer mode*/
    ICM_set_reg(MAG_I2C_ADDR, M_REG_CNTL2, MAG_MODE_4);
    value = IMU_get_byte(MAG_I2C_ADDR, M_REG_CNTL2);
    printf("MAG returned 0x%x \r\n", value);
    /*Set up I2C Master operation for magnetometer*/
    ICM_set_reg(ICM_I2C_ADDR, AGB0_REG_REG_BANK_SEL, USER_BANK_3);
    /*set slave address*/
    ICM_set_reg(ICM_I2C_ADDR, AGB3_REG_I2C_SLV0_ADDR, (1 << 7 | MAG_I2C_ADDR));
    /*set starting register for slave data read*/
    ICM_set_reg(ICM_I2C_ADDR, AGB3_REG_I2C_SLV0_REG, M_REG_ST1);
    /*enable the external device and set number of bytes to read*/
    ICM_set_reg(ICM_I2C_ADDR, AGB3_REG_I2C_SLV0_CTRL, 0b10001001);
    /*now enable I2C master on bank 0*/
    ICM_set_reg(ICM_I2C_ADDR, AGB3_REG_REG_BANK_SEL, 0);
    ICM_set_reg(ICM_I2C_ADDR, AGB0_REG_USER_CTRL, 0b00100000); //I2C master enable

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

uint8_t IMU_get_byte(uint8_t i2c_addr, uint8_t reg_addr) {
    uint8_t ret_val;
    uint8_t err_state;
    I2C_start();
    I2C_sendByte(i2c_addr << 1 | WRITE); //read from device
    I2C_sendByte(reg_addr);
    // I2C_stop();
    I2C_restart();
    err_state = I2C_sendByte(i2c_addr << 1 | READ);
    ret_val = I2C_readByte();
    I2C_stop();
    return ret_val;
}

uint8_t ICM_set_reg(uint8_t i2c_addr, uint8_t reg_addr, uint8_t setting) {
    uint8_t ret_val;
    uint8_t err_state;
    I2C_start();
    I2C_sendByte(i2c_addr << 1 | WRITE); //read from device
    I2C_sendByte(reg_addr);
    I2C_sendByte(setting);
    I2C_stop();
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

void IMU_read_data() {
    uint8_t i;
    uint8_t err_state;
    uint8_t num_bytes = IMU_NUM_BYTES; // TODO fix this for final version to reflect whole array
    I2C_start();
    I2C_sendByte(ICM_I2C_ADDR << 1 | WRITE);
    I2C_sendByte(AGB0_REG_ACCEL_XOUT_H);
    //    I2C_stop();
    I2C_restart();
    I2C_sendByte(ICM_I2C_ADDR << 1 | READ); //read from device

    for (i = 0; i < num_bytes; i++) {
        I2C1CONbits.RCEN = 1; //setting this bit initiates a receive.  Hardware clears after the received has finished
        while (I2C1CONbits.RCEN) { //wait for byte to be received
            ;
        }
        IMU_data[i] = I2C1RCV;
        /*ACK the byte except last one*/
        if (i < (num_bytes - 1)) {
            I2C1CONbits.ACKDT = 0; // NACK the byte
            I2C1CONbits.ACKEN = 1;
            while (I2C1CONbits.ACKEN == 1);
        }
    }
    I2C_stop();
}

void MAG_read_data() {
    uint8_t i;
    uint8_t err_state;
    uint8_t num_bytes = MAG_NUM_BYTES; // TODO fix this for final version to reflect whole array
    I2C_start();
    I2C_sendByte(MAG_I2C_ADDR << 1 | WRITE);
    I2C_sendByte(M_REG_ST1);
    I2C_restart();
    err_state = I2C_sendByte(MAG_I2C_ADDR << 1 | READ);

    for (i = 0; i < num_bytes; i++) {
        I2C1CONbits.RCEN = 1; //setting this bit initiates a receive.  Hardware clears after the received has finished
        while (I2C1CONbits.RCEN) { //wait for byte to be received
            ;
        }
        Mag_data[i] = I2C1RCV;
        /*ACK the byte except last one*/
        if (i < (num_bytes - 1)) {
            I2C1CONbits.ACKDT = 0; // NACK the byte
            I2C1CONbits.ACKEN = 1;
            while (I2C1CONbits.ACKEN == 1);
        }
    }
    I2C_stop();
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
    int value = 0;
    int i;
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyr_x;
    int16_t gyr_y;
    int16_t gyr_z;
    int16_t temp;

    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;

    uint8_t mag_status_1;
    uint8_t mag_status_2;

    int errstate = 0;
    Board_init();
    Serial_init();
    IMU_init();
    printf("\r\nICM-20948 Test Harness %s, %s\r\n", __DATE__, __TIME__);


    while (1) {
        IMU_read_data();
        acc_x = IMU_data[0] << 8 | IMU_data[1];
        acc_y = IMU_data[2] << 8 | IMU_data[3];
        acc_z = IMU_data[4] << 8 | IMU_data[5];
        gyr_x = IMU_data[6] << 8 | IMU_data[7];
        gyr_y = IMU_data[8] << 8 | IMU_data[9];
        gyr_z = IMU_data[10] << 8 | IMU_data[11];
        temp = IMU_data[12] << 8 | IMU_data[13];
        mag_status_1 = IMU_data[14] & 0x3;
        mag_x = IMU_data[16] << 8 | IMU_data[15];
        mag_y = IMU_data[18] << 8 | IMU_data[17];
        mag_z = IMU_data[20] << 8 | IMU_data[19];
        mag_status_2 = IMU_data[21] & 0x4;
        //        MAG_read_data();
        //        mag_status_1 = Mag_data[0] & 0x3;
        //        mag_x = Mag_data[2] << 8 | Mag_data[1];
        //        mag_y = Mag_data[4] << 8 | Mag_data[3];
        //        mag_z = Mag_data[6] << 8 | Mag_data[5];
        //        mag_status_2 = Mag_data[8] & 0x4;
        printf("a:[%d, %d, %d], g:[%d, %d, %d],T:%d, ST1:%d, m:[%d, %d, %d], ST2:%d\r\n", acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, temp, mag_status_1, mag_x, mag_y, mag_z, mag_status_2);
        //        printf("MAG: DR/ODR: %d, %d, %d, %d, HOFL: %x \r\n", mag_status_1, mag_x, mag_y, mag_z, mag_status_2);
        for (i = 0; i < 1000000; i++) {
            ;
        }
    }
}
#endif //ICM_TESTING


