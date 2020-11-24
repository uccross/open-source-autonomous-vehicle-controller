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
#define ICM_I2C_ADDR 0b1101001 //= 0x69
#define BYPASS_EN 0X2
#define MAG_NUM_BYTES 9
#define IMU_NUM_BYTES 23
#define IMU_NUM_AXES 3
#define MAG_MODE_4 0b01000
#define MAG_MODE_3 0b00110
#define MAG_MODE_2 0b00100
#define MAG_MODE_1 0b00010
#define MAG_MODE_0 0b00001
#define MAG_I2C_ADDR 0b0001100  //0x0C
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
    IMU_SEND_ADDR_W,
    IMU_SEND_REG,
    IMU_RESTART,
    IMU_SEND_ADDR_R,
    IMU_RD_DATA,
    IMU_ACK_DATA,
    IMU_DATA_RCVD,
    IMU_STOP,
} IMU_SM_states_t;

static int8_t I2C_is_configured = FALSE;

static uint8_t Mag_data[MAG_NUM_BYTES];
static uint8_t IMU_raw_data[IMU_NUM_BYTES];

struct IMU_output IMU_data = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
struct IMU_output* p_IMU_data = &IMU_data;


static volatile uint8_t IMU_data_ready = 0;
/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                 *
 ******************************************************************************/
/*blocking functions, used for init only*/
void I2C_start(void);
void I2C_stop(void);
void I2C_restart(void);
int I2C_sendByte(unsigned char byte);
unsigned char I2C_readByte(void);
uint8_t I2C_read_reg(uint8_t i2c_addr, uint8_t reg_addr);
uint8_t I2C_set_reg(uint8_t i2c_addr, uint8_t reg_addr, uint8_t setting);
/***********************************/
void IMU_run_state_machine(void);
uint8_t IMU_process_data();
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
    /*clear master interrupt flag*/
    IFS0bits.I2C1MIF = 0;

    I2C1CON = 0; //reset config; turn off I2C
    pb_clk = Board_get_PB_clock();
    I2C1BRG = (pb_clk / (2 * IMU_BAUD)) - 2; //calculate the BRG from the baud rate
    //    printf("I2C BRG: %d", I2C1BRG);
    I2C1CONbits.ON = 1; //enable I2C
    /*Config the two devices for operation*/
    /*turn on ICM by clearing bit 6 and setting bit 1*/
    I2C_set_reg(ICM_I2C_ADDR, AGB0_REG_PWR_MGMT_1, 0x2);
    /*attempt to use I2C bypass feature, this is only in effect if I2C MSTR disabled*/
    I2C_set_reg(ICM_I2C_ADDR, AGB0_REG_INT_PIN_CONFIG, BYPASS_EN);
    /*set magnetometer mode*/
    I2C_set_reg(MAG_I2C_ADDR, M_REG_CNTL2, MAG_MODE_4);
    /*Set up I2C Master operation for magnetometer*/
    I2C_set_reg(ICM_I2C_ADDR, AGB0_REG_REG_BANK_SEL, USER_BANK_3);
    /*set slave address*/
    I2C_set_reg(ICM_I2C_ADDR, AGB3_REG_I2C_SLV0_ADDR, (1 << 7 | MAG_I2C_ADDR));
    /*set starting register for slave data read*/
    I2C_set_reg(ICM_I2C_ADDR, AGB3_REG_I2C_SLV0_REG, M_REG_ST1);
    /*enable the external device and set number of bytes to read*/
    I2C_set_reg(ICM_I2C_ADDR, AGB3_REG_I2C_SLV0_CTRL, 0b10001001);
    /*now enable I2C master on bank 0*/
    I2C_set_reg(ICM_I2C_ADDR, AGB3_REG_REG_BANK_SEL, 0);
    I2C_set_reg(ICM_I2C_ADDR, AGB0_REG_USER_CTRL, 0b00100000); //I2C master enable
    /*restart interrupts*/
    __builtin_enable_interrupts();
    /*enable master interrupt*/
    IEC0bits.I2C1MIE = 1;

    return SUCCESS;
}

/**
 * @Function IMU_is_data_ready(void)
 * @return TRUE or FALSE
 * @brief TRUE if unread data is available
 * @note 
 * @author Aaron Hunter,
 * @modified  */
uint8_t IMU_is_data_ready(void) {
    return IMU_data_ready;
}

/**
 * @Function IMU_get_data(void)
 * @return pointer to IMU_output struct 
 * @brief returns most current data from the IMU
 * @note 
 * @author Aaron Hunter,
 * @modified  */
struct IMU_output* IMU_get_data(void) {
    IMU_process_data();
    return p_IMU_data;
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

uint8_t I2C_read_reg(uint8_t i2c_addr, uint8_t reg_addr) {
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

uint8_t I2C_set_reg(uint8_t i2c_addr, uint8_t reg_addr, uint8_t setting) {
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
    IMU_run_state_machine();
    LATAINV = 0x08; //toggle led
    IFS0bits.I2C1MIF = 0; //clear flag
}

/*deprecated with ISR read*/
void IMU_read_data() {
    uint8_t i;
    uint8_t err_state;
    uint8_t num_bytes = IMU_NUM_BYTES; // TODO fix this for final version to reflect whole array
    LATAINV = 0x8;
    I2C_start();
    LATAINV = 0x8;
    I2C_sendByte(ICM_I2C_ADDR << 1 | WRITE);
    LATAINV = 0x8;
    I2C_sendByte(AGB0_REG_ACCEL_XOUT_H);
    LATAINV = 0x8;
    //    I2C_stop();
    I2C_restart();
    LATAINV = 0x8;
    I2C_sendByte(ICM_I2C_ADDR << 1 | READ); //read from device
    LATAINV = 0x8;

    for (i = 0; i < num_bytes; i++) {
        I2C1CONbits.RCEN = 1; //setting this bit initiates a receive.  Hardware clears after the received has finished
        while (I2C1CONbits.RCEN) { //wait for byte to be received
            ;
        }
        LATAINV = 0x8;
        IMU_raw_data[i] = I2C1RCV;
        /*ACK the byte except last one*/
        if (i < (num_bytes - 1)) {
            I2C1CONbits.ACKDT = 0; // ACK the byte
            I2C1CONbits.ACKEN = 1;
            while (I2C1CONbits.ACKEN == 1);
        } else {
            I2C1CONbits.ACKDT = 1; // NACK the byte
            I2C1CONbits.ACKEN = 1;
            while (I2C1CONbits.ACKEN == 1);
        }
        LATAINV = 0x8;
    }
    I2C_stop();
    LATAINV = 0x8;
}

void IMU_run_state_machine(void) {
    static IMU_SM_states_t current_state = IMU_SEND_ADDR_W;
    IMU_SM_states_t next_state = IMU_SEND_ADDR_W;
    static uint8_t error = FALSE;
    static uint8_t byte_count;

    switch (current_state) {
        case(IMU_SEND_ADDR_W):
            next_state = IMU_SEND_REG;
            I2C1TRN = (ICM_I2C_ADDR << 1 | WRITE);
            /*reset error and data ready flags*/
            IMU_data_ready = FALSE;
            error = FALSE;
            break;
        case(IMU_SEND_REG):
            if (I2C1STATbits.ACKSTAT == NACK) { // check for ack
                error = TRUE;
                I2C1CONbits.PEN = 1; //send stop condition 
                next_state = IMU_STOP;
            } else {
                I2C1TRN = AGB0_REG_ACCEL_XOUT_H; //load device register into I2C transmit buffer
                next_state = IMU_RESTART;
            }
            break;
        case(IMU_RESTART):
            if (I2C1STATbits.ACKSTAT == NACK) { // check for ack
                error = TRUE;
                I2C1CONbits.PEN = 1; //send stop condition 
                next_state = IMU_STOP;
            } else {
                I2C1CONbits.RSEN = 1; //send restart condition
                next_state = IMU_SEND_ADDR_R;
            }
            break;
        case(IMU_SEND_ADDR_R):
            I2C1TRN = (ICM_I2C_ADDR << 1 | READ);
            byte_count = 0; //reset the byte counter
            next_state = IMU_RD_DATA;
            break;
        case(IMU_RD_DATA):
            /*need to check the I2C address was sent (byte_count ==0)*/
            if (byte_count == 0 && I2C1STATbits.ACKSTAT == NACK) {
                error = TRUE;
                I2C1CONbits.PEN = 1; //send stop condition 
                next_state = IMU_STOP;
            } else {
                I2C1CONbits.RCEN = 1; //enable receive data
                next_state = IMU_ACK_DATA;
            }
            break;
        case(IMU_ACK_DATA):
            IMU_raw_data[byte_count] = I2C1RCV; // get and store data in array
            byte_count++; //increment and check byte count
            if (byte_count == IMU_NUM_BYTES) {
                I2C1CONbits.ACKDT = 1; // NACK the byte
                next_state = IMU_DATA_RCVD;
            } else {
                I2C1CONbits.ACKDT = 0; // ACK the byte
                next_state = IMU_RD_DATA;
            }
            I2C1CONbits.ACKEN = 1; //send ACK/NACK
            break;
        case(IMU_DATA_RCVD):
            /*indicate data is ready*/
            IMU_data_ready = 1;
            /*stop the device*/
            I2C1CONbits.PEN = 1; //send stop condition 
            next_state = IMU_STOP;
            break;
        case(IMU_STOP):
            next_state = IMU_SEND_ADDR_W;
            break;
        default:
            I2C1CONbits.PEN = 1; //send stop condition
            next_state = IMU_STOP;
            break;
    }
    current_state = next_state;
}

/**
 * @Function IMU_process_data(void)
 * @return SUCCESS or ERROR
 * @brief converts raw register data into IMU values for output
 * @note mag data has reversed endianness
 * @author Aaron Hunter,
 * @modified  */
uint8_t IMU_process_data() {
    IMU_data.acc.x = IMU_raw_data[0] << 8 | IMU_raw_data[1];
    IMU_data.acc.y = IMU_raw_data[2] << 8 | IMU_raw_data[3];
    IMU_data.acc.z = IMU_raw_data[4] << 8 | IMU_raw_data[5];
    IMU_data.gyro.x = IMU_raw_data[6] << 8 | IMU_raw_data[7];
    IMU_data.gyro.y = IMU_raw_data[8] << 8 | IMU_raw_data[9];
    IMU_data.gyro.z = IMU_raw_data[10] << 8 | IMU_raw_data[11];
    IMU_data.temp = IMU_raw_data[12] << 8 | IMU_raw_data[13];
    IMU_data.mag.x = IMU_raw_data[16] << 8 | IMU_raw_data[15];
    IMU_data.mag.y = IMU_raw_data[18] << 8 | IMU_raw_data[17];
    IMU_data.mag.z = IMU_raw_data[20] << 8 | IMU_raw_data[19];
    /*status 1 is high byte and status 2 is low byte*/
    IMU_data.mag_status = (IMU_raw_data[14] & 0x3) << 8 | (IMU_raw_data[22] & 0x4);
    return SUCCESS;
}

#ifdef ICM_TESTING

int main(void) {
    int value = 0;
    int i;
    int errstate = 0;
    struct IMU_output* p_data;

    Board_init();
    Serial_init();
    IMU_init();
    //set LEDs for troubleshooting
    TRISAbits.TRISA4 = 0; //pin 72 Max32
    TRISAbits.TRISA3 = 0; //built-in LED, pin 13
    LATACLR = 0x18;
    printf("\r\nICM-20948 Test Harness %s, %s\r\n", __DATE__, __TIME__);

    while (1) {
        LATACLR = 0x8;
        if (IMU_is_data_ready() == TRUE) {
            p_data = IMU_get_data();
            printf("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d,%d\r\n",
                    p_data->acc.x, p_data->acc.y, p_data->acc.z,
                    p_data->gyro.x, p_data->gyro.y, p_data->gyro.z,
                    p_data->mag.x, p_data->mag.y, p_data->mag.z,
                    p_data->temp, p_data->mag_status);
        }
        /*delay to not overwhelm the serial port*/
        for (i = 0; i < 1000000; i++) {
            ;
        }
        LATACLR = 0x8;
        I2C1CONbits.SEN = 1;
    }
}
#endif //ICM_TESTING


