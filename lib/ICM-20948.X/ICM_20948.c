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
#include <string.h>
#include <sys/attribs.h>  //for ISR definitions
#include <proc/p32mx795f512l.h>


/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/
#define IMU_I2C_FREQ 400000  //I2C fast mode
#define IMU_SPI_FREQ 5000000ul //5MHz clock rate
#define ICM_DEV_ID 0xea 
#define ICM_I2C_ADDR 0b1101001 //= 0x69
#define BYPASS_EN 0X2
#define MAG_NUM_BYTES 9
#define IMU_NUM_BYTES 23
#define MAG_DEV_ID 0x9
#define MAG_I2C_ADDR 0b0001100  //0x0C
#define MAG_MODE_4 0b01000
#define MAG_MODE_3 0b00110
#define MAG_MODE_2 0b00100
#define MAG_MODE_1 0b00010
#define MAG_MODE_0 0b00001

#define USER_BANK_0 0
#define USER_BANK_1 0b00010000
#define USER_BANK_2 0b00100000
#define USER_BANK_3 0b00110000

#define ACK 0
#define NACK 1
#define READ 1
#define WRITE 0
#define INTERFACE_MODE IMU_SPI_MODE
//#define INTERFACE_MODE IMU_I2C_MODE 
#define IMU_CS_TRIS TRISEbits.TRISE0 //chip select for IMU
#define IMU_CS_LAT LATEbits.LATE0

/*IMU scaling factors*/
#define ACCEL_SCALE 2000  // +/-2000mg
#define ACCEL_DIV 15 // 1/2^15
#define GYRO_SCALE 500 // +/- 500 deg/sec
#define GYRO_DIV 15 // 1/2^15
#define MAG_SCALE 49000 //49000 mGauss FS
#define MAG_DIV 15 // 1/2^15
#define E_g 1000 // expected value of gravity in mG
#define G2MSS 9.80665k/1000 //conversion from g to m/s/s
#define PI 3.141592653589793k
#define DEG2RAD 0.017453292519943295k //conversion from degrees to rad
#define DEG2MRAD  17.453292519943297k //conversion from degrees to mrad
#define E_b 475 //expected value of earth's mag field in mGauss
#define MG2GAUSS 1/1000k  //milliGauss to Gauss
#define T_BIAS 0
#define T_SENSE 333.87k
#define T_OFFSET 21
#define RAW 0
#define SCALED 1
#define HIGHRES 2

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

typedef enum {
    IMU_SPI_SEND_NEXT_REG,
    IMU_SPI_READ_LAST_REG,
} IMU_SPI_SM_states_t;

/*module level variables*/
static uint8_t IMU_raw_data[IMU_NUM_BYTES];
static accum acc_v_raw[3] = {0, 0, 0};
static accum acc_v_scaled[3] = {0, 0, 0};
static accum acc_v_cald[3] = {0, 0, 0};
static accum mag_v_raw[3] = {0, 0, 0};
static accum mag_v_scaled[3] = {0, 0, 0};
static accum mag_v_cald[3] = {0, 0, 0};
static accum gyro_v_raw[3] = {0, 0, 0};
static accum gyro_v_scaled[3] = {0, 0, 0};
static accum gyro_v_cald[3] = {0, 0, 0};
static accum temp_raw = 0;
static accum temp_scaled = 0;
static accum temp_cald = 0;
static int16_t status = 0;

static volatile uint8_t IMU_data_ready = 0;

static accum A_mag[3][3] = {
    {1, 0, 0},
    { 0, 1, 0},
    { 0, 0, 1}
};
static accum b_mag[3] = {0, 0, 0};

static accum A_acc[3][3] = {
    {1, 0, 0},
    { 0, 1, 0},
    { 0, 0, 1}
};
static accum b_acc[3] = {0, 0, 0};

static accum A_gyro[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
};
static accum b_gyro[3] = {0, 0, 0};
/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                 *
 ******************************************************************************/
/*blocking functions, used for init */
static void I2C_start(void);
static void I2C_stop(void);
static void I2C_restart(void);
static int I2C_send_byte(unsigned char byte);
static unsigned char I2C_read_byte(void);
static uint8_t I2C_read_reg(uint8_t i2c_addr, uint8_t reg_addr);
static uint8_t I2C_set_reg(uint8_t i2c_addr, uint8_t reg_addr, uint8_t setting);
static uint8_t SPI_read_reg(uint8_t reg_addr);
static uint8_t SPI_set_reg(uint8_t reg_addr, uint8_t value);
static uint8_t SPI_read_reg(uint8_t reg_addr);
static void SPI_read_data(void);

/***********************************/
static void delay(int cycles);
static void IMU_run_I2C_state_machine(void);
/**
 * @Function IMU_run_SPI_state_machine(uint8_t byte_read)
 * @return none
 * @param byte_read, the byte read from SPI 1 buffer SPI1BUF
 * @brief state machine to reads IMU data registers over SPI
 * @note called with a single SPI read of the IMU
 * @author ahunter
 **/
static void IMU_run_SPI_state_machine(uint8_t byte_read);

/**
 * @Function IMU_process_data(void)
 * @param none
 * @return SUCCESS or ERROR
 * @brief converts raw register data into module variable data
 * @note mag data has reversed byte order
 * @author ahunter
 * @modified  */
static uint8_t IMU_process_data(void);

/**
 * @Function IMU_assign_data_to_output(struct IMU_output* IMU_data);
 * @param IMU_data, a struct with accel, gyro and mag values on 3 axes
 * @return SUCCESS or ERROR
 * @brief sets output struct to point module vector variables
 * @author ahunter
 * @modified  */
static uint8_t IMU_assign_data_to_output(struct IMU_output* IMU_data);

/**
 * @Function cal_to_fix(float A[3][3], b[3], accum * A_fx, accum* b_fx);
 * @param A input float point matrix, b input float point vector 
 * @brief sets the fixed point cal matrix to the float values
 * @return SUCCESS or ERROR
 */
static int8_t cal_to_fix(float A[3][3], float b[3], accum A_fx[3][3], accum b_fx[3]);

/**
 * @function m_v_mult()
 * Multiplies a matrix with a vector
 * @param m A matrix to be multiplied with a vector
 * @param v A vector to be multiplied with a matrix
 * @param v_out The product of the matrix and vector
 * @return SUCCESS or ERROR
 */
static uint8_t m_v_mult(accum m[MSZ][MSZ], accum v[MSZ], accum v_out[MSZ]);

/**
 * @function v_v_add()
 * Add a vector value to a vector
 * @param v1 Vector to add to another vector
 * @param v2 Vector to have a vector added to it
 * @param v_out Vector as sum of two vectors
 */
static void v_v_add(accum v1[MSZ], accum v2[MSZ], accum v_out[MSZ]);

/**
 * @function m_scale()
 * Scales matrix
 * @param s Scalar to scale matrix with
 * @param m Matrix to be scaled
 */
static void m_scale(accum s, accum m[MSZ][MSZ]);

/**
 * @function v_scale()
 * Scales matrix
 * @param s Scalar to scale matrix with
 * @param m Matrix to be scaled
 */
static void v_scale(accum s, accum v[MSZ]);
/*******************************************************************************
 * PUBLIC FUNCTION IMPLEMENTATIONS                                             *
 ******************************************************************************/

/**
 * @Function IMU_init(void)
 * @return SUCCESS or ERROR
 * @brief initializes the IMU using the desired interface
 * @note I2C is bare bones and left for reference.  Preferred method is SPI.
 * I2C is less reliable due to logic level converters in the ICM-20948
 * @author Aaron Hunter,
 * @modified  */
uint8_t IMU_init(char interface_mode) {
    uint32_t pb_clk;
    uint8_t value = 0;
    pb_clk = Board_get_PB_clock();
    if (interface_mode == IMU_I2C_MODE) {
        __builtin_disable_interrupts();
        /*config priority and subpriority--must match IPL level*/
        IPC6bits.I2C1IP = 2;
        IPC6bits.I2C1IS = 0;
        /*clear master interrupt flag*/
        IFS0bits.I2C1MIF = 0;

        I2C1CON = 0; //reset config; turn off I2C
        I2C1BRG = (pb_clk / (2 * IMU_I2C_FREQ)) - 2; //calculate the BRG from the baud rate
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
    } else {
        /*Disable the SPI interrupts in the respective IEC0/1 register.*/
        __builtin_disable_interrupts();
        SPI1CON = 0; // Stop and reset the SPI module by clearing the ON bit.
        /* Note: When using 1:1 PBCLK divisor, the user's software should not
         *  read/write the peripheral's SFRs in the SYSCLK cycle immediately
         *  following the instruction that clears the module's ON bit.*/
        /*initialize chip select pins*/
        IMU_CS_TRIS = 0; // set CS as output 
        IMU_CS_LAT = 1; // deselect device 
        SPI1BUF; // clear receive buffer 
        /*set up SPI1 RX interrupt*/
        IFS0bits.SPI1RXIF = 0; // clear interrupt flag
        IEC0bits.SPI1RXIE = 1; //enable interrupt
        IPC5bits.SPI1IP = 5; //interrupt priority 5
        IPC5bits.SPI1IS = 0; //subpriority 0
        SPI1BRG = pb_clk / (2 * IMU_SPI_FREQ) - 1; // set frequency 
        SPI1STATbits.SPIROV = 0; // clear any overflow condition 
        /*Write the desired settings to the SPIxCON register with MSTEN (SPIxCON<5>) = 1.*/
        SPI1CONbits.MODE32 = 0;
        SPI1CONbits.MODE16 = 0; //8 bit mode
        SPI1CONbits.SMP = 1; /* set sample at end of data*/
        SPI1CONbits.SSEN = 0; // manually drive CS/SS 
        SPI1CONbits.CKP = 1; /*set clock phase to idle high*/
        SPI1CONbits.CKE = 0; /* set to read on falling edge (active --> idle)*/
        SPI1CONbits.MSTEN = 1; // set as master
        SPI1CONbits.ON = 1; /* enable SPI system*/
        /*verify device ID*/
        SPI_set_reg(AGB0_REG_REG_BANK_SEL, USER_BANK_0);
        value = SPI_read_reg(AGB0_REG_WHO_AM_I);
        if (value != ICM_DEV_ID) {
            //printf("IMU not found!\r\n");
            return ERROR;
        }
        //printf("IMU returned who am I = 0x%x \r\n", value);
        SPI_set_reg(AGB0_REG_USER_CTRL, 0x30); //enable master I2C, disable slave I2C interface
        SPI_set_reg(AGB0_REG_PWR_MGMT_1, 0x01); //clear sleep bit and set clock to best available
        /*switch to user bank 3 to configure slave devices*/
        SPI_set_reg(AGB0_REG_REG_BANK_SEL, USER_BANK_3);
        /*get who I am data from mag*/
        SPI_set_reg(AGB3_REG_I2C_SLV4_ADDR, (READ << 7 | MAG_I2C_ADDR)); /*load the I2C address of the magnetometer*/
        SPI_set_reg(AGB3_REG_I2C_SLV4_REG, M_REG_WIA2); //load who I am 2 register
        SPI_set_reg(AGB3_REG_I2C_SLV4_CTRL, 0x80); //execute the read by setting the enable bit
        while (SPI_read_reg(AGB3_REG_I2C_SLV4_CTRL)) { //bit 7 is cleared when the transaction is complete
            ;
        }
        value = SPI_read_reg(AGB3_REG_I2C_SLV4_DI); /*read the data returned by the mag*/
        if (value != MAG_DEV_ID) {
            //printf("Magnetometer not found!\r\n");
            return ERROR;
        }
        //printf("Magnetometer returned who I am 2 = 0x%x\r\n", value); /*mag should return 0x9*/
        SPI_set_reg(AGB3_REG_I2C_SLV4_ADDR, MAG_I2C_ADDR); /*load the I2C address of the magnetometer*/
        /*set the mag parameters on mag control 2 register*/
        SPI_set_reg(AGB3_REG_I2C_SLV4_REG, M_REG_CNTL2);
        SPI_set_reg(AGB3_REG_I2C_SLV4_DO, MAG_MODE_4); //100 Hz output
        SPI_set_reg(AGB3_REG_I2C_SLV4_CTRL, 0x80); //set the enable bit to execute the transaction
        while (SPI_read_reg(AGB3_REG_I2C_SLV4_CTRL)) { //bit 7 is cleared when the transaction is complete
            ;
        }
        /*set up the slave data registers to be read on slave 0*/
        SPI_set_reg(AGB3_REG_I2C_SLV0_ADDR, READ << 7 | MAG_I2C_ADDR);
        SPI_set_reg(AGB3_REG_I2C_SLV0_REG, M_REG_ST1); //starting register for data
        SPI_set_reg(AGB3_REG_I2C_SLV0_CTRL, 0b10001001); //read nine bytes and set enable bit
        /*configurations for gyro and accel*/
        SPI_set_reg(AGB3_REG_REG_BANK_SEL, USER_BANK_2);
        SPI_set_reg(AGB2_REG_GYRO_CONFIG_1, 0b00010011); //119.5hz 3dB low pass, +/-500 dps FS
        SPI_set_reg(AGB2_REG_ACCEL_CONFIG, 0b00010001); // 114Hz 3dB LP, +/-2g FS
        /*return to user bank 0 for data read*/
        SPI_set_reg(AGB3_REG_REG_BANK_SEL, USER_BANK_0);
        /*restart interrupts*/
        __builtin_enable_interrupts();
        /*retrieve calibration matrices and vectors*/
    }
    return SUCCESS;
}

/**
 * @Function IMU_start_data_acq(void);
 * @return none
 * @param none
 * @brief this function starts the SPI data read
 * @author Aaron Hunter
 */
int8_t IMU_start_data_acq(void) {
    int8_t error = FALSE;
    if (IMU_CS_LAT == 0) {
       // printf("IMU error found\r\n");
        SPI1BUF; //read buffer
        IFS0bits.SPI1RXIF = 0; //clear any interrupt flag
        error = TRUE;
    } else {
        error = FALSE;
    }
    uint8_t data_reg = AGB0_REG_ACCEL_XOUT_H;
    data_reg = data_reg | (READ << 7);
    IMU_CS_LAT = 0; //select the IMU 
    SPI1BUF = data_reg; //start SPI transaction 
    if (error) {
        return ERROR;
    }
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
 * @Function IMU_get_raw_data(void)
 * @return pointer to IMU_output struct 
 * @brief returns most current (raw) data from the IMU
 * @note 
 * @author Aaron Hunter,
 * @modified  */
uint8_t IMU_get_raw_data(struct IMU_output* IMU_data) {
    /*only process data if it's new*/
    if (IMU_data_ready == TRUE) {
        IMU_process_data();
        IMU_data_ready = FALSE; //clear the data ready flag
    }
    /* set output data to point at vector components*/
    IMU_data->acc.x = acc_v_raw[0];
    IMU_data->acc.y = acc_v_raw[1];
    IMU_data->acc.z = acc_v_raw[2];
    IMU_data->gyro.x = gyro_v_raw[0];
    IMU_data->gyro.y = gyro_v_raw[1];
    IMU_data->gyro.z = gyro_v_raw[2];
    IMU_data->temp = temp_raw;
    IMU_data->mag.x = mag_v_raw[0];
    IMU_data->mag.y = mag_v_raw[1];
    IMU_data->mag.z = mag_v_raw[2];
    IMU_data->mag_status = status;
    return SUCCESS;
}

/**
 * @Function IMU_get_scaled_data(void)
 * @return pointer to IMU_output struct 
 * @brief returns most current scaled data from the IMU
 * @note 
 * @author Aaron Hunter,
 **/
uint8_t IMU_get_scaled_data(struct IMU_output* IMU_data) {
    int row;
    /*get raw data and store in module vectors*/
    if (IMU_data_ready == TRUE) {
        IMU_process_data();
        IMU_data_ready = FALSE; //clear the data ready flag
    }
    /* set output data to point at vector components*/
    IMU_data->acc.x = acc_v_cald[0];
    IMU_data->acc.y = acc_v_cald[1];
    IMU_data->acc.z = acc_v_cald[2];
    IMU_data->gyro.x = gyro_v_scaled[0];
    IMU_data->gyro.y = gyro_v_scaled[1];
    IMU_data->gyro.z = gyro_v_scaled[2];
    IMU_data->temp = temp_scaled;
    IMU_data->mag.x = mag_v_cald[0];
    IMU_data->mag.y = mag_v_cald[1];
    IMU_data->mag.z = mag_v_cald[2];
    IMU_data->mag_status = status;
    return SUCCESS;
}

/**
 * @Function IMU_set_mag_cal(accum A[MSZ][MSZ], accum b[MSZ])
 * @param cal contains the A scaling matrix and b bias vector for the mag
 * @brief sets scaling matrix and offset vector for magnetometer 
 * @note bias vector is assumed to be normalized to one, so it gets scaled
 * to the expected magnitude of magnetic field, i.e., 475 mGauss
 * @return SUCCESS or ERROR
 * @author Aaron Hunter,
 **/
int8_t IMU_set_mag_cal(accum A[MSZ][MSZ], accum b[MSZ]) {
    if (A != NULL && b != NULL) {
        memcpy(A_mag, A, sizeof (A_mag));
        memcpy(b_mag, b, sizeof (b_mag));
        v_scale(E_b, b_mag); // need to scale the offset into eng units
        return SUCCESS;
    } else {
        return ERROR;
    }
}

/**
 * @Function IMU_set_acc_cal(accum A[MSZ][MSZ], accum b[MSZ])
 * @param A source matrix
 * @param b source offset
 * @brief sets scaling matrix and offset vector for accelerometer 
 * @note calibration is assumed to be normalized to one, so bias is scaled
 * by 1000mG, the expected value of the earth's gravitational field. 
 * @return SUCCESS or ERROR
 * @author Aaron Hunter,
 **/
int8_t IMU_set_acc_cal(accum A[MSZ][MSZ], accum b[MSZ]) {
    if (A != NULL && b != NULL) {
        memcpy(A_acc, A, sizeof (A_acc));
        memcpy(b_acc, b, sizeof (b_acc));
        v_scale(E_g, b_acc); // need to scale the offset into eng units
        return SUCCESS;
    } else {
        return ERROR;
    }
}

/**
 * @Function IMU_set_gyro_cal(accum A[MSZ][MSZ], accum b[MSZ])
 * @param A source matrix
 * @param b source offset
 * @brief sets scaling matrix and offset vector for gyroscope 
 * @note Scaling parameters on diagonal (x,y,z) if cross terms unknown. Also,
 * bias values are assumed in raw counts and get scaled to engineering units
 * @return SUCCESS or ERROR
 * @author Aaron Hunter,
 **/
int8_t IMU_set_gyro_cal(accum A[MSZ][MSZ], accum b[MSZ]) {
    if (A != NULL && b != NULL) {
        memcpy(A_gyro, A, sizeof (A_gyro));
        memcpy(b_gyro, b, sizeof (b_gyro));
        v_scale(GYRO_SCALE, b_gyro); // need to scale the offset into eng units
        return SUCCESS;
    } else {
        return ERROR;
    }
}

/**
 * @Function IMU_get_mag_cal(accum A[MSZ][MSZ], accum b[MSZ])
 * @param A destination matrix
 * @param b destination offset
 * @brief gets scaling matrix and offset vector for magnetometer 
 * @note bias is scaled to mGauss
 * @return SUCCESS or ERROR
 * @author Aaron Hunter,
 **/
int8_t IMU_get_mag_cal(accum A[MSZ][MSZ], accum b[MSZ]) {
    if (A != NULL && b != NULL) {
        memcpy(A, A_mag, sizeof (A_mag));
        memcpy(b, b_mag, sizeof (b_mag));
        return SUCCESS;
    } else {
        return ERROR;
    }
}

/**
 * @Function IMU_get_acc_cal(accum A[MSZ][MSZ], accum b[MSZ])
 * @param A destination matrix
 * @param b destination offset
 * @brief gets scaling matrix and offset vector for accelerometer 
 * @note bias is scaled by 1000mG, the expected value of the earth's 
 * gravitational field. 
 * @return SUCCESS or ERROR
 * @author Aaron Hunter,
 **/
int8_t IMU_get_acc_cal(accum A[MSZ][MSZ], accum b[MSZ]) {
    if (A != NULL && b != NULL) {
        memcpy(A, A_acc, sizeof (A_acc));
        memcpy(b, b_acc, sizeof (b_acc));
        return SUCCESS;
    } else {
        return ERROR;
    }
}

/**
 * @Function IMU_get_gyro_cal(accum A[MSZ][MSZ], accum b[MSZ])
 * @param A destination matrix
 * @param b destination offset
 * @brief gets scaling matrix and offset vector for gyros 
 * @note bias is scaled to dps, not counts
 * @return SUCCESS or ERROR
 * @author Aaron Hunter,
 **/
int8_t IMU_get_gyro_cal(accum A[MSZ][MSZ], accum b[MSZ]) {
    if (A != NULL && b != NULL) {
        memcpy(A, A_gyro, sizeof (A_gyro));
        memcpy(b, b_gyro, sizeof (b_gyro));
        return SUCCESS;
    } else {
        return ERROR;
    }
}

/*******************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATIONS                                            *
 ******************************************************************************/

/**
 * @Function delay(int cycles)
 * @param cycles, number of cycles to delay
 * @brief simlple delay loop
 * @note ~500nsec for one delay, then +12.5 nsec for every increment higher
 * @author ahunter
 */
static void delay(int cycles) {
    int i;
    for (i = 0; i < cycles; i++) {
        ;
    }
}

static void I2C_start(void) {
    I2C1CONbits.SEN = 1;
    while (I2C1CONbits.SEN) {
        ;
    } //wait for start bit to be cleared
}

static void I2C_stop(void) {
    I2C1CONbits.PEN = 1; //send stop condition
    while (I2C1CONbits.PEN) {
        ;
    } //wait for stop bit to be cleared
}

static void I2C_restart(void) {
    I2C1CONbits.RSEN = 1;
    while (I2C1CONbits.RSEN) {
        ;
    } //wait for restart bit to be cleared
}

static int I2C_send_byte(unsigned char byte) {
    I2C1TRN = byte; //if an address bit0 = 0 for write, 1 for read
    while (I2C1STATbits.TRSTAT) { //wait for byte to be transmitted
        ;
    }
    if (I2C1STATbits.ACKSTAT == 1) { //not acknowledged, transmission failed
        return ERROR;
    }
    return SUCCESS;
}

static uint8_t I2C_read_reg(uint8_t i2c_addr, uint8_t reg_addr) {
    uint8_t ret_val;
    uint8_t err_state;
    I2C_start();
    I2C_send_byte(i2c_addr << 1 | WRITE); //read from device
    I2C_send_byte(reg_addr);
    I2C_restart();
    err_state = I2C_send_byte(i2c_addr << 1 | READ);
    ret_val = I2C_read_byte();
    I2C_stop();
    return ret_val;
}

static uint8_t I2C_set_reg(uint8_t i2c_addr, uint8_t reg_addr, uint8_t setting) {
    uint8_t ret_val;
    uint8_t err_state;
    I2C_start();
    I2C_send_byte(i2c_addr << 1 | WRITE); //read from device
    I2C_send_byte(reg_addr);
    I2C_send_byte(setting);
    I2C_stop();
    return SUCCESS;
}

static unsigned char I2C_read_byte(void) {
    I2C1CONbits.RCEN = 1; //setting this bit initiates a receive.  Hardware clears after the received has finished
    while (I2C1CONbits.RCEN) { //wait for byte to be received
        ;
    }
    return I2C1RCV; //return the value in receive buffer
}

/*SPI blocking functions*/

/**
 * @Function SPI_read_reg(uint8_t reg_addr)
 * @param address, hardware address in the encoder
 * @brief sends the next address to read
 * @returns data from previous SPI operation
 * @author ahunter
 */
static uint8_t SPI_read_reg(uint8_t reg_addr) {
    uint8_t data;
    reg_addr = reg_addr | (READ << 7);
    IMU_CS_LAT = 0;
    SPI1BUF = reg_addr;
    while (SPI1STATbits.SPIRBF == FALSE) {
        ;
    }
    data = SPI1BUF;
    reg_addr++;
    SPI1BUF = reg_addr; // We're not going to read this but need set something
    while (SPI1STATbits.SPIRBF == FALSE) {
        ;
    }
    data = SPI1BUF;
    IMU_CS_LAT = 1;
    return (data);
}

/**
 * @Function SPI_set_reg(uint8_t reg_addr)
 * @param address, hardware address in the encoder
 * @brief sends the next address to read
 * @returns data from previous SPI operation
 * @author ahunter
 */
static uint8_t SPI_set_reg(uint8_t reg_addr, uint8_t value) {
    uint8_t data;
    reg_addr = reg_addr | (WRITE << 7);
    IMU_CS_LAT = 0;
    SPI1BUF = reg_addr;
    while (SPI1STATbits.SPIRBF == FALSE) {
        ;
    }
    data = SPI1BUF;
    SPI1BUF = value; //send register setting
    while (SPI1STATbits.SPIRBF == FALSE) {
        ;
    }
    data = SPI1BUF;
    IMU_CS_LAT = 1;
    return data;
}

/**
 * @Function SPI_read_data()
 * @param none
 * @brief blocking function to read SPI data registers
 * @note deprecated with non-blocking interrupt version, left here for
 * documentation purposes
 * @returns none
 * @author ahunter
 */
static void SPI_read_data(void) {
    int i;
    int num_bytes = IMU_NUM_BYTES;
    uint8_t val = 0;
    uint8_t data_reg = AGB0_REG_ACCEL_XOUT_H;
    IMU_CS_LAT = 0;
    data_reg = data_reg | (READ << 7);
    for (i = 0; i < num_bytes; i++) {
        SPI1BUF = data_reg;
        while (SPI1STATbits.SPIRBF == FALSE) {
            ;
        }
        val = SPI1BUF;
        if (i > 0) {
            IMU_raw_data[i - 1] = val;
        }
        data_reg++;
    }
    IMU_CS_LAT = 1;
    IMU_process_data();
    IMU_data_ready = TRUE;
}

/**
 * @Function IMU_I2C_interrupt_handler()
 * @param none
 * @brief reads all IMU data registers once an I2C transaction is initiated by 
 * the user
 * @author ahunter
 */
static void __ISR(_I2C1_VECTOR, IPL2AUTO) IMU_I2C_interrupt_handler(void) {
    IMU_run_I2C_state_machine();
    LATAINV = 0x08; //toggle led
    IFS0bits.I2C1MIF = 0; //clear flag
}

/**
 * @Function IMU_SPI_interrupt_handler()
 * @param none
 * @brief reads all IMU data registers once an SPI transaction is initiated by 
 * the user
 * @author ahunter
 */
static void __ISR(_SPI_1_VECTOR, IPL5AUTO) IMU_SPI_interrupt_handler(void) {
    uint8_t data;
    data = SPI1BUF;
    IFS0bits.SPI1RXIF = 0; // clear interrupt flag
    IMU_run_SPI_state_machine(data);
}

/**
 * @Function IMU_read_data()
 * @param none
 * @brief I2C blocking function to read IMU data registers
 * @note deprecated with IMU_I2C_interrupt_handler, left here for documentation
 * purposes
 * @author ahunter
 */
static void IMU_read_data() {
    uint8_t i;
    uint8_t err_state;
    uint8_t num_bytes = IMU_NUM_BYTES; // TODO fix this for final version to reflect whole array
    LATAINV = 0x8;
    I2C_start();
    LATAINV = 0x8;
    I2C_send_byte(ICM_I2C_ADDR << 1 | WRITE);
    LATAINV = 0x8;
    I2C_send_byte(AGB0_REG_ACCEL_XOUT_H);
    LATAINV = 0x8;
    //    I2C_stop();
    I2C_restart();
    LATAINV = 0x8;
    I2C_send_byte(ICM_I2C_ADDR << 1 | READ); //read from device
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
            while (I2C1CONbits.ACKEN == 1) {
                ;
            }
        } else {
            I2C1CONbits.ACKDT = 1; // NACK the byte
            I2C1CONbits.ACKEN = 1;
            while (I2C1CONbits.ACKEN == 1) {
                ;
            }
        }
        LATAINV = 0x8;
    }
    I2C_stop();
    LATAINV = 0x8;
}

/**
 * @Function IMU_run_I2C_state_machine()
 * @brief state machine run from I2C interrupt to read IMU data registers
 * @note works, but I2C is less reliable and slower than SPI which is the
 * preferred implementation
 * @author ahunter
 */
static void IMU_run_I2C_state_machine(void) {
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
 * @Function IMU_run_SPI_state_machine(uint8_t byte_read)
 * @return none
 * @param byte_read, the byte read from SPI 1 buffer SPI1BUF
 * @brief state machine to reads IMU data registers over SPI
 * @note called with a single SPI read of the IMU
 * @author ahunter
 **/
static void IMU_run_SPI_state_machine(uint8_t byte_read) {
    static uint8_t reg_address = AGB0_REG_ACCEL_XOUT_H;
    static IMU_SPI_SM_states_t current_state = IMU_SPI_SEND_NEXT_REG;
    IMU_SM_states_t next_state = IMU_SPI_SEND_NEXT_REG;
    static uint8_t error = FALSE;
    static int8_t byte_index = -1;
    uint8_t max_index = IMU_NUM_BYTES - 1;
    switch (current_state) {
        case IMU_SPI_SEND_NEXT_REG:
            //store spi buffer in raw data struct
            if (byte_index >= 0) {
                IMU_raw_data[byte_index] = byte_read;
            }
            byte_index++; //increment byte count
            if (byte_index >= max_index) { // only change states after we have read all the data
                next_state = IMU_SPI_READ_LAST_REG;
            } else {
                next_state = current_state;
            }
            SPI1BUF = (++reg_address); //increment and send next register value
            break;
        case IMU_SPI_READ_LAST_REG:
            IMU_raw_data[byte_index] = byte_read; //store last data byte in raw data struct
            IMU_CS_LAT = 1; // deselect IMU
            IMU_data_ready = TRUE; // set data read flag
            byte_index = -1; //reset byte counter
            break;
        default:
            break;
    }
    current_state = next_state;
}

/**
 * @Function IMU_process_data(void)
 * @param none
 * @return SUCCESS or ERROR
 * @brief converts raw register data into module variable data
 * @note mag data has reversed byte order
 * @author ahunter
 * @modified  */
static uint8_t IMU_process_data(void) {
    int row;
    /*store data in module vectors*/
    /*data needs to be converted to short then cast as fixed point */
    acc_v_raw[0] = (accum) (int16_t) (IMU_raw_data[0] << 8 | IMU_raw_data[1]);
    acc_v_raw[1] = (accum) (int16_t) (IMU_raw_data[2] << 8 | IMU_raw_data[3]);
    acc_v_raw[2] = (accum) (int16_t) (IMU_raw_data[4] << 8 | IMU_raw_data[5]);
    gyro_v_raw[0] = (accum) (int16_t) (IMU_raw_data[6] << 8 | IMU_raw_data[7]);
    gyro_v_raw[1] = (accum) (int16_t) (IMU_raw_data[8] << 8 | IMU_raw_data[9]);
    gyro_v_raw[2] = (accum) (int16_t) (IMU_raw_data[10] << 8 | IMU_raw_data[11]);
    temp_raw = (accum) (IMU_raw_data[12] << 8 | IMU_raw_data[13]);
    //need to orient mag data to accel and gyros by rotating around x axis
    mag_v_raw[0] = (accum) (int16_t) (IMU_raw_data[16] << 8 | IMU_raw_data[15]);
    mag_v_raw[1] = (accum) (int16_t) ((IMU_raw_data[18] << 8 | IMU_raw_data[17])*-1);
    mag_v_raw[2] = (accum) (int16_t) ((IMU_raw_data[20] << 8 | IMU_raw_data[19])*-1);
    /*status 1 is high byte and status 2 is low byte*/
    /*status 2 indicates mag overflow only*/
    status = (IMU_raw_data[14] << 8 | IMU_raw_data[22] & 0x8);

    /*calibrate raw data and store in scaled vectors*/
    /*scale outputs */
    for (row = 0; row < MSZ; row++) {
        acc_v_scaled[row] = (acc_v_raw[row] >> ACCEL_DIV) * ACCEL_SCALE;
        mag_v_scaled[row] = (mag_v_raw[row] >> MAG_DIV) * MAG_SCALE;
        gyro_v_scaled[row] = (gyro_v_raw[row] >> GYRO_DIV) * GYRO_SCALE;
    }
    temp_scaled = (temp_raw - T_BIAS) / T_SENSE + T_OFFSET;
    temp_cald = temp_scaled;
    /* Calibrate sensors */
    /* apply calibration matrix to scaled values */
    m_v_mult(A_acc, acc_v_scaled, acc_v_cald);
    m_v_mult(A_mag, mag_v_scaled, mag_v_cald);
    m_v_mult(A_gyro, gyro_v_scaled, gyro_v_cald);
    /* apply offset calibration. scale mag and accelerometer into final units 
     * first.  These are provided by calibration routines to be scaled by the
     * Earth's mag and grav fields */
    /*now add offsets to calibrated values*/
    v_v_add(acc_v_cald, b_acc, acc_v_cald);
    v_v_add(mag_v_cald, b_mag, mag_v_cald);
    v_v_add(gyro_v_cald, b_gyro, gyro_v_cald);
    /*calculate the high resolution values*/
    v_scale(G2MSS,acc_v_cald);
    v_scale(MG2GAUSS,mag_v_cald);
    v_scale(DEG2MRAD, gyro_v_cald);
    return SUCCESS;
}

/**
 * @Function cal_to_fix(float A[3][3], b[3], accum * A_fx, accum* b_fx);
 * @param A input float point matrix, b input float point vector 
 * @brief sets the fixed point cal matrix to the float values
 * @return SUCCESS or ERROR
 */
static int8_t cal_to_fix(float A[3][3], float b[3], accum A_fx[3][3], accum b_fx[3]) {
    int row;
    int col;
    for (row = 0; row < 3; row++) {
        for (col = 0; col < 3; col++) {
            {
                A_fx[row][col] = (accum) A[row][col];
            }
            b_fx[col] = (accum) b[col];
        }
    }
    printf("\r\n-------------------------------------------------\r\n");
    for (row = 0; row < 3; row++) {
        for (col = 0; col < 3; col++) {
            printf("| ");
            {
                printf("%f\t", (float) A_fx[row][col]);
            }

        }
        printf("| %f\t|\r\n", (float) b_fx[row]);
    }
    printf("\r\n-------------------------------------------------\r\n");
    return SUCCESS;
}

/*-----------LINEAR ALGEBRA routines----------------------------------------*/

/**
 * @function m_v_mult()
 * Multiplies a matrix with a vector
 * @param m A matrix to be multiplied with a vector
 * @param v A vector to be multiplied with a matrix
 * @param v_out The product of the matrix and vector
 * @return SUCCESS or ERROR
 */
static uint8_t m_v_mult(accum m[MSZ][MSZ], accum v[MSZ], accum v_out[MSZ]) {
    int row;
    int col;

    for (row = 0; row < MSZ; row++) {
        v_out[row] = 0;
        for (col = 0; col < MSZ; col++) {
            v_out[row] += m[row][col] * v[col];
        }
    }
    return SUCCESS;
}

/**
 * @function v_v_add()
 * Add a vector value to a vector
 * @param v1 Vector to add to another vector
 * @param v2 Vector to have a vector added to it
 * @param v_out Vector as sum of two vectors
 */
static void v_v_add(accum v1[MSZ], accum v2[MSZ], accum v_out[MSZ]) {
    int row;

    for (row = 0; row < MSZ; row++) {
        v_out[row] = v1[row] + v2[row];
    }
}

/**
 * @function m_scale()
 * Scales matrix
 * @param s Scalar to scale matrix with
 * @param m Matrix to be scaled
 */
static void m_scale(accum s, accum m[MSZ][MSZ]) {
    int row;
    int col;

    for (row = 0; row < MSZ; row++) {
        for (col = 0; col < MSZ; col++) {
            m[row][col] *= s;
        }
    }
}

/**
 * @function v_scale()
 * Scales vector
 * @param s Scalar to scale matrix with
 * @param m Matrix to be scaled
 */
static void v_scale(accum s, accum v[MSZ]) {
    int row;

    for (row = 0; row < MSZ; row++) {
        v[row] *= s;
    }
}

#ifdef ICM_TESTING

int main(void) {
    int value = 0;
    int i;
    int row;
    int col;
    int IMU_err = 0;
    struct IMU_output IMU_data_raw = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    struct IMU_output IMU_data_scaled = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    /* test values: mag cal from Dorveaux*/
    accum A_mag_fl[3][3] = {
        {1.13203862, -0.00193396, 0.01600852},
        {-0.00199679, 1.17777219, 0.001252},
        {0.00742354, 0.01193132, 1.08211695}
    };

    accum b_mag_fl[3] = {-0.8959708, 0.08017001, 0.1556801};
    accum A_test[MSZ][MSZ];
    accum b_test[MSZ];

    Board_init();
    Serial_init();
    IMU_err = IMU_init(INTERFACE_MODE);
    //set LEDs for troubleshooting
    TRISAbits.TRISA4 = 0; //pin 72 Max32
    TRISAbits.TRISA3 = 0; //built-in LED, pin 13
    LATACLR = 0x18;
    printf("\r\nICM-20948 Test Harness %s, %s\r\n", __DATE__, __TIME__);
    if (INTERFACE_MODE == IMU_SPI_MODE) {
        printf("SPI interface enabled\r\n");
    } else {
        printf("I2C interface mode enabled\r\n");
    }
    if (IMU_err != SUCCESS) {
        printf("\r\nSensor failed init!\r\n");
        while (1);
    }

    IMU_set_mag_cal(A_mag_fl, b_mag_fl);
    printf(" Test set Calibration matrix and offset values\r\n");
    printf("---------------------------------------------------------\r\n");
    for (row = 0; row < MSZ; row++) {
        for (col = 0; col < MSZ; col++) {
            printf("%f\t", (float) A_mag[row][col]);
        }
        printf("%f\r\n", (float) b_mag[row]);
    }
    printf("---------------------------------------------------------\r\n");
    IMU_get_acc_cal(A_test, b_test);
    printf("Test get Calibration matrix and offset values\r\n");
    printf("---------------------------------------------------------\r\n");
    for (row = 0; row < MSZ; row++) {
        for (col = 0; col < MSZ; col++) {
            printf("%f\t", (float) A_test[row][col]);
        }
        printf("%f\r\n", (float) b_test[row]);
    }
    printf("---------------------------------------------------------\r\n");

    IMU_get_mag_cal(A_test, b_test);
    printf("Test get Calibration matrix and offset values\r\n");
    printf("---------------------------------------------------------\r\n");
    for (row = 0; row < MSZ; row++) {
        for (col = 0; col < MSZ; col++) {
            printf("%f\t", (float) A_test[row][col]);
        }
        printf("%f\r\n", (float) b_test[row]);
    }
    printf("---------------------------------------------------------\r\n");

    while (1) {
        IMU_start_data_acq();
        if (IMU_is_data_ready() == TRUE) {
            IMU_get_raw_data(&IMU_data_raw);
            //            printf("%0.1f\t%0.1f\t%0.1f\t%0.1f\t%0.1f\t%0.1f\t%0.1f\t%0.1f\t%0.1f\t%0.1f\t%x\r\n",
            //                    (float) IMU_data_raw.acc.x, (float) IMU_data_raw.acc.y, (float) IMU_data_raw.acc.z,
            //                    (float) IMU_data_raw.gyro.x, (float) IMU_data_raw.gyro.y, (float) IMU_data_raw.gyro.z,
            //                    (float) IMU_data_raw.mag.x, (float) IMU_data_raw.mag.y, (float) IMU_data_raw.mag.z,
            //                    (float) IMU_data_raw.temp, IMU_data_raw.mag_status);
            IMU_get_scaled_data(&IMU_data_scaled);
            printf("%0.1f\t%0.1f\t%0.1f\t%0.1f\t%0.1f\t%0.1f\t%0.1f\t%0.1f\t%0.1f\t%0.1f\t%x\r\n",
                    (float) IMU_data_scaled.acc.x, (float) IMU_data_scaled.acc.y, (float) IMU_data_scaled.acc.z,
                    (float) IMU_data_scaled.gyro.x, (float) IMU_data_scaled.gyro.y, (float) IMU_data_scaled.gyro.z,
                    (float) (IMU_data_scaled.mag.x), (float) (IMU_data_scaled.mag.y), (float) (IMU_data_scaled.mag.z),
                    (float) IMU_data_scaled.temp, IMU_data_scaled.mag_status);
        }
        /*delay to not overwhelm the serial port*/
        for (i = 0; i < 500000; i++) {
            ;
        }
        LATACLR = 0x18;
    }
}
#endif //ICM_TESTING


