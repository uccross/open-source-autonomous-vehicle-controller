/* 
 * File:   AS5047D.c
 * Author: Aaron Hunter
 * Brief: 
 * Created on Dec 3, 2020 3:54 pm
 * Modified on
 */

/*******************************************************************************
 * #INCLUDES                                                                   *
 ******************************************************************************/

#include "AS5047D.h" // The header file for this source file. 
#include "SerialM32.h"
#include "Board.h"
#include <stdio.h>
#include <sys/attribs.h>  //for ISR definitions
#include <proc/p32mx795f512l.h>

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/
#define ENC_SPI_FREQ 5000000ul //5MHz clock rate
#define READ 1
#define WRITE 0
/*Set up the chip select digital IOs here*/
#define CS1_TRIS TRISEbits.TRISE1 //chip select for LHS rotary encoder
#define CS1_LAT LATEbits.LATE1
#define CS2_TRIS TRISEbits.TRISE2 //chip select for RHS rotary encoder
#define CS2_LAT LATEbits.LATE2 
#define CS3_TRIS TRISEbits.TRISE3 //chip select for RHS rotary encoder
#define CS3_LAT LATEbits.LATE3 

/*Used for debugging the interrupt can be removed eventually*/
#define LED_OUT_TRIS TRISDbits.TRISD3
#define LED_OUT_LAT LATDbits.LATD3
/*important constants*/
#define MAX_VELOCITY 6000 //extrapolated to 100 Hz
#define TWO_PI 16384
/*AS5047D register definitions*/
#define NOP 0x0000
#define ERRFL 0x0001
#define SETTINGS_REG 0x0018
#define DIAAGC 0X3FFC
#define MAG 0X3FFD
#define ANGLEUNC 0X3FFE
#define ANGLE 0x3FFF

/*******************************************************************************
 * PRIVATE TYPEDEFS                                                            *
 ******************************************************************************/

typedef enum {
    ENC_START,
    ENC_NEXT,
    ENC_LAST
} ENC_state_t;

static encoder_t encoder_data[NUM_ENCODERS]; //array of encoder structs
static int8_t data_ready = FALSE; //set in SPI SM at completion of a read cycle

/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                 *
 ******************************************************************************/
/**
 * @Function  int checkParity(uint16_t in)
 * @param int in, the value for which parity is calculated
 * @returns p, parity, either zero (even) or one (odd)
 * @author ahunter
 */
static int check_parity(uint16_t in);

/**
 * @Function insertParityBit(uint_16t in)
 * @param int in, the data to send to encoder
 * @brief calculates parity and inserts the appropriate bit into MSB
 * @returns in, the value to send to encoder
 * @author ahunter
 */
static uint16_t insert_parity_bit(uint16_t in);

/**
 * @Function delay(int cycles)
 * @param cycles, number of cycles to delay
 * @brief simple delay loop
 * @note ~500nsec for one delay, then +12.5 nsec for every increment higher
 * @author ahunter
 */
static void delay(int cycles);

/**
 * @Function readRegister(uint16_t address)
 * @param reg, hardware address in the encoder
 * @brief sends address
 * @returns in, the value to send to encoder
 * @author ahunter
 */
static uint16_t read_register(uint16_t address);

/*blocking function used for init only*/
/**
 * @Function writeRegister(uint16_t address, uint16_t setting)
 * @param reg, hardware address in the encoder
 * @param setting, setting data to write
 * @returns setting read from address after write
 * @author ahunter
 */
static int16_t write_register(uint16_t address, uint16_t setting);

/**
 * @Function void __ISR(_SPI_2_VECTOR, IPL5AUTO) SPI2_interrupt_handler(void);
 * @brief interrupt driven measurements of all AS5047D devices on system.  ISR
 * sets a simple state machine to run each reception of the SPI2RXIF (receive
 * interrupt)
 * @author Aaron Hunter
 */
void __ISR(_SPI_2_VECTOR, IPL5AUTO) SPI2_interrupt_handler(void);

/**
 * @Function void run_encoder_SM(data)
 * @param raw uint16_t data read from SPI2BUF
 * @brief simple FSM to update encoder data struct with multiple encoders
 * @author Aaron Hunter
 */
static void run_encoder_SM(uint16_t data);

/*******************************************************************************
 * PUBLIC FUNCTION IMPLEMENTATIONS                                             *
 ******************************************************************************/

/**
 * @Function Encoder_Init(void)
 * @param freq, frequency of SPI rate
 * @return SUCCESS or ERROR
 * @brief initializes hardware in appropriate mode along with the needed interrupts */
uint8_t Encoder_init(void) {
    uint8_t index;
    uint32_t pb_clk;
    uint16_t setting;
    uint16_t return_value;
    pb_clk = Board_get_PB_clock();
    /* SPI settings */
    __builtin_disable_interrupts();
    SPI2CON = 0; // disable SPI system
    SPI2BUF; // clear receive buffer*/
    SPI2STATbits.SPIROV = 0; // clear any overflow condition 
    SPI2CONbits.MSTEN = 1; // set as master
    SPI2CONbits.SSEN = 0; // manually drive CS/SS 
    SPI2CONbits.MODE32 = 0;
    SPI2CONbits.MODE16 = 1; // set to 16 bit mode
    SPI2BRG = pb_clk / (2 * ENC_SPI_FREQ) - 1; // 
    SPI2CONbits.SMP = 1; /* set sample at end of data*/
    /*NOTE: mode 1 SPI has CKE = 0, CKP = 0*/
    SPI2CONbits.CKP = 0; /*set clock phase to idle low*/
    SPI2CONbits.CKE = 0; /* set to read on falling edge (active --> idle)*/
    /*initialize chip select pins*/
    CS1_TRIS = 0; /* set up CS1 for CS output */
    CS1_LAT = 1; /* deselect encoder 1 (left wheel)*/
    CS2_TRIS = 0; /* set up CS2 for CS output */
    CS2_LAT = 1; /* deselect encoder 2 (right wheel)*/
    CS3_TRIS = 0; /* set up CS3 for CS output */
    CS3_LAT = 1; /* deselect encoder 3 (steering servo)*
    /*LED indicator of ISR  to be removed later*/
    //    LED_OUT_TRIS = 0;
    //    LED_OUT_LAT = 0;
    /*set up SPI2 RX interrupt*/
    IEC1bits.SPI2RXIE = 1; //enable interrupt
    IFS1bits.SPI2RXIF = 0; // clear interrupt flag
    IPC7bits.SPI2IP = 5; //interrupt priority 5
    IPC7bits.SPI2IS = 1; //subpriority 1
    SPI2CONbits.ON = 1; /* enable SPI system*/
    /*Initialize encoder.*/
    setting = 0; //ABI PWM off
    return_value = write_register(SETTINGS_REG, setting);
#ifdef ENC_TESTING
    printf("\r\nEncoder set to:0x%x\r\n", return_value);
#endif
    __builtin_enable_interrupts();
    for (index = 0; index < NUM_ENCODERS; index++) {
        Encoder_init_encoder_data(&encoder_data[index]);
    }
    return SUCCESS;
}

/**
 * @Function Encoder_start_data_acq(void);
 * @return none
 * @param none
 * @brief this function starts the SPI data read
 * @author Aaron Hunter
 **/
void Encoder_start_data_acq(void) {
    CS1_LAT = 0; //select encoder number 1
    SPI2BUF = ANGLE; //read angle register
}

/**
 * @Function int16_t Encoder_get_angle(encoder_enum_t encoder_num);
 * @param encoder number
 * @return 14-bit number representing the raw encoder angle (0-16384)
 * @author Aaron Hunter */
int16_t Encoder_get_angle(encoder_enum_t encoder_num) {
    return encoder_data[encoder_num].next_theta;
}

/**
 * @Function int16_t RotaryEncoder_get_velocity(encoder_enum_t encoder_num);
 * @param encoder number 
 * @return angular velocity measurement
 */
int16_t Encoder_get_velocity(encoder_enum_t encoder_num) {
    return encoder_data[encoder_num].omega;
}

/**
 * @Function void init_encoder_data(void);
 * @brief initializes encoder data struct(s)
 * @author Aaron Hunter
 */
void Encoder_init_encoder_data(encoder_ptr_t enc) {
    int i;
    enc->last_theta = 0;
    enc->next_theta = 0;
    enc->omega = 0;
}

/**
 * @Function Encoder_is_data_ready(void)
 * @param none
 * @brief returns TRUE if there is unread (from Encoder_get_data()) data available
 * @return TRUE or FALSE
 * @author Aaron Hunter
 */
int8_t Encoder_is_data_ready(void) {
    return (data_ready);
}

/**
 * @Function Encoder_get_data(encoder_t * data)
 * @param encoder_t data--to receive private encoder data 
 * @brief copies internal encoder data to data
 * @author Aaron Hunter
 */
int8_t Encoder_get_data(encoder_t * data) {
    uint8_t i;
    for (i = 0; i < NUM_ENCODERS; i++) {
        data[i].last_theta = encoder_data[i].last_theta;
        data[i].next_theta = encoder_data[i].next_theta;
        data[i].omega = encoder_data[i].omega;
    }
    data_ready = FALSE;
    return SUCCESS;
}

/*******************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATIONS                                            *
 ******************************************************************************/

/**
 * @Function int checkParity(uint16_t in)
 * @param uint16_t in, the value for which parity is calculated
 * @returns p, parity, either zero (even) or one (odd)
 * @author ahunter
 */
static int check_parity(uint16_t in) {
    int p = 0; //our parity value
    int i;
    for (i = 1; i < 0x8000;) {
        if (in & i) { //if the bit is odd 
            p = p + 1; //increment parity value
        }
        i = i << 1; //move to next bit 
    }
    p = p % 2; //calculate whether sum is even or odd and return 0 or 1
    return p;
}

static uint16_t insert_parity_bit(uint16_t in) {
    int p;
    p = check_parity(in);
    in = in | (p << 15);
    return in;
}

/**
 * @Function delay(int cycles)
 * @param cycles, number of cycles to delay
 * @brief simple delay loop
 * @note ~500nsec for one delay, then +12.5 nsec for every increment higher
 * @author ahunter
 */
static void delay(int cycles) {
    int i;
    for (i = 0; i < cycles; i++) {
        ;
    }
}

/**
 * @Function read_register(uint16_t address)
 * @param address, hardware address in the encoder
 * @brief sends the next address to read
 * @returns data from previous SPI operation
 * @author ahunter
 */
static uint16_t read_register(uint16_t address) {
    uint16_t data;
    address = address | (READ << 14);
    address = insert_parity_bit(address);

    CS1_LAT = 0;
    SPI2BUF = address;
    while (SPI2STATbits.SPIRBF == FALSE);
    data = SPI2BUF;
    CS1_LAT = 1;
    delay(1);

    CS1_LAT = 0;
    SPI2BUF = 0xC000; //NOP
    while (SPI2STATbits.SPIRBF == FALSE);
    data = SPI2BUF;
    CS1_LAT = 1;
    delay(1);

    return (data);
}

/**
 * @Function write_register(uint16_t address, uint16_t value)
 * @param address, hardware address in the encoder
 * @param value: data to write
 * @brief performs single SPI write
 * @returns value after the write is performed
 * @author ahunter
 */
static int16_t write_register(uint16_t address, uint16_t value) {
    uint16_t data;
    uint16_t address_write;
    uint16_t address_read;
    uint16_t error_val;

    address_write = insert_parity_bit(address); //for write
    address_read = insert_parity_bit((READ << 14) | address);
    value = insert_parity_bit(value);

    /*read ERROR register first to clear any old conditions*/
    error_val = read_register(ERRFL);

    CS1_LAT = 0;
    SPI2BUF = address_write; //register address to be written to
    while (SPI2STATbits.SPIRBF == FALSE);
    data = SPI2BUF;
    CS1_LAT = 1;
    delay(1); //need 350 ns between SPI commands

    CS1_LAT = 0;
    SPI2BUF = value; //value to store
    while (SPI2STATbits.SPIRBF == FALSE);
    data = SPI2BUF;
    CS1_LAT = 1;
    delay(1);

    CS1_LAT = 0;
    SPI2BUF = address_read; //address to be read
    while (SPI2STATbits.SPIRBF == FALSE);
    data = SPI2BUF;
    CS1_LAT = 1;
    delay(1);

    CS1_LAT = 0;
    SPI2BUF = 0xC000; //NOP
    while (SPI2STATbits.SPIRBF == FALSE);
    data = SPI2BUF; //settings data
    CS1_LAT = 1;
    delay(1);

    /*check for errors*/
#ifdef ENC_TESTING
    error_val = read_register(ERRFL);
    error_val = error_val & 0x8;

    if (data & 0xC000 != value & 0xC000) {
        printf("Value written: %d, value read: %d", value, data);
    }
    if (error_val != 0) {
        printf("Error condition returned: %d", error_val);
    }
#endif
    return (data & 0xC000);
}

/**
 * @Function void __ISR(_SPI_2_VECTOR, IPL5AUTO) SPI2_interrupt_handler(void)
 * @brief Interrupt driven data acquisition from encoders
 * @note calls the state machine to perform the reads.  The initial interrupt 
 * is created by selecting the first encoder and reading the angle register
 * @author ahunter
 */
void __ISR(_SPI_2_VECTOR, IPL5AUTO) SPI2_interrupt_handler(void) {
    uint16_t data;
    data = SPI2BUF; //read the data 
    IFS1bits.SPI2RXIF = 0; // clear interrupt flag
    run_encoder_SM(data);

}

/**
 * @Function void run_encoder_SM(void)
 * @brief simple FSM to update encoder data struct with multiple encoders
 * @author Aaron Hunter
 */
static void run_encoder_SM(uint16_t data_short) {
    static ENC_state_t current_state = ENC_START;
    static ENC_state_t next_state = ENC_START;
    static int16_t last_theta;
    static int16_t next_theta;
    int32_t w; //temp variable for instantaneous velocity

    switch (current_state) {
        case ENC_START:
            encoder_data[LEFT_MOTOR].last_theta = encoder_data[LEFT_MOTOR].next_theta;
            /*read LEFT_MOTOR encoder, subtract from 2pi to correct for orientation */
            encoder_data[LEFT_MOTOR].next_theta = TWO_PI - (0x3FFF & data_short); //mask top bits
            /*update encoder 1 data struct*/
            w = encoder_data[LEFT_MOTOR].next_theta - encoder_data[LEFT_MOTOR].last_theta;
            if (w < -MAX_VELOCITY) {
                w = w + TWO_PI;
            }
            if (w > MAX_VELOCITY) {
                w = w - TWO_PI;
            }
            encoder_data[LEFT_MOTOR].omega = (int16_t) w;
            CS1_LAT = 1; /*deselect encoder 1*/
            CS2_LAT = 0; /*select encoder 2*/
            /*write angle register address to encoder 2*/
            SPI2BUF = ANGLE;
            next_state = ENC_NEXT;
            break;
        case(ENC_NEXT):
            encoder_data[RIGHT_MOTOR].last_theta = encoder_data[RIGHT_MOTOR].next_theta;
            /*read the RIGHT_MOTOR angle*/
            encoder_data[RIGHT_MOTOR].next_theta = 0x3FFF & data_short; //mask top bits
            /*update RIGHT_MOTOR data struct*/
            w = encoder_data[RIGHT_MOTOR].next_theta - encoder_data[RIGHT_MOTOR].last_theta;
            if (w < -MAX_VELOCITY) {
                w = w + TWO_PI;
            }
            if (w > MAX_VELOCITY) {
                w = w - TWO_PI;
            }
            encoder_data[RIGHT_MOTOR].omega = (int16_t) w;
            CS2_LAT = 1; /*deselect encoder 2*/
            CS3_LAT = 0; /*select encoder 3*/
            /*write angle register address to encoder 3*/
            SPI2BUF = ANGLE;
            next_state = ENC_LAST;
            break;
        case ENC_LAST:
            encoder_data[HEADING].last_theta = encoder_data[HEADING].next_theta;
            /*read the HEADING angle*/
            encoder_data[HEADING].next_theta = 0x3FFF & data_short; //mask top bits
            /*update HEADING data struct*/
            w = encoder_data[HEADING].next_theta - encoder_data[HEADING].last_theta;
            if (w < -MAX_VELOCITY) {
                w = w + TWO_PI;
            }
            if (w > MAX_VELOCITY) {
                w = w - TWO_PI;
            }
            encoder_data[HEADING].omega = (int16_t) w;
            CS3_LAT = 1; /*deselect encoder 3*/
            data_ready = TRUE;
            next_state = ENC_START;
            break;
        default:
            next_state = ENC_START;
            break;
    }
    current_state = next_state;
}



#ifdef ENC_TESTING

int main(void) {
    uint8_t index;
    int16_t angle_left;
    int16_t angle_right;
    int16_t vel_left;
    int16_t vel_right;
    int16_t phi; // steering angle
    int16_t dphi_dt; //angular velocity of steering servo
    encoder_t enc_data[NUM_ENCODERS];


    Board_init();
    Serial_init();
    printf("\r\nAS5047D Encoder Test Harness %s, %s\r\n", __DATE__, __TIME__);
    Encoder_init();

    for (index = 0; index < NUM_ENCODERS; index++) {
        Encoder_init_encoder_data(&enc_data[index]);
    }

    while (1) {
        Encoder_start_data_acq();
        if (Encoder_is_data_ready() == TRUE) {
            Encoder_get_data(enc_data);
            printf("L: %6d, %6d; R: %6d, %6d, S: %6d, %6d\r",
                    enc_data[LEFT_MOTOR].next_theta,
                    enc_data[LEFT_MOTOR].omega,
                    enc_data[RIGHT_MOTOR].next_theta,
                    enc_data[RIGHT_MOTOR].omega,
                    enc_data[HEADING].next_theta,
                    enc_data[HEADING].omega);
        }
        delay(150000);
    }
}
#endif //ENC_TESTING



