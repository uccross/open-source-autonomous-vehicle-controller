/* 
 * File:   Battery.c
 * Author: Aaron Hunter
 * Brief: 
 * Created on May 2, 2022, 10:36 am
 * Modified on <month> <day>, <year>, <hour> <pm/am>
 */
/*******************************************************************************
 * #INCLUDES                                                                   *
 ******************************************************************************/

#include <proc/p32mx795f512l.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/attribs.h>  //for ISR definitions
#include "xc.h"
#include "Board.h"
#include "SerialM32.h"
#include "Battery.h" // The header file for this source file. 

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/
#define BATTLOW 268
#define AN0 0
#define AN1 1<<0
#define AN2 1<<1
#define AN3 1<<2
#define AN4 1<<3
#define V_SCALE 12 //12.151 measured
/*******************************************************************************
 * MODULE VARIABLES                                                            *
 ******************************************************************************/
static uint16_t battery_voltage = 0;
static uint16_t raw_AD_val = 0;
static int8_t new_data_ready = FALSE;

/*******************************************************************************
 * PRIVATE TYPEDEFS                                                            *
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                 *
 ******************************************************************************/

/*******************************************************************************
 * PUBLIC FUNCTION IMPLEMENTATIONS                                             *
 ******************************************************************************/

/**
 * @Function Battery_init(void);
 * @return ERROR or SUCCESS
 * @brief Initialize the AD system for battery operation
 * @note 
 * @author Aaron Hunter,
 * @modified <Your Name>, <year>.<month>.<day> <hour> <pm/am> */
int8_t Battery_init() {
    // Disable interrupts
    IEC1bits.AD1IE = 0;
    //To configure the ADC module, perform the following steps:
    //1. Configure the analog port pins in AD1PCFG<15:0> (see 17.4.1).
    AD1PCFGbits.PCFG0 = 0; //A0 as analog input
    //2. Select the analog inputs to the ADC multiplexers in AD1CHS<32:0> (see 17.4.2).
   // AD1CHSbits.CH0SA = 0b0000;
    // Not needed, we do scan mode
    AD1CSSLbits.CSSL0 = 1; //select AN0
    //3. Select the format of the ADC result using FORM<2:0> (AD1CON1<10:8>) (see 17.4.3).
    AD1CON1bits.FORM = 0b000; //unsigned 16 bit integer
    //4. Select the sample clock source using SSRC<2:0> (AD1CON1<7:5>) (see 17.4.4).
    AD1CON1bits.SSRC = 0b111; // internal counter handles timing of sampling an conversion
    //5. Select the voltage reference source using VCFG<2:0> (AD1CON2<15:13>) (see 17.4.7).
    AD1CON2bits.VCFG = 0b000; //AVdd and AVss
    //6. Select the Scan mode using CSCNA (AD1CON2<10>) (see 17.4.8).
    AD1CON2bits.CSCNA = 1; //scan inputs
    //7. Set the number of conversions per interrupt SMP<3:0> (AD1CON2<5:2>), if interrupts are
    //to be used (see 17.4.9).
    AD1CON2bits.SMPI = 0b000; //only one channel
    //8. Set Buffer Fill mode using BUFM (AD1CON2<1>) (see 17.4.10).
    AD1CON2bits.BUFM = 0; // Buffer configured as one 16-word buffer ADC1BUF(15...0.)
    //9. Select the MUX to be connected to the ADC in ALTS AD1CON2<0> (see 17.4.11).
    AD1CON2bits.ALTS = 0; // use Mux A settings only
    //10. Select the ADC clock source using ADRC (AD1CON3<15>) (see 17.4.12).
    AD1CON3bits.ADRC = 0; //use peripheral clock as the timer
    //11. Select the sample time using SAMC<4:0> (AD1CON3<12:8>), if auto-convert is to be
    //used (see 17-2).
    AD1CON3bits.SAMC = 0b11111; // 31 time periods T_AD between samples --longest
    //12. Select the ADC clock prescaler using ADCS<7:0> (AD1CON3<7:0>) (see 17.4.12).
    AD1CON3bits.ADCS = 0x32; //can go to 512 
    //13. Turn the ADC module on using AD1CON1<15> (see 17.4.14).
    AD1CON1bits.ON = 1;
    //14. To configure ADC interrupt (if required):
    //a) Clear the AD1IF bit (IFS1<1>) (see 17.7).
    IFS1bits.AD1IF = 0;
    //b) Select ADC interrupt priority AD1IP<2:0> (IPC<28:26>) and subpriority AD1IS<1:0>
    //(IPC<24:24>) if interrupts are to be used (see 17.7).
    IPC6bits.AD1IP = 2;
    IPC6bits.AD1IS = 2;
    IEC1bits.AD1IE = 1; //enable interrupts
    //15. Start the conversion sequence by initiating sampling (see 17.4.15).
   
    AD1CON1bits.ASAM = 1;


    return SUCCESS;
}

/*******************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATIONS                                            *
 ******************************************************************************/

/**
 * @Function ADC_Int_Handler(void);
 * @brief Interrupt handler for the A/D. Reads the pin A0 battery voltage
 * @note 
 * @author Aaron Hunter
 * @modified <Your Name>, <year>.<month>.<day> <hour> <pm/am> */
void __ISR(_ADC_VECTOR, IPL2AUTO) ADC_Int_Handler(void) {
    raw_AD_val = ADC1BUF0;
    new_data_ready = TRUE;
    IFS1bits.AD1IF = 0;
}

#ifdef BATTERY_TEST_HARNESS
#define DELAY(x)    {int wait; for (wait = 0; wait <= x; wait++) {asm("nop");}}
#define A_BIT       18300
#define A_LOT       183000

int main(void) {
    Board_init();
    Serial_init();
    printf("Battery test harness %s, %s \r\n", __DATE__, __TIME__);
    Battery_init();
    while (1) {
        DELAY(A_BIT);
        if (new_data_ready == TRUE) {
            new_data_ready = FALSE;
            printf("%d \r\n", raw_AD_val);
        }
    }
}

#endif
