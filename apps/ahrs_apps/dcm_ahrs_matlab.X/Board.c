/* 
 * File:   Board.c
 * Author: Aaron Hunter
 * Brief: Sets up the microprocessor with the correct timing and vectored int
 * Created on 11/09/2020
 * Modified on 11/10/2020
 */


/*******************************************************************************
 * CPU Configuration                                                                    *
 ******************************************************************************/

// PIC32MX795F512L Configuration Bit Settings

// 'C' source line config statements

// DEVCFG3
#pragma config USERID = 0xFFFF          // Enter Hexadecimal value (Enter Hexadecimal value)
#pragma config FSRSSEL = PRIORITY_7     // SRS Select (SRS Priority 7)
#pragma config FMIIEN = ON              // Ethernet RMII/MII Enable (MII Enabled)
#pragma config FETHIO = ON              // Ethernet I/O Pin Select (Default Ethernet I/O)
#pragma config FCANIO = ON              // CAN I/O Pin Select (Default CAN I/O)
#pragma config FUSBIDIO = ON            // USB USID Selection (Controlled by the USB Module)
#pragma config FVBUSONIO = ON           // USB VBUS ON Selection (Controlled by USB Module)

// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config UPLLIDIV = DIV_12        // USB PLL Input Divider (12x Divider)
#pragma config UPLLEN = OFF             // USB PLL Enable (Disabled and Bypassed)
#pragma config FPLLODIV = DIV_1         // System PLL Output Clock Divider (PLL Divide by 1)

// DEVCFG1
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = ON                // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = XT             // Primary Oscillator Configuration (XT osc mode)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_1           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/1)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS4096           // Watchdog Timer Postscaler (1:4096)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))

// DEVCFG0
#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is disabled)
#pragma config ICESEL = ICS_PGx2        // ICE/ICD Comm Channel Select (ICE EMUC2/EMUD2 pins shared with PGC2/PGD2)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

/*******************************************************************************
 * #INCLUDES                                                                   *
 ******************************************************************************/
#include <xc.h>
#include <stdio.h>
#include <sys/attribs.h>
#include "Board.h" // The header file for this source file. 


/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/
#define SYS_FREQ 80000000ul //80MHz
#define PB_CLOCK_FREQ SYS_FREQ
#define BAUD_RATE 115200    // Baudrate for RS232

/*******************************************************************************
 * PRIVATE TYPEDEFS                                                            *
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                 *
 ******************************************************************************/

/*******************************************************************************
 * PUBLIC FUNCTION IMPLEMENTATIONS                                             *
 ******************************************************************************/
// Perform startup routines:
//  Make uc32_LED1 and uc32_LED2 pins outputs (uc32_USER is by default an input)
//  Initialize the serial port - UART1 (no interrupt) 
//  Enable interrupts

void Board_init() {
    __builtin_disable_interrupts();

    // enable the cache 
    // This command sets the CP0 CONFIG register
    // the lower 4 bits can be either 0b0011 (0x3) or 0b0010 (0x2)
    // to indicate that kseg0 is cacheable (0x3) or uncacheable (0x2)
    // see Chapter 2 "CPU for Devices with M4K Core" of the PIC32 reference manual
    // most of the other bits have prescribed values
    // microchip does not provide a _CP0_SET_CONFIG macro, so we directly use
    // the compiler built-in command _mtc0
    // to disable cache, use 0xa4210582 
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // set the prefectch cache wait state to 2, as per the
    // electrical characteristics data sheet
    CHECONbits.PFMWS = 0x2;

    //enable prefetch for cacheable and noncacheable memory
    CHECONbits.PREFEN = 0x3;

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get B10, B11, B12 and B13 back
    DDPCONbits.JTAGEN = 0;

    __builtin_enable_interrupts();
}

/**
 * @Function Board_get_sys_clock();
 * @brief returns the peripheral clock speed
 * @author Aaron Hunter
 */
uint32_t Board_get_sys_clock(void) {
    return ((uint32_t) SYS_FREQ);
}

/**
 * @Function Board_get_PB_clock();
 * @brief returns the peripheral clock speed
 * @author Aaron Hunter
 */
uint32_t Board_get_PB_clock(void) {
    return ((uint32_t) PB_CLOCK_FREQ);
}

/*******************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATIONS                                            *
 ******************************************************************************/

#ifdef BOARD_TESTING

int main(void) {
    Board_init();

    //    int curPow2 = 12;
    //    int i;
    TRISAbits.TRISA4 = 0; //pin 72 Max32
    LATAbits.LATA4 = 0;
    TRISAbits.TRISA3 = 0; //built-in LED, pin 13
    LATAbits.LATA3 = 1;
//    printf("Board testing\r\n");
//    printf("board freq %d \r\n", Board_get_sys_clock());
//    printf("peripheral clock freq %d", Board_get_PB_clock());
    //will need a scope to test this module, the led should blink at the maximum rate
    while (1) {
        LATAINV = 0x10;  //this is faster by a factor of two than below:
//        LATAbits.LATA4 ^= 1;
    }
}

#endif



