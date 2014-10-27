// ******************************************************************************************* //
// Greg DePaul
// Chris Carry
// Vincent Ippolito
// ECE 372
// Lab3

// ******************************************************************************************* //
// Include file for PIC24FJ64GA002 microcontroller. This include file defines
// MACROS for special function registers (SFR) and control bits within those
// registers.

#include "p24fj64ga002.h"
#include <stdio.h>
#include "lcd.h"

// ******************************************************************************************* //
// Configuration bits for CONFIG1 settings.
//
// Make sure "Configuration Bits set in code." option is checked in MPLAB.
//
// These settings are appropriate for debugging the PIC microcontroller. If you need to
// program the PIC for standalone operation, change the COE_ON option to COE_OFF.

_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF &
          BKBUG_ON & COE_OFF & ICS_PGx1 &
          FWDTEN_OFF & WINDIS_OFF & FWPSA_PR128 & WDTPS_PS32768 )

// ******************************************************************************************* //
// Configuration bits for CONFIG2 settings.
// Make sure "Configuration Bits set in code." option is checked in MPLAB.

_CONFIG2( IESO_OFF & SOSCSEL_SOSC & WUTSEL_LEG & FNOSC_PRIPLL & FCKSM_CSDCMD & OSCIOFNC_OFF &
          IOL1WAY_OFF & I2C1SEL_PRI & POSCMOD_XT )

// ******************************************************************************************* //

// Global Variables
volatile char updateA2D;

// ******************************************************************************************* //

// Function Definitions

// ******************************************************************************************* //

int main(void)
{
    // Initialize Variables
    updateA2D = 1;
    int ADC_value;
    char value[8];
    double AD_value;

    //Initialize the LCD
    LCDInitialize();

    // Configure AD1PCFG register for configuring input pin as analog
    AD1PCFG &= 0xFFFE;
    AD1CON2 = 0;
    AD1CON3 = 0x0101;
    AD1CON1 = 0x20E4;
    AD1CHS = 0;
    AD1CSSL = 0;

    IFS0bits.AD1IF = 0; // Clear A/D conversion interrupt.
    AD1CON1bits.ADON = 1; // Turn on A/D

    while(1)
    {

        while (IFS0bits.AD1IF ==0);     // AD1CON1bits.DONE can be checked instead
	IFS0bits.AD1IF = 0;
	ADC_value = ADC1BUF0;

	sprintf(value, "%6d", ADC_value);
	LCDMoveCursor(0,0);
        LCDPrintString(value);

	AD_value = (ADC_value * 3.3)/1024;
	sprintf(value, "%6.2f", AD_value);
	LCDMoveCursor(1,0);
        LCDPrintString(value);

    }

    return 0;
}