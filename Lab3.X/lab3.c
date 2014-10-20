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

// ******************************************************************************************* //

// Function Definitions

// ******************************************************************************************* //

int main(void)
{
    // Configure AD1PCFG register for configuring input pin as analog
    AD1PCFGbits.PCFG0 = 1;

    // Configure TRIS register bits for Input
    TRISAbits.TRISA0 = 1;

    // Configure CNPU register bits to enable internal pullup resistor for input.
    CNPU1bits.CN2PUE = 1; //IO5 Input
    CNEN1bits.CN2IE = 1;

    // Input Change Notification
    IFS1bits.CNIF = 0;
    IEC1bits.CNIE = 1;

    while(1)
	{

               

        }
    
	return 0;
}