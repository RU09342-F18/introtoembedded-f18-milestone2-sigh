/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 * 
 *                       MSP430 CODE EXAMPLE DISCLAIMER
 *
 * MSP430 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see www.ti.com/grace for a GUI- and www.ti.com/msp430ware
 * for an API functional library-approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
//******************************************************************************
//   MSP430F552x Demo - ADC12, Sample A0, Set P1.0 if A0 > 0.5*AVcc
//
//   Description: A single sample is made on A0 with reference to AVcc.
//   Software sets ADC12SC to start sample and conversion - ADC12SC
//   automatically cleared at EOC. ADC12 internal oscillator times sample (16x)
//   and conversion. In Mainloop MSP430 waits in LPM0 to save power until ADC12
//   conversion complete, ADC12_ISR will force exit from LPM0 in Mainloop on
//   reti. If A0 > 0.5*AVcc, P1.0 set, else reset.
//
//                MSP430F552x
//             -----------------
//         /|\|                 |
//          | |                 |
//          --|RST              |
//            |                 |
//     Vin -->|P6.0/CB0/A0  P1.0|--> LED
//
//   Bhargavi Nisarga
//   Texas Instruments Inc.
//   April 2009
//   Built with CCSv4 and IAR Embedded Workbench Version: 4.21
//******************************************************************************

/*
********************************************************************************
Authors: 
*/

#include <msp430.h>
volatile int temp = 0;
int TensChar = 0;           // Sets the 10's place of the temperature value in degrees Celsius
int OnesChar = 0;

// Initializes Timer A0
void TimerSetup(void)
{
    TA0CTL = TASSEL_1 + MC_1;           // SMCLK set to UP mode
    TA0CCR0 = 4095;                     // PWM Period

    // Fan PWM
    TA0CCR1 = 0;                        // PWM of Fan
    TA0CCTL1 = OUTMOD_7;                // Toggle/Set
    P1DIR |= BIT2;                      // Sets the Red LED to the output direction
    P1SEL |= BIT2;                      // Allows BIT2 to be the output of TA0.1
    // ADC PWM
    TA1CTL = TASSEL_1 + MC_1;           // Sets the Timer for the fan to SMCLK and UP mode
    TA1CCR0 = 4095;                     // Sets thet PWM period of the ADC
    TA1CCTL0 |= CCIE;                   // Enables the compare/capture interrupt of the Timer 1
}

// Initializes UART
void UARTSetup(void)
{
    P4SEL |= (BIT4+BIT5);                   // Allows BIT4 to become the TXD output and BIT5 to become the RXD input
    UCA1CTL1 |= UCSWRST;                    // State Machine Reset + Small Clock Initialization
    UCA1CTL1 |= UCSSEL_2;                   // Sets USCI Clock Source to SMCLK
    UCA1BR0 = 6;                            // 9600 Baud Rate
    UCA1BR1 = 0;                            // 9600 Baud Rate
    UCA1MCTL |= UCBRS_0 + UCBRF_13 + UCOS16;
    UCA1CTL1 &= ~UCSWRST;                   // Initialize USCI State Machine
    UCA1IE |= UCRXIE;                       // Enable USCI_A0 RX interrupt
}

int main(void)
{
    WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
    ADC12CTL0 = ADC12SHT02 + ADC12ON;         // Sampling time, ADC12 on
    //ADC12CTL1 = ADC12SHP + ADC12DIV_7+ ADC12SSEL_1;                     // Use sampling timer

    ADC12CTL1 = ADC12SHP;                     // Use sampling timer
    ADC12IE = 0x01;                           // Enable interrupt
    ADC12CTL0 |= ADC12ENC;
    P6SEL |= 0x01;                              // P6.0 ADC option select
    TimerSetup();
    UARTSetup();

while (1)
  {
    ADC12CTL0 |= ADC12SC;                   // Start sampling/conversion
    
    __bis_SR_register(LPM0_bits + GIE);     // LPM0, ADC12_ISR will force exit
    __no_operation();                       // For debugger
  }
}
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC12_VECTOR))) ADC12_ISR (void)
#else
#error Compiler not supported!
#endif
{
//ADC12CTL0 &= ~ADC12SC; // End Sampling / Conversion 
  switch(__even_in_range(ADC12IV,34))
  {
  case  0: break;                           // Vector  0:  No interrupt
  case  2: break;                           // Vector  2:  ADC overflow
  case  4: break;                           // Vector  4:  ADC timing overflow
  case  6:                                  // Vector  6:  ADC12IFG0
          temp = (ADC12MEM0/10) * (3.3/4096) * (1/0.00625); // Bits * (Volts/Bits) * (Celsius/Volts) Converts ADC output from bits to degrees Celsius
          
          
        TensChar = temp / 10;           // Sets the 10's place of the temperature value in degrees Celsius
        OnesChar = temp - TensChar*10;  // Sets the 1's place of the temperature value in degrees Celsius
        UCA1TXBUF = TensChar + 48;         // Sends the 10's place value over UART
        UCA1TXBUF = OnesChar + 48;         // Sends the 1's place value over UART
        //UCA1TXBUF = temp;
        __bic_SR_register_on_exit(LPM0_bits);   // Exit active CPU
  case  8: break;                           // Vector  8:  ADC12IFG1
  case 10: break;                           // Vector 10:  ADC12IFG2
  case 12: break;                           // Vector 12:  ADC12IFG3
  case 14: break;                           // Vector 14:  ADC12IFG4
  case 16: break;                           // Vector 16:  ADC12IFG5
  case 18: break;                           // Vector 18:  ADC12IFG6
  case 20: break;                           // Vector 20:  ADC12IFG7
  case 22: break;                           // Vector 22:  ADC12IFG8
  case 24: break;                           // Vector 24:  ADC12IFG9
  case 26: break;                           // Vector 26:  ADC12IFG10
  case 28: break;                           // Vector 28:  ADC12IFG11
  case 30: break;                           // Vector 30:  ADC12IFG12
  case 32: break;                           // Vector 32:  ADC12IFG13
  case 34: break;                           // Vector 34:  ADC12IFG14
  default: break; 
  }
  while(!(UCA1IFG & UCTXIFG));
   UCA1IE |= UCRXIE;                       // Enable USCI_A0 RX interrupt
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A1_VECTOR))) USCI_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
    
TA0CCR1 = UCA1RXBUF; // Sets the Period of the PWM as the value sent over UART.
ADC12CTL0 |= ADC12SC;                   // Start sampling/conversion
}

/*#pragma vector = TIMER1_A0_VECTOR
__interrupt void Timer_A1 (void)
{
    TA1CTL |= MC_0 + TACLR;                 // Clears the timer and sets the timer to stop mode
    ADC12CTL0 |= ADC12SC;                   // Start sampling/conversion
    TA1CCTL0 |= CCIE;                       // Enables the compare/capture interrupt of TA1CCR0
}*/
