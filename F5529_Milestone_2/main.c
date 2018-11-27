#include <msp430.h>
#include <math.h>

float current_TEMP;
float ADC_Voltage;
float ADC_Temp;
const perBit = (3.3/4096);
float desired_TEMP;

void ADC_Setup();
void PWM_Setup();
void input_TEMP();
void UART_Setup();

void UART_Setup()
{
    P3SEL |= BIT3+BIT4;                       // P3.3,4 = USCI_A0 TXD/RXD
    UCA0CTL1 |= UCSWRST;                      // **Put state machine in reset**
    UCA0CTL1 |= UCSSEL_2;                     // SMCLK
    // Baud Rate calculation
    // 1000000/9600 = 104.1667
    // Fractional portion = 0.1667
    // Use Table 24-5 in Family User Guide
    UCA0BR0 = 104;                             // 1MHz 115200 (see User's Guide)
    UCA0BR1 = 0;                              // 1MHz 115200
    UCA0MCTL |= UCBRS_3 + UCBRF_0;            // Modulation UCBRSx=1, UCBRFx=0
    UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    UCA0IE |= UCTXIE;                         // Enable USCI_A0 TX interrupt
}

void ADC_Setup()
{
    ADC12CTL0 = ADC12SHT02 + ADC12ON;         // Sampling time, ADC12 on
    ADC12CTL1 = ADC12SHP;                     // Use sampling timer
    ADC12IE = 0x01;                           // Enable interrupt
    ADC12CTL0 |= ADC12ENC;
    P6SEL |= 0x01;                            // P6.0 ADC option select
}

void PWM_Setup()
{
    P2DIR |= BIT0; //P2.0 output
    P2SEL |= BIT0; //2.0 TA1
    TA1CTL = TASSEL_2 + MC_1 + TACLR; // SMCLK, up-down mode, clear
    TA1CCR0 = 500; // PWM Period
    TA1CCR1 = 1; // CCR1 PWM duty cycle
    TA1CCTL1 = OUTMOD_7; // CCR1 toggle/set
}


void inputTEMP()
{
    ADC_Temp = (ADC_Voltage - .424)/.00625;
    current_TEMP = (int)ADC_Temp;
}

void compareTEMP()
{

}

int main(void)
{
    WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
    UART_Setup();
    PWM_Setup();
    ADC_Setup();
    while (1)
    {
        ADC12CTL0 |= ADC12SC;                   // Start sampling/conversion
        ADC_Voltage = ADC12MEM0 * perBit;
        __bis_SR_register(GIE);
    }
}

#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)

{
  switch(__even_in_range(ADC12IV,34))
  {
  case  0: break;                           // Vector  0:  No interrupt
  case  2: break;                           // Vector  2:  ADC overflow
  case  4: break;                           // Vector  4:  ADC timing overflow
  case  6:                                  // Vector  6:  ADC12IFG0
    inputTEMP();
    compareTEMP();
    __bic_SR_register_on_exit(LPM0_bits);   // Exit active CPU
    break;
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
}

#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
  switch(__even_in_range(UCA0IV,4))
  {
  case 0:break;                             // Vector 0 - no interrupt
  case 2:                                   // Vector 2 - RXIFG
    while (!(UCA0IFG & UCTXIFG));             // USCI_A0 TX buffer ready?
    UCA0IFG &= ~UCTXIFG; //Clears TX flag interrupt
    UCA0TXBUF = ADC_Temp;                  //Current temp read from Tx buffer
    break;
  case 4:break;                             // Vector 4 - TXIFG
  default: break;

  if (UCA0IFG & UCRXIFG)
  {
      UCA0IFG &= ~UCRXIFG;
      UCA0RXBUF = desired_TEMP;
  }

  }
}

