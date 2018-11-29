#include <msp430.h>
#include <math.h>

int averager[8] = {0,0,0,0,0,0,0,0};
int average_TEMP = 0;
int current_TEMP = 0;
float ADC_Voltage = 0;
float ADC_Temp = 0;
float perBit = 3.3/4096;
float desired_TEMP = 0;
int error;

void ADC_Setup();
void PWM_Setup();
void input_TEMP();
void UART_Setup();

void UART_Setup()
{
    P4SEL |= BIT4+BIT5;                       // P3.3,4 = USCI_A0 TXD/RXD
    UCA1CTL1 |= UCSWRST;                      // **Put state machine in reset**
    UCA1CTL1 |= UCSSEL_2;                     // SMCLK
    // Baud Rate calculation
    // 1000000/9600 = 104.1667
    // Fractional portion = 0.1667
    // Use Table 24-5 in Family User Guide
    UCA1BR0 = 104;                             // 1MHz 115200 (see User's Guide)
    UCA1BR1 = 0;                              // 1MHz 115200
    UCA1MCTL |= UCBRS_1 + UCBRF_0;            // Modulation UCBRSx=1, UCBRFx=0
    UCA1CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    UCA1IE |= UCRXIE;                         // Enable USCI_A0 TX interrupt
}

void ADC_Setup()
{
    ADC12CTL0 = ADC12SHT02 + ADC12ON;         // Sampling time, ADC12 on
    ADC12CTL1 = ADC12SHP;                     // Use sampling timer
    ADC12IE = 0x01;                           // Enable interrupt

    ADC12CTL0 |= ADC12ENC;
    P6SEL |= 0x01;                            // P6.0 ADC option select
    P6IN |= BIT0;
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
    ADC_Voltage = ADC12MEM0 * perBit;
    ADC_Temp = (ADC_Voltage - .424)/.00625;
    current_TEMP = (int)ADC_Temp;
}

void compareTEMP()
{
    error = average_TEMP - desired_TEMP;
     if (error > 50)
    {
        TA1CCR1 = 500;
    }
    else if ((error <= 50) && (error > 40))
    {
        TA1CCR1 = 400;
    }
    else if ((error <= 40) && (error > 30))
    {
        TA1CCR1 = 300;
    }
    else if ((error <= 30) && (error > 20))
    {
        TA1CCR1 = 200;
    }
    else if ((error <= 20) && (error > 10))
    {
        TA1CCR1 = 100;
    }
    else if ((error <= 10) && (error > 0))
    {
        TA1CCR1 = 1;
    }
    else if (error < 0)
    {
        TA1CCR1 = 0;
    }
}

int main(void)
{
    WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
    UART_Setup();
    PWM_Setup();
    ADC_Setup();
    while (1)
    {
        averager[7] = averager[6];
        averager[6] = averager[5];
        averager[5] = averager[4];
        averager[4] = averager[3];
        averager[3] = averager[2];
        averager[2] = averager[1];
        averager[1] = averager[0];
        averager[0] = current_TEMP;
        average_TEMP = (averager[7] + averager[6] + averager[5] + averager[4]+ averager[3] + averager[2] + averager[1] + averager[0])/8;
        ADC12CTL0 |= ADC12SC;                   // Start sampling/conversion
        __bis_SR_register(LPM0_bits + GIE);
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
      UCA1TXBUF = average_TEMP;
      inputTEMP();
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

#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
    desired_TEMP = UCA1RXBUF;
}
