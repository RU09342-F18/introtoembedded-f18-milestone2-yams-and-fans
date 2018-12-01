#include <msp430.h>
#include <math.h>

unsigned int averager[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //Array for averaging values
//Declarations
int average_TEMP = 0; //Average temperature sent through UART
int current_TEMP = 0; //Latest single temperature reading 
float ADC_Voltage = 0; //Calculated voltage from digital signal
float ADC_Temp = 0; //Temperature with decimal points
int error;
float desired_TEMP = 23; //desired temperature for system to achieve, set around room temp initially


float perBit = 3.3/4096; //Constant for converting digital signal to analog sampled voltage


void ADC_Setup();
void PWM_Setup();
void input_TEMP();
void UART_Setup();

void UART_Setup()
{
    P4SEL |= BIT4+BIT5;                       // P4.4,5 = USCI_A1 TXD/RXD
    UCA1CTL1 |= UCSWRST;                      // **Put state machine in reset**
    UCA1CTL1 |= UCSSEL_2;                     // USCI clock source select, SMCLK
    // Baud Rate calculation
    // 1000000/9600 = 104.1667 (Low Byte)
    // Fractional portion = 0.1667 (High Byte)
    UCA1BR0 = 104;                            // Low byte of baud-rate generator, prescalar setting for 9600 baud rate
    UCA1BR1 = 0;                              // High byte of baud-rate generator
    UCA1CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    UCA1IE |= UCRXIE;                         // Enable USCI_A1 RX interrupt
}

void ADC_Setup()
{
    ADC12CTL0 = ADC12SHT02 + ADC12ON;         // Sampling time, ADC12 on
    ADC12CTL1 = ADC12SHP;                     // Use sampling timer
    ADC12IE = 0x01;                           // Enable interrupt

    ADC12CTL0 |= ADC12ENC;                    // Enable ADC12 conversion
    P6SEL |= 0x01;                            // P6.0 ADC option select
}

void PWM_Setup()
{
    P2DIR |= BIT5;                            // P2.5 output
    P2SEL |= BIT5;                            // 2.5 TA2.2
    TA2CTL = TASSEL_2 + MC_1 + TACLR;         // SMCLK, up mode, clear timer
    TA2CCR0 = 999;                            // PWM Period, 1kHz
    TA2CCR2 = 200;                            // Initial PWM set at 20%
    TA2CCTL2 = OUTMOD_7;                      // Hardware PWM, CCR2 toggle/set
}


void inputTEMP()
{
    ADC_Voltage = ADC12MEM0 * perBit; //Conversion from binary sampled voltage to actual
    ADC_Temp = (ADC_Voltage - .424)/.00625; //Equation for PTAT, voltage => temperature in C
    current_TEMP = (int)ADC_Temp; //Integer value of temperature
}

int main(void)
{
    WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
    UART_Setup(); //UART function call
    PWM_Setup(); //PWM function call
    ADC_Setup(); //ADC function call

    while (1)
    {
     //Average of previous 16 readings
        averager[15] = averager[14];
        averager[14] = averager[13];
        averager[13] = averager[12];
        averager[12] = averager[11];
        averager[11] = averager[10];
        averager[10] = averager[9];
        averager[9] = averager[8];
        averager[8] = averager[7];
        averager[7] = averager[6];
        averager[6] = averager[5];
        averager[5] = averager[4];
        averager[4] = averager[3];
        averager[3] = averager[2];
        averager[2] = averager[1];
        averager[1] = averager[0];
        averager[0] = current_TEMP; //First value in array is the current temperature reading
        //Average calculated by SUM/16
        average_TEMP = (averager[15] + averager[14] + averager[13] + averager[12]+ averager[11] + averager[10] + averager[9] + averager[8] + averager[7] + averager[6] + averager[5] + averager[4]+ averager[3] + averager[2] + averager[1] + averager[0])/16;
        ADC12CTL0 |= ADC12SC;                   // Start sampling/conversion
        __bis_SR_register(LPM0_bits + GIE);
    }
}

//  Analog to Digital Converter interrupt
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)

{
  switch(__even_in_range(ADC12IV,34))
  {
  case  0: break;                           // Vector  0:  No interrupt
  case  2: break;                           // Vector  2:  ADC overflow
  case  4: break;                           // Vector  4:  ADC timing overflow
  case  6:                                  // Vector  6:  ADC12IFG0
      
        inputTEMP(); //Function call for temperature calculations

        error = average_TEMP - desired_TEMP; //Calculates difference between the desired and current temp

        if (error > 0)
        {
            TA2CCR2 = 1000; //Full power, 100% duty cycle, if temperature is too hot
        }
        else if ((error < 0) && (error > -5))
        {
            TA2CCR2 = 500; //50% duty cycle
        }
        else if ((error <= -5) && (error > -10))
        {
            TA2CCR2 = 250; //25% duty cycle
        }
        else if (error < -10)
        {
            TA2CCR2 = 0; //Fan is off
        }
        
//      If negative numbers are added into the 
//      transmit buffer, add 128 to them
        
        if (UCA1TXBUF < 0)
        {
            UCA1TXBUF = UCA1TXBUF + 128;
        }

        UCA1TXBUF = average_TEMP; //Average temperature gets added to the transmit buffer

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

// USCI Interrupt
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
    desired_TEMP = UCA1RXBUF;  //Desired temperature input from the receive buffer
}
