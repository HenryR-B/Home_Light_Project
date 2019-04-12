#include <XC.h>
#include <sys/attribs.h>
#include <stdio.h>
#include <stdlib.h>

#pragma config FNOSC = FRCPLL       // Internal Fast RC oscillator (8 MHz) w/ PLL
#pragma config FPLLIDIV = DIV_2     // Divide FRC before PLL (now 4 MHz)
#pragma config FPLLMUL = MUL_20     // PLL Multiply (now 80 MHz)
#pragma config FPLLODIV = DIV_2     // Divide After PLL (now 40 MHz)
 
#pragma config FWDTEN = OFF         // Watchdog Timer Disabled
#pragma config FPBDIV = DIV_1       // PBCLK = SYCLK

#define SYSCLK 40000000L
#define FREQ 100000L // We need the ISR for timer 1 every 10 us
#define BAUD_RATE 115200L

#define REF_VOLTAGE 3.3
#define PIR_SENSOR 5 //pin7

#define servo1 LATBbits.LATB5		//pin14
#define servo2 LATBbits.LATB14		//pin25, horizontal

void ADCConf(void);
int ADCRead(char analogPIN);
float ADCVoltage(char analogPIN);

#define Baud2BRG(desired_baud)( (SYSCLK / (16*desired_baud))-1)
 
void UART2Configure(int baud_rate)
{
    // Peripheral Pin Select
    U2RXRbits.U2RXR = 4;    //SET RX to RB8
    RPB9Rbits.RPB9R = 2;    //SET RB9 to TX

    U2MODE = 0;         // disable autobaud, TX and RX enabled only, 8N1, idle=HIGH
    U2STA = 0x1400;     // enable TX and RX
    U2BRG = Baud2BRG(baud_rate); // U2BRG = (FPb / (16*baud)) - 1
    
    U2MODESET = 0x8000;     // enable UART2
}

volatile int ISR_pw_h, ISR_pw_v, ISR_cnt = 0, ISR_frc = 0;


// The Interrupt Service Routine for timer 1 is used to generate one or more standard
// hobby servo signals.  The servo signal has a fixed period of 20ms and a pulse width
// between 0.6ms and 2.4ms.
void __ISR(_TIMER_1_VECTOR, IPL5SOFT) Timer1_Handler(void)
{
	IFS0CLR = _IFS0_T1IF_MASK; // Clear timer 1 interrupt flag, bit 4 of IFS0

	ISR_cnt++;
    if (ISR_pw_h >230 || ISR_pw_h < 65) {
        ISR_pw_h = 100;
    }
    else if (ISR_pw_v > 230 || ISR_pw_v < 65) {
        ISR_pw_v = 100;
    }
    else {
        if (ISR_cnt < ISR_pw_v) {
            servo1 = 1; //vertical
        }
        else {
            servo1 = 0;
        }

        if (ISR_cnt < ISR_pw_h) {
            servo2 = 1; //horizontal
        }
        else {
            servo2 = 0;
        }

        if (ISR_cnt >= 2000) {
            ISR_cnt = 0; // 2000 * 10us=20ms
            ISR_frc++;
        }
    }
}

void SetupTimer1(void)
{
	// Explanation here: https://www.youtube.com/watch?v=bu6TTZHnMPY
	__builtin_disable_interrupts();
	PR1 = (SYSCLK / FREQ) - 1; // since SYSCLK/FREQ = PS*(PR1+1)
	TMR1 = 0;
	T1CONbits.TCKPS = 0; // 3=1:256 prescale value, 2=1:64 prescale value, 1=1:8 prescale value, 0=1:1 prescale value
	T1CONbits.TCS = 0;   // Clock source
	T1CONbits.ON = 1;
	IPC1bits.T1IP = 5;
	IPC1bits.T1IS = 0;
	IFS0bits.T1IF = 0;
	IEC0bits.T1IE = 1;

	INTCONbits.MVEC = 1; //Int multi-vector
	__builtin_enable_interrupts();
}

void delay_ms(int msecs)
{
	int ticks;
	ISR_frc = 0;
	ticks = msecs / 20;
	while (ISR_frc < ticks)
		;
}
// Good information about ADC in PIC32 found here:
// http://umassamherstm5.org/tech-tutorials/pic32-tutorials/pic32mx220-tutorials/adc
void ADCConf(void)
{
	AD1CON1CLR = 0x8000; // disable ADC before configuration
	AD1CON1 = 0x00E0;	// internal counter ends sampling and starts conversion (auto-convert), manual sample
	AD1CON2 = 0;		 // AD1CON2<15:13> set voltage reference to pins AVSS/AVDD
	AD1CON3 = 0x0f01;	// TAD = 4*TPB, acquisition time = 15*TAD
	AD1CON1SET = 0x8000; // Enable ADC
}

int ADCRead(char analogPIN)
{
	AD1CHS = analogPIN << 16; // AD1CHS<16:19> controls which analog pin goes to the ADC

	AD1CON1bits.SAMP = 1; // Begin sampling
	while (AD1CON1bits.SAMP)
		; // wait until acquisition is done
	while (!AD1CON1bits.DONE)
		; // wait until conversion done

	return ADC1BUF0; // result stored in ADC1BUF0
}

float ADCVoltage(char analogPIN)
{
	//multiply by refrence voltage, divide by ADC resolution
	return (float)ADCRead(analogPIN) * REF_VOLTAGE / (1023.0);
}

// Needed to by scanf() and gets()
int _mon_getc(int canblock)
{
	char c;
	
    if (canblock)
    {
	    while( !U2STAbits.URXDA); // wait (block) until data available in RX buffer
	    c=U2RXREG;
	    if(c=='\r') c='\n'; // When using PUTTY, pressing <Enter> sends '\r'.  Ctrl-J sends '\n'
		return (int)c;
    }
    else
    {
        if (U2STAbits.URXDA) // if data available in RX buffer
        {
		    c=U2RXREG;
		    if(c=='\r') c='\n';
			return (int)c;
        }
        else
        {
            return -1; // no characters to return
        }
    }
}

void main(void)
{
    float voltage;
	DDPCON = 0;

    CFGCON = 0;

    UART2Configure(115200);  // Configure UART2 for a baud rate of 115200
 
    ANSELB &= ~(1<<3); // Set RB3 as a digital I/O
    TRISB |= (1<<3);   // configure pin RB3 as input
    CNPUB |= (1<<3);   // Enable pull-up resistor for RB3
    
    TRISBbits.TRISB6 = 0;
	LATBbits.LATB6 = 0;	

	// Configure the pin we are using for servo control as output
	TRISBbits.TRISB5 = 0; //servo

	INTCONbits.MVEC = 1;

	SetupTimer1(); // Set timer 1 (to interrupt every 10 us?)
    ADCConf(); //configure ADC
    delay_ms(2000);
    printf("Home Servo Code\n\r\n\r");
    while(1) {
        printf("\n\rTHE LIGHT IS OFF\n\r");
        if ( PORTBbits.RB3 == 0) {
            delay_ms(500);
            
            if (PORTBbits.RB3 == 0) { 
                delay_ms(500);
                
                if (PORTBbits.RB3 == 0) { 
                    printf("\n\rThe Light is ON\n\r");
                    LATBbits.LATB6 = 1;
                    delay_ms(10*1000);
                    
                }
                else
                {
                    LATBbits.LATB6 = 0;
                }
                
            }
            else
            {
                LATBbits.LATB6 = 0;
            }
            
        }
        else
        {
            LATBbits.LATB6 = 0;
        }
        
        delay_ms(500);
    }
}