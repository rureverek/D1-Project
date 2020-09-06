/* embedded_boost.c 
 *
 *  Author: Steve Gunn & Klaus-Peter Zauner 
 * Licence: This work is licensed under the Creative Commons Attribution License. 
 *          View this license at http://creativecommons.org/about/licenses/
 *   Notes: 
 *          - Use with a terminal program
 * 
 *          - F_CPU must be defined to match the clock frequency
 *
 *          - Compile with the options to enable floating point
 *            numbers in printf(): 
 *               -Wl,-u,vfprintf -lprintf_flt -lm
 *
 *          - Pin assignment: 
 *            | Port | Pin | Use                         |
 *            |------+-----+-----------------------------|
 *            | A    | PA0 | Voltage at load             |
 *            | D    | PD0 | Host connection TX (orange) |
 *            | D    | PD1 | Host connection RX (yellow) |
 *            | D    | PD7 | PWM out to drive MOSFET     |
 */

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <avr/interrupt.h>

#define DELAY_MS      100
#define BDRATE_BAUD  9600

#define ADCREF_V     3.3
#define ADCMAXREAD   1023   /* 10 bit ADC */
#define K 6

/* Find out what value gives the maximum
   output voltage for your circuit:
*/
#define PWM_DUTY_MAX 240    /* 94% duty cycle */
		
void init_stdio2uart0(void);
int uputchar0(char c, FILE *stream);
int ugetchar0(FILE *stream);
		
void init_adc(void);
double v_load(void);

void init_pwm(void);
void pwm_duty(uint8_t x);

	int V_target = 0;
	char V_targetc;
	double error = 0;
	double integerror = 0;
	double errorprev = 0;

ISR(INT0_vect)
{
	
V_targetc = ugetchar0(stdin);
V_target = V_targetc - '0';
if(!V_target)V_target = 10;
//if(V_target > 10 && V_target < 2) PORTD |= _BV(PB7); //turn on led
//else PORTD &= ~_BV(PB7);
diferror = 0;
integerror = 0;
error = 0;
}

int main(void)
{
	uint16_t cnt =0;
    int dutyCycle = 255/2; //50% duty cycle    	
	init_stdio2uart0();
	init_pwm(); 
	init_adc();
	double kP=1;//0.5;
	double kI =0.3 ;//1;
	double kD = 0.05;//0.1;
	double Vout;
	double error = 0;
	double integerror = 0;
	double errorprev = 0;
	double diferror = 0;
	//char V_targetc = 3;

	V_targetc = ugetchar0(stdin);
	V_target = V_targetc - '0';
	EICRA |= _BV(ISC01) | _BV(ISC00); //rising edge ext interrupt
	sei();
	EIMSK |= _BV(INT0); //enable
	for(;;) {	    
	    printf( "%04d:  ", cnt );
		printf( "Error: %d:  ", V_target );
	    pwm_duty(dutyCycle);   /* Limited by PWM_DUTY_MAX */
	    	
	   // printf( "  -->  %5.3f Vadc", v_load() );
	
		printf( "  -->  %5.3f Vload\r\n", Vout=v_load());
		//PI controller
		//Change duty cycle based on error
		error=Vout-V_target;
		integerror = integerror + error*(DELAY_MS)/1000;	//integral of e(t)
		diferror = (error-errorprev)/DELAY_MS*1000;					//derivative (slope) of e(t)
		
		dutyCycle = dutyCycle - error*kP; 			// P term
		dutyCycle = dutyCycle - integerror*kI;		// I term
		dutyCycle = dutyCycle - diferror*kD;			// D term
		
		errorprev = error;
	    _delay_ms(DELAY_MS);
	    cnt++;
	
	}
}

int uputchar0(char c, FILE *stream)
{
	if (c == '\n') uputchar0('\r', stream);
	while (!(UCSR0A & _BV(UDRE0)));
	UDR0 = c;
	return c;
}

int ugetchar0(FILE *stream)
{
	while(!(UCSR0A & _BV(RXC0)));
	return UDR0;
}

void init_stdio2uart0(void)
{
	/* Configure UART0 baud rate, one start bit, 8-bit, no parity and one stop bit */
	UBRR0H = (F_CPU/(BDRATE_BAUD*16L)-1) >> 8;
	UBRR0L = (F_CPU/(BDRATE_BAUD*16L)-1);
	UCSR0B = _BV(RXEN0) | _BV(TXEN0);
	UCSR0C = _BV(UCSZ00) | _BV(UCSZ01);

	/* Setup new streams for input and output */
	static FILE uout = FDEV_SETUP_STREAM(uputchar0, NULL, _FDEV_SETUP_WRITE);
	static FILE uin = FDEV_SETUP_STREAM(NULL, ugetchar0, _FDEV_SETUP_READ);

	/* Redirect all standard streams to UART0 */
	stdout = &uout;
	stderr = &uout;
	stdin = &uin;
}


void init_adc (void)
{
    /* REFSx = 0 : Select AREF as reference
     * ADLAR = 0 : Right shift result
     *  MUXx = 0 : Default to channel 0
     */
    ADMUX = 0x00;
    /*  ADEN = 1 : Enable the ADC
     * ADPS2 = 1 : Configure ADC prescaler
     * ADPS1 = 1 : F_ADC = F_CPU / 64
     * ADPS0 = 0 :       = 187.5 kHz
     */
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1);
}


double v_load(void)
{
     uint16_t adcread;
         
     /* Start single conversion */
     ADCSRA |= _BV ( ADSC );
     /* Wait for conversion to complete */
     while ( ADCSRA & _BV ( ADSC ) );
     adcread = ADC;
    
     printf("ADC=%4d", adcread);  
 
     return (double) (K*adcread * ADCREF_V/ADCMAXREAD);
}



void init_pwm(void)
{
    /* TIMER 2 */
    DDRD |= _BV(PD6); /* PWM out */
    DDRD |= _BV(PD7); /* inv. PWM out */
    

    TCCR2A = _BV(WGM20) | /* fast PWM/MAX */
	     _BV(WGM21) | /* fast PWM/MAX */
	     _BV(COM2A1); /* A output */
    TCCR2B = _BV(CS20);   /* no prescaling */   
}


/* Adjust PWM duty cycle
   Keep in mind this is not monotonic
   a 100% duty cycle has no switching
   and consequently will not boost.  
*/
void pwm_duty(uint8_t x) 
{
    x = x > PWM_DUTY_MAX ? PWM_DUTY_MAX : x;
    
    printf("PWM=%3u  ==>  ", x);  

    OCR2A = x;
}

