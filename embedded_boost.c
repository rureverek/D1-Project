/* embedded_boost.c 
 *
 *  Author: Steve Gunn & Klaus-Peter Zauner & Piotr Mikulowski
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
 *            | A    | PA1 | Voltage on 0.3 Resistor     |
 *            | D    | PD0 | Host connection TX (orange) |
 *            | D    | PD1 | Host connection RX (yellow) |
 *            | D    | PD7 | PWM out to drive MOSFET     |
 *            | D    | PD2 | External output (BUTTON)    |
 *            | -    | VCC | External output (BUTTON)    |
 *            | A    | PA7 | User error LED output 	     |
 *            | A    | PA6 | System error LED output     |
 *            | B    |  -  | Control LCD				 |
 *            | C    |  -  | Data LCD				     |			
 *            | -    | GND | Circuit's GND   		     | 
 *			  | A    | PA2 | Negative Dif Input			 |
 *			  | A 	 | PA3 | Positive Dif Input			 |
 */

#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <avr/interrupt.h>
#include "lcd.h"

#define DELAY_MS      100
#define BDRATE_BAUD  9600
#define R_VALUE 0.2
#define ADCREF_V     3.3
#define ADCMAXREAD   1023   /* 10 bit ADC */
#define K_parameter 6
#define PWM_DUTY_MAX 240    /* 94% duty cycle */

		void draw_rectangle(int x,int y, uint16_t c){
	
	rectangle r = {x,x+100,y,y+20};	
	fill_rectangle(r, c);
	rectangle r2 = {x+1,x+99,y+1,y+19};
	fill_rectangle(r2,BLACK);
	
}

	void draw_lines(int x,int y, uint16_t c){
	
	rectangle r = {x,x,y,y+5};	
	fill_rectangle(r, c);

	}
	
	void clear_text(void){
	
	rectangle r = {110,150,10,150};	
	fill_rectangle(r, BLACK);

	}
	
void init_pwm1(int x,int y, uint16_t c){
	
	rectangle r1 = {x,x+200,y,y+10};
	fill_rectangle(r1,c);
	
	rectangle r2 = {x+1,x+199,y+1,y+9};
	fill_rectangle(r2,BLACK);
	
}


void draw_pwm(int x,int y, int pwm, uint16_t c){
	
	int l = (double)pwm/255*200;
	rectangle r = {x,x+l,y,y+10};	
	rectangle r2 = {x+1,x+199,y+1,y+9};
	fill_rectangle(r2,BLACK);
	fill_rectangle(r, c);
	
}

void init_stdio2uart0(void);
int uputchar0(char c, FILE *stream);
int ugetchar0(FILE *stream);
		
void init_adc(void);
double v_load(void);
double i_load(void);

void init_pwm(void);
void pwm_duty(uint8_t x);

	int V_target = 3;
	char Vchar = '3';
	char V_targetc;
	double error = 0;
	double integerror = 0;
	double errorprev = 0;
	double diferror = 0;
	int reset = 1;
	
	


//Code adapted from "PID without PHD" by Tim Wescott
typedef struct
{
double dState; // Last position input
double iState; // Integrator state
double iMax, iMin; // Maximum and minimum allowable integrator state
double iGain, // integral gain
pGain, // proportional gain
dGain; // derivative gain
} SPid;



double UpdatePID(SPid * pid, double error, double position);


ISR(INT0_vect)
{

diferror = 0;
integerror = 0;
error = 0;
reset = 0;

}

int main(void)
{
	SPid pid = {0,0,0,0,0,0,0};
	uint16_t cnt =0;
    int dutyCycle;//255/2; //50% duty cycle    	
	init_stdio2uart0();
	init_pwm(); 
	init_adc();
	double kP;
	double kI;
	double kD;
	double Vout;
	double Iout;
	double Vadc;
	
	begin:
	dutyCycle = 128;
	kP = 7;
	kI = 0;//0.3;
	kD = 0;//0.05;
	Iout = 0;
		
pid.iMax = 1;
pid.iMin = 0.1;
pid.iGain = 0.1;
pid.pGain = 0.1;
pid.dGain = 0.1;
	//SET EXT. INTERRUPT//
	EICRA |= _BV(ISC01) | _BV(ISC00); //rising edge ext interrupt
	sei();
	EIMSK |= _BV(INT0); //enable
	
	//SET LED OUTPUT//
	
	DDRA = _BV(PA7) | _BV(PA6);
	PORTA = 0x00;  //Off
	
	
	/////////////////LCD LOADING//////////////////
	init_lcd();
	display.background = BLACK;
	display.foreground = WHITE;
	clear_screen();
	set_orientation(North);

	clear_screen();
	
	char buf [20]; 
	
	draw_rectangle(5,5,WHITE);
	display.foreground = CYAN;
	display.x = 18; display.y = 13; display_string("VOLTAGE LOAD"); 
	
	draw_rectangle(5,25,WHITE);
	
	display.x = 18; display.y = 33; display_string("VOLTAGE ADC");

	draw_rectangle(5,45,WHITE);
	display.foreground = YELLOW;
	display.x = 18; display.y = 53; display_string("VOLTAGE TARGET");
	display.foreground = RED;
	draw_rectangle(5,65,WHITE);
	
	display.x = 18; display.y = 73; display_string("ERROR");
	
	draw_rectangle(5,85,WHITE);
	
	display.foreground = YELLOW;
	display.x = 18; display.y = 93; display_string("K PROPORTIONAL");
	
	draw_rectangle(5,105,WHITE);
	
	display.x = 18; display.y = 113; display_string("K INTEGRAL");
	
	draw_rectangle(5,125,WHITE);
	
	display.x = 18; display.y = 133; display_string("K DERIVATIVE");
	display.foreground = WHITE;
	
	init_pwm1(20,200,WHITE);
	
	display.x = 110;
	display.y = 180;
	
	display.foreground = GREEN;
	display_string("PWM %");
	
	display.x = 20;
	display.y = 180;
	display_string("0 %");
	
	display.x = 200;
	display.y = 180;
	display_string("100 %");
	
	display.foreground = WHITE;
	
	draw_lines (40,210,	WHITE);
	draw_lines (60,210,	WHITE);
	draw_lines (80,210,	WHITE);
	draw_lines (100,210,WHITE);
	draw_lines (120,210,WHITE);
	draw_lines (140,210,WHITE);
	draw_lines (160,210,WHITE);
	draw_lines (180,210,WHITE);
	draw_lines (200,210,WHITE);


	
	int input;
	int fail;
	char bill;
	char string1[3];
	
	inp:
	
	printf("=====MENU=====\n");
	printf("1. Change target load voltage\n");
	printf("2. Change control system\n");
	printf("3. Resume\n");
	fail = scanf("%d",&input);
	
	if(fail != 1 || (input != 1 && input != 2 && input != 3)){
		if(fail != 1){
			scanf("%c",&bill);
			printf("Input: %c\n", bill);
		}
		else printf("Input: %d\n", input);
		
		goto erroruser;
	}
	printf("Input: %d\n", input);
	
	switch (input)
	{	
		case 1: goto volchange;
				break;
		case 2: goto ctrchange;
				break;
		case 3: goto start;
				break;
		default: goto erroruser;
}
	//CHANGE TARGET VOLTAGE MENU
	volchange:
	printf("=====TARGET VOLTAGE CHANGE=====\n");
	printf("Type desire target load voltage: [range 2-10]\n");
	fail = scanf("%d",&V_target);
	Vchar = V_target + '0';
	if(V_target > 10 || V_target < 2 || fail != 1){
	
		if(fail != 1){scanf("%c",&bill);
			printf("Input: %c\n", bill);
		}
		else
			printf("Input: %d\n", V_target);
		
			printf("Error, input not valid.\n");
		goto erroruser;
		
	}
	printf("Input: %d\n", V_target);
	goto start;
	
	//CHANGE CONTROL 
	ctrchange:
	printf("=====CONTROL SYSTEM CHANGE=====\n");
	printf("1. P Control fast\n");
	printf("2. P Control slow\n");
	printf("3. PI Control\n");
	printf("4. PID Control\n");
	printf("5. Custom Change: kP (x.xx)\n");
	printf("6. Custom Change: kI (x.xx)\n");
	printf("7. Custom Change: kD (x.xx)\n");
	printf("8. Resume (x.xx)\n");
	input = 0;
	fail = scanf("%d",&input);
	
	if(fail != 1 || (input != 1 && input != 2 && input != 3 && input != 4 && input != 5 && input != 6 && input != 7 && input != 8)){
		if(fail != 1){
			scanf("%c",&bill);
			printf("Input: %c\n", bill);
		}
		else printf("Input: %d\n", input);
		printf("Try again\n");
		goto erroruser;
	}
	printf("Input: %d\n", input);
	fgets(string1,2,stdin);
	switch (input){
		
		case 1: 
		
			kP = 7.0;
			kI = 0.0;
			kD = 0.0;
			break;
		case 2: 
		
			kP = 4.0;
			kI = 0.0;
			kD = 0.0;
			break;
		case 3: 
			
			kP = 6.5;
			kI = 0.5;
			kD = 0.0;
			break;
		case 4: 
			
			kP = 5;
			kI = 0.5;
			kD = 0.005;
			break;
		case 5:
			fgets(string1,5,stdin);
			kP = (string1[0] - '0') + ((double) (string1[2] - '0'))*0.1 + ((double) (string1[3] - '0'))*0.01;
			goto ctrchange;
			break;
		case 6:
			fgets(string1,5,stdin);
			kI = (string1[0] - '0') + ((double) (string1[2] - '0'))*0.1 + ((double) (string1[3] - '0'))*0.01;
			goto ctrchange;
			break;			
		case 7:
			fgets(string1,5,stdin);
			kD = (string1[0] - '0') + ((double) (string1[2] - '0'))*0.1 + ((double) (string1[3] - '0'))*0.01;
			goto ctrchange;
			break;
		
		case 8:
			goto start;
			
		default: 
			goto erroruser;
			break;
}
	goto start;
	
	
	//ERROR SYSTEM STATE//
	errorsys:
	printf("=====SYSTEM ERROR=====\n");
	printf("Press any key to reset\n");
	PORTA |= _BV(PA7);		//LED
	scanf("%c %c",&bill,&bill);
	PORTA &= ~_BV(PA6);
	goto begin;
	
	
	//ERROR USER STATE
	erroruser:
	printf("=====USER ERROR=====\n");
	PORTA |= _BV(PA6);
	_delay_ms(1000);
	PORTA &= ~_BV(PA6);
	goto inp;
	
	
	
	start:
	printf("Press button to back to menu...\n");
	////MAIN LOOP////
	
	error = 0;
	reset = 1;
	clear_text();
	cnt = 0;
	while(reset) {	    
	   // printf( "%04d:  ", cnt );
		//printf( "Error: %f:  ", Vout );
	    /* Limited by PWM_DUTY_MAX */
	    	
	   // printf( "  -->  %5.3f Vadc", v_load() );
	
		//printf( "  -->  %5.3f Vload\r\n", Vout=v_load());
		
		Vout=v_load();
		Vadc = (double)Vout / K_parameter;
		
		
		//PID controller
		error= Vout - V_target;	//error calc
		//error=V_target - Vout;	//error calc
		//dutyCycle += UpdatePID(&pid, error, Vout);
		
		integerror = integerror + error*(DELAY_MS)/1000;			//integral of e(t)
		diferror = (error-errorprev)/DELAY_MS*1000;					//derivative (slope) of e(t)
		
		
		dutyCycle = dutyCycle - error*kP; 			// P term
		if(integerror > 0.2) integerror = 0.2;
		dutyCycle = dutyCycle - integerror*kI;		// I term
		if(diferror > 5) diferror = 5;
		dutyCycle = dutyCycle - diferror*kD;		// D term
		
		pwm_duty(dutyCycle);  //Changing duty cycle
		errorprev = error;

	    
	    cnt++;		
		if(error < 0.5 && error > -0.5)cnt = 0;
		
	//LCD DISPLAY
		//	if(dutyCycle > 255)dutyCycle = 255;
			draw_pwm(20,200,dutyCycle,WHITE);
			
			dtostrf(Vout, 4, 2, buf);
			display.foreground = CYAN;
			display.x = 120; display.y = 13; display_string(buf); //DISPLAY V LOAD
			dtostrf(Vadc, 4, 2, buf);
			display.x = 120; display.y = 33; display_string(buf); //DISPLAY V ADC
			display.foreground = YELLOW;
			display.x = 120; display.y = 53;
			
			if(V_target == 10)  display_string("10");
			else				display_char(Vchar); //DISPLAY V TARGET
			
			dtostrf(error, 4, 2, buf);
			display.x = 120; display.y = 73; 
			
			if(error < 0.5 && error > -0.5)display.foreground = GREEN;
			else display.foreground = RED;
			
			display_string(buf); //DISPLAY Error
			display.foreground = YELLOW;
			
			dtostrf(kP, 4, 2, buf);
			display.x = 120; display.y = 93; display_string(buf); //DISPLAY Kp
			
			dtostrf(kI, 4, 2, buf);
			display.x = 120; display.y = 113; display_string(buf); //DISPLAY Ki
			
			dtostrf(kD, 4, 2, buf);
			display.x = 120; display.y = 133; display_string(buf); //DISPLAY Kd

			display.foreground = WHITE;
			
			//SYSTEM ERROR RECEIVER
			if(cnt > 30 && (error > 4 || error < -4)){
				pwm_duty(5);
				break;
			}
			
			_delay_ms(DELAY_MS);
	
	}
	
	if(reset)goto errorsys;
	else goto inp;
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
	ADMUX = 0X00;
     uint16_t adcread;
         
     /* Start single conversion */
     ADCSRA |= _BV ( ADSC );
     /* Wait for conversion to complete */
     while ( ADCSRA & _BV ( ADSC ) );
     adcread = ADC;
    
    // printf("ADC=%4d", adcread);  
 
     return (double) (K_parameter*adcread * ADCREF_V/ADCMAXREAD);
}

double i_load(void)
{
     uint16_t adcread;
     ADMUX = 0x09; //10x gain op-amp
     /* Start single conversion */
     ADCSRA |= _BV ( ADSC );
     /* Wait for conversion to complete */
     while ( ADCSRA & _BV ( ADSC ) );
     adcread = ADC;
    
    // printf("ADC=%4d", adcread);  
 
     return (double) (adcread * ADCREF_V/ADCMAXREAD);
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
    
    //printf("PWM=%3u  ==>  ", x);  

    OCR2A = x;
}

double UpdatePID(SPid * pid, double error, double position)
{
double pTerm, dTerm, iTerm;

pTerm = pid->pGain * error; // calculate the proportional term

// calculate the integral state with appropriate limiting
pid->iState += error;

if (pid->iState > pid->iMax) 
		pid->iState = pid->iMax;
else if (pid->iState < pid->iMin) 
		pid->iState = pid->iMin;
	
iTerm = pid->iGain * pid -> iState; // calculate the integral term

dTerm = pid->dGain * (pid->dState - position);
pid->dState = position;

return pTerm + dTerm + iTerm;
}