#define						F_CPU 16000000

#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include <util/atomic.h>
#include <avr/interrupt.h>					
#include <string.h>
#include <stdlib.h>							
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
//
#define BIT(x) 				(1ul << (x)) 				// makro bitu (max 32 bit)
#define bitIsSet(x,y) 		((x) & (BIT(y)))
#define bitIsClear(x,y) 	!bitIsSet((x), (y))

#define outputLow(port,pin)						port &= ~(1<<pin)
#define outputHigh(port,pin)					port |= (1<<pin)
#define setAsInput(portdir,pin)					portdir &= ~(1<<pin)
#define setAsOutput(portdir,pin)				portdir |= (1<<pin)
//------------------------------------------------------------------
#define LED_1			PD0
#define LED_2			PD1
#define LED_3			PD4
#define LED_4			PD5
//
#define LED_A			PB5
#define LED_B			PB3
#define LED_C			PC2
#define LED_D			PC4
#define LED_E			PC5
#define LED_F			PB4
#define LED_G			PC1
#define LED_DA			PC3
//

#define LED_OFF()		outputHigh(PORTB,LED_A);outputHigh(PORTB,LED_B);outputHigh(PORTC,LED_C);outputHigh(PORTC,LED_D);outputHigh(PORTC,LED_E);outputHigh(PORTB,LED_F);outputHigh(PORTC,LED_G);outputHigh(PORTC,LED_DA);
//
void setLed(uint8_t led , uint8_t val){
	//
	outputHigh(PORTD,LED_1);outputHigh(PORTD,LED_2);outputHigh(PORTD,LED_3);outputHigh(PORTD,LED_4);
	//
	switch(led){
		case 0 : outputLow(PORTD,LED_1); break ;
		case 1 : outputLow(PORTD,LED_2); break ;
		case 2 : outputLow(PORTD,LED_3); break ;
		case 3 : outputLow(PORTD,LED_4); break ;
	}	
	//
	LED_OFF();
	//
	switch(val){
		case 0 : outputLow(PORTB,LED_A);outputLow(PORTB,LED_B);outputLow(PORTC,LED_C);outputLow(PORTC,LED_D);outputLow(PORTC,LED_E);outputLow(PORTB,LED_F);break ;		
		case 1 : outputLow(PORTB,LED_B);outputLow(PORTC,LED_C); break ;		
		case 2 : outputLow(PORTB,LED_A);outputLow(PORTB,LED_B);outputLow(PORTC,LED_D);outputLow(PORTC,LED_E);outputLow(PORTC,LED_G); break ;
		case 3 : outputLow(PORTB,LED_A);outputLow(PORTB,LED_B);outputLow(PORTC,LED_C);outputLow(PORTC,LED_D);outputLow(PORTC,LED_G);break ;		
		case 4 : outputLow(PORTB,LED_B);outputLow(PORTC,LED_C);outputLow(PORTB,LED_F);outputLow(PORTC,LED_G);break ;
		case 5 : outputLow(PORTB,LED_A);outputLow(PORTC,LED_C);outputLow(PORTC,LED_D);outputLow(PORTB,LED_F);outputLow(PORTC,LED_G);break ;
		case 6 : outputLow(PORTB,LED_A);outputLow(PORTC,LED_C);outputLow(PORTC,LED_D);outputLow(PORTC,LED_E);outputLow(PORTB,LED_F);outputLow(PORTC,LED_G);break ;
		case 7 : outputLow(PORTB,LED_A);outputLow(PORTB,LED_B);outputLow(PORTC,LED_C);break ;		
		case 8 : outputLow(PORTB,LED_A);outputLow(PORTB,LED_B);outputLow(PORTC,LED_C);outputLow(PORTC,LED_D);outputLow(PORTC,LED_E);outputLow(PORTB,LED_F);outputLow(PORTC,LED_G);break ;		
		case 9 : outputLow(PORTB,LED_A);outputLow(PORTB,LED_B);outputLow(PORTC,LED_C);outputLow(PORTC,LED_D);outputLow(PORTB,LED_F);outputLow(PORTC,LED_G);break ;		
		//
		case 'a' : outputLow(PORTB,LED_A); break ;
		case 'd' : outputLow(PORTC,LED_D); break ;
		case 'e' : outputLow(PORTB,LED_A);outputLow(PORTC,LED_D);outputLow(PORTC,LED_E);outputLow(PORTB,LED_F);outputLow(PORTC,LED_G);break;
		case 'g' : outputLow(PORTC,LED_G); break ;
	}
}

#define startTimer()			TCNT1=0;TCCR1B = (1<<CS11); // clkI/O/8
#define stopTimer()				TCCR1B &= ~(1 << CS11);
//
#define getWeight(weight)		((double)weight / 1000)
#define getDistance(distance)	((uint16_t)distance * 100)
volatile float					uSeconds ;
volatile uint16_t	     		ek ;
volatile uint8_t				STATUS;
#define maxTimer				5000
// measurement
#define ISR_MEASUREMENT_WAIT	0
#define ISR_MEASUREMENT_START	1
#define ISR_MEASUREMENT_END		2
#define ISR_MEASUREMENT_ERROR	3
//
volatile uint16_t speed = 0 ;

// start
ISR(INT0_vect){	
	startTimer();
	uSeconds = 0;
	STATUS = ISR_MEASUREMENT_START ;	
}

// stop
ISR(INT1_vect){
	stopTimer();
	uSeconds = (TCNT1 / 2);
	//
	if(STATUS == ISR_MEASUREMENT_START ){
		STATUS = ISR_MEASUREMENT_END ;
	}
	// 1 mm / microsecond = 1000 meters / second
	// 100mm distance
	//-------------------
	// 100uS = 999m/s
	// 999us ~ 100m/s
		
	//speed = (uint16_t)(100000 / uSeconds);						
}
volatile uint8_t chr = 'a' ;
volatile uint16_t tim1 = 0 ;

volatile uint8_t changeStatus = 0 ;
volatile uint8_t counter = 0;
volatile uint8_t display[4];
// 62,5kHz / 256 = 256Hz
ISR(TIMER0_OVF_vect){	

	counter++ ;
	if (counter >= 4 ) counter = 0 ;
	
	setLed(counter,display[counter]);

	tim1++ ;
	//~0,5s
	if(tim1 >= 128){		
		//
		if (STATUS != ISR_MEASUREMENT_START){
			changeStatus++ ;
			if(changeStatus >= 10){
				changeStatus = 0 ;
				STATUS = ISR_MEASUREMENT_WAIT;
			}
		}
		//
		switch(chr){
			case 'a' : chr = 'g' ; break ;
			case 'g' : chr = 'd' ; break ;
			case 'd' : chr = 'a' ; break ;
		}
					
		//
		tim1 = 0 ;
	}
}
/**
CS02	CS01	CS00 Description
0		0		0 No clock source (Timer/Counter stopped)
0		0		1 clkI/O/(no prescaling)
0		1		0 clkI/O/8 (from prescaler)
0		1		1 clkI/O/64 (from prescaler)
1		0		0 clkI/O/256 (from prescaler)
1		0		1 clkI/O/1024 (from prescaler)
**/
int main(void){
	_delay_ms(250);
	// Prescaler = FCPU/256
	TCCR0B|=(1<<CS02);
	//Enable Overflow Interrupt Enable
	TIMSK0|=(1<<TOIE0);
	//Initialize Counter
	TCNT0=0;	
	//-------------------------------------------
	DDRC = 0b00111110 ;
	DDRB = 0b00111000 ;
	DDRD = 0b00110011 ;
	//
	PORTC = 0xFF ;
	PORTB = 0xFF ;
	PORTD = 0xFF ;
	//
	EICRA = (0<<ISC01) | (0<<ISC00);	// falling
	EIMSK   |= (1<<INT0) | (1<<INT1);	// INT0 INT2
	STATUS = ISR_MEASUREMENT_WAIT;
	sei();	
	//
	while(1){
		//
		switch(STATUS){
			case ISR_MEASUREMENT_WAIT	: display[0] = chr ; display[1] = chr ;display[2] = chr ;display[3] = chr ;break ;
			//case ISR_MEASUREMENT_START	:  break ;
			
			case ISR_MEASUREMENT_END	:  
				speed = (uint16_t)(100000 / uSeconds);
				ek    = (0.001*speed * speed) / 2 ; // E=(m*v^2)/2 , m= 1gram
				display[3] = ' ' ;
				display[2] = (speed % 10);
				display[1] = ((speed / 10) % 10);
				display[0] = ((speed / 100) % 10);
				//
				
				_delay_ms(2500);
				
				display[2] = (ek % 10);
				display[1] = ((ek / 10) % 10);
				display[0] = ((ek / 100) % 10);
				_delay_ms(2500);				
				//STATUS = ISR_MEASUREMENT_WAIT;
			break ;
			//
			case ISR_MEASUREMENT_ERROR	:  display[0] = 'e' ;display[1] = 'g';display[2] = 'g';display[3] = 'g';break ;
			
		}		
		//
		if(bitIsClear(PIND,PD2)){display[0] = 'e';}
		if(bitIsClear(PIND,PD3)){display[3] = 'e';}			
		//
		_delay_ms(25);
	}
}