#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#include <math.h>

#define a1     _BV(PD0) // black
#define a3    _BV(PD7) // red
#define a2      _BV(PD4) // green
#define a4   _BV(PB0) // blue

#define DELAY  1000 /* milliseconds between steps */

int main(void){  
  DDRB = 0xff;    /* Enable output on all of the B pins */  
  PORTB = 0x00;            /* Set them all to 0v */  
  while(1){                     /* main loop here */    
		PORTD |= a1;
		PORTD &= ~a2;
		PORTD &= ~a3;
		PORTD |= a4;

		_delay_ms(DELAY);

		PORTD |= a1;
		PORTD &= ~a2;
		PORTD |= a3;
		PORTD &= ~a4;

		_delay_ms(DELAY);

		PORTD &= ~a1;
		PORTD |= a2;
		PORTD |= a3;
		PORTD &= ~a4;

		_delay_ms(DELAY);

		PORTD &= ~a1;
		PORTD |= a2;
		PORTD &= ~a3;
		PORTD |= a4;

		_delay_ms(DELAY);
   }
}