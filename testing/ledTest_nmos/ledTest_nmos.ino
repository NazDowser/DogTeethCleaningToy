#include <avr/io.h>
#include <util/delay.h>

#define PWM_R PD3
#define PWM_G PD5
#define PWM_B PD6
#define RED 0
#define GREEN 1
#define BLUE 2

int main(void)
{
  DDRD |= 0b01101000; // pwmRGB to output

  while(1)
  {
    ledSet(RED);
    _delay_ms(1000);
    ledSet(GREEN);
    _delay_ms(1000);
    ledSet(BLUE);
    _delay_ms(1000);
  }
    
  return 0;
}

void ledSet(byte color) {
  PORTD &= ~0b01101000; // clear
  switch(color) {
    case RED:
      setPinHigh(&PORTD, PWM_R); 
    break;
    case GREEN:
      setPinHigh(&PORTD, PWM_G); 
    break;
    case BLUE:
      setPinHigh(&PORTD, PWM_B); 
    break;
  }
}

void setPinHigh(volatile byte *port, byte pin) {
  *port |= _BV(pin);
}

void setPinLow(volatile byte *port, byte pin) {
  *port &= ~_BV(pin);
}