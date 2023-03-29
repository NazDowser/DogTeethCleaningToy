#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#include <math.h>

#define PRESSURE1 PC2
#define PRESSURE2 PC1
#define PRESSURE3 PC0

int dutyCycle = 0;

int main(void)
{
  DDRB |= (1 << PB1); // pwm_motor as output OCR1A
  DDRC = 0x00;
  TCCR1A = (1 << COM1A1) | (1 << COM1A0) | (1 << WGM11) | (1 << WGM10);
  TCCR1B = (1 << WGM12);
  TIMSK1 = (1 << TOIE1); // timer overflow INT enable
  OCR1A = dutyCycle;
  
  adcSet();
  sei(); // enable INT
  
  TCCR1B = (1 << CS10); // no prescaling

  while(1) {
    _delay_ms(1000);
    startConverse();
  }
    
  return 0;
}

void adcSet() {
  ADMUX = (1 << REFS0) | (1 << MUX1);
  ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  DIDR0 = (1 << ADC2D); // digital disable
  startConverse();
}

void startConverse() {
  ADCSRA |= (1 << ADSC);
}

ISR(TIMER1_OVF_vect) {
  OCR1A = dutyCycle;
  startConverse();
}

ISR(ADC_vect) {
  dutyCycle = ADC;
  startConverse();
}

void setPinHigh(volatile byte *port, byte pin) {
  *port |= _BV(pin);
}

void setPinLow(volatile byte *port, byte pin) {
  *port &= ~_BV(pin);
}