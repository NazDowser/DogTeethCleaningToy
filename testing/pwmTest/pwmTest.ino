#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
// 85% inverted minimum
// 24% inverted max
float dutyCycle = 0.5;

int main(void)
{
  // DDRB |= (1 << PB1); // pwm_motor as output OCR1A
  DDRB |= (1 << PB2); // pwm_buzzer as output OCR1B

  // fast pwm 8 bit mode for timer
  // TCCR1A = (1 << COM1A1) | (1 << COM1A0) | (1 << WGM11) | (1 << WGM10);
  TCCR1A = (1 << COM1B1) | (1 << COM1B0) | (1 << WGM11) | (1 << WGM10);
  TCCR1B = (1 << WGM12);

  TIMSK1 = (1 << TOIE1); // timer overflow INT enable
  // OCR1A = int(dutyCycle*1024);
  OCR1B = int(dutyCycle*1024);
  sei(); // enable INT
  
  TCCR1B = (1 << CS10); // no prescaling

  while(1)
  {
    // _delay_ms(1000);
    // dutyCycle += 0.1;
    // if(dutyCycle > 1.0) {
    //   dutyCycle = 0.0;
    // }
  }
    
  return 0;
}

ISR(TIMER1_OVF_vect) {
  // OCR1A = int(dutyCycle*1024); // dynamic dutyCycle changing
  // OCR1B = int(dutyCycle*1024); // dynamic dutyCycle changing
}