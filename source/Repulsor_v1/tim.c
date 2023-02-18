#include "tim.h"


void TIMER1_setup(void)
{

  /// CS12    CS11    CS10      N
  ///   0       0       0       No source
  ///   0       0       1       1
  ///   0       1       0       8
  ///   0       1       1       64
  ///   1       0       0       256
  ///   1       0       1       1024
  ///   1       1       0       Extern Falling
  ///   1       1       1       Extern Rising
    #ifdef F_CPU_16
    /// F_CPU = 16 MHz
  TCCR1A |= (0<<WGM11)|(0<<WGM10);
  TCCR1B |= (0<<WGM13)|(1<<WGM12)|(0<<CS12)|(1<<CS11)|(1<<CS10);
  /// CTC mode, N = 64
  TIMSK1 |= (0<<TOIE1)|(1<<OCIE1A);
  /// CTC interrupt enabled
  OCR1A = 249;
  /// T_ISR = 1.000 ms
  #endif // F_CPU_16
  /// ----------------------------------------------------------
  #ifdef F_CPU_4
    /// F_CPU = 4 MHz
  TCCR1A |= (0<<WGM11)|(0<<WGM10);
  TCCR1B |= (0<<WGM13)|(1<<WGM12)|(0<<CS12)|(1<<CS11)|(0<<CS10);
  /// CTC mode, N = 8
  TIMSK1 |= (0<<TOIE1)|(1<<OCIE1A);
  /// CTC interrupt enabled
  OCR1A = 499;
  /// T_ISR = 1.000 ms

  #endif // F_CPU_4

}

void TIMER0_PWM_setup(void)
{
    /// Set TIMER0 into FAST PWM mode with OCR0A update in BOTTOM TCNT0 position.
    TCCR0A |= (1<<COM0A1)|(0<<COM0A0)|(0<<COM0B1)|(0<<COM0B0)|(1<<WGM01)|(1<<WGM00);
    /// Fast PWM mode, Clear OC0A on Compare Match, set OC0A at BOTTOM (non-inverting mode)
    /// OC0B pin is inactive.

    #ifdef F_CPU_16

    //TCCR0B |= (0<<WGM02)|(0<<CS02)|(1<<CS01)|(0<<CS00); /// TIMER RUNNING
    //TCCR0B |= (0<<WGM02)|(0<<CS02)|(0<<CS01)|(0<<CS00); /// TIMER STOPPED
    TCCR0B |= (1<<WGM02)|(0<<CS02)|(1<<CS01)|(0<<CS00); /// TIMER STARTED, prescaling factor N=8
    ///  Fast PWM mode, OCR0x is updated at BOTTOM position of the counter.
    /// PWM frequency = 7.8125 kHz

    #endif // F_CPU_16

    #ifdef F_CPU_4 /// In case of the 4MHz Arduino nano board:
    //TCCR0B |= (0<<WGM02)|(0<<CS02)|(1<<CS01)|(0<<CS00); /// TIMER RUNNING
    //TCCR0B |= (0<<WGM02)|(0<<CS02)|(0<<CS01)|(0<<CS00); /// TIMER STOPPED
    TCCR0B |= (1<<WGM02)|(0<<CS02)|(0<<CS01)|(1<<CS00); /// TIMER STARTED, N = 1
    ///  Fast PWM mode, OCR0x is updated at BOTTOM position of the counter.
    /// PWM frequency = 62.5 kHz


    #endif // F_CPU_4

//    TIMSK0 |= (0<<TOIE0)|(1<<OCIE0A)|(0<<OCIE0B); ///OCR0A ISR
    /// No interrupt is needed for the PWM mode.
}

void TIMER0_PWM_update(uint8_t PWM_value)
{

//    if(PWM_value>0)
//    OCR0A = PWM_value-1; /// DutyCycle (0 - 127)
//    else
//       OCR0A = PWM_value;
//
    OCR0A = PWM_value; /// DutyCycle (0 - 127)

}

/*
uint16_t TIMER0_get_value(void)
{
   uint16_t val = 0;

   val = timer0_cnt;

   return val;
}
*/

uint16_t TIMER1_get_value(void)
{
   uint16_t val = 0;

   val = timer1_cnt;

   return val;
}


/*
void TIMER0_DECODER_setup(void)
{
    /// Timer0 used for LED blinking

    TCCR0A |= (0<<COM0A1)|(0<<COM0A0)|(0<<COM0B1)|(0<<COM0B0)|(1<<WGM01)|(0<<WGM00);
    /// No output pins enabled.
    //TCCR0B |= (0<<WGM02)|(0<<CS02)|(1<<CS01)|(0<<CS00); /// TIMER RUNNING
    TCCR0B |= (0<<WGM02)|(0<<CS02)|(1<<CS01)|(1<<CS00); /// TIMER STARTED
    /// CTC mode, (N = 64)
  /// CS02    CS01    CS00      N
  ///   0       0       0       No source
  ///   0       0       1       1
  ///   0       1       0       8
  ///   0       1       1       64
  ///   1       0       0       256
  ///   1       0       1       1024
  ///   1       1       0       Extern Falling
  ///   1       1       1       Extern Rising

    TIMSK0 |= (0<<TOIE0)|(1<<OCIE0A)|(0<<OCIE0B);/// Output Compare A ISR
    OCR0A = 249; /// ISR frequency 1.000 kHz
    //OCR0A = 231; /// for bit "0"

}
*/

/// ================== INTERRUPT SERVICE ROUTINE ===============================



ISR(TIMER1_COMPA_vect){
    /// Every 1 ms
    timer1_cnt++;
    /// Also start ADC conversion:
    ADCSRA |= (1<<ADSC);
}

/*
ISR(TIMER0_COMPA_vect)
{
    /// Driving the DCC TX Pin (OUT):
    if(pwm_pin_level==0){
        PORTDCC |= (1<< PWM_PIN); /// Go HIGH
        pwm_pin_level = 1;

    }else{
        PORTDCC &= ~(1<<PWM_PIN); /// Go LOW
        pwm_pin_level = 0;
    }/// end if
    timer0_flag++;

}

*/
