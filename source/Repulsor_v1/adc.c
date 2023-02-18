#include "adc.h"



void ADC_setup(void)
{
//    ADMUX |= (0<<REFS1)|(0<<REFS0)|(1<<ADLAR)|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(0<<MUX0);
    /// AREF pin = Reference, ADC0 input pin, Left Adjusted result (only 8 MSbits are valid)

     ADMUX |= (0<<REFS1)|(0<<REFS0)|(0<<ADLAR)|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(0<<MUX0);
    /// AREF pin = Reference, ADC0 input pin, Right Adjusted result (10 MSbits are valid)
    /// ADCH = [0 0 0 0 0 0 A9 A8], ADCL = [A7 -- A0]
    ADCSRA |= (1<<ADEN)|(0<<ADSC)|(0<<ADATE)|(1<<ADIE)|(1<<ADPS2)|(0<<ADPS1)|(1<<ADPS0);
    /// ADC Enabled,,  f_clk_adc = 500 kHz (P = 32), autotriggering OFF
    /// =============== MAXIMUM F_CLK_ADC = 1 MHz !!!! ===================================
    /// ADC Interrupt Enabled, Auto-triggering Enabled,

    ADCSRB |= (0<<ADTS2)|(0<<ADTS1)|(0<<ADTS0);
    /// ADC conversion Starts by triggering it manually
    adc_value = 0;
}

//uint16_t ADC_start_conversion(void)
//{
//
//}

/// ============ ISR ============================

ISR(ADC_vect)
{
    /// Conversion Completed:
    /// (ADLAR == 1) left adjusted result (8 bits)
//    adc_value = ADCL;
//    adc_value = ADCH;

    /// (ADLAR == 0) right adjusted 10 bit result
    adc_value = ADCL; // 0x00FF
    adc_value |= (uint16_t)(ADCH<<8); // 0x03FF

    adc_flag = 1;
}
