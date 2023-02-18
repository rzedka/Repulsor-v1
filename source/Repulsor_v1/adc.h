#ifndef ADC_H_INCLUDED
#define ADC_H_INCLUDED


#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/interrupt.h>


#include "main.h"

extern volatile uint16_t adc_value;
extern volatile uint8_t adc_flag;

void ADC_setup(void);

#endif // ADC_H_INCLUDED
