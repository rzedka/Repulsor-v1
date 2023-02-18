#ifndef TIM_H_INCLUDED
#define TIM_H_INCLUDED

#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/interrupt.h>

#include "main.h"

extern volatile uint16_t timer0_cnt;
extern volatile uint16_t timer1_cnt;

#ifdef ENCODER

extern volatile uint8_t timer0_flag;
extern volatile uint8_t dcc_tx_level;
#endif

extern volatile uint16_t timer0_cnt; /// driven by TIMER0
extern volatile uint16_t timer1_cnt; /// driven by TIMER1



void TIMER1_setup(void);

void TIMER0_PWM_setup(void);

void TIMER0_PWM_update(uint8_t PWM_value);

uint16_t TIMER1_get_value(void);




#endif // TIM_H_INCLUDED
