#ifndef PID_H_INCLUDED
#define PID_H_INCLUDED

#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/interrupt.h>


#include "main.h"
#include "uart.h"
extern volatile uint8_t rx_array[20];

uint8_t PID_CMD_Parser(int16_t *setpoint, uint16_t *Kp, uint16_t *Ki, uint16_t *Kd, int16_t *y_i0, uint8_t *mode);


#endif // PID_H_INCLUDED
