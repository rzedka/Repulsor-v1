#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED

/// GLOBAL DEFINES:

/// MCU Platform type:
#define ARDUINO_NANO


/// CPU clock frequency:
#define F_CPU_16
//#define F_CPU_4 /// TIMER & UART functions not finished in this CLK mode !!!

/// UART debugging feature:
#define UART_TERM
//#define DEBUG

/// Other:
#define T_IDLE 500 /// [ms]
#define SET_INIT 15
#define KP_INIT 50
#define KI_INIT 0
#define KD_INIT 50

/// LEDs:
#define DDR_LED DDRB
#define PORT_LED PORTB
#define LED_IDLE_PIN PINB5
#define LED_OVF_PIN PINB4

/// MOSFET gate:
#define DDR_FET DDRD
#define PORT_FET PORTD
#define PIN_FET0 PIND6

/// Ctrl button:
#define DDR_BTN DDRD
#define PIN_BTN PIND
#define PORT_BTN PORTD
#define PIN_BTN0 PIND4



#endif // MAIN_H_INCLUDED
