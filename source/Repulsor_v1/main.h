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
#define DEBUG

/// Other:
#define T_IDLE 500 /// [ms]



/// ============ DCC project defines ===========

#define LED_IDLE_PIN PINB5
#define LED_OVF_PIN PINB4
//#define LEDz_PIN PINB3


#ifdef ARDUINO_NANO
//    #define DDRLED0 DDRC
    #define DDRLEDB DDRB
//    #define DDRLED2 DDRD
//    #define PORTLED0 PORTC
    #define PORTLEDB PORTB
//    #define PORTLED2 PORTD
    #define LED0_PIN PINC0
    #define LED1_PIN PINC1
    #define LED2_PIN PINC2
    #define LED3_PIN PINC3
    #define LED4_PIN PINC4
    #define LED5_PIN PINC5
    #define LED6_PIN PINB4
    #define LED7_PIN PINB3
    #define LED8_PIN PINB2
    #define LED9_PIN PINB1
    #define LED10_PIN PINB0
    #define LED11_PIN PIND7
    #define LED12_PIN PIND6


    #define PIN_CTRL PIND
    #define DDR_CTRL DDRD
    #define PORT_CTRL PORTD

    #define CTRL_PIN0 PIND3
    #define CTRL_PIN1 PIND4

    #define DDRDCC DDRD
    #define DCC_RX_PIN PIND2 // ext interrupt pin

#endif // ARDUINO_NANO




    #define T_BLINK 500 /// [ms] traffic light blinking period

    /// External Interrupt pins:
    #define DDR_EXTINT DDRD
    #define INT0_PIN PIND2
    #define INT1_PIN PIND3

#endif // DECODER


#ifdef ARDUINO_NANO
    #define PORTDCC PORTD
    #define DDRDCC DDRD
    #define DCC_TX_PIN PIND4 // ext interrupt pin
    #define LEDx_PIN PINB5
#endif // ARDUINO_NANO


#endif // ENCODER

#endif // MAIN_H_INCLUDED
