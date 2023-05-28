#include "gpio.h"

void GPIO_setup(void)
{
    DDR_LED |= (1<<LED_IDLE_PIN);   // output
    PORT_LED &= ~(1<<LED_IDLE_PIN); // LED_IDLE OFF

    DDR_LED |= (1<<LED_OVF_PIN);   // output
    PORT_LED &= ~(1<<LED_OVF_PIN); // LED_OVF OFF

    DDR_BTN &= ~(1<<PIN_BTN0);   // input
    PORT_BTN |= (1<<PIN_BTN0); // internal pull-up resistor

    ///  The MOSFET is at D4 (OC0A, TIMER0-driven pin)
    DDR_FET |= (1<<PIN_FET0);
//    PORT_FET &= ~(1<<PIN_FET0); // FET0 OFF

    led_flag = 0;
}



void LED_toggle(uint8_t led_bit)
{   /// led_bit = 0x01, 0x02, 0x04, 0x08, ...
    uint8_t bitshift = 0;

    switch(led_bit){
    case 0x01: /// LED0
            bitshift = LED_IDLE_PIN;
        break;
//    case 0x02: /// LED1
//            bitshift = LEDy_PIN;
//        break;
//    case 0x04: /// IDLE process LED
//            bitshift = LEDx_PIN;
//        break;
    default:   /// LED0
            bitshift = LED_IDLE_PIN;

    }// end switch

    if(led_flag&led_bit){
        led_flag &= ~led_bit;
        PORT_LED &= ~(1<<bitshift);
    }else{ /// The LED is off
        led_flag |= led_bit;
        PORT_LED |= (1<<bitshift);
    }
}
