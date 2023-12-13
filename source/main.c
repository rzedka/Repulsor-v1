/*
 */

#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
//#include <util/delay.h>
#include <string.h>
#include <avr/interrupt.h>

#include "main.h"
#include "uart.h"
#include "tim.h"
#include "gpio.h" // contains led_toggle() only
#include "adc.h"
#include "pid.h"

/// Global Variables ============================


    volatile uint8_t rx_array[20];
    #ifdef UART_TERM
        volatile uint8_t uart_flag; /// ISR variable
        volatile unsigned char uart_rx_array[50];       /// ISR variable
        volatile uint8_t uart_idx;          /// ISR variable
        volatile uint8_t uart_char_idx;          /// ISR variable
    #endif // UART_TERM

    volatile uint16_t adc_value;
    volatile uint8_t adc_flag;

    volatile uint16_t timer1_cnt; /// driven by TIMER1

    uint8_t  led_flag; /// Use for IDLE process only

/// ====================


int main(void)
{

    uint16_t ref_timer = 0;
    timer1_cnt = 0;
    /// PID variables:
    int16_t setpoint = SET_INIT;
    int16_t error[2] = {0,0};

    uint16_t k_p = KP_INIT;
    uint16_t k_i = KI_INIT;
    uint16_t k_d = KD_INIT;
    int16_t y_p = 0;
    int16_t y_i[3] = {0,0,0};
    int16_t y_d = 0;
    int16_t y_sum = 0;
    uint8_t pwm_value = 0;
    uint8_t mode = 0; /// oscillation mode
    //uint16_t aux_cnt = 0;
    //int8_t osc_step = 1;

    uint8_t adc_flag_f = 0;
    uint8_t adc_cal_flag = 0; /// ADC calibration done after RESET
    uint16_t adc_cal_value = 0;

    #ifdef UART_TERM
        USART_init();
        uint8_t uart_flag_f = 0;
        char buffer [33];
    #endif // UART_TERM

    GPIO_setup();

//    EIMSK |= (1<< INT0);

    TIMER0_PWM_setup(); /// PWM, 7.8125 kHz (driving pin OC0A)
    TIMER1_setup(); /// (timer 1ms, ADC sampling period)
    ADC_setup(); /// Manual ADC triggering, ISR on,

    sei();/// Enable Interrupts
    #ifdef UART_TERM
        /// ============ Init UART message =============================
        USART_TX_STRING_WAIT("==== Repulsor v1 ====\n");

    #endif // UART_TERM
    /// ======================== LOOP ===================
    while(1){


       /// BACKGROUND BLINK PROCESS:
        if( (TIMER1_get_value() - ref_timer) >= T_IDLE){
            ref_timer = TIMER1_get_value();
            LED_toggle(0x01); /// PINB5 (built-in LED)
            #ifdef UART_TERM
               /* USART_TX_STRING_WAIT("err ");
                USART_TX_STRING_WAIT(itoa(error[1],buffer,10));
                USART_TX_WAIT('\n');
                USART_TX_STRING_WAIT("Y_sum ");
                USART_TX_STRING_WAIT(itoa(y_sum,buffer,10));
                USART_TX_WAIT('\n');
                */
                USART_TX_STRING_WAIT("pwm_value ");
                USART_TX_STRING_WAIT(itoa(pwm_value,buffer,10));
                USART_TX_WAIT('\n');
            #endif // UART_TERM
        }

       /// Button debouncer process


       #ifdef UART_TERM
        /// ============ UART Command Reception =============================
        if(uart_flag_f != uart_flag){
            uart_flag_f = uart_flag;
            PID_CMD_Parser(&setpoint, &k_p, &k_i, &k_d, &y_i[0], &mode);/// int16, uint16, uint16, uint16
            USART_TX_STRING_WAIT("set  k_p  k_i  k_d\n");
            USART_TX_STRING_WAIT(itoa(setpoint,buffer,10));
            USART_TX_WAIT(' ');
            USART_TX_STRING_WAIT(itoa(k_p,buffer,10));
            USART_TX_WAIT(' ');
            USART_TX_STRING_WAIT(itoa(k_i,buffer,10));
            USART_TX_WAIT(' ');
            USART_TX_STRING_WAIT(itoa(k_d,buffer,10));
            USART_TX_WAIT('\n');
            //mult_out = setpoint*setpoint;
            //USART_TX_STRING_WAIT("\n Mult_out = ");
            //USART_TX_STRING_WAIT(itoa(mult_out,buffer,10));
        }

        #endif // UART_TERM

       if(adc_flag_f != adc_flag){
           /// AD Conversion completed:
            adc_flag_f = adc_flag;
            if(!adc_cal_flag){/// ADC Offset Removal:
                adc_cal_flag = 1;
                adc_cal_value = adc_value;
                #ifdef UART_TERM
                    USART_TX_STRING_WAIT("ADC offset: ");
                    USART_TX_STRING_WAIT(itoa( adc_cal_value, buffer, 10) );
                    USART_TX_WAIT('\n');
                #endif UART_TERM
            }///end if
            /// P-I-D calculations:
            error[1] = -adc_value + adc_cal_value;
            error[1] = setpoint - error[1];
            #ifdef DEBUG
                #ifdef UART_TERM
                    USART_TX_STRING_WAIT("err ");
                    USART_TX_STRING_WAIT(itoa(error[1],buffer,10));
                    USART_TX_WAIT('\n');
                #endif // UART_TERM
            #endif // DEBUG

            if(error[1] != error[0]){
                /// If something changed, do the calculations
                /// 1) Proportional
                y_p = error[1]*k_p;
                #ifdef DEBUG
                    #ifdef UART_TERM
                        USART_TX_STRING_WAIT("Y_p ");
                        USART_TX_STRING_WAIT(itoa(y_p, buffer, 10));
                        USART_TX_WAIT('\n');
                    #endif // UART_TERM
                #endif // DEBUG

                /// 2) Integral
                y_i[1] = error[1]*k_i + y_i[0];
                if(y_i[1]>0)
                    y_i[0] = y_i[1];
                else
                    y_i[1] = y_i[0];

                /// 3) Differential
                y_d = (error[1]-error[0])*k_d;

                /// 4) SUM
                y_sum = y_p + y_i[1] + y_d;

                if(y_sum < 0){ /// Overflow indication
                    y_sum =0;
                    PORT_LED |= (1<<LED_OVF_PIN);
                }else{
                    PORT_LED &= ~(1<<LED_OVF_PIN);
                }

                #ifdef DEBUG
                    #ifdef UART_TERM
                        USART_TX_STRING_WAIT("Y_sum ");
                        USART_TX_STRING_WAIT(itoa(y_sum,buffer,10));
                        USART_TX_WAIT('\n');
                    #endif // UART_TERM
                #endif // DEBUG
                //pwm_value = (y_sum>>8)&0x7F; /// Truncate to 7-bit width
                pwm_value = (y_sum>>7); /// 8-bit width
                //pwm_value = 127;
                #ifdef DEBUG
                    #ifdef UART_TERM
                        USART_TX_STRING_WAIT("pwm_value ");
                        USART_TX_STRING_WAIT(itoa(pwm_value,buffer,10));
                        USART_TX_WAIT('\n');
                    #endif // UART_TERM
                #endif // DEBUG

               TIMER0_PWM_update(pwm_value);


            } /// end if
            error[0] = error[1];
       }
    }
    return 0;
}

#ifdef UART_TERM

ISR(USART_RX_vect) /// ====================== UART DATA RECEPTION ===============================================
{ /// UART RX complete Interrupt:
    cli();

    /// UART TERMINAL SETTINGS:
    /// - every message must be terminated with CR+LF (0x0D 0x0A)

    /// All the variables
    if(uart_char_idx > 99){
        uart_char_idx=0; /// Start overwriting the beginning
        rx_array[uart_char_idx] = UDR0; /// read UART buffer
    }else if(uart_char_idx == 0){
        memset(rx_array,'\0',20);
        rx_array[uart_char_idx] = UDR0; /// read UART buffer
        uart_char_idx++;
        //PORTLED |= (1<<LED0_PIN);
    }else{
        rx_array[uart_char_idx] = UDR0; /// read UART buffer
        if(rx_array[uart_char_idx] == 0x0A){ /// end of the CMD (CR char)
            rx_array[uart_char_idx] = 0x00; // erase 0x0A
            rx_array[--uart_char_idx] = 0x00; // erase 0x0D
            uart_flag ++; /// this variable increments each ISR. It is followed by another variable in the loop.
            /// The change of "uart_flag" triggers Command recognition procedure.
            /// However, "uart_flag" can never be modified outside this ISR! It can only be read.
            uart_char_idx = 0;
        }else{
            uart_char_idx++;
        }
    }
    sei();
}
#endif // UART_TERM


