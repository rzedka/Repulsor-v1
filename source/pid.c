#include "pid.h"


uint8_t PID_CMD_Parser(int16_t *setpoint, uint16_t *Kp, uint16_t *Ki, uint16_t *Kd, int16_t *y_i0, uint8_t *mode)
{
    /// Function parses a command from UART.
    /// The command looks like: "CMDHEAD_Value"

    char CMD_Head[5] = {0,0,0,0,0};
    char CMD_Data[15];
    //uint8_t cmd_num = 0;
    uint8_t i = 0;
    memset(CMD_Head,'\0',5);
    memset(CMD_Data,'\0',15);
    /// 1) Separate the CMD Head (4-character) and CMD_Data:

    while(rx_array[i] != '\0'){
        if(i<4){
            CMD_Head[i] = rx_array[i];
        }else{
            CMD_Data[i-4] = rx_array[i];
        }
        i++;
    }

    USART_TX_STRING_WAIT(CMD_Head);
    USART_TX_WAIT('\n');
    //USART_TX_STRING_WAIT(CMD_Data);
    if(!strcmp(CMD_Head,"ACC0")){
        *y_i0 = 0;
    }else if(!strcmp(CMD_Head,"SET_")){ /// Set the Setpoint
        *setpoint = atoi(CMD_Data);
    }else if(!strcmp(CMD_Head,"KP__")){ ///
        *Kp = atoi(CMD_Data);
    }else if(!strcmp(CMD_Head,"KD__")){ ///
        *Kd = atoi(CMD_Data);
    }else if(!strcmp(CMD_Head,"KI__")){ ///
        *Ki = atoi(CMD_Data);
    }else if(!strcmp(CMD_Head,"OSCM")){ /// Toggle oscillating mode
        if(*mode == 0)
            *mode = 1;
        else
            *mode = 0;

    }else{ /// Unknown Command

    }// end if
return 0;
}
