#ifndef PID_H_INCLUDED
#define PID_H_INCLUDED



uint8_t PID_CMD_Parser(int16_t *setpoint, uint16_t *Kp, uint16_t *Ki, uint16_t *Kd, int16_t *y_i0, uint8_t *mode);


#endif // PID_H_INCLUDED
