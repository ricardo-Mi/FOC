#ifndef PID_H
#define PID_H

#include <stdint.h>

int16_t Left_PID_Calculate(int left_actual_speed,int left_target_speed);                     
int16_t Right_PID_Calculate(int right_actual_speed,int right_target_speed);                     


#endif /* PID_H */
