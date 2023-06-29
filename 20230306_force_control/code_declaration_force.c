#include <float.h>
#include <stdio.h>

#define theta Input(0)   //angle
#define irq Input(1)   //current
#define PWM_AH Output(0)  //switch output
#define PWM_AL Output(1)  //switch output
#define PWM_BH Output(2)  //switch output
#define PWM_BL Output(3)  //switch output
#define PWM_CH Output(4)  //switch output
#define PWM_CL Output(5)  //switch output

int hall_position = 0;