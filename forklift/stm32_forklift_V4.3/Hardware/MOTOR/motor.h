#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>	 
#define PWMA   TIM3->CCR1 //A���PWM
#define PWMB   TIM3->CCR2 //A���PWM
#define PWMC   TIM3->CCR3 //B���PWM
#define PWMD   TIM3->CCR4 //B���PWM

void Motor_PWM_Init(u16 arr,u16 psc);
#endif
