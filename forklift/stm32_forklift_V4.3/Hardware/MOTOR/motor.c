#include "motor.h"

void Motor_PWM_Init(u16 arr,u16 psc)
{		

	
	RCC->APB1ENR|=1<<1;       //TIM3ʱ��ʹ��    
	RCC->APB2ENR|=1<<2;       //PORTAʱ��ʹ�� 
	RCC->APB2ENR|=1<<3;       //PORTBʱ��ʹ�� 
	GPIOA->CRL&=0X00FFFFFF;   //PORTA6 7�������
	GPIOA->CRL|=0XBB000000;   //PORTB6 7�������
	GPIOB->CRL&=0XFFFFFF00;   //PORTB0 1�������
	GPIOB->CRL|=0X000000BB;   //PORTB0 1�������
	TIM3->ARR=arr;//�趨�������Զ���װֵ 
	TIM3->PSC=psc;//Ԥ��Ƶ������Ƶ
	TIM3->CCMR1|=6<<4;         //CH1 PWM1ģʽ	
	TIM3->CCMR1|=6<<12;        //CH2 PWM1ģʽ	
	TIM3->CCMR2|=6<<4;         //CH3 PWM1ģʽ	
	TIM3->CCMR2|=6<<12;        //CH4 PWM1ģʽ	
	
	TIM3->CCMR1|=1<<3;         //CH1Ԥװ��ʹ��	  
	TIM3->CCMR1|=1<<11;        //CH2Ԥװ��ʹ��	 
	TIM3->CCMR2|=1<<3;         //CH3Ԥװ��ʹ��	  
	TIM3->CCMR2|=1<<11;        //CH4Ԥװ��ʹ��	  
	
	TIM3->CCER|=1<<0;         //CH1���ʹ��	
	TIM3->CCER|=1<<4;         //CH2���ʹ��	   
	TIM3->CCER|=1<<8;         //CH3���ʹ��	 
	TIM3->CCER|=1<<12;        //CH4���ʹ��

	TIM3->CR1=0x0080;          //ARPEʹ�� 
	TIM3->CR1|=0x01;          //ʹ�ܶ�ʱ��3 	
} 

