#include "motor.h"
void Motor_PWM_Init(u16 arr,u16 psc)
{		 					 
	RCC->APB2ENR|=1<<4;       //PORTCʱ��ʹ��   
	RCC->APB2ENR|=1<<3;       //PORTBʱ��ʹ�� 
	RCC->APB2ENR|=1<<2;       //PORTAʱ��ʹ�� 
	
	GPIOA->CRL&=0XFF00FFFF;   // 
	GPIOA->CRL|=0X00220000;   // PA4 5�������
	
	GPIOC->CRL&=0XFFF0000F;   // 
	GPIOC->CRL|=0X00022220;   // PC1 2 3 4�������

	GPIOB->CRL&=0XFFFFFFF0;   // PB0���ԭ����
	GPIOB->CRL|=0X00000002;   // PB0�������
	
	GPIOB->CRH&=0X0FFFFFFF;   // PB15���ԭ����
	GPIOB->CRH|=0X20000000;   // PB15�������
		
	
	INA1=0;
	INB1=0;
	INC1=0;
	IND1=0;

	INA2=0;
	INB2=0;
	INC2=0;
	IND2=0;
	
	RCC->APB2ENR|=1<<13;       //ʹ��TIM8ʱ��
	RCC->APB2ENR|=1<<4;        //PORTCʱ��ʹ��
	
	GPIOC->CRL&=0X00FFFFFF;    //PORTC 6 7  �������
	GPIOC->CRL|=0XBB000000;    //PORTC 6 7  �������
	GPIOC->CRH&=0XFFFFFF00;    //PORTC 8 9 �������
	GPIOC->CRH|=0X000000BB;    //PORTC 8 9 �������
		
	TIM8->ARR=arr;             //�趨�������Զ���װֵ 
	TIM8->PSC=psc;             //Ԥ��Ƶ������Ƶ
			
	TIM8->CCMR1|=6<<4;         //CH1 PWM1ģʽ	
	TIM8->CCMR1|=6<<12;        //CH2 PWM1ģʽ	
	TIM8->CCMR2|=6<<4;         //CH3 PWM1ģʽ	
	TIM8->CCMR2|=6<<12;        //CH4 PWM1ģʽ	
	
	TIM8->CCMR1|=1<<3;         //CH1Ԥװ��ʹ��	  
	TIM8->CCMR1|=1<<11;        //CH2Ԥװ��ʹ��	 
	TIM8->CCMR2|=1<<3;         //CH3Ԥװ��ʹ��  
	TIM8->CCMR2|=1<<11;        //CH4Ԥװ��ʹ��	  
	
	TIM8->CCER|=1<<0;         //CH1���ʹ��	
 	TIM8->CCER|=1<<4;         //CH2���ʹ��	   
	TIM8->CCER|=1<<8;         //CH3���ʹ��	 
	TIM8->CCER|=1<<12;        //CH4���ʹ��
	
	TIM8->CCR1=0;
  TIM8->CCR2=0;
	TIM8->CCR3=0;
	TIM8->CCR4=0;
	TIM8->BDTR |= 1<<15;       //TIM1����Ҫ��仰�������PWM
	TIM8->CR1=0x0080;          //ARPEʹ�� 
	TIM8->CR1|=0x01;          //ʹ�ܶ�ʱ��	
	
} 
void Steering_engine_PWM_Init(u16 arr,u16 psc)
{		 					 
	RCC->APB2ENR|=1<<11;       //ʹ��TIM1ʱ��
	RCC->APB2ENR|=1<<2;        //PORTAʱ��ʹ��
	GPIOA->CRH&=0XFFFF0FF0;    //PORTA8 11�������
	GPIOA->CRH|=0X0000B00B;    //PORTA8 11�������
		
	TIM1->ARR=arr;             //�趨�������Զ���װֵ 
	TIM1->PSC=psc;             //Ԥ��Ƶ������Ƶ
			
	TIM1->CCMR1|=6<<4;         //CH1 PWM2ģʽ	
	
	TIM1->CCMR2|=6<<12;        //CH4 PWM2ģʽ	
	
	TIM1->CCMR1|=1<<3;         //CH1Ԥװ��ʹ��	  
	  
	TIM1->CCMR2|=1<<11;        //CH4Ԥװ��ʹ��	  
	
	TIM1->CCER|=1<<0;         //CH1���ʹ��	
 
	TIM1->CCER|=1<<12;        //CH4���ʹ��
	TIM1->CCR1=0;

	TIM1->CCR4=0;
	TIM1->BDTR |= 1<<15;       //TIM1����Ҫ��仰�������PWM
	TIM1->CR1=0x0080;          //ARPEʹ�� 
	TIM1->CR1|=0x01;          //ʹ�ܶ�ʱ��					
} 
