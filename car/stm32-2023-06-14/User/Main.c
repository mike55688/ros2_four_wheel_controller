#include "sys.h"
/*********************************************
���ߣ���������
�Ա����̣�http://shop180997663.taobao.com/
*********************************************/
/*******************
�Եѿ�������ϵΪ�ο�
RollΪ��X����ת�ǣ�
PitchΪ��Y����ת�ǣ�
YawΪ��Z����ת�ǡ�
*******************/
int velocityA_Control(int encoder,int target);
int velocityB_Control(int encoder,int target);
int velocityC_Control(int encoder,int target);
int velocityD_Control(int encoder,int target);

void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d);
void Abnormal_state_handling(void);
void  APP_Control(void);//APPָ��
void  PS2_Control(void);//�ֱ�ָ��
void  Key(void);//����ָ��
void Motion_control(void);
void Usart_rt_data(void);
int Go_straight(float Angle,float Target);
void  send_data_to_blue(void);
u8  Start_Flag=0;                    //������־
u8  Start_Flag_one=1;                //������־
u8  Start_Flag_STOP=0;               //������־
u8  show_flag=0;                     //��ʾ��־
u8  delay_30,delay_flag;             //30�����־

u8 flag_move=0;//
u8 flag_action=0;//

u8 PS2_KEY,PS2_LX,PS2_LY,PS2_RX,PS2_RY;
 
u16 Flag_Z1 ,Flag_Z2 ,Flag_Z3  ;//��־λ

u16 FLAG_USART_ON,Count_stop;

int Voltage;//��ص�ѹ
int count_size,count_sum;
int Motor_A,Motor_B,Motor_C,Motor_D;         //���PWM����
int Last_Motor_A,Last_Motor_B,Last_Motor_C,Last_Motor_D;         //���PWM����

int Velocity_center=300;//�趨�����ٶȣ�Ĭ���ٶ�Ϊ150
float Velocity_target_A;
float Velocity_target_B;
float Velocity_target_C;
float Velocity_target_D;

float Encoder_A,Encoder_B,Encoder_C,Encoder_D;   //��������������� 

float Roll,Pitch,Yaw,gyro_Roll,gyro_Pitch,gyro_Yaw ,accel_x,accel_y,accel_z;//XYZ������ת�Ƕȡ����ٶ�

float Velocity_A;		
float Velocity_B;
float Velocity_C;
float Velocity_D;

float Yaw_target;

float line_KP=15,line_KD=25; //ֱ�����߿���PD����

float Velocity_B_KP=1.5,Velocity_B_KI=10;         //���B�ٶȿ���PI����


int main(void)
{ 
	Stm32_Clock_Init(9);            //ϵͳʱ������
	delay_init(72);                 //��ʱ��ʼ��
	JTAG_Set(JTAG_SWD_DISABLE);     //�ر�JTAG�ӿ�
	JTAG_Set(SWD_ENABLE);           //��SWD�ӿ� �������������SWD�ӿڵ���
	
	Motor_PWM_Init(7199,0);         //��ʼ�����PWM 10KHZ 
	
	OLED_Init();                    //OLED��ʼ��
	LED_Init();                     //LED��ʼ��
	KEY_Init();                     //������ʼ��
	usart1_init(72,115200);       //����1��ʼ��
	usart2_init(36,9600);           //����2��ʼ��

  delay_ms(100);
	Dma_Init(1,DMA1_Channel4,(u32)&USART1->DR,(u32)send_buf,65);//��ʼ��DMA1��(ͨ��4������Դsend_buf[]������Ŀ�꣺�����ַ &USART1->DR	������������),��ѭ��ģʽ	 
	USART1->CR3|=1<<7;                 //����1DMA�ж�ʹ��
 	Dma_Enable(DMA1_Channel4,65);      //DMA����ʹ�ܣ�(ͨ��4 ,����������)
	delay_ms(20);                   //��ʱ
	
	Encoder_Init_TIM2();            //��ʼ��������
	Encoder_Init_TIM3();            //��ʼ��������
	Encoder_Init_TIM4();            //��ʼ��������
	Encoder_Init_TIM5();            //��ʼ��������
	
	Adc_Init();                     //adc��ʼ��	
	delay_ms(100);                  //��ʱ

	IIC_Init();                     //IIC��ʼ��
  delay_ms(10);                   //��ʱ		
  MPU6050_initialize();           //MPU6050��ʼ��	
  delay_ms(10);                   //��ʱ	
  DMP_Init();                     //��ʼ��DMP
  delay_ms(400);                  //��ʱ
	PS2_Init();                     //PS2�ֱ���ʼ��
	PS2_SetInit();                  //PS2�ֱ���ʼ��	
	

	delay_ms(100);                  //��ʱ		
  EXTI_Init();                    //MPU6050 5ms��ʱ�жϳ�ʼ��
	while(1)
		{		

			  show_flag++;                  //��OLED����λ��
			  if(show_flag==1) send_data_to_blue();	
				if(show_flag>=5) oled_show();          //oled��ʾ 
			  if(show_flag==7)show_flag=0;
		
			  delay_flag=1;	                         //30ms�жϱ�־λ
				while(delay_flag);                     //������ѭ�����ȴ�5�����жϽ� delay_flag��־λ��0������    		
		} 
}

/**************************************************************************
MPU6050 INT����5�����ж�	 
**************************************************************************/
int EXTI15_10_IRQHandler(void) 
{    
	 if(INT==0)		
	{     
		EXTI->PR=1<<12; //���LINE5�ϵ��жϱ�־λ  
		
		 //****************���������ݻ�ȡ****************************//
		 Encoder_A = (float)Read_Encoder(5);//��ȡ��������ֵ
		 Encoder_B = (float)Read_Encoder(3);//��ȡ��������ֵ	
		 Encoder_C = (float)Read_Encoder(2);//��ȡ��������ֵ
		 Encoder_D = (float)Read_Encoder(4);//��ȡ��������ֵ			
     Read_DMP();//��ȡ�Ƕ���Ϣ

		 Usart_rt_data();	//���������շ�����
				
		//*******************ָ�����*****************//
		 Key();//�������		
		 APP_Control();//APP����ָ��
		 PS2_Control();//PS2ָ��
		
		 Motion_control();//�˶���̬���� 
 		
		 if(Start_Flag==1)
		   {			 
			  Motor_A = +velocityA_Control(+Encoder_A, +Velocity_target_A);
			  Motor_B = +velocityB_Control(+Encoder_B, -Velocity_target_B);
			  Motor_C = +velocityC_Control(+Encoder_C, -Velocity_target_C);				 
			  Motor_D = +velocityD_Control(+Encoder_D, +Velocity_target_D);
			}
		 if(Start_Flag==0)
			 {
          velocityA_Control(0,0);		
          velocityB_Control(0,0);		
          velocityC_Control(0,0);						 
          velocityD_Control(0,0);			 
		   } 
		 
//�����������������������������������������������������������������������������������쳣״̬�رյ���������쳣�������쳣��ͨѶ�쳣����������������������������������������������������������������������������������//
			
 			Abnormal_state_handling();
		
//�����������������������������������������������������������������������������������쳣״̬�رյ���������쳣�������쳣��ͨѶ�쳣����������������������������������������������������������������������������������//	
				
		
    //****************�������****************************// 		 					
		 if(Start_Flag==1&& Start_Flag_one==1&& Start_Flag_STOP==0)
		   {			
			  Set_Pwm(Motor_A,Motor_B,Motor_C,Motor_D);	//PWM��ֵ
		   }
		 else //�رյ����������ֲ���״̬
			 {
				Motor_A=0;Motor_B=0;
				Motor_C=0;Motor_D=0;				 
		    Set_Pwm(0,0,0,0);
				Yaw_target= Yaw;					 
			 }
 			       					
//************************************����**************************************************//	
    if(Voltage<1160&&Voltage>600)Led_Flash(20);//��������ʱLED�ƿ�����˸
		else if(Voltage<2290&&Voltage>1800)Led_Flash(20);//��������ʱLED�ƿ�����˸	 
	  else Led_Flash(100);//led��˸ 
		count_sum+=Get_battery_voltage();//��ѹ�����ۼ�
	  count_size++;
		if(count_size==200) Voltage=count_sum/count_size,count_sum=0,count_size=0;//��ƽ����ѹ 
	 
		if(delay_flag==1)
		  {
			 if(++delay_30==2)delay_30=0,delay_flag=0;  //���������ṩ30ms�ľ�׼��ʱ
		  }
			
 }
	 return 0;	 
} 

void Usart_rt_data(void) 
{
	//**********************************����ͨѶ�շ�********************************************************************************//

         float A_Velocity,B_Velocity,C_Velocity,D_Velocity;		 
			   A_Velocity = (float)((Encoder_A/16384.f)/19.f)*200.f *0.797f;//�ٶ�m/s =����ǰ������/һȦ������/���ٱȣ�*Ƶ��*�����ܳ�m
			   B_Velocity = (float)((Encoder_B/16384.f)/19.f)*200.f *0.797f;//�ٶ�m/s =����ǰ������/һȦ������/���ٱȣ�*Ƶ��*�����ܳ�m
				 C_Velocity = (float)((Encoder_C/16384.f)/19.f)*200.f *0.797f;//�ٶ�m/s =����ǰ������/һȦ������/���ٱȣ�*Ƶ��*�����ܳ�m
				 D_Velocity = (float)((Encoder_D/16384.f)/19.f)*200.f *0.797f;//�ٶ�m/s =����ǰ������/һȦ������/���ٱȣ�*Ƶ��*�����ܳ�m

							          /*<01>*/data_u[0]  = Start_Flag;     //����������أ�1���� 0ֹͣ
							          /*<02>*/data_u[1]  = gyro_Roll ;     //X����ٶ�ԭʼ��ֵ
							          /*<03>*/data_u[2]  = gyro_Pitch ;    //Y����ٶ�ԭʼ��ֵ 
							          /*<04>*/data_u[3]  = gyro_Yaw ;      //Z����ٶ�ԭʼ��ֵ
							          /*<05>*/data_u[4]  = accel_x ;       //X����ٶ�ԭʼ��ֵ
							          /*<06>*/data_u[5]  = accel_y ;       //Y����ٶ�ԭʼ��ֵ
							          /*<07>*/data_u[6]  = accel_z ;       //Z����ٶ�ԭʼ��ֵ				 
							          /*<08>*/data_u[7]  = Yaw ;           //Z��Ƕ�
							          /*<09>*/data_u[8]  = A_Velocity ;    //A���ٶ�
							          /*<10>*/data_u[9]  = B_Velocity ;    //B���ٶ�			 
							          /*<11>*/data_u[10] = C_Velocity ;    //C���ٶ�
							          /*<12>*/data_u[11] = D_Velocity ;    //D���ٶ�	 
							          /*<13>*/data_u[12] = Voltage ;       //��ѹֵ*100
							          /*<14>*/data_u[13] = 0 ;             //Ԥ��
							          /*<15>*/data_u[14] = 0 ;             //Ԥ��			 
				     if(DMA1->ISR&(1<<13))//ͨ��4������ɱ�־(��ͨ����-1��*4+1)
			       {				 
			        Dma_Disenable(DMA1_Channel4);//DMA���˳��ܣ�ͨ��2 	

              Send_data_ROS();//����15������

								//��������				
			        DMA1->IFCR|=1<<13;//���ͨ��4������ɱ�־
			        Dma_Enable(DMA1_Channel4,65);//DMA����ʹ�ܣ�ͨ��1                    
			       }						 
	   if(FLAG_USART==1)//������ڱ�־λ��1����ʾ�����һ�����ݴ���
	     { 		u8 sum=0;		 				 						 
						for(u8 j=0;j<52;j++)sum+=USART1_data[j];//����У���	
						if(sum==USART1_data[52]){
														
            //���㴮�ڽ��յ������ݣ���Ϊ������̺�ʱ�Ƚϳ��������ж�������Ӱ��ʱ��						 
						   Start_Flag    =   b2f(USART1_data[ 4], USART1_data[ 5], USART1_data[ 6], USART1_data[ 7]); //��ȡ����λ
						   Velocity_A    =   b2f(USART1_data[ 8], USART1_data[ 9], USART1_data[10], USART1_data[11]);//��ȡA���ٶ�
					     Velocity_B    =   b2f(USART1_data[12], USART1_data[13], USART1_data[14], USART1_data[15]);//��ȡB���ٶ�
						   Velocity_C    =   b2f(USART1_data[16], USART1_data[17], USART1_data[18], USART1_data[19]);//��ȡC���ٶ�
					     Velocity_D    =   b2f(USART1_data[20], USART1_data[21], USART1_data[22], USART1_data[23]);//��ȡD���ٶ�							

						}               
            USART1_data[52]	=0XFF;								
            i_u=0; 
						sum=0; 																														
				    FLAG_USART_ON=0;
					  FLAG_USART=0;//���ڱ�־λ��0			 
			 }
		 
}

void Abnormal_state_handling(void){//�쳣״̬����

	    //****************����쳣����****************************// 		
    if(
			  (Motor_A>=+7100 && Encoder_A<-100)||(Motor_A<=-7100 && Encoder_A>+100)|| 
  		  (Motor_B>=+7100 && Encoder_B<-100)||(Motor_B<=-7100 && Encoder_B>+100)||  
		    (Motor_C>=+7100 && Encoder_C<-100)||(Motor_C<=-7100 && Encoder_C>+100)||  
		    (Motor_D>=+7100 && Encoder_D<-100)||(Motor_D<=-7100 && Encoder_D>+100)
		  )Flag_Z1++;
		else Flag_Z1=0;
		if(Flag_Z1>300)Start_Flag_one=0;//����״̬����1.5���ж�����������ʧЧ������߻�������߽ӷ�����ֹͣ�������Ҫ�������ز��ܽ��
			
	 	if(
 			  (abs(Motor_A)>=+7100 &&fabs(Encoder_A)<10)||
			  (abs(Motor_B)>=+7100 &&fabs(Encoder_B)<10)||
		    (abs(Motor_C)>=+7100 &&fabs(Encoder_C)<10)||
		    (abs(Motor_D)>=+7100 &&fabs(Encoder_D)<10)
			)Flag_Z2++;
		else Flag_Z2=0;
		if(Flag_Z2>1000)Start_Flag_one=0;//����״̬����5���ж�Ϊ�����ת����������𻵣���ֹͣ�������Ҫ�������ز��ܽ��	
		
	    //****************��ѹ�쳣����****************************//
    int all_motor_abs;
    all_motor_abs =	abs(Motor_A) + abs(Motor_B) + abs(Motor_C) + abs(Motor_D);	
         if(Voltage<2220 && all_motor_abs<8000)Flag_Z3++;
    else if(Voltage<2150 && all_motor_abs<12000 && all_motor_abs>=8000)Flag_Z3++;
    else if(Voltage<2100 && all_motor_abs<18000 && all_motor_abs>=12000)Flag_Z3++;
    else if(Voltage<2000 && all_motor_abs<29000 && all_motor_abs>=20000)Flag_Z3++;
    else if(Voltage<1700 && all_motor_abs<29000 && all_motor_abs>=20000)Flag_Z3+=20;		
 		else Flag_Z3=0; 
	  if(Flag_Z3>4000)Start_Flag_one=0;//����״̬����20���ж�Ϊ��ѹ�쳣��ֹͣ���������Ҫ�������ز��ܽ��
		
    if(Voltage<700)Start_Flag_STOP=1;//����״̬�ж�Ϊδ��ͨ���ֻ��USB�ڹ���������������������Σ�յģ�����ֹͣ
    else Start_Flag_STOP=0;	

    //****************ͨѶ�쳣����****************************//

  	if(FLAG_USART_ON<600)FLAG_USART_ON++;//�ڴ������ݴ����������㣬������ֳ���ĳ����ֵ֤��û�н��յ�����Э��Ĵ�������	
		
//����������������������������������������������������������������������������������PWM���ƣ���ֹ�ջ������������������������������������������������������������������������������//	
			
		     if((Motor_A - Last_Motor_A) > +500) Motor_A = Last_Motor_A + 500;//����PWM�ı�������
		else if((Motor_A - Last_Motor_A) < -500) Motor_A = Last_Motor_A - 500;//����PWM�ı�������
		   Last_Motor_A = Motor_A;
		     if((Motor_B - Last_Motor_B) > +500) Motor_B = Last_Motor_B + 500;//����PWM�ı�������
		else if((Motor_B - Last_Motor_B) < -500) Motor_B = Last_Motor_B - 500;//����PWM�ı�������
		   Last_Motor_B = Motor_B;
		     if((Motor_C - Last_Motor_C) > +500) Motor_C = Last_Motor_C + 500;//����PWM�ı�������
		else if((Motor_C - Last_Motor_C) < -500) Motor_C = Last_Motor_C - 500;//����PWM�ı�������
		   Last_Motor_C = Motor_C;	
		     if((Motor_D - Last_Motor_D) > +500) Motor_D = Last_Motor_D + 500;//����PWM�ı�������
		else if((Motor_D - Last_Motor_D) < -500) Motor_D = Last_Motor_D - 500;//����PWM�ı�������
		   Last_Motor_D = Motor_D;				 
			 		
		if(Motor_A>+7100)Motor_A=+7100;//PWM�޷�
	  if(Motor_A<-7100)Motor_A=-7100;//PWM�޷�
		
		if(Motor_B>+7100)Motor_B=+7100;//PWM�޷�
	  if(Motor_B<-7100)Motor_B=-7100;//PWM�޷�
		
		if(Motor_C>+7100)Motor_C=+7100;//PWM�޷�
	  if(Motor_C<-7100)Motor_C=-7100;//PWM�޷�	
		
		if(Motor_D>+7100)Motor_D=+7100;//PWM�޷�
	  if(Motor_D<-7100)Motor_D=-7100;//PWM�޷�
			
		if(RED_KEY==0)Start_Flag=0;//�������أ����£���ֹͣ		
}
/**************************************************************************
����ָ��
**************************************************************************/
void Key(void)
{	
static u8 flag=0;
flag=click_N_Double(70);	
if(flag==1&&Start_Flag==0)Start_Flag=1,anjian_app=0,flag=0; //����һ��������־λ��ת
if(flag==1&&Start_Flag==1)Start_Flag=0,anjian_app=0,flag=0; //����һ��������־λ��ת	
	
}
/**************************************************************************
PS2�ֱ�ָ��
**************************************************************************/
void  PS2_Control(void)
{	
	
static int	I=0;
	    if(I++==10)
			{

				PS2_KEY=PS2_DataKey();//�ɼ�PS2����
			  PS2_LX=PS2_AnologData(PSS_LX);//�ɼ�PS2��ҡ��X��
			  PS2_LY=PS2_AnologData(PSS_LY);//�ɼ�PS2��ҡ��Y��
			  PS2_RX=PS2_AnologData(PSS_RX);//�ɼ�PS2��ҡ��X��
			  PS2_RY=PS2_AnologData(PSS_RY);//�ɼ�PS2��ҡ��Y��
				I=0;
			}
			     
                if(PS2_KEY==13)Start_Flag=1;//������������
			     else if(PS2_KEY==15)Start_Flag=0;//��������ֹͣ
			     else if(PS2_KEY==11)Velocity_center=300;//��ʳָ��ť1
			     else if(PS2_KEY==9) Velocity_center=400;//��ʳָ��ť2
			     else if(PS2_KEY==12)Velocity_center=800;//��ʳָ��ť1
			     else if(PS2_KEY==10)Velocity_center=1600;//��ʳָ��ť2
	

		      if(PS2_LY<88  && PS2_LX>88 &&PS2_LX<168 && PS2_RX>88 &&PS2_RX<168 && PS2_RY>88 &&PS2_RY<168)flag_move = 1;//ǰ��
		 else if(PS2_LY>168 && PS2_LX>88 &&PS2_LX<168 && PS2_RX>88 &&PS2_RX<168 && PS2_RY>88 &&PS2_RY<168)flag_move = 2;//����
//		 else if(PS2_LX<88  && PS2_LY>88 &&PS2_LY<168 && PS2_RX>88 &&PS2_RX<168 && PS2_RY>88 &&PS2_RY<168)flag_move = 7;//�����
//		 else if(PS2_LX>168 && PS2_LY>88 &&PS2_LY<168 && PS2_RX>88 &&PS2_RX<168 && PS2_RY>88 &&PS2_RY<168)flag_move = 8;//�Һ���
		 else if(PS2_LX>88 &&PS2_LX<168 && PS2_LY>88 &&PS2_LY<168 && PS2_RX<10  && PS2_RY>88 &&PS2_RY<168)flag_move = 7;//(��ҡ����)����ת			
     else if(PS2_LX>88 &&PS2_LX<168 && PS2_LY>88 &&PS2_LY<168 && PS2_RX>245 && PS2_RY>88 &&PS2_RY<168)flag_move = 8;//(��ҡ���Ҳ�)����ת
			
		 else if(PS2_LX<88  && PS2_LY<88  && PS2_RX>88 &&PS2_RX<168 && PS2_RY>88 &&PS2_RY<168)flag_move = 3;//��б��
		 else if(PS2_LX>168 && PS2_LY>168 && PS2_RX>88 &&PS2_RX<168 && PS2_RY>88 &&PS2_RY<168)flag_move = 6;//��б��	
			
		 else if(PS2_LX>168 && PS2_LY<88 && PS2_RX>88 &&PS2_RX<168 && PS2_RY>88 &&PS2_RY<168)flag_move = 4;//��б��
		 else if(PS2_LX<88 && PS2_LY>168 && PS2_RX>88 &&PS2_RX<168 && PS2_RY>88 &&PS2_RY<168)flag_move = 5;//��б��
			
     else if(PS2_LX>88 &&PS2_LX<168 && PS2_LY>88 &&PS2_LY<168 && PS2_RX>88 &&PS2_RX<168 && PS2_RY>88 &&PS2_RY<168)flag_move = 0;//ֹͣ
			
}
/**************************************************************************
APP����ָ��
**************************************************************************/
void  APP_Control(void)
{
      if(anjian_app==1)Start_Flag=1;
 else if(anjian_app==2)Start_Flag=0;	
       if(anjian_app==5)Velocity_center=300;
  else if(anjian_app==6)Velocity_center=400;	
  else if(anjian_app==7)Velocity_center=800;	
  else if(anjian_app==8)Velocity_center=1600;	
	
	        if(yaogan_app >=250 && yaogan_app <290)flag_move = 1;//ǰ��
	   else if(yaogan_app >=70  && yaogan_app <110)flag_move = 2;//����
	
	   else if(yaogan_app >=205 && yaogan_app <245)flag_move = 3;//��б��
	   else if(yaogan_app >=295 && yaogan_app <335)flag_move = 4;//��б��
	
	   else if(yaogan_app >=115 && yaogan_app <155)flag_move = 5;//��б��
	   else if(yaogan_app >=25  && yaogan_app <65 )flag_move = 6;//��б��
	
		 else if(yaogan_app >=160 && yaogan_app <200)flag_move = 7;//����ת
		 else if( (yaogan_app >=0 && yaogan_app <20) || 
			   (yaogan_app >=340 && yaogan_app <=360) )flag_move = 8;//����ת	
		 else flag_move = 0;
}
/****************************************************************************************
�˶�����
****************************************************************************************/
void Motion_control(void)
{
static int Velocity_mid=300;
if(Velocity_mid < Velocity_center)Velocity_mid+=2;
if(Velocity_mid > Velocity_center)Velocity_mid-=4;	
if(Start_Flag ==0||flag_move ==0)Velocity_mid=300;	
	
	if(FLAG_USART_ON>300){
     if(flag_move==1)Velocity_target_A = Velocity_target_B = Velocity_target_C = Velocity_target_D = + Velocity_mid;
     if(flag_move==2)Velocity_target_A = Velocity_target_B = Velocity_target_C = Velocity_target_D = - Velocity_mid;
		
     if(flag_move==3)Velocity_target_A = + Velocity_mid,
			               Velocity_target_B = + 0.7F*Velocity_mid,
		                 Velocity_target_C = + 0.5F*Velocity_mid,
			               Velocity_target_D = + 0.95F*Velocity_mid;
		
     if(flag_move==4)Velocity_target_B = + Velocity_mid,
			               Velocity_target_A = + 0.7F*Velocity_mid,
		                 Velocity_target_D = + 0.5F*Velocity_mid,
			               Velocity_target_C = + 0.95F*Velocity_mid;

     if(flag_move==5)Velocity_target_A = - Velocity_mid,
			               Velocity_target_B = - 0.7F*Velocity_mid,
		                 Velocity_target_C = - 0.5F*Velocity_mid,
			               Velocity_target_D = - 0.95F*Velocity_mid;
		
     if(flag_move==6)Velocity_target_B = - Velocity_mid,
			               Velocity_target_A = - 0.7F*Velocity_mid,
		                 Velocity_target_D = - 0.5F*Velocity_mid,
			               Velocity_target_C = - 0.95F*Velocity_mid;

     if(flag_move==7)Velocity_target_A = + Velocity_center,
			               Velocity_target_B = - Velocity_center,
		                 Velocity_target_C = - Velocity_center,
			               Velocity_target_D = + Velocity_center;
		
     if(flag_move==8)Velocity_target_A = - Velocity_center,
			               Velocity_target_B = + Velocity_center,
		                 Velocity_target_C = + Velocity_center,
			               Velocity_target_D = - Velocity_center;
		
     if(flag_move==0)Velocity_target_A = Velocity_target_B = Velocity_target_C = Velocity_target_D = 0;		
	} 

  
	 //ת������ROS�Ŀ���ָ��
	if(FLAG_USART_ON<300){

	 Velocity_target_A = (float)(Velocity_A /0.797f/200.f)*19.f*16384.f;//���ٶ�m/s/�����ܳ�m/Ƶ�ʣ�*���ٱ�*һȦ������
	 Velocity_target_B = (float)(Velocity_B /0.797f/200.f)*19.f*16384.f;//���ٶ�m/s/�����ܳ�m/Ƶ�ʣ�*���ٱ�*һȦ������
	 Velocity_target_C = (float)(Velocity_C /0.797f/200.f)*19.f*16384.f;//���ٶ�m/s/�����ܳ�m/Ƶ�ʣ�*���ٱ�*һȦ������
	 Velocity_target_D = (float)(Velocity_D /0.797f/200.f)*19.f*16384.f;//���ٶ�m/s/�����ܳ�m/Ƶ�ʣ�*���ٱ�*һȦ������		
	} 
			
}
/****************************************************************************************
����ʽPI����
****************************************************************************************/
int velocityA_Control(int encoder,int target)
{
	  static float velocity,bias,Last_bias; 	
		bias = target - encoder;       //�ٶ��˲�  	
		velocity += Velocity_B_KP*(bias-Last_bias) + bias*Velocity_B_KI/100;//�ٶȿ���	
    Last_bias = bias;
	  if(Start_Flag==0)Last_bias = 0,velocity = 0;//��������
	  return velocity;
}
/****************************************************************************************
����ʽPI����
****************************************************************************************/
int velocityB_Control(int encoder,int target)
{
	  static float velocity,bias,Last_bias; 	
		bias = target - encoder;       //�ٶ��˲�  	
		velocity += Velocity_B_KP*(bias-Last_bias) + bias*Velocity_B_KI/100;//�ٶȿ���	
    Last_bias = bias;
	  if(Start_Flag==0)Last_bias = 0,velocity = 0;//��������
	  return velocity;
}
/****************************************************************************************
����ʽPI����
****************************************************************************************/
int velocityC_Control(int encoder,int target)
{
	  static float velocity,bias,Last_bias; 	
		bias = target - encoder;       //�ٶ��˲�  	
		velocity += Velocity_B_KP*(bias-Last_bias) + bias*Velocity_B_KI/100;//�ٶȿ���	
    Last_bias = bias;
	  if(Start_Flag==0)Last_bias = 0,velocity = 0;//��������
	  return velocity;
}
/****************************************************************************************
����ʽPI����
****************************************************************************************/
int velocityD_Control(int encoder,int target)
{
	  static float velocity,bias,Last_bias; 	
		bias = target - encoder;       //�ٶ��˲�  	
		velocity += Velocity_B_KP*(bias-Last_bias) + bias*Velocity_B_KI/100;//�ٶȿ���	
    Last_bias = bias;
	  if(Start_Flag==0)Last_bias = 0,velocity = 0;//��������
	  return velocity;
}
/**************************************************************************
�������ܣ�ֱ������ʱ�Ƕȳ������
**************************************************************************/
int Go_straight(float Angle,float Target)
{  
	static float Last,Bias,Differential;
	       float PWM,Least;
  	Least =Angle-Target;//��ȡƫ��
	  if(Least<+100&&Least>-100)     //�ٽ�ת������
		{
          Bias *=0.8;		           //һ�׵�ͨ�˲� 
          Bias += Least*0.2;	     //һ�׵�ͨ�˲� 
	        Differential=Bias-Last;  //��ȡƫ��仯��
	        Last=Bias;               //������һ�ε�ƫ��
		      PWM=Bias*line_KP+Differential*line_KD; //λ�ÿ��� 
		}
		else  {//�ٽ�ת������
			     Bias *=0.8;		         //һ�׵�ͨ�˲� 
           Bias += Least*0.2;	     //һ�׵�ͨ�˲� 
	         Differential=Bias-Last; //��ȡƫ��仯��
	         Last=Bias;              //������һ�ε�ƫ��
		       PWM=-(Bias*line_KP+Differential*line_KD);//λ�ÿ��� 
		      }
    if(Start_Flag==0)Last=0,Bias=0,Differential=0;//ֹͣʱ����������
	  return PWM;
	
}


/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
*************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d)
{
	  if(motor_a<0)     INA1=1,INA2=0,PWMA=-motor_a;//
	  else              INA1=0,INA2=1,PWMA=+motor_a;//
	
	  if(motor_b<0)     INB1=1,INB2=0,PWMB=-motor_b;//
	  else              INB1=0,INB2=1,PWMB=+motor_b;//
	
		if(motor_c<0)     INC1=1,INC2=0,PWMC=-motor_c;//
	  else              INC1=0,INC2=1,PWMC=+motor_c;//
	
	  if(motor_d<0)     IND1=1,IND2=0,PWMD=-motor_d;//
	  else              IND1=0,IND2=1,PWMD=+motor_d;//
}
void  send_data_to_blue(void)
{

          printf("  A:");					
          printf("%.2fV",((float)Voltage)/100);
					
          printf("B:");						
          printf("%.3f",(float)(Encoder_A-Encoder_B-Encoder_C+Encoder_D)/4.F);
					
          printf("C:");							
          printf("%.1fV",(float)Roll);
					
          printf("D:");						
          printf("%.1fV",(float)Pitch);
					
          printf("E:");						
          printf("%.1fV",(float)Yaw);
					
          printf("F:");						
          printf("%d",0);//���͵�ǰ����ֵ���ֻ�APP Ȼ����usart.c �ļ����洮��2�жϺ��������������λ���Ĳ����޸�


          printf("G");						
          printf("%d",(short)Velocity_B_KP);
          printf("H");						
          printf("%d",(short)Velocity_B_KI);
          printf("I");						
          printf("%d",(short)line_KP);
          printf("J");						
          printf("%d",(short)line_KD);
          printf("K");						
          printf("%d",(short)0);				 
		

          printf("; ");	
	
}

