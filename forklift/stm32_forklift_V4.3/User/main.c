#include "sys.h"
void Key(void);
void Correction(void);
void Controller(void);
void MOVE(int32_t H1,int32_t L1,int32_t H2,int32_t L2,uint8_t C1,uint8_t C2);
void Send_data(uint8_t flag,uint8_t state,uint8_t speed,int32_t yw);
void Send_Data_To_U1(int32_t H1,int32_t L1,int32_t H2,int32_t L2,uint8_t En1,uint8_t En2,uint8_t En3,uint8_t En4,uint8_t C1,uint8_t C2,int32_t VOL);
void TIM1_Init(u32 arr,u16 psc)  //��ʱ�ж�
{  
	RCC->APB2ENR|=1<<11;//TIM1ʱ��ʹ��    
 	TIM1->ARR=arr;      //�趨�������Զ���װֵ   
	TIM1->PSC=psc;      //Ԥ��Ƶ����Ƶ
	TIM1->DIER|=1<<0;   //��������ж�				
	TIM1->DIER|=1<<6;   //�������ж�	   
	TIM1->CR1|=0x01;    //ʹ�ܶ�ʱ��
	MY_NVIC_Init(1,2,TIM1_UP_IRQn,2);
}
int Voltage;
int count_sum;
int go_to=10000;
uint16_t count_num;
uint16_t Count_A=0;
uint16_t Count_B_1=0;
uint16_t Count_C_1=0;
uint16_t Count_B_2=0;
uint16_t Count_C_2=0;
uint16_t Count_D=0;

uint8_t S_data[9];

uint8_t EN_Moto1=1;
uint8_t EN_Moto2=1;
uint8_t EN_Moto3=1;
uint8_t EN_Moto4=1;

uint8_t vol_flag=1;
uint8_t st_flag1=1;
uint8_t st_flag2=1;
uint8_t st_flag3=1;
uint8_t st_flag4=1;

uint8_t Clamp_flag1=0;
uint8_t Clamp_flag2=0;
uint8_t Claw1=0;
uint8_t Claw2=0;

uint8_t Arm_flag1=0;
uint8_t Arm_flag2=0;
uint8_t Arm_flag3=0;
uint8_t Arm_flag4=0;

int32_t yw1=0;
int32_t yw2=0;
int32_t yw3=0;
int32_t yw4=0;

int32_t r_yw1=0;
int32_t r_yw2=0;
int32_t r_yw3=0;
int32_t r_yw4=0;

int32_t zero_yw1=0;
int32_t zero_yw2=0;
int32_t zero_yw3=0;
int32_t zero_yw4=0;

int32_t target_yw1=0;
int32_t target_yw2=0;
int32_t target_yw3=0;
int32_t target_yw4=0;

int32_t Error_yw1=0;
int32_t Error_yw2=0;
int32_t Error_yw3=0;
int32_t Error_yw4=0;

int32_t Height1;
int32_t length1;
int32_t Height2;
int32_t length2;

int32_t S_Height1;
int32_t S_length1;
int32_t S_Height2;
int32_t S_length2;
uint8_t S_Claw1=0;
uint8_t S_Claw2=0;

uint8_t State_flag=0;
uint8_t send_buf[32];

uint8_t let_go=0;

int main(void)
{	       
		 
  Stm32_Clock_Init(9);               //ϵͳʱ������
	delay_init(72);                    //��ʱ��ʼ��
	JTAG_Set(JTAG_SWD_DISABLE);        //�ر�JTAG�ӿ�
	//JTAG_Set(SWD_ENABLE);              //��SWD�ӿ� �������������SWD�ӿڵ���
  Adc_Init();
	LED_Init();                        //��ʼ���� LED ���ӵ�Ӳ���ӿ�
	KEY_Init();                        //������ʼ��
  while(go_to){Voltage = Get_Adc(0)*3.3*11.0*100/1.0/4096;if(Voltage>2250)go_to--;}//��ѹ������Ҫ���ܼ���ִ��
	usart1_init(72,115200);            //����1��ʼ��
  usart2_init(36,115200);            //����2��ʼ��
	delay_ms(20);                      //��ʱ	
  Dma_Init(DMA1_Channel4,(u32)&USART1->DR,(u32)send_buf,31);//��ʼ��DMA1��(ͨ��4������Դsend_buf[]������Ŀ�꣺�����ַ &USART1->DR	������������),��ѭ��ģʽ	 
	USART1->CR3|=1<<7;                 //����1DMA�ж�ʹ��
 	Dma_Enable(DMA1_Channel4,31);      //DMA����ʹ�ܣ�(ͨ��4 ,����������)	
	delay_ms(20);                      //��ʱ		
	Dma_Init(DMA1_Channel7,(u32)&USART2->DR,(u32)S_data,9);//��ʼ��DMA1��ͨ��7������Դsend_buf[54]������Ŀ�꣺�����ַ &USART2->DR	������������9,��ѭ��ģʽ	 
	USART2->CR3|=1<<7;                 //����2DMA�ж�ʹ��
 	Dma_Enable(DMA1_Channel7,9);      //DMA����ʹ�ܣ�ͨ��7 ,����������53
	delay_ms(20);                      //��ʱ	
	delay_ms(1000);
	delay_ms(1000);

	Correction();
		
  delay_ms(500);
  TIM1_Init(100-1,7200);//10����   
	while(1)
		{		
       delay_ms(500);
		
		} 
}
void TIM1_UP_IRQHandler(void)  
{     
	if(TIM1->SR&0X0001)//��ʱ�ж�
	{ 		
		TIM1->SR&=~(1<<0);//�����ʱ��1�жϱ�־λ
		   
		   Key();

			//********************����������λ��������start********************************//	
       if(FLAG_USART==1)//�������1��־λ��1����ʾ�����һ�����ݴ���
	        { 				 	
						static u8 sum;  
	          for(u8 j=0;j<26;j++)sum+=USART1_data[j];    //����У���	
            if(USART1_data[3]==22&&sum==USART1_data[26])
									{					
						                   Height1  = (int32_t)((USART1_data[4]<<0) |(USART1_data[5]<<8) |(USART1_data[6]<<16) |(USART1_data[7]<<24));//��λ����
														   length1  = (int32_t)((USART1_data[8]<<0) |(USART1_data[9]<<8) |(USART1_data[10]<<16)|(USART1_data[11]<<24));//��λ����
															 Height2  = (int32_t)((USART1_data[12]<<0)|(USART1_data[13]<<8)|(USART1_data[14]<<16)|(USART1_data[15]<<24));//��λ����
															 length2  = (int32_t)((USART1_data[16]<<0)|(USART1_data[17]<<8)|(USART1_data[18]<<16)|(USART1_data[19]<<24));//��λ����
										
										           EN_Moto1 = USART1_data[20];
										           EN_Moto2 = USART1_data[21]; 
										           EN_Moto3 = USART1_data[22]; 
										           EN_Moto4 = USART1_data[23];
 										
						                   Claw1    = USART1_data[24]; 
														   Claw2    = USART1_data[25];                             										
									}	
						sum=0;	
						USART1_data[26]=0XFF;//�����У��λ												 																								
					  FLAG_USART=0;//���ڱ�־λ��0							
			   }
			//********************����������λ��������end********************************//	
				 
			//*********************����1�������ݵ���λ��start*************************************//	
       S_Height1 =(r_yw1-zero_yw1)/16384*4;
       S_length1 =(r_yw2-zero_yw2)/16384*4;	
       S_Height2 =(r_yw3-zero_yw3)/16384*4;	
       S_length2 =(r_yw4-zero_yw4)/16384*4;				 
	     if(DMA1->ISR&(1<<13))//ͨ��4������ɱ�־(��ͨ����-1��*4+1)
			      {			 
			        Dma_Disenable(DMA1_Channel4);//DMA���˳��ܣ�ͨ��4	
							
              Send_Data_To_U1(S_Height1,S_length1,S_Height2,S_length2,st_flag1,st_flag2,st_flag3,st_flag4,S_Claw1,S_Claw2,Voltage);//�������ݣ��߶�1������1���߶�2������2�����1234ʹ��λ��צ��1��״̬��צ��2��״̬����ѹADֵ��	
              //Send_Data_To_U1(0,0,0,0,1,1,1,1,0,0,0);//�������ݣ��߶�1������1���߶�2������2�����1234ʹ��λ��צ��1��״̬��צ��2��״̬��	
							
			        DMA1->IFCR|=1<<13;//���ͨ��4������ɱ�־
			        Dma_Enable(DMA1_Channel4,31);//DMA����ʹ�ܣ�ͨ��1                   
			      }
			//*********************����1�������ݵ���λ��end*************************************//		
			 
      //****************������צλ���뿪�Ͽ���********************************************//						
			 MOVE(Height1,length1,Height2,length2,Claw1,Claw2);//�߶Ⱥͳ��� ��Χ0-330 ��λ���� ��צ����0/1 ��/��				 
						
			 count_sum+=Get_Adc(0)*3.3*11.0*100/1.0/4096;//��ѹ�����ۼ�
	     count_num++;
		   if(count_num==150) Voltage=count_sum/count_num,count_num=0,count_sum=0;//��ƽ����ѹ 

       if(Voltage>2280)Led_Flash(50);			 
       else Led_Flash(12);
       if(Voltage<2230&&Count_D<400)Count_D++;
			 if(Count_D>300){vol_flag=0;}//��ѹ�����õ��ʧ��			 
	}
}
void MOVE(int32_t H1,int32_t L1,int32_t H2,int32_t L2,uint8_t C1,uint8_t C2){
	  if(H1<0)H1=0;
	  if(H1>280)H1=280;	
	  if(L1<0)L1=0;
	  if(L1>440)L1=440;
	  if(H2<0)H2=0;
	  if(H2>280)H2=280;	
	  if(L2<0)L2=0;
	  if(L2>440)L2=440;	
	
	  target_yw1 = -H1/4*16384;//ÿȦ4����  	
	  target_yw2 = +L1/4*16384;//ÿȦ4����  
	  target_yw3 = -H2/4*16384;//ÿȦ4����
	  target_yw4 = +L2/4*16384;//ÿȦ4����
	  Clamp_flag1=C1;
	  Clamp_flag2=C2;	
	
	  if(vol_flag==1){
	    st_flag1=EN_Moto1;
	    st_flag2=EN_Moto2;
	    st_flag3=EN_Moto3;
	    st_flag4=EN_Moto4;
		}else{
	    st_flag1=0;
	    st_flag2=0;
	    st_flag3=0;
	    st_flag4=0;
		} 
	
	  Controller();
	
    Error_yw1=yw1-r_yw1;
    Error_yw2=yw2-r_yw2;
    Error_yw3=yw3-r_yw3;
    Error_yw4=yw4-r_yw4;	
}
void Controller(void){//
static u8 c_flag_1=0;
static u8 c_flag_2=0;		
			 if(Limit_KEY1==0)Arm_flag1=1;else Arm_flag1=0; //����������λ���أ�����ֹͣ
			 if(Limit_KEY2==0)Arm_flag2=1;else Arm_flag2=0; //����������λ���أ�����ֹͣ
			 if(Limit_KEY3==0)Arm_flag3=1;else Arm_flag3=0; //����������λ���أ�����ֹͣ
			 if(Limit_KEY4==0)Arm_flag4=1;else Arm_flag4=0; //����������λ���أ�����ֹͣ	
       if(Limit_KEY5==0||Limit_KEY6==0){Arm_flag1=0;Arm_flag2=0;Arm_flag3=0;Arm_flag4=0;}
				 Count_A++;//���ڼ���
				 /*************************/
				 if(Count_A==1){//�������1��������
              if(Arm_flag1==1 ){
								  yw1 = zero_yw1 +target_yw1;
								  if(yw1>zero_yw1)yw1=zero_yw1; 
								  if(yw1<(zero_yw1 -16384*85)) yw1=zero_yw1-16384*85;//�趨�г���85Ȧ,340����
                  Send_data(0X01,st_flag1,1,yw1);//ʶ��λ,״̬λ 0 1/ֹͣ ����,����,ywΪ��ǰת�� ��תһȦ����Ϊ16384
							}
              else{
								  yw1 = r_yw1;//ֹͣʱĿ��ֵ��ΪΪ�ش�ֵ
							    memset(S_data,0,9);
                  Send_data(0X01,0,1,yw1);//ʶ��λ,״̬λ 0 1/ֹͣ ����,����,ywΪ��ǰת�� ��תһȦ����Ϊ16384
							}														
					 }
				 /*************************/
				 else if(Count_A==7){//�������2��������					  
              if(Arm_flag2==1 ){
								  yw2 = zero_yw2 +target_yw2;
								  if(yw2<zero_yw2)yw2=zero_yw2; 
								  if(yw2>(zero_yw2 +16384*110)) yw2=zero_yw2+16384*110;//�趨�г���85Ȧ,340����
                  Send_data(0X02,st_flag2,1,yw2);//ʶ��λ,״̬λ 0 1/ֹͣ ����,����,ywΪ��ǰת�� ��תһȦ����Ϊ16384
							}
              else{
								  yw2 = r_yw2;//ֹͣʱĿ��ֵ��ΪΪ�ش�ֵ
							    memset(S_data,0,9);
                  Send_data(0X02,0,1,yw2);//ʶ��λ,״̬λ 0 1/ֹͣ ����,����,ywΪ��ǰת�� ��תһȦ����Ϊ16384
							}

					 }
				 /*************************/
				 else if(Count_A==13){//�������3��������					  
              if(Arm_flag3==1 ){
								  yw3 = zero_yw3 +target_yw3;
								  if(yw3>zero_yw3)yw3=zero_yw3; 
								  if(yw3<(zero_yw3 -16384*85)) yw3=zero_yw3-16384*85;//�趨�г���85Ȧ,340����
	                  Send_data(0X03,st_flag3,1,yw3);//ʶ��λ,״̬λ 0 1/ֹͣ ����,����,ywΪ��ǰת�� ��תһȦ����Ϊ16384
							}
              else{
								  yw3 = r_yw3;//ֹͣʱĿ��ֵ��ΪΪ�ش�ֵ
                  Send_data(0X03,0,1,yw3);//ʶ��λ,״̬λ 0 1/ֹͣ ����,����,ywΪ��ǰת�� ��תһȦ����Ϊ16384
							}	
					 }
				 /*************************/
				 else if(Count_A==19){//�������4��������					  
              if(Arm_flag4==1 ){
								  yw4 = zero_yw4 +target_yw4;
								  if(yw4<zero_yw4)yw4=zero_yw4; 
								  if(yw4>(zero_yw4 +16384*110)) yw4=zero_yw4+16384*110;//�趨�г���85Ȧ,340����
                  Send_data(0X04,st_flag4,1,yw4);//ʶ��λ,״̬λ 0 1/ֹͣ ����,����,ywΪ��ǰת�� ��תһȦ����Ϊ16384
							}
              else{
								  yw4 = r_yw4;//ֹͣʱĿ��ֵ��ΪΪ�ش�ֵ
                  Send_data(0X04,0,1,yw4);//ʶ��λ,״̬λ 0 1/ֹͣ ����,����,ywΪ��ǰת�� ��תһȦ����Ϊ16384
							}

					 }	
				 /*************************/ 
				 else if(Count_A==25){//��צ1��������
					  c_flag_1=!c_flag_1;
					  if(c_flag_1==0){
					    if(Clamp_flag1==1){//�պ�
								 S_data[0]=	0X01;//ʶ��λ
                 S_data[1]=	0X06;//����λ
                 S_data[2]=	0X00;//		
                 S_data[3]=	0X37;
                 S_data[4]=	0X00;
                 S_data[5]=	0X01;
                 S_data[6]=	0XF9;
			           S_data[7]= 0XC4;
						  }
					    else{//�ɿ�
		             S_data[0]=	0X01;//ʶ��λ
                 S_data[1]=	0X06;//����λ
                 S_data[2]=	0X00;//		
                 S_data[3]=	0X36;
                 S_data[4]=	0X00;
                 S_data[5]=	0X01;
                 S_data[6]=	0XA8;
			           S_data[7]= 0X04;
						  }	
						}	else{//�鿴��צ״̬
								 S_data[0]=	0X01;//ʶ��λ
                 S_data[1]=	0X03;//����λ
                 S_data[2]=	0X00;//		
                 S_data[3]=	0X40;
                 S_data[4]=	0X00;
                 S_data[5]=	0X01;
                 S_data[6]=	0X85;
			           S_data[7]= 0XDE;
						}
			      if(DMA1->ISR&(1<<25))//ͨ��7������ɱ�־
			      {			 
			        Dma_Disenable(DMA1_Channel7);//DMA���˳��ܣ�ͨ��7 	
					
			        DMA1->IFCR|=1<<25;//���ͨ��7������ɱ�־
			        Dma_Enable(DMA1_Channel7,8);//DMA����ʹ�ܣ�ͨ��7                    
			      }						
					 }
				 /*************************/
				 else if(Count_A==31){//��צ2��������
					  c_flag_2=!c_flag_2;
					  if(c_flag_2==0){					 
					    if(Clamp_flag2==1){//�պ�
		             S_data[0]=	0X02;//ʶ��λ
                 S_data[1]=	0X06;//����λ
                 S_data[2]=	0X00;//		
                 S_data[3]=	0X37;
                 S_data[4]=	0X00;
                 S_data[5]=	0X01;
                 S_data[6]=	0XF9;
			           S_data[7]= 0XF7;
						  }
					    else{//�ɿ�
		             S_data[0]=	0X02;//ʶ��λ
                 S_data[1]=	0X06;//����λ
                 S_data[2]=	0X00;//		
                 S_data[3]=	0X36;
                 S_data[4]=	0X00;
                 S_data[5]=	0X01;
                 S_data[6]=	0XA8;
			           S_data[7]= 0X37;
						  }	
						}	else{//�鿴��צ״̬
								 S_data[0]=	0X02;//ʶ��λ
                 S_data[1]=	0X03;//����λ
                 S_data[2]=	0X00;//		
                 S_data[3]=	0X40;
                 S_data[4]=	0X00;
                 S_data[5]=	0X01;
                 S_data[6]=	0X85;
			           S_data[7]= 0XED;
						}
						
			      if(DMA1->ISR&(1<<25))//ͨ��7������ɱ�־
			      {			 
			        Dma_Disenable(DMA1_Channel7);//DMA���˳��ܣ�ͨ��7 	
					
			        DMA1->IFCR|=1<<25;//���ͨ��7������ɱ�־
			        Dma_Enable(DMA1_Channel7,8);//DMA����ʹ�ܣ�ͨ��7                    
			      }
					 }
				 /*************************/
				 else if(Count_A>=37){Count_A=0;
					 }	

}

void Correction(void){//У������λ��
static u8 num=0;
	  Led_Flash(1);//�����˱�ʾ׼�����
	  while(1){if(let_go==1)break;}//����1���յ���Ϣ����ʼУ��
	  for(uint16_t i=0;i<65535;i++){ 
			if(Limit_KEY1==1&&Limit_KEY2==1&&Limit_KEY3==1&&Limit_KEY4==1)num++;
			if(num>3)break;//���л��鶼��������λ���غ�5��ѭ������У��
			
			      if(Limit_KEY1==0&& Limit_KEY5==1&& Limit_KEY6==1){
							  yw1=+16384*95;//δ��������λ���أ���ʱ�뷽��ת95Ȧ
                Send_data(0X01,1,0,yw1);//ʶ��λ,״̬λ 0 1/ֹͣ ����,����,ywΪ��ǰת�� ��תһȦ����Ϊ16384
						}
			      else{
							  yw1=r_yw1-16384;//������λ���غ�ֹͣת��������Ŀ��ֵ˳ʱ���תһȦ��δִ��
                Send_data(0X01,0,0,yw1);//ʶ��λ,״̬λ 0 1/ֹͣ ����,����,ywΪ��ǰת�� ��תһȦ����Ϊ16384
						}			
		        delay_ms(30);	
			      if(Limit_KEY2==0&& Limit_KEY5==1&& Limit_KEY6==1){
							  yw2=-16384*115;//δ��������λ���أ���ʱ�뷽��ת95Ȧ
                Send_data(0X02,1,0,yw2);//ʶ��λ,״̬λ 0 1/ֹͣ ����,����,ywΪ��ǰת�� ��תһȦ����Ϊ16384
						}
			      else{
							  yw2=r_yw2+16384*2;//������λ���غ�ֹͣת��������Ŀ��ֵ˳ʱ���תһȦ��δִ��
                Send_data(0X02,0,0,yw2);//ʶ��λ,״̬λ 0 1/ֹͣ ����,����,ywΪ��ǰת�� ��תһȦ����Ϊ16384		
							}			
		       delay_ms(30);
			      if(Limit_KEY3==0&& Limit_KEY5==1&& Limit_KEY6==1){
							  yw3=+16384*95;//δ��������λ���أ���ʱ�뷽��ת95Ȧ
                Send_data(0X03,1,0,yw3);//ʶ��λ,״̬λ 0 1/ֹͣ ����,����,ywΪ��ǰת�� ��תһȦ����Ϊ16384
						}
			      else{
							  yw3=r_yw3-16384;//������λ���غ�ֹͣת��������Ŀ��ֵ˳ʱ���תһȦ��δִ��
                Send_data(0X03,0,0,yw3);//ʶ��λ,״̬λ 0 1/ֹͣ ����,����,ywΪ��ǰת�� ��תһȦ����Ϊ16384
						}			
		        delay_ms(30);	
			      if(Limit_KEY4==0&& Limit_KEY5==1&& Limit_KEY6==1){
							  yw4=-16384*115;//δ��������λ���أ���ʱ�뷽��ת95Ȧ
                Send_data(0X04,1,0,yw4);//ʶ��λ,״̬λ 0 1/ֹͣ ����,����,ywΪ��ǰת�� ��תһȦ����Ϊ16384
						}
			      else{
							  yw4=r_yw4+16384*2;//������λ���غ�ֹͣת��������Ŀ��ֵ˳ʱ���תһȦ��δִ��
                Send_data(0X04,0,0,yw4);//ʶ��λ,״̬λ 0 1/ֹͣ ����,����,ywΪ��ǰת�� ��תһȦ����Ϊ16384		
							}			
		       delay_ms(30);								
	}

	delay_ms(50);	
	for(uint16_t i=0;i<5;i++){

              Send_data(0X01,1,1,yw1);//ʶ��λ,״̬λ 0 1/ֹͣ ����,����,ywΪ��ǰת�� ��תһȦ����Ϊ16384
	            delay_ms(50);	

              Send_data(0X02,1,1,yw2);//ʶ��λ,״̬λ 0 1/ֹͣ ����,����,ywΪ��ǰת�� ��תһȦ����Ϊ16384
              delay_ms(50);
							
              Send_data(0X03,1,1,yw3);//ʶ��λ,״̬λ 0 1/ֹͣ ����,����,ywΪ��ǰת�� ��תһȦ����Ϊ16384
	            delay_ms(50);	

              Send_data(0X04,1,1,yw4);//ʶ��λ,״̬λ 0 1/ֹͣ ����,����,ywΪ��ǰת�� ��תһȦ����Ϊ16384
              delay_ms(50);							
	}
	
	delay_ms(100);								
	//��¼�ϵ�ʱ�ĽǶ�
	zero_yw1=r_yw1;//��¼��ǰ�Ƕȣ����õ�ǰ�Ƕ�Ϊ��㡣
	zero_yw2=r_yw2;//
	zero_yw3=r_yw3;//��¼��ǰ�Ƕȣ����õ�ǰ�Ƕ�Ϊ��㡣
	zero_yw4=r_yw4;//	
  delay_ms(100);	
}
/**************************************************************************
����ָ��
**************************************************************************/
void Key(void)//����ɨ�裬���жϻ�ȡ��ʱ������Ƶ��Ӱ��
{	
static u8 flag=0;

flag=click_N_Double(60);	
if(flag==1 ){st_flag1=!st_flag1;st_flag2=!st_flag2;st_flag3=!st_flag3;st_flag4=!st_flag4; flag=0;}//��������

}
void Send_data(uint8_t flag,uint8_t state,uint8_t speed,int32_t yw){//����2�������ݵ�SP3 485����
	            memset(S_data,0,9);
		          S_data[0]=	0XAA;//֡ͷ
              S_data[1]=	flag;//ʶ��λ
              S_data[2]=	state;//״̬λ 0 1/ֹͣ ����
              S_data[3]=	speed;//����			
              S_data[4]=	yw>>0;//ywΪ��ǰת�� ��תһȦ����Ϊ16384
              S_data[5]=	yw>>8;//
              S_data[6]=	yw>>16;//
              S_data[7]=	yw>>24;//
			        S_data[8]=0;
              for(uint8_t i=0;i<8;i++)	S_data[8]+=	S_data[i];	//ǰ8�����ݺ�У��λ				

	          if(DMA1->ISR&(1<<25))//ͨ��7������ɱ�־
			      {			 
			        Dma_Disenable(DMA1_Channel7);//DMA���˳��ܣ�ͨ��7 	
					
			        DMA1->IFCR|=1<<25;//���ͨ��7������ɱ�־
			        Dma_Enable(DMA1_Channel7,9);//DMA����ʹ�ܣ�ͨ��7                    
			      }
}
void Send_Data_To_U1(int32_t H1,int32_t L1,int32_t H2,int32_t L2,uint8_t En1,uint8_t En2,uint8_t En3,uint8_t En4,uint8_t C1,uint8_t C2,int32_t VOL){//����1�������ݵ���λ��

    send_buf[30]=0;  //У��������
    send_buf[0]=0XAA;   //֡ͷ
	  send_buf[1]=0XAA;   //֡ͷ
    send_buf[2]=0XF1;   //������
    send_buf[3]=26;     //���ݳ���
	
              send_buf[4]=	H1>>0;// 
              send_buf[5]=	H1>>8;//
              send_buf[6]=	H1>>16;//
              send_buf[7]=	H1>>24;//

              send_buf[8]=	L1>>0;// 
              send_buf[9]=	L1>>8;//
              send_buf[10]=	L1>>16;//
              send_buf[11]=	L1>>24;//

              send_buf[12]=	H2>>0;// 
              send_buf[13]=	H2>>8;//
              send_buf[14]=	H2>>16;//
              send_buf[15]=	H2>>24;//

              send_buf[16]=	L2>>0;// 
              send_buf[17]=	L2>>8;//
              send_buf[18]=	L2>>16;//
              send_buf[19]=	L2>>24;//
							
              send_buf[20]=	En1;//
              send_buf[21]=	En2;//
							send_buf[22]=	En3;//
              send_buf[23]=	En4;//
							
              send_buf[24]=	C1;//
              send_buf[25]=	C2;//
							
              send_buf[26]=	VOL>>0;// 
              send_buf[27]=	VOL>>8;//
              send_buf[28]=	VOL>>16;//
              send_buf[29]=	VOL>>24;//							
							
    for(uint8_t i=0;i<30;i++)send_buf[30]+=send_buf[i];//����У���
}
//*************************//
