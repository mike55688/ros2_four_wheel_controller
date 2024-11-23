#include "usart.h"	  
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef�� d in stdio.h. */ 
FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
int _sys_exit(int x) 
{ 
	x = x; 
	return 0;
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      

	while((USART1->SR&0X40)==0);
	USART1->DR = (u8) ch;      
  return ch;
}
#endif 

//******************************����1******************************************//
/////////////////////////////////////////////////////////////////////////////////
//****************usart1����һ���ֽ�************************************//
void usart1_send(u8 data)
{
	USART1->DR = data;
	while((USART1->SR&0x40)==0);	
}
//****************����1��ʼ��************************************//
void usart1_init(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//�õ�USARTDIV
	mantissa=temp;				 //�õ���������
	fraction=(temp-mantissa)*16; //�õ�С������	 
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->APB2ENR|=1<<2;   //ʹ��PORTA��ʱ��  
	RCC->APB2ENR|=1<<14;  //ʹ�ܴ���ʱ�� 
	GPIOA->CRH&=0XFFFFF00F;//IO״̬����
	GPIOA->CRH|=0X000008B0;//IO״̬����
	GPIOA->ODR|=1<<9;	  
	RCC->APB2RSTR|=1<<14;   //��λ����1
	RCC->APB2RSTR&=~(1<<14);//ֹͣ��λ	   	   
	//����������
 	USART1->BRR=mantissa; // ����������	 
	USART1->CR1|=0X200C;  //1λֹͣ,��У��λ.
	USART1->CR1|=1<<8;    //PE�ж�ʹ��
	USART1->CR1|=1<<5;    //���ջ������ǿ��ж�ʹ��	    	
	MY_NVIC_Init(0,1,USART1_IRQn,2);//��ռ���ȼ�2,��Ӧ���ȼ�2,��2
}

//******************************����1�����ж�*************************************//
u8 USART1_data[27];
u8 data_len=0;
u8 FLAG_USART=0;	

int USART1_IRQHandler(void)
{	
	if(USART1->SR&(1<<5))//���յ�����
	{	      
			USART1_data[data_len] = USART1->DR;//��ȡ����
		  let_go=1;
      if(USART1_data[0]==0XAA){
			  data_len++;
				if(data_len>26)FLAG_USART=1,data_len=0;
			}	
   }
return 0;	
}
 


//******************************����2********************************//
///////////////////////////////////////////////////////////////////////
//****************************usart2����һ���ֽ�*********************//
void usart2_send(u8 data)
{
	USART2->DR = data;
	while((USART2->SR&0x40)==0);	
}
//******************************����2��ʼ��**************************//
void usart2_init(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//�õ�USARTDIV
	mantissa=temp;				 //�õ���������
	fraction=(temp-mantissa)*16; //�õ�С������	 
  mantissa<<=4;
	mantissa+=fraction; 
	RCC->APB2ENR|=1<<2;   //ʹ��PORTA��ʱ��  
	RCC->APB1ENR|=1<<17;  //ʹ�ܴ���ʱ�� 
	GPIOA->CRL&=0XFFFF00FF; 
	GPIOA->CRL|=0X00008B00;//IO״̬����
  
	RCC->APB1RSTR|=1<<17;   //��λ����1
	RCC->APB1RSTR&=~(1<<17);//ֹͣ��λ	   	   
	//����������
 	USART2->BRR=mantissa; // ����������	 
	USART2->CR1|=0X200C;  //1λֹͣ,��У��λ.
	//ʹ�ܽ����ж�
	USART2->CR1|=1<<8;    //PE�ж�ʹ��
	USART2->CR1|=1<<5;    //���ջ������ǿ��ж�ʹ��	    	
	MY_NVIC_Init(0,1,USART2_IRQn,2);//��2��������ȼ� 
}


const uint16_t polynom = 0xA001;
uint8_t Crc_L;
uint8_t Crc_H;
uint16_t Crc; 
uint16_t crc16bitbybit(uint8_t *ptr, uint16_t len)
{
    uint8_t i;
    uint16_t crc = 0xffff;
 
    if (len == 0) {
        len = 1;
    }
    while (len--) {
        crc ^= *ptr;
        for (i = 0; i<8; i++)
        {
            if (crc & 1) {
                crc >>= 1;
                crc ^= polynom;
            }
            else {
                crc >>= 1;
            }
        }
        ptr++;
    }
    return(crc);
}
//**************************����2�����ж�***********************//
int Usart2_Receive;
int Usart3_Receive;
u8 data_U2[8];
u8 flag_mode_app=0;
int anjian_app,huakuai_app,yaogan_app=510;
u8 len_2;
u8 Sum_1;

u32 data;
/*********************************************
���ߣ���������
�Ա����̣�http://shop180997663.taobao.com/
*********************************************/
int USART2_IRQHandler(void)
{	

	if(USART2->SR&(1<<5))//���յ�����
	{	   
    data_U2[len_2]=USART2->DR;
			switch(len_2)
			{ 				
				case 0:
					if(data_U2[0]==0XBB||data_U2[0]==0X01||data_U2[0]==0X02)len_2++;//֡ͷ
					break;
				case 1:
          if(data_U2[1]==0X01||data_U2[1]==0X02||data_U2[1]==0X03||data_U2[1]==0X04)len_2++;//ʶ��λ
				  else {len_2=0;memset(data_U2,0,sizeof(7));}//�Ǳ���ָ����ò���
					break;				
				case 2:
				case 3:            
				case 4:            
				case 5:                      
          len_2++;
					break;				
				case 6:
					Sum_1=0;
				  if(data_U2[0]==0XBB){//֡ͷ��ʾ˿�˲������
            for(uint8_t i=0;i<6;i++)Sum_1+=	data_U2[i];	//ǰ6�����ݺ�У��λ
				    if(data_U2[6] == Sum_1){ 
						  if(data_U2[1]==0x01)r_yw1=(int32_t)((data_U2[2]<<0)|(data_U2[3]<<8)|(data_U2[4]<<16)|(data_U2[5]<<24));
					    if(data_U2[1]==0x02)r_yw2=(int32_t)((data_U2[2]<<0)|(data_U2[3]<<8)|(data_U2[4]<<16)|(data_U2[5]<<24));
					    if(data_U2[1]==0x03)r_yw3=(int32_t)((data_U2[2]<<0)|(data_U2[3]<<8)|(data_U2[4]<<16)|(data_U2[5]<<24));
					    if(data_U2[1]==0x04)r_yw4=(int32_t)((data_U2[2]<<0)|(data_U2[3]<<8)|(data_U2[4]<<16)|(data_U2[5]<<24));						
					  }
					}else{//֡ͷ��ʾ��צ�������
						Crc=crc16bitbybit(data_U2, 5);//ǰ5�����ݽ���CRCУ�飬���У����
						Crc_L=Crc>>0;//У�����8λ
						Crc_H=Crc>>8;//У�����8λ
						if(data_U2[0]==0X01&&data_U2[1]==0X03&&Crc_L==data_U2[5]&&Crc_H==data_U2[6]){S_Claw1=data_U2[4];}
						if(data_U2[0]==0X02&&data_U2[1]==0X03&&Crc_L==data_U2[5]&&Crc_H==data_U2[6]){S_Claw2=data_U2[4];}						
					}
					len_2=0;memset(data_U2,0,sizeof(7));//����У���Ƿ�ͨ�������ò���	
					break;
				default:
					break;				
			}	    		
  }
return 0;	

}
//*******************����3��ʼ��,PCLK2 ʱ��Ƶ��(Mhz),bound:������************************//
void usart3_init(u32 pclk2,u32 bound)
{  	 
  float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//�õ�USARTDIV
	mantissa=temp;				 //�õ���������
	fraction=(temp-mantissa)*16; //�õ�С������	 
  mantissa<<=4;
	mantissa+=fraction; 
	RCC->APB2ENR|=1<<0;    //��������ʱ��
	RCC->APB2ENR|=1<<3;   //ʹ��PORTB��ʱ��  
	RCC->APB1ENR|=1<<18;  //ʹ�ܴ���ʱ�� 
	GPIOB->CRH&=0XFFFF00FF; 
	GPIOB->CRH|=0X00008B00;//IO״̬����
	GPIOB->ODR|=1<<10;	 
	RCC->APB1RSTR|=1<<18;   //��λ����1
	RCC->APB1RSTR&=~(1<<18);//ֹͣ��λ	   	   
	//����������
 	USART3->BRR=mantissa; // ����������	 
	USART3->CR1|=0X200C;  //1λֹͣ,��У��λ.
	//ʹ�ܽ����ж�
	USART3->CR1|=1<<8;    //PE�ж�ʹ��
	USART3->CR1|=1<<5;    //���ջ������ǿ��ж�ʹ��    	
	MY_NVIC_Init(0,1,USART3_IRQn,2);//��2��������ȼ� 
}

//****************************�������ܣ�����3�����ж�***************************************//
int USART3_IRQHandler(void)
{	
	if(USART3->SR&(1<<5))//���յ�����
	{	   

					Usart3_Receive = USART3->DR;//��ȡ����		
			 
   }
return 0;	
}
