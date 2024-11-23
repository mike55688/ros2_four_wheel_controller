#include "DMA.h" 	 

  
//DMAͨ����ʼ������
//����˵��:
//          DMA_CHx      :ѡ��DMA����ͨ��,DMA1��1-7,DMA2��1-5
//          P_Adress     :�����ַ
//          M_Adress     :�洢����ַ	 
void Dma_Init(u8 num,DMA_Channel_TypeDef * DMA_CHx,u32 P_Address ,u32 M_Address,u16 Number)
{
	    if(num==1)
	    RCC->AHBENR |= 1<<0;//����DMA1ʱ��
      else if(num==2)
	    RCC->AHBENR |= 1<<1;//����DMA2ʱ��
			
	    delay_ms(10);                
	    //�������ú�ͨ�������õ�ַ
	    DMA_CHx -> CPAR = P_Address;    //�趨����Ĵ�����ַ
	    DMA_CHx -> CMAR = M_Address;    //�趨���ݼĴ�����ַ
	    delay_ms(10);
	    DMA_CHx -> CNDTR = Number;       //���ݴ�����	
	    delay_ms(10);
			DMA_CHx -> CCR  &= 0x00000000;       //��λ  
		  delay_ms(10);
	    DMA_CHx -> CCR  |= 0<<14;          //0:�Ǵ洢�����洢��ģʽ; 1:�洢�����洢��ģʽ 
	   	//�趨Ϊ�е����ȼ�
			DMA_CHx -> CCR  |= 0<<13;          //00:�� 01:�� 10:�� 11:��� 
	    DMA_CHx -> CCR  |= 0<<12;          //ͨ�����ȼ�,��[13:12]��λ����
			//�趨�洢���ݿ��
      DMA_CHx -> CCR  |= 0<<11;          //00:8λ 01:16λ 10:32λ 11:���� 
	    DMA_CHx -> CCR  |= 0<<10;          //�洢�����ݿ��,��[11:10]����

			//�趨�������ݿ��
		  DMA_CHx -> CCR  |= 0<<9;           //00:8λ 01:16λ 10:32λ 11:����
	    DMA_CHx -> CCR  |= 0<<8;           //�������ݿ��,��[9:8]��λ����

			//�趨��ַ����
			DMA_CHx -> CCR  |= 1<<7;           //0:��ִ�д洢����ַ�������� 1:ִ�д洢����ַ��������			
	    DMA_CHx -> CCR  |= 0<<6;           //0:��ִ�������ַ��������   1:ִ�������ַ��������

	   	//�趨���ݴ��䷽��
			DMA_CHx -> CCR  |= 0<<5;           //0:��ִ��ѭ������ 1:ִ��ѭ������ 
	    DMA_CHx -> CCR  |= 1<<4;           //�趨���ݴ��䷽��   0:������� 1:�Ӵ洢����
			
//      DMA_CHx -> CCR  |= 1<<3;           //����������жϣ���дһ�������ĵ�ַ���򣬽������MDA�������
//      DMA_CHx -> CCR  |= 1<<2;           //����봫���ж�			
//	    DMA_CHx -> CCR  |= 1<<1;           //����������ж�
	    
	    		
}
	 
	 	 
//DMAͨ��ʹ��
//����˵��:
//          DMA_CHx      :ѡ��DMA������ͨ��,DMA1��1-7,DMA2��1-5
//          Number       :���ݴ�����
void Dma_Enable(DMA_Channel_TypeDef * DMA_CHx,u16 Number)
{
	    DMA_CHx -> CNDTR = Number;       //���ݴ�����
	    DMA_CHx -> CCR |= 1<<0;          //��ʼDMA����
}
//DMAͨ������
//����˵��:
//          DMA_CHx      :ѡ��DMA������ͨ��,DMA1��1-7,DMA2��1-5
//          Number       :���ݴ�����
void Dma_Disenable(DMA_Channel_TypeDef * DMA_CHx)
{
	    DMA_CHx->CCR&=~(1<<0);       //�ر�DMA����
}
