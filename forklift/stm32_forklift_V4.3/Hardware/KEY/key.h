#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"

#define KEY PAin(8)
#define Limit_KEY1 PAin(4)
#define Limit_KEY2 PAin(5)
#define Limit_KEY3 PAin(6)
#define Limit_KEY4 PBin(12)
#define Limit_KEY5 PBin(13)
#define Limit_KEY6 PBin(14)
void KEY_Init(void);          //������ʼ��
u8 click_N_Double (u8 time);  //��������ɨ���˫������ɨ��
u8 click(void);               //��������ɨ��
u8 Long_Press(void);
#endif 
