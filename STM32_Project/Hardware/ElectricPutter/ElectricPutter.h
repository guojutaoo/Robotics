#ifndef _ElectricPutter_H
#define _ElectricPutter_H
#include "stm32f10x.h"
#define NP1 GPIO_Pin_0		//PF0,���1NP����������
#define PP1 GPIO_Pin_1		//PF1,���1PP����������
#define NP2 GPIO_Pin_2		//PF2,���2NP����������
#define PP2 GPIO_Pin_3		//PF3,���2PP����������
#define NP3 GPIO_Pin_4		//PF4,���3NP����������
#define PP3 GPIO_Pin_5		//PF5,���3PP����������
#define NP4 GPIO_Pin_6		//PF6,���4NP����������
#define PP4 GPIO_Pin_7		//PF7,���4PP����������
#define NP5 GPIO_Pin_8		//PF8,���5NP����������
#define PP5 GPIO_Pin_9		//PF9,���5PP����������
#define NP6 GPIO_Pin_10		//PF10,���6NP����������
#define PP6 GPIO_Pin_11		//PF11,���6PP����������
#define NP7 GPIO_Pin_12		//PF12,���7NP����������
#define PP7 GPIO_Pin_13		//PF13,���7PP����������
#define NP8 GPIO_Pin_14		//PF14,���8NP����������
#define PP8 GPIO_Pin_15		//PF15,���8PP����������

#define NP9  GPIO_Pin_0		//PG0,���9NP����������
#define PP9  GPIO_Pin_1		//PG1,���9PP����������
#define NP10 GPIO_Pin_2		//PG2,���10NP����������
#define PP10 GPIO_Pin_3		//PG3,���10PP����������
#define NP11 GPIO_Pin_4		//PG4,���11NP����������
#define PP11 GPIO_Pin_5		//PG5,���11PP����������
#define NP12 GPIO_Pin_6		//PG6,���12NP����������
#define PP12 GPIO_Pin_7		//PG7,���12PP����������
#define NP13 GPIO_Pin_8		//PG8,���13NP����������
#define PP13 GPIO_Pin_9		//PG9,���13PP����������
#define NP14 GPIO_Pin_10	//PG10,���14NP����������
#define PP14 GPIO_Pin_11	//PG11,���14PP����������
#define NP15 GPIO_Pin_12	//PG12,���15NP����������
#define PP15 GPIO_Pin_13	//PG13,���15PP����������
//����iic2��
#define SVON    0x44   		//����ŷ�on��Ӧio��չоƬ��ַ�����һλ����0��д������
#define RESET   0x42   		//��˱��������Ӧio��չоƬ��ַ�����һλ����0��д������
#define SETUP   0x40   		//���ԭ��ع��Ӧio��չоƬ��ַ�����һλ����0��д������
//����iic1��
#define BUSY    0x40   		//��������ж�Ӧio��չоƬ��ַ�����һλ����0��д������
#define SETON   0x42   		//���ԭ��ع���ɶ�Ӧio��չоƬ��ַ�����һλ����0��д������
#define INP    	0x44   		//��˶�λ��ɶ�Ӧio��չоƬ��ַ�����һλ����0��д������
#define ESTOP   0x48   		//��˽���ֹͣ��Ӧio��չоƬ��ַ�����һλ����0��д������
#define ALARM   0x46   		//��˱�����Ӧio��չоƬ��ַ�����һλ����0��д������


void ElectricPutter_Init(void);	  //�˿ڳ�ʼ��
u16 CAT9555_ReadByte(u8 QJaddr); //CAT9555�Ķ�����
void CAT9555_WriteByte(u8 QJaddr,u8 dt0,u8 dt1);//CAT9555��д����
void ElectricPutter_DDMove(float DGlen,u8 DGdir,u8 DGbh);
void ElectricPutter_Move(float DGlen,u8 DGdir,u8 DGbh);	  //����ƶ�����
//void DG_Reverse(u32 DGPulse);
#endif



