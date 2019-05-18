#ifndef _SCA1000_H
#define _SCA1000_H
#include "stm32f10x.h"
/*����ȫ�ֱ���*/
extern float realAX1;		//��ϴ�۴�۽Ƕ�
extern float realAX2;		//��ϴ��С�۽Ƕ�
extern float realAX3;		//��ϴ����ϴװ�ýǶ�
/* ����SCA1000 ʹ�õ�ƬѡIO�� */
#define SCA1000_CS1 GPIO_Pin_5		//SCA1000_CS1Ƭѡ����PE5�˿�
#define SCA1000_CS2 GPIO_Pin_6		//SCA1000_CS2Ƭѡ����PE6�˿�
#define SCA1000_CS3 GPIO_Pin_7		//SCA1000_CS3Ƭѡ����PE7�˿�

#define SCA1000_CS1_SET GPIO_SetBits(GPIOE, GPIO_Pin_5)		//�ر�Ƭѡ1
#define SCA1000_CS1_CLR GPIO_ResetBits(GPIOE, GPIO_Pin_5)	//��Ƭѡ1
#define SCA1000_CS2_SET GPIO_SetBits(GPIOE, GPIO_Pin_6)		//�ر�Ƭѡ2
#define SCA1000_CS2_CLR GPIO_ResetBits(GPIOE, GPIO_Pin_6)	//��Ƭѡ2
#define SCA1000_CS3_SET GPIO_SetBits(GPIOE, GPIO_Pin_7)	 	//�ر�Ƭѡ2
#define SCA1000_CS3_CLR GPIO_ResetBits(GPIOE, GPIO_Pin_7)	//��Ƭѡ2

/* ����ָ��� */
#define RDAX    0x10   //��X��Ƕȿ���ָ��
/* �����ⲿ���ú��� */
void SCA1000_Init(void);  		//SCA1000��ʼ��
void SCA1000_ReadAXData(void);  //SCA1000��ȡX��Ƕ�
void SPI2_Init(void); 			//SPI2��ʼ��		
u8 SPI2_WriteReadData(u8 dat);	//SPI2д��һ���ֽ�����ͬʱ��ȡһ���ֽ�����
#endif

