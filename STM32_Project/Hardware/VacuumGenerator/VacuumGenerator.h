#ifndef _VacuumGenerator_H
#define _VacuumGenerator_H
#include "stm32f10x.h"

#define PresureSensor1 GPIO_Pin_0		//ѹ��������1ģ��������ڣ�PC0
#define PresureSensor2 GPIO_Pin_1		//ѹ��������2ģ��������ڣ�PC1
#define PresureSensor3 GPIO_Pin_2		//ѹ��������3ģ��������ڣ�PC2
#define PresureSensor4 GPIO_Pin_3		//ѹ��������4ģ��������ڣ�PC3

#define SupplyValve1  GPIO_Pin_8		//��շ�����������1��PE8
#define SupplyValve2  GPIO_Pin_9		//��շ�����������2��PE9
#define SupplyValve3  GPIO_Pin_10		//��շ�����������3��PE10
#define SupplyValve4  GPIO_Pin_11		//��շ�����������4��PE11
#define ReleaseValve1  GPIO_Pin_12		//��շ������ͷŷ�1��PE12
#define ReleaseValve2  GPIO_Pin_13		//��շ������ͷŷ�2��PE13
#define ReleaseValve3  GPIO_Pin_14		//��շ������ͷŷ�3��PE14
#define ReleaseValve4  GPIO_Pin_15		//��շ������ͷŷ�3��PE15

void adc_init(void);					//adc��ʼ������
void VacuumGenerator_init(void);		//��շ�������ʼ������
float  Pressure_convert(void);			//ѹ��ת������
#endif
