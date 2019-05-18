#ifndef _ServoMotor_H
#define _ServoMotor_H
#include "stm32f10x.h"
#define CA1 GPIO_Pin_0		//PD0,���1A����������
#define CB1 GPIO_Pin_1		//PD1,���1B����������
#define CA2 GPIO_Pin_3		//PD3,���2A����������
#define CB2 GPIO_Pin_4		//PD4,���2B����������
#define CA3 GPIO_Pin_5		//PD5,���3A����������
#define CB3 GPIO_Pin_6		//PD6,���3B����������
#define CA4 GPIO_Pin_7		//PD7,���4A����������
#define CB4 GPIO_Pin_8		//PD8,���4B����������
#define CA5 GPIO_Pin_9		//PD9,���5A����������
#define CB5 GPIO_Pin_10		//PD10,���5B����������
#define CA6 GPIO_Pin_11		//PD11,���6A����������
#define CB6 GPIO_Pin_12		//PD12,���6B����������
#define CA7 GPIO_Pin_13		//PD13,���7A����������
#define CB7 GPIO_Pin_14		//PD14,���7B����������
#define TX_485 GPIO_Pin_2	//485���ڷ��Ͷ˿�
#define RX_485 GPIO_Pin_3	//485���ڽ��ն˿�
#define CS_485 GPIO_Pin_8	//485���ڷ���/����Ƭѡ�˿�,PA8
#define CS_485_Send 	GPIO_SetBits(GPIOA,CS_485)	 //CS-485��PA8,�շ�ʹ�ܶˣ�1����
#define CS_485_Receive  GPIO_ResetBits(GPIOA,CS_485) //CS-485��PA8,�շ�ʹ�ܶˣ�0����

void ServoMotor_Init(void);					//�ŷ������ʼ������
void rs485_init(void);	   					//485���ڳ�ʼ������
void RS485_Send_Data(u8 *buf,u8 len);      	//RS485�������ݺ���
void RS485_Receive_Data(u8 *buf,u8 *len);  	//RS485�������ݺ���
void Check_hhb(u8 DJbh);					//��ز�У�麯��
u32 Encoder_Value(u8 DJbh);					//��ȡ���������ݺ���
void ServoMotor_TurnAngle(float DJAngle,u8 DJdir,u8 DJbh); //�����ת����
float Encoder_Angle(u8 DJbh);	 			//������������ת��Ϊ�Ƕ�ֵ����
void ServoMotor_Reset(u8 DJbh);				//���ԭ�㸴�麯��

#endif
