#ifndef _ServoMotor_H
#define _ServoMotor_H
#include "stm32f10x.h"
#define CA1 GPIO_Pin_0		//PD0,电机1A相脉冲输入
#define CB1 GPIO_Pin_1		//PD1,电机1B相脉冲输入
#define CA2 GPIO_Pin_3		//PD3,电机2A相脉冲输入
#define CB2 GPIO_Pin_4		//PD4,电机2B相脉冲输入
#define CA3 GPIO_Pin_5		//PD5,电机3A相脉冲输入
#define CB3 GPIO_Pin_6		//PD6,电机3B相脉冲输入
#define CA4 GPIO_Pin_7		//PD7,电机4A相脉冲输入
#define CB4 GPIO_Pin_8		//PD8,电机4B相脉冲输入
#define CA5 GPIO_Pin_9		//PD9,电机5A相脉冲输入
#define CB5 GPIO_Pin_10		//PD10,电机5B相脉冲输入
#define CA6 GPIO_Pin_11		//PD11,电机6A相脉冲输入
#define CB6 GPIO_Pin_12		//PD12,电机6B相脉冲输入
#define CA7 GPIO_Pin_13		//PD13,电机7A相脉冲输入
#define CB7 GPIO_Pin_14		//PD14,电机7B相脉冲输入
#define TX_485 GPIO_Pin_2	//485串口发送端口
#define RX_485 GPIO_Pin_3	//485串口接收端口
#define CS_485 GPIO_Pin_8	//485串口发送/接收片选端口,PA8
#define CS_485_Send 	GPIO_SetBits(GPIOA,CS_485)	 //CS-485，PA8,收发使能端，1发送
#define CS_485_Receive  GPIO_ResetBits(GPIOA,CS_485) //CS-485，PA8,收发使能端，0接收

void ServoMotor_Init(void);					//伺服电机初始化函数
void rs485_init(void);	   					//485串口初始化函数
void RS485_Send_Data(u8 *buf,u8 len);      	//RS485发送数据函数
void RS485_Receive_Data(u8 *buf,u8 *len);  	//RS485接收数据函数
void Check_hhb(u8 DJbh);					//后回波校验函数
u32 Encoder_Value(u8 DJbh);					//读取编码器数据函数
void ServoMotor_TurnAngle(float DJAngle,u8 DJdir,u8 DJbh); //电机旋转函数
float Encoder_Angle(u8 DJbh);	 			//编码器数字量转化为角度值函数
void ServoMotor_Reset(u8 DJbh);				//电机原点复归函数

#endif
