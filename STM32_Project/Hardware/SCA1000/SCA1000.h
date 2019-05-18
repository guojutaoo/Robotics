#ifndef _SCA1000_H
#define _SCA1000_H
#include "stm32f10x.h"
/*声明全局变量*/
extern float realAX1;		//清洗臂大臂角度
extern float realAX2;		//清洗臂小臂角度
extern float realAX3;		//清洗臂清洗装置角度
/* 定义SCA1000 使用的片选IO口 */
#define SCA1000_CS1 GPIO_Pin_5		//SCA1000_CS1片选接在PE5端口
#define SCA1000_CS2 GPIO_Pin_6		//SCA1000_CS2片选接在PE6端口
#define SCA1000_CS3 GPIO_Pin_7		//SCA1000_CS3片选接在PE7端口

#define SCA1000_CS1_SET GPIO_SetBits(GPIOE, GPIO_Pin_5)		//关闭片选1
#define SCA1000_CS1_CLR GPIO_ResetBits(GPIOE, GPIO_Pin_5)	//打开片选1
#define SCA1000_CS2_SET GPIO_SetBits(GPIOE, GPIO_Pin_6)		//关闭片选2
#define SCA1000_CS2_CLR GPIO_ResetBits(GPIOE, GPIO_Pin_6)	//打开片选2
#define SCA1000_CS3_SET GPIO_SetBits(GPIOE, GPIO_Pin_7)	 	//关闭片选2
#define SCA1000_CS3_CLR GPIO_ResetBits(GPIOE, GPIO_Pin_7)	//打开片选2

/* 定义指令表 */
#define RDAX    0x10   //读X轴角度控制指令
/* 声明外部调用函数 */
void SCA1000_Init(void);  		//SCA1000初始化
void SCA1000_ReadAXData(void);  //SCA1000读取X轴角度
void SPI2_Init(void); 			//SPI2初始化		
u8 SPI2_WriteReadData(u8 dat);	//SPI2写入一个字节数据同时读取一个字节数据
#endif

