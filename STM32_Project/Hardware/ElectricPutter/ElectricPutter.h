#ifndef _ElectricPutter_H
#define _ElectricPutter_H
#include "stm32f10x.h"
#define NP1 GPIO_Pin_0		//PF0,电杠1NP相脉冲输入
#define PP1 GPIO_Pin_1		//PF1,电杠1PP相脉冲输入
#define NP2 GPIO_Pin_2		//PF2,电杠2NP相脉冲输入
#define PP2 GPIO_Pin_3		//PF3,电杠2PP相脉冲输入
#define NP3 GPIO_Pin_4		//PF4,电杠3NP相脉冲输入
#define PP3 GPIO_Pin_5		//PF5,电杠3PP相脉冲输入
#define NP4 GPIO_Pin_6		//PF6,电杠4NP相脉冲输入
#define PP4 GPIO_Pin_7		//PF7,电杠4PP相脉冲输入
#define NP5 GPIO_Pin_8		//PF8,电杠5NP相脉冲输入
#define PP5 GPIO_Pin_9		//PF9,电杠5PP相脉冲输入
#define NP6 GPIO_Pin_10		//PF10,电杠6NP相脉冲输入
#define PP6 GPIO_Pin_11		//PF11,电杠6PP相脉冲输入
#define NP7 GPIO_Pin_12		//PF12,电杠7NP相脉冲输入
#define PP7 GPIO_Pin_13		//PF13,电杠7PP相脉冲输入
#define NP8 GPIO_Pin_14		//PF14,电杠8NP相脉冲输入
#define PP8 GPIO_Pin_15		//PF15,电杠8PP相脉冲输入

#define NP9  GPIO_Pin_0		//PG0,电杠9NP相脉冲输入
#define PP9  GPIO_Pin_1		//PG1,电杠9PP相脉冲输入
#define NP10 GPIO_Pin_2		//PG2,电杠10NP相脉冲输入
#define PP10 GPIO_Pin_3		//PG3,电杠10PP相脉冲输入
#define NP11 GPIO_Pin_4		//PG4,电杠11NP相脉冲输入
#define PP11 GPIO_Pin_5		//PG5,电杠11PP相脉冲输入
#define NP12 GPIO_Pin_6		//PG6,电杠12NP相脉冲输入
#define PP12 GPIO_Pin_7		//PG7,电杠12PP相脉冲输入
#define NP13 GPIO_Pin_8		//PG8,电杠13NP相脉冲输入
#define PP13 GPIO_Pin_9		//PG9,电杠13PP相脉冲输入
#define NP14 GPIO_Pin_10	//PG10,电杠14NP相脉冲输入
#define PP14 GPIO_Pin_11	//PG11,电杠14PP相脉冲输入
#define NP15 GPIO_Pin_12	//PG12,电杠15NP相脉冲输入
#define PP15 GPIO_Pin_13	//PG13,电杠15PP相脉冲输入
//挂在iic2上
#define SVON    0x44   		//电杆伺服on对应io扩展芯片地址，最后一位都置0（写操作）
#define RESET   0x42   		//电杆报警清除对应io扩展芯片地址，最后一位都置0（写操作）
#define SETUP   0x40   		//电杆原点回归对应io扩展芯片地址，最后一位都置0（写操作）
//挂在iic1上
#define BUSY    0x40   		//电杆运行中对应io扩展芯片地址，最后一位都置0（写操作）
#define SETON   0x42   		//电杆原点回归完成对应io扩展芯片地址，最后一位都置0（写操作）
#define INP    	0x44   		//电杆定位完成对应io扩展芯片地址，最后一位都置0（写操作）
#define ESTOP   0x48   		//电杆紧急停止对应io扩展芯片地址，最后一位都置0（写操作）
#define ALARM   0x46   		//电杆报警对应io扩展芯片地址，最后一位都置0（写操作）


void ElectricPutter_Init(void);	  //端口初始化
u16 CAT9555_ReadByte(u8 QJaddr); //CAT9555的读操作
void CAT9555_WriteByte(u8 QJaddr,u8 dt0,u8 dt1);//CAT9555的写操作
void ElectricPutter_DDMove(float DGlen,u8 DGdir,u8 DGbh);
void ElectricPutter_Move(float DGlen,u8 DGdir,u8 DGbh);	  //电杆移动函数
//void DG_Reverse(u32 DGPulse);
#endif



