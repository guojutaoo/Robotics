#ifndef _usart_232_H
#define _usart_232_H
#include "stm32f10x.h"

extern u8 CleanArmPathflag;		//清洗臂清按照上位机清洗轨迹清洗开始标志
extern u8 PointMoveflag;     	//点动控制模式开始标志，1：启动点动模式
extern u8 StraightWalkFlag;
extern u8 ForwordWalkFlag;
extern u8 BackwordWalkFlag;
extern u8 LTurnMoveFlag;
extern u8 RTurnMoveFlag;
void usart_232_init(void);							//串口1初始化函数
void RS232_Send_Data(u8 *buf,u8 len);			//RS232发送数据函数
void RS232_Receive_Data(u8 *buf,u8 *len);		//RS232接收数据函数
void bytetofloat(u8 *buf,float *Repara,u8 len);	//数据类型转换：4个字节型数据（4*8位）转换一个浮点型数据（32位）
float Hex_To_Decimal(u8 *Byte);			 		//十六进制转化为浮点数
void FloatToByte(float floatNum,u8 *byteArry);  //浮点数到十六进制转换
void StraightWalk_function(int m);						//数组缓存
void DD_functions(void);						//点动控制模式：客户端下发指令下位机接收指令对应功能操作 
void CleanArmPathPlan_functions(void);					//清洗臂路径规划控制模式：客户端下发指令下位机接收指令对应功能操作 
void BufStorage(u8 len);						//数组缓存
void SendDataToPC(float state);					//发送函数：下位机将机器人各关节的状态实时发送给上位机
void CleanArm_Postion(void); 					//清洗臂末端当前位置采集函数
void TurnWalk_function(int n);

#endif
