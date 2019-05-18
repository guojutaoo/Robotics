#ifndef _VacuumGenerator_H
#define _VacuumGenerator_H
#include "stm32f10x.h"

#define PresureSensor1 GPIO_Pin_0		//压力传感器1模拟量输入口，PC0
#define PresureSensor2 GPIO_Pin_1		//压力传感器2模拟量输入口，PC1
#define PresureSensor3 GPIO_Pin_2		//压力传感器3模拟量输入口，PC2
#define PresureSensor4 GPIO_Pin_3		//压力传感器4模拟量输入口，PC3

#define SupplyValve1  GPIO_Pin_8		//真空发生器供给阀1，PE8
#define SupplyValve2  GPIO_Pin_9		//真空发生器供给阀2，PE9
#define SupplyValve3  GPIO_Pin_10		//真空发生器供给阀3，PE10
#define SupplyValve4  GPIO_Pin_11		//真空发生器供给阀4，PE11
#define ReleaseValve1  GPIO_Pin_12		//真空发生器释放阀1，PE12
#define ReleaseValve2  GPIO_Pin_13		//真空发生器释放阀2，PE13
#define ReleaseValve3  GPIO_Pin_14		//真空发生器释放阀3，PE14
#define ReleaseValve4  GPIO_Pin_15		//真空发生器释放阀3，PE15

void adc_init(void);					//adc初始化函数
void VacuumGenerator_init(void);		//真空发生器初始化函数
float  Pressure_convert(void);			//压力转换函数
#endif
