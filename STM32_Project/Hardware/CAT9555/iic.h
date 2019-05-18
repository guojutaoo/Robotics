#ifndef _iic_H
#define _iic_H
#include "stm32f10x.h"
#include "systick.h"
/*****************声明I2C2变量及函数*****************/
#define I2C2_SCL GPIO_Pin_10	  //PB10
#define I2C2_SDA GPIO_Pin_11	  //PB11
#define GPIO_I2C2 GPIOB

#define I2C2_SCL_H GPIO_SetBits(GPIO_I2C2,I2C2_SCL)
#define I2C2_SCL_L GPIO_ResetBits(GPIO_I2C2,I2C2_SCL)

#define I2C2_SDA_H GPIO_SetBits(GPIO_I2C2,I2C2_SDA)
#define I2C2_SDA_L GPIO_ResetBits(GPIO_I2C2,I2C2_SDA)

void I2C2_INIT(void);
void I2C2_SDA_OUT(void);
void I2C2_SDA_IN(void);
void I2C2_Start(void);
void I2C2_Stop(void);
void I2C2_Ack(void);
void I2C2_NAck(void);
u8   I2C2_Wait_Ack(void);
void I2C2_Send_Byte(u8 txd);
u8   I2C2_Read_Byte(u8 ack);
/*****************声明I2C1变量及函数*****************/
#define I2C1_SCL GPIO_Pin_6	  //PB6
#define I2C1_SDA GPIO_Pin_7	  //PB7
#define GPIO_I2C1 GPIOB

#define I2C1_SCL_H GPIO_SetBits(GPIO_I2C1,I2C1_SCL)
#define I2C1_SCL_L GPIO_ResetBits(GPIO_I2C1,I2C1_SCL)

#define I2C1_SDA_H GPIO_SetBits(GPIO_I2C1,I2C1_SDA)
#define I2C1_SDA_L GPIO_ResetBits(GPIO_I2C1,I2C1_SDA)

void I2C1_INIT(void);
void I2C1_SDA_OUT(void);
void I2C1_SDA_IN(void);
void I2C1_Start(void);
void I2C1_Stop(void);
void I2C1_Ack(void);
void I2C1_NAck(void);
u8   I2C1_Wait_Ack(void);
void I2C1_Send_Byte(u8 txd);
u8   I2C1_Read_Byte(u8 ack);
#endif
