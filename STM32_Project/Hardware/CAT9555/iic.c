#include "iic.h"
/*****************I2C2��غ���*****************/
void I2C2_INIT()
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	GPIO_InitStructure.GPIO_Pin=I2C2_SCL|I2C2_SDA;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	I2C2_SCL_H;
	I2C2_SDA_H;
}
void I2C2_SDA_OUT()
{
    GPIO_InitTypeDef GPIO_InitStructure;	
	
	GPIO_InitStructure.GPIO_Pin=I2C2_SDA;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}

void I2C2_SDA_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	
	GPIO_InitStructure.GPIO_Pin=I2C2_SDA;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}

//������ʼ�ź�
void I2C2_Start(void)
{
    I2C2_SDA_OUT();
	
	I2C2_SDA_H;
	I2C2_SCL_H;
	delay_us(5);
	I2C2_SDA_L;
	delay_us(6);
	I2C2_SCL_L;
}

//����ֹͣ�ź�
void I2C2_Stop(void)
{
   I2C2_SDA_OUT();

   I2C2_SCL_L;
   I2C2_SDA_L;
   I2C2_SCL_H;
   delay_us(6);
   I2C2_SDA_H;
   delay_us(6);
}

//��������Ӧ���ź�ACK
void I2C2_Ack(void)
{
   I2C2_SCL_L;
   I2C2_SDA_OUT();
   I2C2_SDA_L;
   delay_us(2);
   I2C2_SCL_H;
   delay_us(5);
   I2C2_SCL_L;
}

//����������Ӧ���ź�NACK
void I2C2_NAck(void)
{
   I2C2_SCL_L;
   I2C2_SDA_OUT();
   I2C2_SDA_H;
   delay_us(2);
   I2C2_SCL_H;
   delay_us(5);
   I2C2_SCL_L;
}
//�ȴ��ӻ�Ӧ���ź�
//����ֵ��1 ����Ӧ��ʧ��
//		  0 ����Ӧ��ɹ�
u8 I2C2_Wait_Ack(void)
{
	u8 tempTime=0;

	I2C2_SDA_IN();

	I2C2_SDA_H;
	delay_us(1);
	I2C2_SCL_H;
	delay_us(1);

	while(GPIO_ReadInputDataBit(GPIO_I2C2,I2C2_SDA))
	{
		tempTime++;
		if(tempTime>250)
		{
			I2C2_Stop();
			return 1;
		}	 
	}

	I2C2_SCL_L;
	return 0;
}
//I2C2 ����һ���ֽ�
void I2C2_Send_Byte(u8 txd)
{
	u8 i=0;

	I2C2_SDA_OUT();
	I2C2_SCL_L;//����ʱ�ӿ�ʼ���ݴ���

	for(i=0;i<8;i++)
	{
		if((txd&0x80)>0) //0x80  1000 0000
			I2C2_SDA_H;
		else
			I2C2_SDA_L;

		txd<<=1;
		I2C2_SCL_H;
		delay_us(2); //��������
		I2C2_SCL_L;
		delay_us(2);
	}
}

//I2C2 ��ȡһ���ֽ�

u8 I2C2_Read_Byte(u8 ack)
{
   u8 i=0,receive=0;

   I2C2_SDA_IN();
   for(i=0;i<8;i++)
   {
   		I2C2_SCL_L;
		delay_us(2);
		I2C2_SCL_H;
		receive<<=1;
		if(GPIO_ReadInputDataBit(GPIO_I2C2,I2C2_SDA))
		   receive++;
		delay_us(1);	
   }

   	if(ack==0)
	   	I2C2_NAck();
	else
		I2C2_Ack();

	return receive;
}
/*****************I2C1��غ���*****************/

void I2C1_INIT()
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	GPIO_InitStructure.GPIO_Pin=I2C1_SCL|I2C1_SDA;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	I2C1_SCL_H;
	I2C1_SDA_H;
}
void I2C1_SDA_OUT()
{
    GPIO_InitTypeDef GPIO_InitStructure;	
	
	GPIO_InitStructure.GPIO_Pin=I2C1_SDA;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}

void I2C1_SDA_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	
	GPIO_InitStructure.GPIO_Pin=I2C1_SDA;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}

//������ʼ�ź�
void I2C1_Start(void)
{
    I2C1_SDA_OUT();
	
	I2C1_SDA_H;
	I2C1_SCL_H;
	delay_us(5);
	I2C1_SDA_L;
	delay_us(6);
	I2C1_SCL_L;
}

//����ֹͣ�ź�
void I2C1_Stop(void)
{
   I2C1_SDA_OUT();

   I2C1_SCL_L;
   I2C1_SDA_L;
   I2C1_SCL_H;
   delay_us(6);
   I2C1_SDA_H;
   delay_us(6);
}

//��������Ӧ���ź�ACK
void I2C1_Ack(void)
{
   I2C1_SCL_L;
   I2C1_SDA_OUT();
   I2C1_SDA_L;
   delay_us(2);
   I2C1_SCL_H;
   delay_us(5);
   I2C1_SCL_L;
}

//����������Ӧ���ź�NACK
void I2C1_NAck(void)
{
   I2C1_SCL_L;
   I2C1_SDA_OUT();
   I2C1_SDA_H;
   delay_us(2);
   I2C1_SCL_H;
   delay_us(5);
   I2C1_SCL_L;
}
//�ȴ��ӻ�Ӧ���ź�
//����ֵ��1 ����Ӧ��ʧ��
//		  0 ����Ӧ��ɹ�
u8 I2C1_Wait_Ack(void)
{
	u8 tempTime=0;

	I2C1_SDA_IN();

	I2C1_SDA_H;
	delay_us(1);
	I2C1_SCL_H;
	delay_us(1);

	while(GPIO_ReadInputDataBit(GPIO_I2C1,I2C1_SDA))
	{
		tempTime++;
		if(tempTime>250)
		{
			I2C1_Stop();
			return 1;
		}	 
	}

	I2C1_SCL_L;
	return 0;
}
//I2C1 ����һ���ֽ�
void I2C1_Send_Byte(u8 txd)
{
	u8 i=0;

	I2C1_SDA_OUT();
	I2C1_SCL_L;//����ʱ�ӿ�ʼ���ݴ���

	for(i=0;i<8;i++)
	{
		if((txd&0x80)>0) //0x80  1000 0000
			I2C1_SDA_H;
		else
			I2C1_SDA_L;

		txd<<=1;
		I2C1_SCL_H;
		delay_us(2); //��������
		I2C1_SCL_L;
		delay_us(2);
	}
}

//I2C1 ��ȡһ���ֽ�

u8 I2C1_Read_Byte(u8 ack)
{
   u8 i=0,receive=0;

   I2C1_SDA_IN();
   for(i=0;i<8;i++)
   {
   		I2C1_SCL_L;
		delay_us(2);
		I2C1_SCL_H;
		receive<<=1;
		if(GPIO_ReadInputDataBit(GPIO_I2C1,I2C1_SDA))
		   receive++;
		delay_us(1);	
   }

   	if(ack==0)
	   	I2C1_NAck();
	else
		I2C1_Ack();

	return receive;
}
