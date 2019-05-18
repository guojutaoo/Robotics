#include"MNspi.h"
#include"systick.h"
#include "math.h"
void MNspi_init()
{
   GPIO_InitTypeDef GPIO_InitStructure;
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOG,ENABLE);

	GPIO_InitStructure.GPIO_Pin = RFSCA1000_CS2|RHSCA1000_CS2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

   	GPIO_InitStructure.GPIO_Pin = MNspi_CLK|LHSCA1000_CS1|LFSCA1000_CS1|MNspi_MOSI;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = RFSCA1000_CS1|RHSCA1000_CS1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	
	GPIO_InitStructure.GPIO_Pin = MNspi_MISO;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LFSCA1000_CS2|LHSCA1000_CS2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	LFSCA1000_CS1_SET;	
	LFSCA1000_CS2_SET;
	RFSCA1000_CS1_SET;
	RFSCA1000_CS2_SET;
	LHSCA1000_CS1_SET;
	LHSCA1000_CS2_SET;
	RHSCA1000_CS1_SET;
	RHSCA1000_CS2_SET;

	MNspi_Reset(1);
	MNspi_Reset(2);	
	MNspi_Reset(3);	
	MNspi_Reset(4);
	MNspi_Reset(5);
	MNspi_Reset(6);	
	MNspi_Reset(7);	
	MNspi_Reset(8);
}
void MNspi_Stop(int n)
{
   GPIO_ResetBits(GPIOB, MNspi_CLK);
   switch(n)
   {  
   case 1:GPIO_SetBits(GPIOB, LFSCA1000_CS1);break;
   case 2:GPIO_SetBits(GPIOG, LFSCA1000_CS2);break;
   case 3:GPIO_SetBits(GPIOB, LHSCA1000_CS1);break;
   case 4:GPIO_SetBits(GPIOG, LHSCA1000_CS2);break;
   case 5:GPIO_SetBits(GPIOC, RFSCA1000_CS1);break;
   case 6:GPIO_SetBits(GPIOA, RFSCA1000_CS2);break;
   case 7:GPIO_SetBits(GPIOC, RHSCA1000_CS1);break;
   case 8:GPIO_SetBits(GPIOA, RHSCA1000_CS2);break;
   }
   
   delay_ms(10);
}
void MNspi_Reset(int n)
{
	MNspi_Stop(n);
}
void MNspi_Start(int n)
{
	GPIO_ResetBits(GPIOB, MNspi_CLK);
	 switch(n)
   {  
   case 1:GPIO_ResetBits(GPIOB, LFSCA1000_CS1);break;
   case 2:GPIO_ResetBits(GPIOG, LFSCA1000_CS2);break;
   case 3:GPIO_ResetBits(GPIOB, LHSCA1000_CS1);break;
   case 4:GPIO_ResetBits(GPIOG, LHSCA1000_CS2);break;
   case 5:GPIO_ResetBits(GPIOC, RFSCA1000_CS1);break;
   case 6:GPIO_ResetBits(GPIOA, RFSCA1000_CS2);break;
   case 7:GPIO_ResetBits(GPIOC, RHSCA1000_CS1);break;
   case 8:GPIO_ResetBits(GPIOA, RHSCA1000_CS2);break;
   }
	delay_ms(10);
}
void MNspi_WriteByte(u8 dat)
{
   u8 i; 
   for(i=0;i<8;i++)  
   { 
     GPIO_ResetBits(GPIOB, MNspi_CLK);
     delay_us(100); 
     if(dat&0X80)
     {
      GPIO_SetBits(GPIOB, MNspi_MOSI);
     }
     else
     {
       GPIO_ResetBits(GPIOB, MNspi_MOSI);
     }    
      dat = dat << 1;         
     delay_us(100);         
      GPIO_SetBits(GPIOB, MNspi_CLK);
      delay_us(100);     
    } 
 GPIO_ResetBits(GPIOB, MNspi_CLK);
}
 int MNspi_ReadBits(int n)
{
	int rtemp=0,i;
    for(i=0;i<n;i++)
    {
       rtemp=rtemp<<1;
        GPIO_ResetBits(GPIOB, MNspi_CLK);
        delay_us(100);
        
        if (GPIO_ReadInputDataBit(GPIOD,MNspi_MISO) == 1)
        {
            rtemp = rtemp|0x0001;
        }
        else
        {
            rtemp = rtemp&0xfffe;
        } 
		 GPIO_SetBits(GPIOB, MNspi_CLK);
        delay_us(100);     
    }
    return rtemp;
}
//函数功能：返回各个角度传感器所测角度
//n:1,返回左前腿大臂传感器角度值
//  2,返回左前腿小臂传感器角度值
//  3,返回左后腿大臂传感器角度值
//  4,返回左后腿小臂传感器角度值
//  5,返回右前腿大臂传感器角度值
//  6,返回右前腿小臂传感器角度值
//  7,返回右后腿大臂传感器角度值
//  8,返回右后腿小臂传感器角度值
 int SPI_GetLXN(int n) 
{ 	int i;  
	int LXN=0; 
    float realLXN;
    MNspi_init();
    for(i=0;i<50;i++)
	{   
    MNspi_Start(n); 
    MNspi_WriteByte(LRDAX);  
    LXN+=MNspi_ReadBits(11);     
    MNspi_Stop(n); 
	delay_ms(2);
	} 
	realLXN=asin(((float)LXN/50.0-1026.0)/493)*(180/3.14);  
    return realLXN; 
}  		 

