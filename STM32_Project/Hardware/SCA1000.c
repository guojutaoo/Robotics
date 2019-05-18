#include "SCA1000.h"
#include "math.h"
#include "systick.h"
#include "printf.h"

/****************************************************************************
* Function Name  : SCA1000_Init
* Description    : 初始化SCA1000的IO和SPI.
* Input          : None
* Output         : None
* Return         : None
****************************************************************************/

void SCA1000_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
	
	/* SCA1000_CS PG13 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOG, &GPIO_InitStructure);
	GPIO_SetBits(GPIOG, GPIO_Pin_13);


}
/****************************************************************************
* Function Name  : MNSPI_Init
* Description    : 初始化SPI的IO口.
****************************************************************************/
void MNSPI_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure; //声明一个结构体变量，用来初始化GPIO

	SystemInit();	//时钟初始化
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	/*  配置GPIO的模式和IO口 */
	GPIO_InitStructure.GPIO_Pin=SPI_MOSI|SPI_CLK;  //选择你要设置的IO口
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;	 //设置推挽输出模式
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	  //设置传输速率
	GPIO_Init(GPIOB,&GPIO_InitStructure); 	   /* 初始化GPIO */
   	/*  配置GPIO的模式和IO口-MISO输入 */
	GPIO_InitStructure.GPIO_Pin=SPI_MISO;  //选择你要设置的IO口
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;	//浮空输入
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	  //设置传输速率
	GPIO_Init(GPIOB,&GPIO_InitStructure); 	   /* 初始化GPIO */
}

//主机写一个BYTE到器件 
  //最终状态:CLK = LOW 
  //功能：发送控制命令
void MNSPI_WriteByte(uint8_t wdata )
{ 
   uint8_t i; 
   for(i = 0;i < 8;i++)  
   { 
	 GPIO_ResetBits(GPIOB,SPI_CLK);
     delay_us(100);
    // if((wdata&0x80)==0x80)
	  if(wdata&0x80)
     {
	   GPIO_SetBits(GPIOB, SPI_MOSI);
     }
     else
     {
	   GPIO_ResetBits(GPIOB,SPI_MOSI);
     }    
      wdata = wdata << 1; 
      GPIO_SetBits(GPIOB,SPI_CLK);
      delay_us(100);     
    } 
   GPIO_ResetBits(GPIOB,SPI_CLK); 
  }
//主机读入N个比特
//n 限制为16及其以下
//结束状态　CLK　=　0
u16 MNSPI_ReadBits(int n)
{
    u16 rtemp=0x0000,i;
    for( i = 0 ; i < n ; i++ )
    {
        rtemp= rtemp<<1;
		GPIO_ResetBits(GPIOB,SPI_CLK);     
        delay_us(100);         
        if (GPIO_ReadInputDataBit(GPIOB,SPI_MISO)==1)
        {
            rtemp = rtemp|0x0001;
        }
        else
        {
            rtemp = rtemp&0xFFFE;
        }	
		GPIO_SetBits(GPIOB,SPI_CLK);
		
        delay_us(100);
    }
    return rtemp; 
	 
}
float MNSPI_GetX(void) 
{ 
  	u16 AXData;
	float realAX;//x轴角度模拟量
    GPIO_ResetBits(GPIOB,SPI_CLK);
	GPIO_ResetBits(GPIOG,SPI_CS);
   	delay_ms(50); 
    MNSPI_WriteByte(0x10);     
    AXData = MNSPI_ReadBits(11);     
    GPIO_ResetBits(GPIOB,SPI_CLK);//时钟极性为低电平有效，即CPOL=0(下降沿发送数据，上升沿接收数据)，时钟为空闲 
	delay_ms(50); 
   	GPIO_SetBits(GPIOG,SPI_CS);
   
	       
	realAX = asin(((float)AXData-1024.0)/492)*(180/3.14);
	return realAX;
}  


