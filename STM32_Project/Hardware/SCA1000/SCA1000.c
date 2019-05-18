#include "SCA1000.h"
#include "math.h"
#include "systick.h"
//#include "spi.h"
/*全局变量*/
float realAX1;		//清洗臂大臂角度
float realAX2;		//清洗臂小臂角度
float realAX3;		//清洗臂清洗装置角度
u8 SCAflag1=1;      //读取大臂角度数据标志位
u8 SCAflag2=1;      //读取小臂角度数据标志位
u8 SCAflag3=1;      //读取洗装置角度数据标志位

/****************************************************************************
* Function Name  : SPI2_Init
* Description    : 初始化SPI2
* Input          : None
* Output         : None
* Return         : None
****************************************************************************/
void SPI2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

    /* SPI的IO口和SPI外设打开时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

    /* SPI的IO口设置 SPI2_SCK=PB13,SPI2_MOSI=PB15,SPI2_MIS0=PB14*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15|GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
/***************************************************************************/
/************************* 设置SPI的参数 ***********************************/
/***************************************************************************/
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//选择全双工SPI模式
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;     //主机模式
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; //8位SPI
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;       //时钟悬空高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;      //在第二个时钟采集数据
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		  //Nss使用软件控制
	/* 选择波特率预分频为256 */
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;//从最高位开始传输
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStructure);
	SPI_Cmd(SPI2, ENABLE);

	SPI2_WriteReadData(0xff);//启动spi2

}      

/****************************************************************************
* Function Name  : SCA1000_Init
* Description    : 初始化SCA1000的片选IO和SPI.
SCA1000的片选IO分别挂在PE5.6.7
****************************************************************************/
void SCA1000_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	 //声明一个结构体变量，用来初始化GPIO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	
	/* 初始化SCA1000_CS1.2.3 */
	GPIO_InitStructure.GPIO_Pin = SCA1000_CS1|SCA1000_CS2|SCA1000_CS3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	/*关闭SCA1000片选1.2.3*/
	SCA1000_CS1_SET;
	SCA1000_CS2_SET;
	SCA1000_CS3_SET;
	//初始化SCA1000的SPI2
	SPI2_Init();				
}

/****************************************************************************
* Function Name  : SPI2_WriteReadData
* Description    : 使用SPI2写入一个字节数据同时读取一个字节数据。
* Input          : dat：写入的数据
* Output         : None
* Return         : 读取到的数据
*                * 读取失败返回0xFF
****************************************************************************/
u8 SPI2_WriteReadData(u8 dat)
{
	uint16_t i = 0;
    /* 当发送缓冲器空 */	
 	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
	{
		i++;
		if(i > 200)
		{
			return 0;
		}
	}
    /* 发送数据 */
   	SPI_I2S_SendData(SPI2, dat);
	i = 0;
	/* 等待接收缓冲器为非空 */
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
	{
	   	i++;
		if(i > 200)
		{
			return 0;
		}
	}
 	/* 将读取到的数值返回 */
 	return SPI_I2S_ReceiveData(SPI2);		
}

/* SCA1000读取11位数据，并将11位数字量转换为角度值 */
//同时读取大臂、小臂、清洗装置角度值
void SCA1000_ReadAXData(void)
{
	u32 AXData1=0,AXData2=0,AXData3=0,i;
	u16 readBuff[2]={0};//采集数字量缓存数组
/***************采集1次大臂角度值***************/
	if(SCAflag1==1)
	{		
		for(i=0;i<50;) //采集50次X轴数字量，取平均值
		{
			SCA1000_CS1_CLR;    		//打开片选			
			SPI2_WriteReadData(0x10);  //写读取传感器X轴数字量命令
			readBuff[0]=SPI2_WriteReadData(0xff);	//读取高8位数据
			readBuff[1]=SPI2_WriteReadData(0xff); 	//读取低8位数据
		   	if (((readBuff[0])<<3)|((readBuff[1])>>5)<1517 && ((readBuff[0])<<3)|((readBuff[1])>>5)>531 )
			{ 					
			  	AXData1=AXData1+(((readBuff[0])<<3)|((readBuff[1])>>5));//把16位数据转换成11位数据，并得到50次累加值
				i++;	   
			}
			SCA1000_CS1_SET;   //关闭片选
			delay_ms(2);
		}
		realAX1 = asin(((float)AXData1/50.0-1024.0)/491.52)*(180/3.14);
//		realAX1 = asin(((float)AXData1/50.0-1024.0)/492)*(180/3.14);
		SCAflag1=0;
		SCA1000_CS1_SET;   //关闭片选
	}
/***************采集1次小臂角度值***************/
	if(SCAflag2==1)
	{		
		for(i=0;i<50;) //采集50次X轴数字量，取平均值
		{
			SCA1000_CS2_CLR;    		//打开片选			
			SPI2_WriteReadData(0x10);   //写读取传感器X轴数字量命令
			readBuff[0]=SPI2_WriteReadData(0xff);	//读取高8位数据
			readBuff[1]=SPI2_WriteReadData(0xff); 	//读取低8位数据
		   	if (((readBuff[0])<<3)|((readBuff[1])>>5)<1517 && ((readBuff[0])<<3)|((readBuff[1])>>5)>531 )
			{ 
			  	AXData2=AXData2+(((readBuff[0])<<3)|((readBuff[1])>>5));//把16位数据转换成11位数据，并得到50次累加值
				i++;	   
			}
			SCA1000_CS2_SET;   //关闭片选
			delay_ms(2);
		}
		realAX2 = asin(((float)AXData2/50.0-1024.0)/491.52)*(180/3.14);
		SCAflag2=0;
		SCA1000_CS2_SET;   //关闭片选
	}
/***************采集1次清洗装置角度值***************/
	if(SCAflag3==1)
	{		
		for(i=0;i<50;) //采集50次X轴数字量，取平均值
		{
			SCA1000_CS3_CLR;    		//打开片选			
			SPI2_WriteReadData(0x10);   //写读取传感器X轴数字量命令
			readBuff[0]=SPI2_WriteReadData(0xff);	//读取高8位数据
			readBuff[1]=SPI2_WriteReadData(0xff); 	//读取低8位数据
		   	if (((readBuff[0])<<3)|((readBuff[1])>>5)<1517 && ((readBuff[0])<<3)|((readBuff[1])>>5)>531 )
			{ 
			  	AXData3=AXData3+(((readBuff[0])<<3)|((readBuff[1])>>5));//把16位数据转换成11位数据，并得到50次累加值
				i++;	   
			}
			SCA1000_CS3_SET;   //关闭片选
			delay_ms(2);
		}
		realAX3 = asin(((float)AXData3/50.0-1024.0)/491.52)*(180/3.14);
		SCAflag3=0;
		SCA1000_CS3_SET;   //关闭片选
	}
}

