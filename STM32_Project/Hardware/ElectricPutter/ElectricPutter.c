#include "ElectricPutter.h"
#include "systick.h"
#include "iic.h"
/*******************************************************************************
* 函 数 名         : ElectricPutter_Init
* 函数功能		   : ElectricPutter初始化函数并将I2C初始化
* NP1.PP1 -	NP8.PP8	挂在PF口，NP9.PP9 -	NP15.PP15	挂在PG口
*BUSY/SETON/INP/ESPON/ALARM挂接在I2C1上，输入口
*SETON/RESET/SVON挂接在I2C2上，输出口
* 输    出         : 无
*******************************************************************************/
void ElectricPutter_Init()	  //端口初始化
{
	GPIO_InitTypeDef GPIO_InitStructure; //声明一个结构体变量，用来初始化GPIO
	SystemInit();						 //时钟初始化
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF|RCC_APB2Periph_GPIOG,ENABLE);

	/*  配置GPIO的模式和IO口 */
	GPIO_InitStructure.GPIO_Pin=NP1|PP1|NP2|PP2|NP3|PP3|NP4|PP4|NP5|PP5|NP6|PP6|NP7|PP7|NP8|PP8;  //选择你要设置的IO口
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;	  //设置推挽输出模式
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	  //设置传输速率
	GPIO_Init(GPIOF,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=NP9|PP9|NP10|PP10|NP11|PP11|NP12|PP12|NP13|PP13|NP14|PP14|NP15|PP15;  //选择你要设置的IO口
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;	 //设置推挽输出模式
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	  //设置传输速率
	GPIO_Init(GPIOG,&GPIO_InitStructure);
	I2C2_INIT();			//I2C2初始化
	I2C1_INIT();			//I2C1初始化
}

//QJaddr为CAT9555器件iic地址,MLByte:命令字节
//对CAT9555的读操作需要两个步骤：第一.写配置寄存器，将扩展的I/O配置为输入口；第二.读输出端口寄存器
//BUSY/SETON/INP/ESPON/ALARM挂接在I2C1上，输入口
u16 CAT9555_ReadByte(u8 QJaddr)
{
 	u16 temp=0;
	I2C1_Start();   				//启动IIC总线
	I2C1_Send_Byte(QJaddr);			//发送CAT9555器件iic地址，最后一位为0写操作
	I2C1_Wait_Ack();				//等待从机应答信号
	I2C1_Send_Byte(0x06);			//发送命令字节，配置端口0
	I2C1_Wait_Ack();	   			//等待从机应答信号
	I2C1_Send_Byte(0xFF); 			//发送配置端口寄存器0值，将前八个io口配置为输入
	I2C1_Wait_Ack();				//等待从机应答信号
	I2C1_Send_Byte(0xFF); 			//发送配置端口寄存器1数据，将后八个io口配置为输入
	I2C1_Wait_Ack();				//等待从机应答信号
	I2C1_Stop();	 				//结束总线
	//读输出端口寄存器			
	I2C1_Start();  					//启动IIC总线
	I2C1_Send_Byte(QJaddr);			//发送CAT9555器件iic地址，最后一位为0写操作
	I2C1_Wait_Ack();	   			//等待从机应答信号
	I2C1_Send_Byte(0x01);			//发送命令字节，配置输入端口1
	I2C1_Wait_Ack();	   			//等待从机应答信号

	I2C1_Start();  					//重新启动IIC总线
	QJaddr=QJaddr+1;			//将CAT9555器件iic地址最后一位置为1读操作
	I2C1_Send_Byte(QJaddr);			//发送CAT9555器件iic地址，最后一位为1读操作
	I2C1_Wait_Ack();	   			//等待从机应答信号
//	temp=I2C1_Read_Byte(0); 		// 0代表 NACK，每发送一个字节，子地址自增

	temp=I2C1_Read_Byte(1); //  1   代表 ACK ,读取端口1的值
	temp<<=8;
	temp|=I2C1_Read_Byte(0); //  0  代表 NACK,读取端口0的值

	I2C1_NAck();	 //等待主机应答信号
	I2C1_Stop();	 //结束总线	
	return temp;	

}

//QJaddr为CAT9555器件iic地址,MLByte:命令字节
//对CAT9555的写操作需要两个步骤：第一.写配置寄存器，将扩展的I/O配置为输出口；第二.写输出端口寄存器
//写操作都是对SETON/RESET/SVON（输出口），I2C2
//dt0对应端口0，dt1对应端口1：0：工作，1：不工作；
void CAT9555_WriteByte(u8 QJaddr,u8 dt0,u8 dt1)
{
	//写配置寄存器
	I2C2_Start();   				//启动IIC总线
	I2C2_Send_Byte(QJaddr);			//发送CAT9555器件iic地址，最后一位为0写操作
	I2C2_Wait_Ack();				//等待从机应答信号
	I2C2_Send_Byte(0x06);			//发送命令字节，配置端口0
	I2C2_Wait_Ack();	   			//等待从机应答信号
	I2C2_Send_Byte(0x00); 			//发送配置端口寄存器0值，将前八个io口配置为输出
	I2C2_Wait_Ack();				//等待从机应答信号
	I2C2_Send_Byte(0x00); 			//发送配置端口寄存器1数据，将后八个io口配置为输出
	I2C2_Wait_Ack();				//等待从机应答信号
	I2C2_Stop();	 				//结束总线
	//写输出端口寄存器
	I2C2_Start();   				//启动IIC总线
	I2C2_Send_Byte(QJaddr);			//发送CAT9555器件iic地址，最后一位为0写操作
	I2C2_Wait_Ack();				//等待从机应答信号
	I2C2_Send_Byte(0x02);			//发送命令字节，输出端口0
	I2C2_Wait_Ack();	   			//等待从机应答信号
	I2C2_Send_Byte(dt0); 			//发送输出端口寄存器0数据
	I2C2_Wait_Ack();				//等待从机应答信号
	I2C2_Send_Byte(dt1); 			//发送输出端口寄存器1数据
	I2C2_Wait_Ack();				//等待从机应答信号
	I2C2_Stop();	 				//结束总线
	delay_ms(10); 	
}
//点动控制模式：电杆移动函数
//DGlen:电杆移动长度MM，DGdir：0后退（反转）；1前进(正转) ,DGbh:电杆编号
//脉冲控制模式：3
//说明：steps计算公式只能放在case语句中，否则电杆一直工作，且steps只能是u32或者int型变量
void ElectricPutter_DDMove(float DGlen,u8 DGdir,u8 DGbh)	  //电杆移动函数
{
   	u32 i,steps;
	switch(DGbh)
	{
		case 1:
			steps=800*DGlen/3;	//800个脉冲对应3mm，当移动len长度时对应的脉冲个数
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP1);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP1);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP1);
				delay_us(80);			//延时约为8us
				GPIO_SetBits(GPIOF,PP1);
				delay_us(80);			//延时约为8us
			}
	  	break;
		case 2:
			steps=800*DGlen/3;	//800个脉冲对应3mm，当移动len长度时对应的脉冲个数
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP2);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP2);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP2);
				delay_us(80);			//延时约为8us
				GPIO_SetBits(GPIOF,PP2);
				delay_us(80);			//延时约为8us
			}
	  	break;
		case 3:
			steps=800*DGlen/3;	//800个脉冲对应3mm，当移动len长度时对应的脉冲个数
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP3);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP3);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP3);
				delay_us(80);			//延时约为8us
				GPIO_SetBits(GPIOF,PP3);
				delay_us(80);			//延时约为8us
			}
	  	break;
		case 4:
			steps=800*DGlen/3;	//800个脉冲对应3mm，当移动len长度时对应的脉冲个数
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP4);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP4);
			}
			
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP4);
				delay_us(80);			//延时约为8us
				GPIO_SetBits(GPIOF,PP4);
				delay_us(80);			//延时约为8us
			}
	  	break;
		case 5:
			steps=800*DGlen/3;	//800个脉冲对应3mm，当移动len长度时对应的脉冲个数
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP5);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP5);
			}			
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP5);
				delay_us(80);			//延时约为8us
				GPIO_SetBits(GPIOF,PP5);
				delay_us(80);			//延时约为8us
			}
	  	break;
		case 6:
			steps=800*DGlen/3;	//800个脉冲对应3mm，当移动len长度时对应的脉冲个数
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP6);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP6);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP6);
				delay_us(80);			//延时约为8us
				GPIO_SetBits(GPIOF,PP6);
				delay_us(80);			//延时约为8us
			}
	  	break;
		case 7:
			steps=800*DGlen/3;	//800个脉冲对应3mm，当移动len长度时对应的脉冲个数
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP7);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP7);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP7);
				delay_us(80);			//延时约为8us
				GPIO_SetBits(GPIOF,PP7);
				delay_us(80);			//延时约为8us
			}
	  	break;
		case 8:
			steps=800*DGlen/3;	//800个脉冲对应3mm，当移动len长度时对应的脉冲个数
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP8);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP8);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP8);
				delay_us(80);			//延时约为8us
				GPIO_SetBits(GPIOF,PP8);
				delay_us(80);			//延时约为8us
			}
	  	break;
		case 9:
			steps=800*DGlen/3;	//800个脉冲对应3mm，当移动len长度时对应的脉冲个数
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOG,NP9);      
			}
			else
			{
				GPIO_ResetBits(GPIOG,NP9);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOG,PP9);
				delay_us(80);			//延时约为8us
				GPIO_SetBits(GPIOG,PP9);
				delay_us(80);			//延时约为8us
			}
	  	break;
		case 10:
			steps=800*DGlen/3;	//800个脉冲对应3mm，当移动len长度时对应的脉冲个数
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOG,NP10);      
			}
			else
			{
				GPIO_ResetBits(GPIOG,NP10);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOG,PP10);
				delay_us(80);			//延时约为8us
				GPIO_SetBits(GPIOG,PP10);
				delay_us(80);			//延时约为8us
			}
	  	break;
		case 11:
			steps=800*DGlen/3;	//800个脉冲对应3mm，当移动len长度时对应的脉冲个数
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOG,NP11);      
			}
			else
			{
				GPIO_ResetBits(GPIOG,NP11);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOG,PP11);
				delay_us(80);			//延时约为8us
				GPIO_SetBits(GPIOG,PP11);
				delay_us(80);			//延时约为8us
			}									  
	  	break;
		case 12:
			steps=800*DGlen/3;	//800个脉冲对应3mm，当移动len长度时对应的脉冲个数
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOG,NP12);      
			}
			else
			{
				GPIO_ResetBits(GPIOG,NP12);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOG,PP12);
				delay_us(80);			//延时约为8us
				GPIO_SetBits(GPIOG,PP12);
				delay_us(80);			//延时约为8us
			}
	  	break;
		case 13:
			steps=800*DGlen/4;	//800个脉冲对应4mm，当移动len长度时对应的脉冲个数
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOG,NP13);      
			}
			else
			{
				GPIO_ResetBits(GPIOG,NP13);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOG,PP13);
				delay_us(80);			//延时约为8us
				GPIO_SetBits(GPIOG,PP13);
				delay_us(80);			//延时约为8us
			}
	  	break;
		case 14:
			steps=800*DGlen/3;	//800个脉冲对应3mm，当移动len长度时对应的脉冲个数
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOG,NP14);      
			}
			else
			{
				GPIO_ResetBits(GPIOG,NP14);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOG,PP14);
				delay_us(80);			//延时约为8us
				GPIO_SetBits(GPIOG,PP14);
				delay_us(80);			//延时约为8us
			}
	  	break;
		case 15:
			steps=800*DGlen/3;	//800个脉冲对应3mm，当移动len长度时对应的脉冲个数
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOG,NP15);      
			}
			else
			{
				GPIO_ResetBits(GPIOG,NP15);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOG,PP15);
				delay_us(80);			//延时约为8us
				GPIO_SetBits(GPIOG,PP15);
				delay_us(80);			//延时约为8us
			}
	  	break;
		default:
	  	break;	
	}
}


//电杆移动函数
//DGlen:电杆移动长度MM，DGdir：0后退（反转）；1前进(正转) ,DGbh:电杆编号
//脉冲控制模式：3
//说明：steps计算公式只能放在case语句中，否则电杆一直工作，且steps只能是u32或者int型变量
void ElectricPutter_Move(float DGlen,u8 DGdir,u8 DGbh)	  //电杆移动函数
{
   	u32 i,steps;
	switch(DGbh)
	{
		case 1:
			steps=800*DGlen/3;	//800个脉冲对应3mm，当移动len长度时对应的脉冲个数
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP1);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP1);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP1);
				delay_us(8);			//延时约为8us
				GPIO_SetBits(GPIOF,PP1);
				delay_us(8);			//延时约为8us
			}
	  	break;
		case 2:
			steps=800*DGlen/3;	//800个脉冲对应3mm，当移动len长度时对应的脉冲个数
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP2);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP2);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP2);
				delay_us(8);			//延时约为8us
				GPIO_SetBits(GPIOF,PP2);
				delay_us(8);			//延时约为8us
			}
	  	break;
		case 3:
			steps=800*DGlen/3;	//800个脉冲对应3mm，当移动len长度时对应的脉冲个数
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP3);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP3);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP3);
				delay_us(8);			//延时约为8us
				GPIO_SetBits(GPIOF,PP3);
				delay_us(8);			//延时约为8us
			}
	  	break;
		case 4:
			steps=800*DGlen/3;	//800个脉冲对应3mm，当移动len长度时对应的脉冲个数
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP4);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP4);
			}
			
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP4);
				delay_us(8);			//延时约为8us
				GPIO_SetBits(GPIOF,PP4);
				delay_us(8);			//延时约为8us
			}
	  	break;
		case 5:
			steps=800*DGlen/3;	//800个脉冲对应3mm，当移动len长度时对应的脉冲个数
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP5);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP5);
			}			
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP5);
				delay_us(8);			//延时约为8us
				GPIO_SetBits(GPIOF,PP5);
				delay_us(8);			//延时约为8us
			}
	  	break;
		case 6:
			steps=800*DGlen/3;	//800个脉冲对应3mm，当移动len长度时对应的脉冲个数
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP6);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP6);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP6);
				delay_us(8);			//延时约为8us
				GPIO_SetBits(GPIOF,PP6);
				delay_us(8);			//延时约为8us
			}
	  	break;
		case 7:
			steps=800*DGlen/3;	//800个脉冲对应3mm，当移动len长度时对应的脉冲个数
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP7);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP7);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP7);
				delay_us(8);			//延时约为8us
				GPIO_SetBits(GPIOF,PP7);
				delay_us(8);			//延时约为8us
			}
	  	break;
		case 8:
			steps=800*DGlen/3;	//800个脉冲对应3mm，当移动len长度时对应的脉冲个数
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP8);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP8);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP8);
				delay_us(8);			//延时约为8us
				GPIO_SetBits(GPIOF,PP8);
				delay_us(8);			//延时约为8us
			}
	  	break;
		case 9:
			steps=800*DGlen/3;	//800个脉冲对应3mm，当移动len长度时对应的脉冲个数
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOG,NP9);      
			}
			else
			{
				GPIO_ResetBits(GPIOG,NP9);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOG,PP9);
				delay_us(8);			//延时约为8us
				GPIO_SetBits(GPIOG,PP9);
				delay_us(8);			//延时约为8us
			}
	  	break;
		case 10:
			steps=800*DGlen/3;	//800个脉冲对应3mm，当移动len长度时对应的脉冲个数
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOG,NP10);      
			}
			else
			{
				GPIO_ResetBits(GPIOG,NP10);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOG,PP10);
				delay_us(8);			//延时约为8us
				GPIO_SetBits(GPIOG,PP10);
				delay_us(8);			//延时约为8us
			}
	  	break;
		case 11:
			steps=800*DGlen/3;	//800个脉冲对应3mm，当移动len长度时对应的脉冲个数
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOG,NP11);      
			}
			else
			{
				GPIO_ResetBits(GPIOG,NP11);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOG,PP11);
				delay_us(8);			//延时约为8us
				GPIO_SetBits(GPIOG,PP11);
				delay_us(8);			//延时约为8us
			}									  
	  	break;
		case 12:
			steps=800*DGlen/3;	//800个脉冲对应3mm，当移动len长度时对应的脉冲个数
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOG,NP12);      
			}
			else
			{
				GPIO_ResetBits(GPIOG,NP12);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOG,PP12);
				delay_us(8);			//延时约为8us
				GPIO_SetBits(GPIOG,PP12);
				delay_us(8);			//延时约为8us
			}
	  	break;
		case 13:
			steps=800*DGlen/4;	//800个脉冲对应4mm，当移动len长度时对应的脉冲个数
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOG,NP13);      
			}
			else
			{
				GPIO_ResetBits(GPIOG,NP13);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOG,PP13);
				delay_us(800);			//延时约为8us
				GPIO_SetBits(GPIOG,PP13);
				delay_us(800);			//延时约为8us
			}
	  	break;
		case 14:
			steps=800*DGlen/3;	//800个脉冲对应3mm，当移动len长度时对应的脉冲个数
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOG,NP14);      
			}
			else
			{
				GPIO_ResetBits(GPIOG,NP14);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOG,PP14);
				delay_us(20);			//延时约为8us
				GPIO_SetBits(GPIOG,PP14);
				delay_us(20);			//延时约为8us
			}
	  	break;
		case 15:
			steps=800*DGlen/3;	//800个脉冲对应3mm，当移动len长度时对应的脉冲个数
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOG,NP15);      
			}
			else
			{
				GPIO_ResetBits(GPIOG,NP15);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOG,PP15);
				delay_us(20);			//延时约为8us
				GPIO_SetBits(GPIOG,PP15);
				delay_us(20);			//延时约为8us
			}
	  	break;
			case 16:
			steps=800*DGlen/3;	//800个脉冲对应3mm，当移动len长度时对应的脉冲个数
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP7);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP7);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP7);
				delay_us(500);			//延时约为8us
				GPIO_SetBits(GPIOF,PP7);
				delay_us(500);			//延时约为8us
			}
	  	break;
		case 17:
			steps=800*DGlen/3;	//800个脉冲对应3mm，当移动len长度时对应的脉冲个数
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP8);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP8);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP8);
				delay_us(500);			//延时约为8us
				GPIO_SetBits(GPIOF,PP8);
				delay_us(500);			//延时约为8us
			}
	  	break;
		default:
	  	break;	
	}
}





