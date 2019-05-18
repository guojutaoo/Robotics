#include "ServoMotor.h"
#include "systick.h"
#include"CRC16.h"
#include "math.h"
//接收缓存区 	
u8 RS485_RX_BUF[64]; //接收缓冲,最大64个字节.
u8 RS485_RX_CNT=0;	 //接收到的数据长度
u8 flagstart=0;		//显示编码器采集数据开始标志
u8 hhbflag=1; 		//后回波校验标志


/*******************************************************************************
* 函 数 名         : rs485_init
* 函数功能		   : IO端口及串口2，时钟初始化函数	   
* TX-485接PA2，RX-485接PA3，CS-485接PA8
*******************************************************************************/
void rs485_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;	 //声明一个结构体变量，用来初始化GPIO

	USART_InitTypeDef USART_InitStructure;	 //串口结构体定义

	NVIC_InitTypeDef NVIC_InitStructure;	//中断结构体定义

	//打开时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);

	/*  配置485总线接收、发送、片选GPIO的模式和IO口 */
	//485总线发送端口
	GPIO_InitStructure.GPIO_Pin=TX_485;					//TX-485,485串口发送端口，输出PA2
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;		//复用推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	
	GPIO_Init(GPIOA,&GPIO_InitStructure);				 	
	//485总线收发片选端口
	GPIO_InitStructure.GPIO_Pin=CS_485;					//CS-485，PA8,收发使能端，0接收，1发送
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;	    //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);		   


   //485总线接收端口
	GPIO_InitStructure.GPIO_Pin=RX_485;					//RX-485，485串口接收端口输入PA3
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;	//模拟输入
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);			 /* 初始化GPIO */


	USART_InitStructure.USART_BaudRate = 9600;			   		//波特率设置为9600
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//数据长8位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;	   //1位停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;		   //无效验
	USART_InitStructure.USART_HardwareFlowControl =USART_HardwareFlowControl_None; //失能硬件流
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;		 			//开启发送和接受模式
	USART_Init(USART2, &USART_InitStructure);	   /* 初始化USART2 */
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);//使能串口2接收中断
	USART_ClearFlag(USART2,USART_FLAG_TC);	 	//清除USARTx的待处理标志位
	USART_Cmd(USART2,ENABLE);					//使能串口2

	/* 设置NVIC参数 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; 	   		//打开USART2的全局中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//抢占优先级为0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 		 	//响应优先级为1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 		 	//使能
	NVIC_Init(&NVIC_InitStructure);

}
void USART2_IRQHandler(void)
{
	u8 res;
//	USART_ClearFlag(USART2,USART_FLAG_TC);	 //不能加，加了只能发送一次数据，第二次发送不了   
	if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET)
	{	 
		res=USART_ReceiveData(USART2);			 
		if(RS485_RX_CNT<9)
		{
			RS485_RX_BUF[RS485_RX_CNT]=res;		//记录接收到的值
			RS485_RX_CNT++;						//接收数据增加1 
		} 
	}  											 
}
/*******************************************************************************
* 函 数 名         : ServoMotor_Init
* 函数功能		   : ServoMotor初始化函数，并将rs485初始化
* CA1.CB1 -	CA6.CB6	挂在PD口
* 输    出         : 无
*******************************************************************************/
void ServoMotor_Init()	  //端口初始化
{
	GPIO_InitTypeDef GPIO_InitStructure; //声明一个结构体变量，用来初始化GPIO
	SystemInit();	//时钟初始化
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);

	/*  配置GPIO的模式和IO口 */
	GPIO_InitStructure.GPIO_Pin=CA1|CB1|CA2|CB2|CA3|CB3|CA4|CB4|CA5|CB5|CA6|CB6|CA7|CB7;  //设置电机脉冲控制的IO口
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;	 	//设置推挽输出模式
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	  	//设置传输速率
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	rs485_init();	//rs485初始化
}
////****RS485发送数据函数****////
//RS485发送len个字节.
//buf:发送区首地址
//len:发送的字节数(为了和本代码的接收匹配,这里建议不要超过64个字节)
void RS485_Send_Data(u8 *buf,u8 len)
{
	u8 t;
	CS_485_Send;	  	//发送数据片选,设置为发送模式
	delay_ms(1);		//等待发送片选信号稳定
  	for(t=0;t<len;t++)	//循环发送数据
	{
		while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);//等待字节发送完成		  
		USART_SendData(USART2,buf[t]);			 				//发送数据
	}
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);	//等待字节发送完成	 
	RS485_RX_CNT=0;	  
	CS_485_Receive;	 	//设置为接收模式 
	delay_ms(1);					
}
/****RS485接收数据函数****/
//RS485查询接收到的数据
//buf:接收缓存首地址
//len:读到的数据长度
void RS485_Receive_Data(u8 *buf,u8 *len)
{
	u8 rxlen=RS485_RX_CNT;
	u8 i=0;
	*len=0;							//默认为0
	delay_ms(10);					//等待10ms,连续超过10ms没有接收到一个数据,则认为接收结束
	if(rxlen==RS485_RX_CNT&&rxlen)	//接收到了数据,且接收完成了
	{
		for(i=0;i<rxlen;i++)
		{
			buf[i]=RS485_RX_BUF[i];	
		}		
		*len=RS485_RX_CNT;			//记录本次数据长度
		RS485_RX_CNT=0;				//清零
	}
} 
/****后回波校验函数****/
//函数功能：校验伺服电机后回波响应，串口2接收到后回波响应，置位编码器数据采集开始标志位
//DJbh:电机编号
void Check_hhb(u8 DJbh)
{
	u8 hhbbuf[8]={0x01,0x08,0x00,0x00,0x01,0x02,0x60,0x5A};
	u8 buf[64];
	u8 len;
	u16 crc;
	switch(DJbh)
	{
		case 1:
			hhbbuf[0]=0x01;		//局号1
			hhbflag=1;
			break;
		case 2:
			hhbbuf[0]=0x02;	 	//局号2
			hhbflag=1;
			break;
		case 3:
			hhbbuf[0]=0x03;		//局号3
			hhbflag=1;
			break;
		case 4:
			hhbbuf[0]=0x04;		//局号4
			hhbflag=1;
			break;
		case 5:
			hhbbuf[0]=0x05;		//局号5
			hhbflag=1;
			break;
		case 6:
			hhbbuf[0]=0x06;		//局号6
			hhbflag=1;
			break;
		case 7:
			hhbbuf[0]=0x07;		//局号7
			hhbflag=1;
			break;
		default:
	  		break;
	} 		
	crc=Get_CRC16(hhbbuf,6);//计算CRC校验值	
	hhbbuf[7]=crc>>8;	 	//CRC校验高8位
	hhbbuf[6]=crc&0xff;  	//CRC校验低8位
	if(hhbflag==1)
	{	
		hhbflag=0;
		RS485_Send_Data(hhbbuf,8);	//后回波校验8个字节 									   
	}
	delay_ms(20);					//等待主机接收数据完全
	RS485_Receive_Data(buf,&len);
	if(len)							//如果485总线接收到有8个数据，则进行后回波校验
	{
		if(buf[0]==hhbbuf[0]&&buf[1]==hhbbuf[1]&&buf[2]==hhbbuf[2]&&buf[3]==hhbbuf[3]&&buf[4]==hhbbuf[4]&&buf[5]==hhbbuf[5]&&buf[6]==hhbbuf[6]&&buf[7]==hhbbuf[7])
		{
			flagstart=1; 			//置位编码器采集数据开始标志
		}
	}
}
/****读取编码器数据函数****/
//功能：监测数据帧的接收,调用功能函数,来执行主机命令的相关动作（采集编码器数据）
//DJbh:电机编号
u32 Encoder_Value(u8 DJbh)
{
	u8 	buf[64];
	u16 crc;
	u8 crch,crcl,len;
	u32 BMvalue=0; //编码器返回数据
	u8 buftxd[8]={0x01,0x03,0x10,0x06,0x00,0x02,0x20,0xCA};//串口发送字节储存数组(读取局号为1的编码器数据)
	switch(DJbh)
	{
		case 1:
			buftxd[0]=0x01;	//局号1
			break;
		case 2:
			buftxd[0]=0x02;	//局号2
			break;
		case 3:
			buftxd[0]=0x03;	//局号3
			break;
		case 4:
			buftxd[0]=0x04;	//局号4
			break;
		case 5:
			buftxd[0]=0x05;	//局号5
			break;
		case 6:
			buftxd[0]=0x06;	//局号6
			break;
		case 7:
			buftxd[0]=0x07;	//局号7
			break;
		default:
	  		break;
	} 		
	crc=Get_CRC16(buftxd,6);//计算CRC校验值	
	buftxd[7]=crc>>8;	 	//CRC校验高8位
	buftxd[6]=crc&0xff;  	//CRC校验低8位	
	if(flagstart) //编码器采集命令达到时，读取处理该命令
	{
		flagstart=0;
		RS485_Send_Data(buftxd,8);//单片机向伺服控制器发送读取编码器数据指令，8个字节
	}
	delay_ms(20);	//等待主机接收数据完全
	RS485_Receive_Data(buf,&len);
	if(len)			//如果485总线接收到有数据，则开始读取编码器数据
	{
		if(buf[0]==buftxd[0]&&buf[1]==0x03&&buf[2]==0x04) //匹配本机地址是否为buftxd[0](与函数传递进来的电机编号相同)
		{
			crc=Get_CRC16(buf,7);//计算CRC校验值	
			crch=crc>>8;
			crcl=crc&0xff;
			if((buf[7]==crcl)&&(buf[8]==crch)) //校验crc值
			{			
				BMvalue=((buf[3]<<24)|(buf[4]<<16)|(buf[5]<<8)|buf[6]);
			}			
		}
	}				
	return BMvalue;				
}
/****编码器数字量转化为当前角度值函数****/
//DJbh:电机编号；
//value:编码器采集数字量，DJAngle：电机当前位置角度值
float Encoder_Angle(u8 DJbh)
{
	float DJAngle,k;
	u32  value;	
	Check_hhb(DJbh);	// 后回波校验  
	value=Encoder_Value(DJbh);//读取编码器值 
	if(DJbh==1||DJbh==2)
	{
	 k=10.0; //每度是10个脉冲
	}
	else
	{
	 k=1000.0;//每度是1000个脉冲
	}
	if(value/1000000000==4)//电机处于顺时针转动位置
	{
	   DJAngle=(value-4294967295)/k;
	}
	else  //电机处于逆时针转动位置
	{
		DJAngle=value/k;   //值为正
	}
   return  DJAngle;
}
/****** 电机原点复归函数*****/
//DJbh:电机编号；
//3600个脉冲对应360度 ,每度是10个脉冲
//value:编码器采集数字量，DJAngle：电机旋转角度
//修改：将u32 DJAngle全部改成float型
void ServoMotor_Reset(u8 DJbh)	  
{	
	u32  value;
	float DJAngle,k;	
	Check_hhb(DJbh);	// 后回波校验  
	value=Encoder_Value(DJbh);//读取编码器值
	if(DJbh==1||DJbh==2)
	{
	 k=10.0; //每度是10个脉冲
	}
	else
	{
	 k=1000.0;//每度是1000个脉冲
	}
 	if(value/1000000000==4)//电机处于顺时针转动位置
	{
		DJAngle=(4294967295-value)/k;
		ServoMotor_TurnAngle(DJAngle,0,DJbh);	//电机1逆时针转回到原点位置
	}
	else  //电机处于逆时针转动位置
	{
		 DJAngle=value/k;
		 ServoMotor_TurnAngle(DJAngle,1,DJbh);	//电机1顺时针转回到原点位置
	}
}

/****** 电机旋转函数*****/
//DJdir:0:逆时针,1: 顺时针;DJbh:电机编号；DJAngle：电机旋转角度
//3600*100个脉冲对应360度 ,每度是1000个脉冲,DJsteps：电机旋转脉冲数
//修改：将u32 DJAngle全部改成float型
void ServoMotor_TurnAngle(float DJAngle,u8 DJdir,u8 DJbh)	  
{
	u32 i,DJsteps;
	switch(DJbh)
	{
		case 1:
			DJsteps=DJAngle*10;//将电机旋转角度转化为脉冲数，每度是1000个脉冲
			if(DJdir==1)
			{
				GPIO_SetBits(GPIOD,CB1); //电机1顺时针旋转     
			}
			else
			{
				GPIO_ResetBits(GPIOD,CB1);//电机1逆时针转动
			}
			for(i=0;i<DJsteps;i++)
			{ 
				GPIO_ResetBits(GPIOD,CA1);
				delay_us(60);
				GPIO_SetBits(GPIOD,CA1);
				delay_us(60);
			}
			break;
		case 2:
			DJsteps=DJAngle*10;//将电机旋转角度转化为脉冲数，每度是1000个脉冲
			if(DJdir==1)
			{
				GPIO_SetBits(GPIOD,CB2); //电机2顺时针旋转     
			}
			else
			{
				GPIO_ResetBits(GPIOD,CB2);//电机2逆时针转动
			}
			for(i=0;i<DJsteps;i++)
			{ 
				GPIO_ResetBits(GPIOD,CA2);
				delay_us(20);//延时约为3us
				GPIO_SetBits(GPIOD,CA2);
				delay_us(20);//延时约为3us
			}
			break;
		case 3:
			DJsteps=DJAngle*1000;//将电机旋转角度转化为脉冲数，每度是1000个脉冲
			if(DJdir==1)
			{
				GPIO_SetBits(GPIOD,CB3); //电机3顺时针旋转     
			}
			else
			{
				GPIO_ResetBits(GPIOD,CB3);//电机3逆时针转动
			}
			for(i=0;i<DJsteps;i++)
			{ 
				GPIO_ResetBits(GPIOD,CA3);
				delay_us(60);//延时约为3us
				GPIO_SetBits(GPIOD,CA3);
				delay_us(60);//延时约为3us
			}
			break;
		case 4:
			DJsteps=DJAngle*1000;//将电机旋转角度转化为脉冲数，每度是1000个脉冲
			if(DJdir==1)
			{
				GPIO_SetBits(GPIOD,CB4); //电机4顺时针旋转     
			}
			else
			{
				GPIO_ResetBits(GPIOD,CB4);//电机4逆时针转动
			}
			for(i=0;i<DJsteps;i++)
			{ 
				GPIO_ResetBits(GPIOD,CA4);
				delay_us(60);//延时约为3us
				GPIO_SetBits(GPIOD,CA4);
				delay_us(60);//延时约为3us
			}
			break;
		case 5:
			DJsteps=DJAngle*1000;//将电机旋转角度转化为脉冲数，每度是1000个脉冲
			if(DJdir==1)
			{
				GPIO_SetBits(GPIOD,CB5); //电机5顺时针旋转     
			}
			else
			{
				GPIO_ResetBits(GPIOD,CB5);//电机5逆时针转动
			}
			for(i=0;i<DJsteps;i++)
			{ 
				GPIO_ResetBits(GPIOD,CA5);
				delay_us(60);//延时约为3us
				GPIO_SetBits(GPIOD,CA5);
				delay_us(60);//延时约为3us
			}
			break;
		case 6:
			DJsteps=DJAngle*1000;//将电机旋转角度转化为脉冲数，每度是1000个脉冲
			if(DJdir==1)
			{
				GPIO_SetBits(GPIOD,CB6); //电机5顺时针旋转     
			}
			else
			{
				GPIO_ResetBits(GPIOD,CB6);//电机5逆时针转动
			}
			for(i=0;i<DJsteps;i++)
			{ 
				GPIO_ResetBits(GPIOD,CA6);
				delay_us(60);//延时约为3us
				GPIO_SetBits(GPIOD,CA6);
				delay_us(60);//延时约为3us
			}
			break;
		case 7:
			DJsteps=DJAngle*1000;//将电机旋转角度转化为脉冲数，每度是1000个脉冲
			if(DJdir==1)
			{
				GPIO_SetBits(GPIOD,CB7); //电机5顺时针旋转     
			}
			else
			{
				GPIO_ResetBits(GPIOD,CB7);//电机5逆时针转动
			}
			for(i=0;i<DJsteps;i++)
			{ 
				GPIO_ResetBits(GPIOD,CA7);
				delay_us(200);//延时约为3us
				GPIO_SetBits(GPIOD,CA7);
				delay_us(200);//延时约为3us
			}
			break;
		default:
	  		break;
	} 		
}
 

