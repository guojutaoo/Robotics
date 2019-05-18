#include "usart_232.h"
#include "systick.h"
#include "ElectricPutter.h"
#include "ServoMotor.h"
#include "CleanArm.h"
#include "printf.h"
#include "BodyMove.h"
#include "math.h"
u8 RS232_RX_BUF[64]; 	//接收缓冲,最大64个字节.
u8 RS232_RX_CNT=0;	 	//接收到的数据长度
u8 CleanArmPathflag=0;  //清洗臂清按照上位机清洗轨迹清洗开始标志，1：开始清洗
u8 PointMoveflag=0;     //点动控制模式开始标志，1：启动点动模式
u8 StraightWalkFlag=0;
u8 ForwordWalkFlag=0;
u8 BackwordWalkFlag=0;
u8 LTurnMoveFlag=0;
u8 RTurnMoveFlag=0;
u16 CleanArmPoints=0;  //上位机发送清洗臂清晰轨迹坐标点数
int FPointsNum,FRunCycle;
int BPointsNum,BRunCycle;
int LTPointsNum,LTurnCycle;
int RTPointsNum,RTurnCycle;
float LTRoundAngle;
float RTRoundAngle;
float LTCompenLen;
float RTCompenLen;
float FWalkAngle[4];
float BWalkAngle[4];
float LTWalkAngle[4];
float RTWalkAngle[4];
float CleanArmbuf[800]; //清洗臂坐标点储存数组缓存区，上位机最多传送200*3个浮点型数据，数据最大是800mm
float FWalkPoints[800];
float BWalkPoints[800];
float LTWalkPoints[800];
float RTWalkPoints[800];    
//static float fx[4]={4.34,4.29,4.23,4.17};
//static float fy[4]={3.16,3.24,3.31,3.39};
//float Repara[11];       //11个单精度浮点型参数
/*******************************************************************************
 *RS232串口通讯系列函数
*******************************************************************************/
void usart_232_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;	 //声明一个结构体变量，用来初始化GPIO

	USART_InitTypeDef USART_InitStructure;	 //串口结构体定义

	NVIC_InitTypeDef NVIC_InitStructure;	//中断结构体定义

	//打开时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB,ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	 //打开时钟

	//	/*  配置GPIO的模式和IO口 */
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;//TX			   //串口输出PA9
//	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;//普中单片机测试232串口
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;	    //复用推挽输出
	GPIO_Init(GPIOA,&GPIO_InitStructure);  /* 初始化串口输入IO */
//	GPIO_Init(GPIOB,&GPIO_InitStructure);  //普中单片机测试232串口	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;//RX			 //串口输入PA10
//	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;//TX		  //普中单片机测试232串口	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;		  //模拟输入
	GPIO_Init(GPIOA,&GPIO_InitStructure); /* 初始化GPIO */
//	GPIO_Init(GPIOB,&GPIO_InitStructure);//普中单片机测试232串口		   


	USART_InitStructure.USART_BaudRate = 9600;			   		//波特率设置为9600
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//数据长8位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;	   //1位停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;		   //无效验
	USART_InitStructure.USART_HardwareFlowControl =USART_HardwareFlowControl_None; //失能硬件流
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;		 			//开启发送和接受模式
	USART_Init(USART1, &USART_InitStructure);	   /* 初始化USART2 */
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//使能串口2接收中断
	USART_ClearFlag(USART1,USART_FLAG_TC);	 	//清除USARTx的待处理标志位
	USART_Cmd(USART1,ENABLE);					//使能串口2
	
	/* 设置NVIC参数 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; 	   		//打开USART3的全局中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//抢占优先级为0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 		 	//响应优先级为2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 		 	//使能
	NVIC_Init(&NVIC_InitStructure);		 

}
void USART1_IRQHandler(void)	//串口1中断函数
{
	u8 res;
	if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET) //RXNE(读数据寄存器非空)当该位被置 1的时候，表示已经有数据被接收到了
	{
		res=USART_ReceiveData(USART1);			 
		if(RS232_RX_CNT<46)
		{
			RS232_RX_BUF[RS232_RX_CNT]=res;		//记录接收到的值
			RS232_RX_CNT++;						//接收数据增加1
			if(RS232_RX_BUF[0]==0xB5)
			{
			 	if(	RS232_RX_CNT==30)		  //清洗臂轨迹规划与机器人直线运动时上位机发送30个字节
				{
					//注意：当缓存数组函数放在中断中时，上位机发送的指令必须是30个字节，
					//其中只要有一个不是30个字节整个程序就不会执行必须重新复位重新发送指令才可以正常使用
					//当把储存函数放在main的while(1)中时不会出现上述漏洞
					//放在中断中主要问题是当下位机接收到一帧数据(任意数量字节)时未将RS232_RX_CNT和buf数组清零	
					  BufStorage(7); 					//将上位机发送的数据储存到CleanArmbuf[256]中
				}
			}						 
			if(RS232_RX_BUF[0]==0xA5)
			{
				if(	RS232_RX_CNT==46)		  //点动控制模式时上位机发送46个字节
				{
//					//注意：当函数放在中断中时，每接收一次中断执行一次程序，放在main的while(1)可不停执行程序
//					//上位机发送的指令必须是46个字节，其中只要有一个不是46个字节整个程序就不会执行必须重新复位重新发送指令才可以正常使用
//					//当把函数放在main的while(1)中时不会出现上述漏洞
//					DD_functions();	    //点动控制模式：客户端下发指令下位机接收指令对应功能操作
					PointMoveflag=1;	//1：启动点动模式
				}
			}
		}	
	}
}
////****RS232发送数据函数****////
//RS232发送len个字节.
//buf:发送区首地址
//len:发送的字节数
void RS232_Send_Data(u8 *buf,u8 len)
{
	u8 t;
  	for(t=0;t<len;t++)	//循环发送数据
	{
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);//等待字节发送完成		  
		USART_SendData(USART1,buf[t]);//发送数据
		delay_ms(2);	   //发送数据延时，否则发送数据丢失
	}
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);	//等待字节发送完成,TC发送完成标志	 
	RS232_RX_CNT=0;	  //清零
}

/****RS232接收数据函数****/
//RS232查询接收到的数据
//buf:接收缓存首地址
//len:读到的数据长度
void RS232_Receive_Data(u8 *buf,u8 *len)
{
	u8 rxlen=RS232_RX_CNT;
	u8 i=0;
	*len=0;							//默认为0
	delay_ms(10);
	if(rxlen==RS232_RX_CNT&&rxlen)	//接收到了数据,且接收完成了
	{
		for(i=0;i<rxlen;i++)
		{
			buf[i]=RS232_RX_BUF[i];
		}		
		*len=RS232_RX_CNT;			//记录本次数据长度
		RS232_RX_CNT=0;				//清零
	}
}


//数据类型转换：4个字节型数据（4*8位）转换一个浮点型数据（32位）
//buf[]:将上位机发送过来数据缓存区
//Repara[]:函数功能位，将4字节数据转换为一个浮点型数据
//len：Repara[]函数功能位数组长度
void bytetofloat(u8 *buf,float *Repara,u8 len)
{  
  u8 i=0,j=0,hexbyte[4],bufstart,bufend;
  switch(len)//按功能进行操作,根据不同的功能将包头包尾赋值
  {
    //点动控制模式，2+4*11=46个字节
 	case 11:
		bufstart=0xA5;	//包头0xA5
		bufend=0x5A;	//包尾0x5A		
	    break;
	//清洗轨迹规划控制模式，2+4*7=30个字节
 	case 7:
		bufstart=0xB5;	//包头0xB5
		bufend=0x5B;	//包尾0x5B					
	    break;
	default:
		break;
  } 	
  //包头bufstart,包尾bufend
  if(buf[0]==bufstart && buf[1+len*4]==bufend)
  {
    for(i=0;i<len;i++)
    {
      j=i*4;
	  hexbyte[0]=buf[j+1];
	  hexbyte[1]=buf[j+2];
	  hexbyte[2]=buf[j+3];
	  hexbyte[3]=buf[j+4];   
      Repara[i]=Hex_To_Decimal(hexbyte);    
    }
  }
}
//十六进制转化为浮点数
float Hex_To_Decimal(u8 *Byte)
{
  return *((float*)Byte);
}
//浮点数到十六进制转换
void FloatToByte(float floatNum,u8 *byteArry)
{
	u8 i;
    u8* pchar=(u8*)&floatNum;
    for(i=0;i<4;i++)
    {
		*byteArry=*pchar;
		pchar++;
		byteArry++;	
    }
}
/*******************************************************************************
 *清洗臂与上位机通讯相关函数
*******************************************************************************/
//清洗臂根据上位机指令完成路径规划对应操作 
void CleanArmPathPlan_functions(void)
{
	float dx,dy,dz;			//上位机发送相邻坐标之间的差值
	u8 i;
	CleanArm_FirstPostion(); //清洗臂末端初始位置采集函数
	delay_ms(500);
  	for(i=0;i<CleanArmPoints-1;i++)	  //发送数据点数为CleanArmPoints，中间存在CleanArmPoints-1条线段
	{
	   //上位机建立坐标与下位机建立坐标相互转换：X下=-Y上，Y下=-X上,Z下=Z上
	   dy=-CleanArmbuf[3+i*3]+CleanArmbuf[0+i*3];	//将上位机X轴坐标(储存在CleanArmbuf[0+i*3])差值转换到下位机Y轴坐标差值
	   dx=-CleanArmbuf[4+i*3]+CleanArmbuf[1+i*3];	//将上位机Y轴坐标(储存在CleanArmbuf[0+i*3])差值转换到下位机X轴坐标值差值
	   dz=CleanArmbuf[5+i*3]-CleanArmbuf[2+i*3];	//Z轴坐标差值
//	    printf("i=%d\n",i);
//	   printf("dy=%f\n",dy);
//	   printf("dx=%f\n",dx);
//	   printf("dz=%f\n",dz);
	   CleanArm_PathMove(dx,dy,dz);	 				//清洗臂根据上位机轨迹规划指令移动函数
	   if(i==CleanArmPoints-2)//执行完整个语句i才+1，所以i接收完最后一个数应该是CleanArmPoints-2
	   {
	   		CleanArmPathflag=0;			//执行完上位机轨迹规划后将清洗臂开始清洗标志位置0
//			Angleflag=1;				//启动清洗臂末端初始位置采集，下次发送清洗臂轨迹规划命令时重新采集初始位置

	   }
	}
}
void CleanArm_Postion(void) //清洗臂末端当前位置采集函数
{
	CleanArm_a2=103.33*(3.14/180)-acos((129210.5-CleanArm_Len1*CleanArm_Len1)/105432.86);//得到弧度值
	CleanArm_a3=-acos((184172.09-CleanArm_Len2*CleanArm_Len2)/151880)-1.92*(3.14/180);	 //得到弧度值
	CleanArm_a4=(90-fabs(CleanArm_a2)-fabs(CleanArm_a3))*(3.14/180);					 //得到弧度值
	CleanArm_X=cos(CleanArm_a1)*(300*cos(CleanArm_a2+CleanArm_a3)+600*cos(CleanArm_a2)); //初始状态X轴坐标值
	CleanArm_Y=sin(CleanArm_a1)*(300*cos(CleanArm_a2+CleanArm_a3)+600*cos(CleanArm_a2)); //初始状态Y轴坐标值
	CleanArm_Z=300*sin(CleanArm_a2+CleanArm_a3)+600*sin(CleanArm_a2);		 			 //初始状态Z轴坐标值
//		printf("CleanArm_a2=%f\n",CleanArm_a2);
//		printf("CleanArm_a3=%f\n",CleanArm_a3);
//		printf("CleanArm_X=%f\n",CleanArm_X);
//		printf("CleanArm_Y=%f\n",CleanArm_Y);
//		printf("CleanArm_Z=%f\n",CleanArm_Z);
//		printf("CleanArm_Len1=%f\n",CleanArm_Len1);
//		printf("CleanArm_Len2=%f\n",CleanArm_Len2);
//		printf("CleanArm_Len3=%f\n",CleanArm_Len3);
}
//数组缓存：将上位机发送的多帧数据进行缓存
// PointsNum,RunCycle表示坐标点总数和运行周期
// WalkAngle[4]存储运动过程中四个电机的转动角度
// WalkPoints[800]存储直线运动过程中的各个坐标
//len:一帧数据除了包头包尾有几个浮点型数据
//功能：将上位机发送的直线运动的相关数据进行保存
 void BufStorage(u8 len)
{ 
 float Leg1Angle,Leg2Angle,Leg3Angle,Leg4Angle;
 u8 bufr[64],ZJlen;
 float Reparar[7];				
 float Reparas[7];
 u8 bufs[30];
 int i;
// int j,m;
 static u16 fk;
 static u16 bk;
 static u16 ck;
 static u16 ltk;
 static u16 rtk;
 RS232_Receive_Data(bufr,&ZJlen);
 bytetofloat(bufr,Reparar,7);
 if(ZJlen)
 {  
  if(bufr[0]==0xB5 && bufr[1+len*4]==0x5B)
 {
 //直线行走标志位
       	if((int)(Reparar[1])==16)
		{
		  StraightWalkFlag=1;
		}
	  if((int)(Reparar[0])==5 && (int)(Reparar[1])==0)   //清洗臂发送分解次数指令
		  {
			  CleanArmPoints=(int)(Reparar[5]);		
//			  printf("Points=%d\n",CleanArmPoints);
			  ck=0;
		  }
		  if((int)(Reparar[0])==5 && (int)(Reparar[1])==1)  //清洗臂发送坐标数据指令
		  {
			 if(ck<CleanArmPoints)
			 {
				 CleanArmbuf[0+ck*3]=Reparar[2];	  //将储存在Repara[2]中的X轴坐标值赋值到CleanArmbuf[]中
				 CleanArmbuf[1+ck*3]=Reparar[3];	  //将储存在Repara[3]中的Y轴坐标值赋值到CleanArmbuf[]中
				 CleanArmbuf[2+ck*3]=Reparar[4];	  //将储存在Repara[4]中的Z轴坐标值赋值到CleanArmbuf[]中
				 ck++;
//				 printf("K=%d\n",k);
				 if(ck==CleanArmPoints)
				 {
					CleanArmPathflag=1;
					Angleflag=1;				//启动清洗臂末端初始位置采集，下次发送清洗臂轨迹规划命令时重新采集初始位置
//					for(n=0;n<CleanArmPoints;n++)
//					{
//					printf("n=%d\n",n);
//					printf("CleanArmbuf[]=%f\n",CleanArmbuf[n*3]);
//					}
				 }
			  }
			}


 //前进时的上位机通讯程序
 		if((int)(Reparar[1])==3)
		{
		   FRunCycle=(int)(Reparar[5]);
		   FPointsNum=(int)(Reparar[6]);
//		   printf("FRunCycle=%d,FPointsNum=%d\n",FRunCycle,FPointsNum);
		   bufs[0]=0xB5;  //发送数据包头
           bufs[29]=0x5B; //发送数据包尾
//         //向上位机发送当前四条腿四个电机的角度
        Leg1Angle=Encoder_Angle(3);   // 采集腿1电机角度
		Leg2Angle=Encoder_Angle(4);   // 采集腿2电机角度
		Leg3Angle=Encoder_Angle(5);   // 采集腿3电机角度
		Leg4Angle=Encoder_Angle(6);   // 采集腿4电机角度

//		Leg1Angle=25;   // 采集腿1电机角度
//		Leg2Angle=25;   // 采集腿2电机角度
//		Leg3Angle=25;   // 采集腿3电机角度
//		Leg4Angle=25;   // 采集腿4电机角度

        Reparas[0]=0;
		Reparas[1]=4;
		Reparas[2]=Leg1Angle;
		Reparas[3]=Leg2Angle;
		Reparas[4]=Leg3Angle;
		Reparas[5]=Leg4Angle;
		Reparas[6]=0;
	
	 for(i=0;i<7;i++)
	 {
	 FloatToByte(Reparas[i],&bufs[i*4+1]);
	 } 
	 delay_ms(200);
	 RS232_Send_Data(bufs,30);//发送四条腿的初始角度
	  }
	if((int)(Reparar[1])==2)
	{
	    FWalkAngle[0]=Reparar[2];//存取腿1转动角度值
	    FWalkAngle[1]=Reparar[3];//存取腿2转动角度值
		FWalkAngle[2]=Reparar[4];//存取腿1转动角度值
		FWalkAngle[3]=Reparar[5];//存取腿1转动角度值 	
//		printf("FWalkAngle1=%f",FWalkAngle[0]);
//		printf("FWalkAngle2=%f",FWalkAngle[1]);
//		printf("FWalkAngle3=%f",FWalkAngle[2]);
//		printf("FWalkAngle4=%f",FWalkAngle[3]);
		fk=0;
	}
	if((int)(Reparar[1])==8)
	{
		 if(fk<FPointsNum)
		 {
		 	     FWalkPoints[0+fk*3]=Reparar[2];	  //将储存在Repara[2]中的X轴坐标值赋值到WalkPoints[]中
				 FWalkPoints[1+fk*3]=Reparar[3];	  //将储存在Repara[3]中的Y轴坐标值赋值到WalkPoints[]中
				 FWalkPoints[2+fk*3]=Reparar[4];	    //将储存在Repara[4]中的Z轴坐标值赋值到WalkPoints[]中
//				  printf("x=%f",FWalkPoints[0+fk*3]);
//				  printf("y=%f",FWalkPoints[1+fk*3]);
//				  printf("z=%f\n",FWalkPoints[2+fk*3]);
				 fk++;	
		 }
		 if(fk==FPointsNum)
		 {
		   ForwordWalkFlag=1;
		 }
	}
//后退时的上位机通讯程序
 		if((int)(Reparar[1])==6)
		{
		   BRunCycle=(int)(Reparar[5]);
		   BPointsNum=(int)(Reparar[6]);
//		   printf("BRunCycle=%d,BPointsNum=%d\n",BRunCycle,BPointsNum);
		   bufs[0]=0xB5;  //发送数据包头
           bufs[29]=0x5B; //发送数据包尾
         //向上位机发送当前四条腿四个电机的角度
        Leg1Angle=Encoder_Angle(3);   // 采集腿1电机角度
		Leg2Angle=Encoder_Angle(4);   // 采集腿2电机角度
		Leg3Angle=Encoder_Angle(5);   // 采集腿3电机角度
		Leg4Angle=Encoder_Angle(6);   // 采集腿4电机角度

//		Leg1Angle=25;   // 采集腿1电机角度
//		Leg2Angle=25;   // 采集腿2电机角度
//		Leg3Angle=25;   // 采集腿3电机角度
//		Leg4Angle=25;   // 采集腿4电机角度

        Reparas[0]=0;
		Reparas[1]=4;
		Reparas[2]=Leg1Angle;
		Reparas[3]=Leg2Angle;
		Reparas[4]=Leg3Angle;
		Reparas[5]=Leg4Angle;
		Reparas[6]=0;
	
	 for(i=0;i<7;i++)
	 {
	 FloatToByte(Reparas[i],&bufs[i*4+1]);
	 } 
	 delay_ms(200);
	 RS232_Send_Data(bufs,30);//发送四条腿的初始角度
	  }
	if((int)(Reparar[1])==7)	   
	{
	    BWalkAngle[0]=Reparar[2];//存取腿1转动角度值
	    BWalkAngle[1]=Reparar[3];//存取腿2转动角度值
		BWalkAngle[2]=Reparar[4];//存取腿1转动角度值
		BWalkAngle[3]=Reparar[5];//存取腿1转动角度值
		bk=0;
	}
	if((int)(Reparar[1])==9)
	{
		 if(bk<BPointsNum)
		 {
		 	     BWalkPoints[0+bk*3]=Reparar[2];	  //将储存在Repara[2]中的X轴坐标值赋值到WalkPoints[]中
				 BWalkPoints[1+bk*3]=Reparar[3];	  //将储存在Repara[3]中的Y轴坐标值赋值到WalkPoints[]中
				 BWalkPoints[2+bk*3]=Reparar[4];	    //将储存在Repara[4]中的Z轴坐标值赋值到WalkPoints[]中
				 bk++;	
		 }
		 if(bk==BPointsNum)
		 {
		   BackwordWalkFlag=1;
		 }
	}
//右拐弯时的上位机通讯程序
   if((int)(Reparar[1])==10)
   {
     RTCompenLen=Reparar[3];
   	 RTPointsNum=(int)(Reparar[4]);
	 RTurnCycle=(int)(Reparar[5]);
	 RTRoundAngle=Reparar[6];
	       bufs[0]=0xB5;  //发送数据包头
           bufs[29]=0x5B; //发送数据包尾
         //向上位机发送当前四条腿四个电机的角度
        Leg1Angle=Encoder_Angle(3);   // 采集腿1电机角度
		Leg2Angle=Encoder_Angle(4);   // 采集腿2电机角度
		Leg3Angle=Encoder_Angle(5);   // 采集腿3电机角度
		Leg4Angle=Encoder_Angle(6);   // 采集腿4电机角度

//		Leg1Angle=25;   // 采集腿1电机角度
//		Leg2Angle=25;   // 采集腿2电机角度
//		Leg3Angle=25;   // 采集腿3电机角度
//		Leg4Angle=25;   // 采集腿4电机角度

        Reparas[0]=0;
		Reparas[1]=5;
		Reparas[2]=Leg1Angle;
		Reparas[3]=Leg2Angle;
		Reparas[4]=Leg3Angle;
		Reparas[5]=Leg4Angle;
		Reparas[6]=0;
	
	 for(i=0;i<7;i++)
	 {
	 FloatToByte(Reparas[i],&bufs[i*4+1]);
	 } 
	 delay_ms(200);
	 RS232_Send_Data(bufs,30);//发送四条腿的初始角度
   }
   if((int)(Reparar[1])==11)
   {
   	    RTWalkAngle[0]=Reparar[2];//存取腿1转动角度值
	    RTWalkAngle[1]=Reparar[3];//存取腿2转动角度值
		RTWalkAngle[2]=Reparar[4];//存取腿1转动角度值
		RTWalkAngle[3]=Reparar[5];//存取腿1转动角度值
		rtk=0;
   }
   if((int)(Reparar[0])==3 && (int)(Reparar[1])==12)
   {
   		  if(rtk<RTPointsNum)
		 {
		 	     RTWalkPoints[0+rtk*3]=Reparar[2];	  //将储存在Repara[2]中的X轴坐标值赋值到WalkPoints[]中
				 RTWalkPoints[1+rtk*3]=Reparar[3];	  //将储存在Repara[3]中的Y轴坐标值赋值到WalkPoints[]中
				 RTWalkPoints[2+rtk*3]=Reparar[4];	    //将储存在Repara[4]中的Z轴坐标值赋值到WalkPoints[]中
				 rtk++;	
		 }
		 if(rtk==RTPointsNum)
		 {
		   RTurnMoveFlag=1;
		 }
   }
    //左拐弯时的上位机通讯程序
   if((int)(Reparar[1])==13)
   {
     LTCompenLen=Reparar[3];
   	 LTPointsNum=(int)(Reparar[4]);
	 LTurnCycle=(int)(Reparar[5]);
	 LTRoundAngle=Reparar[6];
	       bufs[0]=0xB5;  //发送数据包头
           bufs[29]=0x5B; //发送数据包尾
         //向上位机发送当前四条腿四个电机的角度
        Leg1Angle=Encoder_Angle(3);   // 采集腿1电机角度
		Leg2Angle=Encoder_Angle(4);   // 采集腿2电机角度
		Leg3Angle=Encoder_Angle(5);   // 采集腿3电机角度
		Leg4Angle=Encoder_Angle(6);   // 采集腿4电机角度

//		Leg1Angle=25;   // 采集腿1电机角度
//		Leg2Angle=25;   // 采集腿2电机角度
//		Leg3Angle=25;   // 采集腿3电机角度
//		Leg4Angle=25;   // 采集腿4电机角度

        Reparas[0]=0;
		Reparas[1]=5;
		Reparas[2]=Leg1Angle;
		Reparas[3]=Leg2Angle;
		Reparas[4]=Leg3Angle;
		Reparas[5]=Leg4Angle;
		Reparas[6]=0;
	
	 for(i=0;i<7;i++)
	 {
	 FloatToByte(Reparas[i],&bufs[i*4+1]);
	 } 
	 delay_ms(200);
	 RS232_Send_Data(bufs,30);//发送四条腿的初始角度
   }
   if((int)(Reparar[1])==14)
   {
   	    LTWalkAngle[0]=Reparar[2];//存取腿1转动角度值
	    LTWalkAngle[1]=Reparar[3];//存取腿2转动角度值
		LTWalkAngle[2]=Reparar[4];//存取腿1转动角度值
		LTWalkAngle[3]=Reparar[5];//存取腿1转动角度值
		ltk=0;
   }
   if((int)(Reparar[0])==3 && (int)(Reparar[1])==15)
   {
   		  if(ltk<LTPointsNum)
		 {
		 	     LTWalkPoints[0+ltk*3]=Reparar[2];	  //将储存在Repara[2]中的X轴坐标值赋值到WalkPoints[]中
				 LTWalkPoints[1+ltk*3]=Reparar[3];	  //将储存在Repara[3]中的Y轴坐标值赋值到WalkPoints[]中
				 LTWalkPoints[2+ltk*3]=Reparar[4];	    //将储存在Repara[4]中的Z轴坐标值赋值到WalkPoints[]中
				 ltk++;	
		 }
		 if(ltk==LTPointsNum)
		 {
		   LTurnMoveFlag=1;
		 }
   }
	 }
	 
 }
//   RS232_RX_CNT=0;				//将232串口接收到的数据总数清零
//	for(j=0;j<len*4+2;j++)
//	{
//	    bufr[j]=0x00;				//将buf数组清零，否则每次执行都是之前存储的数组
//	}
//	for(m=0;m<len;m++)
//	{
//	    Reparar[m]=0;				//将Reparar数组清零，否则每次执行都是之前存储的数组
//	}
}
//发送函数：下位机将机器人各关节的状态实时发送给上位机
//state:1.2.3.4代表腿，5代表清洗臂 
void SendDataToPC(float state)
{
  u8 SendData[66];		//发送一帧数据为66个字节
  u8 i;
  float SendPara[16];	//发送16个浮点型数据缓存区
  SendData[0]=0xA5;		//包头
  SendData[65]=0x5A; 	//包尾
  SendPara[0]=state;
  switch((int)state)			//按功能进行操作
  {
	    //"1"发送腿1数据
	    case 1:					
	    	break;
		//"2"发送腿2数据
	    case 2:					
	    	break;
		//"3"发送腿3数据
	    case 3:					
	    	break;
		//"4"发送腿4数据
	    case 4:					
	    	break;
		//"5"发送清洗臂数据
	    case 5:
			SendPara[1]=-CleanArm_Y;	//清洗臂X轴坐标
			SendPara[2]=CleanArm_X;	 	//清洗臂Y轴坐标
			SendPara[3]=CleanArm_Z;	 	//清洗臂Z轴坐标
			SendPara[4]=CleanArm_a1*(180/3.14);	//清洗臂底座电机角度值
			SendPara[5]=CleanArm_Len1;	//清洗臂电杆1长度
			SendPara[6]=CleanArm_Len2;	//清洗臂电杆2长度
			SendPara[7]=CleanArm_Len3;	//清洗臂电杆3长度
			SendPara[8]=0.0;
			SendPara[9]=CleanArm_a4*(180/3.14);	//清洗盘电机角度值
			SendPara[10]=CleanArm_a2*(180/3.14);	//清洗臂大臂角度值
			SendPara[11]=CleanArm_a3*(180/3.14);	//清洗臂小臂角度值
			SendPara[12]=0.0;
			SendPara[13]=0.0;
			SendPara[14]=0.0;
			SendPara[15]=0.0; //附腿上值

	    	break;
		 default:
			break;
  } 	
 	for(i=0;i<16;i++)
	{
	 FloatToByte(SendPara[i],&SendData[i*4+1]);	//将16个浮点型数据转换成字节型数据储存在SendData[]中
	}
	 RS232_Send_Data(SendData,66);//发送数据到上位机
}
/*******************************************************************************
 *点动模式与机器人直线和拐弯行走相关函数
*******************************************************************************/
////点动控制模式：客户端下发指令下位机接收指令对应功能操作 
void DD_functions(void)
{
  u8 buf[64],ZJlen;
  float Repara[11];
  float len=0.5,Angle=1.0; //动作一次电杆申/缩距离为2.0mm，电机转动角度为1.0度
  RS232_Receive_Data(buf,&ZJlen);
  bytetofloat(buf,Repara,11);
//
//  if(ZJlen)							//如果232串口接收到数据，则进行对应功能操作，不加则上位机需要按键按下不断发送通讯指令
//  {
	   //包头bufstart,包尾bufend
	if(buf[0]==0xA5 && buf[45]==0x5A)
	{
	  switch((int)(Repara[0]))//按功能进行操作
	  {
	    //"1"启动客户端的鼠标控制模式
	    case 1:					
	    	break;
	    //"2"客户端控制机器人进行单轴点动
	    case 2:
			switch((int)(Repara[2]))//四足/清洗臂选择
	        {
	          //"1"对机器人腿1进行操作
	          case 1:
				 switch((int)(Repara[1]))//按功能进行操作
			     {
		          //"0"按键抬起
		          case 0:
					PointMoveflag=0;	//0：关闭点动模式
		          	break;
		          //"1"电机3角度逆时针转动（+）
		          case 1:
				  	ServoMotor_TurnAngle(Angle,0,3);	           
				  	break;
				  //"2"电机3角度顺时针转动（-）
		          case 2:
				  	ServoMotor_TurnAngle(Angle,1,3);	           
				  	break;
				  //"3"电杆1前进（+）
		          case 3:
				  	ElectricPutter_DDMove(len,1,1);	           
				  	break;
				  //"4"电杆1收缩（-）
		          case 4:
				  	ElectricPutter_DDMove(len,0,1);	           
				  	break;
				  //"5"电杆2前进（+）
		          case 5:
				  	ElectricPutter_DDMove(len,1,2);	           
				  	break;
				  //"6"电杆2收缩（-）
		          case 6:
				  	ElectricPutter_DDMove(len,0,2);	           
				  	break;
				  //"7"电杆3前进（+）
		          case 7:
				  	ElectricPutter_DDMove(len,1,3);	           
				  	break;
				  //"8"电杆3收缩（-）
		          case 8:
				  	ElectricPutter_DDMove(len,0,3);	           
				  	break;
				  //吸盘（开）
				  case 9:
				  	break;
				  //吸盘（合）
				  case 10:
				  	break;
				  default:
			  		break;
				 } 		
	          	break;
	          //"2"对机器人腿2进行操作
	          case 2:
			     switch((int)(Repara[1]))//按功能进行操作
			     {
		          //"0"按键抬起
		          case 0:
				  	PointMoveflag=0;	//0：关闭点动模式
		          	break;
		          //"1"电机4角度逆时针转动（+）
		          case 1:
				  	ServoMotor_TurnAngle(Angle,0,4);	           
				  	break;
				  //"2"电机4角度顺时针转动（-）
		          case 2:
				  	ServoMotor_TurnAngle(Angle,1,4);	           
				  	break;
				  //"3"电杆10前进（+）
		          case 3:
				  	ElectricPutter_DDMove(len,1,10);	           
				  	break;
				  //"4"电杆10收缩（-）
		          case 4:
				  	ElectricPutter_DDMove(len,0,10);	           
				  	break;
				  //"5"电杆11前进（+）
		          case 5:
				  	ElectricPutter_DDMove(len,1,11);	           
				  	break;
				  //"6"电杆11收缩（-）
		          case 6:
				  	ElectricPutter_DDMove(len,0,11);	           
				  	break;
				  //"7"电杆12前进（+）
		          case 7:
				  	ElectricPutter_DDMove(len,1,12);	           
				  	break;
				  //"8"电杆12收缩（-）
		          case 8:
				  	ElectricPutter_DDMove(len,0,12);	           
				  	break;
				  //吸盘（开）
				  case 9:
				  	break;
				  //吸盘（合）
				  case 10:
				  	break;
				  default:
			  		break;
				 } 			           
			  	break;
			  //"3"对机器人腿3进行操作
	          case 3:
			     switch((int)(Repara[1]))//按功能进行操作
			     {
		          //"0"按键抬起
		          case 0:
				  	PointMoveflag=0;	//0：关闭点动模式
		          	break;
		          //"1"电机5角度逆时针转动（+）
		          case 1:
				  	ServoMotor_TurnAngle(Angle,0,5);	           
				  	break;
				  //"2"电机5角度顺时针转动（-）
		          case 2:
				  	ServoMotor_TurnAngle(Angle,1,5);	           
				  	break;
				  //"3"电杆7前进（+）
		          case 3:
				  	ElectricPutter_DDMove(len,1,7);	           
				  	break;
				  //"4"电杆7收缩（-）
		          case 4:
				  	ElectricPutter_DDMove(len,0,7);	           
				  	break;
				  //"5"电杆8前进（+）
		          case 5:
				  	ElectricPutter_DDMove(len,1,8);	           
				  	break;
				  //"6"电杆8收缩（-）
		          case 6:
				  	ElectricPutter_DDMove(len,0,8);	           
				  	break;
				  //"7"电杆9前进（+）
		          case 7:
				  	ElectricPutter_DDMove(len,1,9);	           
				  	break;
				  //"8"电杆9收缩（-）
		          case 8:
				  	ElectricPutter_DDMove(len,0,9);	           
				  	break;
				  //吸盘（开）
				  case 9:
				  	break;
				  //吸盘（合）
				  case 10:
				  	break;
				  default:
			  		break;
				 } 		
	          	break;
	          //"4"对机器人腿4进行操作
	          case 4:
			     switch((int)(Repara[1]))//按功能进行操作
			     {
		          //"0"按键抬起
		          case 0:
				  	PointMoveflag=0;	//0：关闭点动模式
		          	break;
		          //"1"电机6角度逆时针转动（+）
		          case 1:
				  	ServoMotor_TurnAngle(Angle,0,6);	           
				  	break;
				  //"2"电机6角度顺时针转动（-）
		          case 2:
				  	ServoMotor_TurnAngle(Angle,1,6);	           
				  	break;
				  //"3"电杆4前进（+）
		          case 3:
				  	ElectricPutter_DDMove(len,1,4);	           
				  	break;
				  //"4"电杆4收缩（-）
		          case 4:
				  	ElectricPutter_DDMove(len,0,4);	           
				  	break;
				  //"5"电杆5前进（+）
		          case 5:
				  	ElectricPutter_DDMove(len,1,5);	           
				  	break;
				  //"6"电杆5收缩（-）
		          case 6:
				  	ElectricPutter_DDMove(len,0,5);	           
				  	break;
				  //"7"电杆6前进（+）
		          case 7:
				  	ElectricPutter_DDMove(len,1,6);	           
				  	break;
				  //"8"电杆6收缩（-）
		          case 8:
				  	ElectricPutter_DDMove(len,0,6);	           
				  	break;
				  //吸盘（开）
				  case 9:
				  	break;
				  //吸盘（合）
				  case 10:
				  	break;
				  default:
			  		break;
				 } 			           
			  	break;
			  //"5"对机器人清洗臂进行操作
	          case 5:
			    switch((int)(Repara[1]))//按功能进行操作
			    {
		          //"0"按键抬起
		          case 0:
				  	delay_ms(200);
				  	PointMoveflag=0;	//0：关闭点动模式
					SendDataToPC(5);	//发送清洗臂当前状态
					delay_ms(20);
					SendDataToPC(5);	//发送清洗臂当前状态
					delay_ms(20);
					SendDataToPC(5);	//发送清洗臂当前状态
		          	break;
		          //"1"电机7角度逆时针转动（+）
		          case 1:
				  	ServoMotor_TurnAngle(Angle,0,7);
					CleanArm_a1=CleanArm_a1+Angle*(3.14/180); 	//清洗臂底座电机转动后的角度值,将角度转换为弧度值
//						printf("CleanArm_a1=%f\n",CleanArm_a1*(180/3.14));	           
				  	break;
				  //"2"电机7角度顺时针转动（-）
		          case 2:
				  	ServoMotor_TurnAngle(Angle,1,7);
					CleanArm_a1=CleanArm_a1-Angle*(3.14/180); 	//清洗臂底座电机转动后的角度值,将角度转换为弧度值	           
				  	break;
				  //"3"电杆13前进（+）
		          case 3:
				  	ElectricPutter_DDMove(len,1,13);
					CleanArm_Len1=CleanArm_Len1+len;			//电动后清洗臂电杆1的长度
//					printf("CleanArm_Len1=%f\n",CleanArm_Len1);	           
				  	break;
				  //"4"电杆13收缩（-）
		          case 4:
				  	ElectricPutter_DDMove(len,0,13);
					CleanArm_Len1=CleanArm_Len1-len;			//电动后清洗臂电杆1的长度	           
				  	break;
				  //"5"电杆14前进（+）
		          case 5:
				  	ElectricPutter_DDMove(len,1,14);
					CleanArm_Len2=CleanArm_Len2+len;			//电动后清洗臂电杆2的长度	           
				  	break;
				  //"6"电杆14收缩（-）
		          case 6:
				  	ElectricPutter_DDMove(len,0,14);
					CleanArm_Len2=CleanArm_Len2-len;			//电动后清洗臂电杆2的长度	           
				  	break;
				  //"7"电杆15前进（+）
		          case 7:
				  	ElectricPutter_DDMove(len,1,15);
					CleanArm_Len3=CleanArm_Len3+len;			//电动后清洗臂电杆3的长度	           
				  	break;
				  //"8"电杆15收缩（-）
		          case 8:
				  	ElectricPutter_DDMove(len,0,15);
					CleanArm_Len3=CleanArm_Len3-len;			//电动后清洗臂电杆3的长度	           
				  	break;
				  //毛刷伺服电机逆时针转动（电机1）
				  case 11:
				    ServoMotor_TurnAngle(Angle,0,1); 
				  	break;
				  //毛刷伺服电机顺时针转动（电机1）
				  case 12:
				    ServoMotor_TurnAngle(Angle,1,1);
				  	break;
				  //清洗盘伺服电机逆时针转动（电机2）
				  case 13:
				    ServoMotor_TurnAngle(Angle,0,1);
					CleanArm_a4=CleanArm_a4+Angle*(3.14/180); 	//清洗臂清洗盘电机转动后的角度值,将角度转换为弧度值 
				  	break;
				  //清洗盘伺服电机顺时针转动（电机2）
				  case 14:
				    ServoMotor_TurnAngle(Angle,1,1);
					CleanArm_a4=CleanArm_a4-Angle*(3.14/180); 	//清洗臂清洗盘电机转动后的角度值,将角度转换为弧度值
				  	break;
				  default:
				  	delay_ms(200);
				  	PointMoveflag=0;	//0：关闭点动模式
					SendDataToPC(5);	//发送清洗臂当前状态
					delay_ms(20);
					SendDataToPC(5);	//发送清洗臂当前状态
					delay_ms(20);
					SendDataToPC(5);	//发送清洗臂当前状态
			  		break;
				 } 			           
			  	break;
			  default:
			  	PointMoveflag=0;	//0：关闭点动模式
		  		break;
			} 			
		}
	}
	CleanArm_Postion();	   //每执行一次点动模式计算一次反馈发给上位机的各个参数
//	delay_ms(400);
//	SendDataToPC(5);
//}
}
//直线行走程序
//m=1时前进，m=0时后退
void StraightWalk_function(int m)
{
int i;
float y1,y2,n;
if(m==1)
{
//前进初始状态
   ElectricPutter_Move(20,0,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(10,0,3);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,0,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(10,0,6);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,0,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(10,1,4);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,0,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(10,1,5);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);       
for(i=0;i<FRunCycle;i++)
   {
   ElectricPutter_Move(20,0,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(FWalkAngle[3],1,6);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   y1=RHLegOriginal(2);
   //3 
   ElectricPutter_Move(20,0,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(FWalkAngle[2],0,5);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   y2=RHLegOriginal(2);
   n=(y2-y1)/10;
   //4
   ElectricPutter_Move(20,0,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(FWalkAngle[0],1,3);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   
   //5
   ElectricPutter_Move(20,0,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(FWalkAngle[1],0,4);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500); 
   
   //6
  BodyLegMove1(-(y2-y1),n);
  delay_ms(1500);
  delay_ms(1500);
  }
   ElectricPutter_Move(20,0,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(6);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500); 
   ElectricPutter_Move(20,0,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(5);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,0,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(3);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,0,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(4);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);  
  }
if(m==0)
{
//后退初始状态
   ElectricPutter_Move(20,0,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(10,0,3);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,0,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(10,0,6);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,0,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(10,1,4);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,0,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(10,1,5);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);       
   for(i=0;i<BRunCycle;i++)
   {
   BodyLegMove1(76,7.6);
   delay_ms(1500);
   delay_ms(1500);
   //3
   ElectricPutter_Move(20,0,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(BWalkAngle[0],0,3);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   //4
   ElectricPutter_Move(20,0,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(BWalkAngle[3],0,6);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   //5
   ElectricPutter_Move(20,0,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(BWalkAngle[1],1,4);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   
   //6
   ElectricPutter_Move(20,0,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(BWalkAngle[2],1,5);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500); 
   }
   ElectricPutter_Move(20,0,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(6);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500); 
   ElectricPutter_Move(20,0,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(5);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,0,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(3);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,0,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(4);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);  
}
  StraightWalkFlag=0;
  BackwordWalkFlag=0;
  ForwordWalkFlag=0;
}
//拐弯行走程序
//n=1时右拐弯，n=0时左拐弯
void TurnWalk_function(int n)
{
  float x0,y0,z0,dx,dy,dz;
  float x,y,z,m7,m8,djrx;
  int i,j;
  //左拐弯 
  if(n==0)
  {
  //左拐弯初始状态
   ElectricPutter_Move(20,0,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(LTRoundAngle,0,6);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);

   ElectricPutter_Move(20,0,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(LTRoundAngle,0,5);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);

   ElectricPutter_Move(20,0,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(LTRoundAngle,0,4);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);

   ElectricPutter_Move(20,0,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(LTRoundAngle,0,3);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
  for(j=0;j<LTurnCycle+3;j++)
  {  
  x0=RHLegOriginal(1);
  y0=RHLegOriginal(2);
  z0=RHLegOriginal(3);
  x=x0,y=y0,z=z0;
  for(i=0;i<LTPointsNum-1;i++)
  {
   	dx=-LTWalkPoints[3+i*3]+LTWalkPoints[0+i*3];
	dy=-LTWalkPoints[4+i*3]+LTWalkPoints[1+i*3];
	dz=-LTWalkPoints[5+i*3]+LTWalkPoints[2+i*3];
//    dx=-fx[i];
//	dy=-fy[i];
	x+=dx;
	y+=dy;
	z+=dz;
	djrx=RHLegLocation(x,y,z,1)-RHLegLocation(x-dx,y-dy,z-dz,1);
	m7=RHLegLocation(x,y,z,2)-RHLegLocation(x-dx,y-dy,z-dz,2);	
	m8=RHLegLocation(x,y,z,3)-RHLegLocation(x-dx,y-dy,z-dz,3);
	 if(djrx>0)
	{
     BodyDJMove_SameE(djrx);
	}
	else
	{
	 BodyDJMove_SameF(-djrx);
	}
	if(m7>0)
	{
     ElectricPutter_MoveSame147A2(m7,1);
	}
	else
	{
	ElectricPutter_MoveSame147A2(-m7,0);
	}	

	if(m8>0)
	{
	ElectricPutter_MoveSame258B2(m8,1);
	}
	else
	{
	 ElectricPutter_MoveSame258B2(-m8,0);					    
	}   
}
  if(j<LTurnCycle+2)
  {
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
	//1	
	if(j==0)
	{																														
   ElectricPutter_Move(20,0,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0xe7,0xff);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(40,0,5);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(6);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(LTWalkAngle[3],0,6);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,5);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   }
   else
   {
   	 ElectricPutter_Move(20,0,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0xe7,0xff);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(6);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(LTWalkAngle[3],0,6);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,5);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   }
   //2
   ElectricPutter_Move(20,0,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0x3f,0xff);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(5);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(LTWalkAngle[2],0,5);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,8);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   //3
   ElectricPutter_Move(20,0,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0xff,0xf9);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(LTWalkAngle[1],0,4);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,11);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   //4
   ElectricPutter_Move(20,0,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0xfc,0xff);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(3);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(LTWalkAngle[0],0,3);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,2);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   }
   else
   {
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);  
   ElectricPutter_Move(20,0,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500); 
   CAT9555_WriteByte(SETUP,0xe7,0xff);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(6);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,5);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   
   //2 
   ElectricPutter_Move(20,0,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0x3f,0xff);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(5);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,8);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
  
   //3
   ElectricPutter_Move(20,0,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0xff,0xf9);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
  ElectricPutter_Move(30,1,11);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   //4
   ElectricPutter_Move(20,0,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0xfc,0xff);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(6);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,2);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   }
}
}
//右拐弯
if(n==1)
  {
//  RTurnCycle=2;
  //右拐弯初始状态
   ElectricPutter_Move(20,0,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(RTRoundAngle,1,6);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);

   ElectricPutter_Move(20,0,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(RTRoundAngle,1,5);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);

   ElectricPutter_Move(20,0,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(RTRoundAngle,1,4);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);

   ElectricPutter_Move(20,0,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(RTRoundAngle,1,3);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
  for(j=0;j<RTurnCycle+3;j++)
  {  
  x0=RHLegOriginal(1);
  y0=RHLegOriginal(2);
  z0=RHLegOriginal(3);
  x=x0,y=y0,z=z0;
  for(i=0;i<RTPointsNum-1;i++)
  {
   	dx=-RTWalkPoints[3+i*3]+RTWalkPoints[0+i*3];
	dy=-RTWalkPoints[4+i*3]+RTWalkPoints[1+i*3];
	dz=-RTWalkPoints[5+i*3]+RTWalkPoints[2+i*3];
//    dx=-fx[i];
//	dy=-fy[i];
	x+=dx;
	y+=dy;
	z+=dz;
	djrx=RHLegLocation(x,y,z,1)-RHLegLocation(x-dx,y-dy,z-dz,1);
	m7=RHLegLocation(x,y,z,2)-RHLegLocation(x-dx,y-dy,z-dz,2);	
	m8=RHLegLocation(x,y,z,3)-RHLegLocation(x-dx,y-dy,z-dz,3);
	 if(djrx>0)
	{
     BodyDJMove_SameE(djrx);
	}
	else
	{
	 BodyDJMove_SameF(-djrx);
	}
	if(m7>0)
	{
     ElectricPutter_MoveSame147A2(m7,1);
	}
	else
	{
	ElectricPutter_MoveSame147A2(-m7,0);
	}	

	if(m8>0)
	{
	ElectricPutter_MoveSame258B2(m8,1);
	}
	else
	{
	 ElectricPutter_MoveSame258B2(-m8,0);					    
	}   
}
   if(j<RTurnCycle+2)
   {
	//1		
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   if(j==0)
   {																													
   ElectricPutter_Move(20,0,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0xe7,0xff);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,0,5);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(6);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(RTWalkAngle[3],1,6);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,5);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   }
   else
   {
   ElectricPutter_Move(20,0,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0xe7,0xff);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(6);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(RTWalkAngle[3],1,6);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,5);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   }
   //2
   ElectricPutter_Move(20,0,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0x3f,0xff);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(5);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(RTWalkAngle[2],1,5);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,8);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   //3
   ElectricPutter_Move(20,0,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0xff,0xf9);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(RTWalkAngle[1],1,4);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,11);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   //4
   ElectricPutter_Move(20,0,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0xfc,0xff);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(3);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(RTWalkAngle[0],1,3);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,2);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   }
   else
   {
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,0,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0xe7,0xff);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500); 
   ServoMotor_Reset(6);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,5);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
  
   //2 
   ElectricPutter_Move(20,0,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0x3f,0xff);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(5);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,8);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   
   //3
   ElectricPutter_Move(20,0,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0xff,0xf9);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,11);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   //4
   ElectricPutter_Move(20,0,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0xfc,0xff);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(6);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,2);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   }
}
}
RTurnMoveFlag=0;
LTurnMoveFlag=0;
}
