#include "time.h"
#include "ServoMotor.h"
/*******************************************************************************
* 函 数 名         : time_init
* 函数功能		   : 定时器3端口初始化函数	   
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void time_init()
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;	 //声明一个结构体变量，用来初始化GPIO

	NVIC_InitTypeDef NVIC_InitStructure;

	/* 开启定时器3时钟 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);//清除TIMx的中断待处理位:TIM 中断源
	TIM_TimeBaseInitStructure.TIM_Period = 2000;//设置自动重装载寄存器周期的值
	TIM_TimeBaseInitStructure.TIM_Prescaler = 35999;//设置用来作为TIMx时钟频率预分频值，100Khz计数频率
	TIM_TimeBaseInitStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM向上计数模式
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);	
	TIM_Cmd(TIM3,ENABLE); //使能或者失能TIMx外设
	/* 设置中断参数，并打开中断 */
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE );	//使能或者失能指定的TIM中断
	
	/* 设置NVIC参数 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //打开TIM3_IRQn的全局中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;	//抢占优先级为0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;  //响应优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	//使能
	NVIC_Init(&NVIC_InitStructure);	
}

void TIM3_IRQHandler()	 //定时器3中断函数 ，定时1秒进行中断
{
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
//	UartRxMonitor(1);
}

//void TIM2_Configuration(void) //用于Modbus通信的3.5T
//{
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	/*初始化为默认值*/
//	TIM_DeInit(TIM2);
////	TIM_InternalClockConfig(TIM2);	
//   	/* TIM2 clock enable */
//  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
//	
//	/* TIM2做定时器，基础设置*/
//	/*T=3.5*( 1 +数据位+奇偶校验+ 停止位)/ 波特率
//	t=3.5*（1+8+0+1）/波特率
//	由于t1.5 和 t3.5 的定时，隐含着大量的对中断的管理.在高通信速率下，这导致CPU 负担加
//	重。因此，在通信速率等于或低于19200 Bps 时，这两个定时必须严格遵守；对于波特率大于
//	19200 Bps 的情形，应该使用2 个定时的固定值：建议的字符间超时时间(t1.5)为750μs，
//	帧间的超时时间(t1.5) 为1.750ms。*/
//	TIM_TimeBaseStructure.TIM_Period = 175*2;		               //计数值:175 定时1750us 19200 3.5个字符长度
//	TIM_TimeBaseStructure.TIM_Prescaler =720-1;    	               //预分频,除数:720, 10us
//	TIM_TimeBaseStructure.TIM_ClockDivision = 0;  	           //时钟分频因子为1
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;    //向上计数
//	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);	               // Time base configuration
//
//	/*预先清除更新中断位*/
//	TIM_ARRPreloadConfig(TIM2, ENABLE);//使能预装载
//	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
//	
////	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
////TIM_SetCounter(TIM2, 0);
//
//	/* 配置溢出中断*/
//	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); //允许更新中断
//	TIM_Cmd(TIM2,DISABLE); 
//		/* 设置NVIC参数 */
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
//	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn; //打开TIM3_IRQn的全局中断
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;	//抢占优先级为0
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;  //响应优先级为0
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	//使能
//	NVIC_Init(&NVIC_InitStructure);	
//}
//void TIM2_IRQHandler(void)
//{ 
//   if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)//判断是否发生TIM2更新中断
//   {
//		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);//清除TIM2的中断待处理位
//		TIM_ClearFlag(TIM2, TIM_FLAG_Update);//清除TIM2待处理标志位
//		TIM_Cmd(TIM2, DISABLE);
//		if(recenum >= 8)
//		{
//			recenum = 0;
//			Uart1_rev_flag = 1;//接收完毕一帧，置位标志位，通知主函数调用接收处理函数
//			USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);//失能串口2接收中断
//			GPIO_SetBits(GPIOG,GPIO_Pin_3);//485发送使能,停止接收
//		}
//		else
//		{
//			recenum = 0;
//			
//		}
//	}
//}

