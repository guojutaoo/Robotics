#include "input.h"
#include "systick.h"
u8 TIM5CH1_CAPTURE_STA; //输入捕获状态 
u16 TIM5CH1_CAPTURE_VAL;//输入捕获值

/*******************************************************************************
* 函 数 名         : input_init
* 函数功能		   : 输入捕获配置初始化	   定时器5通道1输入捕获配置 
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void input_init()
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;//声明一个结构体变量，用来初始化定时器
	TIM_ICInitTypeDef TIM5_ICInitStructure;

	NVIC_InitTypeDef NVIC_InitStructure;

	/* 开启定时器5时钟 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);	 //使能TIM5时钟

	TIM_ClearITPendingBit(TIM5,TIM_IT_Update|TIM_IT_CC1); //清除中断和捕获标志位

	TIM_TimeBaseInitStructure.TIM_Period = 0xffff;	 //设定计数器自动重装值 	
	TIM_TimeBaseInitStructure.TIM_Prescaler = 71;   //以1Mhz的频率计数 一次即是1us
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;	  //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	 //TIM向上计数模式
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	TIM5_ICInitStructure.TIM_Channel = TIM_Channel_1; //选择输入端 IC1映射到TI1上 
	TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //上升沿捕获 
	TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上 
	TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; //配置输入分频,不分频 
	TIM5_ICInitStructure.TIM_ICFilter = 0x00; //IC1F=0000 配置输入滤波器 不滤波 
	TIM_ICInit(TIM5, &TIM5_ICInitStructure); //初始化TIM5输入捕获通道1	
	
	//中断分组初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn;	//打开TIM5的全局中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;	//抢占优先级为0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1; //响应优先级为1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	  //使能
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM5,ENABLE); //使能或者失能TIMx外设
	TIM_ITConfig(TIM5, TIM_IT_Update|TIM_IT_CC1, ENABLE );	//使能或者失能指定的TIM中断
		
}
//超声波传感器初始化，PD13接超声波传感器input引脚
void RBURF02_init()
{
    GPIO_InitTypeDef GPIO_InitStructure; //声明一个结构体变量，用来初始化GPIO

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);

	/*  配置GPIO的模式和IO口 */
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;  //选择你要设置的IO口
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;	 //设置推挽输出模式
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	  //设置传输速率
	GPIO_Init(GPIOD,&GPIO_InitStructure); 	   /* 初始化GPIO */
   	
}
//超声波传感器返回距离
float RBURF02_distance()
{
	u32 temp;
	float x1,distance1;
	GPIO_ResetBits(GPIOD,GPIO_Pin_13); //使发出发出超声波信号接口低电平2 μs 
	delay_us(2); 
	GPIO_SetBits(GPIOD,GPIO_Pin_13); //使发出发出超声波信号接口高电平10μs ，这里是至少10μs 
	delay_us(10); 
	GPIO_ResetBits(GPIOD,GPIO_Pin_13); // 保持发出超声波信号接口低电平
	if((TIM5CH1_CAPTURE_STA&0x80))	  //成功捕获到了一次上升沿
	{
		temp=TIM5CH1_CAPTURE_STA&0x3f;
		temp=temp*65536;		  //溢出时间总和
		temp=temp+TIM5CH1_CAPTURE_VAL;//得到总的高电平时间
		TIM5CH1_CAPTURE_STA=0; //开启下一次捕获	
	}
	distance1 = temp;  //  读出接收脉冲的时间 
	distance1 = distance1/58;       // 将脉冲时间转化为距离（单位：厘米） 
	x1 = distance1 * 100.0;
	distance1 = x1 / 100.0; //保留两位小数 
	return	distance1;
}
void TIM5_IRQHandler()	  //定时器5输入捕获中断函数
{
	if((TIM5CH1_CAPTURE_STA&0X80)==0)//还未成功捕获
	{
		if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)	
		{
			if(TIM5CH1_CAPTURE_STA&0X40) //已经捕获到高电平了
			{
				if((TIM5CH1_CAPTURE_STA&0x3f)==0x3f)//高电平太长了
				{	
					TIM5CH1_CAPTURE_STA|=0x80;	//标记成功捕获了一次
					TIM5CH1_CAPTURE_VAL=0xffff;		
				}
				else
				{
					TIM5CH1_CAPTURE_STA++;	
				}
			}
		}	
	}
	if (TIM_GetITStatus(TIM5, TIM_IT_CC1) != RESET) //捕获1发生捕获事件
	{
		if(TIM5CH1_CAPTURE_STA&0X40) //捕获到一个下降沿
		{
			TIM5CH1_CAPTURE_STA|=0X80; //标记成功捕获到一次上升沿 
			TIM5CH1_CAPTURE_VAL=TIM_GetCapture1(TIM5); //获得TIMx输入捕获1的值
			TIM_OC1PolarityConfig(TIM5,TIM_ICPolarity_Rising); //设置为上升沿捕获	
		}
		else
		{
			TIM5CH1_CAPTURE_STA=0; //清空 
			TIM5CH1_CAPTURE_VAL=0; 
			TIM_SetCounter(TIM5,0); 
			TIM5CH1_CAPTURE_STA|=0X40; //标记捕获到了上升沿 
			TIM_OC1PolarityConfig(TIM5,TIM_ICPolarity_Falling); //设置为下降沿捕获
		}	
	}
	TIM_ClearITPendingBit(TIM5, TIM_IT_CC1|TIM_IT_Update); //清除中断标志位
}

