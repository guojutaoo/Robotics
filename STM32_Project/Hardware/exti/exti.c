#include "exti.h"
#include "systick.h"
/*******************************************************************************
* 函 数 名         : exti_init
* 函数功能		   : 外部中断2端口初始化函数	   
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void exti_init()  //外部中断初始化
{
	GPIO_InitTypeDef GPIO_InitStructure;

	EXTI_InitTypeDef EXTI_InitStructure;

	NVIC_InitTypeDef NVIC_InitStructure;

	/* 开启GPIO时钟 */
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	  //开启io口复用功能（外部中断）
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);

	GPIO_InitStructure.GPIO_Pin=KEY1;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	 
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource0);//选择GPIO管脚用作外部中断线路
	//此处一定要记住给端口管脚加上中断外部线路
	/* 设置外部中断的模式 */ 
	EXTI_InitStructure.EXTI_Line=EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure); 
	
	/* 设置NVIC参数 */   //中断优先级	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);		 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn; 	//打开EXTI0的全局中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //抢占优先级为0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  //响应优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 		  //使能
	NVIC_Init(&NVIC_InitStructure); 		
}
void EXTI0_IRQHandler()	   //外部中断0中断函数
{
	if(EXTI_GetITStatus(EXTI_Line0)==SET)
	{
   		EXTI_ClearITPendingBit(EXTI_Line0);//清除EXTI线路挂起位
		delay_ms(10);//消抖处理
		if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0)==Bit_RESET)	   //key1按键按下
		{
			delay_ms(10);//消抖处理
			if(GPIO_ReadOutputDataBit(GPIOD,GPIO_Pin_13)==Bit_RESET)
			{
				//LED 熄灭
			   GPIO_SetBits(GPIOD,GPIO_Pin_13);	
			}
			else
			{
			   //LED 发光
				GPIO_ResetBits(GPIOD,GPIO_Pin_13);
			}
		} 
		while(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0)==0);	  //key1按键松开跳出循环
	}		
}

