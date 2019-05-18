#include "time.h"
#include "ServoMotor.h"
/*******************************************************************************
* �� �� ��         : time_init
* ��������		   : ��ʱ��3�˿ڳ�ʼ������	   
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void time_init()
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;	 //����һ���ṹ�������������ʼ��GPIO

	NVIC_InitTypeDef NVIC_InitStructure;

	/* ������ʱ��3ʱ�� */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);//���TIMx���жϴ�����λ:TIM �ж�Դ
	TIM_TimeBaseInitStructure.TIM_Period = 2000;//�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler = 35999;//����������ΪTIMxʱ��Ƶ��Ԥ��Ƶֵ��100Khz����Ƶ��
	TIM_TimeBaseInitStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);	
	TIM_Cmd(TIM3,ENABLE); //ʹ�ܻ���ʧ��TIMx����
	/* �����жϲ����������ж� */
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE );	//ʹ�ܻ���ʧ��ָ����TIM�ж�
	
	/* ����NVIC���� */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //��TIM3_IRQn��ȫ���ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;	//��ռ���ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;  //��Ӧ���ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	//ʹ��
	NVIC_Init(&NVIC_InitStructure);	
}

void TIM3_IRQHandler()	 //��ʱ��3�жϺ��� ����ʱ1������ж�
{
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
//	UartRxMonitor(1);
}

//void TIM2_Configuration(void) //����Modbusͨ�ŵ�3.5T
//{
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	/*��ʼ��ΪĬ��ֵ*/
//	TIM_DeInit(TIM2);
////	TIM_InternalClockConfig(TIM2);	
//   	/* TIM2 clock enable */
//  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
//	
//	/* TIM2����ʱ������������*/
//	/*T=3.5*( 1 +����λ+��żУ��+ ֹͣλ)/ ������
//	t=3.5*��1+8+0+1��/������
//	����t1.5 �� t3.5 �Ķ�ʱ�������Ŵ����Ķ��жϵĹ���.�ڸ�ͨ�������£��⵼��CPU ������
//	�ء���ˣ���ͨ�����ʵ��ڻ����19200 Bps ʱ����������ʱ�����ϸ����أ����ڲ����ʴ���
//	19200 Bps �����Σ�Ӧ��ʹ��2 ����ʱ�Ĺ̶�ֵ��������ַ��䳬ʱʱ��(t1.5)Ϊ750��s��
//	֡��ĳ�ʱʱ��(t1.5) Ϊ1.750ms��*/
//	TIM_TimeBaseStructure.TIM_Period = 175*2;		               //����ֵ:175 ��ʱ1750us 19200 3.5���ַ�����
//	TIM_TimeBaseStructure.TIM_Prescaler =720-1;    	               //Ԥ��Ƶ,����:720, 10us
//	TIM_TimeBaseStructure.TIM_ClockDivision = 0;  	           //ʱ�ӷ�Ƶ����Ϊ1
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;    //���ϼ���
//	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);	               // Time base configuration
//
//	/*Ԥ����������ж�λ*/
//	TIM_ARRPreloadConfig(TIM2, ENABLE);//ʹ��Ԥװ��
//	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
//	
////	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
////TIM_SetCounter(TIM2, 0);
//
//	/* ��������ж�*/
//	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); //��������ж�
//	TIM_Cmd(TIM2,DISABLE); 
//		/* ����NVIC���� */
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
//	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn; //��TIM3_IRQn��ȫ���ж�
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;	//��ռ���ȼ�Ϊ0
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;  //��Ӧ���ȼ�Ϊ0
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	//ʹ��
//	NVIC_Init(&NVIC_InitStructure);	
//}
//void TIM2_IRQHandler(void)
//{ 
//   if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)//�ж��Ƿ���TIM2�����ж�
//   {
//		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);//���TIM2���жϴ�����λ
//		TIM_ClearFlag(TIM2, TIM_FLAG_Update);//���TIM2�������־λ
//		TIM_Cmd(TIM2, DISABLE);
//		if(recenum >= 8)
//		{
//			recenum = 0;
//			Uart1_rev_flag = 1;//�������һ֡����λ��־λ��֪ͨ���������ý��մ�����
//			USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);//ʧ�ܴ���2�����ж�
//			GPIO_SetBits(GPIOG,GPIO_Pin_3);//485����ʹ��,ֹͣ����
//		}
//		else
//		{
//			recenum = 0;
//			
//		}
//	}
//}

