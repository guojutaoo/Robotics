#include "input.h"
#include "systick.h"
u8 TIM5CH1_CAPTURE_STA; //���벶��״̬ 
u16 TIM5CH1_CAPTURE_VAL;//���벶��ֵ

/*******************************************************************************
* �� �� ��         : input_init
* ��������		   : ���벶�����ó�ʼ��	   ��ʱ��5ͨ��1���벶������ 
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void input_init()
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;//����һ���ṹ�������������ʼ����ʱ��
	TIM_ICInitTypeDef TIM5_ICInitStructure;

	NVIC_InitTypeDef NVIC_InitStructure;

	/* ������ʱ��5ʱ�� */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);	 //ʹ��TIM5ʱ��

	TIM_ClearITPendingBit(TIM5,TIM_IT_Update|TIM_IT_CC1); //����жϺͲ����־λ

	TIM_TimeBaseInitStructure.TIM_Period = 0xffff;	 //�趨�������Զ���װֵ 	
	TIM_TimeBaseInitStructure.TIM_Prescaler = 71;   //��1Mhz��Ƶ�ʼ��� һ�μ���1us
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;	  //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	 //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	TIM5_ICInitStructure.TIM_Channel = TIM_Channel_1; //ѡ������� IC1ӳ�䵽TI1�� 
	TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //�����ز��� 
	TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1�� 
	TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; //���������Ƶ,����Ƶ 
	TIM5_ICInitStructure.TIM_ICFilter = 0x00; //IC1F=0000 ���������˲��� ���˲� 
	TIM_ICInit(TIM5, &TIM5_ICInitStructure); //��ʼ��TIM5���벶��ͨ��1	
	
	//�жϷ����ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn;	//��TIM5��ȫ���ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;	//��ռ���ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1; //��Ӧ���ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	  //ʹ��
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM5,ENABLE); //ʹ�ܻ���ʧ��TIMx����
	TIM_ITConfig(TIM5, TIM_IT_Update|TIM_IT_CC1, ENABLE );	//ʹ�ܻ���ʧ��ָ����TIM�ж�
		
}
//��������������ʼ����PD13�ӳ�����������input����
void RBURF02_init()
{
    GPIO_InitTypeDef GPIO_InitStructure; //����һ���ṹ�������������ʼ��GPIO

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);

	/*  ����GPIO��ģʽ��IO�� */
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;  //ѡ����Ҫ���õ�IO��
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;	 //�����������ģʽ
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	  //���ô�������
	GPIO_Init(GPIOD,&GPIO_InitStructure); 	   /* ��ʼ��GPIO */
   	
}
//���������������ؾ���
float RBURF02_distance()
{
	u32 temp;
	float x1,distance1;
	GPIO_ResetBits(GPIOD,GPIO_Pin_13); //ʹ���������������źŽӿڵ͵�ƽ2 ��s 
	delay_us(2); 
	GPIO_SetBits(GPIOD,GPIO_Pin_13); //ʹ���������������źŽӿڸߵ�ƽ10��s ������������10��s 
	delay_us(10); 
	GPIO_ResetBits(GPIOD,GPIO_Pin_13); // ���ַ����������źŽӿڵ͵�ƽ
	if((TIM5CH1_CAPTURE_STA&0x80))	  //�ɹ�������һ��������
	{
		temp=TIM5CH1_CAPTURE_STA&0x3f;
		temp=temp*65536;		  //���ʱ���ܺ�
		temp=temp+TIM5CH1_CAPTURE_VAL;//�õ��ܵĸߵ�ƽʱ��
		TIM5CH1_CAPTURE_STA=0; //������һ�β���	
	}
	distance1 = temp;  //  �������������ʱ�� 
	distance1 = distance1/58;       // ������ʱ��ת��Ϊ���루��λ�����ף� 
	x1 = distance1 * 100.0;
	distance1 = x1 / 100.0; //������λС�� 
	return	distance1;
}
void TIM5_IRQHandler()	  //��ʱ��5���벶���жϺ���
{
	if((TIM5CH1_CAPTURE_STA&0X80)==0)//��δ�ɹ�����
	{
		if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)	
		{
			if(TIM5CH1_CAPTURE_STA&0X40) //�Ѿ����񵽸ߵ�ƽ��
			{
				if((TIM5CH1_CAPTURE_STA&0x3f)==0x3f)//�ߵ�ƽ̫����
				{	
					TIM5CH1_CAPTURE_STA|=0x80;	//��ǳɹ�������һ��
					TIM5CH1_CAPTURE_VAL=0xffff;		
				}
				else
				{
					TIM5CH1_CAPTURE_STA++;	
				}
			}
		}	
	}
	if (TIM_GetITStatus(TIM5, TIM_IT_CC1) != RESET) //����1���������¼�
	{
		if(TIM5CH1_CAPTURE_STA&0X40) //����һ���½���
		{
			TIM5CH1_CAPTURE_STA|=0X80; //��ǳɹ�����һ�������� 
			TIM5CH1_CAPTURE_VAL=TIM_GetCapture1(TIM5); //���TIMx���벶��1��ֵ
			TIM_OC1PolarityConfig(TIM5,TIM_ICPolarity_Rising); //����Ϊ�����ز���	
		}
		else
		{
			TIM5CH1_CAPTURE_STA=0; //��� 
			TIM5CH1_CAPTURE_VAL=0; 
			TIM_SetCounter(TIM5,0); 
			TIM5CH1_CAPTURE_STA|=0X40; //��ǲ����������� 
			TIM_OC1PolarityConfig(TIM5,TIM_ICPolarity_Falling); //����Ϊ�½��ز���
		}	
	}
	TIM_ClearITPendingBit(TIM5, TIM_IT_CC1|TIM_IT_Update); //����жϱ�־λ
}

