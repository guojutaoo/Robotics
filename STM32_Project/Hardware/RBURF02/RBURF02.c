#include "RBURF02.h"
u8 TIM5CH1_CAPTURE_STA; //���벶��״̬ 
u16 TIM5CH1_CAPTURE_VAL;//���벶��ֵ

/*******************************************************************************
* �� �� ��         : RBURF02_input_init
* ��������		   : ���벶�����ó�ʼ��	   ��ʱ��5ͨ��1���벶������ 
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void RBURF02_input_init()
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
void RBURF02_inputtime()
{
	u32 temp;
	if((TIM5CH1_CAPTURE_STA&0x80))	  //�ɹ�������һ��������
	{
		temp=TIM5CH1_CAPTURE_STA&0x3f;
		temp=temp*65536;		  //���ʱ���ܺ�
		temp=temp+TIM5CH1_CAPTURE_VAL;//�õ��ܵĸߵ�ƽʱ��
		TIM5CH1_CAPTURE_STA=0; //������һ�β���	
	}
}


