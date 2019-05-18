#include "exti.h"
#include "systick.h"
/*******************************************************************************
* �� �� ��         : exti_init
* ��������		   : �ⲿ�ж�2�˿ڳ�ʼ������	   
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void exti_init()  //�ⲿ�жϳ�ʼ��
{
	GPIO_InitTypeDef GPIO_InitStructure;

	EXTI_InitTypeDef EXTI_InitStructure;

	NVIC_InitTypeDef NVIC_InitStructure;

	/* ����GPIOʱ�� */
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	  //����io�ڸ��ù��ܣ��ⲿ�жϣ�
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);

	GPIO_InitStructure.GPIO_Pin=KEY1;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	 
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource0);//ѡ��GPIO�ܽ������ⲿ�ж���·
	//�˴�һ��Ҫ��ס���˿ڹܽż����ж��ⲿ��·
	/* �����ⲿ�жϵ�ģʽ */ 
	EXTI_InitStructure.EXTI_Line=EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure); 
	
	/* ����NVIC���� */   //�ж����ȼ�	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);		 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn; 	//��EXTI0��ȫ���ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //��ռ���ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  //��Ӧ���ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 		  //ʹ��
	NVIC_Init(&NVIC_InitStructure); 		
}
void EXTI0_IRQHandler()	   //�ⲿ�ж�0�жϺ���
{
	if(EXTI_GetITStatus(EXTI_Line0)==SET)
	{
   		EXTI_ClearITPendingBit(EXTI_Line0);//���EXTI��·����λ
		delay_ms(10);//��������
		if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0)==Bit_RESET)	   //key1��������
		{
			delay_ms(10);//��������
			if(GPIO_ReadOutputDataBit(GPIOD,GPIO_Pin_13)==Bit_RESET)
			{
				//LED Ϩ��
			   GPIO_SetBits(GPIOD,GPIO_Pin_13);	
			}
			else
			{
			   //LED ����
				GPIO_ResetBits(GPIOD,GPIO_Pin_13);
			}
		} 
		while(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0)==0);	  //key1�����ɿ�����ѭ��
	}		
}

