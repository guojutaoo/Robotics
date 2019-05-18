#include "VacuumGenerator.h"

/*******************************************************************************
* �� �� ��         : adc_init
* ��������		   : IO�˿�ʱ�ӳ�ʼ������	   
//ѹ��������ģ��������ڷֱ����PC0��PC1��PC2��PC3��
*******************************************************************************/
void adc_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO|RCC_APB2Periph_ADC1,ENABLE);

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);//12M  ���14M ����ADCʱ�ӣ�ADCCLK��

	GPIO_InitStructure.GPIO_Pin=PresureSensor1|PresureSensor2|PresureSensor3|PresureSensor4;//����ѹ��������ģ��������ڣ�PC0��PC1��PC2��PC3
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;	//ģ������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);


	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; 
	ADC_InitStructure.ADC_ScanConvMode = DISABLE; 
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; 
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; 
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 
	ADC_InitStructure.ADC_NbrOfChannel = 1; 
	ADC_Init(ADC1, &ADC_InitStructure);
	
	//����ָ��ADC�Ĺ�����ͨ�����������ǵ�ת��˳��Ͳ���ʱ��
	ADC_RegularChannelConfig(ADC1,ADC_Channel_1,1,ADC_SampleTime_239Cycles5);
	
	ADC_Cmd(ADC1,ENABLE);	

	ADC_ResetCalibration(ADC1);//����ָ����ADC��У׼�Ĵ���
	while(ADC_GetResetCalibrationStatus(ADC1));//��ȡADC����У׼�Ĵ�����״̬
	
	ADC_StartCalibration(ADC1);//��ʼָ��ADC��У׼״̬
	while(ADC_GetCalibrationStatus(ADC1));//��ȡָ��ADC��У׼����

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);//ʹ�ܻ���ʧ��ָ����ADC�����ת����������
}
//��շ�������ʼ������
//��շ���������������PE8.9.10.11���ͷŷ�����PE12.13.14.15
void VacuumGenerator_init()
{
	GPIO_InitTypeDef GPIO_InitStructure; //����һ���ṹ�������������ʼ��GPIO
	SystemInit();	//ʱ�ӳ�ʼ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);

	/*  ����GPIO��ģʽ��IO�� */
	GPIO_InitStructure.GPIO_Pin=SupplyValve1|SupplyValve2|SupplyValve3|SupplyValve4|ReleaseValve1|ReleaseValve2|ReleaseValve3|ReleaseValve4;  //������շ��������Ƶ�IO��
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;	 	//�����������ģʽ
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	  	//���ô�������
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	 	
}
//ѹ��ת������
//���ɼ���ADֵת��Ϊѹ��ֵ
float  Pressure_convert()
{
 	float ad=0;
	u8 i;
	while(1)
	{
		ad=0;
		for(i=0;i<50;i++)//��ȡ50�ε�AD��ֵȡ��ƽ������Ϊ׼ȷ	
		{
			ADC_SoftwareStartConvCmd(ADC1, ENABLE);
			while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));//ת��������־λ
			ad=ad+ADC_GetConversionValue(ADC1);//�������һ��ADCx�������ת�����		
		}
		ad=(ad*5/50/4096-3)*50;
		return ad;		
	}		

}



