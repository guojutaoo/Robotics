#include "VacuumGenerator.h"

/*******************************************************************************
* 函 数 名         : adc_init
* 函数功能		   : IO端口时钟初始化函数	   
//压力传感器模拟量输入口分别挂在PC0、PC1、PC2、PC3口
*******************************************************************************/
void adc_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO|RCC_APB2Periph_ADC1,ENABLE);

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);//12M  最大14M 设置ADC时钟（ADCCLK）

	GPIO_InitStructure.GPIO_Pin=PresureSensor1|PresureSensor2|PresureSensor3|PresureSensor4;//设置压力传感器模拟量输入口，PC0、PC1、PC2、PC3
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;	//模拟输入
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);


	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; 
	ADC_InitStructure.ADC_ScanConvMode = DISABLE; 
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; 
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; 
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 
	ADC_InitStructure.ADC_NbrOfChannel = 1; 
	ADC_Init(ADC1, &ADC_InitStructure);
	
	//设置指定ADC的规则组通道，设置它们的转化顺序和采样时间
	ADC_RegularChannelConfig(ADC1,ADC_Channel_1,1,ADC_SampleTime_239Cycles5);
	
	ADC_Cmd(ADC1,ENABLE);	

	ADC_ResetCalibration(ADC1);//重置指定的ADC的校准寄存器
	while(ADC_GetResetCalibrationStatus(ADC1));//获取ADC重置校准寄存器的状态
	
	ADC_StartCalibration(ADC1);//开始指定ADC的校准状态
	while(ADC_GetCalibrationStatus(ADC1));//获取指定ADC的校准程序

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);//使能或者失能指定的ADC的软件转换启动功能
}
//真空发生器初始化函数
//真空发生器供给阀接在PE8.9.10.11，释放阀接在PE12.13.14.15
void VacuumGenerator_init()
{
	GPIO_InitTypeDef GPIO_InitStructure; //声明一个结构体变量，用来初始化GPIO
	SystemInit();	//时钟初始化
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);

	/*  配置GPIO的模式和IO口 */
	GPIO_InitStructure.GPIO_Pin=SupplyValve1|SupplyValve2|SupplyValve3|SupplyValve4|ReleaseValve1|ReleaseValve2|ReleaseValve3|ReleaseValve4;  //设置真空发生器控制的IO口
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;	 	//设置推挽输出模式
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	  	//设置传输速率
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	 	
}
//压力转换函数
//将采集的AD值转换为压力值
float  Pressure_convert()
{
 	float ad=0;
	u8 i;
	while(1)
	{
		ad=0;
		for(i=0;i<50;i++)//读取50次的AD数值取其平均数较为准确	
		{
			ADC_SoftwareStartConvCmd(ADC1, ENABLE);
			while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));//转换结束标志位
			ad=ad+ADC_GetConversionValue(ADC1);//返回最近一次ADCx规则组的转换结果		
		}
		ad=(ad*5/50/4096-3)*50;
		return ad;		
	}		

}



