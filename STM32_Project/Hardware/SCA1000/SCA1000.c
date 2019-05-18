#include "SCA1000.h"
#include "math.h"
#include "systick.h"
//#include "spi.h"
/*ȫ�ֱ���*/
float realAX1;		//��ϴ�۴�۽Ƕ�
float realAX2;		//��ϴ��С�۽Ƕ�
float realAX3;		//��ϴ����ϴװ�ýǶ�
u8 SCAflag1=1;      //��ȡ��۽Ƕ����ݱ�־λ
u8 SCAflag2=1;      //��ȡС�۽Ƕ����ݱ�־λ
u8 SCAflag3=1;      //��ȡϴװ�ýǶ����ݱ�־λ

/****************************************************************************
* Function Name  : SPI2_Init
* Description    : ��ʼ��SPI2
* Input          : None
* Output         : None
* Return         : None
****************************************************************************/
void SPI2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

    /* SPI��IO�ں�SPI�����ʱ�� */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

    /* SPI��IO������ SPI2_SCK=PB13,SPI2_MOSI=PB15,SPI2_MIS0=PB14*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15|GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
/***************************************************************************/
/************************* ����SPI�Ĳ��� ***********************************/
/***************************************************************************/
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//ѡ��ȫ˫��SPIģʽ
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;     //����ģʽ
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; //8λSPI
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;       //ʱ�����ոߵ�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;      //�ڵڶ���ʱ�Ӳɼ�����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		  //Nssʹ���������
	/* ѡ������Ԥ��ƵΪ256 */
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;//�����λ��ʼ����
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStructure);
	SPI_Cmd(SPI2, ENABLE);

	SPI2_WriteReadData(0xff);//����spi2

}      

/****************************************************************************
* Function Name  : SCA1000_Init
* Description    : ��ʼ��SCA1000��ƬѡIO��SPI.
SCA1000��ƬѡIO�ֱ����PE5.6.7
****************************************************************************/
void SCA1000_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	 //����һ���ṹ�������������ʼ��GPIO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	
	/* ��ʼ��SCA1000_CS1.2.3 */
	GPIO_InitStructure.GPIO_Pin = SCA1000_CS1|SCA1000_CS2|SCA1000_CS3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	/*�ر�SCA1000Ƭѡ1.2.3*/
	SCA1000_CS1_SET;
	SCA1000_CS2_SET;
	SCA1000_CS3_SET;
	//��ʼ��SCA1000��SPI2
	SPI2_Init();				
}

/****************************************************************************
* Function Name  : SPI2_WriteReadData
* Description    : ʹ��SPI2д��һ���ֽ�����ͬʱ��ȡһ���ֽ����ݡ�
* Input          : dat��д�������
* Output         : None
* Return         : ��ȡ��������
*                * ��ȡʧ�ܷ���0xFF
****************************************************************************/
u8 SPI2_WriteReadData(u8 dat)
{
	uint16_t i = 0;
    /* �����ͻ������� */	
 	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
	{
		i++;
		if(i > 200)
		{
			return 0;
		}
	}
    /* �������� */
   	SPI_I2S_SendData(SPI2, dat);
	i = 0;
	/* �ȴ����ջ�����Ϊ�ǿ� */
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
	{
	   	i++;
		if(i > 200)
		{
			return 0;
		}
	}
 	/* ����ȡ������ֵ���� */
 	return SPI_I2S_ReceiveData(SPI2);		
}

/* SCA1000��ȡ11λ���ݣ�����11λ������ת��Ϊ�Ƕ�ֵ */
//ͬʱ��ȡ��ۡ�С�ۡ���ϴװ�ýǶ�ֵ
void SCA1000_ReadAXData(void)
{
	u32 AXData1=0,AXData2=0,AXData3=0,i;
	u16 readBuff[2]={0};//�ɼ���������������
/***************�ɼ�1�δ�۽Ƕ�ֵ***************/
	if(SCAflag1==1)
	{		
		for(i=0;i<50;) //�ɼ�50��X����������ȡƽ��ֵ
		{
			SCA1000_CS1_CLR;    		//��Ƭѡ			
			SPI2_WriteReadData(0x10);  //д��ȡ������X������������
			readBuff[0]=SPI2_WriteReadData(0xff);	//��ȡ��8λ����
			readBuff[1]=SPI2_WriteReadData(0xff); 	//��ȡ��8λ����
		   	if (((readBuff[0])<<3)|((readBuff[1])>>5)<1517 && ((readBuff[0])<<3)|((readBuff[1])>>5)>531 )
			{ 					
			  	AXData1=AXData1+(((readBuff[0])<<3)|((readBuff[1])>>5));//��16λ����ת����11λ���ݣ����õ�50���ۼ�ֵ
				i++;	   
			}
			SCA1000_CS1_SET;   //�ر�Ƭѡ
			delay_ms(2);
		}
		realAX1 = asin(((float)AXData1/50.0-1024.0)/491.52)*(180/3.14);
//		realAX1 = asin(((float)AXData1/50.0-1024.0)/492)*(180/3.14);
		SCAflag1=0;
		SCA1000_CS1_SET;   //�ر�Ƭѡ
	}
/***************�ɼ�1��С�۽Ƕ�ֵ***************/
	if(SCAflag2==1)
	{		
		for(i=0;i<50;) //�ɼ�50��X����������ȡƽ��ֵ
		{
			SCA1000_CS2_CLR;    		//��Ƭѡ			
			SPI2_WriteReadData(0x10);   //д��ȡ������X������������
			readBuff[0]=SPI2_WriteReadData(0xff);	//��ȡ��8λ����
			readBuff[1]=SPI2_WriteReadData(0xff); 	//��ȡ��8λ����
		   	if (((readBuff[0])<<3)|((readBuff[1])>>5)<1517 && ((readBuff[0])<<3)|((readBuff[1])>>5)>531 )
			{ 
			  	AXData2=AXData2+(((readBuff[0])<<3)|((readBuff[1])>>5));//��16λ����ת����11λ���ݣ����õ�50���ۼ�ֵ
				i++;	   
			}
			SCA1000_CS2_SET;   //�ر�Ƭѡ
			delay_ms(2);
		}
		realAX2 = asin(((float)AXData2/50.0-1024.0)/491.52)*(180/3.14);
		SCAflag2=0;
		SCA1000_CS2_SET;   //�ر�Ƭѡ
	}
/***************�ɼ�1����ϴװ�ýǶ�ֵ***************/
	if(SCAflag3==1)
	{		
		for(i=0;i<50;) //�ɼ�50��X����������ȡƽ��ֵ
		{
			SCA1000_CS3_CLR;    		//��Ƭѡ			
			SPI2_WriteReadData(0x10);   //д��ȡ������X������������
			readBuff[0]=SPI2_WriteReadData(0xff);	//��ȡ��8λ����
			readBuff[1]=SPI2_WriteReadData(0xff); 	//��ȡ��8λ����
		   	if (((readBuff[0])<<3)|((readBuff[1])>>5)<1517 && ((readBuff[0])<<3)|((readBuff[1])>>5)>531 )
			{ 
			  	AXData3=AXData3+(((readBuff[0])<<3)|((readBuff[1])>>5));//��16λ����ת����11λ���ݣ����õ�50���ۼ�ֵ
				i++;	   
			}
			SCA1000_CS3_SET;   //�ر�Ƭѡ
			delay_ms(2);
		}
		realAX3 = asin(((float)AXData3/50.0-1024.0)/491.52)*(180/3.14);
		SCAflag3=0;
		SCA1000_CS3_SET;   //�ر�Ƭѡ
	}
}

