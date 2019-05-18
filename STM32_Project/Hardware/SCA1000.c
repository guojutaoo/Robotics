#include "SCA1000.h"
#include "math.h"
#include "systick.h"
#include "printf.h"

/****************************************************************************
* Function Name  : SCA1000_Init
* Description    : ��ʼ��SCA1000��IO��SPI.
* Input          : None
* Output         : None
* Return         : None
****************************************************************************/

void SCA1000_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
	
	/* SCA1000_CS PG13 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOG, &GPIO_InitStructure);
	GPIO_SetBits(GPIOG, GPIO_Pin_13);


}
/****************************************************************************
* Function Name  : MNSPI_Init
* Description    : ��ʼ��SPI��IO��.
****************************************************************************/
void MNSPI_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure; //����һ���ṹ�������������ʼ��GPIO

	SystemInit();	//ʱ�ӳ�ʼ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	/*  ����GPIO��ģʽ��IO�� */
	GPIO_InitStructure.GPIO_Pin=SPI_MOSI|SPI_CLK;  //ѡ����Ҫ���õ�IO��
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;	 //�����������ģʽ
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	  //���ô�������
	GPIO_Init(GPIOB,&GPIO_InitStructure); 	   /* ��ʼ��GPIO */
   	/*  ����GPIO��ģʽ��IO��-MISO���� */
	GPIO_InitStructure.GPIO_Pin=SPI_MISO;  //ѡ����Ҫ���õ�IO��
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;	//��������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	  //���ô�������
	GPIO_Init(GPIOB,&GPIO_InitStructure); 	   /* ��ʼ��GPIO */
}

//����дһ��BYTE������ 
  //����״̬:CLK = LOW 
  //���ܣ����Ϳ�������
void MNSPI_WriteByte(uint8_t wdata )
{ 
   uint8_t i; 
   for(i = 0;i < 8;i++)  
   { 
	 GPIO_ResetBits(GPIOB,SPI_CLK);
     delay_us(100);
    // if((wdata&0x80)==0x80)
	  if(wdata&0x80)
     {
	   GPIO_SetBits(GPIOB, SPI_MOSI);
     }
     else
     {
	   GPIO_ResetBits(GPIOB,SPI_MOSI);
     }    
      wdata = wdata << 1; 
      GPIO_SetBits(GPIOB,SPI_CLK);
      delay_us(100);     
    } 
   GPIO_ResetBits(GPIOB,SPI_CLK); 
  }
//��������N������
//n ����Ϊ16��������
//����״̬��CLK��=��0
u16 MNSPI_ReadBits(int n)
{
    u16 rtemp=0x0000,i;
    for( i = 0 ; i < n ; i++ )
    {
        rtemp= rtemp<<1;
		GPIO_ResetBits(GPIOB,SPI_CLK);     
        delay_us(100);         
        if (GPIO_ReadInputDataBit(GPIOB,SPI_MISO)==1)
        {
            rtemp = rtemp|0x0001;
        }
        else
        {
            rtemp = rtemp&0xFFFE;
        }	
		GPIO_SetBits(GPIOB,SPI_CLK);
		
        delay_us(100);
    }
    return rtemp; 
	 
}
float MNSPI_GetX(void) 
{ 
  	u16 AXData;
	float realAX;//x��Ƕ�ģ����
    GPIO_ResetBits(GPIOB,SPI_CLK);
	GPIO_ResetBits(GPIOG,SPI_CS);
   	delay_ms(50); 
    MNSPI_WriteByte(0x10);     
    AXData = MNSPI_ReadBits(11);     
    GPIO_ResetBits(GPIOB,SPI_CLK);//ʱ�Ӽ���Ϊ�͵�ƽ��Ч����CPOL=0(�½��ط������ݣ������ؽ�������)��ʱ��Ϊ���� 
	delay_ms(50); 
   	GPIO_SetBits(GPIOG,SPI_CS);
   
	       
	realAX = asin(((float)AXData-1024.0)/492)*(180/3.14);
	return realAX;
}  


