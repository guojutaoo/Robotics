#include "ElectricPutter.h"
#include "systick.h"
#include "iic.h"
/*******************************************************************************
* �� �� ��         : ElectricPutter_Init
* ��������		   : ElectricPutter��ʼ����������I2C��ʼ��
* NP1.PP1 -	NP8.PP8	����PF�ڣ�NP9.PP9 -	NP15.PP15	����PG��
*BUSY/SETON/INP/ESPON/ALARM�ҽ���I2C1�ϣ������
*SETON/RESET/SVON�ҽ���I2C2�ϣ������
* ��    ��         : ��
*******************************************************************************/
void ElectricPutter_Init()	  //�˿ڳ�ʼ��
{
	GPIO_InitTypeDef GPIO_InitStructure; //����һ���ṹ�������������ʼ��GPIO
	SystemInit();						 //ʱ�ӳ�ʼ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF|RCC_APB2Periph_GPIOG,ENABLE);

	/*  ����GPIO��ģʽ��IO�� */
	GPIO_InitStructure.GPIO_Pin=NP1|PP1|NP2|PP2|NP3|PP3|NP4|PP4|NP5|PP5|NP6|PP6|NP7|PP7|NP8|PP8;  //ѡ����Ҫ���õ�IO��
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;	  //�����������ģʽ
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	  //���ô�������
	GPIO_Init(GPIOF,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=NP9|PP9|NP10|PP10|NP11|PP11|NP12|PP12|NP13|PP13|NP14|PP14|NP15|PP15;  //ѡ����Ҫ���õ�IO��
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;	 //�����������ģʽ
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	  //���ô�������
	GPIO_Init(GPIOG,&GPIO_InitStructure);
	I2C2_INIT();			//I2C2��ʼ��
	I2C1_INIT();			//I2C1��ʼ��
}

//QJaddrΪCAT9555����iic��ַ,MLByte:�����ֽ�
//��CAT9555�Ķ�������Ҫ�������裺��һ.д���üĴ���������չ��I/O����Ϊ����ڣ��ڶ�.������˿ڼĴ���
//BUSY/SETON/INP/ESPON/ALARM�ҽ���I2C1�ϣ������
u16 CAT9555_ReadByte(u8 QJaddr)
{
 	u16 temp=0;
	I2C1_Start();   				//����IIC����
	I2C1_Send_Byte(QJaddr);			//����CAT9555����iic��ַ�����һλΪ0д����
	I2C1_Wait_Ack();				//�ȴ��ӻ�Ӧ���ź�
	I2C1_Send_Byte(0x06);			//���������ֽڣ����ö˿�0
	I2C1_Wait_Ack();	   			//�ȴ��ӻ�Ӧ���ź�
	I2C1_Send_Byte(0xFF); 			//�������ö˿ڼĴ���0ֵ����ǰ�˸�io������Ϊ����
	I2C1_Wait_Ack();				//�ȴ��ӻ�Ӧ���ź�
	I2C1_Send_Byte(0xFF); 			//�������ö˿ڼĴ���1���ݣ�����˸�io������Ϊ����
	I2C1_Wait_Ack();				//�ȴ��ӻ�Ӧ���ź�
	I2C1_Stop();	 				//��������
	//������˿ڼĴ���			
	I2C1_Start();  					//����IIC����
	I2C1_Send_Byte(QJaddr);			//����CAT9555����iic��ַ�����һλΪ0д����
	I2C1_Wait_Ack();	   			//�ȴ��ӻ�Ӧ���ź�
	I2C1_Send_Byte(0x01);			//���������ֽڣ���������˿�1
	I2C1_Wait_Ack();	   			//�ȴ��ӻ�Ӧ���ź�

	I2C1_Start();  					//��������IIC����
	QJaddr=QJaddr+1;			//��CAT9555����iic��ַ���һλ��Ϊ1������
	I2C1_Send_Byte(QJaddr);			//����CAT9555����iic��ַ�����һλΪ1������
	I2C1_Wait_Ack();	   			//�ȴ��ӻ�Ӧ���ź�
//	temp=I2C1_Read_Byte(0); 		// 0���� NACK��ÿ����һ���ֽڣ��ӵ�ַ����

	temp=I2C1_Read_Byte(1); //  1   ���� ACK ,��ȡ�˿�1��ֵ
	temp<<=8;
	temp|=I2C1_Read_Byte(0); //  0  ���� NACK,��ȡ�˿�0��ֵ

	I2C1_NAck();	 //�ȴ�����Ӧ���ź�
	I2C1_Stop();	 //��������	
	return temp;	

}

//QJaddrΪCAT9555����iic��ַ,MLByte:�����ֽ�
//��CAT9555��д������Ҫ�������裺��һ.д���üĴ���������չ��I/O����Ϊ����ڣ��ڶ�.д����˿ڼĴ���
//д�������Ƕ�SETON/RESET/SVON������ڣ���I2C2
//dt0��Ӧ�˿�0��dt1��Ӧ�˿�1��0��������1����������
void CAT9555_WriteByte(u8 QJaddr,u8 dt0,u8 dt1)
{
	//д���üĴ���
	I2C2_Start();   				//����IIC����
	I2C2_Send_Byte(QJaddr);			//����CAT9555����iic��ַ�����һλΪ0д����
	I2C2_Wait_Ack();				//�ȴ��ӻ�Ӧ���ź�
	I2C2_Send_Byte(0x06);			//���������ֽڣ����ö˿�0
	I2C2_Wait_Ack();	   			//�ȴ��ӻ�Ӧ���ź�
	I2C2_Send_Byte(0x00); 			//�������ö˿ڼĴ���0ֵ����ǰ�˸�io������Ϊ���
	I2C2_Wait_Ack();				//�ȴ��ӻ�Ӧ���ź�
	I2C2_Send_Byte(0x00); 			//�������ö˿ڼĴ���1���ݣ�����˸�io������Ϊ���
	I2C2_Wait_Ack();				//�ȴ��ӻ�Ӧ���ź�
	I2C2_Stop();	 				//��������
	//д����˿ڼĴ���
	I2C2_Start();   				//����IIC����
	I2C2_Send_Byte(QJaddr);			//����CAT9555����iic��ַ�����һλΪ0д����
	I2C2_Wait_Ack();				//�ȴ��ӻ�Ӧ���ź�
	I2C2_Send_Byte(0x02);			//���������ֽڣ�����˿�0
	I2C2_Wait_Ack();	   			//�ȴ��ӻ�Ӧ���ź�
	I2C2_Send_Byte(dt0); 			//��������˿ڼĴ���0����
	I2C2_Wait_Ack();				//�ȴ��ӻ�Ӧ���ź�
	I2C2_Send_Byte(dt1); 			//��������˿ڼĴ���1����
	I2C2_Wait_Ack();				//�ȴ��ӻ�Ӧ���ź�
	I2C2_Stop();	 				//��������
	delay_ms(10); 	
}
//�㶯����ģʽ������ƶ�����
//DGlen:����ƶ�����MM��DGdir��0���ˣ���ת����1ǰ��(��ת) ,DGbh:��˱��
//�������ģʽ��3
//˵����steps���㹫ʽֻ�ܷ���case����У�������һֱ��������stepsֻ����u32����int�ͱ���
void ElectricPutter_DDMove(float DGlen,u8 DGdir,u8 DGbh)	  //����ƶ�����
{
   	u32 i,steps;
	switch(DGbh)
	{
		case 1:
			steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP1);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP1);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP1);
				delay_us(80);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOF,PP1);
				delay_us(80);			//��ʱԼΪ8us
			}
	  	break;
		case 2:
			steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP2);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP2);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP2);
				delay_us(80);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOF,PP2);
				delay_us(80);			//��ʱԼΪ8us
			}
	  	break;
		case 3:
			steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP3);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP3);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP3);
				delay_us(80);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOF,PP3);
				delay_us(80);			//��ʱԼΪ8us
			}
	  	break;
		case 4:
			steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP4);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP4);
			}
			
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP4);
				delay_us(80);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOF,PP4);
				delay_us(80);			//��ʱԼΪ8us
			}
	  	break;
		case 5:
			steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP5);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP5);
			}			
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP5);
				delay_us(80);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOF,PP5);
				delay_us(80);			//��ʱԼΪ8us
			}
	  	break;
		case 6:
			steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP6);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP6);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP6);
				delay_us(80);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOF,PP6);
				delay_us(80);			//��ʱԼΪ8us
			}
	  	break;
		case 7:
			steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP7);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP7);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP7);
				delay_us(80);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOF,PP7);
				delay_us(80);			//��ʱԼΪ8us
			}
	  	break;
		case 8:
			steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP8);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP8);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP8);
				delay_us(80);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOF,PP8);
				delay_us(80);			//��ʱԼΪ8us
			}
	  	break;
		case 9:
			steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOG,NP9);      
			}
			else
			{
				GPIO_ResetBits(GPIOG,NP9);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOG,PP9);
				delay_us(80);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOG,PP9);
				delay_us(80);			//��ʱԼΪ8us
			}
	  	break;
		case 10:
			steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOG,NP10);      
			}
			else
			{
				GPIO_ResetBits(GPIOG,NP10);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOG,PP10);
				delay_us(80);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOG,PP10);
				delay_us(80);			//��ʱԼΪ8us
			}
	  	break;
		case 11:
			steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOG,NP11);      
			}
			else
			{
				GPIO_ResetBits(GPIOG,NP11);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOG,PP11);
				delay_us(80);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOG,PP11);
				delay_us(80);			//��ʱԼΪ8us
			}									  
	  	break;
		case 12:
			steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOG,NP12);      
			}
			else
			{
				GPIO_ResetBits(GPIOG,NP12);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOG,PP12);
				delay_us(80);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOG,PP12);
				delay_us(80);			//��ʱԼΪ8us
			}
	  	break;
		case 13:
			steps=800*DGlen/4;	//800�������Ӧ4mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOG,NP13);      
			}
			else
			{
				GPIO_ResetBits(GPIOG,NP13);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOG,PP13);
				delay_us(80);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOG,PP13);
				delay_us(80);			//��ʱԼΪ8us
			}
	  	break;
		case 14:
			steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOG,NP14);      
			}
			else
			{
				GPIO_ResetBits(GPIOG,NP14);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOG,PP14);
				delay_us(80);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOG,PP14);
				delay_us(80);			//��ʱԼΪ8us
			}
	  	break;
		case 15:
			steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOG,NP15);      
			}
			else
			{
				GPIO_ResetBits(GPIOG,NP15);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOG,PP15);
				delay_us(80);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOG,PP15);
				delay_us(80);			//��ʱԼΪ8us
			}
	  	break;
		default:
	  	break;	
	}
}


//����ƶ�����
//DGlen:����ƶ�����MM��DGdir��0���ˣ���ת����1ǰ��(��ת) ,DGbh:��˱��
//�������ģʽ��3
//˵����steps���㹫ʽֻ�ܷ���case����У�������һֱ��������stepsֻ����u32����int�ͱ���
void ElectricPutter_Move(float DGlen,u8 DGdir,u8 DGbh)	  //����ƶ�����
{
   	u32 i,steps;
	switch(DGbh)
	{
		case 1:
			steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP1);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP1);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP1);
				delay_us(8);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOF,PP1);
				delay_us(8);			//��ʱԼΪ8us
			}
	  	break;
		case 2:
			steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP2);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP2);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP2);
				delay_us(8);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOF,PP2);
				delay_us(8);			//��ʱԼΪ8us
			}
	  	break;
		case 3:
			steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP3);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP3);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP3);
				delay_us(8);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOF,PP3);
				delay_us(8);			//��ʱԼΪ8us
			}
	  	break;
		case 4:
			steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP4);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP4);
			}
			
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP4);
				delay_us(8);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOF,PP4);
				delay_us(8);			//��ʱԼΪ8us
			}
	  	break;
		case 5:
			steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP5);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP5);
			}			
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP5);
				delay_us(8);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOF,PP5);
				delay_us(8);			//��ʱԼΪ8us
			}
	  	break;
		case 6:
			steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP6);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP6);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP6);
				delay_us(8);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOF,PP6);
				delay_us(8);			//��ʱԼΪ8us
			}
	  	break;
		case 7:
			steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP7);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP7);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP7);
				delay_us(8);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOF,PP7);
				delay_us(8);			//��ʱԼΪ8us
			}
	  	break;
		case 8:
			steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP8);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP8);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP8);
				delay_us(8);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOF,PP8);
				delay_us(8);			//��ʱԼΪ8us
			}
	  	break;
		case 9:
			steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOG,NP9);      
			}
			else
			{
				GPIO_ResetBits(GPIOG,NP9);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOG,PP9);
				delay_us(8);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOG,PP9);
				delay_us(8);			//��ʱԼΪ8us
			}
	  	break;
		case 10:
			steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOG,NP10);      
			}
			else
			{
				GPIO_ResetBits(GPIOG,NP10);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOG,PP10);
				delay_us(8);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOG,PP10);
				delay_us(8);			//��ʱԼΪ8us
			}
	  	break;
		case 11:
			steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOG,NP11);      
			}
			else
			{
				GPIO_ResetBits(GPIOG,NP11);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOG,PP11);
				delay_us(8);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOG,PP11);
				delay_us(8);			//��ʱԼΪ8us
			}									  
	  	break;
		case 12:
			steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOG,NP12);      
			}
			else
			{
				GPIO_ResetBits(GPIOG,NP12);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOG,PP12);
				delay_us(8);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOG,PP12);
				delay_us(8);			//��ʱԼΪ8us
			}
	  	break;
		case 13:
			steps=800*DGlen/4;	//800�������Ӧ4mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOG,NP13);      
			}
			else
			{
				GPIO_ResetBits(GPIOG,NP13);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOG,PP13);
				delay_us(800);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOG,PP13);
				delay_us(800);			//��ʱԼΪ8us
			}
	  	break;
		case 14:
			steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOG,NP14);      
			}
			else
			{
				GPIO_ResetBits(GPIOG,NP14);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOG,PP14);
				delay_us(20);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOG,PP14);
				delay_us(20);			//��ʱԼΪ8us
			}
	  	break;
		case 15:
			steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOG,NP15);      
			}
			else
			{
				GPIO_ResetBits(GPIOG,NP15);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOG,PP15);
				delay_us(20);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOG,PP15);
				delay_us(20);			//��ʱԼΪ8us
			}
	  	break;
			case 16:
			steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP7);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP7);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP7);
				delay_us(500);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOF,PP7);
				delay_us(500);			//��ʱԼΪ8us
			}
	  	break;
		case 17:
			steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP8);      
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP8);
			}
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP8);
				delay_us(500);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOF,PP8);
				delay_us(500);			//��ʱԼΪ8us
			}
	  	break;
		default:
	  	break;	
	}
}





