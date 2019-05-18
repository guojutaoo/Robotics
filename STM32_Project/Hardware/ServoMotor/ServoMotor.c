#include "ServoMotor.h"
#include "systick.h"
#include"CRC16.h"
#include "math.h"
//���ջ����� 	
u8 RS485_RX_BUF[64]; //���ջ���,���64���ֽ�.
u8 RS485_RX_CNT=0;	 //���յ������ݳ���
u8 flagstart=0;		//��ʾ�������ɼ����ݿ�ʼ��־
u8 hhbflag=1; 		//��ز�У���־


/*******************************************************************************
* �� �� ��         : rs485_init
* ��������		   : IO�˿ڼ�����2��ʱ�ӳ�ʼ������	   
* TX-485��PA2��RX-485��PA3��CS-485��PA8
*******************************************************************************/
void rs485_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;	 //����һ���ṹ�������������ʼ��GPIO

	USART_InitTypeDef USART_InitStructure;	 //���ڽṹ�嶨��

	NVIC_InitTypeDef NVIC_InitStructure;	//�жϽṹ�嶨��

	//��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);

	/*  ����485���߽��ա����͡�ƬѡGPIO��ģʽ��IO�� */
	//485���߷��Ͷ˿�
	GPIO_InitStructure.GPIO_Pin=TX_485;					//TX-485,485���ڷ��Ͷ˿ڣ����PA2
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;		//�����������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	
	GPIO_Init(GPIOA,&GPIO_InitStructure);				 	
	//485�����շ�Ƭѡ�˿�
	GPIO_InitStructure.GPIO_Pin=CS_485;					//CS-485��PA8,�շ�ʹ�ܶˣ�0���գ�1����
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;	    //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);		   


   //485���߽��ն˿�
	GPIO_InitStructure.GPIO_Pin=RX_485;					//RX-485��485���ڽ��ն˿�����PA3
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;	//ģ������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);			 /* ��ʼ��GPIO */


	USART_InitStructure.USART_BaudRate = 9600;			   		//����������Ϊ9600
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//���ݳ�8λ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;	   //1λֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;		   //��Ч��
	USART_InitStructure.USART_HardwareFlowControl =USART_HardwareFlowControl_None; //ʧ��Ӳ����
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;		 			//�������ͺͽ���ģʽ
	USART_Init(USART2, &USART_InitStructure);	   /* ��ʼ��USART2 */
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);//ʹ�ܴ���2�����ж�
	USART_ClearFlag(USART2,USART_FLAG_TC);	 	//���USARTx�Ĵ������־λ
	USART_Cmd(USART2,ENABLE);					//ʹ�ܴ���2

	/* ����NVIC���� */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; 	   		//��USART2��ȫ���ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//��ռ���ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 		 	//��Ӧ���ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 		 	//ʹ��
	NVIC_Init(&NVIC_InitStructure);

}
void USART2_IRQHandler(void)
{
	u8 res;
//	USART_ClearFlag(USART2,USART_FLAG_TC);	 //���ܼӣ�����ֻ�ܷ���һ�����ݣ��ڶ��η��Ͳ���   
	if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET)
	{	 
		res=USART_ReceiveData(USART2);			 
		if(RS485_RX_CNT<9)
		{
			RS485_RX_BUF[RS485_RX_CNT]=res;		//��¼���յ���ֵ
			RS485_RX_CNT++;						//������������1 
		} 
	}  											 
}
/*******************************************************************************
* �� �� ��         : ServoMotor_Init
* ��������		   : ServoMotor��ʼ������������rs485��ʼ��
* CA1.CB1 -	CA6.CB6	����PD��
* ��    ��         : ��
*******************************************************************************/
void ServoMotor_Init()	  //�˿ڳ�ʼ��
{
	GPIO_InitTypeDef GPIO_InitStructure; //����һ���ṹ�������������ʼ��GPIO
	SystemInit();	//ʱ�ӳ�ʼ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);

	/*  ����GPIO��ģʽ��IO�� */
	GPIO_InitStructure.GPIO_Pin=CA1|CB1|CA2|CB2|CA3|CB3|CA4|CB4|CA5|CB5|CA6|CB6|CA7|CB7;  //���õ��������Ƶ�IO��
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;	 	//�����������ģʽ
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	  	//���ô�������
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	rs485_init();	//rs485��ʼ��
}
////****RS485�������ݺ���****////
//RS485����len���ֽ�.
//buf:�������׵�ַ
//len:���͵��ֽ���(Ϊ�˺ͱ�����Ľ���ƥ��,���ｨ�鲻Ҫ����64���ֽ�)
void RS485_Send_Data(u8 *buf,u8 len)
{
	u8 t;
	CS_485_Send;	  	//��������Ƭѡ,����Ϊ����ģʽ
	delay_ms(1);		//�ȴ�����Ƭѡ�ź��ȶ�
  	for(t=0;t<len;t++)	//ѭ����������
	{
		while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);//�ȴ��ֽڷ������		  
		USART_SendData(USART2,buf[t]);			 				//��������
	}
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);	//�ȴ��ֽڷ������	 
	RS485_RX_CNT=0;	  
	CS_485_Receive;	 	//����Ϊ����ģʽ 
	delay_ms(1);					
}
/****RS485�������ݺ���****/
//RS485��ѯ���յ�������
//buf:���ջ����׵�ַ
//len:���������ݳ���
void RS485_Receive_Data(u8 *buf,u8 *len)
{
	u8 rxlen=RS485_RX_CNT;
	u8 i=0;
	*len=0;							//Ĭ��Ϊ0
	delay_ms(10);					//�ȴ�10ms,��������10msû�н��յ�һ������,����Ϊ���ս���
	if(rxlen==RS485_RX_CNT&&rxlen)	//���յ�������,�ҽ��������
	{
		for(i=0;i<rxlen;i++)
		{
			buf[i]=RS485_RX_BUF[i];	
		}		
		*len=RS485_RX_CNT;			//��¼�������ݳ���
		RS485_RX_CNT=0;				//����
	}
} 
/****��ز�У�麯��****/
//�������ܣ�У���ŷ������ز���Ӧ������2���յ���ز���Ӧ����λ���������ݲɼ���ʼ��־λ
//DJbh:������
void Check_hhb(u8 DJbh)
{
	u8 hhbbuf[8]={0x01,0x08,0x00,0x00,0x01,0x02,0x60,0x5A};
	u8 buf[64];
	u8 len;
	u16 crc;
	switch(DJbh)
	{
		case 1:
			hhbbuf[0]=0x01;		//�ֺ�1
			hhbflag=1;
			break;
		case 2:
			hhbbuf[0]=0x02;	 	//�ֺ�2
			hhbflag=1;
			break;
		case 3:
			hhbbuf[0]=0x03;		//�ֺ�3
			hhbflag=1;
			break;
		case 4:
			hhbbuf[0]=0x04;		//�ֺ�4
			hhbflag=1;
			break;
		case 5:
			hhbbuf[0]=0x05;		//�ֺ�5
			hhbflag=1;
			break;
		case 6:
			hhbbuf[0]=0x06;		//�ֺ�6
			hhbflag=1;
			break;
		case 7:
			hhbbuf[0]=0x07;		//�ֺ�7
			hhbflag=1;
			break;
		default:
	  		break;
	} 		
	crc=Get_CRC16(hhbbuf,6);//����CRCУ��ֵ	
	hhbbuf[7]=crc>>8;	 	//CRCУ���8λ
	hhbbuf[6]=crc&0xff;  	//CRCУ���8λ
	if(hhbflag==1)
	{	
		hhbflag=0;
		RS485_Send_Data(hhbbuf,8);	//��ز�У��8���ֽ� 									   
	}
	delay_ms(20);					//�ȴ���������������ȫ
	RS485_Receive_Data(buf,&len);
	if(len)							//���485���߽��յ���8�����ݣ�����к�ز�У��
	{
		if(buf[0]==hhbbuf[0]&&buf[1]==hhbbuf[1]&&buf[2]==hhbbuf[2]&&buf[3]==hhbbuf[3]&&buf[4]==hhbbuf[4]&&buf[5]==hhbbuf[5]&&buf[6]==hhbbuf[6]&&buf[7]==hhbbuf[7])
		{
			flagstart=1; 			//��λ�������ɼ����ݿ�ʼ��־
		}
	}
}
/****��ȡ���������ݺ���****/
//���ܣ��������֡�Ľ���,���ù��ܺ���,��ִ�������������ض������ɼ����������ݣ�
//DJbh:������
u32 Encoder_Value(u8 DJbh)
{
	u8 	buf[64];
	u16 crc;
	u8 crch,crcl,len;
	u32 BMvalue=0; //��������������
	u8 buftxd[8]={0x01,0x03,0x10,0x06,0x00,0x02,0x20,0xCA};//���ڷ����ֽڴ�������(��ȡ�ֺ�Ϊ1�ı���������)
	switch(DJbh)
	{
		case 1:
			buftxd[0]=0x01;	//�ֺ�1
			break;
		case 2:
			buftxd[0]=0x02;	//�ֺ�2
			break;
		case 3:
			buftxd[0]=0x03;	//�ֺ�3
			break;
		case 4:
			buftxd[0]=0x04;	//�ֺ�4
			break;
		case 5:
			buftxd[0]=0x05;	//�ֺ�5
			break;
		case 6:
			buftxd[0]=0x06;	//�ֺ�6
			break;
		case 7:
			buftxd[0]=0x07;	//�ֺ�7
			break;
		default:
	  		break;
	} 		
	crc=Get_CRC16(buftxd,6);//����CRCУ��ֵ	
	buftxd[7]=crc>>8;	 	//CRCУ���8λ
	buftxd[6]=crc&0xff;  	//CRCУ���8λ	
	if(flagstart) //�������ɼ�����ﵽʱ����ȡ���������
	{
		flagstart=0;
		RS485_Send_Data(buftxd,8);//��Ƭ�����ŷ����������Ͷ�ȡ����������ָ�8���ֽ�
	}
	delay_ms(20);	//�ȴ���������������ȫ
	RS485_Receive_Data(buf,&len);
	if(len)			//���485���߽��յ������ݣ���ʼ��ȡ����������
	{
		if(buf[0]==buftxd[0]&&buf[1]==0x03&&buf[2]==0x04) //ƥ�䱾����ַ�Ƿ�Ϊbuftxd[0](�뺯�����ݽ����ĵ�������ͬ)
		{
			crc=Get_CRC16(buf,7);//����CRCУ��ֵ	
			crch=crc>>8;
			crcl=crc&0xff;
			if((buf[7]==crcl)&&(buf[8]==crch)) //У��crcֵ
			{			
				BMvalue=((buf[3]<<24)|(buf[4]<<16)|(buf[5]<<8)|buf[6]);
			}			
		}
	}				
	return BMvalue;				
}
/****������������ת��Ϊ��ǰ�Ƕ�ֵ����****/
//DJbh:�����ţ�
//value:�������ɼ���������DJAngle�������ǰλ�ýǶ�ֵ
float Encoder_Angle(u8 DJbh)
{
	float DJAngle,k;
	u32  value;	
	Check_hhb(DJbh);	// ��ز�У��  
	value=Encoder_Value(DJbh);//��ȡ������ֵ 
	if(DJbh==1||DJbh==2)
	{
	 k=10.0; //ÿ����10������
	}
	else
	{
	 k=1000.0;//ÿ����1000������
	}
	if(value/1000000000==4)//�������˳ʱ��ת��λ��
	{
	   DJAngle=(value-4294967295)/k;
	}
	else  //���������ʱ��ת��λ��
	{
		DJAngle=value/k;   //ֵΪ��
	}
   return  DJAngle;
}
/****** ���ԭ�㸴�麯��*****/
//DJbh:�����ţ�
//3600�������Ӧ360�� ,ÿ����10������
//value:�������ɼ���������DJAngle�������ת�Ƕ�
//�޸ģ���u32 DJAngleȫ���ĳ�float��
void ServoMotor_Reset(u8 DJbh)	  
{	
	u32  value;
	float DJAngle,k;	
	Check_hhb(DJbh);	// ��ز�У��  
	value=Encoder_Value(DJbh);//��ȡ������ֵ
	if(DJbh==1||DJbh==2)
	{
	 k=10.0; //ÿ����10������
	}
	else
	{
	 k=1000.0;//ÿ����1000������
	}
 	if(value/1000000000==4)//�������˳ʱ��ת��λ��
	{
		DJAngle=(4294967295-value)/k;
		ServoMotor_TurnAngle(DJAngle,0,DJbh);	//���1��ʱ��ת�ص�ԭ��λ��
	}
	else  //���������ʱ��ת��λ��
	{
		 DJAngle=value/k;
		 ServoMotor_TurnAngle(DJAngle,1,DJbh);	//���1˳ʱ��ת�ص�ԭ��λ��
	}
}

/****** �����ת����*****/
//DJdir:0:��ʱ��,1: ˳ʱ��;DJbh:�����ţ�DJAngle�������ת�Ƕ�
//3600*100�������Ӧ360�� ,ÿ����1000������,DJsteps�������ת������
//�޸ģ���u32 DJAngleȫ���ĳ�float��
void ServoMotor_TurnAngle(float DJAngle,u8 DJdir,u8 DJbh)	  
{
	u32 i,DJsteps;
	switch(DJbh)
	{
		case 1:
			DJsteps=DJAngle*10;//�������ת�Ƕ�ת��Ϊ��������ÿ����1000������
			if(DJdir==1)
			{
				GPIO_SetBits(GPIOD,CB1); //���1˳ʱ����ת     
			}
			else
			{
				GPIO_ResetBits(GPIOD,CB1);//���1��ʱ��ת��
			}
			for(i=0;i<DJsteps;i++)
			{ 
				GPIO_ResetBits(GPIOD,CA1);
				delay_us(60);
				GPIO_SetBits(GPIOD,CA1);
				delay_us(60);
			}
			break;
		case 2:
			DJsteps=DJAngle*10;//�������ת�Ƕ�ת��Ϊ��������ÿ����1000������
			if(DJdir==1)
			{
				GPIO_SetBits(GPIOD,CB2); //���2˳ʱ����ת     
			}
			else
			{
				GPIO_ResetBits(GPIOD,CB2);//���2��ʱ��ת��
			}
			for(i=0;i<DJsteps;i++)
			{ 
				GPIO_ResetBits(GPIOD,CA2);
				delay_us(20);//��ʱԼΪ3us
				GPIO_SetBits(GPIOD,CA2);
				delay_us(20);//��ʱԼΪ3us
			}
			break;
		case 3:
			DJsteps=DJAngle*1000;//�������ת�Ƕ�ת��Ϊ��������ÿ����1000������
			if(DJdir==1)
			{
				GPIO_SetBits(GPIOD,CB3); //���3˳ʱ����ת     
			}
			else
			{
				GPIO_ResetBits(GPIOD,CB3);//���3��ʱ��ת��
			}
			for(i=0;i<DJsteps;i++)
			{ 
				GPIO_ResetBits(GPIOD,CA3);
				delay_us(60);//��ʱԼΪ3us
				GPIO_SetBits(GPIOD,CA3);
				delay_us(60);//��ʱԼΪ3us
			}
			break;
		case 4:
			DJsteps=DJAngle*1000;//�������ת�Ƕ�ת��Ϊ��������ÿ����1000������
			if(DJdir==1)
			{
				GPIO_SetBits(GPIOD,CB4); //���4˳ʱ����ת     
			}
			else
			{
				GPIO_ResetBits(GPIOD,CB4);//���4��ʱ��ת��
			}
			for(i=0;i<DJsteps;i++)
			{ 
				GPIO_ResetBits(GPIOD,CA4);
				delay_us(60);//��ʱԼΪ3us
				GPIO_SetBits(GPIOD,CA4);
				delay_us(60);//��ʱԼΪ3us
			}
			break;
		case 5:
			DJsteps=DJAngle*1000;//�������ת�Ƕ�ת��Ϊ��������ÿ����1000������
			if(DJdir==1)
			{
				GPIO_SetBits(GPIOD,CB5); //���5˳ʱ����ת     
			}
			else
			{
				GPIO_ResetBits(GPIOD,CB5);//���5��ʱ��ת��
			}
			for(i=0;i<DJsteps;i++)
			{ 
				GPIO_ResetBits(GPIOD,CA5);
				delay_us(60);//��ʱԼΪ3us
				GPIO_SetBits(GPIOD,CA5);
				delay_us(60);//��ʱԼΪ3us
			}
			break;
		case 6:
			DJsteps=DJAngle*1000;//�������ת�Ƕ�ת��Ϊ��������ÿ����1000������
			if(DJdir==1)
			{
				GPIO_SetBits(GPIOD,CB6); //���5˳ʱ����ת     
			}
			else
			{
				GPIO_ResetBits(GPIOD,CB6);//���5��ʱ��ת��
			}
			for(i=0;i<DJsteps;i++)
			{ 
				GPIO_ResetBits(GPIOD,CA6);
				delay_us(60);//��ʱԼΪ3us
				GPIO_SetBits(GPIOD,CA6);
				delay_us(60);//��ʱԼΪ3us
			}
			break;
		case 7:
			DJsteps=DJAngle*1000;//�������ת�Ƕ�ת��Ϊ��������ÿ����1000������
			if(DJdir==1)
			{
				GPIO_SetBits(GPIOD,CB7); //���5˳ʱ����ת     
			}
			else
			{
				GPIO_ResetBits(GPIOD,CB7);//���5��ʱ��ת��
			}
			for(i=0;i<DJsteps;i++)
			{ 
				GPIO_ResetBits(GPIOD,CA7);
				delay_us(200);//��ʱԼΪ3us
				GPIO_SetBits(GPIOD,CA7);
				delay_us(200);//��ʱԼΪ3us
			}
			break;
		default:
	  		break;
	} 		
}
 

