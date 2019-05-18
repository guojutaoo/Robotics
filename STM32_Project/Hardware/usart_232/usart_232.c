#include "usart_232.h"
#include "systick.h"
#include "ElectricPutter.h"
#include "ServoMotor.h"
#include "CleanArm.h"
#include "printf.h"
#include "BodyMove.h"
#include "math.h"
u8 RS232_RX_BUF[64]; 	//���ջ���,���64���ֽ�.
u8 RS232_RX_CNT=0;	 	//���յ������ݳ���
u8 CleanArmPathflag=0;  //��ϴ���尴����λ����ϴ�켣��ϴ��ʼ��־��1����ʼ��ϴ
u8 PointMoveflag=0;     //�㶯����ģʽ��ʼ��־��1�������㶯ģʽ
u8 StraightWalkFlag=0;
u8 ForwordWalkFlag=0;
u8 BackwordWalkFlag=0;
u8 LTurnMoveFlag=0;
u8 RTurnMoveFlag=0;
u16 CleanArmPoints=0;  //��λ��������ϴ�������켣�������
int FPointsNum,FRunCycle;
int BPointsNum,BRunCycle;
int LTPointsNum,LTurnCycle;
int RTPointsNum,RTurnCycle;
float LTRoundAngle;
float RTRoundAngle;
float LTCompenLen;
float RTCompenLen;
float FWalkAngle[4];
float BWalkAngle[4];
float LTWalkAngle[4];
float RTWalkAngle[4];
float CleanArmbuf[800]; //��ϴ������㴢�����黺��������λ����ഫ��200*3�����������ݣ����������800mm
float FWalkPoints[800];
float BWalkPoints[800];
float LTWalkPoints[800];
float RTWalkPoints[800];    
//static float fx[4]={4.34,4.29,4.23,4.17};
//static float fy[4]={3.16,3.24,3.31,3.39};
//float Repara[11];       //11�������ȸ����Ͳ���
/*******************************************************************************
 *RS232����ͨѶϵ�к���
*******************************************************************************/
void usart_232_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;	 //����һ���ṹ�������������ʼ��GPIO

	USART_InitTypeDef USART_InitStructure;	 //���ڽṹ�嶨��

	NVIC_InitTypeDef NVIC_InitStructure;	//�жϽṹ�嶨��

	//��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB,ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	 //��ʱ��

	//	/*  ����GPIO��ģʽ��IO�� */
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;//TX			   //�������PA9
//	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;//���е�Ƭ������232����
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;	    //�����������
	GPIO_Init(GPIOA,&GPIO_InitStructure);  /* ��ʼ����������IO */
//	GPIO_Init(GPIOB,&GPIO_InitStructure);  //���е�Ƭ������232����	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;//RX			 //��������PA10
//	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;//TX		  //���е�Ƭ������232����	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;		  //ģ������
	GPIO_Init(GPIOA,&GPIO_InitStructure); /* ��ʼ��GPIO */
//	GPIO_Init(GPIOB,&GPIO_InitStructure);//���е�Ƭ������232����		   


	USART_InitStructure.USART_BaudRate = 9600;			   		//����������Ϊ9600
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//���ݳ�8λ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;	   //1λֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;		   //��Ч��
	USART_InitStructure.USART_HardwareFlowControl =USART_HardwareFlowControl_None; //ʧ��Ӳ����
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;		 			//�������ͺͽ���ģʽ
	USART_Init(USART1, &USART_InitStructure);	   /* ��ʼ��USART2 */
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//ʹ�ܴ���2�����ж�
	USART_ClearFlag(USART1,USART_FLAG_TC);	 	//���USARTx�Ĵ������־λ
	USART_Cmd(USART1,ENABLE);					//ʹ�ܴ���2
	
	/* ����NVIC���� */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; 	   		//��USART3��ȫ���ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//��ռ���ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 		 	//��Ӧ���ȼ�Ϊ2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 		 	//ʹ��
	NVIC_Init(&NVIC_InitStructure);		 

}
void USART1_IRQHandler(void)	//����1�жϺ���
{
	u8 res;
	if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET) //RXNE(�����ݼĴ����ǿ�)����λ���� 1��ʱ�򣬱�ʾ�Ѿ������ݱ����յ���
	{
		res=USART_ReceiveData(USART1);			 
		if(RS232_RX_CNT<46)
		{
			RS232_RX_BUF[RS232_RX_CNT]=res;		//��¼���յ���ֵ
			RS232_RX_CNT++;						//������������1
			if(RS232_RX_BUF[0]==0xB5)
			{
			 	if(	RS232_RX_CNT==30)		  //��ϴ�۹켣�滮�������ֱ���˶�ʱ��λ������30���ֽ�
				{
					//ע�⣺���������麯�������ж���ʱ����λ�����͵�ָ�������30���ֽڣ�
					//����ֻҪ��һ������30���ֽ���������Ͳ���ִ�б������¸�λ���·���ָ��ſ�������ʹ��
					//���Ѵ��溯������main��while(1)��ʱ�����������©��
					//�����ж�����Ҫ�����ǵ���λ�����յ�һ֡����(���������ֽ�)ʱδ��RS232_RX_CNT��buf��������	
					  BufStorage(7); 					//����λ�����͵����ݴ��浽CleanArmbuf[256]��
				}
			}						 
			if(RS232_RX_BUF[0]==0xA5)
			{
				if(	RS232_RX_CNT==46)		  //�㶯����ģʽʱ��λ������46���ֽ�
				{
//					//ע�⣺�����������ж���ʱ��ÿ����һ���ж�ִ��һ�γ��򣬷���main��while(1)�ɲ�ִͣ�г���
//					//��λ�����͵�ָ�������46���ֽڣ�����ֻҪ��һ������46���ֽ���������Ͳ���ִ�б������¸�λ���·���ָ��ſ�������ʹ��
//					//���Ѻ�������main��while(1)��ʱ�����������©��
//					DD_functions();	    //�㶯����ģʽ���ͻ����·�ָ����λ������ָ���Ӧ���ܲ���
					PointMoveflag=1;	//1�������㶯ģʽ
				}
			}
		}	
	}
}
////****RS232�������ݺ���****////
//RS232����len���ֽ�.
//buf:�������׵�ַ
//len:���͵��ֽ���
void RS232_Send_Data(u8 *buf,u8 len)
{
	u8 t;
  	for(t=0;t<len;t++)	//ѭ����������
	{
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);//�ȴ��ֽڷ������		  
		USART_SendData(USART1,buf[t]);//��������
		delay_ms(2);	   //����������ʱ�����������ݶ�ʧ
	}
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);	//�ȴ��ֽڷ������,TC������ɱ�־	 
	RS232_RX_CNT=0;	  //����
}

/****RS232�������ݺ���****/
//RS232��ѯ���յ�������
//buf:���ջ����׵�ַ
//len:���������ݳ���
void RS232_Receive_Data(u8 *buf,u8 *len)
{
	u8 rxlen=RS232_RX_CNT;
	u8 i=0;
	*len=0;							//Ĭ��Ϊ0
	delay_ms(10);
	if(rxlen==RS232_RX_CNT&&rxlen)	//���յ�������,�ҽ��������
	{
		for(i=0;i<rxlen;i++)
		{
			buf[i]=RS232_RX_BUF[i];
		}		
		*len=RS232_RX_CNT;			//��¼�������ݳ���
		RS232_RX_CNT=0;				//����
	}
}


//��������ת����4���ֽ������ݣ�4*8λ��ת��һ�����������ݣ�32λ��
//buf[]:����λ�����͹������ݻ�����
//Repara[]:��������λ����4�ֽ�����ת��Ϊһ������������
//len��Repara[]��������λ���鳤��
void bytetofloat(u8 *buf,float *Repara,u8 len)
{  
  u8 i=0,j=0,hexbyte[4],bufstart,bufend;
  switch(len)//�����ܽ��в���,���ݲ�ͬ�Ĺ��ܽ���ͷ��β��ֵ
  {
    //�㶯����ģʽ��2+4*11=46���ֽ�
 	case 11:
		bufstart=0xA5;	//��ͷ0xA5
		bufend=0x5A;	//��β0x5A		
	    break;
	//��ϴ�켣�滮����ģʽ��2+4*7=30���ֽ�
 	case 7:
		bufstart=0xB5;	//��ͷ0xB5
		bufend=0x5B;	//��β0x5B					
	    break;
	default:
		break;
  } 	
  //��ͷbufstart,��βbufend
  if(buf[0]==bufstart && buf[1+len*4]==bufend)
  {
    for(i=0;i<len;i++)
    {
      j=i*4;
	  hexbyte[0]=buf[j+1];
	  hexbyte[1]=buf[j+2];
	  hexbyte[2]=buf[j+3];
	  hexbyte[3]=buf[j+4];   
      Repara[i]=Hex_To_Decimal(hexbyte);    
    }
  }
}
//ʮ������ת��Ϊ������
float Hex_To_Decimal(u8 *Byte)
{
  return *((float*)Byte);
}
//��������ʮ������ת��
void FloatToByte(float floatNum,u8 *byteArry)
{
	u8 i;
    u8* pchar=(u8*)&floatNum;
    for(i=0;i<4;i++)
    {
		*byteArry=*pchar;
		pchar++;
		byteArry++;	
    }
}
/*******************************************************************************
 *��ϴ������λ��ͨѶ��غ���
*******************************************************************************/
//��ϴ�۸�����λ��ָ�����·���滮��Ӧ���� 
void CleanArmPathPlan_functions(void)
{
	float dx,dy,dz;			//��λ��������������֮��Ĳ�ֵ
	u8 i;
	CleanArm_FirstPostion(); //��ϴ��ĩ�˳�ʼλ�òɼ�����
	delay_ms(500);
  	for(i=0;i<CleanArmPoints-1;i++)	  //�������ݵ���ΪCleanArmPoints���м����CleanArmPoints-1���߶�
	{
	   //��λ��������������λ�����������໥ת����X��=-Y�ϣ�Y��=-X��,Z��=Z��
	   dy=-CleanArmbuf[3+i*3]+CleanArmbuf[0+i*3];	//����λ��X������(������CleanArmbuf[0+i*3])��ֵת������λ��Y�������ֵ
	   dx=-CleanArmbuf[4+i*3]+CleanArmbuf[1+i*3];	//����λ��Y������(������CleanArmbuf[0+i*3])��ֵת������λ��X������ֵ��ֵ
	   dz=CleanArmbuf[5+i*3]-CleanArmbuf[2+i*3];	//Z�������ֵ
//	    printf("i=%d\n",i);
//	   printf("dy=%f\n",dy);
//	   printf("dx=%f\n",dx);
//	   printf("dz=%f\n",dz);
	   CleanArm_PathMove(dx,dy,dz);	 				//��ϴ�۸�����λ���켣�滮ָ���ƶ�����
	   if(i==CleanArmPoints-2)//ִ�����������i��+1������i���������һ����Ӧ����CleanArmPoints-2
	   {
	   		CleanArmPathflag=0;			//ִ������λ���켣�滮����ϴ�ۿ�ʼ��ϴ��־λ��0
//			Angleflag=1;				//������ϴ��ĩ�˳�ʼλ�òɼ����´η�����ϴ�۹켣�滮����ʱ���²ɼ���ʼλ��

	   }
	}
}
void CleanArm_Postion(void) //��ϴ��ĩ�˵�ǰλ�òɼ�����
{
	CleanArm_a2=103.33*(3.14/180)-acos((129210.5-CleanArm_Len1*CleanArm_Len1)/105432.86);//�õ�����ֵ
	CleanArm_a3=-acos((184172.09-CleanArm_Len2*CleanArm_Len2)/151880)-1.92*(3.14/180);	 //�õ�����ֵ
	CleanArm_a4=(90-fabs(CleanArm_a2)-fabs(CleanArm_a3))*(3.14/180);					 //�õ�����ֵ
	CleanArm_X=cos(CleanArm_a1)*(300*cos(CleanArm_a2+CleanArm_a3)+600*cos(CleanArm_a2)); //��ʼ״̬X������ֵ
	CleanArm_Y=sin(CleanArm_a1)*(300*cos(CleanArm_a2+CleanArm_a3)+600*cos(CleanArm_a2)); //��ʼ״̬Y������ֵ
	CleanArm_Z=300*sin(CleanArm_a2+CleanArm_a3)+600*sin(CleanArm_a2);		 			 //��ʼ״̬Z������ֵ
//		printf("CleanArm_a2=%f\n",CleanArm_a2);
//		printf("CleanArm_a3=%f\n",CleanArm_a3);
//		printf("CleanArm_X=%f\n",CleanArm_X);
//		printf("CleanArm_Y=%f\n",CleanArm_Y);
//		printf("CleanArm_Z=%f\n",CleanArm_Z);
//		printf("CleanArm_Len1=%f\n",CleanArm_Len1);
//		printf("CleanArm_Len2=%f\n",CleanArm_Len2);
//		printf("CleanArm_Len3=%f\n",CleanArm_Len3);
}
//���黺�棺����λ�����͵Ķ�֡���ݽ��л���
// PointsNum,RunCycle��ʾ�������������������
// WalkAngle[4]�洢�˶��������ĸ������ת���Ƕ�
// WalkPoints[800]�洢ֱ���˶������еĸ�������
//len:һ֡���ݳ��˰�ͷ��β�м�������������
//���ܣ�����λ�����͵�ֱ���˶���������ݽ��б���
 void BufStorage(u8 len)
{ 
 float Leg1Angle,Leg2Angle,Leg3Angle,Leg4Angle;
 u8 bufr[64],ZJlen;
 float Reparar[7];				
 float Reparas[7];
 u8 bufs[30];
 int i;
// int j,m;
 static u16 fk;
 static u16 bk;
 static u16 ck;
 static u16 ltk;
 static u16 rtk;
 RS232_Receive_Data(bufr,&ZJlen);
 bytetofloat(bufr,Reparar,7);
 if(ZJlen)
 {  
  if(bufr[0]==0xB5 && bufr[1+len*4]==0x5B)
 {
 //ֱ�����߱�־λ
       	if((int)(Reparar[1])==16)
		{
		  StraightWalkFlag=1;
		}
	  if((int)(Reparar[0])==5 && (int)(Reparar[1])==0)   //��ϴ�۷��ͷֽ����ָ��
		  {
			  CleanArmPoints=(int)(Reparar[5]);		
//			  printf("Points=%d\n",CleanArmPoints);
			  ck=0;
		  }
		  if((int)(Reparar[0])==5 && (int)(Reparar[1])==1)  //��ϴ�۷�����������ָ��
		  {
			 if(ck<CleanArmPoints)
			 {
				 CleanArmbuf[0+ck*3]=Reparar[2];	  //��������Repara[2]�е�X������ֵ��ֵ��CleanArmbuf[]��
				 CleanArmbuf[1+ck*3]=Reparar[3];	  //��������Repara[3]�е�Y������ֵ��ֵ��CleanArmbuf[]��
				 CleanArmbuf[2+ck*3]=Reparar[4];	  //��������Repara[4]�е�Z������ֵ��ֵ��CleanArmbuf[]��
				 ck++;
//				 printf("K=%d\n",k);
				 if(ck==CleanArmPoints)
				 {
					CleanArmPathflag=1;
					Angleflag=1;				//������ϴ��ĩ�˳�ʼλ�òɼ����´η�����ϴ�۹켣�滮����ʱ���²ɼ���ʼλ��
//					for(n=0;n<CleanArmPoints;n++)
//					{
//					printf("n=%d\n",n);
//					printf("CleanArmbuf[]=%f\n",CleanArmbuf[n*3]);
//					}
				 }
			  }
			}


 //ǰ��ʱ����λ��ͨѶ����
 		if((int)(Reparar[1])==3)
		{
		   FRunCycle=(int)(Reparar[5]);
		   FPointsNum=(int)(Reparar[6]);
//		   printf("FRunCycle=%d,FPointsNum=%d\n",FRunCycle,FPointsNum);
		   bufs[0]=0xB5;  //�������ݰ�ͷ
           bufs[29]=0x5B; //�������ݰ�β
//         //����λ�����͵�ǰ�������ĸ�����ĽǶ�
        Leg1Angle=Encoder_Angle(3);   // �ɼ���1����Ƕ�
		Leg2Angle=Encoder_Angle(4);   // �ɼ���2����Ƕ�
		Leg3Angle=Encoder_Angle(5);   // �ɼ���3����Ƕ�
		Leg4Angle=Encoder_Angle(6);   // �ɼ���4����Ƕ�

//		Leg1Angle=25;   // �ɼ���1����Ƕ�
//		Leg2Angle=25;   // �ɼ���2����Ƕ�
//		Leg3Angle=25;   // �ɼ���3����Ƕ�
//		Leg4Angle=25;   // �ɼ���4����Ƕ�

        Reparas[0]=0;
		Reparas[1]=4;
		Reparas[2]=Leg1Angle;
		Reparas[3]=Leg2Angle;
		Reparas[4]=Leg3Angle;
		Reparas[5]=Leg4Angle;
		Reparas[6]=0;
	
	 for(i=0;i<7;i++)
	 {
	 FloatToByte(Reparas[i],&bufs[i*4+1]);
	 } 
	 delay_ms(200);
	 RS232_Send_Data(bufs,30);//���������ȵĳ�ʼ�Ƕ�
	  }
	if((int)(Reparar[1])==2)
	{
	    FWalkAngle[0]=Reparar[2];//��ȡ��1ת���Ƕ�ֵ
	    FWalkAngle[1]=Reparar[3];//��ȡ��2ת���Ƕ�ֵ
		FWalkAngle[2]=Reparar[4];//��ȡ��1ת���Ƕ�ֵ
		FWalkAngle[3]=Reparar[5];//��ȡ��1ת���Ƕ�ֵ 	
//		printf("FWalkAngle1=%f",FWalkAngle[0]);
//		printf("FWalkAngle2=%f",FWalkAngle[1]);
//		printf("FWalkAngle3=%f",FWalkAngle[2]);
//		printf("FWalkAngle4=%f",FWalkAngle[3]);
		fk=0;
	}
	if((int)(Reparar[1])==8)
	{
		 if(fk<FPointsNum)
		 {
		 	     FWalkPoints[0+fk*3]=Reparar[2];	  //��������Repara[2]�е�X������ֵ��ֵ��WalkPoints[]��
				 FWalkPoints[1+fk*3]=Reparar[3];	  //��������Repara[3]�е�Y������ֵ��ֵ��WalkPoints[]��
				 FWalkPoints[2+fk*3]=Reparar[4];	    //��������Repara[4]�е�Z������ֵ��ֵ��WalkPoints[]��
//				  printf("x=%f",FWalkPoints[0+fk*3]);
//				  printf("y=%f",FWalkPoints[1+fk*3]);
//				  printf("z=%f\n",FWalkPoints[2+fk*3]);
				 fk++;	
		 }
		 if(fk==FPointsNum)
		 {
		   ForwordWalkFlag=1;
		 }
	}
//����ʱ����λ��ͨѶ����
 		if((int)(Reparar[1])==6)
		{
		   BRunCycle=(int)(Reparar[5]);
		   BPointsNum=(int)(Reparar[6]);
//		   printf("BRunCycle=%d,BPointsNum=%d\n",BRunCycle,BPointsNum);
		   bufs[0]=0xB5;  //�������ݰ�ͷ
           bufs[29]=0x5B; //�������ݰ�β
         //����λ�����͵�ǰ�������ĸ�����ĽǶ�
        Leg1Angle=Encoder_Angle(3);   // �ɼ���1����Ƕ�
		Leg2Angle=Encoder_Angle(4);   // �ɼ���2����Ƕ�
		Leg3Angle=Encoder_Angle(5);   // �ɼ���3����Ƕ�
		Leg4Angle=Encoder_Angle(6);   // �ɼ���4����Ƕ�

//		Leg1Angle=25;   // �ɼ���1����Ƕ�
//		Leg2Angle=25;   // �ɼ���2����Ƕ�
//		Leg3Angle=25;   // �ɼ���3����Ƕ�
//		Leg4Angle=25;   // �ɼ���4����Ƕ�

        Reparas[0]=0;
		Reparas[1]=4;
		Reparas[2]=Leg1Angle;
		Reparas[3]=Leg2Angle;
		Reparas[4]=Leg3Angle;
		Reparas[5]=Leg4Angle;
		Reparas[6]=0;
	
	 for(i=0;i<7;i++)
	 {
	 FloatToByte(Reparas[i],&bufs[i*4+1]);
	 } 
	 delay_ms(200);
	 RS232_Send_Data(bufs,30);//���������ȵĳ�ʼ�Ƕ�
	  }
	if((int)(Reparar[1])==7)	   
	{
	    BWalkAngle[0]=Reparar[2];//��ȡ��1ת���Ƕ�ֵ
	    BWalkAngle[1]=Reparar[3];//��ȡ��2ת���Ƕ�ֵ
		BWalkAngle[2]=Reparar[4];//��ȡ��1ת���Ƕ�ֵ
		BWalkAngle[3]=Reparar[5];//��ȡ��1ת���Ƕ�ֵ
		bk=0;
	}
	if((int)(Reparar[1])==9)
	{
		 if(bk<BPointsNum)
		 {
		 	     BWalkPoints[0+bk*3]=Reparar[2];	  //��������Repara[2]�е�X������ֵ��ֵ��WalkPoints[]��
				 BWalkPoints[1+bk*3]=Reparar[3];	  //��������Repara[3]�е�Y������ֵ��ֵ��WalkPoints[]��
				 BWalkPoints[2+bk*3]=Reparar[4];	    //��������Repara[4]�е�Z������ֵ��ֵ��WalkPoints[]��
				 bk++;	
		 }
		 if(bk==BPointsNum)
		 {
		   BackwordWalkFlag=1;
		 }
	}
//�ҹ���ʱ����λ��ͨѶ����
   if((int)(Reparar[1])==10)
   {
     RTCompenLen=Reparar[3];
   	 RTPointsNum=(int)(Reparar[4]);
	 RTurnCycle=(int)(Reparar[5]);
	 RTRoundAngle=Reparar[6];
	       bufs[0]=0xB5;  //�������ݰ�ͷ
           bufs[29]=0x5B; //�������ݰ�β
         //����λ�����͵�ǰ�������ĸ�����ĽǶ�
        Leg1Angle=Encoder_Angle(3);   // �ɼ���1����Ƕ�
		Leg2Angle=Encoder_Angle(4);   // �ɼ���2����Ƕ�
		Leg3Angle=Encoder_Angle(5);   // �ɼ���3����Ƕ�
		Leg4Angle=Encoder_Angle(6);   // �ɼ���4����Ƕ�

//		Leg1Angle=25;   // �ɼ���1����Ƕ�
//		Leg2Angle=25;   // �ɼ���2����Ƕ�
//		Leg3Angle=25;   // �ɼ���3����Ƕ�
//		Leg4Angle=25;   // �ɼ���4����Ƕ�

        Reparas[0]=0;
		Reparas[1]=5;
		Reparas[2]=Leg1Angle;
		Reparas[3]=Leg2Angle;
		Reparas[4]=Leg3Angle;
		Reparas[5]=Leg4Angle;
		Reparas[6]=0;
	
	 for(i=0;i<7;i++)
	 {
	 FloatToByte(Reparas[i],&bufs[i*4+1]);
	 } 
	 delay_ms(200);
	 RS232_Send_Data(bufs,30);//���������ȵĳ�ʼ�Ƕ�
   }
   if((int)(Reparar[1])==11)
   {
   	    RTWalkAngle[0]=Reparar[2];//��ȡ��1ת���Ƕ�ֵ
	    RTWalkAngle[1]=Reparar[3];//��ȡ��2ת���Ƕ�ֵ
		RTWalkAngle[2]=Reparar[4];//��ȡ��1ת���Ƕ�ֵ
		RTWalkAngle[3]=Reparar[5];//��ȡ��1ת���Ƕ�ֵ
		rtk=0;
   }
   if((int)(Reparar[0])==3 && (int)(Reparar[1])==12)
   {
   		  if(rtk<RTPointsNum)
		 {
		 	     RTWalkPoints[0+rtk*3]=Reparar[2];	  //��������Repara[2]�е�X������ֵ��ֵ��WalkPoints[]��
				 RTWalkPoints[1+rtk*3]=Reparar[3];	  //��������Repara[3]�е�Y������ֵ��ֵ��WalkPoints[]��
				 RTWalkPoints[2+rtk*3]=Reparar[4];	    //��������Repara[4]�е�Z������ֵ��ֵ��WalkPoints[]��
				 rtk++;	
		 }
		 if(rtk==RTPointsNum)
		 {
		   RTurnMoveFlag=1;
		 }
   }
    //�����ʱ����λ��ͨѶ����
   if((int)(Reparar[1])==13)
   {
     LTCompenLen=Reparar[3];
   	 LTPointsNum=(int)(Reparar[4]);
	 LTurnCycle=(int)(Reparar[5]);
	 LTRoundAngle=Reparar[6];
	       bufs[0]=0xB5;  //�������ݰ�ͷ
           bufs[29]=0x5B; //�������ݰ�β
         //����λ�����͵�ǰ�������ĸ�����ĽǶ�
        Leg1Angle=Encoder_Angle(3);   // �ɼ���1����Ƕ�
		Leg2Angle=Encoder_Angle(4);   // �ɼ���2����Ƕ�
		Leg3Angle=Encoder_Angle(5);   // �ɼ���3����Ƕ�
		Leg4Angle=Encoder_Angle(6);   // �ɼ���4����Ƕ�

//		Leg1Angle=25;   // �ɼ���1����Ƕ�
//		Leg2Angle=25;   // �ɼ���2����Ƕ�
//		Leg3Angle=25;   // �ɼ���3����Ƕ�
//		Leg4Angle=25;   // �ɼ���4����Ƕ�

        Reparas[0]=0;
		Reparas[1]=5;
		Reparas[2]=Leg1Angle;
		Reparas[3]=Leg2Angle;
		Reparas[4]=Leg3Angle;
		Reparas[5]=Leg4Angle;
		Reparas[6]=0;
	
	 for(i=0;i<7;i++)
	 {
	 FloatToByte(Reparas[i],&bufs[i*4+1]);
	 } 
	 delay_ms(200);
	 RS232_Send_Data(bufs,30);//���������ȵĳ�ʼ�Ƕ�
   }
   if((int)(Reparar[1])==14)
   {
   	    LTWalkAngle[0]=Reparar[2];//��ȡ��1ת���Ƕ�ֵ
	    LTWalkAngle[1]=Reparar[3];//��ȡ��2ת���Ƕ�ֵ
		LTWalkAngle[2]=Reparar[4];//��ȡ��1ת���Ƕ�ֵ
		LTWalkAngle[3]=Reparar[5];//��ȡ��1ת���Ƕ�ֵ
		ltk=0;
   }
   if((int)(Reparar[0])==3 && (int)(Reparar[1])==15)
   {
   		  if(ltk<LTPointsNum)
		 {
		 	     LTWalkPoints[0+ltk*3]=Reparar[2];	  //��������Repara[2]�е�X������ֵ��ֵ��WalkPoints[]��
				 LTWalkPoints[1+ltk*3]=Reparar[3];	  //��������Repara[3]�е�Y������ֵ��ֵ��WalkPoints[]��
				 LTWalkPoints[2+ltk*3]=Reparar[4];	    //��������Repara[4]�е�Z������ֵ��ֵ��WalkPoints[]��
				 ltk++;	
		 }
		 if(ltk==LTPointsNum)
		 {
		   LTurnMoveFlag=1;
		 }
   }
	 }
	 
 }
//   RS232_RX_CNT=0;				//��232���ڽ��յ���������������
//	for(j=0;j<len*4+2;j++)
//	{
//	    bufr[j]=0x00;				//��buf�������㣬����ÿ��ִ�ж���֮ǰ�洢������
//	}
//	for(m=0;m<len;m++)
//	{
//	    Reparar[m]=0;				//��Reparar�������㣬����ÿ��ִ�ж���֮ǰ�洢������
//	}
}
//���ͺ�������λ���������˸��ؽڵ�״̬ʵʱ���͸���λ��
//state:1.2.3.4�����ȣ�5������ϴ�� 
void SendDataToPC(float state)
{
  u8 SendData[66];		//����һ֡����Ϊ66���ֽ�
  u8 i;
  float SendPara[16];	//����16�����������ݻ�����
  SendData[0]=0xA5;		//��ͷ
  SendData[65]=0x5A; 	//��β
  SendPara[0]=state;
  switch((int)state)			//�����ܽ��в���
  {
	    //"1"������1����
	    case 1:					
	    	break;
		//"2"������2����
	    case 2:					
	    	break;
		//"3"������3����
	    case 3:					
	    	break;
		//"4"������4����
	    case 4:					
	    	break;
		//"5"������ϴ������
	    case 5:
			SendPara[1]=-CleanArm_Y;	//��ϴ��X������
			SendPara[2]=CleanArm_X;	 	//��ϴ��Y������
			SendPara[3]=CleanArm_Z;	 	//��ϴ��Z������
			SendPara[4]=CleanArm_a1*(180/3.14);	//��ϴ�۵�������Ƕ�ֵ
			SendPara[5]=CleanArm_Len1;	//��ϴ�۵��1����
			SendPara[6]=CleanArm_Len2;	//��ϴ�۵��2����
			SendPara[7]=CleanArm_Len3;	//��ϴ�۵��3����
			SendPara[8]=0.0;
			SendPara[9]=CleanArm_a4*(180/3.14);	//��ϴ�̵���Ƕ�ֵ
			SendPara[10]=CleanArm_a2*(180/3.14);	//��ϴ�۴�۽Ƕ�ֵ
			SendPara[11]=CleanArm_a3*(180/3.14);	//��ϴ��С�۽Ƕ�ֵ
			SendPara[12]=0.0;
			SendPara[13]=0.0;
			SendPara[14]=0.0;
			SendPara[15]=0.0; //������ֵ

	    	break;
		 default:
			break;
  } 	
 	for(i=0;i<16;i++)
	{
	 FloatToByte(SendPara[i],&SendData[i*4+1]);	//��16������������ת�����ֽ������ݴ�����SendData[]��
	}
	 RS232_Send_Data(SendData,66);//�������ݵ���λ��
}
/*******************************************************************************
 *�㶯ģʽ�������ֱ�ߺ͹���������غ���
*******************************************************************************/
////�㶯����ģʽ���ͻ����·�ָ����λ������ָ���Ӧ���ܲ��� 
void DD_functions(void)
{
  u8 buf[64],ZJlen;
  float Repara[11];
  float len=0.5,Angle=1.0; //����һ�ε����/������Ϊ2.0mm�����ת���Ƕ�Ϊ1.0��
  RS232_Receive_Data(buf,&ZJlen);
  bytetofloat(buf,Repara,11);
//
//  if(ZJlen)							//���232���ڽ��յ����ݣ�����ж�Ӧ���ܲ�������������λ����Ҫ�������²��Ϸ���ͨѶָ��
//  {
	   //��ͷbufstart,��βbufend
	if(buf[0]==0xA5 && buf[45]==0x5A)
	{
	  switch((int)(Repara[0]))//�����ܽ��в���
	  {
	    //"1"�����ͻ��˵�������ģʽ
	    case 1:					
	    	break;
	    //"2"�ͻ��˿��ƻ����˽��е���㶯
	    case 2:
			switch((int)(Repara[2]))//����/��ϴ��ѡ��
	        {
	          //"1"�Ի�������1���в���
	          case 1:
				 switch((int)(Repara[1]))//�����ܽ��в���
			     {
		          //"0"����̧��
		          case 0:
					PointMoveflag=0;	//0���رյ㶯ģʽ
		          	break;
		          //"1"���3�Ƕ���ʱ��ת����+��
		          case 1:
				  	ServoMotor_TurnAngle(Angle,0,3);	           
				  	break;
				  //"2"���3�Ƕ�˳ʱ��ת����-��
		          case 2:
				  	ServoMotor_TurnAngle(Angle,1,3);	           
				  	break;
				  //"3"���1ǰ����+��
		          case 3:
				  	ElectricPutter_DDMove(len,1,1);	           
				  	break;
				  //"4"���1������-��
		          case 4:
				  	ElectricPutter_DDMove(len,0,1);	           
				  	break;
				  //"5"���2ǰ����+��
		          case 5:
				  	ElectricPutter_DDMove(len,1,2);	           
				  	break;
				  //"6"���2������-��
		          case 6:
				  	ElectricPutter_DDMove(len,0,2);	           
				  	break;
				  //"7"���3ǰ����+��
		          case 7:
				  	ElectricPutter_DDMove(len,1,3);	           
				  	break;
				  //"8"���3������-��
		          case 8:
				  	ElectricPutter_DDMove(len,0,3);	           
				  	break;
				  //���̣�����
				  case 9:
				  	break;
				  //���̣��ϣ�
				  case 10:
				  	break;
				  default:
			  		break;
				 } 		
	          	break;
	          //"2"�Ի�������2���в���
	          case 2:
			     switch((int)(Repara[1]))//�����ܽ��в���
			     {
		          //"0"����̧��
		          case 0:
				  	PointMoveflag=0;	//0���رյ㶯ģʽ
		          	break;
		          //"1"���4�Ƕ���ʱ��ת����+��
		          case 1:
				  	ServoMotor_TurnAngle(Angle,0,4);	           
				  	break;
				  //"2"���4�Ƕ�˳ʱ��ת����-��
		          case 2:
				  	ServoMotor_TurnAngle(Angle,1,4);	           
				  	break;
				  //"3"���10ǰ����+��
		          case 3:
				  	ElectricPutter_DDMove(len,1,10);	           
				  	break;
				  //"4"���10������-��
		          case 4:
				  	ElectricPutter_DDMove(len,0,10);	           
				  	break;
				  //"5"���11ǰ����+��
		          case 5:
				  	ElectricPutter_DDMove(len,1,11);	           
				  	break;
				  //"6"���11������-��
		          case 6:
				  	ElectricPutter_DDMove(len,0,11);	           
				  	break;
				  //"7"���12ǰ����+��
		          case 7:
				  	ElectricPutter_DDMove(len,1,12);	           
				  	break;
				  //"8"���12������-��
		          case 8:
				  	ElectricPutter_DDMove(len,0,12);	           
				  	break;
				  //���̣�����
				  case 9:
				  	break;
				  //���̣��ϣ�
				  case 10:
				  	break;
				  default:
			  		break;
				 } 			           
			  	break;
			  //"3"�Ի�������3���в���
	          case 3:
			     switch((int)(Repara[1]))//�����ܽ��в���
			     {
		          //"0"����̧��
		          case 0:
				  	PointMoveflag=0;	//0���رյ㶯ģʽ
		          	break;
		          //"1"���5�Ƕ���ʱ��ת����+��
		          case 1:
				  	ServoMotor_TurnAngle(Angle,0,5);	           
				  	break;
				  //"2"���5�Ƕ�˳ʱ��ת����-��
		          case 2:
				  	ServoMotor_TurnAngle(Angle,1,5);	           
				  	break;
				  //"3"���7ǰ����+��
		          case 3:
				  	ElectricPutter_DDMove(len,1,7);	           
				  	break;
				  //"4"���7������-��
		          case 4:
				  	ElectricPutter_DDMove(len,0,7);	           
				  	break;
				  //"5"���8ǰ����+��
		          case 5:
				  	ElectricPutter_DDMove(len,1,8);	           
				  	break;
				  //"6"���8������-��
		          case 6:
				  	ElectricPutter_DDMove(len,0,8);	           
				  	break;
				  //"7"���9ǰ����+��
		          case 7:
				  	ElectricPutter_DDMove(len,1,9);	           
				  	break;
				  //"8"���9������-��
		          case 8:
				  	ElectricPutter_DDMove(len,0,9);	           
				  	break;
				  //���̣�����
				  case 9:
				  	break;
				  //���̣��ϣ�
				  case 10:
				  	break;
				  default:
			  		break;
				 } 		
	          	break;
	          //"4"�Ի�������4���в���
	          case 4:
			     switch((int)(Repara[1]))//�����ܽ��в���
			     {
		          //"0"����̧��
		          case 0:
				  	PointMoveflag=0;	//0���رյ㶯ģʽ
		          	break;
		          //"1"���6�Ƕ���ʱ��ת����+��
		          case 1:
				  	ServoMotor_TurnAngle(Angle,0,6);	           
				  	break;
				  //"2"���6�Ƕ�˳ʱ��ת����-��
		          case 2:
				  	ServoMotor_TurnAngle(Angle,1,6);	           
				  	break;
				  //"3"���4ǰ����+��
		          case 3:
				  	ElectricPutter_DDMove(len,1,4);	           
				  	break;
				  //"4"���4������-��
		          case 4:
				  	ElectricPutter_DDMove(len,0,4);	           
				  	break;
				  //"5"���5ǰ����+��
		          case 5:
				  	ElectricPutter_DDMove(len,1,5);	           
				  	break;
				  //"6"���5������-��
		          case 6:
				  	ElectricPutter_DDMove(len,0,5);	           
				  	break;
				  //"7"���6ǰ����+��
		          case 7:
				  	ElectricPutter_DDMove(len,1,6);	           
				  	break;
				  //"8"���6������-��
		          case 8:
				  	ElectricPutter_DDMove(len,0,6);	           
				  	break;
				  //���̣�����
				  case 9:
				  	break;
				  //���̣��ϣ�
				  case 10:
				  	break;
				  default:
			  		break;
				 } 			           
			  	break;
			  //"5"�Ի�������ϴ�۽��в���
	          case 5:
			    switch((int)(Repara[1]))//�����ܽ��в���
			    {
		          //"0"����̧��
		          case 0:
				  	delay_ms(200);
				  	PointMoveflag=0;	//0���رյ㶯ģʽ
					SendDataToPC(5);	//������ϴ�۵�ǰ״̬
					delay_ms(20);
					SendDataToPC(5);	//������ϴ�۵�ǰ״̬
					delay_ms(20);
					SendDataToPC(5);	//������ϴ�۵�ǰ״̬
		          	break;
		          //"1"���7�Ƕ���ʱ��ת����+��
		          case 1:
				  	ServoMotor_TurnAngle(Angle,0,7);
					CleanArm_a1=CleanArm_a1+Angle*(3.14/180); 	//��ϴ�۵������ת����ĽǶ�ֵ,���Ƕ�ת��Ϊ����ֵ
//						printf("CleanArm_a1=%f\n",CleanArm_a1*(180/3.14));	           
				  	break;
				  //"2"���7�Ƕ�˳ʱ��ת����-��
		          case 2:
				  	ServoMotor_TurnAngle(Angle,1,7);
					CleanArm_a1=CleanArm_a1-Angle*(3.14/180); 	//��ϴ�۵������ת����ĽǶ�ֵ,���Ƕ�ת��Ϊ����ֵ	           
				  	break;
				  //"3"���13ǰ����+��
		          case 3:
				  	ElectricPutter_DDMove(len,1,13);
					CleanArm_Len1=CleanArm_Len1+len;			//�綯����ϴ�۵��1�ĳ���
//					printf("CleanArm_Len1=%f\n",CleanArm_Len1);	           
				  	break;
				  //"4"���13������-��
		          case 4:
				  	ElectricPutter_DDMove(len,0,13);
					CleanArm_Len1=CleanArm_Len1-len;			//�綯����ϴ�۵��1�ĳ���	           
				  	break;
				  //"5"���14ǰ����+��
		          case 5:
				  	ElectricPutter_DDMove(len,1,14);
					CleanArm_Len2=CleanArm_Len2+len;			//�綯����ϴ�۵��2�ĳ���	           
				  	break;
				  //"6"���14������-��
		          case 6:
				  	ElectricPutter_DDMove(len,0,14);
					CleanArm_Len2=CleanArm_Len2-len;			//�綯����ϴ�۵��2�ĳ���	           
				  	break;
				  //"7"���15ǰ����+��
		          case 7:
				  	ElectricPutter_DDMove(len,1,15);
					CleanArm_Len3=CleanArm_Len3+len;			//�綯����ϴ�۵��3�ĳ���	           
				  	break;
				  //"8"���15������-��
		          case 8:
				  	ElectricPutter_DDMove(len,0,15);
					CleanArm_Len3=CleanArm_Len3-len;			//�綯����ϴ�۵��3�ĳ���	           
				  	break;
				  //ëˢ�ŷ������ʱ��ת�������1��
				  case 11:
				    ServoMotor_TurnAngle(Angle,0,1); 
				  	break;
				  //ëˢ�ŷ����˳ʱ��ת�������1��
				  case 12:
				    ServoMotor_TurnAngle(Angle,1,1);
				  	break;
				  //��ϴ���ŷ������ʱ��ת�������2��
				  case 13:
				    ServoMotor_TurnAngle(Angle,0,1);
					CleanArm_a4=CleanArm_a4+Angle*(3.14/180); 	//��ϴ����ϴ�̵��ת����ĽǶ�ֵ,���Ƕ�ת��Ϊ����ֵ 
				  	break;
				  //��ϴ���ŷ����˳ʱ��ת�������2��
				  case 14:
				    ServoMotor_TurnAngle(Angle,1,1);
					CleanArm_a4=CleanArm_a4-Angle*(3.14/180); 	//��ϴ����ϴ�̵��ת����ĽǶ�ֵ,���Ƕ�ת��Ϊ����ֵ
				  	break;
				  default:
				  	delay_ms(200);
				  	PointMoveflag=0;	//0���رյ㶯ģʽ
					SendDataToPC(5);	//������ϴ�۵�ǰ״̬
					delay_ms(20);
					SendDataToPC(5);	//������ϴ�۵�ǰ״̬
					delay_ms(20);
					SendDataToPC(5);	//������ϴ�۵�ǰ״̬
			  		break;
				 } 			           
			  	break;
			  default:
			  	PointMoveflag=0;	//0���رյ㶯ģʽ
		  		break;
			} 			
		}
	}
	CleanArm_Postion();	   //ÿִ��һ�ε㶯ģʽ����һ�η���������λ���ĸ�������
//	delay_ms(400);
//	SendDataToPC(5);
//}
}
//ֱ�����߳���
//m=1ʱǰ����m=0ʱ����
void StraightWalk_function(int m)
{
int i;
float y1,y2,n;
if(m==1)
{
//ǰ����ʼ״̬
   ElectricPutter_Move(20,0,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(10,0,3);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,0,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(10,0,6);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,0,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(10,1,4);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,0,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(10,1,5);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);       
for(i=0;i<FRunCycle;i++)
   {
   ElectricPutter_Move(20,0,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(FWalkAngle[3],1,6);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   y1=RHLegOriginal(2);
   //3 
   ElectricPutter_Move(20,0,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(FWalkAngle[2],0,5);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   y2=RHLegOriginal(2);
   n=(y2-y1)/10;
   //4
   ElectricPutter_Move(20,0,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(FWalkAngle[0],1,3);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   
   //5
   ElectricPutter_Move(20,0,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(FWalkAngle[1],0,4);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500); 
   
   //6
  BodyLegMove1(-(y2-y1),n);
  delay_ms(1500);
  delay_ms(1500);
  }
   ElectricPutter_Move(20,0,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(6);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500); 
   ElectricPutter_Move(20,0,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(5);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,0,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(3);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,0,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(4);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);  
  }
if(m==0)
{
//���˳�ʼ״̬
   ElectricPutter_Move(20,0,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(10,0,3);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,0,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(10,0,6);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,0,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(10,1,4);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,0,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(10,1,5);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);       
   for(i=0;i<BRunCycle;i++)
   {
   BodyLegMove1(76,7.6);
   delay_ms(1500);
   delay_ms(1500);
   //3
   ElectricPutter_Move(20,0,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(BWalkAngle[0],0,3);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   //4
   ElectricPutter_Move(20,0,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(BWalkAngle[3],0,6);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   //5
   ElectricPutter_Move(20,0,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(BWalkAngle[1],1,4);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   
   //6
   ElectricPutter_Move(20,0,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(BWalkAngle[2],1,5);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500); 
   }
   ElectricPutter_Move(20,0,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(6);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500); 
   ElectricPutter_Move(20,0,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(5);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,0,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(3);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,0,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(4);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);  
}
  StraightWalkFlag=0;
  BackwordWalkFlag=0;
  ForwordWalkFlag=0;
}
//�������߳���
//n=1ʱ�ҹ��䣬n=0ʱ�����
void TurnWalk_function(int n)
{
  float x0,y0,z0,dx,dy,dz;
  float x,y,z,m7,m8,djrx;
  int i,j;
  //����� 
  if(n==0)
  {
  //������ʼ״̬
   ElectricPutter_Move(20,0,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(LTRoundAngle,0,6);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);

   ElectricPutter_Move(20,0,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(LTRoundAngle,0,5);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);

   ElectricPutter_Move(20,0,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(LTRoundAngle,0,4);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);

   ElectricPutter_Move(20,0,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(LTRoundAngle,0,3);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
  for(j=0;j<LTurnCycle+3;j++)
  {  
  x0=RHLegOriginal(1);
  y0=RHLegOriginal(2);
  z0=RHLegOriginal(3);
  x=x0,y=y0,z=z0;
  for(i=0;i<LTPointsNum-1;i++)
  {
   	dx=-LTWalkPoints[3+i*3]+LTWalkPoints[0+i*3];
	dy=-LTWalkPoints[4+i*3]+LTWalkPoints[1+i*3];
	dz=-LTWalkPoints[5+i*3]+LTWalkPoints[2+i*3];
//    dx=-fx[i];
//	dy=-fy[i];
	x+=dx;
	y+=dy;
	z+=dz;
	djrx=RHLegLocation(x,y,z,1)-RHLegLocation(x-dx,y-dy,z-dz,1);
	m7=RHLegLocation(x,y,z,2)-RHLegLocation(x-dx,y-dy,z-dz,2);	
	m8=RHLegLocation(x,y,z,3)-RHLegLocation(x-dx,y-dy,z-dz,3);
	 if(djrx>0)
	{
     BodyDJMove_SameE(djrx);
	}
	else
	{
	 BodyDJMove_SameF(-djrx);
	}
	if(m7>0)
	{
     ElectricPutter_MoveSame147A2(m7,1);
	}
	else
	{
	ElectricPutter_MoveSame147A2(-m7,0);
	}	

	if(m8>0)
	{
	ElectricPutter_MoveSame258B2(m8,1);
	}
	else
	{
	 ElectricPutter_MoveSame258B2(-m8,0);					    
	}   
}
  if(j<LTurnCycle+2)
  {
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
	//1	
	if(j==0)
	{																														
   ElectricPutter_Move(20,0,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0xe7,0xff);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(40,0,5);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(6);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(LTWalkAngle[3],0,6);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,5);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   }
   else
   {
   	 ElectricPutter_Move(20,0,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0xe7,0xff);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(6);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(LTWalkAngle[3],0,6);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,5);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   }
   //2
   ElectricPutter_Move(20,0,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0x3f,0xff);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(5);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(LTWalkAngle[2],0,5);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,8);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   //3
   ElectricPutter_Move(20,0,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0xff,0xf9);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(LTWalkAngle[1],0,4);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,11);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   //4
   ElectricPutter_Move(20,0,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0xfc,0xff);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(3);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(LTWalkAngle[0],0,3);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,2);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   }
   else
   {
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);  
   ElectricPutter_Move(20,0,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500); 
   CAT9555_WriteByte(SETUP,0xe7,0xff);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(6);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,5);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   
   //2 
   ElectricPutter_Move(20,0,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0x3f,0xff);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(5);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,8);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
  
   //3
   ElectricPutter_Move(20,0,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0xff,0xf9);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
  ElectricPutter_Move(30,1,11);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   //4
   ElectricPutter_Move(20,0,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0xfc,0xff);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(6);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,2);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   }
}
}
//�ҹ���
if(n==1)
  {
//  RTurnCycle=2;
  //�ҹ����ʼ״̬
   ElectricPutter_Move(20,0,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(RTRoundAngle,1,6);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);

   ElectricPutter_Move(20,0,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(RTRoundAngle,1,5);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);

   ElectricPutter_Move(20,0,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(RTRoundAngle,1,4);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);

   ElectricPutter_Move(20,0,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(RTRoundAngle,1,3);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,1,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
  for(j=0;j<RTurnCycle+3;j++)
  {  
  x0=RHLegOriginal(1);
  y0=RHLegOriginal(2);
  z0=RHLegOriginal(3);
  x=x0,y=y0,z=z0;
  for(i=0;i<RTPointsNum-1;i++)
  {
   	dx=-RTWalkPoints[3+i*3]+RTWalkPoints[0+i*3];
	dy=-RTWalkPoints[4+i*3]+RTWalkPoints[1+i*3];
	dz=-RTWalkPoints[5+i*3]+RTWalkPoints[2+i*3];
//    dx=-fx[i];
//	dy=-fy[i];
	x+=dx;
	y+=dy;
	z+=dz;
	djrx=RHLegLocation(x,y,z,1)-RHLegLocation(x-dx,y-dy,z-dz,1);
	m7=RHLegLocation(x,y,z,2)-RHLegLocation(x-dx,y-dy,z-dz,2);	
	m8=RHLegLocation(x,y,z,3)-RHLegLocation(x-dx,y-dy,z-dz,3);
	 if(djrx>0)
	{
     BodyDJMove_SameE(djrx);
	}
	else
	{
	 BodyDJMove_SameF(-djrx);
	}
	if(m7>0)
	{
     ElectricPutter_MoveSame147A2(m7,1);
	}
	else
	{
	ElectricPutter_MoveSame147A2(-m7,0);
	}	

	if(m8>0)
	{
	ElectricPutter_MoveSame258B2(m8,1);
	}
	else
	{
	 ElectricPutter_MoveSame258B2(-m8,0);					    
	}   
}
   if(j<RTurnCycle+2)
   {
	//1		
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   if(j==0)
   {																													
   ElectricPutter_Move(20,0,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0xe7,0xff);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,0,5);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(6);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(RTWalkAngle[3],1,6);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,5);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   }
   else
   {
   ElectricPutter_Move(20,0,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0xe7,0xff);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(6);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(RTWalkAngle[3],1,6);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,5);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   }
   //2
   ElectricPutter_Move(20,0,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0x3f,0xff);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(5);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(RTWalkAngle[2],1,5);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,8);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   //3
   ElectricPutter_Move(20,0,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0xff,0xf9);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(RTWalkAngle[1],1,4);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,11);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   //4
   ElectricPutter_Move(20,0,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0xfc,0xff);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(3);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_TurnAngle(RTWalkAngle[0],1,3);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,2);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   }
   else
   {
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(20,0,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0xe7,0xff);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500); 
   ServoMotor_Reset(6);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,5);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
  
   //2 
   ElectricPutter_Move(20,0,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0x3f,0xff);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(5);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,8);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,7);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   
   //3
   ElectricPutter_Move(20,0,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0xff,0xf9);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(4);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,11);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,10);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   //4
   ElectricPutter_Move(20,0,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   CAT9555_WriteByte(SETUP,0xfc,0xff);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ServoMotor_Reset(6);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,2);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   ElectricPutter_Move(30,1,1);
   delay_ms(1500);
   delay_ms(1500);
   delay_ms(1500);
   }
}
}
RTurnMoveFlag=0;
LTurnMoveFlag=0;
}
