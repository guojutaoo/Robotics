#include "public.h"
#include "systick.h"
#include "printf.h"
#include "ServoMotor.h"
#include "ElectricPutter.h"
#include "SCA1000.h"
#include "math.h"
#include "CleanArm.h"
#include "BodyMove.h"
#include "usart_232.h"
#include "MNspi.h"
//��λ���ۺϿ��Ƴ���
//int main()
//{
//   	ElectricPutter_Init();	 			//��˳�ʼ��				
//	ServoMotor_Init();
// 	usart_232_init();
//	MNspi_init();
//	SCA1000_Init();
//	CAT9555_WriteByte(SVON,0x00,0x00); 	//���е���ŷ�svon
//	delay_ms(1500);
//	delay_ms(1500);
//	delay_ms(1500);
//	//�ڳɳ�ʼ״̬
//   CAT9555_WriteByte(SETUP,0xff,0xdf);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
////   ElectricPutter_DDMove(15,0,13);
////   delay_ms(1500);
////   delay_ms(1500);
////   delay_ms(1500);
////   ElectricPutter_Move(70,1,14);			   
////   delay_ms(1500);
////   delay_ms(1500);
////   delay_ms(1500);
//   CAT9555_WriteByte(SETUP,0xb6,0xfd); //���1,4,7,10��ԭ�㸴��	
//   delay_ms(1500);
//   delay_ms(1500);
//   CAT9555_WriteByte(SETUP,0x6d,0xfb);//���2,5,8,11��ԭ�㸴��
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500); 
//   ServoMotor_Reset(3);
//   ServoMotor_Reset(4);
//   ServoMotor_Reset(5);
//   ServoMotor_Reset(6);
////   ServoMotor_Reset(7);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500); 
//   ElectricPutter_MoveSame258B1(30,1); 
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_MoveSame147A1(30,1);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//    CleanArm_FirstPostion(); //��ϴ��ĩ�˳�ʼλ�òɼ�����
//	delay_ms(1500);
//	SendDataToPC(5);
//	delay_ms(50);
//	SendDataToPC(5);
//	delay_ms(50);
//	SendDataToPC(5);
//	delay_ms(50);
//   while(1)
//   {
////  if(StraightWalkFlag==1)
////	{
////     BodyDJMove_SameA(10);
////     delay_ms(1500);
////     delay_ms(1500);
////     delay_ms(1500);
////	 StraightWalkFlag=0;
////	}
//   	if(ForwordWalkFlag==1)
//	 {
//	   StraightWalk_function(1);
//	 }
//    if(BackwordWalkFlag==1)
//	 {
//	   StraightWalk_function(0);
//	 }
//   if(LTurnMoveFlag==1)
//   {	 
//   	 TurnWalk_function(0);
//   }
//   if(RTurnMoveFlag==1)
//   {	 
//   	 TurnWalk_function(1);
//   }
//   if(PointMoveflag==1)//�㶯����ģʽ��ʼ��־��1�������㶯ģʽ
//	   {
//		 DD_functions();	    //�㶯����ģʽ���ͻ����·�ָ����λ������ָ���Ӧ���ܲ���
//	   }
//   if(CleanArmPathflag==1)
//		{
//			CleanArmPathPlan_functions();   //��ϴ�۸�����λ��ָ�����·���滮��Ӧ����
//			delay_ms(400);
//			SendDataToPC(5);   //������ϴ�۵�ǰ״̬
//			delay_ms(20);
//			SendDataToPC(5);	//������ϴ�۵�ǰ״̬
//			delay_ms(20);
//			SendDataToPC(5);	//������ϴ�۵�ǰ״̬
//		}	
//   }
//}
//��������������
//int main()
//{
//	ElectricPutter_Init();	 			//��˳�ʼ��				
//	ServoMotor_Init();
// 	usart_232_init();
//	MNspi_init();
//	SCA1000_Init();	
//	CAT9555_WriteByte(SVON,0x00,0x00); 	//���е���ŷ�svon
//	delay_ms(1500);
//	delay_ms(1500);
//	delay_ms(1500);
//   CAT9555_WriteByte(SETUP,0xb6,0xfd); //���1,4,7,10��ԭ�㸴��	
//   delay_ms(1500);
//   delay_ms(1500);
//   CAT9555_WriteByte(SETUP,0x6d,0xfb);//���2,5,8,11��ԭ�㸴��
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500); 
//   ServoMotor_Reset(3);
//   ServoMotor_Reset(4);
//   ServoMotor_Reset(5);
//   ServoMotor_Reset(6);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500); 
//   //1
//   ElectricPutter_MoveSame258B1(30,1); 
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_MoveSame147A1(30,1);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500); 
//   BodyLegMove3(-50.0,10.0);
//}
//�����������ߵ���������
//int main()
//{
// 	ElectricPutter_Init();	 			//��˳�ʼ��				
//	ServoMotor_Init();
// 	usart_232_init();
//	MNspi_init();
//	SCA1000_Init();
//    printf_init();
//	CAT9555_WriteByte(SVON,0x00,0x00); 	//���е���ŷ�svon
//	delay_ms(1500);
//	delay_ms(1500);
//	delay_ms(1500);
//    CAT9555_WriteByte(SETUP,0x3f,0xff);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ServoMotor_Reset(5);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_Move(30,1,8);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_Move(30,1,7);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   RHLegMove3(-50,10);
//}
//���Թ���������
//int main()
//{
//	float y1,y2,n;
//	ElectricPutter_Init();	 			//��˳�ʼ��				
//	CAT9555_WriteByte(SVON,0x00,0x00); 	//���е���ŷ�svon
//	ServoMotor_Init();
// 	usart_232_init();
//	MNspi_init();
//	SCA1000_Init();
//	delay_ms(1500);
//	delay_ms(1500);
//	delay_ms(1500);
//	CAT9555_WriteByte(SETUP,0xff,0xdf);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_DDMove(15,0,13);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_Move(70,1,14);			   
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   CAT9555_WriteByte(SETUP,0xb6,0xfd); //���1,4,7,10��ԭ�㸴��	
//   delay_ms(1500);
//   delay_ms(1500);
//   CAT9555_WriteByte(SETUP,0x6d,0xfb);//���2,5,8,11��ԭ�㸴��
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500); 
//   ServoMotor_Reset(3);
//   ServoMotor_Reset(4);
//   ServoMotor_Reset(5);
//   ServoMotor_Reset(6);
//   ServoMotor_Reset(7);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500); 
//   //1
//   BodyDJMove_SameE(5);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_MoveSame258B(30,1); 
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_MoveSame147A(30,1);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   BodyDJMove_SameF(5);
//   while(1)
//   {
//   	  	if(PointMoveflag==1)//�㶯����ģʽ��ʼ��־��1�������㶯ģʽ
//	   {
//		 DD_functions();	    //�㶯����ģʽ���ͻ����·�ָ����λ������ָ���Ӧ���ܲ���
//	   }
//   }
//}
//���Ի�������������
//int main()
//{
//   //	float y1,y2,n;
//	ElectricPutter_Init();	 			//��˳�ʼ��				
//	CAT9555_WriteByte(SVON,0x00,0x00); 	//���е���ŷ�svon
//	ServoMotor_Init();
// 	usart_232_init();
//	MNspi_init();
//	SCA1000_Init();
//	delay_ms(1500);
//	delay_ms(1500);
//	delay_ms(1500);
//	CAT9555_WriteByte(SETUP,0xff,0xdf);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_DDMove(15,0,13);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_Move(70,1,14);			   
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   CAT9555_WriteByte(SETUP,0xb6,0xfd); //���1,4,7,10��ԭ�㸴��	
//   delay_ms(1500);
//   delay_ms(1500);
//   CAT9555_WriteByte(SETUP,0x6d,0xfb);//���2,5,8,11��ԭ�㸴��
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500); 
//   ServoMotor_Reset(3);
//   ServoMotor_Reset(4);
//   ServoMotor_Reset(5);
//   ServoMotor_Reset(6);
//   ServoMotor_Reset(7);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500); 
//   //1
//   BodyDJMove_SameE(10);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_MoveSame258B1(30,1); 
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_MoveSame147A1(30,1);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   while(1)
//   {
//	
//   if(PointMoveflag==1)//�㶯����ģʽ��ʼ��־��1�������㶯ģʽ
//	   {
//		 DD_functions();	    //�㶯����ģʽ���ͻ����·�ָ����λ������ָ���Ӧ���ܲ���
//	   }
//   }
//}
//ֱ�����ߺ���ϴ����ϴ�켣������
//int main()
//{
//	float y1,y2,n;
//	ElectricPutter_Init();	 			//��˳�ʼ��				
//	CAT9555_WriteByte(SVON,0x00,0x00); 	//���е���ŷ�svon
//	ServoMotor_Init();
// 	usart_232_init();
//	MNspi_init();
//	SCA1000_Init();
//	delay_ms(1500);
//	delay_ms(1500);
//	delay_ms(1500);
//	CAT9555_WriteByte(SETUP,0xff,0xdf);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_Move(15,0,13);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_Move(40,1,14);			   
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   CAT9555_WriteByte(SETUP,0xb6,0xfd); //���1,4,7,10��ԭ�㸴��	
//   delay_ms(1500);
//   delay_ms(1500);
//   CAT9555_WriteByte(SETUP,0x6d,0xfb);//���2,5,8,11��ԭ�㸴��
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500); 
//   ServoMotor_Reset(3);
//   ServoMotor_Reset(4);
//   ServoMotor_Reset(5);
//   ServoMotor_Reset(6);
//   ServoMotor_Reset(7);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500); 
//   //1
//   BodyDJMove_SameA(10);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_MoveSame258B1(30,1); 
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_MoveSame147A1(30,1);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);  
//	CleanArm_FirstPostion(); //��ϴ��ĩ�˳�ʼλ�òɼ�����
//	delay_ms(1500);
//	SendDataToPC(5);
//	delay_ms(50);
//	SendDataToPC(5);
//	delay_ms(50);
//	SendDataToPC(5);
//	delay_ms(50);
//	while(1)
//	{
//	   if(ForwordWalkFlag==1)
//	   {
//	   	 StraightWalk_function(1);
//	   }
//	   if(BackwordWalkFlag==1)
//	   {
//	   	 StraightWalk_function(0);
//	   }
//	   if(PointMoveflag==1)//�㶯����ģʽ��ʼ��־��1�������㶯ģʽ
//	   {
//		 DD_functions();	    //�㶯����ģʽ���ͻ����·�ָ����λ������ָ���Ӧ���ܲ���
//	   }
//	   if(CleanArmPathflag==1)
//		{
//			CleanArmPathPlan_functions();   //��ϴ�۸�����λ��ָ�����·���滮��Ӧ����
//			delay_ms(400);
//			SendDataToPC(5);   //������ϴ�۵�ǰ״̬
//			delay_ms(20);
//			SendDataToPC(5);	//������ϴ�۵�ǰ״̬
//			delay_ms(20);
//			SendDataToPC(5);	//������ϴ�۵�ǰ״̬
//		}	
//	}
//
//}
//����ͨѶ������
int main()
{
   ServoMotor_Init();
   ServoMotor_TurnAngle(90,0,4);
   delay_ms(1500);
   ServoMotor_Reset(4);
   delay_ms(1500);
   ServoMotor_TurnAngle(90,1,4);
}
//����ֱ��������
//int main()
//{
//   float x0,y0,z0,l7,l8,theta0;
//   ElectricPutter_Init();
//   ServoMotor_Init();
//   printf_init();
//   MNspi_init();
//   CAT9555_WriteByte(SVON,0x00,0x00);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500); 
//   CAT9555_WriteByte(SETUP,0x3f,0xff);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ServoMotor_Reset(5);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_Move(30,1,8);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_Move(30,1,7);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ServoMotor_TurnAngle(10,0,5);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   RHLegMovetiaoshi(-3.03,-1.7,-0.4);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//  RHLegMovetiaoshi(-3.03,-1.7,-0.4);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//  RHLegMovetiaoshi(-3.03,-1.7,-0.4);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
// RHLegMovetiaoshi(-3.03,-1.7,-0.4);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
// RHLegMovetiaoshi(-3.03,-1.7,-0.4);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//RHLegMovetiaoshi(-3.03,1.7,0.4);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//RHLegMovetiaoshi(-3.03,1.7,0.4);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
// RHLegMovetiaoshi(-3.03,1.7,0.4);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
// RHLegMovetiaoshi(-3.03,1.7,0.4);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
// RHLegMovetiaoshi(-3.03,0.7,0.4);
//   theta0=Encoder_Angle(5);																															
//   x0=RHLegOriginal(1);
//   y0=RHLegOriginal(2);
//   z0=RHLegOriginal(3);
//   l7=RHLegOriginal(4);
//   l8=RHLegOriginal(5);
//   printf("x0=%f,y0=%f,z0=%f\n",x0,y0,z0); 
////   printf("theta=%f,l7=%f,l8=%f\n",theta0,l7,l8); 
//}
//��������ǰֱ������������
//int main()
//{
//   float y1,y2,n;
//   ElectricPutter_Init();
//   ServoMotor_Init();
//   MNspi_init();
//   usart3_init();
//   printf_init();
//   CAT9555_WriteByte(SVON,0x00,0x00);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   CAT9555_WriteByte(SETUP,0xff,0xdf);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_Move(15,0,13);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_Move(70,1,14);			   
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   CAT9555_WriteByte(SETUP,0xb6,0xfd); //���1,4,7,10��ԭ�㸴��	
//   delay_ms(1500);
//   delay_ms(1500);
//   CAT9555_WriteByte(SETUP,0x6d,0xfb);//���2,5,8,11��ԭ�㸴��
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500); 
//   ServoMotor_Reset(3);
//   ServoMotor_Reset(4);
//   ServoMotor_Reset(5);
//   ServoMotor_Reset(6);
//   ServoMotor_Reset(7);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500); 
//   //1
//   BodyDJMove_SameA(10);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_MoveSame258B(30,1); 
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_MoveSame147A(30,1);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//      //2
//   ElectricPutter_Move(20,0,4);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ServoMotor_TurnAngle(20,1,6);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_Move(20,1,4);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//////   y1=RHLegOriginal(2);
//////   printf("y1=%f",y1);
//   //3 
//   ElectricPutter_Move(20,0,7);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ServoMotor_TurnAngle(20,0,5);
////   n=(y2-y1)/10;																				 
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_Move(20,1,7);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   y2=RHLegOriginal(2);
   //   //4
//   ElectricPutter_Move(20,0,1);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ServoMotor_TurnAngle(20,1,3);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_Move(20,1,1);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   
//   //5
//   ElectricPutter_Move(20,0,10);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ServoMotor_TurnAngle(20,0,4);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_Move(20,1,10);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500); 
//   
//   //6
//  BodyLegMove1(-(y2-y1),n);
//  delay_ms(1500);
//  delay_ms(1500);
////   while(1)
////   {
////   StraightWalk_function();
////   }
//}
//���������ֱ���˶�������
//int main()
//{
//   float y1,y2,n;
//   ElectricPutter_Init();
//   ServoMotor_Init();
//   MNspi_init();
//   usart3_init();
//   CAT9555_WriteByte(SVON,0x00,0x00);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   CAT9555_WriteByte(SETUP,0xff,0xdf);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_Move(15,0,13);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_Move(70,1,14);			   
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   CAT9555_WriteByte(SETUP,0xb6,0xfd); //���1,4,7,10��ԭ�㸴��	
//   delay_ms(1500);
//   delay_ms(1500);
//   CAT9555_WriteByte(SETUP,0x6d,0xfb);//���2,5,8,11��ԭ�㸴��
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500); 
//   ServoMotor_Reset(3);
//   ServoMotor_Reset(4);
//   ServoMotor_Reset(5);
//   ServoMotor_Reset(6);
//   ServoMotor_Reset(7);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500); 
//   //1
//   BodyDJMove_SameA(10);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_MoveSame258B(30,1); 
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_MoveSame147A(30,1);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);  
//   while(1)
//   {
//	 if(PointMoveflag==1)//�㶯����ģʽ��ʼ��־��1�������㶯ģʽ
//	   {
//		 DD_functions();	    //�㶯����ģʽ���ͻ����·�ָ����λ������ָ���Ӧ���ܲ���
//	   }
//
//   StraightWalk_function();
//   }
//}
//   //2
//   BodyLegMove1(76,7.6);
//   delay_ms(1500);
//   delay_ms(1500);
//   //3
//   ElectricPutter_Move(20,0,1);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ServoMotor_TurnAngle(20,0,3);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_Move(20,1,1);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   //4
//   ElectricPutter_Move(20,0,4);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ServoMotor_TurnAngle(20,0,6);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_Move(20,1,4);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   //5
//   ElectricPutter_Move(20,0,10);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ServoMotor_TurnAngle(20,1,4);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_Move(20,1,10);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   
//   //6
//   ElectricPutter_Move(20,0,7);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ServoMotor_TurnAngle(20,1,5);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_Move(20,1,7);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500); 



    //2
//   for(i=0;i<2;i++)
//   {
//   ElectricPutter_Move(20,0,4);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ServoMotor_TurnAngle(20,1,6);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_Move(20,1,4);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   y1=RHLegOriginal(2);
//   //3 
//   ElectricPutter_Move(20,0,7);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ServoMotor_TurnAngle(20,0,5);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_Move(20,1,7);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   y2=RHLegOriginal(2);
//   n=(y2-y1)/10;
//   //4
//   ElectricPutter_Move(20,0,1);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ServoMotor_TurnAngle(20,1,3);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_Move(20,1,1);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   
//   //5
//   ElectricPutter_Move(20,0,10);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ServoMotor_TurnAngle(20,0,4);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_Move(20,1,10);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500); 
//   
//   //6
//  BodyLegMove1(-(y2-y1),n);
//  delay_ms(1500);
//  delay_ms(1500);
//  }



//    //4
//   y1=RHLegOriginal(2);

//   y2=RHLegOriginal(2);
//   n=(y2-y1)/10;
//
//     //6
//   BodyLegMove(-(y2-y1),n);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//
//   //7
//   ElectricPutter_Move(20,0,1);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ServoMotor_TurnAngle(30,1,3);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_Move(20,1,1);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//
//    //9
//   ElectricPutter_Move(20,0,10);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ServoMotor_TurnAngle(30,0,4);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_Move(20,1,10);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   
 

//  ServoMotor_Reset(5);
//   delay_ms(1500);
//   delay_ms(1500);
//   ServoMotor_TurnAngle(80,0,5);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//  ElectricPutter_Move(30,1,7);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_Move(30,1,8);
//   x0=RHLegOriginal(1);
//   y0=RHLegOriginal(2);
//   z0=RHLegOriginal(3);
//   theta10=Encoder_Angle(5);
//   printf("x0=%f,y0=%f,z0=%f,theta10=%f\n",x0,y0,z0,theta10);
//   n=266/10;
//   BodyLegMove(-266,n);
//

//  




//   CAT9555_WriteByte(SETUP,0x00,0x00);

//  ServoMotor_Reset(3);
//  ServoMotor_Reset(4);
//  ServoMotor_Reset(5);
//  ServoMotor_Reset(6);
//  BodyDJMove_SameB(55);


//   delay_ms(1500);
//   delay_ms(1500);

//  ElectricPutter_MoveSame369C(10,1);
//   CAT9555_WriteByte(SVON,0x00,0x00);
//   delay_ms(1500);
//   ServoMotor_Reset(5);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_Move(30,1,7);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_Move(30,1,8);
//   x0=RHLegOriginal(1);
//   y0=RHLegOriginal(2);
//   z0=RHLegOriginal(3);
//   theta10=Encoder_Angle(5);
//   printf("x0=%f,y0=%f,z0=%f,theta10=%f\n",x0,y0,z0,theta10);
//   n=283/10;
//   RHLegMove(-283,n);

//   //1
//   ServoMotor_Reset(3);
//   ServoMotor_Reset(4);
//   ServoMotor_Reset(5);
//   ServoMotor_Reset(6);
//   BodyDJMove_SameB(45);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_MoveSame147A(30,1);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_MoveSame258B(30,1);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
  
//
//

//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_Move(10,0,8); 
//   x0=RHLegOriginal(1);
//   y0=RHLegOriginal(2);
//   z0=RHLegOriginal(3);
//   l1=RHLegOriginal(4);
//   l2=RHLegOriginal(5);
//   theta10=Encoder_Angle(5);
//   printf("x0=%f,y0=%f,z0=%f,theta10=%f\n",x0,y0,z0,theta10);
 
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_Move(30,1,7);
//   delay_ms(1500);
//   delay_ms(1500);
//   delay_ms(1500);
//   ElectricPutter_Move(30,1,8);
//   ServoMotor_Reset(5);
//   delay_ms(1500);
//   ServoMotor_TurnAngle(45,1,5);
//   y1=RHLegOriginal(2);
//   delay_ms(1500);
//   delay_ms(1500);
//   ServoMotor_TurnAngle(80,0,5);
//   y2=RHLegOriginal(2);
//   n=(y2-y1)/10;
//   RHLegMove(-(y2-y1),n);
////	 ServoMotor_TurnAngle(10,1,5);
////	 delay_ms(1500);
////	 delay_ms(1500);
////	 delay_ms(1500);
////     RHLegMove(100);
//   theta1o=Encoder_Angle(5);
//   printf("x0=%f,y0=%f,z0=%f,l1=%f,l2=%f\n",x0,y0,z0,l1,l2);
//   printf("theta1o=%f",theta1o);
//    RHLegMove(-100);
//	ServoMotor_Reset(5);
//	
//	ServoMotor_TurnAngle(45,1,5);
//	delay_ms(1500);
//	x0=RHLegOriginal(1);
// 	y0=RHLegOriginal(2);
//	z0=RHLegOriginal(3); 
//    printf("x0=%f,y0=%f,z0=%f\n",x0,y0,z0);
//	delay_ms(1500);
//	ServoMotor_TurnAngle(80,0,5);
//	x0=RHLegOriginal(1);
// 	y0=RHLegOriginal(2);
//	z0=RHLegOriginal(3);
//	theta10=Encoder_Angle(5);
//	printf("x0=%f,y0=%f,z0=%f,theta10=%f\n",x0,y0,z0,theta10);
//  RHLegMove(-250);


//inl1=RHLegOriginal(4);t main()
//{	l2=RHLegOriginal(5);
//	ElectricPutter_Init();	 			//��˳�ʼ��				
//	CAT9555_WriteByte(SVON,0x00,0x00); 	//���е���ŷ�svon
//	ServoMotor_Init();  				//�ŷ������ʼ��
//	delay_ms(1500);
//	delay_ms(1500);
//	delay_ms(1500);
//	//1
//	ElectricPutter_Move(20,0,13);	  //3�ŵ����ǰ�ƶ�20mm
//	delay_ms(1500);
//	delay_ms(1500);
//	ElectricPutter_Move(10,0,15);	  //3�ŵ����ǰ�ƶ�20mm
//	delay_ms(1500);
//	delay_ms(1500);
//	//2
//	ServoMotor_TurnAngle(18,1,7);//˳ʱ��
//
//	ElectricPutter_Move(10,1,14);	  //3�ŵ����ǰ�ƶ�20mm
//	delay_ms(1500);
//	delay_ms(1500);
//	ElectricPutter_Move(5,1,15);	  //3�ŵ����ǰ�ƶ�20mm
//	delay_ms(1500);
////	delay_ms(1500);
//	//3
//	ServoMotor_TurnAngle(18,1,7);//˳ʱ��
//	ElectricPutter_Move(10,0,14);	  //3�ŵ����ǰ�ƶ�20mm
//	delay_ms(1500);
//	delay_ms(1500);
//	ElectricPutter_Move(5,0,15);	  //3�ŵ����ǰ�ƶ�20mm
//	delay_ms(1500);
//	delay_ms(1500);		
//	ServoMotor_TurnAngle(36,0,7);//˳ʱ��
//	ElectricPutter_Move(10,1,15);	  //3�ŵ����ǰ�ƶ�20mm
//	delay_ms(1500);
//	delay_ms(1500);
//	ElectricPutter_Move(20,1,13);	  //3�ŵ����ǰ�ƶ�20mm
//}
//////֫��滮
//int main()
//{
//	u8 i=0;
//	ElectricPutter_Init();	  	//��˳�ʼ��
//	CAT9555_WriteByte(SVON,0x00,0x00); //��3.5������ŷ�svon��0��������1��������
//	ServoMotor_Init();  		//�ŷ������ʼ��
//	delay_ms(1500);
//	delay_ms(1500);				//����485�������ݸ��Ų��ȴ��ŷ��������
//	delay_ms(1500);
////	ServoMotor_Reset(3);		//1�ŵ��ԭ�㸴��
////	delay_ms(1500);
////	delay_ms(1500);
////	ServoMotor_Reset(4);		//1�ŵ��ԭ�㸴��
////	delay_ms(1500);
////	delay_ms(1500);
////	ServoMotor_Reset(5);		//1�ŵ��ԭ�㸴��
////	delay_ms(1500);
////	delay_ms(1500);
////	ServoMotor_Reset(6);		//1�ŵ��ԭ�㸴��
////	delay_ms(1500);
////	delay_ms(1500);
////	BodyDJMove_SameA1(45);
////	delay_ms(1500);
////	delay_ms(1500);
//	//2.
//	for(i=0;i<3;i++)
//	{
//		ElectricPutter_Move(50,0,4);	  //3�ŵ����ǰ�ƶ�20mm
//		delay_ms(1500);
//	   	ServoMotor_TurnAngle(80,1,6);//˳ʱ��
//	   	delay_ms(1500);
//	   	ElectricPutter_Move(50,1,4);	  //3�ŵ����ǰ�ƶ�20mm
//		delay_ms(1500);
//		delay_ms(1500);
//		delay_ms(1500);
//		//4.
//		ElectricPutter_Move(50,0,7);	  //3�ŵ����ǰ�ƶ�20mm
//		delay_ms(1500);
//	   	ServoMotor_TurnAngle(80,0,5);//��ʱ��
//	   	delay_ms(1500);
//	   	ElectricPutter_Move(50,1,7);	  //3�ŵ����ǰ�ƶ�20mm
//		delay_ms(1500);
//		delay_ms(1500);
//		delay_ms(1500);
//		//6.
//		BodyDJMove_SameA(80);
//		delay_ms(1500);
//		delay_ms(1500);
////		delay_ms(1500);
//	//	delay_ms(1500);
//		//7.
//		ElectricPutter_Move(50,0,1);	  //3�ŵ����ǰ�ƶ�20mm
//		delay_ms(1500);
//		ServoMotor_TurnAngle(80,1,3);//˳ʱ��
//		delay_ms(1500);
//		ElectricPutter_Move(50,1,1);	  //3�ŵ����ǰ�ƶ�20mm
//		delay_ms(1500);
//		delay_ms(1500);
//		delay_ms(1500);
//		//9.
//	   	ElectricPutter_Move(50,0,10);	  //3�ŵ����ǰ�ƶ�20mm
//		delay_ms(1500);
//		ServoMotor_TurnAngle(80,0,4);//˳ʱ��
//		delay_ms(1500);
//		ElectricPutter_Move(50,1,10);	  //3�ŵ����ǰ�ƶ�20mm
//		delay_ms(1500);
//		delay_ms(1500);
//		delay_ms(1500);
//		//	//1
//		ElectricPutter_Move(20,0,13);	  //3�ŵ����ǰ�ƶ�20mm
//		delay_ms(1500);
////		delay_ms(1500);
//		ElectricPutter_Move(10,0,15);	  //3�ŵ����ǰ�ƶ�20mm
//		delay_ms(1500);
////		delay_ms(1500);
//		//2
//		ServoMotor_TurnAngle(18,1,7);//˳ʱ��
//	
//		ElectricPutter_Move(10,1,14);	  //3�ŵ����ǰ�ƶ�20mm
//		delay_ms(1500);
////		delay_ms(1500);
//		ElectricPutter_Move(5,1,15);	  //3�ŵ����ǰ�ƶ�20mm
//		delay_ms(1500);
//		//3
//		ServoMotor_TurnAngle(18,1,7);//˳ʱ��
//		ElectricPutter_Move(10,0,14);	  //3�ŵ����ǰ�ƶ�20mm
//		delay_ms(1500);
////		delay_ms(1500);
//		ElectricPutter_Move(5,0,15);	  //3�ŵ����ǰ�ƶ�20mm
//		delay_ms(1500);
////		delay_ms(1500);		
//		ServoMotor_TurnAngle(36,0,7);//˳ʱ��
//		ElectricPutter_Move(10,1,15);	  //3�ŵ����ǰ�ƶ�20mm
//		delay_ms(1500);
////		delay_ms(1500);
//		ElectricPutter_Move(20,1,13);	  //3�ŵ����ǰ�ƶ�20mm
//		delay_ms(1500);
//		delay_ms(1500);
//	}
//}
//float SameDJAngle=10;
//u32 i,SameDJsteps;
//printf_init();	//printf���ڳ�ʼ��
//	SameDJsteps=1000*SameDJAngle;
//	while(1)
//	{
//	printf("a1=   %d\n",SameDJsteps);
//	}
//	}
//int main()
//{
//	ElectricPutter_Init();	  	//��˳�ʼ��
//	CAT9555_WriteByte(SVON,0x00,0x00); //��3.5������ŷ�svon��0��������1��������
////	ServoMotor_Init();  		//�ŷ������ʼ��		
//	delay_ms(1500);				//����485�������ݸ��Ų��ȴ��ŷ��������
//	delay_ms(1500);
//	delay_ms(1500);
////	delay_ms(500);
//			//����485�������ݸ��Ų��ȴ��ŷ��������
////	ServoMotor_Reset(5);		//1�ŵ��ԭ�㸴��
////	delay_ms(1500);
////	ServoMotor_TurnAngle(10,0,2);		//���1��ʱ����ת45��
//	ElectricPutter_Move(10,0,1);	  //3�ŵ����ǰ�ƶ�20mm
//	delay_ms(1500);
//	delay_ms(1500);
//	ElectricPutter_Move(10,0,4);	  //3�ŵ����ǰ�ƶ�20mm
//	delay_ms(1500);
//	delay_ms(1500);
//	ElectricPutter_Move(10,0,7);	  //3�ŵ����ǰ�ƶ�20mm
//	delay_ms(1500);
//	delay_ms(1500);
//	ElectricPutter_Move(10,0,10);	  //3�ŵ����ǰ�ƶ�20mm
////	delay_ms(1500);
////		delay_ms(1500);
//////	  	delay_ms(1500);
////	ElectricPutter_Move(10,0,15);	  //3�ŵ����ǰ�ƶ�20mm	
////	delay_ms(1500);
////	delay_ms(1500);
////	delay_ms(1500);
////	delay_ms(1500);
////		delay_ms(1500);
////	ElectricPutter_Move(100,1,14);	  //3�ŵ����ǰ�ƶ�20mm
////		delay_ms(1500);
////	delay_ms(1500);
////	delay_ms(1500);
////	delay_ms(1500);
////	delay_ms(1500);
////	ElectricPutter_Move(100,0,14);	  //3�ŵ����ǰ�ƶ�20mm
////	delay_ms(1500);
////	delay_ms(1500);
////	delay_ms(1500);
////	delay_ms(1500);
////		delay_ms(1500);
////	ElectricPutter_Move(30,1,15);	  //5�ŵ����ǰ�ƶ�20mm
////	delay_ms(1500);
////	delay_ms(1500);
////	delay_ms(1500);
////	delay_ms(1500);
////	delay_ms(1500);
////	ElectricPutter_Move(30,0,15);	  //5�ŵ����ǰ�ƶ�20mm
//
//}
//int main()
//{ 
//	float a1,a2,a3,x,y,z,m,len1,len2,len3,dl1,dl2,dl3,da1;
//	printf_init();	//printf���ڳ�ʼ��
////	//��ȡ��ϴ���������ٶȴ������Ƕ�ֵ
////	a1=0; 	//��ȡ5�ŵ������ϴ�۵����������ʼ�Ƕ�,���Ƕ�ת��Ϊ����ֵ
////	a2=(7.8-7.8)*(3.14/180);	//��۳�ʼ�Ƕ�ֵ�����Ƕ�ת��Ϊ����ֵ
////	a3=(-45.96-a2-14)*(3.14/180);//С�۳�ʼ�Ƕ�ֵ�����Ƕ�ת��Ϊ����ֵ��fabs()�󸡵���������ֵ
//////	a4=realAX3;				//��ϴ�̳�ʼ�Ƕ�ֵ//��ʱ����Ҫ
////
////	x=cos(a1)*(300*cos(a2+a3)+600*cos(a2));		//��ʼ״̬X������ֵ
////	y=sin(a1)*(300*cos(a2+a3)+600*cos(a2)); 	//��ʼ״̬Y������ֵ
////	z=300*sin(a2+a3)+600*sin(a2);		 		//��ʼ״̬Z������ֵ
////	len1=sqrt(129210.5-105432.86*cos((125.13-20.13)*(3.14/180)-a2)); //��ʼ״̬��ϴ���ϵ��1��������������ܳ���
////	len2=sqrt(184172.09-151880*cos(-a3-1.92*(3.14/180))); 			//��ʼ״̬��ϴ���ϵ��2��������������ܳ���
////	len3=sqrt(98549.76-61440*cos(-a3-a2)); 
//	x=749.99;	  		//�ƶ���X������ֵ
//	y=0;	 		//�ƶ���Y������ֵ
//	z=-259.8;	 		//�ƶ���Z������ֵ
//	a1=atan(y/x);			   			//�ƶ��������Ӧ����ֵ
//	m=x*cos(a1)+y*sin(a1);
//	da1=(atan(y/x)-a1)*(180/3.14);  	//��������ƶ���仯�Ƕ�ֵda1
//	a3=-acos((z*z+m*m-450000)/360000);	//�ƶ���С�۶�Ӧ����ֵ,a3ʼ��Ϊ��
//	a2=atan((600*z-300*(sin(a3)*m-z*cos(a3)))/(600*m+300*(cos(a3)*m+z*sin(a3))))*(180/3.14);	//�ƶ����۶�Ӧ����ֵ
//	a3=a3*(180/3.14); 						//��ʼ״̬��ϴ���ϵ��3��������������ܳ��� 
//	while(1)
//	{
//	 printf("a1=   %f\n",a1);
//	 printf("a2=   %f\n",a2);
//	 printf("a3=   %f\n",a3);
//	}
//}
//u8 flag=1,flag1=1;
////��ȡ�Ƕȴ�����ֵ
//int main()
//{ 
//u8 i=5;
//	SCA1000_Init();						//sca1000 ��ʼ��
////	ServoMotor_Init();
//while(i--)
//{
//  delay_ms(1000);
//  CleanArm_Move(0,40,0);
//  }
//	float a1,a2,a3,x,y,z,m,len1,len2,len3,dl1,dl2,dl3,da1;
//	
//	printf_init();	//printf���ڳ�ʼ��
//	SCA1000_Init();
// 	SCA1000_ReadAXData();				//��ȡ��ϴ���������ٶȴ������Ƕ�ֵ
//	delay_ms(1000);
//	if(flag==1)
//	{
//		a1=0*(3.14/180); 	//��ȡ5�ŵ������ϴ�۵����������ʼ�Ƕ�,���Ƕ�ת��Ϊ����ֵ
//		a2=(7.8-fabs(realAX1))*(3.14/180);	//��۳�ʼ�Ƕ�ֵ�����Ƕ�ת��Ϊ����ֵ
//		a3=(-fabs(realAX3)-a2-14)*(3.14/180);//С�۳�ʼ�Ƕ�ֵ�����Ƕ�ת��Ϊ����ֵ��fabs()�󸡵���������ֵ
//	//	a4=realAX3;				//��ϴ�̳�ʼ�Ƕ�ֵ//��ʱ����Ҫ
//	
//		x=cos(a1)*(300*cos(a2+a3)+600*cos(a2));		//��ʼ״̬X������ֵ
//		y=sin(a1)*(300*cos(a2+a3)+600*cos(a2)); 	//��ʼ״̬Y������ֵ
//		z=300*sin(a2+a3)+600*sin(a2);		 		//��ʼ״̬Z������ֵ
//		len1=sqrt(129210.5-105432.86*cos(105*(3.14/180)-a2)); //��ʼ״̬��ϴ���ϵ��1��������������ܳ���
//		len2=sqrt(184172.09-151880*cos(-a3-1.92*(3.14/180))); 			//��ʼ״̬��ϴ���ϵ��2��������������ܳ���
//		len3=sqrt(98549.76-61440*cos(-a3-a2));  						//��ʼ״̬��ϴ���ϵ��3��������������ܳ���
//		flag=0;
//	}
//	if(flag1==1)
//	{
//	/****����X���ƶ�dx��Y���ƶ�dy��Z���ƶ�dz����ʱ��������ϴ�۸����ؽڶ�Ӧ�Ƕ�ֵ��Ӧ����ֵa1,a2,a3 ****/
//		x=x+0;	  		//�ƶ���X������ֵ
//		y=y+40;	 		//�ƶ���Y������ֵ
//		z=z+0;	 		//�ƶ���Z������ֵ
//		m=x*cos(a1)+y*sin(a1);
//		da1=(atan(y/x)-a1)*(180/3.14);  	//��������ƶ���仯�Ƕ�ֵda1
//		a1=atan(y/x);			   			//�ƶ��������Ӧ����ֵ
//		a3=-acos((z*z+m*m-450000)/360000);	//�ƶ���С�۶�Ӧ����ֵ,a3ʼ��Ϊ��
//		a2=atan((600*z-300*(sin(a3)*m-z*cos(a3)))/(600*m+300*(cos(a3)*m+z*sin(a3))));	//�ƶ����۶�Ӧ����ֵ
//	/****�ƶ�һ�ξ������ϴ������Ӧ�綯�Ƹ��ƶ���Ծ������****/	
//		dl1=sqrt(129210.5-105432.86*cos(105*(3.14/180)-a2))-len1;				//�ƶ������ϵ���ƶ�����
//		dl2=sqrt(184172.09-151880*cos(-a3-1.92*(3.14/180)))-len2;						//�ƶ���С���ϵ���ƶ�����
//		dl3=sqrt(98549.76-61440*cos(-a3-a2))-len3;
//	}				
//	while(1)
//	{ 
//		printf("da1=  %f\n",da1);
//		printf("dl1=  %f\n",dl1);
//		printf("dl2=  %f\n",dl2);
//		printf("dl3=  %f\n",dl3);
//		delay_ms(1000);	 
//	} 
//}

////�ŷ����������
//int main()
//{
//	ServoMotor_Init();  //�ŷ������ʼ��
//	delay_ms(1500);	
//	ServoMotor_Reset(1);//1�ŵ��ԭ�㸴��
////	ServoMotor_TurnAngle(20,1,1);		//���1˳ʱ����ת45��
////	Check_hhb();		//��ز�У��
////	Encoder_Value();  	//��ȡ������ֵ
//}
////�ŷ����������
//int main()
//{
//	u16 aa;
//	ElectricPutter_Init();	  //�˿ڳ�ʼ��
////	printf_init();	//printf���ڳ�ʼ��
//	CAT9555_WriteByte(SVON,0xEF,0xFF); //��5���ŷ�svon
//	delay_ms(1500);
//	delay_ms(1500);
//	ElectricPutter_Move(10,1,5);	  //3�ŵ����ǰ�ƶ�20mm
////	delay_ms(1500);
////	delay_ms(1500);
////	aa=CAT9555_ReadByte(INP);
////	printf("   %X\n",aa);
//	if(aa==0xFFEF)
//	{
//	  ElectricPutter_Move(20,1,5);	  //3�ŵ����ǰ�ƶ�20mm
//	}
//	if(aa==0xFFFF)
//	{
//	  ElectricPutter_Move(20,0,5);	  //3�ŵ����ǰ�ƶ�20mm
//	}
//}
//int main()
//{
//	ElectricPutter_Init();	  	//��˳�ʼ��
//	CAT9555_WriteByte(SVON,0xE3,0xFF); //��3.5������ŷ�svon��0��������1��������
//	ServoMotor_Init();  		//�ŷ������ʼ��		
//	delay_ms(1500);				//����485�������ݸ��Ų��ȴ��ŷ��������
//	ServoMotor_Reset(1);		//1�ŵ��ԭ�㸴��
//	ServoMotor_TurnAngle(45,0,1);		//���1��ʱ����ת45��
//	ElectricPutter_Move(30,1,5);	  //5�ŵ����ǰ�ƶ�20mm
//	ElectricPutter_Move(20,1,3);	  //3�ŵ����ǰ�ƶ�20mm
//	ServoMotor_TurnAngle(45,1,1);		//���1˳ʱ����ת45��
//	ElectricPutter_Move(20,0,3);	  //3�ŵ������ƶ�20mm
//	ServoMotor_TurnAngle(45,1,1);		//���1˳ʱ����ת45��
//	ElectricPutter_Move(20,1,3);	  //3�ŵ����ǰ�ƶ�20mm
//	ElectricPutter_Move(30,0,5);	  //5�ŵ������ƶ�30mm
//	ServoMotor_TurnAngle(45,0,1);		//���1��ʱ����ת45��
//	ElectricPutter_Move(20,0,3);	  //3�ŵ������ƶ�20mm
//}
