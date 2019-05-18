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
//上位机综合控制程序
//int main()
//{
//   	ElectricPutter_Init();	 			//电杆初始化				
//	ServoMotor_Init();
// 	usart_232_init();
//	MNspi_init();
//	SCA1000_Init();
//	CAT9555_WriteByte(SVON,0x00,0x00); 	//所有电杆伺服svon
//	delay_ms(1500);
//	delay_ms(1500);
//	delay_ms(1500);
//	//摆成初始状态
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
//   CAT9555_WriteByte(SETUP,0xb6,0xfd); //电杆1,4,7,10号原点复归	
//   delay_ms(1500);
//   delay_ms(1500);
//   CAT9555_WriteByte(SETUP,0x6d,0xfb);//电杆2,5,8,11号原点复归
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
//    CleanArm_FirstPostion(); //清洗臂末端初始位置采集函数
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
//   if(PointMoveflag==1)//点动控制模式开始标志，1：启动点动模式
//	   {
//		 DD_functions();	    //点动控制模式：客户端下发指令下位机接收指令对应功能操作
//	   }
//   if(CleanArmPathflag==1)
//		{
//			CleanArmPathPlan_functions();   //清洗臂根据上位机指令完成路径规划对应操作
//			delay_ms(400);
//			SendDataToPC(5);   //发送清洗臂当前状态
//			delay_ms(20);
//			SendDataToPC(5);	//发送清洗臂当前状态
//			delay_ms(20);
//			SendDataToPC(5);	//发送清洗臂当前状态
//		}	
//   }
//}
//测试左走主程序
//int main()
//{
//	ElectricPutter_Init();	 			//电杆初始化				
//	ServoMotor_Init();
// 	usart_232_init();
//	MNspi_init();
//	SCA1000_Init();	
//	CAT9555_WriteByte(SVON,0x00,0x00); 	//所有电杆伺服svon
//	delay_ms(1500);
//	delay_ms(1500);
//	delay_ms(1500);
//   CAT9555_WriteByte(SETUP,0xb6,0xfd); //电杆1,4,7,10号原点复归	
//   delay_ms(1500);
//   delay_ms(1500);
//   CAT9555_WriteByte(SETUP,0x6d,0xfb);//电杆2,5,8,11号原点复归
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
//测试左走右走单腿主程序
//int main()
//{
// 	ElectricPutter_Init();	 			//电杆初始化				
//	ServoMotor_Init();
// 	usart_232_init();
//	MNspi_init();
//	SCA1000_Init();
//    printf_init();
//	CAT9555_WriteByte(SVON,0x00,0x00); 	//所有电杆伺服svon
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
//测试拐弯主程序
//int main()
//{
//	float y1,y2,n;
//	ElectricPutter_Init();	 			//电杆初始化				
//	CAT9555_WriteByte(SVON,0x00,0x00); 	//所有电杆伺服svon
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
//   CAT9555_WriteByte(SETUP,0xb6,0xfd); //电杆1,4,7,10号原点复归	
//   delay_ms(1500);
//   delay_ms(1500);
//   CAT9555_WriteByte(SETUP,0x6d,0xfb);//电杆2,5,8,11号原点复归
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
//   	  	if(PointMoveflag==1)//点动控制模式开始标志，1：启动点动模式
//	   {
//		 DD_functions();	    //点动控制模式：客户端下发指令下位机接收指令对应功能操作
//	   }
//   }
//}
//测试弧线流畅主程序
//int main()
//{
//   //	float y1,y2,n;
//	ElectricPutter_Init();	 			//电杆初始化				
//	CAT9555_WriteByte(SVON,0x00,0x00); 	//所有电杆伺服svon
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
//   CAT9555_WriteByte(SETUP,0xb6,0xfd); //电杆1,4,7,10号原点复归	
//   delay_ms(1500);
//   delay_ms(1500);
//   CAT9555_WriteByte(SETUP,0x6d,0xfb);//电杆2,5,8,11号原点复归
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
//   if(PointMoveflag==1)//点动控制模式开始标志，1：启动点动模式
//	   {
//		 DD_functions();	    //点动控制模式：客户端下发指令下位机接收指令对应功能操作
//	   }
//   }
//}
//直线行走和清洗臂清洗轨迹主程序
//int main()
//{
//	float y1,y2,n;
//	ElectricPutter_Init();	 			//电杆初始化				
//	CAT9555_WriteByte(SVON,0x00,0x00); 	//所有电杆伺服svon
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
//   CAT9555_WriteByte(SETUP,0xb6,0xfd); //电杆1,4,7,10号原点复归	
//   delay_ms(1500);
//   delay_ms(1500);
//   CAT9555_WriteByte(SETUP,0x6d,0xfb);//电杆2,5,8,11号原点复归
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
//	CleanArm_FirstPostion(); //清洗臂末端初始位置采集函数
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
//	   if(PointMoveflag==1)//点动控制模式开始标志，1：启动点动模式
//	   {
//		 DD_functions();	    //点动控制模式：客户端下发指令下位机接收指令对应功能操作
//	   }
//	   if(CleanArmPathflag==1)
//		{
//			CleanArmPathPlan_functions();   //清洗臂根据上位机指令完成路径规划对应操作
//			delay_ms(400);
//			SendDataToPC(5);   //发送清洗臂当前状态
//			delay_ms(20);
//			SendDataToPC(5);	//发送清洗臂当前状态
//			delay_ms(20);
//			SendDataToPC(5);	//发送清洗臂当前状态
//		}	
//	}
//
//}
//测试通讯主程序
int main()
{
   ServoMotor_Init();
   ServoMotor_TurnAngle(90,0,4);
   delay_ms(1500);
   ServoMotor_Reset(4);
   delay_ms(1500);
   ServoMotor_TurnAngle(90,1,4);
}
//测试直线主程序
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
//机器人向前直线行走主程序
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
//   CAT9555_WriteByte(SETUP,0xb6,0xfd); //电杆1,4,7,10号原点复归	
//   delay_ms(1500);
//   delay_ms(1500);
//   CAT9555_WriteByte(SETUP,0x6d,0xfb);//电杆2,5,8,11号原点复归
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
//机器人向后直线运动主程序
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
//   CAT9555_WriteByte(SETUP,0xb6,0xfd); //电杆1,4,7,10号原点复归	
//   delay_ms(1500);
//   delay_ms(1500);
//   CAT9555_WriteByte(SETUP,0x6d,0xfb);//电杆2,5,8,11号原点复归
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
//	 if(PointMoveflag==1)//点动控制模式开始标志，1：启动点动模式
//	   {
//		 DD_functions();	    //点动控制模式：客户端下发指令下位机接收指令对应功能操作
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
//	ElectricPutter_Init();	 			//电杆初始化				
//	CAT9555_WriteByte(SVON,0x00,0x00); 	//所有电杆伺服svon
//	ServoMotor_Init();  				//伺服电机初始化
//	delay_ms(1500);
//	delay_ms(1500);
//	delay_ms(1500);
//	//1
//	ElectricPutter_Move(20,0,13);	  //3号电杆向前移动20mm
//	delay_ms(1500);
//	delay_ms(1500);
//	ElectricPutter_Move(10,0,15);	  //3号电杆向前移动20mm
//	delay_ms(1500);
//	delay_ms(1500);
//	//2
//	ServoMotor_TurnAngle(18,1,7);//顺时针
//
//	ElectricPutter_Move(10,1,14);	  //3号电杆向前移动20mm
//	delay_ms(1500);
//	delay_ms(1500);
//	ElectricPutter_Move(5,1,15);	  //3号电杆向前移动20mm
//	delay_ms(1500);
////	delay_ms(1500);
//	//3
//	ServoMotor_TurnAngle(18,1,7);//顺时针
//	ElectricPutter_Move(10,0,14);	  //3号电杆向前移动20mm
//	delay_ms(1500);
//	delay_ms(1500);
//	ElectricPutter_Move(5,0,15);	  //3号电杆向前移动20mm
//	delay_ms(1500);
//	delay_ms(1500);		
//	ServoMotor_TurnAngle(36,0,7);//顺时针
//	ElectricPutter_Move(10,1,15);	  //3号电杆向前移动20mm
//	delay_ms(1500);
//	delay_ms(1500);
//	ElectricPutter_Move(20,1,13);	  //3号电杆向前移动20mm
//}
//////肢体规划
//int main()
//{
//	u8 i=0;
//	ElectricPutter_Init();	  	//电杆初始化
//	CAT9555_WriteByte(SVON,0x00,0x00); //第3.5个电杆伺服svon，0：工作，1：不工作
//	ServoMotor_Init();  		//伺服电机初始化
//	delay_ms(1500);
//	delay_ms(1500);				//消除485发送数据干扰并等待伺服电机就绪
//	delay_ms(1500);
////	ServoMotor_Reset(3);		//1号电机原点复归
////	delay_ms(1500);
////	delay_ms(1500);
////	ServoMotor_Reset(4);		//1号电机原点复归
////	delay_ms(1500);
////	delay_ms(1500);
////	ServoMotor_Reset(5);		//1号电机原点复归
////	delay_ms(1500);
////	delay_ms(1500);
////	ServoMotor_Reset(6);		//1号电机原点复归
////	delay_ms(1500);
////	delay_ms(1500);
////	BodyDJMove_SameA1(45);
////	delay_ms(1500);
////	delay_ms(1500);
//	//2.
//	for(i=0;i<3;i++)
//	{
//		ElectricPutter_Move(50,0,4);	  //3号电杆向前移动20mm
//		delay_ms(1500);
//	   	ServoMotor_TurnAngle(80,1,6);//顺时针
//	   	delay_ms(1500);
//	   	ElectricPutter_Move(50,1,4);	  //3号电杆向前移动20mm
//		delay_ms(1500);
//		delay_ms(1500);
//		delay_ms(1500);
//		//4.
//		ElectricPutter_Move(50,0,7);	  //3号电杆向前移动20mm
//		delay_ms(1500);
//	   	ServoMotor_TurnAngle(80,0,5);//逆时针
//	   	delay_ms(1500);
//	   	ElectricPutter_Move(50,1,7);	  //3号电杆向前移动20mm
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
//		ElectricPutter_Move(50,0,1);	  //3号电杆向前移动20mm
//		delay_ms(1500);
//		ServoMotor_TurnAngle(80,1,3);//顺时针
//		delay_ms(1500);
//		ElectricPutter_Move(50,1,1);	  //3号电杆向前移动20mm
//		delay_ms(1500);
//		delay_ms(1500);
//		delay_ms(1500);
//		//9.
//	   	ElectricPutter_Move(50,0,10);	  //3号电杆向前移动20mm
//		delay_ms(1500);
//		ServoMotor_TurnAngle(80,0,4);//顺时针
//		delay_ms(1500);
//		ElectricPutter_Move(50,1,10);	  //3号电杆向前移动20mm
//		delay_ms(1500);
//		delay_ms(1500);
//		delay_ms(1500);
//		//	//1
//		ElectricPutter_Move(20,0,13);	  //3号电杆向前移动20mm
//		delay_ms(1500);
////		delay_ms(1500);
//		ElectricPutter_Move(10,0,15);	  //3号电杆向前移动20mm
//		delay_ms(1500);
////		delay_ms(1500);
//		//2
//		ServoMotor_TurnAngle(18,1,7);//顺时针
//	
//		ElectricPutter_Move(10,1,14);	  //3号电杆向前移动20mm
//		delay_ms(1500);
////		delay_ms(1500);
//		ElectricPutter_Move(5,1,15);	  //3号电杆向前移动20mm
//		delay_ms(1500);
//		//3
//		ServoMotor_TurnAngle(18,1,7);//顺时针
//		ElectricPutter_Move(10,0,14);	  //3号电杆向前移动20mm
//		delay_ms(1500);
////		delay_ms(1500);
//		ElectricPutter_Move(5,0,15);	  //3号电杆向前移动20mm
//		delay_ms(1500);
////		delay_ms(1500);		
//		ServoMotor_TurnAngle(36,0,7);//顺时针
//		ElectricPutter_Move(10,1,15);	  //3号电杆向前移动20mm
//		delay_ms(1500);
////		delay_ms(1500);
//		ElectricPutter_Move(20,1,13);	  //3号电杆向前移动20mm
//		delay_ms(1500);
//		delay_ms(1500);
//	}
//}
//float SameDJAngle=10;
//u32 i,SameDJsteps;
//printf_init();	//printf串口初始化
//	SameDJsteps=1000*SameDJAngle;
//	while(1)
//	{
//	printf("a1=   %d\n",SameDJsteps);
//	}
//	}
//int main()
//{
//	ElectricPutter_Init();	  	//电杆初始化
//	CAT9555_WriteByte(SVON,0x00,0x00); //第3.5个电杆伺服svon，0：工作，1：不工作
////	ServoMotor_Init();  		//伺服电机初始化		
//	delay_ms(1500);				//消除485发送数据干扰并等待伺服电机就绪
//	delay_ms(1500);
//	delay_ms(1500);
////	delay_ms(500);
//			//消除485发送数据干扰并等待伺服电机就绪
////	ServoMotor_Reset(5);		//1号电机原点复归
////	delay_ms(1500);
////	ServoMotor_TurnAngle(10,0,2);		//电机1逆时针旋转45度
//	ElectricPutter_Move(10,0,1);	  //3号电杆向前移动20mm
//	delay_ms(1500);
//	delay_ms(1500);
//	ElectricPutter_Move(10,0,4);	  //3号电杆向前移动20mm
//	delay_ms(1500);
//	delay_ms(1500);
//	ElectricPutter_Move(10,0,7);	  //3号电杆向前移动20mm
//	delay_ms(1500);
//	delay_ms(1500);
//	ElectricPutter_Move(10,0,10);	  //3号电杆向前移动20mm
////	delay_ms(1500);
////		delay_ms(1500);
//////	  	delay_ms(1500);
////	ElectricPutter_Move(10,0,15);	  //3号电杆向前移动20mm	
////	delay_ms(1500);
////	delay_ms(1500);
////	delay_ms(1500);
////	delay_ms(1500);
////		delay_ms(1500);
////	ElectricPutter_Move(100,1,14);	  //3号电杆向前移动20mm
////		delay_ms(1500);
////	delay_ms(1500);
////	delay_ms(1500);
////	delay_ms(1500);
////	delay_ms(1500);
////	ElectricPutter_Move(100,0,14);	  //3号电杆向前移动20mm
////	delay_ms(1500);
////	delay_ms(1500);
////	delay_ms(1500);
////	delay_ms(1500);
////		delay_ms(1500);
////	ElectricPutter_Move(30,1,15);	  //5号电杆向前移动20mm
////	delay_ms(1500);
////	delay_ms(1500);
////	delay_ms(1500);
////	delay_ms(1500);
////	delay_ms(1500);
////	ElectricPutter_Move(30,0,15);	  //5号电杆向前移动20mm
//
//}
//int main()
//{ 
//	float a1,a2,a3,x,y,z,m,len1,len2,len3,dl1,dl2,dl3,da1;
//	printf_init();	//printf串口初始化
////	//读取清洗臂三个加速度传感器角度值
////	a1=0; 	//读取5号电机（清洗臂底座电机）初始角度,将角度转换为弧度值
////	a2=(7.8-7.8)*(3.14/180);	//大臂初始角度值，将角度转换为弧度值
////	a3=(-45.96-a2-14)*(3.14/180);//小臂初始角度值，将角度转换为弧度值，fabs()求浮点型数绝对值
//////	a4=realAX3;				//清洗盘初始角度值//暂时不需要
////
////	x=cos(a1)*(300*cos(a2+a3)+600*cos(a2));		//初始状态X轴坐标值
////	y=sin(a1)*(300*cos(a2+a3)+600*cos(a2)); 	//初始状态Y轴坐标值
////	z=300*sin(a2+a3)+600*sin(a2);		 		//初始状态Z轴坐标值
////	len1=sqrt(129210.5-105432.86*cos((125.13-20.13)*(3.14/180)-a2)); //初始状态清洗臂上电杆1本体加伸缩长度总长度
////	len2=sqrt(184172.09-151880*cos(-a3-1.92*(3.14/180))); 			//初始状态清洗臂上电杆2本体加伸缩长度总长度
////	len3=sqrt(98549.76-61440*cos(-a3-a2)); 
//	x=749.99;	  		//移动后X轴坐标值
//	y=0;	 		//移动后Y轴坐标值
//	z=-259.8;	 		//移动后Z轴坐标值
//	a1=atan(y/x);			   			//移动后底座对应弧度值
//	m=x*cos(a1)+y*sin(a1);
//	da1=(atan(y/x)-a1)*(180/3.14);  	//底座电机移动后变化角度值da1
//	a3=-acos((z*z+m*m-450000)/360000);	//移动后小臂对应弧度值,a3始终为负
//	a2=atan((600*z-300*(sin(a3)*m-z*cos(a3)))/(600*m+300*(cos(a3)*m+z*sin(a3))))*(180/3.14);	//移动后大臂对应弧度值
//	a3=a3*(180/3.14); 						//初始状态清洗臂上电杆3本体加伸缩长度总长度 
//	while(1)
//	{
//	 printf("a1=   %f\n",a1);
//	 printf("a2=   %f\n",a2);
//	 printf("a3=   %f\n",a3);
//	}
//}
//u8 flag=1,flag1=1;
////读取角度传感器值
//int main()
//{ 
//u8 i=5;
//	SCA1000_Init();						//sca1000 初始化
////	ServoMotor_Init();
//while(i--)
//{
//  delay_ms(1000);
//  CleanArm_Move(0,40,0);
//  }
//	float a1,a2,a3,x,y,z,m,len1,len2,len3,dl1,dl2,dl3,da1;
//	
//	printf_init();	//printf串口初始化
//	SCA1000_Init();
// 	SCA1000_ReadAXData();				//读取清洗臂三个加速度传感器角度值
//	delay_ms(1000);
//	if(flag==1)
//	{
//		a1=0*(3.14/180); 	//读取5号电机（清洗臂底座电机）初始角度,将角度转换为弧度值
//		a2=(7.8-fabs(realAX1))*(3.14/180);	//大臂初始角度值，将角度转换为弧度值
//		a3=(-fabs(realAX3)-a2-14)*(3.14/180);//小臂初始角度值，将角度转换为弧度值，fabs()求浮点型数绝对值
//	//	a4=realAX3;				//清洗盘初始角度值//暂时不需要
//	
//		x=cos(a1)*(300*cos(a2+a3)+600*cos(a2));		//初始状态X轴坐标值
//		y=sin(a1)*(300*cos(a2+a3)+600*cos(a2)); 	//初始状态Y轴坐标值
//		z=300*sin(a2+a3)+600*sin(a2);		 		//初始状态Z轴坐标值
//		len1=sqrt(129210.5-105432.86*cos(105*(3.14/180)-a2)); //初始状态清洗臂上电杆1本体加伸缩长度总长度
//		len2=sqrt(184172.09-151880*cos(-a3-1.92*(3.14/180))); 			//初始状态清洗臂上电杆2本体加伸缩长度总长度
//		len3=sqrt(98549.76-61440*cos(-a3-a2));  						//初始状态清洗臂上电杆3本体加伸缩长度总长度
//		flag=0;
//	}
//	if(flag1==1)
//	{
//	/****沿着X轴移动dx，Y轴移动dy，Z轴移动dz距离时，计算清洗臂各个关节对应角度值对应弧度值a1,a2,a3 ****/
//		x=x+0;	  		//移动后X轴坐标值
//		y=y+40;	 		//移动后Y轴坐标值
//		z=z+0;	 		//移动后Z轴坐标值
//		m=x*cos(a1)+y*sin(a1);
//		da1=(atan(y/x)-a1)*(180/3.14);  	//底座电机移动后变化角度值da1
//		a1=atan(y/x);			   			//移动后底座对应弧度值
//		a3=-acos((z*z+m*m-450000)/360000);	//移动后小臂对应弧度值,a3始终为负
//		a2=atan((600*z-300*(sin(a3)*m-z*cos(a3)))/(600*m+300*(cos(a3)*m+z*sin(a3))));	//移动后大臂对应弧度值
//	/****移动一段距离后，清洗臂上相应电动推杆移动相对距离计算****/	
//		dl1=sqrt(129210.5-105432.86*cos(105*(3.14/180)-a2))-len1;				//移动后大臂上电杆移动长度
//		dl2=sqrt(184172.09-151880*cos(-a3-1.92*(3.14/180)))-len2;						//移动后小臂上电杆移动长度
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

////伺服电机主函数
//int main()
//{
//	ServoMotor_Init();  //伺服电机初始化
//	delay_ms(1500);	
//	ServoMotor_Reset(1);//1号电机原点复归
////	ServoMotor_TurnAngle(20,1,1);		//电机1顺时针旋转45度
////	Check_hhb();		//后回波校验
////	Encoder_Value();  	//读取编码器值
//}
////伺服电机主函数
//int main()
//{
//	u16 aa;
//	ElectricPutter_Init();	  //端口初始化
////	printf_init();	//printf串口初始化
//	CAT9555_WriteByte(SVON,0xEF,0xFF); //第5个伺服svon
//	delay_ms(1500);
//	delay_ms(1500);
//	ElectricPutter_Move(10,1,5);	  //3号电杆向前移动20mm
////	delay_ms(1500);
////	delay_ms(1500);
////	aa=CAT9555_ReadByte(INP);
////	printf("   %X\n",aa);
//	if(aa==0xFFEF)
//	{
//	  ElectricPutter_Move(20,1,5);	  //3号电杆向前移动20mm
//	}
//	if(aa==0xFFFF)
//	{
//	  ElectricPutter_Move(20,0,5);	  //3号电杆向前移动20mm
//	}
//}
//int main()
//{
//	ElectricPutter_Init();	  	//电杆初始化
//	CAT9555_WriteByte(SVON,0xE3,0xFF); //第3.5个电杆伺服svon，0：工作，1：不工作
//	ServoMotor_Init();  		//伺服电机初始化		
//	delay_ms(1500);				//消除485发送数据干扰并等待伺服电机就绪
//	ServoMotor_Reset(1);		//1号电机原点复归
//	ServoMotor_TurnAngle(45,0,1);		//电机1逆时针旋转45度
//	ElectricPutter_Move(30,1,5);	  //5号电杆向前移动20mm
//	ElectricPutter_Move(20,1,3);	  //3号电杆向前移动20mm
//	ServoMotor_TurnAngle(45,1,1);		//电机1顺时针旋转45度
//	ElectricPutter_Move(20,0,3);	  //3号电杆向后移动20mm
//	ServoMotor_TurnAngle(45,1,1);		//电机1顺时针旋转45度
//	ElectricPutter_Move(20,1,3);	  //3号电杆向前移动20mm
//	ElectricPutter_Move(30,0,5);	  //5号电杆向后移动30mm
//	ServoMotor_TurnAngle(45,0,1);		//电机1逆时针旋转45度
//	ElectricPutter_Move(20,0,3);	  //3号电杆向后移动20mm
//}
