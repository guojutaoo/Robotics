#include "CleanArm.h"
#include "SCA1000.h"
#include "ServoMotor.h"
#include "systick.h"
#include "math.h"
#include "ElectricPutter.h"
#include "printf.h"
u8 Angleflag=1;	  //清洗臂末端初始位置采集标志位，1：采集开始；0：采集结束
//底座、大臂、小臂对应初始角度对应弧度值：CleanArm_a1,CleanArm_a2,CleanArm_a3；
//清洗盘初始坐标值CleanArm_X,CleanArm_Y,CleanArm_Z,
//清洗臂大臂、小臂、清洗盘上三个电杆对应长度为CleanArm_Len1,CleanArm_Len2,CleanArm_Len3
float CleanArm_a1,CleanArm_a2,CleanArm_a3,CleanArm_a4,CleanArm_X,CleanArm_Y,CleanArm_Z,CleanArm_Len1,CleanArm_Len2,CleanArm_Len3;

//清洗臂末端初始位置采集函数
void CleanArm_FirstPostion(void)
{
 	SCA1000_ReadAXData();				//读取清洗臂三个加速度传感器角度值
	delay_ms(1000);
	if(Angleflag==1)
	{
//		CleanArm_a1=-1.5*(3.14/180);
//	   CleanArm_a2=(7.8-2.32)*(3.14/180);
//	   CleanArm_a3=(-41.13-CleanArm_a2-14)*(3.14/180);
//	   CleanArm_a1=15*(3.14/180);
//	   CleanArm_a2=(7.8-7.8)*(3.14/180);
//	   CleanArm_a3=(-45.96-CleanArm_a2-14)*(3.14/180); 
		CleanArm_a1=Encoder_Angle(7)*(3.14/180); 	//读取7号电机（清洗臂底座电机）初始角度,将角度转换为弧度值
		CleanArm_a2=(7.8-fabs(realAX1))*(3.14/180);	//大臂初始角度值，将角度转换为弧度值
		CleanArm_a3=(-fabs(realAX3)-CleanArm_a2-14)*(3.14/180);//小臂初始角度值，将角度转换为弧度值，fabs()求浮点型数绝对值
////		CleanArm_a4=Encoder_Angle(2)*(3.14/180);	//清洗盘电机初始角度值//暂时不需要

		CleanArm_X=cos(CleanArm_a1)*(300*cos(CleanArm_a2+CleanArm_a3)+600*cos(CleanArm_a2));		//初始状态X轴坐标值
		CleanArm_Y=sin(CleanArm_a1)*(300*cos(CleanArm_a2+CleanArm_a3)+600*cos(CleanArm_a2)); 	//初始状态Y轴坐标值
		CleanArm_Z=300*sin(CleanArm_a2+CleanArm_a3)+600*sin(CleanArm_a2);		 		//初始状态Z轴坐标值
		CleanArm_Len1=sqrt(129210.5-105432.86*cos(103.33*(3.14/180)-CleanArm_a2)); //初始状态清洗臂上电杆1本体加伸缩长度总长度
		CleanArm_Len2=sqrt(184172.09-151880*cos(-CleanArm_a3-1.92*(3.14/180))); 			//初始状态清洗臂上电杆2本体加伸缩长度总长度
		CleanArm_Len3=sqrt(98549.76-61440*cos(-CleanArm_a3-CleanArm_a2));  						//初始状态清洗臂上电杆3本体加伸缩长度总长度
		Angleflag=0;	  //清洗臂末端初始位置采集结束
//		printf(" %f",CleanArm_a1*(180/3.14));
//		printf(" %f",CleanArm_a2*(180/3.14));
//		printf(" %f",CleanArm_a3*(180/3.14));
//		printf("  %f",CleanArm_X);
//		printf("  %f",CleanArm_Y);
//		printf("  %f",CleanArm_Z);
	}
}


/****清洗臂根据上位机轨迹规划指令移动函数****/
//函数功能：先根据传感器采集的初始角度计算清洗盘末端坐标；
//底座、大臂、小臂对应初始角度对应弧度值：a1,a2,a3；清洗盘初始坐标值x,y,z
//清洗臂大臂、小臂、清洗盘上三个电杆对应长度为len1，len2，len3
//电杆移动的相对距离dl1,dl2,dl3,底座电机移动后变化角度da1
void CleanArm_PathMove(float dx,float dy,float dz)
{
	float m,dl1,dl2,dl3,da1;
	u8 flag1=1;
	if(flag1==1)
	{
	/****沿着X轴移动dx，Y轴移动dy，Z轴移动dz距离时，计算清洗臂各个关节对应角度值对应弧度值CleanArm_a1,CleanArm_a2,CleanArm_a3 ****/
		CleanArm_X=CleanArm_X+dx;	  		//移动后X轴坐标值
		CleanArm_Y=CleanArm_Y+dy;	 		//移动后Y轴坐标值
		CleanArm_Z=CleanArm_Z+dz;	 		//移动后Z轴坐标值
		da1=(atan(CleanArm_Y/CleanArm_X)-CleanArm_a1)*(180/3.14);  	//底座电机移动后变化角度值dCleanArm_a1
		CleanArm_a1=atan(CleanArm_Y/CleanArm_X);			   			//移动后底座对应弧度值
		m=CleanArm_X*cos(CleanArm_a1)+CleanArm_Y*sin(CleanArm_a1);
		CleanArm_a3=-acos((CleanArm_Z*CleanArm_Z+m*m-450000)/360000);	//移动后小臂对应弧度值,CleanArm_a3始终为负
		CleanArm_a2=atan((600*CleanArm_Z-300*(sin(CleanArm_a3)*m-CleanArm_Z*cos(CleanArm_a3)))/(600*m+300*(cos(CleanArm_a3)*m+CleanArm_Z*sin(CleanArm_a3))));	//移动后大臂对应弧度值
	/****移动一段距离后，清洗臂上相应电动推杆移动相对距离计算****/	
		dl1=sqrt(129210.5-105432.86*cos(103.33*(3.14/180)-CleanArm_a2))-CleanArm_Len1;				//移动后大臂上电杆移动长度
		dl2=sqrt(184172.09-151880*cos(-CleanArm_a3-1.92*(3.14/180)))-CleanArm_Len2;				//移动后小臂上电杆移动长度
		dl3=sqrt(98549.76-61440*cos(-CleanArm_a3-CleanArm_a2))-CleanArm_Len3;								//移动后清洗盘上电杆移动长度
		CleanArm_Len1=sqrt(129210.5-105432.86*cos(103.33*(3.14/180)-CleanArm_a2));
		CleanArm_Len2=sqrt(184172.09-151880*cos(-CleanArm_a3-1.92*(3.14/180)));
		CleanArm_Len3=sqrt(98549.76-61440*cos(-CleanArm_a3-CleanArm_a2));

//		printf("%f",CleanArm_a1*(180/3.14));
//		printf("  %f",da1);
//		printf("  %f",CleanArm_X);
//		printf("  %f",CleanArm_Y);
//		printf("  %f",CleanArm_Z);
//	 	printf("  %f",CleanArm_Len1);
//		printf("  %f",dl1);
//		printf("  %f",CleanArm_Len2);
//		printf("  %f",dl2);
//		printf("  %f",CleanArm_Len3);	
//		printf("  %f\n",dl3);							
		if(da1>0)//底座电机逆时针转动
		{
		 	ServoMotor_TurnAngle(da1,0,7);		//逆时针转动，电机编号为7
//			ServoMotor_TurnAngle(da1,0,2);		//顺时针转动，电机编号为2 ，电机2由于反装，所以转动方向都是和正装（电机轴朝上）相反的
												//电机2始终与电机7转动方向相反
		}
		else
		{
			ServoMotor_TurnAngle(-da1,1,7); 		//顺时针转动，电机编号为7
//			ServoMotor_TurnAngle(-da1,1,2);		    //逆时针转动，电机编号为2，电机2和7方向相反
		}
		if(dl3>0)
		{
			ElectricPutter_Move(dl3,1,15);	  	//电杆向前移动函数，电杆编号为15
		}
		else
		{
			ElectricPutter_Move(-dl3,0,15);	  	//电杆向后移动函数，电杆编号为15
		}
		if(dl2>0)
		{
			ElectricPutter_Move(dl2,1,14);	  	//电杆向前移动函数，电杆编号为14
		}
		else
		{
			ElectricPutter_Move(-dl2,0,14);	 	//电杆向后移动函数，电杆编号为14
		}
		if(dl1>0)
		{
			ElectricPutter_Move(dl1,1,13);	  	//电杆向前移动函数，电杆编号为13
		}
		else
		{
			ElectricPutter_Move(-dl1,0,13);	  	//电杆向后移动函数，电杆编号为13
		}
		
		
//	   delay_ms(100);
	}
}
