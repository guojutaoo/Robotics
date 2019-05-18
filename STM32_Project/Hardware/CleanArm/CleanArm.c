#include "CleanArm.h"
#include "SCA1000.h"
#include "ServoMotor.h"
#include "systick.h"
#include "math.h"
#include "ElectricPutter.h"
#include "printf.h"
u8 Angleflag=1;	  //��ϴ��ĩ�˳�ʼλ�òɼ���־λ��1���ɼ���ʼ��0���ɼ�����
//��������ۡ�С�۶�Ӧ��ʼ�Ƕȶ�Ӧ����ֵ��CleanArm_a1,CleanArm_a2,CleanArm_a3��
//��ϴ�̳�ʼ����ֵCleanArm_X,CleanArm_Y,CleanArm_Z,
//��ϴ�۴�ۡ�С�ۡ���ϴ����������˶�Ӧ����ΪCleanArm_Len1,CleanArm_Len2,CleanArm_Len3
float CleanArm_a1,CleanArm_a2,CleanArm_a3,CleanArm_a4,CleanArm_X,CleanArm_Y,CleanArm_Z,CleanArm_Len1,CleanArm_Len2,CleanArm_Len3;

//��ϴ��ĩ�˳�ʼλ�òɼ�����
void CleanArm_FirstPostion(void)
{
 	SCA1000_ReadAXData();				//��ȡ��ϴ���������ٶȴ������Ƕ�ֵ
	delay_ms(1000);
	if(Angleflag==1)
	{
//		CleanArm_a1=-1.5*(3.14/180);
//	   CleanArm_a2=(7.8-2.32)*(3.14/180);
//	   CleanArm_a3=(-41.13-CleanArm_a2-14)*(3.14/180);
//	   CleanArm_a1=15*(3.14/180);
//	   CleanArm_a2=(7.8-7.8)*(3.14/180);
//	   CleanArm_a3=(-45.96-CleanArm_a2-14)*(3.14/180); 
		CleanArm_a1=Encoder_Angle(7)*(3.14/180); 	//��ȡ7�ŵ������ϴ�۵����������ʼ�Ƕ�,���Ƕ�ת��Ϊ����ֵ
		CleanArm_a2=(7.8-fabs(realAX1))*(3.14/180);	//��۳�ʼ�Ƕ�ֵ�����Ƕ�ת��Ϊ����ֵ
		CleanArm_a3=(-fabs(realAX3)-CleanArm_a2-14)*(3.14/180);//С�۳�ʼ�Ƕ�ֵ�����Ƕ�ת��Ϊ����ֵ��fabs()�󸡵���������ֵ
////		CleanArm_a4=Encoder_Angle(2)*(3.14/180);	//��ϴ�̵����ʼ�Ƕ�ֵ//��ʱ����Ҫ

		CleanArm_X=cos(CleanArm_a1)*(300*cos(CleanArm_a2+CleanArm_a3)+600*cos(CleanArm_a2));		//��ʼ״̬X������ֵ
		CleanArm_Y=sin(CleanArm_a1)*(300*cos(CleanArm_a2+CleanArm_a3)+600*cos(CleanArm_a2)); 	//��ʼ״̬Y������ֵ
		CleanArm_Z=300*sin(CleanArm_a2+CleanArm_a3)+600*sin(CleanArm_a2);		 		//��ʼ״̬Z������ֵ
		CleanArm_Len1=sqrt(129210.5-105432.86*cos(103.33*(3.14/180)-CleanArm_a2)); //��ʼ״̬��ϴ���ϵ��1��������������ܳ���
		CleanArm_Len2=sqrt(184172.09-151880*cos(-CleanArm_a3-1.92*(3.14/180))); 			//��ʼ״̬��ϴ���ϵ��2��������������ܳ���
		CleanArm_Len3=sqrt(98549.76-61440*cos(-CleanArm_a3-CleanArm_a2));  						//��ʼ״̬��ϴ���ϵ��3��������������ܳ���
		Angleflag=0;	  //��ϴ��ĩ�˳�ʼλ�òɼ�����
//		printf(" %f",CleanArm_a1*(180/3.14));
//		printf(" %f",CleanArm_a2*(180/3.14));
//		printf(" %f",CleanArm_a3*(180/3.14));
//		printf("  %f",CleanArm_X);
//		printf("  %f",CleanArm_Y);
//		printf("  %f",CleanArm_Z);
	}
}


/****��ϴ�۸�����λ���켣�滮ָ���ƶ�����****/
//�������ܣ��ȸ��ݴ������ɼ��ĳ�ʼ�Ƕȼ�����ϴ��ĩ�����ꣻ
//��������ۡ�С�۶�Ӧ��ʼ�Ƕȶ�Ӧ����ֵ��a1,a2,a3����ϴ�̳�ʼ����ֵx,y,z
//��ϴ�۴�ۡ�С�ۡ���ϴ����������˶�Ӧ����Ϊlen1��len2��len3
//����ƶ�����Ծ���dl1,dl2,dl3,��������ƶ���仯�Ƕ�da1
void CleanArm_PathMove(float dx,float dy,float dz)
{
	float m,dl1,dl2,dl3,da1;
	u8 flag1=1;
	if(flag1==1)
	{
	/****����X���ƶ�dx��Y���ƶ�dy��Z���ƶ�dz����ʱ��������ϴ�۸����ؽڶ�Ӧ�Ƕ�ֵ��Ӧ����ֵCleanArm_a1,CleanArm_a2,CleanArm_a3 ****/
		CleanArm_X=CleanArm_X+dx;	  		//�ƶ���X������ֵ
		CleanArm_Y=CleanArm_Y+dy;	 		//�ƶ���Y������ֵ
		CleanArm_Z=CleanArm_Z+dz;	 		//�ƶ���Z������ֵ
		da1=(atan(CleanArm_Y/CleanArm_X)-CleanArm_a1)*(180/3.14);  	//��������ƶ���仯�Ƕ�ֵdCleanArm_a1
		CleanArm_a1=atan(CleanArm_Y/CleanArm_X);			   			//�ƶ��������Ӧ����ֵ
		m=CleanArm_X*cos(CleanArm_a1)+CleanArm_Y*sin(CleanArm_a1);
		CleanArm_a3=-acos((CleanArm_Z*CleanArm_Z+m*m-450000)/360000);	//�ƶ���С�۶�Ӧ����ֵ,CleanArm_a3ʼ��Ϊ��
		CleanArm_a2=atan((600*CleanArm_Z-300*(sin(CleanArm_a3)*m-CleanArm_Z*cos(CleanArm_a3)))/(600*m+300*(cos(CleanArm_a3)*m+CleanArm_Z*sin(CleanArm_a3))));	//�ƶ����۶�Ӧ����ֵ
	/****�ƶ�һ�ξ������ϴ������Ӧ�綯�Ƹ��ƶ���Ծ������****/	
		dl1=sqrt(129210.5-105432.86*cos(103.33*(3.14/180)-CleanArm_a2))-CleanArm_Len1;				//�ƶ������ϵ���ƶ�����
		dl2=sqrt(184172.09-151880*cos(-CleanArm_a3-1.92*(3.14/180)))-CleanArm_Len2;				//�ƶ���С���ϵ���ƶ�����
		dl3=sqrt(98549.76-61440*cos(-CleanArm_a3-CleanArm_a2))-CleanArm_Len3;								//�ƶ�����ϴ���ϵ���ƶ�����
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
		if(da1>0)//���������ʱ��ת��
		{
		 	ServoMotor_TurnAngle(da1,0,7);		//��ʱ��ת����������Ϊ7
//			ServoMotor_TurnAngle(da1,0,2);		//˳ʱ��ת����������Ϊ2 �����2���ڷ�װ������ת�������Ǻ���װ������ᳯ�ϣ��෴��
												//���2ʼ������7ת�������෴
		}
		else
		{
			ServoMotor_TurnAngle(-da1,1,7); 		//˳ʱ��ת����������Ϊ7
//			ServoMotor_TurnAngle(-da1,1,2);		    //��ʱ��ת����������Ϊ2�����2��7�����෴
		}
		if(dl3>0)
		{
			ElectricPutter_Move(dl3,1,15);	  	//�����ǰ�ƶ���������˱��Ϊ15
		}
		else
		{
			ElectricPutter_Move(-dl3,0,15);	  	//�������ƶ���������˱��Ϊ15
		}
		if(dl2>0)
		{
			ElectricPutter_Move(dl2,1,14);	  	//�����ǰ�ƶ���������˱��Ϊ14
		}
		else
		{
			ElectricPutter_Move(-dl2,0,14);	 	//�������ƶ���������˱��Ϊ14
		}
		if(dl1>0)
		{
			ElectricPutter_Move(dl1,1,13);	  	//�����ǰ�ƶ���������˱��Ϊ13
		}
		else
		{
			ElectricPutter_Move(-dl1,0,13);	  	//�������ƶ���������˱��Ϊ13
		}
		
		
//	   delay_ms(100);
	}
}
