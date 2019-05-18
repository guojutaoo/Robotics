#include "BodyMove.h"
#include "ServoMotor.h"
#include "systick.h"
#include "ElectricPutter.h"
#include "math.h"
#include "printf.h"
#include "MNspi.h"

/*****************���������ϵ����ת��ͬ�ĽǶ�ֵϵ�к���******************/
//��������:�ĸ������ת��ͬ�Ƕ�ֵ��DJAngle��
//���4.5˳ʱ����ת
//���3.6��ʱ����ת
void BodyDJMove_SameA(float SameDJAngle)
{
	s32 i,SameDJsteps,a=1;
	if(a==1)	
	SameDJsteps=1000*SameDJAngle;//�������ת�Ƕ�ת��Ϊ��������ÿ����1000������
	GPIO_SetBits(GPIOD,CB4); //���1˳ʱ����ת
	GPIO_SetBits(GPIOD,CB5); //���3˳ʱ����ת 
	GPIO_ResetBits(GPIOD,CB3);//���2��ʱ��ת��
	GPIO_ResetBits(GPIOD,CB6);//���4��ʱ��ת��
	for(i=0;i<SameDJsteps;i++)
	{ 
		GPIO_ResetBits(GPIOD,CA3);
		GPIO_ResetBits(GPIOD,CA4);
		GPIO_ResetBits(GPIOD,CA5);
		GPIO_ResetBits(GPIOD,CA6);
		delay_us(60);
		GPIO_SetBits(GPIOD,CA3);
		GPIO_SetBits(GPIOD,CA4);
		GPIO_SetBits(GPIOD,CA5);
		GPIO_SetBits(GPIOD,CA6);
		delay_us(60);
	}
}
//��������:�ĸ������ת��ͬ�Ƕ�ֵ��DJAngle��
//���4.5��ʱ����ת
//���3.6˳ʱ����ת
void BodyDJMove_SameRA(float SameDJAngle)
{
	s32 i,SameDJsteps,a=1;
	if(a==1)	
	SameDJsteps=1000*SameDJAngle;//�������ת�Ƕ�ת��Ϊ��������ÿ����1000������
	GPIO_SetBits(GPIOD,CB3); //���1˳ʱ����ת
	GPIO_SetBits(GPIOD,CB6); //���3˳ʱ����ת 
	GPIO_ResetBits(GPIOD,CB4);//���2��ʱ��ת��
	GPIO_ResetBits(GPIOD,CB5);//���4��ʱ��ת��
	for(i=0;i<SameDJsteps;i++)
	{ 
		GPIO_ResetBits(GPIOD,CA3);
		GPIO_ResetBits(GPIOD,CA4);
		GPIO_ResetBits(GPIOD,CA5);
		GPIO_ResetBits(GPIOD,CA6);
		delay_us(60);
		GPIO_SetBits(GPIOD,CA3);
		GPIO_SetBits(GPIOD,CA4);
		GPIO_SetBits(GPIOD,CA5);
		GPIO_SetBits(GPIOD,CA6);
		delay_us(60);
	}
}
//���3.5˳ʱ����ת
//���4.6��ʱ����ת
void BodyDJMove_SameB(float SameDJAngle)
{
	s32 i,DJsteps,a=1;
	if(a==1)	
	DJsteps=1000*SameDJAngle;//�������ת�Ƕ�ת��Ϊ��������ÿ����1000������
	GPIO_SetBits(GPIOD,CB3); //���3˳ʱ����ת
	GPIO_SetBits(GPIOD,CB5);//���5˳ʱ��ת��
	GPIO_ResetBits(GPIOD,CB6); //���4��ʱ����ת 
	GPIO_ResetBits(GPIOD,CB4);//���6��ʱ��ת��
	
	for(i=0;i<DJsteps;i++)
	{ 
		GPIO_ResetBits(GPIOD,CA3);
		GPIO_ResetBits(GPIOD,CA4);
		GPIO_ResetBits(GPIOD,CA5);
		GPIO_ResetBits(GPIOD,CA6);
		delay_us(60);
		GPIO_SetBits(GPIOD,CA3);
		GPIO_SetBits(GPIOD,CA4);
		GPIO_SetBits(GPIOD,CA5);
		GPIO_SetBits(GPIOD,CA6);
		delay_us(60);
	}
}
//��������:���������ת��ͬ�Ƕ�ֵ��DJAngle��
//���4˳ʱ����ת
//���3��ʱ����ת
void BodyDJMove_SameC(float SameDJAngle)
{
	s32 i,SameDJsteps,a=1;
	if(a==1)	
	SameDJsteps=1000*SameDJAngle;//�������ת�Ƕ�ת��Ϊ��������ÿ����1000������
	GPIO_SetBits(GPIOD,CB4); //���4˳ʱ����ת
	GPIO_ResetBits(GPIOD,CB3);//���3��ʱ��ת��
	for(i=0;i<SameDJsteps;i++)
	{ 
		GPIO_ResetBits(GPIOD,CA3);
		GPIO_ResetBits(GPIOD,CA4);
		delay_us(60);
		GPIO_SetBits(GPIOD,CA3);
		GPIO_SetBits(GPIOD,CA4);
		delay_us(60);
	}
}
//��������:���������ת��ͬ�Ƕ�ֵ��DJAngle��
//���5˳ʱ����ת
//���6��ʱ����ת
void BodyDJMove_SameD(float SameDJAngle)
{
	s32 i,SameDJsteps,a=1;
	if(a==1)	
	SameDJsteps=1000*SameDJAngle;//�������ת�Ƕ�ת��Ϊ��������ÿ����1000������
	GPIO_SetBits(GPIOD,CB5); //���4˳ʱ����ת
	GPIO_ResetBits(GPIOD,CB6);//���3��ʱ��ת��
	for(i=0;i<SameDJsteps;i++)
	{ 
		GPIO_ResetBits(GPIOD,CA5);
		GPIO_ResetBits(GPIOD,CA6);
		delay_us(60);
		GPIO_SetBits(GPIOD,CA5);
		GPIO_SetBits(GPIOD,CA6);
		delay_us(60);
	}
}
//3,4,5,6ͬʱ��ʱ����ת
void BodyDJMove_SameE(float SameDJAngle)
{
	s32 i,SameDJsteps,a=1;
	if(a==1)	
	SameDJsteps=1000*SameDJAngle;//�������ת�Ƕ�ת��Ϊ��������ÿ����1000������
    GPIO_ResetBits(GPIOD,CB3);//���2��ʱ��ת��
	GPIO_ResetBits(GPIOD,CB4);//���4��ʱ��ת��
	GPIO_ResetBits(GPIOD,CB5);//���2��ʱ��ת��
	GPIO_ResetBits(GPIOD,CB6);//���4��ʱ��ת��
	for(i=0;i<SameDJsteps;i++)
	{ 
		GPIO_ResetBits(GPIOD,CA3);
		GPIO_ResetBits(GPIOD,CA4);
		GPIO_ResetBits(GPIOD,CA5);
		GPIO_ResetBits(GPIOD,CA6);
		delay_us(100);
		GPIO_SetBits(GPIOD,CA3);
		GPIO_SetBits(GPIOD,CA4);
		GPIO_SetBits(GPIOD,CA5);
		GPIO_SetBits(GPIOD,CA6);
		delay_us(100);
	}
}
//3,4,5,6ͬʱ˳ʱ����ת
void BodyDJMove_SameF(float SameDJAngle)
{
	s32 i,SameDJsteps,a=1;
	if(a==1)	
	SameDJsteps=1000*SameDJAngle;//�������ת�Ƕ�ת��Ϊ��������ÿ����1000������
    GPIO_SetBits(GPIOD,CB3); //���1˳ʱ����ת
	GPIO_SetBits(GPIOD,CB4); //���3˳ʱ����ת 
	GPIO_SetBits(GPIOD,CB5); //���1˳ʱ����ת
	GPIO_SetBits(GPIOD,CB6); //���3˳ʱ����ת
	for(i=0;i<SameDJsteps;i++)
	{ 
		GPIO_ResetBits(GPIOD,CA3);
		GPIO_ResetBits(GPIOD,CA4);
		GPIO_ResetBits(GPIOD,CA5);
		GPIO_ResetBits(GPIOD,CA6);
		delay_us(100);
		GPIO_SetBits(GPIOD,CA3);
		GPIO_SetBits(GPIOD,CA4);
		GPIO_SetBits(GPIOD,CA5);
		GPIO_SetBits(GPIOD,CA6);
		delay_us(100);
	}
}


/*****************���������ϵ����ת��ͬ�ĽǶ�ֵϵ�к���******************/
void ElectricPutter_MoveSame14(float DGlen,u8 DGdir)
{
	        u32 i,steps;
        	steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
			    GPIO_SetBits(GPIOF,NP1); 
				GPIO_SetBits(GPIOF,NP4);       
			}
			else
			{
			    GPIO_ResetBits(GPIOF,NP1);
				GPIO_ResetBits(GPIOF,NP4);
			}
			
			for(i=0;i<steps;i++)
			{ 
			    GPIO_ResetBits(GPIOF,PP1);
				GPIO_ResetBits(GPIOF,PP4);
				delay_us(8);
				GPIO_SetBits(GPIOF,PP1);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOF,PP4);
				delay_us(8);			//��ʱԼΪ8us
			}
}
void ElectricPutter_MoveSame7A(float DGlen,u8 DGdir)
{
            u32 i,steps;
        	steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{  
				GPIO_SetBits(GPIOF,NP7);
				GPIO_SetBits(GPIOG,NP10);         
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP7);
				GPIO_ResetBits(GPIOG,NP10);
			}
			
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP7);
				GPIO_ResetBits(GPIOG,PP10);
				delay_us(8);
				GPIO_SetBits(GPIOF,PP7);
				GPIO_SetBits(GPIOG,PP10);
				delay_us(8);			//��ʱԼΪ8us
			}
}
void ElectricPutter_MoveSame25(float DGlen,u8 DGdir)
{
            u32 i,steps;
        	steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
			    GPIO_SetBits(GPIOF,NP2); 
				GPIO_SetBits(GPIOF,NP5);       
			}
			else
			{
			    GPIO_ResetBits(GPIOF,NP2);
				GPIO_ResetBits(GPIOF,NP5);
			}
			
			for(i=0;i<steps;i++)
			{ 
			    GPIO_ResetBits(GPIOF,PP2);
				GPIO_ResetBits(GPIOF,PP5);
				delay_us(8);
				GPIO_SetBits(GPIOF,PP2);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOF,PP5);
				delay_us(8);			//��ʱԼΪ8us
			}
}
void ElectricPutter_MoveSame8B(float DGlen,u8 DGdir)
{
            u32 i,steps;
        	steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP8);
				GPIO_SetBits(GPIOG,NP11);         
			}
			else
			{
			    GPIO_ResetBits(GPIOF,NP2);
				GPIO_ResetBits(GPIOF,NP5);
				GPIO_ResetBits(GPIOF,NP8);
				GPIO_ResetBits(GPIOG,NP11);
			}
			
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP8);
				GPIO_ResetBits(GPIOG,PP11);
				delay_us(8);
				GPIO_SetBits(GPIOF,PP8);
				GPIO_SetBits(GPIOG,PP11);
				delay_us(8);			//��ʱԼΪ8us
			}
}
void ElectricPutter_MoveSame28(float DGlen,u8 DGdir)
{
            u32 i,steps;
        	steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
			    GPIO_SetBits(GPIOF,NP2); 
				GPIO_SetBits(GPIOF,NP8);       
			}
			else
			{
			    GPIO_ResetBits(GPIOF,NP2);
				GPIO_ResetBits(GPIOF,NP8);
			}
			
			for(i=0;i<steps;i++)
			{ 
			    GPIO_ResetBits(GPIOF,PP2);
				GPIO_ResetBits(GPIOF,PP8);
				delay_us(8);
				GPIO_SetBits(GPIOF,PP2);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOF,PP8);
				delay_us(8);			//��ʱԼΪ8us
			}
}
void ElectricPutter_MoveSame5B(float DGlen,u8 DGdir)
{
            u32 i,steps;
        	steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP5);
				GPIO_SetBits(GPIOG,NP11);         
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP5);
				GPIO_ResetBits(GPIOG,NP11);
			}
			
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP5);
				GPIO_ResetBits(GPIOG,PP11);
				delay_us(8);
				GPIO_SetBits(GPIOF,PP5);
				GPIO_SetBits(GPIOG,PP11);
				delay_us(8);			//��ʱԼΪ8us
			}
}
void ElectricPutter_MoveSame45(float DGlen,u8 DGdir)
{
            u32 i,steps;
        	steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP4);
				GPIO_SetBits(GPIOF,NP5);         
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP4);
				GPIO_ResetBits(GPIOF,NP5);
			}
			
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP4);
				GPIO_ResetBits(GPIOF,PP5);
				delay_us(8);
				GPIO_SetBits(GPIOF,PP4);
				GPIO_SetBits(GPIOF,PP5);
				delay_us(8);			//��ʱԼΪ8us
			}
}
void ElectricPutter_MoveSame78(float DGlen,u8 DGdir)
{
            u32 i,steps;
        	steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP7);
				GPIO_SetBits(GPIOF,NP8);         
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP7);
				GPIO_ResetBits(GPIOF,NP8);
			}
			
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP7);
				GPIO_ResetBits(GPIOF,PP8);
				delay_us(8);
				GPIO_SetBits(GPIOF,PP7);
				GPIO_SetBits(GPIOF,PP8);
				delay_us(8);			//��ʱԼΪ8us
			}
}
void ElectricPutter_MoveSameAB(float DGlen,u8 DGdir)
{
            u32 i,steps;
        	steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOG,NP10);
				GPIO_SetBits(GPIOG,NP11);         
			}
			else
			{
				GPIO_ResetBits(GPIOG,NP10);
				GPIO_ResetBits(GPIOG,NP11);
			}
			
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOG,PP10);
				GPIO_ResetBits(GPIOG,PP11);
				delay_us(8);
				GPIO_SetBits(GPIOG,PP10);
				GPIO_SetBits(GPIOG,PP11);
				delay_us(8);			//��ʱԼΪ8us
			}
}
void ElectricPutter_MoveSame12(float DGlen,u8 DGdir)
{
            u32 i,steps;
        	steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
				GPIO_SetBits(GPIOF,NP1);
				GPIO_SetBits(GPIOF,NP2);         
			}
			else
			{
				GPIO_ResetBits(GPIOF,NP1);
				GPIO_ResetBits(GPIOF,NP2);
			}
			
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP1);
				GPIO_ResetBits(GPIOF,PP2);
				delay_us(8);
				GPIO_SetBits(GPIOF,PP1);
				GPIO_SetBits(GPIOF,PP2);
				delay_us(8);			//��ʱԼΪ8us
			}
}
//�������ܣ��ĸ����ͬʱ������ͬ����
void ElectricPutter_MoveSame147A1(float DGlen,u8 DGdir)
{
	        u32 i,steps;
        	steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
			    GPIO_SetBits(GPIOF,NP1); 
				GPIO_SetBits(GPIOF,NP4);
				GPIO_SetBits(GPIOF,NP7);
				GPIO_SetBits(GPIOG,NP10);         
			}
			else
			{
			    GPIO_ResetBits(GPIOF,NP1);
				GPIO_ResetBits(GPIOF,NP4);
				GPIO_ResetBits(GPIOF,NP7);
				GPIO_ResetBits(GPIOG,NP10);
			}
			
			for(i=0;i<steps;i++)
			{ 
			    GPIO_ResetBits(GPIOF,PP1);
				GPIO_ResetBits(GPIOF,PP4);
				GPIO_ResetBits(GPIOF,PP7);
				GPIO_ResetBits(GPIOG,PP10);
				delay_us(8);
				GPIO_SetBits(GPIOF,PP1);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOF,PP4);
				GPIO_SetBits(GPIOF,PP7);
				GPIO_SetBits(GPIOG,PP10);
				delay_us(8);			//��ʱԼΪ8us
			}
}
//�������ܣ�1,4,7,10�ŵ��ͬʱ������ͬ����
//DGdir=1ʱ��1,7�쳤��4,10���̣�
//DGdir=0ʱ��1,7���̣�4,10�쳤��
void ElectricPutter_MoveSame147A2(float DGlen,u8 DGdir)
{
	        u32 i,steps;
        	steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
			    GPIO_SetBits(GPIOF,NP1);
				GPIO_ResetBits(GPIOF,NP4); 
				GPIO_SetBits(GPIOF,NP7);	
				GPIO_ResetBits(GPIOG,NP10);	       
			}
			else
			{
			    GPIO_ResetBits(GPIOF,NP1);
				GPIO_SetBits(GPIOF,NP4);
				GPIO_ResetBits(GPIOF,NP7);
				GPIO_SetBits(GPIOG,NP10);  
			}
			
			for(i=0;i<steps;i++)
			{ 
			    GPIO_ResetBits(GPIOF,PP1);
				GPIO_ResetBits(GPIOF,PP4);
				GPIO_ResetBits(GPIOF,PP7);
				GPIO_ResetBits(GPIOG,PP10);
				delay_us(8);
				GPIO_SetBits(GPIOF,PP1);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOF,PP4);
				GPIO_SetBits(GPIOF,PP7);
				GPIO_SetBits(GPIOG,PP10);
				delay_us(8);			//��ʱԼΪ8us
			}
}
void ElectricPutter_MoveSame147A3(float DGlen,u8 DGdir)
{
	        u32 i,steps;
        	steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
			    GPIO_SetBits(GPIOF,NP1);
				GPIO_ResetBits(GPIOF,NP4); 
				GPIO_SetBits(GPIOF,NP7);	
				GPIO_ResetBits(GPIOG,NP10);	       
			}
			else
			{
			    GPIO_ResetBits(GPIOF,NP1);
				GPIO_SetBits(GPIOF,NP4);
				GPIO_ResetBits(GPIOF,NP7);
				GPIO_SetBits(GPIOG,NP10);  
			}
			
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP4);
				GPIO_ResetBits(GPIOF,PP7);
				delay_us(8);
				GPIO_SetBits(GPIOF,PP4);
				GPIO_SetBits(GPIOF,PP7);
				delay_us(8);
				GPIO_ResetBits(GPIOF,PP4);
				GPIO_ResetBits(GPIOF,PP7);
				delay_us(8);
				GPIO_SetBits(GPIOF,PP4);
				GPIO_SetBits(GPIOF,PP7);
				delay_us(8);
			    GPIO_ResetBits(GPIOF,PP1);
				GPIO_ResetBits(GPIOF,PP4);
				GPIO_ResetBits(GPIOF,PP7);
				GPIO_ResetBits(GPIOG,PP10);
				delay_us(8);
				GPIO_SetBits(GPIOF,PP1);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOF,PP4);
				GPIO_SetBits(GPIOF,PP7);
				GPIO_SetBits(GPIOG,PP10);
				delay_us(8);			//��ʱԼΪ8us
			}
}
void ElectricPutter_MoveSame147A4(float DGlen,u8 DGdir)
{
	        u32 i,steps;
        	steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
			    GPIO_ResetBits(GPIOF,NP1);	
				GPIO_ResetBits(GPIOF,NP4); 
				GPIO_SetBits(GPIOF,NP7);
				GPIO_SetBits(GPIOG,NP10);	
			       
			}
			else
			{	
			    GPIO_SetBits(GPIOF,NP1);
		     	GPIO_SetBits(GPIOF,NP4);
			    GPIO_ResetBits(GPIOF,NP7);
				GPIO_ResetBits(GPIOG,NP10);
				  
			}
			
			for(i=0;i<steps;i++)
			{ 
			    GPIO_ResetBits(GPIOF,PP1);
				GPIO_ResetBits(GPIOF,PP4);
				GPIO_ResetBits(GPIOF,PP7);
				GPIO_ResetBits(GPIOG,PP10);
				delay_us(500);
				GPIO_SetBits(GPIOF,PP1);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOF,PP4);
				GPIO_SetBits(GPIOF,PP7);
				GPIO_SetBits(GPIOG,PP10);
				delay_us(500);			//��ʱԼΪ8us
			}
}

void ElectricPutter_MoveSame258B1(float DGlen,u8 DGdir)
{
            u32 i,steps;
        	steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
			    GPIO_SetBits(GPIOF,NP2); 
				GPIO_SetBits(GPIOF,NP5);
				GPIO_SetBits(GPIOF,NP8);
				GPIO_SetBits(GPIOG,NP11);         
			}
			else
			{
			    GPIO_ResetBits(GPIOF,NP2);
				GPIO_ResetBits(GPIOF,NP5);
				GPIO_ResetBits(GPIOF,NP8);
				GPIO_ResetBits(GPIOG,NP11);
			}
			
			for(i=0;i<steps;i++)
			{ 
			    GPIO_ResetBits(GPIOF,PP2);
				GPIO_ResetBits(GPIOF,PP5);
				GPIO_ResetBits(GPIOF,PP8);
				GPIO_ResetBits(GPIOG,PP11);
				delay_us(8);
				GPIO_SetBits(GPIOF,PP2);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOF,PP5);
				GPIO_SetBits(GPIOF,PP8);
				GPIO_SetBits(GPIOG,PP11);
				delay_us(8);			//��ʱԼΪ8us
			}
}
//�������ܣ�2,5,8,11�ŵ��ͬʱ������ͬ����
//DGdir=1ʱ��2,8�쳤��5,11���̣�
//DGdir=0ʱ��2,8���̣�5,11�쳤��
void ElectricPutter_MoveSame258B2(float DGlen,u8 DGdir)
{
	        u32 i,steps;
        	steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
			    GPIO_SetBits(GPIOF,NP2);
				GPIO_ResetBits(GPIOF,NP5); 
				GPIO_SetBits(GPIOF,NP8);	
				GPIO_ResetBits(GPIOG,NP11);	       
			}
			else
			{
			    GPIO_ResetBits(GPIOF,NP2);
				GPIO_SetBits(GPIOF,NP5);
				GPIO_ResetBits(GPIOF,NP8);
				GPIO_SetBits(GPIOG,NP11);  
			}
			
			for(i=0;i<steps;i++)
			{ 
			    GPIO_ResetBits(GPIOF,PP2);
				GPIO_ResetBits(GPIOF,PP5);
				GPIO_ResetBits(GPIOF,PP8);
				GPIO_ResetBits(GPIOG,PP11);
				delay_us(8);
				GPIO_SetBits(GPIOF,PP2);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOF,PP5);
				GPIO_SetBits(GPIOF,PP8);
				GPIO_SetBits(GPIOG,PP11);
				delay_us(8);			//��ʱԼΪ8us
			}
}
void ElectricPutter_MoveSame258B3(float DGlen,u8 DGdir)
{
	        u32 i,steps;
        	steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
			    GPIO_SetBits(GPIOF,NP2);
				GPIO_ResetBits(GPIOF,NP5); 
				GPIO_SetBits(GPIOF,NP8);	
				GPIO_ResetBits(GPIOG,NP11);	       
			}
			else
			{
			    GPIO_ResetBits(GPIOF,NP2);
				GPIO_SetBits(GPIOF,NP5);
				GPIO_ResetBits(GPIOF,NP8);
				GPIO_SetBits(GPIOG,NP11);  
			}
			
			for(i=0;i<steps;i++)
			{ 
				GPIO_ResetBits(GPIOF,PP5);
				GPIO_ResetBits(GPIOF,PP8);
				delay_us(8);
				GPIO_SetBits(GPIOF,PP5);
				GPIO_SetBits(GPIOF,PP8);
				delay_us(8);
				GPIO_ResetBits(GPIOF,PP5);
				GPIO_ResetBits(GPIOF,PP8);
				delay_us(8);
				GPIO_SetBits(GPIOF,PP5);
				GPIO_SetBits(GPIOF,PP8);
				delay_us(8);
			    GPIO_ResetBits(GPIOF,PP2);
				GPIO_ResetBits(GPIOF,PP5);
				GPIO_ResetBits(GPIOF,PP8);
				GPIO_ResetBits(GPIOG,PP11);
				delay_us(8);
				GPIO_SetBits(GPIOF,PP2);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOF,PP5);
				GPIO_SetBits(GPIOF,PP8);
				GPIO_SetBits(GPIOG,PP11);
				delay_us(8);			//��ʱԼΪ8us
			}
}
void ElectricPutter_MoveSame258B4(float DGlen,u8 DGdir)
{
	        u32 i,steps;
        	steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{	
			    
				GPIO_ResetBits(GPIOF,NP2); 	
				GPIO_ResetBits(GPIOF,NP5);
				GPIO_SetBits(GPIOF,NP8);
			    GPIO_SetBits(GPIOG,NP11);	       
			}
			else
			{	
			    GPIO_SetBits(GPIOF,NP2);
			    GPIO_SetBits(GPIOF,NP5);
			   	GPIO_ResetBits(GPIOF,NP8); 	
				GPIO_ResetBits(GPIOG,NP11);
				
			}
			
			for(i=0;i<steps;i++)
			{ 
			    GPIO_ResetBits(GPIOF,PP2);
				GPIO_ResetBits(GPIOF,PP5);
				GPIO_ResetBits(GPIOF,PP8);
				GPIO_ResetBits(GPIOG,PP11);
				delay_us(500);
				GPIO_SetBits(GPIOF,PP2);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOF,PP5);
				GPIO_SetBits(GPIOF,PP8);
				GPIO_SetBits(GPIOG,PP11);
				delay_us(500);			//��ʱԼΪ8us
			}
}
void ElectricPutter_MoveSame369C(float DGlen,u8 DGdir)
{
            u32 i,steps;
        	steps=800*DGlen/3;	//800�������Ӧ3mm�����ƶ�len����ʱ��Ӧ���������
			if(DGdir==1)
			{
			    GPIO_SetBits(GPIOF,NP3); 
				GPIO_SetBits(GPIOF,NP6);
				GPIO_SetBits(GPIOG,NP9);
				GPIO_SetBits(GPIOG,NP12);         
			}
			else
			{
			    GPIO_ResetBits(GPIOF,NP3);
				GPIO_ResetBits(GPIOF,NP6);
				GPIO_ResetBits(GPIOG,NP9);
				GPIO_ResetBits(GPIOG,NP12);
			}
			
			for(i=0;i<steps;i++)
			{ 
			    GPIO_ResetBits(GPIOF,PP3);
				GPIO_ResetBits(GPIOF,PP6);
				GPIO_ResetBits(GPIOG,PP9);
				GPIO_ResetBits(GPIOG,PP12);
				delay_us(8);
				GPIO_SetBits(GPIOF,PP3);			//��ʱԼΪ8us
				GPIO_SetBits(GPIOF,PP6);
				GPIO_SetBits(GPIOG,PP9);
				GPIO_SetBits(GPIOG,PP12);
				delay_us(8);			//��ʱԼΪ8us
			}
}


/*****************���ȵ�λ����Ϣ���غ���******************/
//�������ܣ������Ȳ��ĵ�ǰ����
//n:1,���ص�ǰx����
//  2,���ص�ǰy����
//  3,���ص�ǰz����
//  4,���ص�ǰ����Ƹ��������
//  5,���ص�ǰС���Ƹ��������
float RHLegOriginal(int n)
{
  float realRHAX1,realRHAX2;
  float  Lengo7,Lengo8,xo,yo,zo;
  float  theta1o,theta2o,theta3o;
//  MNspi_init();
  realRHAX1=SPI_GetLXN(7);
  realRHAX2=SPI_GetLXN(8);
  Lengo7=sqrt(45303.84-46886.4*cos((112.9-realRHAX1)/180*3.14))-62;
  Lengo8=sqrt(138050.09-87290*cos((186.8-realRHAX1+realRHAX2)/180*3.14)) -248;
  theta1o=Encoder_Angle(5)/180*3.14;
//  theta1o=0;
  theta2o=(realRHAX1-10.1)/180*3.14;
  theta3o=(-realRHAX1+10.1+realRHAX2-17)/180*3.14;
  xo=350*cos(theta1o)*cos(theta2o)+420*cos(theta1o)*cos(theta2o)*cos(theta3o)-420*cos(theta1o)*sin(theta2o)*sin(theta3o);
  yo=350*cos(theta2o)*sin(theta1o)+420*cos(theta2o)*cos(theta3o)*sin(theta1o)-420*sin(theta1o)*sin(theta2o)*sin(theta3o);
  zo=350*sin(theta2o)+420*cos(theta2o)*sin(theta3o)+420*cos(theta3o)*sin(theta2o);
 if(n==1)
 {
   return xo;
 }
 else if(n==2)
 {
  return yo;
 }
 else if(n==3)
 {
   return zo;
 }
 else if(n==4)
 {
  return Lengo7;
 }
 else if(n==5)
 {
  return Lengo8;
 }
 else
 {
 return 0;
 }
}								 
//�������ܣ���������㷵���Ȳ��˶���Ϣ
//n:1,���ص����ת�Ƕ�
//  2,���ش�۵�˾���
//  3,����С�۵�˾���
float RHLegLocation(float x,float y,float z,int n)
{
  float Leng7,Leng8;
  float  theta1,theta2,theta3;
  float rtheta1,rtheta2,rtheta3;
  theta1=atan(y/x);
  theta3=-acos(((x*cos(theta1)+y*sin(theta1))*(x*cos(theta1)+y*sin(theta1))+z*z-298900)/294000);
  theta2=atan(((350+420*cos(theta3))*z-(x*cos(theta1)+y*sin(theta1))*420*sin(theta3))/((350+420*cos(theta3))*(x*cos(theta1)+y*sin(theta1))+420*z*sin(theta3)));
  rtheta1=theta1*180/3.14;
  rtheta2=theta2*180/3.14;
  rtheta3=theta3*180/3.14;
  Leng7=sqrt(45303.84-46886.4*cos((102.8-rtheta2)/180*3.14))-62;
  Leng8=sqrt(138050.09-87290*cos((193.7+rtheta3)/180*3.14))-248;
  if(n==1)
  {
   return rtheta1;
  }
  else if(n==2)
  {
  return Leng7;
  }
  else if(n==3)
  {
  return Leng8;
  }
  else
  {
  return 0;
  }
}


/*****************���ȵ��˶�����******************/
void RHLegMove1(int RHLdistance,int n)
{
  float x0,y0,z0;
  float x,y,z,m7,m8,djrx;
  int i;
  int v1=RHLdistance/n;
  printf_init();
  x0=RHLegOriginal(1);
  y0=RHLegOriginal(2);
  z0=RHLegOriginal(3);
  x=x0,y=y0,z=z0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
  for(i=0;i<n;i++)
  {
   	y+=v1;
	djrx=RHLegLocation(x,y,z,1)-RHLegLocation(x,y-v1,z,1);
	m7=RHLegLocation(x,y,z,2)-RHLegLocation(x,y-v1,z,2);	
	m8=RHLegLocation(x,y,z,3)-RHLegLocation(x,y-v1,z,3);
	 if(djrx>0)
	{
	ServoMotor_TurnAngle(djrx,0,5);
	}
	else
	{
	 ServoMotor_TurnAngle(-djrx,1,5);
	}
	if(m7>0)
	{
	ElectricPutter_Move(m7,1,7);
	}
	else
	{
	ElectricPutter_Move(-m7,0,7);
	}	

	if(m8>0)
	{
	 ElectricPutter_Move(m8,1,8);
	}
	else
	{
	  ElectricPutter_Move(-m8,0,8);						    
	}
    printf("djrx=%f,m7=%f,m8=%f\n",djrx,m7,m8); 
  }
}
void RHLegMovetiaoshi(float djrx,float m7,float m8)
{
//     float x0,y0,z0;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
	 if(djrx>0)
	{
	ServoMotor_TurnAngle(djrx,0,5);
	}
	else
	{
	 ServoMotor_TurnAngle(-djrx,1,5);
	}
	if(m7>0)
	{
	ElectricPutter_Move(m7,1,7);
	}
	else
	{
	ElectricPutter_Move(-m7,0,7);
	}	

	if(m8>0)
	{
	 ElectricPutter_Move(m8,1,8);
	}
	else
	{
	  ElectricPutter_Move(-m8,0,8);						    
	}
//  x0=RHLegOriginal(1);
//  y0=RHLegOriginal(2);
//  z0=RHLegOriginal(3);    
//  printf("x0=%f,y0=%f,z0=%f\n",x0,y0,z0); 
}
void RHLegMove2(int n)
{
  float x0,y0,z0;
  float x,y,z,m7,m8,djrx;
  int i;
  float vx=-3.5;
  float vy=-4;
  printf_init();
  x0=RHLegOriginal(1);
  y0=RHLegOriginal(2);
  z0=RHLegOriginal(3);
  x=x0,y=y0,z=z0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
  for(i=0;i<n;i++)
  {
    x+=vx;
   	y+=vy;
	djrx=RHLegLocation(x,y,z,1)-RHLegLocation(x-vx,y-vy,z,1);
	m7=RHLegLocation(x,y,z,2)-RHLegLocation(x-vx,y-vy,z,2);	
	m8=RHLegLocation(x,y,z,3)-RHLegLocation(x-vx,y-vy,z,3);
	 if(djrx>0)
	{
	ServoMotor_TurnAngle(djrx,0,5);
	}
	else
	{
	 ServoMotor_TurnAngle(-djrx,1,5);
	}
	if(m7>0)
	{
	ElectricPutter_Move(m7,1,7);
	}
	else
	{
	ElectricPutter_Move(-m7,0,7);
	}	

	if(m8>0)
	{
	 ElectricPutter_Move(m8,1,8);
	}
	else
	{
	  ElectricPutter_Move(-m8,0,8);						    
	}
    printf("djrx=%f,m7=%f,m8=%f\n",djrx,m7,m8); 
  }

}
void RHLegMove3(int RHLdistance,int n)
{
  float x0,y0,z0;
  float x,y,z,m7,m8,djrx;
  int i;
  int v1=RHLdistance/n;
  printf_init();
  x0=RHLegOriginal(1);
  y0=RHLegOriginal(2);
  z0=RHLegOriginal(3);
  x=x0,y=y0,z=z0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
  for(i=0;i<n;i++)
  {
   	x+=v1;
	djrx=RHLegLocation(x,y,z,1)-RHLegLocation(x-v1,y,z,1);
	m7=RHLegLocation(x,y,z,2)-RHLegLocation(x-v1,y,z,2);	
	m8=RHLegLocation(x,y,z,3)-RHLegLocation(x-v1,y,z,3);
	 if(djrx>0)
	{
	ServoMotor_TurnAngle(djrx,0,5);
	}
	else
	{
	 ServoMotor_TurnAngle(-djrx,1,5);
	}
	if(m7>0)
	{
	ElectricPutter_Move(m7,1,16);
	}
	else
	{
	ElectricPutter_Move(-m7,0,16);
	}	

	if(m8>0)
	{
	 ElectricPutter_Move(m8,1,17);
	}
	else
	{
	  ElectricPutter_Move(-m8,0,17);						    
	}
    printf("djrx=%f,m7=%f,m8=%f\n",djrx,m7,m8); 
  }

}


/*****************������˶�����******************/
//�Ȳ�ֱ�����߳��� ��ֱ�ߣ�
void BodyLegMove1(float RHLdistance,float n)
{
  float x0,y0,z0;
  float x,y,z,m7,m8,djrx;
  int i;
  float v1=RHLdistance/n;
  printf_init();
  x0=RHLegOriginal(1);
  y0=RHLegOriginal(2);
  z0=RHLegOriginal(3);
  x=x0,y=y0,z=z0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
  for(i=0;i<n;i++)
  {
   	y+=v1;
	djrx=RHLegLocation(x,y,z,1)-RHLegLocation(x,y-v1,z,1);
	m7=RHLegLocation(x,y,z,2)-RHLegLocation(x,y-v1,z,2);	
	m8=RHLegLocation(x,y,z,3)-RHLegLocation(x,y-v1,z,3);
	 if(djrx>0)
	{
     BodyDJMove_SameRA(djrx);
	}
	else
	{
	 BodyDJMove_SameA(-djrx);
	}
	if(m7>0)
	{
     ElectricPutter_MoveSame147A1(m7,1);
	}
	else
	{
	ElectricPutter_MoveSame147A1(-m7,0);
	}	

	if(m8>0)
	{
	ElectricPutter_MoveSame258B1(m8,1);
	}
	else
	{
	 ElectricPutter_MoveSame258B1(-m8,0);					    
	}
    printf("djrx=%f,m7=%f,m8=%f\n",djrx,m7,m8); 
  }

}
//�Ȳ��������߳��򣨹��䣩
void BodyLegMove2(int n)
{
  float x0,y0,z0;
  float x,y,z,m7,m8,djrx;
  int i; 
  float vx=-4.0;
  float vy=-3.5;
  x0=RHLegOriginal(1);
  y0=RHLegOriginal(2);
  z0=RHLegOriginal(3);
  x=x0,y=y0,z=z0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
  for(i=0;i<n;i++)
  {
   	x+=vx;
   	y+=vy;
	djrx=RHLegLocation(x,y,z,1)-RHLegLocation(x-vx,y-vy,z,1);
	m7=RHLegLocation(x,y,z,2)-RHLegLocation(x-vx,y-vy,z,2);	
	m8=RHLegLocation(x,y,z,3)-RHLegLocation(x-vx,y-vy,z,3);
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
    printf("djrx=%f,m7=%f,m8=%f\n",djrx,m7,m8); 
  }

}
void BodyLegMove3(float RHLdistance,float n)
{
  float x0,y0,z0;
  float x,y,z,m7,m8,djrx;
  int i;
  float v1=RHLdistance/n;
  x0=RHLegOriginal(1);
  y0=RHLegOriginal(2);
  z0=RHLegOriginal(3);
  x=x0,y=y0,z=z0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
  for(i=0;i<n;i++)
  {
   	x+=v1;
	djrx=RHLegLocation(x,y,z,1)-RHLegLocation(x-v1,y,z,1);
	m7=RHLegLocation(x,y,z,2)-RHLegLocation(x-v1,y,z,2);	
	m8=RHLegLocation(x,y,z,3)-RHLegLocation(x-v1,y,z,3);
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
     ElectricPutter_MoveSame147A4(m7,1);
	}
	else
	{
	ElectricPutter_MoveSame147A4(-m7,0);
	}	

	if(m8>0)
	{
	ElectricPutter_MoveSame258B4(m8,1);
	}
	else
	{
	 ElectricPutter_MoveSame258B4(-m8,0);					    
	}
    printf("djrx=%f,m7=%f,m8=%f\n",djrx,m7,m8); 
  }

}









