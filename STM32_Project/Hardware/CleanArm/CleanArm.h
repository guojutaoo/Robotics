#ifndef _CleanArm_H
#define _CleanArm_H
#include "stm32f10x.h"
extern u8 Angleflag;	  //��ϴ��ĩ�˳�ʼλ�òɼ���־λ
extern float CleanArm_a1,CleanArm_a2,CleanArm_a3,CleanArm_a4,CleanArm_X,CleanArm_Y,CleanArm_Z,CleanArm_Len1,CleanArm_Len2,CleanArm_Len3;

void CleanArm_PathMove(float dx,float dy,float dz);//��ϴ�۸�����λ���켣�滮ָ���ƶ�����
void CleanArm_FirstPostion(void);//��ϴ��ĩ�˳�ʼλ�òɼ�����

#endif 
