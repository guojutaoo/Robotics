#ifndef _CleanArm_H
#define _CleanArm_H
#include "stm32f10x.h"
extern u8 Angleflag;	  //清洗臂末端初始位置采集标志位
extern float CleanArm_a1,CleanArm_a2,CleanArm_a3,CleanArm_a4,CleanArm_X,CleanArm_Y,CleanArm_Z,CleanArm_Len1,CleanArm_Len2,CleanArm_Len3;

void CleanArm_PathMove(float dx,float dy,float dz);//清洗臂根据上位机轨迹规划指令移动函数
void CleanArm_FirstPostion(void);//清洗臂末端初始位置采集函数

#endif 
