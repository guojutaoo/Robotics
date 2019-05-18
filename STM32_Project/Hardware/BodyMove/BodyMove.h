#ifndef _BodyMove_H
#define _BodyMove_H
#include "stm32f10x.h"
void BodyDJMove_SameA(float SameDJAngle);
void BodyDJMove_SameRA(float SameDJAngle); //肢体电机移动相同角度函数
void BodyDJMove_SameB(float SameDJAngle);
void BodyDJMove_SameC(float SameDJAngle);  //3，4
void BodyDJMove_SameD(float SameDJAngle);  //5，6
void BodyDJMove_SameE(float SameDJAngle);
void BodyDJMove_SameF(float SameDJAngle);
float RHLegOriginal(int n);
float RHLegLocation(float x,float y,float z,int n);
void RHLegMove1(int RHLdistance,int n);
void RHLegMovetiaoshi(float djrx,float m7,float m8);
void RHLegMove2(int n);	 
void RHLegMove3(int RHLdistance,int n);
void ElectricPutter_MoveSame14(float DGlen,u8 DGdir);
void ElectricPutter_MoveSame7A(float DGlen,u8 DGdir);
void ElectricPutter_MoveSame25(float DGlen,u8 DGdir);
void ElectricPutter_MoveSame8B(float DGlen,u8 DGdir);
void ElectricPutter_MoveSame28(float DGlen,u8 DGdir);
void ElectricPutter_MoveSame5B(float DGlen,u8 DGdir);
void ElectricPutter_MoveSame12(float DGlen,u8 DGdir);
void ElectricPutter_MoveSame45(float DGlen,u8 DGdir);
void ElectricPutter_MoveSame78(float DGlen,u8 DGdir);
void ElectricPutter_MoveSameAB(float DGlen,u8 DGdir);
void ElectricPutter_MoveSame147A1(float DGlen,u8 DGdir);
void ElectricPutter_MoveSame147A2(float DGlen,u8 DGdir);
void ElectricPutter_MoveSame147A3(float DGlen,u8 DGdir);
void ElectricPutter_MoveSame147A4(float DGlen,u8 DGdir);
void ElectricPutter_MoveSame258B1(float DGlen,u8 DGdir);
void ElectricPutter_MoveSame258B2(float DGlen,u8 DGdir);
void ElectricPutter_MoveSame258B3(float DGlen,u8 DGdir);
void ElectricPutter_MoveSame258B4(float DGlen,u8 DGdir);
void ElectricPutter_MoveSame369C (float DGlen,u8 DGdir);
void BodyLegMove1(float RHLdistance,float n);
void BodyLegMove2(int n);
void BodyLegMove3(float RHLdistance,float n);
#endif 


