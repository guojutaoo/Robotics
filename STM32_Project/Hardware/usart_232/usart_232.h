#ifndef _usart_232_H
#define _usart_232_H
#include "stm32f10x.h"

extern u8 CleanArmPathflag;		//��ϴ���尴����λ����ϴ�켣��ϴ��ʼ��־
extern u8 PointMoveflag;     	//�㶯����ģʽ��ʼ��־��1�������㶯ģʽ
extern u8 StraightWalkFlag;
extern u8 ForwordWalkFlag;
extern u8 BackwordWalkFlag;
extern u8 LTurnMoveFlag;
extern u8 RTurnMoveFlag;
void usart_232_init(void);							//����1��ʼ������
void RS232_Send_Data(u8 *buf,u8 len);			//RS232�������ݺ���
void RS232_Receive_Data(u8 *buf,u8 *len);		//RS232�������ݺ���
void bytetofloat(u8 *buf,float *Repara,u8 len);	//��������ת����4���ֽ������ݣ�4*8λ��ת��һ�����������ݣ�32λ��
float Hex_To_Decimal(u8 *Byte);			 		//ʮ������ת��Ϊ������
void FloatToByte(float floatNum,u8 *byteArry);  //��������ʮ������ת��
void StraightWalk_function(int m);						//���黺��
void DD_functions(void);						//�㶯����ģʽ���ͻ����·�ָ����λ������ָ���Ӧ���ܲ��� 
void CleanArmPathPlan_functions(void);					//��ϴ��·���滮����ģʽ���ͻ����·�ָ����λ������ָ���Ӧ���ܲ��� 
void BufStorage(u8 len);						//���黺��
void SendDataToPC(float state);					//���ͺ�������λ���������˸��ؽڵ�״̬ʵʱ���͸���λ��
void CleanArm_Postion(void); 					//��ϴ��ĩ�˵�ǰλ�òɼ�����
void TurnWalk_function(int n);

#endif
