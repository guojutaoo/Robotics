#ifndef _input_H
#define _input_H
#include "stm32f10x.h"
void input_init(void);
void RBURF02_init(void);
float RBURF02_distance(void);
//定义全局变量
extern u8 TIM5CH1_CAPTURE_STA; //输入捕获状态 
extern u16 TIM5CH1_CAPTURE_VAL;//输入捕获值

#endif 
