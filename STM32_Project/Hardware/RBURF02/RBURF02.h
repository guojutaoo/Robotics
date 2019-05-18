#ifndef _RBURF02_H
#define _RBURF02_H
#include "stm32f10x.h"
void RBURF02_input_init(void);

//定义全局变量
extern u8 TIM5CH1_CAPTURE_STA; //输入捕获状态 
extern u16 TIM5CH1_CAPTURE_VAL;//输入捕获值

#endif 
