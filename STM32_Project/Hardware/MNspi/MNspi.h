#ifndef _MNspi_H
#define _MNspi_H

#include "stm32f10x.h"
#define MNspi_CLK GPIO_Pin_5
#define MNspi_MISO GPIO_Pin_15
#define MNspi_MOSI GPIO_Pin_8

#define LRDAX 0x10

#define LFSCA1000_CS1 GPIO_Pin_9
#define LFSCA1000_CS2 GPIO_Pin_14
#define RFSCA1000_CS1 GPIO_Pin_13
#define RFSCA1000_CS2 GPIO_Pin_12
#define LHSCA1000_CS1 GPIO_Pin_12
#define LHSCA1000_CS2 GPIO_Pin_15
#define RHSCA1000_CS1 GPIO_Pin_9
#define RHSCA1000_CS2 GPIO_Pin_11

#define LFSCA1000_CS1_SET GPIO_SetBits(GPIOB, GPIO_Pin_9)	
#define LFSCA1000_CS1_CLR GPIO_ResetBits(GPIOB, GPIO_Pin_9)
#define LFSCA1000_CS2_SET GPIO_SetBits(GPIOG, GPIO_Pin_14)	
#define LFSCA1000_CS2_CLR GPIO_ResetBits(GPIOG, GPIO_Pin_14)
#define RFSCA1000_CS1_SET GPIO_SetBits(GPIOC, GPIO_Pin_13)	
#define RFSCA1000_CS1_CLR GPIO_ResetBits(GPIOC, GPIO_Pin_13)
#define RFSCA1000_CS2_SET GPIO_SetBits(GPIOA, GPIO_Pin_12)	
#define RFSCA1000_CS2_CLR GPIO_ResetBits(GPIOA, GPIO_Pin_12)
#define LHSCA1000_CS1_SET GPIO_SetBits(GPIOB, GPIO_Pin_12)	
#define LHSCA1000_CS1_CLR GPIO_ResetBits(GPIOB, GPIO_Pin_12)
#define LHSCA1000_CS2_SET GPIO_SetBits(GPIOG, GPIO_Pin_15)	
#define LHSCA1000_CS2_CLR GPIO_ResetBits(GPIOG, GPIO_Pin_15)
#define RHSCA1000_CS1_SET GPIO_SetBits(GPIOC, GPIO_Pin_9)	
#define RHSCA1000_CS1_CLR GPIO_ResetBits(GPIOC, GPIO_Pin_9)
#define RHSCA1000_CS2_SET GPIO_SetBits(GPIOA, GPIO_Pin_11)	
#define RHSCA1000_CS2_CLR GPIO_ResetBits(GPIOA, GPIO_Pin_11)

void MNspi_init(void);
void MNspi_Stop(int n);
void MNspi_Reset(int n);
void MNspi_Start(int n);
void MNspi_WriteByte(u8 dat);
int	MNspi_ReadBits(int n);
int SPI_GetLXN(int n);
#endif

