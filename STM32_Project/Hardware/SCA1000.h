#ifndef _SCA1000_H
#define _SCA1000_H
#include "stm32f10x.h"
#include "sca.h"
/* ����spi ʹ�õ�IO�� */
#define SPI_CS GPIO_Pin_13		//SPI_CS
#define SPI_MOSI GPIO_Pin_15		//SPI_MOSI
#define SPI_MISO GPIO_Pin_14		//SPI_MISO
#define SPI_CLK GPIO_Pin_13		//SPI_CLK

 
/* ����SCA1000 ʹ�õ�IO�� */
#define SCA1000_CS_SET GPIO_SetBits(GPIOG, GPIO_Pin_13)
#define SCA1000_CS_CLR GPIO_ResetBits(GPIOG, GPIO_Pin_13)

/* ����ָ��� */
#define RDAX    0x10   //��X��Ƕȿ���ָ��
/* �����ⲿ���ú��� */
void SCA1000_Init(void);
void SCA1000_ReadData(void);
void SCA1000_Write(void);
void MNSPI_Init(void);
void MNSPI_WriteByte(uint8_t wdata);
u16 MNSPI_ReadBits( int n );
float MNSPI_GetX(void);
#endif
