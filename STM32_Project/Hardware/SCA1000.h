#ifndef _SCA1000_H
#define _SCA1000_H
#include "stm32f10x.h"
#include "sca.h"
/* 定义spi 使用的IO口 */
#define SPI_CS GPIO_Pin_13		//SPI_CS
#define SPI_MOSI GPIO_Pin_15		//SPI_MOSI
#define SPI_MISO GPIO_Pin_14		//SPI_MISO
#define SPI_CLK GPIO_Pin_13		//SPI_CLK

 
/* 定义SCA1000 使用的IO口 */
#define SCA1000_CS_SET GPIO_SetBits(GPIOG, GPIO_Pin_13)
#define SCA1000_CS_CLR GPIO_ResetBits(GPIOG, GPIO_Pin_13)

/* 定义指令表 */
#define RDAX    0x10   //读X轴角度控制指令
/* 声明外部调用函数 */
void SCA1000_Init(void);
void SCA1000_ReadData(void);
void SCA1000_Write(void);
void MNSPI_Init(void);
void MNSPI_WriteByte(uint8_t wdata);
u16 MNSPI_ReadBits( int n );
float MNSPI_GetX(void);
#endif
