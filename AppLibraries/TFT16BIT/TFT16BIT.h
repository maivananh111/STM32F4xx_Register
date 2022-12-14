/*
 * TFT16BIT.h
 *
 *  Created on: Nov 2, 2022
 *      Author: anh
 */

#ifndef TFT16BIT_H_
#define TFT16BIT_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "stdlib.h"
#include "stm32f4xx.h"

#define LCD_CS_PORT    GPIOD  //Chip Select
#define LCD_CD_PORT    GPIOD  //Command/Data
#define LCD_WR_PORT    GPIOD  //LCD Write
#define LCD_RD_PORT    GPIOD  //LCD Read
#define LCD_RST_PORT   GPIOB  //LCD Reset

#define LCD_CS    7  //Chip Select
#define LCD_CD    13  //Command/Data
#define LCD_WR    5  //LCD Write
#define LCD_RD    4  //LCD Read
#define LCD_RST   2  //LCD Reset

#define CS_IDLE      LCD_CS_PORT  -> BSRR    |= (1<<LCD_CS)
#define CD_DATA      LCD_CD_PORT  -> BSRR    |= (1<<LCD_CD)
#define WR_IDLE      LCD_WR_PORT  -> BSRR    |= (1<<LCD_WR)
#define RD_IDLE      LCD_RD_PORT  -> BSRR    |= (1<<LCD_RD)
#define RST_IDLE     LCD_RST_PORT -> BSRR    |= (1<<LCD_RST)

#define CS_ACTIVE    LCD_CS_PORT  -> BSRR    |= (1<<LCD_CS)<<16
#define CD_COMMAND   LCD_CD_PORT  -> BSRR    |= (1<<LCD_CD)<<16
#define WR_ACTIVE    LCD_WR_PORT  -> BSRR    |= (1<<LCD_WR)<<16
#define RD_ACTIVE    LCD_RD_PORT  -> BSRR    |= (1<<LCD_RD)<<16
#define RST_ACTIVE   LCD_RST_PORT -> BSRR    |= (1<<LCD_RST)<<16

#define WR_STROBE    {WR_ACTIVE; WR_IDLE;}

#define DATAPORTINIT {\
	GPIOD -> MODER &= 0x0FC0FFF0U;\
	GPIOD -> MODER |= 0x50150005U;\
	GPIOE -> MODER &= 0x00003FFFU;\
	GPIOE -> MODER |= 0x55554000U;\
	\
	GPIOD -> OTYPER &= 0x000038FCU;\
	GPIOE -> OTYPER &= 0x0000007FU;\
	\
	GPIOD -> OSPEEDR |= 0xF03F000FU;\
	GPIOE -> OSPEEDR |= 0xFFFFC000U;\
	\
	GPIOD -> PUPDR |= 0x50150005U;\
	GPIOE -> PUPDR |= 0x55554000U;\
}

#define DATAWRITEDIR {\
	GPIOD -> MODER |= 0x50150005U;\
	GPIOE -> MODER |= 0x55554000U;\
}

#define DATAREADDIR {\
	GPIOD -> MODER &= 0x0FC0FFF0U;\
	GPIOE -> MODER &= 0x00003FFFU;\
}

#define TFT_WRITE(data) {\
	GPIOD -> BSRR |= (0x0000C703U << 16);\
	GPIOE -> BSRR |= (0x0000FF80U << 16);\
	\
	GPIOD -> BSRR |= ((data&0xE000U) >> 5U) | ((data&0x0CU) >> 2U) | ((data&0x03U) << 14U);\
	GPIOE -> BSRR |= ((data&0x1FF0U) << 3U);\
}




typedef enum{
	TFT_Portrait,
	TFT_LandScape,
	TFT_PortraitMirror,
	TFT_LandScapeMirror,
} TFT_Orientation;


void setReadDir(void);
void setWriteDir(void);

void TFT_Init(uint16_t width, uint16_t height);
uint16_t TFT_ReadDriverID(void);

void TFT_SetOrientation(TFT_Orientation rotation);
void TFT_SetWindow(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1);

void TFT_FillScreen(uint16_t color);
void TFT_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

void TFT_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void TFT_DrawLine(uint16_t color,uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

void TFT_PushData(uint16_t *data, uint32_t len);


#define  BLACK   0x0000
#define  BLUE    0x001F
#define  RED     0x0F800
#define  GREEN   0x07E0
#define  CYAN    0x07FF
#define  MAGENTA 0xF81F
#define  YELLOW  0xFFE0
#define  WHITE   0xFFFF

#define TFT_SOFTRESET			0x01
#define TFT_MEMCONTROL			0x36
#define TFT_COLADDRSET			0x2A
#define TFT_PAGEADDRSET			0x2B
#define TFT_MEMORYWRITE			0x2C


#define ILI9341_MADCTL_MY			0x80
#define ILI9341_MADCTL_MX			0x40
#define ILI9341_MADCTL_MV			0x20
#define ILI9341_MADCTL_ML			0x10
#define ILI9341_MADCTL_RGB			0x00
#define ILI9341_MADCTL_BGR			0x08
#define ILI9341_MADCTL_MH			0x04

#ifdef __cplusplus
}
#endif

#endif /* TFT16BIT_H_ */
