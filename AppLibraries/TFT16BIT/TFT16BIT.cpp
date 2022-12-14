/*
 * TFT16BIT.cpp
 *
 *  Created on: Nov 2, 2022
 *      Author: anh
 */

#include "TFT16BIT.h"
#include "RCC_F4xx.h"



static uint16_t WIDTH  = 0, width = 0;
static uint16_t HEIGHT = 0, height = 0;
static uint8_t TFT_PortraitConfig = ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR;
static uint8_t TFT_LandscapeConfig = ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR;
static uint8_t TFT_PortraitMirrorConfig = ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR;
static uint8_t TFT_LandscapeMirrorConfig = ILI9341_MADCTL_MX | ILI9341_MADCTL_MY | ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR;

static void TFT_CtrlPinInit(void);

static void TFT_Write16(uint16_t data);

static void TFT_SendCommand(uint8_t cmd);
static void TFT_SendData(uint8_t data);

static void TFT_SoftReset(void);
static void TFT_HardReset(void);


static void TFT_CtrlPinInit(void){
	/* MODER */
	LCD_CS_PORT -> MODER  |= (1<<(LCD_CS*2));
	LCD_CD_PORT -> MODER  |= (1<<(LCD_CD*2));
	LCD_WR_PORT -> MODER  |= (1<<(LCD_WR*2));
	LCD_RD_PORT -> MODER  |= (1<<(LCD_RD*2));
	LCD_RST_PORT -> MODER |= (1<<(LCD_RST*2));
	/* OTYPER */
	LCD_CS_PORT -> OTYPER  &=~ (1<<(LCD_CS));
	LCD_CD_PORT -> OTYPER  &=~ (1<<(LCD_CD));
	LCD_WR_PORT -> OTYPER  &=~ (1<<(LCD_WR));
	LCD_RD_PORT -> OTYPER  &=~ (1<<(LCD_RD));
	LCD_RST_PORT -> OTYPER &=~ (1<<(LCD_RST));
	/* OSPEEDR */
	LCD_CS_PORT -> OSPEEDR  |= (3<<(LCD_CS*2));
	LCD_CD_PORT -> OSPEEDR  |= (3<<(LCD_CD*2));
	LCD_WR_PORT -> OSPEEDR  |= (3<<(LCD_WR*2));
	LCD_RD_PORT -> OSPEEDR  |= (3<<(LCD_RD*2));
	LCD_RST_PORT -> OSPEEDR |= (3<<(LCD_RST*2));
	/* PUPDR */
	LCD_CS_PORT -> PUPDR  &=~ (3<<(LCD_CS*2));
	LCD_CD_PORT -> PUPDR  &=~ (3<<(LCD_CD*2));
	LCD_WR_PORT -> PUPDR  &=~ (3<<(LCD_WR*2));
	LCD_RD_PORT -> PUPDR  &=~ (3<<(LCD_RD*2));
	LCD_RST_PORT -> PUPDR &=~ (3<<(LCD_RST*2));
}
/*
static void DATAWRITEDIR(void) {
	GPIOD -> MODER |= 0x50150005U;
	GPIOE -> MODER |= 0x55554000U;
}

static void DATAREADDIR(void) {
	GPIOD -> MODER &= 0x0FC0FFF0U;
	GPIOE -> MODER &= 0x00003FFFU;
}

static void TFT_WRITE(uint16_t data) {
	GPIOD -> BSRR |= (0x0000C703U << 16);
	GPIOE -> BSRR |= (0x0000FF80U << 16);

	GPIOD -> BSRR |= ((data&0xE000U) >> 5U) | ((data&0x0CU) >> 2U) | ((data&0x03U) << 14U);
	GPIOE -> BSRR |= ((data&0x1FF0U) << 3U);
}
*/
//--------------------------------------------------------------
static void TFT_SendCommand(uint8_t cmd){
	CD_COMMAND;//????? ? ????????? ??????? ???????
	CS_ACTIVE;//????? ???????
	TFT_WRITE(cmd);
	WR_STROBE;
	CD_DATA;
	CS_IDLE;
}

//--------------------------------------------------------------
static void TFT_SendData(uint8_t data){
	CS_ACTIVE;
	TFT_WRITE(data);
	WR_STROBE;
	CS_IDLE;
}

//--------------------------------------------------------------
static void TFT_Write16(uint16_t data){
	TFT_WRITE(data);
	WR_STROBE;
}
//--------------------------------------------------------------
static void TFT_SoftReset(void){
	TFT_SendCommand(TFT_SOFTRESET);
	delay_ms(50);
}

static void TFT_HardReset(void){
	RST_ACTIVE;
	delay_ms(100);
	RST_IDLE;
}

void TFT_SetWindow(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1){
	TFT_SendCommand(TFT_COLADDRSET);
	TFT_SendData((x0 >> 8) & 0xFF);
	TFT_SendData(x0 & 0xFF);
	TFT_SendData((x1 >> 8) & 0xFF);
	TFT_SendData(x1 & 0xFF);
	TFT_SendCommand(TFT_PAGEADDRSET);
	TFT_SendData((y0 >> 8) & 0xFF);
	TFT_SendData(y0 & 0xFF);
	TFT_SendData((y1 >> 8) & 0xFF);
	TFT_SendData(y1 & 0xFF);
	TFT_SendCommand(TFT_MEMORYWRITE);
}

void TFT_SetOrientation(TFT_Orientation rotation){
	TFT_SendCommand(TFT_MEMCONTROL);
	switch(rotation){
		case TFT_Portrait:
			TFT_SendData(TFT_PortraitConfig);
			width = WIDTH;
			height = HEIGHT;
		break;
		case TFT_PortraitMirror:
			TFT_SendData(TFT_PortraitMirrorConfig);
			width = WIDTH;
			height = HEIGHT;
		break;
		case TFT_LandScape:
			TFT_SendData(TFT_LandscapeConfig);
			width = HEIGHT;
			height = WIDTH;
		break;
		case TFT_LandScapeMirror:
			TFT_SendData(TFT_LandscapeMirrorConfig);
			width = HEIGHT;
			height = WIDTH;
		break;
	}
	TFT_SetWindow(0, 0, width - 1, height - 1);
}

void TFT_PushData(uint16_t *data, uint32_t len){

	CD_COMMAND;
	TFT_SendCommand(0x2C);
	CD_DATA;
	CS_ACTIVE;
	uint32_t numw = 0;
	while(numw++ < len) TFT_Write16(*data++);
	CS_IDLE;
}

void TFT_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color){
	TFT_SetWindow(x, y, x+w, y+h);
	TFT_SendCommand(0x2C);
	CS_ACTIVE;
	uint32_t size = w*h, i=0;
	while(i++ < size) TFT_Write16(color);
	CS_IDLE;
	TFT_SetWindow(0, 0, width - 1, height - 1);
}

void TFT_Init(uint16_t width, uint16_t height){
	WIDTH = width;
	HEIGHT = height;

	TFT_CtrlPinInit();
	DATAPORTINIT;
	RD_IDLE;

	TFT_HardReset();
	TFT_SoftReset();

	TFT_SendCommand(0x28);

	TFT_SendCommand(0xC0);
	TFT_SendData(0x10);
	TFT_SendData(0x10);

	TFT_SendCommand(0xC1);
	TFT_SendData(0x41);

	TFT_SendCommand(0xC5);
	TFT_SendData(0x00);
	TFT_SendData(0x22);
	TFT_SendData(0x80);
	TFT_SendData(0x40);

	TFT_SendCommand(0x36);
	TFT_SendData(0x68);

	TFT_SendCommand(0xB0);
	TFT_SendData(0x00);

	TFT_SendCommand(0xB1);
	TFT_SendData(0xB0);
	TFT_SendData(0x11);

	TFT_SendCommand(0xB4);
	TFT_SendData(0x02);

	TFT_SendCommand(0xB6);
	TFT_SendData(0x02);
	TFT_SendData(0x02);
	TFT_SendData(0x3B);


	TFT_SendCommand(0xB7);
	TFT_SendData(0xC6);

	TFT_SendCommand(0x3A);
	TFT_SendData(0x55);

	TFT_SendCommand(0xF7);
	TFT_SendData(0xA9);
	TFT_SendData(0x51);
	TFT_SendData(0x2C);
	TFT_SendData(0x82);

	TFT_SendCommand(0x11);
	delay_ms(100);
	TFT_SendCommand(0x29);
	delay_ms(100);
	TFT_SendCommand(0x2C);
}

uint16_t TFT_ReadDriverID(void){
	uint16_t id = 0;
//	lcdWriteCommand(ILI9341_READID4);
//	id = lcdReadData();
//	id = lcdReadData();
//	id = ((uint16_t) lcdReadData() << 8);
//	id |= lcdReadData();
	return id;
}








