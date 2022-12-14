/*
 * ESP32_API.cpp
 *
 *  Created on: Dec 13, 2022
 *      Author: anh
 */

#include "ESP32_API.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdarg.h"
#include "string.h"


static GPIO_TypeDef *_rst1_port;
static uint16_t _rst1_pin;
static GPIO_TypeDef *_rst2_port;
static uint16_t _rst2_pin;

static USART *_usart;


void ESP32_SendCommand(const char *cmd, const char *format, ...){
	char *Temp_buffer = NULL;
	va_list args;
	va_start(args, format);

	uint16_t length = vsnprintf(Temp_buffer, 0, format, args);
	Temp_buffer = (char *)malloc(length * sizeof(char) + 1);
	vsprintf(Temp_buffer, format, args);
	va_end(args);

	char *Output_buffer = (char *)malloc((strlen(cmd) + 2 + length + 2) * sizeof(char));
	sprintf(Output_buffer, "%s: %s\r\n", cmd, Temp_buffer);

//	delay_ms(10);
	_usart -> SendString(Output_buffer);
	free(Temp_buffer);
	free(Output_buffer);
}

void ESP32_API_Init(USART *usart, GPIO_TypeDef *rst1_port, uint16_t rst1_pin, GPIO_TypeDef *rst2_port, uint16_t rst2_pin){
	_usart = usart;
	_rst1_port = rst1_port;
	_rst1_pin = rst1_pin;
	_rst2_port = rst2_port;
	_rst2_pin = rst2_pin;

	GPIO_PortClockEnable(_rst1_port);
	GPIO_PortClockEnable(_rst2_port);

	GPIO_Init(_rst1_port, _rst1_pin, GPIO_OUTPUT_PUSHPULL);
	GPIO_Init(_rst2_port, _rst2_pin, GPIO_OUTPUT_PUSHPULL);

	GPIO_Set(_rst1_port, _rst1_pin);
	GPIO_Set(_rst2_port, _rst2_pin);

}

void ESP32_Hardware_Reset(void){
	GPIO_Init(_rst1_port, _rst1_pin, GPIO_OUTPUT_PUSHPULL);
	GPIO_Init(_rst2_port, _rst2_pin, GPIO_OUTPUT_PUSHPULL);

	GPIO_Reset(_rst1_port, _rst1_pin);
	GPIO_Reset(_rst2_port, _rst2_pin);

	delay_ms(50);

	GPIO_Set(_rst1_port, _rst1_pin);
	GPIO_Set(_rst2_port, _rst2_pin);

	delay_ms(100);

	GPIO_Init(_rst1_port, _rst1_pin, GPIO_INPUT_PULLUP);
	GPIO_Init(_rst2_port, _rst2_pin, GPIO_INPUT_PULLUP);
}

void ESP32_WiFi_On(char *SSID, char *PASS){
	ESP32_SendCommand(CMD_WIFI_ON, "%s,%s", SSID, PASS);
}

void ESP32_WiFi_Off(void){
	ESP32_SendCommand(CMD_WIFI_OFF, NODATA);
}

void ESP32_Ethernet_Init(void){
	ESP32_SendCommand(CMD_ETH_ON, NODATA);
}

void ESP32_Ethernet_Off(void){
	ESP32_SendCommand(CMD_ETH_OFF, NODATA);
}















