/*
 * ESP32_API.h
 *
 *  Created on: Dec 13, 2022
 *      Author: anh
 */

#ifndef ESP32_API_H_
#define ESP32_API_H_

#include "stm32f4xx.h"
#include "USART_F4xx.h"
#include "GPIO_F4xx.h"
#include "RCC_F4xx.h"


#define NODATA        "*"
#define CMD_WIFI_ON   "WIFI ON"
#define CMD_WIFI_OFF  "WIFI OFF"
#define CMD_ETH_ON    "ETH ON"
#define CMD_ETH_OFF   "ETH OFF"

#define CMD_ON        "ON"
#define CMD_OFF       "OFF"

#define CMD_GET_DATA  "GETDATA"
#define CMD_POST_DATA "UPDATE"

// PC4-PC5
void ESP32_SendCommand(const char *cmd, const char *format, ...);

void ESP32_API_Init(USART *usart, GPIO_TypeDef *rst1_port, uint16_t rst1_pin, GPIO_TypeDef *rst2_port, uint16_t rst2_pin);
void ESP32_Hardware_Reset(void);

void ESP32_WiFi_On(char *SSID, char *PASS);
void ESP32_WiFi_Off(void);
void ESP32_Ethernet_On(void);
void ESP32_Ethernet_Off(void);




#endif /* ESP32_API_H_ */
