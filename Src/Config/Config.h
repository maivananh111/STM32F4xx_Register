/*
 * Config.h
 *
 *  Created on: Nov 7, 2022
 *      Author: anh
 */

#ifndef CONFIG_H_
#define CONFIG_H_


#include "PERIPH_USED.h"
#include "PERIPH_STATUS.h"
#include "SYSTEM_F4xx.h"
#include "STM_LOG.h"
#include "IT_F4xx.h"
#include "RCC_F4xx.h"
#include "DMA_F4xx.h"
#include "GPIO_F4xx.h"
#include "USART_F4xx.h"
#include "SPI_F4xx.hpp"
#include "I2C_F4xx.h"
#include "TFT16BIT.h"
#include "SPIFLASH.h"
#include "LORA.h"
#include "DS3231.h"

#ifdef __cplusplus
extern "C"{
#endif


/* LoRa have highest priority */
#define LORA_PRIORITY 10

#define UART_PRIORITY 3
#define UART_TXDMA_PRIORITY 1

#define SPIFLS_RXDMA_PRIORITY 5

extern PowerConfig_t pwr;
extern FlashMemConfig_t flash;
extern RCC_Config_t rcc;


extern DMA_Config_t uart_log_TxDma_conf;
extern DMA *uart_log_TxDma;
extern USART_Config_t uart_log_conf;
extern USART *uart_log;
extern volatile uint8_t dma_uart_log_tx_flag;

extern DMA_Config_t fls_spi_RxDma_conf;
extern DMA *fls_spi_RxDma;
extern SPI_Config_t fls_spi_conf;
extern SPI<uint8_t> fls_spi;
extern SPIFLASH spiflash;

extern I2C_Config_t i2c1_conf;
extern I2C i2c1;

extern SPI_Config_t lora_spi_conf;
extern SPI<uint8_t> lora_spi;
extern LoRa lora;

extern DS3231_Time_t time;
extern uint8_t *Rxbuf;

void Periph_Initialize(void);
void AppLayer_Initialize(void);


void DMA2_Stream0_TranferComplete_CallBack(void *Paramemter, DMA_Event_t event);
void DMA2_Stream7_TranferComplete_CallBack(void *Paramemter, DMA_Event_t event);

void USART1_Event_Callback(void *Parameter, USART_Event_t event);
void LoRaReceive(void *arg, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H_ */
