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
#include "TFT16BIT.h"
#include "SPIFLASH.h"

#ifdef __cplusplus
extern "C"{
#endif

extern PowerConfig_t pwr;
extern FlashMemConfig_t flash;
extern RCC_Config_t rcc;


extern USART_Config_t uart_log_conf;
extern USART uart_log;


extern DMA_Config_t fls_spi_TxDma_conf;
extern DMA fls_spi_TxDma;
extern SPI_Config_t fls_spi_conf;
extern SPI<uint8_t> fls_spi;
extern SPIFLASH spiflash;



void Periph_Initialize(void);
void AppLayer_Initialize(void);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H_ */
