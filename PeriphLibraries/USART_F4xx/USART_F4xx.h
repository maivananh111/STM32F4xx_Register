/*
 * USART_F4xx.h
 *
 *  Created on: 29 thg 7, 2022
 *      Author: A315-56
 */

#ifndef USART_F4XX_H_
#define USART_F4XX_H_

#include "PERIPH_USED.h"

#ifdef ENABLE_USART


#include "stdio.h"
#include "stm32f4xx.h"
#include "PERIPH_STATUS.h"
#include "DMA_F4xx.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef enum{
	USART_NORMAL_DMA = 0,
	USART_INTERRUPT,
} USART_Type;

typedef enum{
	USART_INTR_RX = 1,
	USART_INTR_TX,
	USART_INTR_TX_RX,
} USART_InterruptSelect;

typedef struct{
	uint32_t Baudrate;
	USART_Type Type = USART_NORMAL_DMA;
	USART_InterruptSelect InterruptSelect = USART_INTR_RX;
	uint32_t InterruptPriority = 0;
	GPIO_TypeDef *Port;
	uint16_t TxPin;
	uint16_t RxPin;
	DMA *TxDma = NULL;
	DMA *RxDma = NULL;
} USART_Config_t;

class USART {
	public:
		USART(USART_TypeDef *usart);
		void Init(USART_Config_t *conf);

		Result_t Transmit(uint8_t Data);
		Result_t SendString(char *String);
		Result_t Receive(uint8_t *Data);

		Result_t TransmitDMA(uint8_t *TxData, uint16_t Length);
		Result_t ReceiveDMA(uint8_t *RxData, uint16_t Length);

		Result_t Stop_DMA(void);

		DMA *_TxDma, *_RxDma;


	private:
		USART_TypeDef *_usart;
		USART_Config_t *_conf = NULL;
};



#ifdef __cplusplus
}
#endif

#endif

#endif /* USART_F4XX_H_ */

