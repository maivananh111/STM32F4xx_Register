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
	USART_INTERRUPT_DMA,
} USART_Type;

typedef enum{
	USART_INTR_RX = 1,
	USART_INTR_TX,
	USART_INTR_TX_RX,
} USART_InterruptSelect;

typedef enum{
	USART_Event_NoEvent,
	USART_Event_TransmitComplete,
	USART_Event_ReceiveComplete,
	USART_Event_BufferOverFlow,
	USART_Event_IdleDetected,
	USART_Event_ReceiveEndChar,
	USART_Event_Error,
} USART_Event_t;

typedef enum{
	USART_Reception_Normal,
	USART_Reception_ToEndChar,
	USART_Reception_ToIdle,
} USART_Reception_t;

typedef struct{
	uint32_t usart_baudrate;
	USART_Type usart_type = USART_NORMAL_DMA;
	USART_InterruptSelect usart_interruptselect = USART_INTR_RX;
	uint32_t usart_interruptpriority = 0;
	GPIO_TypeDef *TxPort;
	uint16_t TxPin;
	GPIO_TypeDef *RxPort;
	uint16_t RxPin;
	DMA *TxDma = NULL;
	DMA *RxDma = NULL;
} USART_Config_t;

class USART {
	public:
		USART(USART_TypeDef *usart);
		void Init(USART_Config_t *conf);

		Result_t Event_Register_Handler(void (*Event_Callback)(void *param, USART_Event_t event), void *param);

		Result_t Transmit(uint8_t TxData);
		Result_t Transmit(uint8_t *TxData, uint16_t len);
		Result_t SendString(char *String);

		Result_t Receive(uint8_t *RxData, uint16_t len);
		Result_t Receive(uint8_t *RxData);
		uint8_t GetData(void);
		Result_t GetBuffer(uint8_t **DestBuffer);
		uint16_t GetDataLength(void);

		Result_t TransmitIT(uint8_t *TxData, uint16_t len);
		Result_t ReceiveIT(uint16_t BufferSize);
		Result_t ReceiveToEndCharIT(uint16_t BufferSize, char EndChar = '\0');
		Result_t StopReceiveIT(void);

		Result_t TransmitDMA(uint8_t *TxData, uint16_t Length);
		Result_t ReceiveDMA(uint8_t *RxData, uint16_t Length);

		Result_t ReceiveUntilIdleIT(uint16_t BufferSize);
		Result_t ReceiveUntilIdleDMA(uint16_t BufferSize);
		Result_t StopReceiveUntilIdleIT(void);
		Result_t StopReceiveUntilIdleDMA(void);

		Result_t Stop_DMA(void);

		USART_Config_t *GetConfig(void);

		USART_TypeDef *_usart;

		DMA *_TxDma, *_RxDma;

		uint8_t *RxBuffer = NULL;
		uint16_t RxLength, RxCount;
		char EndChar = '\0';
		USART_Reception_t Reception = USART_Reception_Normal;

		void *Parameter;
		void (*Event_Callback)(void *param, USART_Event_t event);

	private:

		USART_Config_t *_conf = NULL;


};


extern USART usart1;
extern USART usart2;
extern USART usart3;
extern USART uart4;
extern USART uart5;
extern USART usart6;

void USART_IRQ_Handler(USART *usart);

void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);
void UART4_IRQHandler(void);
void UART5_IRQHandler(void);
void USART6_IRQHandler(void);



#ifdef __cplusplus
}
#endif

#endif

#endif /* USART_F4XX_H_ */

