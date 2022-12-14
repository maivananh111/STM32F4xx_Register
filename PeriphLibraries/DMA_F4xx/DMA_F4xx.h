/*
 * DMA_F4xx.h
 *
 *  Created on: Nov 11, 2022
 *      Author: anh
 */

#ifndef DMA_F4XX_H_
#define DMA_F4XX_H_

#include "PERIPH_USED.h"

#ifdef ENABLE_DMA

#include "stm32f4xx.h"
#include "PERIPH_STATUS.h"

#ifdef __cplusplus
extern "C"{
#endif


typedef enum{
	DMA_Channel0 = (0U),
	DMA_Channel1,
	DMA_Channel2,
	DMA_Channel3,
	DMA_Channel4,
	DMA_Channel5,
	DMA_Channel6,
	DMA_Channel7,
}DMA_Channel_t;

typedef enum{
	DMA_PERIH_TO_MEM = (0U),
	DMA_MEM_TO_PERIPH,
	DMA_MEM_TO_MEM,
}DMA_Direction_t;

typedef enum{
	DMA_NORMAL = (0U),
	DMA_CIRCULAR,
} DMA_Mode_t;

typedef enum{
	DMA_DATA8BIT,
	DMA_DATA16BIT,
	DMA_DATA32BIT,
} DMA_DataSize_t;

typedef enum{
	DMA_NOFIFO = (0U),
	DMA_FIF0 = DMA_SxFCR_DMDIS,
}DMA_FifoEn_t;

typedef enum{
	DMA_SINGLE_TRANFER = (0U),
	DMA_BURST_4INCREMENTAL,
	DMA_BURST_8INCREMENTAL,
	DMA_BURST_16INCREMENTAL,
} DMA_Burst_t;

typedef enum{
	DMA_CHANNEL_PRIORITY_LOW = (0U),
	DMA_CHANNEL_PRIORITY_MEDIUM,
	DMA_CHANNEL_PRIORITY_HIGH,
	DMA_CHANNEL_PRIORITY_VERYHIGH,
} DMA_ChannelPriority_t;

typedef enum{
	DMA_TRANSFER_COMPLETE_INTERRUPT = DMA_SxCR_TCIE,
	DMA_HALF_TRANSFER_INTERRUPT = DMA_SxCR_HTIE,
	DMA_TRANSFER_ERROR_INTERRUPT = DMA_SxCR_TEIE,
	DMA_DIRECTMODE_ERROR_INTERRUPT = DMA_SxCR_DMEIE,
} DMA_InterruptSelect_t;

typedef enum{
	DMA_Event_NoEvent,
	DMA_Event_Tranfer_Complete,
	DMA_Event_Half_Tranfer,
	DMA_Event_Tranfer_Error,
} DMA_Event_t;

typedef struct {
	DMA_TypeDef           *dma;
	DMA_Stream_TypeDef 	  *dma_stream;
	DMA_Channel_t 		  dma_channel;
	DMA_Direction_t 	  dma_direction = DMA_MEM_TO_PERIPH;
	DMA_Mode_t 	 		  dma_mode = DMA_NORMAL;
	DMA_DataSize_t 		  dma_data_size = DMA_DATA8BIT;
	DMA_FifoEn_t 		  dma_fifo = DMA_NOFIFO;
	DMA_Burst_t 		  dma_burst = DMA_BURST_4INCREMENTAL;
	DMA_ChannelPriority_t dma_channel_priority = DMA_CHANNEL_PRIORITY_HIGH;
	uint32_t 			  dma_interrupt_select = DMA_TRANSFER_COMPLETE_INTERRUPT | DMA_HALF_TRANSFER_INTERRUPT | DMA_TRANSFER_ERROR_INTERRUPT;
	uint32_t 			  dma_interrupt_priority = 0;
} DMA_Config_t;


class DMA {
	public:
		DMA(DMA_TypeDef *dma);
		Result_t Init(DMA_Config_t *conf);

		void Event_Register_Handler(void (*Event_Callback)(void *Parameter, DMA_Event_t event), void *Parameter);

		Result_t Start(uint32_t Src_Address, uint32_t Dest_Address, uint32_t Number_Data);
		Result_t Stop(void);

		uint16_t GetCounter(void);

		Result_t PollForTranfer(DMA_InterruptSelect_t PollLevel, uint32_t TimeOut);

		DMA_Config_t *GetConfig(void);

		void (*Event_Callback)(void *Parameter, DMA_Event_t event);
		void *Parameter;

	private:
		void ClearIFCR(__IO uint32_t Value);
		void ClearAllIntrFlag(void);
		__IO uint32_t GetISR(void);

		DMA_TypeDef *_dma;
		DMA_Config_t *_conf;
		Status_t _state = READY;
		IRQn_Type _IRQn = DMA1_Stream0_IRQn;
		__IO uint32_t _Intr_Index = 0U;
		uint32_t Stream = 0U;
		uint8_t DMA_Reg_Level = 0U; // 0 is LOW register.
		volatile uint32_t *ICFR = 0x00000000U;
		volatile uint32_t *ISR  = 0x00000000U;
};

extern DMA dma1_stream0;
extern DMA dma1_stream1;
extern DMA dma1_stream2;
extern DMA dma1_stream3;
extern DMA dma1_stream4;
extern DMA dma1_stream5;
extern DMA dma1_stream6;
extern DMA dma1_stream7;

extern DMA dma2_stream0;
extern DMA dma2_stream1;
extern DMA dma2_stream2;
extern DMA dma2_stream3;
extern DMA dma2_stream4;
extern DMA dma2_stream5;
extern DMA dma2_stream6;
extern DMA dma2_stream7;

void DMA_IRQ_Handler(DMA_TypeDef *dma, DMA_Stream_TypeDef *stream, DMA *dmaclass);

/* DMA1 IRQ HANDLER */
#ifdef ENABLE_DMA1_STREAM0
void DMA1_Stream0_IRQHandler(void);
#endif
#ifdef ENABLE_DMA1_STREAM1
void DMA1_Stream1_IRQHandler(void);
#endif
#ifdef ENABLE_DMA1_STREAM2
void DMA1_Stream2_IRQHandler(void);
#endif
#ifdef ENABLE_DMA1_STREAM3
void DMA1_Stream3_IRQHandler(void);
#endif
#ifdef ENABLE_DMA1_STREAM4
void DMA1_Stream4_IRQHandler(void);
#endif
#ifdef ENABLE_DMA1_STREAM5
void DMA1_Stream5_IRQHandler(void);
#endif
#ifdef ENABLE_DMA1_STREAM6
void DMA1_Stream6_IRQHandler(void);
#endif
#ifdef ENABLE_DMA1_STREAM7
void DMA1_Stream7_IRQHandler(void);
#endif

/* DMA2 IRQ HANDLER */
#ifdef ENABLE_DMA2_STREAM0
void DMA2_Stream0_IRQHandler(void);
#endif
#ifdef ENABLE_DMA2_STREAM1
void DMA2_Stream1_IRQHandler(void);
#endif
#ifdef ENABLE_DMA2_STREAM2
void DMA2_Stream2_IRQHandler(void);
#endif
#ifdef ENABLE_DMA2_STREAM3
void DMA2_Stream3_IRQHandler(void);
#endif
#ifdef ENABLE_DMA2_STREAM4
void DMA2_Stream4_IRQHandler(void);
#endif
#ifdef ENABLE_DMA2_STREAM5
void DMA2_Stream5_IRQHandler(void);
#endif
#ifdef ENABLE_DMA2_STREAM6
void DMA2_Stream6_IRQHandler(void);
#endif
#ifdef ENABLE_DMA2_STREAM7
void DMA2_Stream7_IRQHandler(void);
#endif
#ifdef __cplusplus
}
#endif

#endif

#endif /* DMA_F4XX_H_ */
