/*
 * IT_F4xx.cpp
 *
 *  Created on: Oct 18, 2022
 *      Author: anh
 */

#include "IT_F4xx.h"
#include "PERIPH_USED.h"
#include "RCC_F4xx.h"
#include "USART_F4xx.h"
#include "DMA_F4xx.h"
#include "TIM_F4xx.h"


/* EXTERNAL INTERRUPT */
#define EXTI_LINE_INDEX 6U

static void EXTI_IRQHandler(uint16_t Pin);

void EXTI_Init(GPIO_TypeDef *Port, uint16_t Pin, EXTI_EdgeDetect_t Edge, uint32_t Priority){
	if(!(RCC -> APB2ENR & RCC_APB2ENR_SYSCFGEN)) RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	EXTI -> PR = (1U << Pin);

	uint8_t CRPos = 0;
	if(Pin < 4U) 					CRPos = 0;
	else if(Pin >= 4U && Pin < 8U)  CRPos = 1;
	else if(Pin >= 8U && Pin < 12U) CRPos = 2;
	else 							CRPos = 3;

	__IO uint32_t tmpreg = SYSCFG -> EXTICR[CRPos];

	tmpreg &=~ (0x0F << ((Pin - CRPos*4U) * 4U));
	tmpreg = (uint32_t)(((((uint32_t)Port & 0xFF00U) >> 8U) / 4U) << ((Pin - CRPos*4U) * 4U));

	SYSCFG -> EXTICR[CRPos] = tmpreg;

	if(Edge & EXTI_RTSR_TR0) EXTI -> RTSR |= (1U << Pin);
	if(Edge & EXTI_FTSR_TR0) EXTI -> FTSR |= (1U << Pin);

	EXTI -> IMR |= (1U << Pin);

	IRQn_Type IRQn;
	if(Pin < 5U) IRQn = (IRQn_Type)(Pin + EXTI_LINE_INDEX);
	else if(Pin >= 5U && Pin < 9U) IRQn = EXTI9_5_IRQn;
	else 						   IRQn = EXTI15_10_IRQn;
	__NVIC_SetPriority(IRQn, Priority);
	__NVIC_EnableIRQ(IRQn);

	EXTI -> PR = (1U << Pin);
}


static void EXTI_IRQHandler(uint16_t Pin){
	if(EXTI -> PR & (1U << Pin)){
		EXTI_Callback(Pin);
		EXTI -> PR = (1U << Pin);
	}
}






void NMI_Handler(void){
	STM_Restart(1);
}

void HardFault_Handler(void){
	STM_Restart(1);
}

void MemManage_Handler(void){
	STM_Restart(1);
}

void BusFault_Handler(void){
	STM_Restart(1);
}

void UsageFault_Handler(void){
	STM_Restart(1);
}

void DebugMon_Handler(void){
	STM_Restart(1);
}

__WEAK void EXTI_Callback(uint16_t Pin){}

#ifdef ENABLE_EXTI0
void EXTI0_IRQHandler(void){
	EXTI_IRQHandler(0);
}
#endif
#ifdef ENABLE_EXTI1
void EXTI1_IRQHandler(void){
	EXTI_IRQHandler(1);
}
#endif
#ifdef ENABLE_EXTI2
void EXTI2_IRQHandler(void){
	EXTI_IRQHandler(2);
}
#endif
#ifdef ENABLE_EXTI3
void EXTI3_IRQHandler(void){
	EXTI_IRQHandler(3);
}
#endif
#ifdef ENABLE_EXTI4
void EXTI4_IRQHandler(void){
	EXTI_IRQHandler(4);
}
#endif
#ifdef ENABLE_EXTI9_5
void EXTI9_5_IRQHandler(void){
	EXTI_IRQHandler(5);
	EXTI_IRQHandler(6);
	EXTI_IRQHandler(7);
	EXTI_IRQHandler(8);
	EXTI_IRQHandler(9);
}
#endif
#ifdef ENABLE_EXTI15_10
void EXTI15_10_IRQHandler(void){
	EXTI_IRQHandler(10);
	EXTI_IRQHandler(11);
	EXTI_IRQHandler(12);
	EXTI_IRQHandler(13);
	EXTI_IRQHandler(14);
	EXTI_IRQHandler(15);
}
#endif

// SYSTEM TICK TIMER INTERRUPT HANDLER.
#ifdef ENABLE_SYSTEMTICK
#include "RCC_F4xx.h"
void SysTick_Handler(void){
	Tick++;

}


/* DMA1 IRQ HANDLER */
#ifdef ENABLE_DMA1_STREAM0
void DMA1_Stream0_IRQHandler(void){
	DMA_IRQ_Handler(DMA1, DMA1_Stream0, &dma1_stream0);
}
#endif
#ifdef ENABLE_DMA1_STREAM1
void DMA1_Stream1_IRQHandler(void){
	DMA_IRQ_Handler(DMA1, DMA1_Stream1, &dma1_stream1);
}
#endif
#ifdef ENABLE_DMA1_STREAM2
void DMA1_Stream2_IRQHandler(void){
	DMA_IRQ_Handler(DMA1, DMA1_Stream2, &dma1_stream2);
}
#endif
#ifdef ENABLE_DMA1_STREAM3
void DMA1_Stream3_IRQHandler(void){
	DMA_IRQ_Handler(DMA1, DMA1_Stream3, &dma1_stream3);
}
#endif
#ifdef ENABLE_DMA1_STREAM4
void DMA1_Stream4_IRQHandler(void){
	DMA_IRQ_Handler(DMA1, DMA1_Stream4, &dma1_stream4);
}
#endif
#ifdef ENABLE_DMA1_STREAM5
void DMA1_Stream5_IRQHandler(void){
	DMA_IRQ_Handler(DMA1, DMA1_Stream5, &dma1_stream5);
}
#endif
#ifdef ENABLE_DMA1_STREAM6
void DMA1_Stream6_IRQHandler(void){
	DMA_IRQ_Handler(DMA1, DMA1_Stream6, &dma1_stream6);
}
#endif
#ifdef ENABLE_DMA1_STREAM7
void DMA1_Stream7_IRQHandler(void){
	DMA_IRQ_Handler(DMA1, DMA1_Stream7, &dma1_stream7);
}
#endif

/* DMA2 IRQ HANDLER */
#ifdef ENABLE_DMA2_STREAM0
void DMA2_Stream0_IRQHandler(void){
	DMA_IRQ_Handler(DMA2, DMA2_Stream0, &dma2_stream0);
}
#endif
#ifdef ENABLE_DMA2_STREAM1
void DMA2_Stream1_IRQHandler(void){
	DMA_IRQ_Handler(DMA2, DMA2_Stream1, &dma2_stream1);
}
#endif
#ifdef ENABLE_DMA2_STREAM2
void DMA2_Stream2_IRQHandler(void){
	DMA_IRQ_Handler(DMA2, DMA2_Stream2, &dma2_stream2);
}
#endif
#ifdef ENABLE_DMA2_STREAM3
void DMA2_Stream3_IRQHandler(void){
	DMA_IRQ_Handler(DMA2, DMA2_Stream3, &dma2_stream3);
}
#endif
#ifdef ENABLE_DMA2_STREAM4
void DMA2_Stream4_IRQHandler(void){
	DMA_IRQ_Handler(DMA2, DMA2_Stream4, &dma2_stream4);
}
#endif
#ifdef ENABLE_DMA2_STREAM5
void DMA2_Stream5_IRQHandler(void){
	DMA_IRQ_Handler(DMA2, DMA2_Stream5, &dma2_stream5);
}
#endif
#ifdef ENABLE_DMA2_STREAM6
void DMA2_Stream6_IRQHandler(void){
	DMA_IRQ_Handler(DMA2, DMA2_Stream6, &dma2_stream6);
}
#endif
#ifdef ENABLE_DMA2_STREAM7
void DMA2_Stream7_IRQHandler(void){
	DMA_IRQ_Handler(DMA2, DMA2_Stream7, &dma2_stream7);
}
#endif






#ifdef ENABLE_USART1
void USART1_IRQHandler(void){
	USART_IRQ_Handler(&usart1);
}
#endif
#ifdef ENABLE_USART2
void USART2_IRQHandler(void){
	USART_IRQ_Handler(&usart2);
}
#endif
#ifdef ENABLE_USART3
void USART3_IRQHandler(void){
	USART_IRQ_Handler(&usart3);
}
#endif
#ifdef ENABLE_UART4
void UART4_IRQHandler(void){
	USART_IRQ_Handler(&uart4);
}
#endif
#ifdef ENABLE_UART5
void UART5_IRQHandler(void){
	USART_IRQ_Handler(&uart5);
}
#endif
#ifdef ENABLE_USART6
void USART6_IRQHandler(void){
	USART_IRQ_Handler(&usart6);
}
#endif





#ifdef ENABLE_TIM1
#ifndef ENABLE_TIM9
void TIM1_BRK_TIM9_IRQHandler(void){
	TIM_IRQHandler(&tim1);
}
#endif
#ifndef ENABLE_TIM10
void TIM1_UP_TIM10_IRQHandler(void){
	TIM_IRQHandler(&tim1);
}
#endif
#ifndef ENABLE_TIM11
void TIM1_TRG_COM_TIM11_IRQHandler(void){
	TIM_IRQHandler(&tim1);
}
#endif
void TIM1_CC_IRQHandler(void){
	TIM_IRQHandler(&tim1);
}
#endif

#ifdef ENABLE_TIM2
void TIM2_IRQHandler(void){
	TIM_IRQHandler(&tim2);
}
#endif

#ifdef ENABLE_TIM3
void TIM3_IRQHandler(void){
	TIM_IRQHandler(&tim3);
}
#endif

#ifdef ENABLE_TIM4
void TIM4_IRQHandler(void){
	TIM_IRQHandler(&tim4);
}
#endif

#ifdef ENABLE_TIM5
void TIM5_IRQHandler(void){
	TIM_IRQHandler(&tim5);
}
#endif

#ifdef ENABLE_TIM6
void TIM6_DAC_IRQHandler(void){
	TIM_IRQHandler(&tim6);
}
#endif

#ifdef ENABLE_TIM7
void TIM7_IRQHandler(void){
	TIM_IRQHandler(&tim7);
}
#endif

#ifdef ENABLE_TIM8
#ifndef ENABLE_TIM12
void TIM8_BRK_TIM12_IRQHandler(void){
	TIM_IRQHandler(&tim8);
}
#endif
#ifndef ENABLE_TIM13
void TIM8_UP_TIM13_IRQHandler(void){
	TIM_IRQHandler(&tim8);
}
#endif
#ifndef ENABLE_TIM14
void TIM8_TRG_COM_TIM14_IRQHandler(void){
	TIM_IRQHandler(&tim8);
}
#endif
void TIM8_CC_IRQHandler(void){
	TIM_IRQHandler(&tim8);
}
#endif

#ifdef ENABLE_TIM9
void TIM1_BRK_TIM9_IRQHandler(void){
#ifdef ENABLE_TIM1
	TIM_IRQHandler(&tim1);
#endif
	TIM_IRQHandler(&tim9);
}
#endif

#ifdef ENABLE_TIM10
void TIM1_UP_TIM10_IRQHandler(void){
#ifdef ENABLE_TIM1
	TIM_IRQHandler(&tim1);
#endif
	TIM_IRQHandler(&tim10);
}
#endif

#ifdef ENABLE_TIM11
void TIM1_TRG_COM_TIM11_IRQHandler(void){
#ifdef ENABLE_TIM1
	TIM_IRQHandler(&tim1);
#endif
	TIM_IRQHandler(&tim11);
}
#endif

#ifdef ENABLE_TIM12
void TIM8_BRK_TIM12_IRQHandler(void){
#ifdef ENABLE_TIM8
	TIM_IRQHandler(&tim8);
#endif
	TIM_IRQHandler(&tim12);
}
#endif

#ifdef ENABLE_TIM13
void TIM8_UP_TIM13_IRQHandler(void){
#ifdef ENABLE_TIM8
	TIM_IRQHandler(&tim8);
#endif
	TIM_IRQHandler(&tim13);
}
#endif

#ifdef ENABLE_TIM14
void TIM8_TRG_COM_TIM14_IRQHandler(void){
#ifdef ENABLE_TIM8
	TIM_IRQHandler(&tim8);
#endif
	TIM_IRQHandler(&tim14);
}
#endif

#endif
