/*
 * RCC_F4xx.h
 *
 *  Created on: Oct 10, 2022
 *      Author: anh
 */

#ifndef RCC_F4XX_H_
#define RCC_F4XX_H_

#include "PERIPH_USED.h"

#ifdef ENABLE_RCC

#include "stdio.h"
#include "stm32f4xx.h"
#include "PERIPH_STATUS.h"

#ifdef __cplusplus
 extern "C" {
#endif

#if !defined  (HSE_VALUE)
	#define HSE_VALUE 8000000U
	#define HSE_VALUE_MAX 25000000U
#endif

#if !defined  (HSI_VALUE)
	#define HSI_VALUE 16000000U
#endif

#define MAX_TICK 0xFFFFFFFFU

#define DEBUG_DISABLE_JTAG

typedef enum {
	HSI_CRYSTAL,
	HSE_CRYSTAL,
} SysClockSource_t;

typedef enum {
	HSI = RCC_CFGR_SW_HSI,
	HSE = RCC_CFGR_SW_HSE,
	PLLCLK = RCC_CFGR_SW_PLL,
} SysClockMux_t;

typedef enum {
	SYSCLK,
	AHB,
	APB1,
	APB2,
	QBUS,
} PCLKBus_t;


typedef struct{
	SysClockSource_t rcc_clocksource;
	uint32_t         rcc_hsifrequency;
	uint32_t 		 rcc_hsitrim;
	uint32_t 		 rcc_hsefrequency;
	SysClockMux_t 	 rcc_clockmux;
	uint32_t 		 rcc_systemfrequency;
	uint32_t 		 rcc_ahbprescaler;
	uint32_t 		 rcc_apb1prescaler;
	uint32_t 		 rcc_apb2prescaler;
	struct PLLn{
		uint32_t rcc_pllm;
		uint32_t rcc_plln;
		uint32_t rcc_pllp;
		uint32_t rcc_pllq;
	} rcc_pll;
} RCC_Config_t;

extern volatile uint32_t Tick;

Result_t RCC_SystemClock_Init(RCC_Config_t *conf);

#ifdef ENABLE_I2S
Result_t I2SClock_Init(void);
#endif

uint32_t RCC_GetBusFreq(PCLKBus_t Bus);
#ifdef ENABLE_SYSTEMTICK
uint32_t RCC_GetTick(void);
void RCC_Delay_ms(uint32_t delay_ms);
#endif

void delay_ms_Init(void (*delay)(uint32_t ms));
void delay_ms(uint32_t ms);
void gettick_Init(uint32_t (*get_tick)(void));
uint32_t gettick(void);

void STM_Restart(uint16_t Time);


#ifdef __cplusplus
}
#endif

#endif

#endif /* RCC_F4XX_H_ */
