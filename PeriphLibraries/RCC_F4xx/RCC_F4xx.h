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
} PCLKBus_t;


typedef struct{
	SysClockSource_t CLOCK_SOURCE;
	uint32_t HSI_CRYSTAL_FREQUENCY;
	uint32_t HSI_TRIM_VALUE;
	uint32_t HSE_CRYSTAL_FREQUENCY;
	SysClockMux_t CLOCK_MUX;
	uint32_t SYSTEM_FREQUENCY;
	uint32_t AHB_PRESCALER;
	uint32_t APB1_PRESCALER;
	uint32_t APB2_PRESCALER;
	struct PLLn{
		uint32_t PLLM;
		uint32_t PLLN;
		uint32_t PLLP;
		uint32_t PLLQ;
		uint32_t PLLI2SN;
		uint32_t PLLI2SR;
	} PLL;
} RCC_Config_t;

extern RCC_Config_t *rcc_configuration;
extern volatile uint32_t Tick;

Result_t HSClock_Init(RCC_Config_t *conf);
Result_t I2SClock_Init(void);

uint32_t GetBusFreq(PCLKBus_t Bus);
#ifdef ENABLE_SYSTEMTICK
uint32_t GetTick(void);
void TickDelay_ms(uint32_t delay_ms);
#endif

void STM_Restart(uint16_t Time);


#ifdef __cplusplus
}
#endif

#endif

#endif /* RCC_F4XX_H_ */
