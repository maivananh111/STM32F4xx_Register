/*
 * RCC_F4xx.cpp
 *
 *  Created on: Oct 10, 2022
 *      Author: anh
 */
#include "PERIPH_USED.h"

#ifdef ENABLE_RCC

#include "RCC_F4xx.h"
#include "math.h"
#include "string.h"

volatile uint32_t Tick;
static RCC_Config_t *_conf;

static void (*_delay)(uint32_t ms);
static uint32_t (*_gettick)(void);


Result_t RCC_SystemClock_Init(RCC_Config_t *conf){
	Result_t res = {OKE};
	__IO uint32_t tmpreg;
	_conf = conf;

	Result_Init(&res, OKE, 0U, __FUNCTION__, __FILE__);

	if(!(RCC -> APB2ENR & RCC_APB2ENR_SYSCFGEN)) RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	if(_conf -> rcc_clocksource == HSI_CRYSTAL){
		RCC -> CR |= RCC_CR_HSION;
		res = WaitFlagTimeoutBasic(&(RCC -> CR), RCC_CR_HSIRDY, FLAG_SET, DEFAULT_TIMEOUT);
		if(!CheckResult(res)) {
			Set_Line(&res, __LINE__);
			return res;
		}
		RCC -> CR |= (_conf -> rcc_hsitrim << RCC_CR_HSITRIM_Pos);
	}
	else if(_conf -> rcc_clocksource == HSE_CRYSTAL){
		RCC -> CR |= RCC_CR_HSEON;
		res = WaitFlagTimeoutBasic(&(RCC -> CR), RCC_CR_HSERDY, FLAG_SET, DEFAULT_TIMEOUT);
		if(!CheckResult(res)) {
			Set_Line(&res, __LINE__);
			return res;
		}
	}

	if(_conf -> rcc_clockmux == PLLCLK){
		RCC -> CR &=~ RCC_CR_PLLON;
		res = WaitFlagTimeoutBasic(&(RCC -> CR), RCC_CR_PLLRDY, FLAG_RESET, DEFAULT_TIMEOUT);
		if(!CheckResult(res)) {
			Set_Line(&res, __LINE__);
			return res;
		}

		tmpreg = RCC -> CFGR;
		tmpreg &=~ (RCC_CFGR_HPRE_Msk | RCC_CFGR_PPRE1_Msk | RCC_CFGR_PPRE2_Msk);
		tmpreg |= (_conf -> rcc_ahbprescaler | _conf -> rcc_apb1prescaler | _conf -> rcc_apb2prescaler);
		RCC -> CFGR = tmpreg;

		tmpreg = RCC -> PLLCFGR;
		tmpreg &=~ (0xF << 28U);
		tmpreg &=~ (RCC_PLLCFGR_PLLM_Msk | RCC_PLLCFGR_PLLN_Msk | RCC_PLLCFGR_PLLQ_Msk | RCC_PLLCFGR_PLLQ_Msk);
		tmpreg |= (_conf -> rcc_pll.rcc_pllm << RCC_PLLCFGR_PLLM_Pos) | (_conf -> rcc_pll.rcc_plln << RCC_PLLCFGR_PLLN_Pos)
				| (((_conf -> rcc_pll.rcc_pllp >> 1U) - 1U) << RCC_PLLCFGR_PLLP_Pos) | (_conf -> rcc_pll.rcc_pllq << RCC_PLLCFGR_PLLQ_Pos);
		RCC -> PLLCFGR = tmpreg;

		if(_conf -> rcc_clocksource == HSI_CRYSTAL) RCC -> PLLCFGR &=~ RCC_PLLCFGR_PLLSRC;
		else if(_conf -> rcc_clocksource == HSE_CRYSTAL) RCC -> PLLCFGR |= RCC_PLLCFGR_PLLSRC;

		RCC -> CR |= RCC_CR_PLLON;
		res = WaitFlagTimeoutBasic(&(RCC -> CR), RCC_CR_PLLRDY, FLAG_SET, DEFAULT_TIMEOUT);
		if(!CheckResult(res)) {
			Set_Line(&res, __LINE__);
			return res;
		}


		tmpreg = RCC -> CFGR;
		tmpreg &=~ RCC_CFGR_SW_Msk;
		tmpreg |= RCC_CFGR_SW_PLL;
		RCC -> CFGR = tmpreg;
		res = WaitFlagTimeoutBasic(&(RCC -> CFGR), RCC_CFGR_SWS_PLL, FLAG_SET, DEFAULT_TIMEOUT);
		if(!CheckResult(res)) {
			Set_Line(&res, __LINE__);
			return res;
		}
	}
	else if(_conf -> rcc_clockmux == HSE){
		tmpreg = RCC -> CFGR;
		tmpreg &=~ RCC_CFGR_SW_Msk;
		tmpreg |= RCC_CFGR_SW_HSE;
		RCC -> CFGR = tmpreg;
		res = WaitFlagTimeoutBasic(&(RCC -> CFGR), RCC_CFGR_SWS_HSE, FLAG_SET, DEFAULT_TIMEOUT);
		if(!CheckResult(res)) {
			Set_Line(&res, __LINE__);
			return res;
		}
	}

	else if(_conf -> rcc_clockmux == HSI){
		tmpreg = RCC -> CFGR;
		tmpreg &=~ RCC_CFGR_SW_Msk;
		tmpreg |= RCC_CFGR_SW_HSI;
		RCC -> CFGR = tmpreg;
		res = WaitFlagTimeoutBasic(&(RCC -> CFGR), RCC_CFGR_SWS_HSI, FLAG_RESET, DEFAULT_TIMEOUT);
		if(!CheckResult(res)) {
			Set_Line(&res, __LINE__);
			return res;
		}
	}
	(void)tmpreg;

	SystemInit();
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / 1000U);
	return res;
}

#ifdef ENABLE_I2S
Result_t I2SClock_Init(void){
	Result_t res = {OKE};
	__IO uint32_t tmpreg;

	Result_Init(&res, OKE, 0U, __FUNCTION__, __FILE__);

	tmpreg = RCC -> PLLI2SCFGR;
	tmpreg &=~ (RCC_PLLI2SCFGR_PLLI2SN_Msk | RCC_PLLI2SCFGR_PLLI2SR_Msk);
	tmpreg |= (_conf -> PLL.PLLI2SN << RCC_PLLI2SCFGR_PLLI2SN_Pos) | (_conf -> PLL.PLLI2SR << RCC_PLLI2SCFGR_PLLI2SN_Pos);
	RCC -> PLLI2SCFGR = tmpreg;

	RCC -> CFGR &=~ RCC_CFGR_I2SSRC;

	RCC -> CR |= RCC_CR_PLLI2SON;
	res = WaitFlagTimeoutBasic(&(RCC -> CR), RCC_CR_PLLI2SRDY, FLAG_SET, DEFAULT_TIMEOUT);
	if(!CheckResult(res)) {
		Set_Line(&res, __LINE__);
		return res;
	}

	return res;
}
#endif

uint32_t RCC_GetBusFreq(PCLKBus_t Bus){
	switch(Bus){
		case SYSCLK:
			if(_conf -> rcc_clocksource == HSE_CRYSTAL){ // HSE.
				if(_conf -> rcc_clockmux == HSE) return (uint32_t)HSE_VALUE;
				else if(_conf -> rcc_clockmux == PLLCLK) return (uint32_t)(((HSE_VALUE / _conf -> rcc_pll.rcc_pllm) * _conf -> rcc_pll.rcc_plln) / _conf -> rcc_pll.rcc_pllp);
			}
			else{ // HSI.
				if(_conf -> rcc_clockmux == HSI) return (uint32_t)HSI_VALUE;
				else if(_conf -> rcc_clockmux == PLLCLK) return (uint32_t)(((HSE_VALUE / _conf -> rcc_pll.rcc_pllm) * _conf -> rcc_pll.rcc_plln) / _conf -> rcc_pll.rcc_pllp);
			}
		break;

		case AHB:
			return (uint32_t)SystemCoreClock;
		break;

		case APB1:
			return (uint32_t)(SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos]);
		break;

		case APB2:
			return (uint32_t)(SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos]);
		break;
		case QBUS:
			return (uint32_t)((SystemCoreClock / _conf -> rcc_pll.rcc_pllp) / _conf -> rcc_pll.rcc_pllq);
		break;

	}
	return 0;
}

#ifdef ENABLE_SYSTEMTICK
uint32_t RCC_GetTick(void){
	return Tick;
}

void RCC_Delay_ms(uint32_t delay_ms){
	uint32_t tickstart = gettick();
	uint32_t wait = delay_ms;

	if (wait < MAX_TICK) wait += 1UL;
	while ((gettick() - tickstart) < wait);
}
#endif

void delay_ms_Init(void (*delay)(uint32_t ms)){
	_delay = delay;
}

void delay_ms(uint32_t ms){
	_delay(ms);
}

void gettick_Init(uint32_t (*get_tick)(void)){
	_gettick = get_tick;
}

uint32_t gettick(void){
	return _gettick();
}

void STM_Restart(uint16_t Time){
	__NVIC_SystemReset();
}


#endif









