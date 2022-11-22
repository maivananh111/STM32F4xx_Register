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
RCC_Config_t *_conf;


Result_t HSClock_Init(RCC_Config_t *conf){
	Result_t res = {OKE};
	__IO uint32_t tmpreg;
	_conf = conf;

	Set_Result(&res, __LINE__, __FUNCTION__, __FILE__);

	if(!(RCC -> APB2ENR & RCC_APB2ENR_SYSCFGEN)) RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	if(_conf -> CLOCK_SOURCE == HSI_CRYSTAL){
		RCC -> CR |= RCC_CR_HSION;
//		while(!(RCC -> CR & RCC_CR_HSIRDY));
		res = WaitFlagTimeoutBasic(&(RCC -> CR), RCC_CR_HSIRDY, FLAG_SET, DEFAULT_TIMEOUT);
		if(!CheckResult(res)) {Set_Result(&res, __LINE__, __FUNCTION__, __FILE__); return res;}
		RCC -> CR |= (_conf -> HSI_TRIM_VALUE << RCC_CR_HSITRIM_Pos);
	}
	else if(_conf -> CLOCK_SOURCE == HSE_CRYSTAL){
		RCC -> CR |= RCC_CR_HSEON;
//		while(!(RCC -> CR & RCC_CR_HSERDY));
		res = WaitFlagTimeoutBasic(&(RCC -> CR), RCC_CR_HSERDY, FLAG_SET, DEFAULT_TIMEOUT);
		if(!CheckResult(res)) {Set_Result(&res, __LINE__, __FUNCTION__, __FILE__); return res;}
	}

	if(_conf -> CLOCK_MUX == PLLCLK){
		RCC -> CR &=~ RCC_CR_PLLON;
//		while((RCC -> CR & RCC_CR_PLLRDY));
		res = WaitFlagTimeoutBasic(&(RCC -> CR), RCC_CR_PLLRDY, FLAG_RESET, DEFAULT_TIMEOUT);
		if(res.Status != OKE) {res.CodeLine = __LINE__; return res;}

		tmpreg = RCC -> CFGR;
		tmpreg &=~ (RCC_CFGR_HPRE_Msk | RCC_CFGR_PPRE1_Msk | RCC_CFGR_PPRE2_Msk);
		tmpreg |= (_conf -> AHB_PRESCALER | _conf -> APB1_PRESCALER | _conf -> APB2_PRESCALER);
		RCC -> CFGR = tmpreg;

		tmpreg = RCC -> PLLCFGR;
		tmpreg &=~ (0xF << 28U);
		tmpreg &=~ (RCC_PLLCFGR_PLLM_Msk | RCC_PLLCFGR_PLLN_Msk | RCC_PLLCFGR_PLLQ_Msk | RCC_PLLCFGR_PLLQ_Msk);
		tmpreg |= (_conf -> PLL.PLLM << RCC_PLLCFGR_PLLM_Pos) | (_conf -> PLL.PLLN << RCC_PLLCFGR_PLLN_Pos)
				| (((_conf -> PLL.PLLP >> 1U) - 1U) << RCC_PLLCFGR_PLLP_Pos) | (_conf -> PLL.PLLQ << RCC_PLLCFGR_PLLQ_Pos);
		RCC -> PLLCFGR = tmpreg;

		if(_conf -> CLOCK_SOURCE == HSI_CRYSTAL) RCC -> PLLCFGR &=~ RCC_PLLCFGR_PLLSRC;
		else if(_conf -> CLOCK_SOURCE == HSE_CRYSTAL) RCC -> PLLCFGR |= RCC_PLLCFGR_PLLSRC;

		RCC -> CR |= RCC_CR_PLLON;
//		while(!(RCC -> CR & RCC_CR_PLLRDY));
		res = WaitFlagTimeoutBasic(&(RCC -> CR), RCC_CR_PLLRDY, FLAG_SET, DEFAULT_TIMEOUT);
		if(!CheckResult(res)) {Set_Result(&res, __LINE__, __FUNCTION__, __FILE__); return res;}


		tmpreg = RCC -> CFGR;
		tmpreg &=~ RCC_CFGR_SW_Msk;
		tmpreg |= RCC_CFGR_SW_PLL;
		RCC -> CFGR = tmpreg;
//		while(!(RCC -> CFGR & RCC_CFGR_SWS_PLL));
		res = WaitFlagTimeoutBasic(&(RCC -> CFGR), RCC_CFGR_SWS_PLL, FLAG_SET, DEFAULT_TIMEOUT);
		if(!CheckResult(res)) {Set_Result(&res, __LINE__, __FUNCTION__, __FILE__); return res;}
	}
	else if(_conf -> CLOCK_MUX == HSE){
		tmpreg = RCC -> CFGR;
		tmpreg &=~ RCC_CFGR_SW_Msk;
		tmpreg |= RCC_CFGR_SW_HSE;
		RCC -> CFGR = tmpreg;
		res = WaitFlagTimeoutBasic(&(RCC -> CFGR), RCC_CFGR_SWS_HSE, FLAG_SET, DEFAULT_TIMEOUT);
		if(!CheckResult(res)) {Set_Result(&res, __LINE__, __FUNCTION__, __FILE__); return res;}
	}

	else if(_conf -> CLOCK_MUX == HSI){
		tmpreg = RCC -> CFGR;
		tmpreg &=~ RCC_CFGR_SW_Msk;
		tmpreg |= RCC_CFGR_SW_HSI;
		RCC -> CFGR = tmpreg;
		res = WaitFlagTimeoutBasic(&(RCC -> CFGR), RCC_CFGR_SWS_HSI, FLAG_RESET, DEFAULT_TIMEOUT);
		if(!CheckResult(res)) {Set_Result(&res, __LINE__, __FUNCTION__, __FILE__); return res;}
	}
	(void)tmpreg;

	SystemInit();
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / 1000U);
	return res;
}


Result_t I2SClock_Init(void){
	Result_t res = {OKE};
	__IO uint32_t tmpreg;

	Set_Result(&res, 0U, __FUNCTION__, __FILE__);

	tmpreg = RCC -> PLLI2SCFGR;
	tmpreg &=~ (RCC_PLLI2SCFGR_PLLI2SN_Msk | RCC_PLLI2SCFGR_PLLI2SR_Msk);
	tmpreg |= (_conf -> PLL.PLLI2SN << RCC_PLLI2SCFGR_PLLI2SN_Pos) | (_conf -> PLL.PLLI2SR << RCC_PLLI2SCFGR_PLLI2SN_Pos);
	RCC -> PLLI2SCFGR = tmpreg;

	RCC -> CFGR &=~ RCC_CFGR_I2SSRC;

	RCC -> CR |= RCC_CR_PLLI2SON;
	res = WaitFlagTimeoutBasic(&(RCC -> CR), RCC_CR_PLLI2SRDY, FLAG_SET, DEFAULT_TIMEOUT);
	if(!CheckResult(res)) {Set_Result(&res, __LINE__, __FUNCTION__, __FILE__); return res;}

	return res;
}

uint32_t GetBusFreq(PCLKBus_t Bus){
	switch(Bus){
		case SYSCLK:
			if(_conf -> CLOCK_SOURCE == HSE_CRYSTAL){ // HSE.
				if(_conf -> CLOCK_MUX == HSE) return (uint32_t)HSE_VALUE;
				else if(_conf -> CLOCK_MUX == PLLCLK) return (uint32_t)(((HSE_VALUE / _conf -> PLL.PLLM) * _conf -> PLL.PLLN) / _conf -> PLL.PLLP);
			}
			else{ // HSI.
				if(_conf -> CLOCK_MUX == HSI) return (uint32_t)HSI_VALUE;
				else if(_conf -> CLOCK_MUX == PLLCLK) return (uint32_t)(((HSE_VALUE / _conf -> PLL.PLLM) * _conf -> PLL.PLLN) / _conf -> PLL.PLLP);
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
	}
	return 0;
}

#ifdef ENABLE_SYSTEMTICK
uint32_t GetTick(void){
	return Tick;
}

void TickDelay_ms(uint32_t delay_ms){
	uint32_t tickstart = GetTick();
	uint32_t wait = delay_ms;

	if (wait < MAX_TICK) wait += 1UL;
	while ((GetTick() - tickstart) < wait);
}
#endif

void STM_Restart(uint16_t Time){
	__NVIC_SystemReset();
}


#endif









