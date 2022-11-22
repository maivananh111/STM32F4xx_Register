/*
 * SYSTEM_F4xx.cpp
 *
 *  Created on: Oct 12, 2022
 *      Author: anh
 */
#include "SYSTEM_F4xx.h"
#include "stdio.h"

FlashMemConfig_t *_FlashMem;
PowerConfig_t *_Power;
DebugMode_t _Mode;

void Power_Configuration(PowerConfig_t *pwr_conf){
	_Power = pwr_conf;
	/* ENABLE SYSCFG CLOCK */
	if(!(RCC -> APB2ENR & RCC_APB2ENR_SYSCFGEN)) RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	/* ENABLE POWER INTERFACE CLOCK */
	if(_Power -> ClockEnable == true) (RCC -> APB1ENR |= RCC_APB1ENR_PWREN);
	else  (RCC -> APB1ENR &=~ RCC_APB1ENR_PWREN);
	/* SET VOLT SCALE
	 *  - SCALE 2 FOR EVICE CLOCK BELOW MAX FREQUENCY */
	if(_Power -> VoltScale2) PWR -> CR &=~ PWR_CR_VOS;
	else{
		PWR -> CR |= PWR_CR_VOS;
		while(!(PWR -> CSR & PWR_CSR_VOSRDY));
	}

}

void FlashMem_Configuration(FlashMemConfig_t *flh_conf){
	_FlashMem = flh_conf;

	/* ENABLE SYSCFG CLOCK */
	if(!(RCC -> APB2ENR & RCC_APB2ENR_SYSCFGEN)) RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	__IO uint32_t tmpreg = FLASH -> ACR;
	/* SET FLASH LATENCY */
	tmpreg &=~ FLASH_ACR_LATENCY_Msk; // CLEAR ALL LATENCY BIT.
	uint32_t latency= (uint32_t)(_FlashMem -> System_Clock / 30000000U);
	if(_FlashMem -> System_Clock == 30000000U || _FlashMem -> System_Clock == 60000000U || _FlashMem -> System_Clock == 90000000U
    || _FlashMem -> System_Clock == 120000000U || _FlashMem -> System_Clock == 150000000U || _FlashMem -> System_Clock == 180000000U) latency -= 1;
	_FlashMem -> Latency = latency;
	tmpreg |= (uint32_t)(latency << FLASH_ACR_LATENCY_Pos);

	/* SET FLASH DATA CACHE */
	if(_FlashMem -> Data_Cache) tmpreg |= FLASH_ACR_DCEN;
	else tmpreg &=~ FLASH_ACR_DCEN;

	/* SET FLASH INSTUCTION CACHE */
	if(_FlashMem -> Instruction_Cache) tmpreg |= FLASH_ACR_ICEN;
	else tmpreg &=~ FLASH_ACR_ICEN;

	/* SET FLASH PREFETCH CACHE */
	if(_FlashMem -> Prefetch) tmpreg |= FLASH_ACR_PRFTEN;
	else tmpreg &=~ FLASH_ACR_PRFTEN;

	FLASH -> ACR = tmpreg;
}

void Update_Latency(void){
	/* SET FLASH LATENCY */
	uint32_t tmpreg = FLASH -> ACR;
	tmpreg &=~ FLASH_ACR_LATENCY_Msk; // CLEAR ALL LATENCY BIT.
	uint32_t latency= (uint32_t)(SystemCoreClock / 30000000U);
	if(SystemCoreClock == 30000000U || SystemCoreClock == 60000000U || SystemCoreClock == 90000000U
    || SystemCoreClock == 120000000U || SystemCoreClock == 150000000U || SystemCoreClock == 180000000U) latency -= 1;
	_FlashMem -> Latency = latency;
	tmpreg |= (uint32_t)(latency << FLASH_ACR_LATENCY_Pos);
	FLASH -> ACR |= tmpreg;
}





















