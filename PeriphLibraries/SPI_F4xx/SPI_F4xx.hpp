/*
 * SPI_F4xx.hpp
 *
 *  Created on: Nov 12, 2022
 *      Author: anh
*/

#pragma once

#include "GPIO_F4xx.h"

#include "PERIPH_STATUS.h"
#include "STM_LOG.h"
#include "stm32f4xx.h"
#include "DMA_F4xx.h"


#define SPI_LOG_DEBUG_ENABLE

#define SPI_TIMEOUT 100UL

typedef enum{
	SPI_FULLDUPLEXMASTER = 0UL,
	SPI_HALFDUPLEXMASTER,
} SPI_Mode;

typedef enum{
	SPI_NORMAL_DMA = 0UL,
	SPI_INTERRUPT,
} SPI_Type;

typedef enum{
	SPI_INTR_RX = SPI_CR2_RXNEIE,
	SPI_INTR_TX = SPI_CR2_TXEIE,
	SPI_INTR_TX_RX = (SPI_CR2_RXNEIE | SPI_CR2_TXEIE),
} SPI_InterruptSelect;

typedef enum{
	SPI_DATA8BIT = 0x00UL,
	SPI_DATA16BIT = SPI_CR1_DFF,
} SPI_DataSize;

typedef enum{
	SPI_DATAMSB = 0x00UL,
	SPI_DATALSB = SPI_CR1_LSBFIRST,
} SPI_DataFormat;

typedef enum{
	SPI_CLOCKDIV2 = 0UL,
	SPI_CLOCKDIV4,
	SPI_CLOCKDIV8,
	SPI_CLOCKDIV16,
	SPI_CLOCKDIV32,
	SPI_CLOCKDIV64,
	SPI_CLOCKDIV128,
	SPI_CLOCKDIV256,
} SPI_ClockDiv;

typedef enum{
	SPI_CPOL0_CPHA0 = 0UL,
	SPI_CPOL0_CPHA1 = SPI_CR1_CPHA,
	SPI_CPOL1_CPHA0 = SPI_CR1_CPOL,
	SPI_CPOL1_CPHA1 = (SPI_CR1_CPOL | SPI_CR1_CPHA),
} SPI_ClockSample;

typedef struct {
	SPI_Mode 			spi_mode             = SPI_FULLDUPLEXMASTER;
	SPI_Type 			spi_type             = SPI_NORMAL_DMA;
	SPI_InterruptSelect spi_IinterruptSelect = SPI_INTR_RX;
	SPI_DataSize 		spi_datasize         = SPI_DATA8BIT;
	SPI_DataFormat 		spi_dataformat  	 = SPI_DATAMSB;
	SPI_ClockDiv 		spi_clockdivision    = SPI_CLOCKDIV4;
	SPI_ClockSample 	spi_clocksample 	 = SPI_CPOL0_CPHA0;
	uint32_t 			spi_interruptpriority = 0;
	IRQn_Type 			spi_irq;
	GPIO_TypeDef 		*CLKPort;
	uint16_t 			CLKPin;
	GPIO_TypeDef 		*MISOPort;
	uint16_t 			MISOPin;
	GPIO_TypeDef 		*MOSIPort;
	uint16_t 			MOSIPin;
	DMA 				*TxDma = NULL;
	DMA 				*RxDma = NULL;
} SPI_Config_t;


template <typename DataSize>
class SPI {
	public:
		SPI(SPI_TypeDef *Spi);
		void Init(SPI_Config_t *conf);

		Result_t Transmit(DataSize *TxData, uint32_t Data_Number);
		Result_t Receive(DataSize *RxData, uint32_t Data_Number);
		Result_t Transmit_Receive(DataSize *TxData, DataSize *RxData, uint32_t Data_Number);

		Result_t Transmit(DataSize TxData);
		Result_t Receive(DataSize *RxData);
		Result_t Transmit_Receive(DataSize TxData, DataSize *RxData);

		Result_t Transmit_DMA(DataSize *TxData, uint32_t Data_Number);
		Result_t Receive_DMA(DataSize *RxData, uint32_t Data_Number);
		Result_t Transmit_Receive_DMA(DataSize *TxData, DataSize *RxData, uint32_t Data_Number);

		Result_t Stop_DMA(void);

		DMA *_TxDma, *_RxDma;

	private:
		SPI_Config_t *_conf = NULL;
		SPI_TypeDef *_spi;


};


template <typename DataSize>
SPI<DataSize>::SPI(SPI_TypeDef *Spi){
	_spi = Spi;
}

template <typename DataSize>
void SPI<DataSize>::Init(SPI_Config_t *conf){
	_conf = conf;
	_TxDma = _conf->TxDma;
	_RxDma = _conf->RxDma;

	if     (_spi == SPI1) RCC -> APB2ENR |= RCC_APB2ENR_SPI1EN;
	else if(_spi == SPI2) RCC -> APB1ENR |= RCC_APB1ENR_SPI2EN;
	else if(_spi == SPI3) RCC -> APB1ENR |= RCC_APB1ENR_SPI3EN;

	GPIO_PortClockEnable(_conf -> CLKPort);
	GPIO_PortClockEnable(_conf -> MISOPort);
	GPIO_PortClockEnable(_conf -> MOSIPort);

	if(_spi == SPI1 || _spi == SPI2){
		GPIO_AlternateFunction(_conf -> CLKPort, _conf -> CLKPin, AF5_SPI1_2);
		GPIO_AlternateFunction(_conf -> MISOPort, _conf -> MISOPin, AF5_SPI1_2);
		GPIO_AlternateFunction(_conf -> MOSIPort, _conf -> MOSIPin, AF5_SPI1_2);
	}
	else{
		GPIO_AlternateFunction(_conf -> CLKPort, _conf -> CLKPin, AF6_SPI3);
		GPIO_AlternateFunction(_conf -> MISOPort, _conf -> MISOPin, AF6_SPI3);
		GPIO_AlternateFunction(_conf -> MOSIPort, _conf -> MOSIPin, AF6_SPI3);
	}

	_spi -> CR1 = 0x00UL;

	_spi -> CR1 &=~ SPI_CR1_SPE;
	_spi -> CR1 &=~ SPI_CR1_RXONLY;
	_spi -> CR1 |= _conf->spi_clocksample | SPI_CR1_MSTR | (_conf->spi_clockdivision << SPI_CR1_BR_Pos) | _conf->spi_dataformat;
	_spi -> CR1 |= _conf->spi_datasize | SPI_CR1_SSM | SPI_CR1_SSI;
	if(_conf->spi_mode == SPI_HALFDUPLEXMASTER) _spi -> CR1 |= SPI_CR1_BIDIMODE;

	_spi -> CR2 = 0x00UL;
	if(_conf->spi_type == SPI_INTERRUPT) _spi -> CR2 = _conf->spi_IinterruptSelect;

	if(_conf->spi_type == SPI_INTERRUPT){
		__NVIC_SetPriority(_conf->spi_irq, _conf->spi_interruptpriority);
		__NVIC_EnableIRQ(_conf->spi_irq);
	}

	_spi -> CR1 |= SPI_CR1_SPE;
}

template <typename DataSize>
Result_t SPI<DataSize>::Transmit(DataSize *TxData, uint32_t Data_Number){
	Result_t res = {OKE};
	Result_Init(&res, OKE, 0U, __FUNCTION__, __FILE__);
	uint32_t TxCount = Data_Number;

	if(_conf->spi_mode == SPI_HALFDUPLEXMASTER){
		_spi -> CR1 &=~ SPI_CR1_SPE;
		_spi -> CR1 |= SPI_CR1_BIDIOE;
	}

	if((_spi -> CR1 & SPI_CR1_SPE) != SPI_CR1_SPE) _spi -> CR1 |= SPI_CR1_SPE;

	while(TxCount--){
		res = WaitFlagTimeout(&(_spi -> SR), SPI_SR_TXE, FLAG_SET, SPI_TIMEOUT);
		if(!CheckResult(res)) {
			Set_Line(&res, __LINE__);
//			return res;
		}

		_spi -> DR = (uint32_t)(*TxData);

		if(_conf->spi_datasize == SPI_DATA8BIT) TxData += sizeof(uint8_t);
		else if(_conf->spi_datasize == SPI_DATA16BIT) TxData += sizeof(uint16_t);
	}

	res = WaitFlagTimeout(&(_spi -> SR), SPI_SR_BSY, FLAG_RESET, SPI_TIMEOUT);
	if(!CheckResult(res)) {
		Set_Line(&res, __LINE__);
//			return res;
	}

	if(_conf->spi_mode == SPI_FULLDUPLEXMASTER){
		__IO uint32_t clr_ovr = _spi -> DR;
		clr_ovr = _spi -> SR;
		(void)clr_ovr;
	}
	return res;
}

template <typename DataSize>
Result_t SPI<DataSize>::Receive(DataSize *RxData, uint32_t Data_Number){
	Result_t res = {OKE};
	Result_Init(&res, OKE, 0U, __FUNCTION__, __FILE__);
	uint32_t RxCount = 0;

	if(_conf->spi_mode == SPI_HALFDUPLEXMASTER) {
		Set_Status_Line(&res, NOTSUPPORT, __LINE__);
		return res;
	}

	if((_spi -> CR1 & SPI_CR1_SPE) != SPI_CR1_SPE) _spi -> CR1 |= SPI_CR1_SPE;

	while(RxCount++ < Data_Number){
		res = WaitFlagTimeout(&(_spi -> SR), SPI_SR_TXE, FLAG_SET, SPI_TIMEOUT);
		if(!CheckResult(res)) {
			Set_Line(&res, __LINE__);
	//			return res;
		}
		_spi -> DR = 0x00UL;

		res = WaitFlagTimeout(&(_spi -> SR), SPI_SR_RXNE, FLAG_SET, SPI_TIMEOUT);
		if(!CheckResult(res)) {
			Set_Line(&res, __LINE__);
	//			return res;
		}
		*RxData = (DataSize)_spi -> DR;

		if(_conf->spi_datasize == SPI_DATA8BIT) RxData += sizeof(uint8_t);
		else if(_conf->spi_datasize == SPI_DATA16BIT) RxData += sizeof(uint16_t);
	}

	res = WaitFlagTimeout(&(_spi -> SR), SPI_SR_BSY, FLAG_RESET, SPI_TIMEOUT);
	if(!CheckResult(res)) {
		Set_Line(&res, __LINE__);
//			return res;
	}

	if(_conf->spi_mode == SPI_FULLDUPLEXMASTER){
		__IO uint32_t clr_ovr = _spi -> DR;
		clr_ovr = _spi -> SR;
		(void)clr_ovr;
	}
	return res;
}

template <typename DataSize>
Result_t SPI<DataSize>::Transmit_Receive(DataSize *TxData, DataSize *RxData, uint32_t Data_Number){
	Result_t res = {OKE};
	Result_Init(&res, OKE, 0U, __FUNCTION__, __FILE__);
	uint32_t TxRxCount = Data_Number;

	if(_conf->spi_mode == SPI_HALFDUPLEXMASTER) {
		Set_Status_Line(&res, NOTSUPPORT, __LINE__);
		return res;
	}

	if((_spi -> CR1 & SPI_CR1_SPE) != SPI_CR1_SPE) _spi -> CR1 |= SPI_CR1_SPE;

	while(TxRxCount--){
		res = WaitFlagTimeout(&(_spi -> SR), SPI_SR_TXE, FLAG_SET, SPI_TIMEOUT);
		if(!CheckResult(res)) {
			Set_Line(&res, __LINE__);
	//			return res;
		}
		_spi -> DR = *TxData;

		res = WaitFlagTimeout(&(_spi -> SR), SPI_SR_RXNE, FLAG_SET, SPI_TIMEOUT);
		if(!CheckResult(res)) {
			Set_Line(&res, __LINE__);
	//			return res;
		}
		*RxData = (DataSize)_spi -> DR;

		if(_conf->spi_datasize == SPI_DATA8BIT) {
			TxData += sizeof(uint8_t);
			RxData += sizeof(uint8_t);
		}
		else if(_conf->spi_datasize == SPI_DATA16BIT) {
			TxData += sizeof(uint16_t);
			RxData += sizeof(uint16_t);
		}
	}

	res = WaitFlagTimeout(&(_spi -> SR), SPI_SR_BSY, FLAG_RESET, SPI_TIMEOUT);
	if(!CheckResult(res)) {
		Set_Line(&res, __LINE__);
//			return res;
	}

	if(_conf->spi_mode == SPI_FULLDUPLEXMASTER){
		__IO uint32_t clr_ovr = _spi -> DR;
		clr_ovr = _spi -> SR;
		(void)clr_ovr;
	}
	return res;
}

template <typename DataSize>
Result_t SPI<DataSize>::Transmit(DataSize TxData){
	Result_t res = {OKE};
	Result_Init(&res, OKE, 0U, __FUNCTION__, __FILE__);

	res = Transmit(&TxData, 1);
	return res;
}

template <typename DataSize>
Result_t SPI<DataSize>::Receive(DataSize *RxData){
	Result_t res = {OKE};
	Result_Init(&res, OKE, 0U, __FUNCTION__, __FILE__);

	res = Receive(RxData, 1);
	return res;
}

template <typename DataSize>
Result_t SPI<DataSize>::Transmit_Receive(DataSize TxData, DataSize *RxData){
	Result_t res = {OKE};
	Result_Init(&res, OKE, 0U, __FUNCTION__, __FILE__);

	res = Transmit_Receive(&TxData, RxData, 1);
	return res;
}

template <typename DataSize>
Result_t SPI<DataSize>::Transmit_DMA(DataSize *TxData, uint32_t Data_Number){
	Result_t res = {OKE};
	Result_Init(&res, OKE, 0U, __FUNCTION__, __FILE__);

	_spi -> CR1 &=~ SPI_CR1_SPE;
	if(_conf -> spi_mode == SPI_HALFDUPLEXMASTER) _spi -> CR1 |= SPI_CR1_BIDIOE;

	_spi -> CR2 &=~ SPI_CR2_TXDMAEN;

	res = _TxDma -> Start((uint32_t)TxData, (uint32_t) &(_spi -> DR), Data_Number);
	if(!CheckResult(res)) {
		Set_Line(&res, __LINE__);
//			return res;
	}

	_spi -> CR1 |= SPI_CR1_SPE;
	_spi -> CR2 |= SPI_CR2_ERRIE;
	_spi -> CR2 |= SPI_CR2_TXDMAEN;

	return res;
}

template <typename DataSize>
Result_t SPI<DataSize>::Receive_DMA(DataSize *RxData, uint32_t Data_Number){
	Result_t res = {OKE};
	Result_Init(&res, OKE, 0U, __FUNCTION__, __FILE__);

	_spi -> CR1 &=~ SPI_CR1_SPE;
	_spi -> CR2 &=~ SPI_CR2_RXDMAEN;

	res = _RxDma -> Start((uint32_t) &(_spi -> DR), (uint32_t)RxData, Data_Number);
	if(!CheckResult(res)) {
		Set_Line(&res, __LINE__);
//			return res;
	}

	_spi -> CR1 |= SPI_CR1_SPE;
	_spi -> CR2 |= SPI_CR2_ERRIE;
	_spi -> CR2 |= SPI_CR2_RXDMAEN;

	return res;
}

template <typename DataSize>
Result_t SPI<DataSize>::Transmit_Receive_DMA(DataSize *TxData, DataSize *RxData, uint32_t Data_Number){
	Result_t res = {OKE};
	Result_Init(&res, OKE, 0U, __FUNCTION__, __FILE__);

	_spi -> CR1 &=~ SPI_CR1_SPE;
	_spi -> CR2 &=~ (SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

	res = _RxDma -> Start((uint32_t) &(_spi -> DR), (uint32_t)RxData, Data_Number);
	if(!CheckResult(res)) {
		Set_Line(&res, __LINE__);
//			return res;
	}

	res = _TxDma -> Start((uint32_t)TxData, (uint32_t) &(_spi -> DR), Data_Number);
	if(!CheckResult(res)) {
		Set_Line(&res, __LINE__);
//			return res;
	}

	_spi -> CR1 |= SPI_CR1_SPE;
	_spi -> CR2 |= SPI_CR2_ERRIE;
	_spi -> CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;

	return res;
}

template <typename DataSize>
Result_t SPI<DataSize>::Stop_DMA(void){
	Result_t res = {OKE};
	Result_Init(&res, OKE, 0U, __FUNCTION__, __FILE__);

	if(_spi -> CR2 & SPI_CR2_RXDMAEN) {
		res = _RxDma -> Stop();
		if(!CheckResult(res)) {
			Set_Line(&res, __LINE__);
	//			return res;
		}
	}

	if(_spi -> CR2 & SPI_CR2_TXDMAEN) {
		res = _TxDma -> Stop();
		if(!CheckResult(res)) {
			Set_Line(&res, __LINE__);
	//			return res;
		}
	}
	_spi -> CR2 &=~ (SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

	return res;
}












