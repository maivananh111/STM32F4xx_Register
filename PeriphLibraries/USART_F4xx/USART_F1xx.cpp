/*
 * USART_F4xx.cpp
 *
 *  Created on: 20 thg 11, 2022
 *      Author: anh
 */
#include "PERIPH_USED.h"
#ifdef ENABLE_USART

#include "GPIO_F4xx.h"
#include "RCC_F4xx.h"
#include "math.h"
#include "USART_F4xx.h"
#include "STM_LOG.h"


#define USART_TIMEOUT 100U

USART::USART(USART_TypeDef *usart){
	_usart = usart;
}

void USART::Init(USART_Config_t *conf){
	_conf = conf;
	_RxDma = _conf -> RxDma;
	_TxDma = _conf -> TxDma;
	uint32_t USART_BusFreq = 0UL;

	GPIO_PortClockEnable(_conf -> Port);
	if(_usart == USART1 || _usart == USART2 || _usart == USART3){
		GPIO_AlternateFunction(_conf -> Port, _conf -> TxPin, AF7_USART1_3);
		GPIO_AlternateFunction(_conf -> Port, _conf -> RxPin, AF7_USART1_3);
	}
	else{
		GPIO_AlternateFunction(_conf -> Port, _conf -> TxPin, AF8_USART4_6);
		GPIO_AlternateFunction(_conf -> Port, _conf -> RxPin, AF8_USART4_6);
	}


	if(_usart == USART1 || _usart == USART6) {
		RCC -> APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_USART6EN;
		USART_BusFreq = GetBusFreq(APB2);
	}
	else {
		RCC -> APB1ENR |= RCC_APB1ENR_USART2EN | RCC_APB1ENR_USART3EN | RCC_APB1ENR_UART4EN | RCC_APB1ENR_UART5EN;
		USART_BusFreq = GetBusFreq(APB1);
	}

	_usart -> CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
	if(_conf -> Type == USART_INTERRUPT) {
		if(_conf -> InterruptSelect & USART_INTR_RX) _usart -> CR1 |= USART_CR1_RXNEIE;
		if(_conf -> InterruptSelect & USART_INTR_TX) _usart -> CR1 |= USART_CR1_TCIE;
	}

	float USARTDIV = (float)(USART_BusFreq/(_conf -> Baudrate * 16.0));
	uint16_t DIV_Fraction = 0x00UL;
	uint16_t DIV_Mantissa = (uint16_t)USARTDIV;

	float Fraction = (float)(((float)(((uint16_t)(USARTDIV * 100.0) - (uint16_t)(DIV_Mantissa * 100.0)) / 100.0)) * 16.0);
	DIV_Fraction = ceil(Fraction);
	_usart -> BRR = (DIV_Mantissa << 4) | (DIV_Fraction << 0);

	if(_conf -> Type == USART_INTERRUPT){
		IRQn_Type IRQ = USART1_IRQn;
		if	   (_usart == USART1) IRQ = USART1_IRQn;
		else if(_usart == USART2) IRQ = USART2_IRQn;
		else if(_usart == USART3) IRQ = USART3_IRQn;
		else if(_usart == UART4)  IRQ = UART4_IRQn;
		else if(_usart == UART5)  IRQ = UART5_IRQn;
		else if(_usart == USART6) IRQ = USART6_IRQn;

		__NVIC_SetPriority(IRQ, _conf -> InterruptPriority);
		__NVIC_EnableIRQ(IRQ);
	}
	Transmit('\n');
}

Result_t USART::Transmit(uint8_t Data){
	Result_t res = {OKE};

	_usart -> DR = Data;

	res = WaitFlagTimeout(&(_usart -> SR), USART_SR_TC, FLAG_SET, USART_TIMEOUT);
	if(!CheckResult(res)){ Set_Result(&res, __LINE__, __FUNCTION__, __FILE__);
		return res;
	}

	volatile uint32_t tmp = _usart -> SR;
	tmp = _usart -> DR;
	(void)tmp;

	return res;
}

Result_t USART::SendString(char *String){
	Result_t res = {OKE};

	while(*String) {
		res = Transmit(*String++);

		if(!CheckResult(res)){ Set_Result(&res, __LINE__, __FUNCTION__, __FILE__);
			return res;
		}
	}
	return res;
}

Result_t USART::Receive(uint8_t *Data){
	Result_t res = {OKE};

	res = WaitFlagTimeout(&(_usart -> SR), USART_SR_RXNE, FLAG_SET, USART_TIMEOUT);
	if(!CheckResult(res)){ Set_Result(&res, __LINE__, __FUNCTION__, __FILE__);
		return res;
	}

	*Data = _usart -> DR;

	return res;
}

Result_t USART::TransmitDMA(uint8_t *TxData, uint16_t Length){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	_usart -> CR3 &=~ USART_CR3_DMAT;
	res = _TxDma -> Start((uint32_t)TxData, (uint32_t)&_usart -> DR, Length);
	if(res.Status != OKE){res.CodeLine = __LINE__;
		return res;
	}
	_usart -> SR &=~ USART_SR_TC;
	_usart -> CR3 |= USART_CR3_DMAT;

	return res;
}

Result_t USART::ReceiveDMA(uint8_t *RxData, uint16_t Length){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	_usart -> CR3 &=~ USART_CR3_DMAR;
	res = _RxDma -> Start((uint32_t)&_usart -> DR, (uint32_t)RxData, Length);
	if(res.Status != OKE){res.CodeLine = __LINE__;
		return res;
	}
	volatile uint32_t tmp = _usart -> SR;
	tmp = _usart -> DR;
	(void)tmp;

	_usart -> CR1 |= USART_CR1_PEIE;
	_usart -> CR3 |= USART_CR3_EIE;
	_usart -> CR3 |= USART_CR3_DMAR;

	return res;
}

Result_t USART::Stop_DMA(void){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	if(_usart -> CR3 & USART_CR3_DMAT){
		_usart -> CR3 &=~ USART_CR3_DMAT;
		res = _TxDma -> Stop();
		if(res.Status != OKE){res.CodeLine = __LINE__;
			return res;
		}
		_usart -> CR1 &=~ USART_CR1_TXEIE;
	}

	if(_usart -> CR3 & USART_CR3_DMAR){
		_usart -> CR3 &=~ USART_CR3_DMAR;
		res = _RxDma -> Stop();
		if(res.Status != OKE){res.CodeLine = __LINE__;
			return res;
		}
		_usart -> CR1 &=~ USART_CR1_PEIE;
		_usart -> CR3 &=~ USART_CR3_EIE;
	}

	else{
		res.CodeLine = __LINE__;
		res.Status = ERR;
	}

	return res;
}


#endif





