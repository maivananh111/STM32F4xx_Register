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
#include "string.h"
#include "USART_F4xx.h"
#include "PERIPH_STATUS.h"


USART usart1(USART1);
USART usart2(USART2);
USART usart3(USART3);
USART uart4 (UART4);
USART uart5 (UART5);
USART usart6(USART6);


#define USART_TIMEOUT 100U

USART::USART(USART_TypeDef *usart){
	_usart = usart;
}

void USART::Init(USART_Config_t *conf){
	_conf = conf;
	_RxDma = _conf -> RxDma;
	_TxDma = _conf -> TxDma;
	uint32_t USART_BusFreq = 0UL;

	GPIO_PortClockEnable(_conf -> TxPort);
	GPIO_PortClockEnable(_conf -> RxPort);
	if(_usart == USART1 || _usart == USART2 || _usart == USART3){
		GPIO_AlternateFunction(_conf -> TxPort, _conf -> TxPin, AF7_USART1_3);
		GPIO_AlternateFunction(_conf -> RxPort, _conf -> RxPin, AF7_USART1_3);
	}
	else{
		GPIO_AlternateFunction(_conf -> TxPort, _conf -> TxPin, AF8_USART4_6);
		GPIO_AlternateFunction(_conf -> RxPort, _conf -> RxPin, AF8_USART4_6);
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

	float USARTDIV = (float)(USART_BusFreq/(_conf -> usart_baudrate * 16.0));
	uint16_t DIV_Fraction = 0x00UL;
	uint16_t DIV_Mantissa = (uint16_t)USARTDIV;

	float Fraction = (float)(((float)(((uint16_t)(USARTDIV * 100.0) - (uint16_t)(DIV_Mantissa * 100.0)) / 100.0)) * 16.0);
	DIV_Fraction = ceil(Fraction);
	_usart -> BRR = (DIV_Mantissa << 4) | (DIV_Fraction << 0);

	if(_conf -> usart_type == USART_INTERRUPT || _conf -> usart_type == USART_INTERRUPT_DMA){
		IRQn_Type IRQ = USART1_IRQn;
		if	   (_usart == USART1) IRQ = USART1_IRQn;
		else if(_usart == USART2) IRQ = USART2_IRQn;
		else if(_usart == USART3) IRQ = USART3_IRQn;
		else if(_usart == UART4)  IRQ = UART4_IRQn;
		else if(_usart == UART5)  IRQ = UART5_IRQn;
		else if(_usart == USART6) IRQ = USART6_IRQn;

		__NVIC_SetPriority(IRQ, _conf -> usart_interruptpriority);
		__NVIC_EnableIRQ(IRQ);
	}
	Transmit('\n');
}

Result_t USART::Event_Register_Handler(void (*Event_Callback)(void *param, USART_Event_t event), void *param){
	Result_t res = {OKE};
	Result_Init(&res, OKE, __LINE__, __FUNCTION__, __FILE__);
	if(_conf -> usart_type == USART_INTERRUPT || _conf -> usart_type == USART_INTERRUPT_DMA) {
		this -> Event_Callback = Event_Callback;
		Parameter = param;

		return res;
	}
	else
		Set_Status_Line(&res, NOTSUPPORT, __LINE__);

	return res;


}


Result_t USART::Transmit(uint8_t TxData){
	Result_t res = {OKE};
	Result_Init(&res, OKE, __LINE__, __FUNCTION__, __FILE__);

	_usart -> DR = TxData;

	res = WaitFlagTimeout(&(_usart -> SR), USART_SR_TC, FLAG_SET, USART_TIMEOUT);
	if(!CheckResult(res)){
		Set_Line(&res, __LINE__);
		return res;
	}

	volatile uint32_t tmp = _usart -> SR;
	tmp = _usart -> DR;
	(void)tmp;

	return res;
}



Result_t USART::Transmit(uint8_t *TxData, uint16_t len){
	Result_t res = {OKE};
	Result_Init(&res, OKE, __LINE__, __FUNCTION__, __FILE__);
	uint16_t TxCount = len;

	while(TxCount--) {
		_usart -> DR = *TxData++;

		res = WaitFlagTimeout(&(_usart -> SR), USART_SR_TC, FLAG_SET, USART_TIMEOUT);
		if(!CheckResult(res)){
			Set_Line(&res, __LINE__);
			return res;
		}
	}
	return res;
}



Result_t USART::SendString(char *String){
	Result_t res = {OKE};
	Result_Init(&res, OKE, __LINE__, __FUNCTION__, __FILE__);

	while(*String) {
		_usart -> DR = *String++;

		res = WaitFlagTimeout(&(_usart -> SR), USART_SR_TC, FLAG_SET, USART_TIMEOUT);
		if(!CheckResult(res)){
			Set_Line(&res, __LINE__);
			return res;
		}
	}
	return res;
}



Result_t USART::Receive(uint8_t *RxData){
	Result_t res = {OKE};
	Result_Init(&res, OKE, __LINE__, __FUNCTION__, __FILE__);

	res = WaitFlagTimeout(&(_usart -> SR), USART_SR_RXNE, FLAG_SET, USART_TIMEOUT);
	if(!CheckResult(res)){
		Set_Line(&res, __LINE__);
		return res;
	}

	(*RxData) = _usart -> DR;

	return res;
}



Result_t USART::Receive(uint8_t *RxData, uint16_t len){
	Result_t res = {OKE};
	Result_Init(&res, OKE, __LINE__, __FUNCTION__, __FILE__);
	__IO uint16_t RxCount = len;
	while(RxCount--){
		res = WaitFlagTimeout(&(_usart -> SR), USART_SR_RXNE, FLAG_SET, USART_TIMEOUT);
		if(!CheckResult(res)){
			Set_Line(&res, __LINE__);
			return res;
		}

		(*RxData++) = _usart -> DR;
	}
	return res;
}



Result_t USART::TransmitIT(uint8_t *TxData, uint16_t len){
	Result_t res = {OKE};
	Result_Init(&res, OKE, __LINE__, __FUNCTION__, __FILE__);

	if(_conf -> usart_type == USART_INTERRUPT || _conf -> usart_type == USART_INTERRUPT_DMA) {
		if(_conf -> usart_interruptselect & USART_INTR_TX || _conf -> usart_interruptselect & USART_INTR_TX_RX) _usart -> CR1 |= USART_CR1_TCIE;
	}
	else{
		Set_Status_Line(&res, NOTSUPPORT, __LINE__);
		return res;
	}

	_usart -> CR1 |= USART_CR1_PEIE;
	_usart -> CR3 |= USART_CR3_EIE;

	volatile uint32_t tmp = _usart -> SR;
	tmp = _usart -> DR;

	uint16_t TxLength = len;
	uint16_t TxCount  = 0;

	while(TxCount++ < TxLength) {
		_usart -> DR = *TxData++;

		res = WaitFlagTimeout(&(_usart -> SR), USART_SR_TC, FLAG_SET, USART_TIMEOUT);
		if(!CheckResult(res)){
			Set_Line(&res, __LINE__);
			return res;
		}
	}

	_usart -> CR1 &=~ (USART_CR1_PEIE | USART_CR1_TCIE);
	_usart -> CR3 &=~ USART_CR3_EIE;

	tmp = _usart -> SR;
	tmp = _usart -> DR;
	(void)tmp;

	return res;
}



Result_t USART::ReceiveIT(uint16_t BufferSize){
	Result_t res = {OKE};
	Result_Init(&res, OKE, __LINE__, __FUNCTION__, __FILE__);

	if(_conf -> usart_type == USART_INTERRUPT || _conf -> usart_type == USART_INTERRUPT_DMA) {
		if(_conf -> usart_interruptselect & USART_INTR_RX || _conf -> usart_interruptselect & USART_INTR_TX_RX) _usart -> CR1 |= USART_CR1_RXNEIE;
	}
	else{
		Set_Status_Line(&res, NOTSUPPORT, __LINE__);
		return res;
	}

	RxCount = BufferSize;
	RxLength = 0;
	Reception = USART_Reception_Normal;
	if(RxBuffer != NULL) {
		free(RxBuffer);
		RxBuffer = (uint8_t *)malloc(RxCount * sizeof(uint8_t));
	}
	else{
		RxBuffer = (uint8_t *)malloc(RxCount * sizeof(uint8_t));
	}

	_usart -> CR1 |= USART_CR1_PEIE;
	_usart -> CR3 |= USART_CR3_EIE;

	volatile uint32_t tmp = _usart -> SR;
	tmp = _usart -> DR;
	(void)tmp;

	return res;
}

Result_t USART::ReceiveToEndCharIT(uint16_t BufferSize, char EndChar){
	Result_t res = {OKE};
	Result_Init(&res, OKE, __LINE__, __FUNCTION__, __FILE__);

	if(_conf -> usart_type == USART_INTERRUPT || _conf -> usart_type == USART_INTERRUPT_DMA) {
		if(_conf -> usart_interruptselect & USART_INTR_RX || _conf -> usart_interruptselect & USART_INTR_TX_RX) _usart -> CR1 |= USART_CR1_RXNEIE;
	}
	else{
		Set_Status_Line(&res, NOTSUPPORT, __LINE__);
		return res;
	}

	RxCount = BufferSize;
	RxLength = 0;
	Reception = USART_Reception_ToEndChar;
	this -> EndChar = EndChar;
	if(RxBuffer != NULL) {
		free(RxBuffer);
		RxBuffer = (uint8_t *)malloc(RxCount * sizeof(uint8_t));
	}
	else{
		RxBuffer = (uint8_t *)malloc(RxCount * sizeof(uint8_t));
	}

	_usart -> CR1 |= USART_CR1_PEIE;
	_usart -> CR3 |= USART_CR3_EIE;

	volatile uint32_t tmp = _usart -> SR;
	tmp = _usart -> DR;
	(void)tmp;

	return res;
}

Result_t USART::StopReceiveIT(void){
	Result_t res = {OKE};
	Result_Init(&res, OKE, __LINE__, __FUNCTION__, __FILE__);

	if(_conf -> usart_type == USART_INTERRUPT || _conf -> usart_type == USART_INTERRUPT_DMA) {
		if(((_conf -> usart_interruptselect & USART_INTR_RX) || (_conf -> usart_interruptselect & USART_INTR_TX_RX))\
				&& (_usart -> CR1 & USART_CR1_RXNEIE)){
			_usart -> CR1 &=~ USART_CR1_RXNEIE;

			if(RxBuffer != NULL) free(RxBuffer);
			RxCount = 0;
			RxLength = 0;

			return res;
		}
	}

	Set_Status_Line(&res, NOTSUPPORT, __LINE__);
	return res;
}


uint8_t USART::GetData(void){
	return _usart -> DR;
}

Result_t USART::GetBuffer(uint8_t **DestBuffer){
	Result_t res = {OKE};
	Result_Init(&res, OKE, __LINE__, __FUNCTION__, __FILE__);

	if(RxBuffer != NULL){
		*DestBuffer = RxBuffer;
		return res;
	}

	Set_Status_Line(&res, ERR, __LINE__);
	return res;
}

uint16_t USART::GetDataLength(void){
	return RxLength;
}


Result_t USART::TransmitDMA(uint8_t *TxData, uint16_t Length){
	Result_t res = {OKE};
	Result_Init(&res, OKE, __LINE__, __FUNCTION__, __FILE__);

	if(_conf -> usart_type == USART_INTERRUPT){
		Set_Status_Line(&res, NOTSUPPORT, __LINE__);
		return res;
	}

	_usart -> CR3 &=~ USART_CR3_DMAT;
	res = _TxDma -> Start((uint32_t)TxData, (uint32_t)&_usart -> DR, Length);
	if(!CheckResult(res)){
		Set_Line(&res, __LINE__);
		return res;
	}
	_usart -> SR &=~ USART_SR_TC;
	_usart -> CR3 |= USART_CR3_DMAT;

	return res;
}

Result_t USART::ReceiveDMA(uint8_t *RxData, uint16_t Length){
	Result_t res = {OKE};
	Result_Init(&res, OKE, __LINE__, __FUNCTION__, __FILE__);

	if(_conf -> usart_type == USART_INTERRUPT){
		Set_Status_Line(&res, NOTSUPPORT, __LINE__);
		return res;
	}

	_usart -> CR3 &=~ USART_CR3_DMAR;
	res = _RxDma -> Start((uint32_t)&_usart -> DR, (uint32_t)RxData, Length);
	if(!CheckResult(res)){
		Set_Line(&res, __LINE__);
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
	Result_t res = {OKE};
	Result_Init(&res, OKE, __LINE__, __FUNCTION__, __FILE__);

	if(_conf -> usart_type == USART_INTERRUPT){
		Set_Status_Line(&res, NOTSUPPORT, __LINE__);
		return res;
	}

	if(_usart -> CR3 & USART_CR3_DMAT){
		_usart -> CR3 &=~ USART_CR3_DMAT;
		res = _TxDma -> Stop();
		if(!CheckResult(res)){
			res.CodeLine = __LINE__;
			return res;
		}
		_usart -> CR1 &=~ USART_CR1_TXEIE;
	}

	if(_usart -> CR3 & USART_CR3_DMAR){
		_usart -> CR3 &=~ USART_CR3_DMAR;
		res = _RxDma -> Stop();
		if(!CheckResult(res)){
			res.CodeLine = __LINE__;
			return res;
		}
		_usart -> CR1 &=~ USART_CR1_PEIE;
		_usart -> CR3 &=~ USART_CR3_EIE;
	}

	else{
		Set_Status_Line(&res, ERR, __LINE__);
	}

	return res;
}

Result_t USART::ReceiveUntilIdleIT(uint16_t BufferSize){
	Result_t res = {OKE};
	Result_Init(&res, OKE, __LINE__, __FUNCTION__, __FILE__);

	if(_conf -> usart_type == USART_INTERRUPT || _conf -> usart_type == USART_INTERRUPT_DMA) {
		if(_conf -> usart_interruptselect & USART_INTR_RX || _conf -> usart_interruptselect & USART_INTR_TX_RX) _usart -> CR1 |= USART_CR1_RXNEIE;
	}
	else{
		Set_Status_Line(&res, NOTSUPPORT, __LINE__);
		return res;
	}

	RxCount = BufferSize;
	RxLength = 0;
	Reception = USART_Reception_ToIdle;
	if(RxBuffer != NULL) {
		free(RxBuffer);
		RxBuffer = (uint8_t *)malloc(RxCount * sizeof(uint8_t));
	}
	else{
		RxBuffer = (uint8_t *)malloc(RxCount * sizeof(uint8_t));
	}

	_usart -> CR1 |= USART_CR1_PEIE;
	_usart -> CR3 |= USART_CR3_EIE;

	volatile uint32_t tmp = _usart -> SR;
	tmp = _usart -> DR;
	(void)tmp;

	_usart -> CR1 |= USART_CR1_IDLEIE;

	return res;
}

Result_t USART::ReceiveUntilIdleDMA(uint16_t BufferSize){
	Result_t res = {OKE};
	Result_Init(&res, OKE, __LINE__, __FUNCTION__, __FILE__);

	if(_conf -> usart_type == USART_INTERRUPT_DMA) {
		if(_conf -> usart_interruptselect & USART_INTR_RX || _conf -> usart_interruptselect & USART_INTR_TX_RX) _usart -> CR1 |= USART_CR1_RXNEIE;
	}
	else{
		Set_Status_Line(&res, NOTSUPPORT, __LINE__);
		return res;
	}

	RxCount = BufferSize;
	RxLength = 0;
	Reception = USART_Reception_ToIdle;
	if(RxBuffer != NULL) {
		free(RxBuffer);
		RxBuffer = (uint8_t *)malloc(RxCount * sizeof(uint8_t));
	}
	else{
		RxBuffer = (uint8_t *)malloc(RxCount * sizeof(uint8_t));
	}

	_usart -> CR3 &=~ USART_CR3_DMAR;
	res = _RxDma -> Start((uint32_t)&_usart -> DR, (uint32_t)RxBuffer, BufferSize);
	if(!CheckResult(res)){
		Set_Line(&res, __LINE__);
		return res;
	}

	volatile uint32_t tmp = _usart -> SR;
	tmp = _usart -> DR;
	(void)tmp;

	_usart -> CR1 |= USART_CR1_PEIE;
	_usart -> CR3 |= USART_CR3_EIE;
	_usart -> CR3 |= USART_CR3_DMAR;

	tmp = _usart -> SR;
	tmp = _usart -> DR;
	(void)tmp;

	_usart -> CR1 |= USART_CR1_IDLEIE;

	return res;
}


Result_t USART::StopReceiveUntilIdleIT(void){
	Result_t res = {OKE};
	Result_Init(&res, OKE, __LINE__, __FUNCTION__, __FILE__);

	if(_conf -> usart_type == USART_INTERRUPT || _conf -> usart_type == USART_INTERRUPT_DMA) {
		if(((_conf -> usart_interruptselect & USART_INTR_RX) || (_conf -> usart_interruptselect & USART_INTR_TX_RX))\
				&& ((_usart -> CR1 & USART_CR1_RXNEIE) && (_usart -> CR1 & USART_CR1_IDLEIE))){
			_usart -> CR1 &=~ (USART_CR1_RXNEIE | USART_CR1_IDLEIE);

			if(RxBuffer != NULL) free(RxBuffer);
			RxCount = 0;
			RxLength = 0;

			return res;
		}
	}

	Set_Status_Line(&res, NOTSUPPORT, __LINE__);
	return res;
}

USART_Config_t *USART::GetConfig(void){
	return _conf;
}


void USART_IRQ_Handler(USART *usart){
	uint32_t StatusReg = usart -> _usart -> SR, Control1Reg = usart -> _usart -> CR1;
	USART_Event_t event = USART_Event_NoEvent;
	if(StatusReg & USART_SR_RXNE && Control1Reg & USART_CR1_RXNEIE) {
		volatile uint32_t tmp = usart -> _usart -> SR;
		tmp = usart -> _usart -> DR;
		(void)tmp;

		event = USART_Event_ReceiveComplete;
		usart -> _usart -> SR &=~ USART_SR_RXNE;

		if(usart -> RxLength < usart -> RxCount)
			usart -> RxBuffer[usart -> RxLength] = usart -> _usart -> DR;
		else{
			event = USART_Event_BufferOverFlow;
			goto EventCB;
		}
		if(usart -> Reception == USART_Reception_ToEndChar){
			if(usart -> RxBuffer[usart -> RxLength] == usart -> EndChar) {
				event = USART_Event_ReceiveEndChar;
				usart -> _usart -> CR1 &=~ USART_CR1_PEIE;
				usart -> _usart -> CR3 &=~ USART_CR3_EIE;
			}
		}
		usart -> RxLength++;
		goto EventCB;
	}

	if(StatusReg & USART_SR_TC&& Control1Reg & USART_CR1_TCIE) {
		volatile uint32_t tmp = usart -> _usart -> SR;
		tmp = usart -> _usart -> DR;
		(void)tmp;

		event = USART_Event_TransmitComplete;
		usart -> _usart -> SR &=~ USART_SR_TC;

		goto EventCB;
	}

	if(StatusReg & USART_SR_IDLE && Control1Reg & USART_CR1_IDLEIE) {
		volatile uint32_t tmp = usart -> _usart -> SR;
		tmp = usart -> _usart -> DR;
		(void)tmp;

		if(usart -> Reception == USART_Reception_ToIdle){
			usart -> _usart -> SR &=~ (USART_SR_IDLE | USART_SR_RXNE);
			usart -> _usart -> CR1 &=~ (USART_CR1_PEIE | USART_CR1_IDLEIE);
			if(usart -> _usart -> CR3 & USART_CR3_DMAR){
				usart -> RxCount = usart -> _RxDma -> GetCounter();
				if(usart -> _RxDma -> GetConfig() -> dma_mode != DMA_CIRCULAR){
					usart -> _usart -> CR3 &=~ (USART_CR3_EIE | USART_CR3_DMAR);
					usart -> _RxDma -> Stop();
				}
				goto EventCB;
			}
			else{
				event = USART_Event_IdleDetected;
				usart -> _usart -> CR3 &=~ USART_CR3_EIE;
				goto EventCB;
			}
		}
	}

	EventCB:
	usart -> Event_Callback(usart -> Parameter, event);

}


#endif





