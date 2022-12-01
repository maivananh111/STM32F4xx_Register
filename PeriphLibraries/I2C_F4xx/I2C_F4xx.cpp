/*
 * I2C_F4xx.cpp
 *
 *  Created on: Nov 7, 2022
 *      Author: anh
 */

#include "PERIPH_USED.h"

#ifdef ENABLE_I2C

#include "I2C_F4xx.h"
#include "RCC_F4xx.h"
#include "GPIO_F4xx.h"
#include "STM_LOG.h"

#define I2C_BUSY_TIMEOUT            1000U     // 25ms.
#define I2C_DEFAULT_TIMEOUT         1000U     // 100ms.
#define I2C_MIN_FREQ_STANDARD       2000000U // Minimum is 2MHz of APB1 bus in standard mode.
#define I2C_MIN_FREQ_FAST           4000000U // Minimum is 4MHz of APB1 bus in standard mode.

I2C::I2C(I2C_TypeDef *i2c){
	_i2c = i2c;
}

Result_t I2C::Init(I2C_Config_t *conf){
	_conf = conf;
	_TxDma = _conf -> TxDma;
	_RxDma = _conf -> RxDma;

	Result_t res = {OKE};
	Set_Result_State(&res, OKE, 0U, __FUNCTION__, __FILE__);

	if(_conf -> i2c_mode == I2C_STANDARD_MODE && _conf -> i2c_frequency > 100000U) {
		Set_Result_State(&res, ERR, 0U, __FUNCTION__, __FILE__);
		return res;
	}

	/* ***********************I2C CLK ENABLE*********************** */
	if(_i2c == I2C1)      RCC -> APB1ENR |= RCC_APB1ENR_I2C1EN;
	else if(_i2c == I2C2) RCC -> APB1ENR |= RCC_APB1ENR_I2C2EN;
	else if(_i2c == I2C3) RCC -> APB1ENR |= RCC_APB1ENR_I2C3EN;

	/* ***********************GPIO INIT********************** */
	GPIO_AlternateFunction(_conf -> SCLPort, _conf -> SCLPin, AF4_I2C1_3);
	GPIO_AF_Type(_conf -> SCLPort, _conf -> SCLPin, GPIO_OUTPUT_OPENDRAIN);
	GPIO_AlternateFunction(_conf -> SDAPort, _conf -> SDAPin, AF4_I2C1_3);
	GPIO_AF_Type(_conf -> SDAPort, _conf -> SDAPin, GPIO_OUTPUT_OPENDRAIN);

	/* ***********************DISABLE I2C********************** */
	_i2c -> CR1 &=~ I2C_CR1_PE;
	/* ***********************RESET I2C********************** */
	_i2c -> CR1 |= I2C_CR1_SWRST;
	TickDelay_ms(5);
	_i2c -> CR1 &= ~I2C_CR1_SWRST;
	/* ***********************CHECK MINIMUM ALLOWED FREQUENCY********************** */
	uint32_t apb1_freq = GetBusFreq(APB1);
	if(((_conf -> i2c_frequency <= 100000U)? (apb1_freq < I2C_MIN_FREQ_STANDARD) : (apb1_freq < I2C_MIN_FREQ_FAST))) {
		Set_Result_State(&res, ERR, __LINE__, __FUNCTION__, __FILE__);
		return res;
	}
	/* ***********************SET ABP1 FREQUENCY TO CR2 REGISTER********************** */
	_i2c -> CR2 |= (uint32_t)((apb1_freq / 1000000U) >> I2C_CR2_FREQ_Pos);
	/* ***********************SET SCL RISE TIME********************** */
	_i2c -> TRISE = (uint32_t)((_conf -> i2c_mode == I2C_STANDARD_MODE)? ((apb1_freq/1000000U) + 1U) : ((((apb1_freq/1000000U) * 300U) / 1000U) + 1U));
	/* ***********************SET I2C SPEED********************** */
	if(_conf -> i2c_mode == I2C_STANDARD_MODE){
		_i2c -> CCR &=~ I2C_CCR_FS;
		// T_high = T_low = CCR * T_PCLK1, T = 2 * T_high.
		uint32_t ccr_tmp = (uint32_t)((apb1_freq - 1U)/(2U * _conf -> i2c_frequency) + 1U);
		if(ccr_tmp < 4U) ccr_tmp = 4U;
		_i2c -> CCR |= ccr_tmp;
	}
	else if(_conf -> i2c_mode == I2C_FAST_MODE){
		_i2c -> CCR |= I2C_CCR_FS;
		if(_conf -> i2c_clockduty == I2C_DUTY_2){
			_i2c -> CCR &=~ I2C_CCR_DUTY;
			// T_high = (1/2)T_low = CCR * TPCLK1, T = 3 * T_high.
			uint32_t ccr_tmp = (uint32_t)((apb1_freq)/(3U * _conf -> i2c_frequency));
			if(ccr_tmp < 1U) ccr_tmp = 1U;
			_i2c -> CCR |= ccr_tmp;

		}
		else if(_conf -> i2c_clockduty == I2C_DUTY_16_9){
			_i2c -> CCR |= I2C_CCR_DUTY;
			// T_high = (9/16)T_low = CCR * TPCLK1, T = 25 * T_high.
			uint32_t ccr_tmp = (uint32_t)((apb1_freq)/(25U * _conf -> i2c_frequency));
			if(ccr_tmp < 4U) ccr_tmp = 4U;
			_i2c -> CCR |= ccr_tmp;
		}
	}
	/* ***********************ENABLE I2C PERIPHERAL********************** */
	_i2c -> CR1 |= I2C_CR1_PE;
	/* ***********************CLEAR ALL FLAG********************** */
	_i2c -> SR1 = 0UL;
	_i2c -> SR2 = 0UL;

	return res;
}

void I2C::ACKError_Action(void){
	_i2c -> CR1 |= I2C_CR1_STOP;
	_i2c -> SR1 = ~(I2C_SR1_AF & 0xFFFF);
}

void I2C::ClearADDR(void){
	__IO uint32_t tmp = _i2c -> SR1 | _i2c -> SR2;
	(void)tmp;
}

Result_t I2C::WaitBusy(void){
	Result_t res = {OKE};
	Set_Result_State(&res, BUSY, 0U, __FUNCTION__, __FILE__);

	/* ***********************CHECK BUSY FLAG********************** */
	res = WaitFlagTimeout(&(_i2c -> SR2), I2C_SR2_BUSY, FLAG_RESET, I2C_BUSY_TIMEOUT);
	if(CheckResult(res)){
		Set_Result_State(&res, READY, __LINE__, __FUNCTION__, __FILE__);
		return res;
	}

	Set_Result_State(&res, BUSY, __LINE__, __FUNCTION__, __FILE__);

	return res;
}

Result_t I2C::SendStart(void){
	Result_t res = {OKE};
	Set_Result_State(&res, OKE, 0U, __FUNCTION__, __FILE__);

	/* ***********************WAIT I2C NOT BUSY********************** */
	res = WaitBusy();
	if(!CheckStatus(res)){
		Set_Result(&res, __LINE__, __FUNCTION__, __FILE__);
		return res;
	}
	/* ***********************DISABLE POS********************** */
	_i2c -> CR1 &=~ I2C_CR1_POS;
	/* ***********************GENERATE START********************** */
	_i2c -> CR1 |= I2C_CR1_START;
	/* ***********************WAIT SB FLAG IS SET********************** */
	res = WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_SB, FLAG_SET, I2C_DEFAULT_TIMEOUT);
	if(!CheckResult(res)){
		Set_Result(&res, __LINE__, __FUNCTION__, __FILE__);
		return res;
	}

	return res;
}

Result_t I2C::SendRepeatStart(void){
	Result_t res = {OKE};
	Set_Result_State(&res, OKE, 0U, __FUNCTION__, __FILE__);

	/* ***********************GENERATE START********************** */
	_i2c -> CR1 |= I2C_CR1_START;
	/* ***********************WAIT SB FLAG IS SET********************** */
	res = WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_SB, FLAG_SET, I2C_DEFAULT_TIMEOUT);
	if(!CheckResult(res)){
		Set_Result(&res, __LINE__, __FUNCTION__, __FILE__);
		return res;
	}

	return res;
}

Result_t I2C::SendSlaveAddr(uint16_t Slave_Address, I2C_Action_t Action){
	Result_t res = {OKE};
	Set_Result_State(&res, OKE, 0U, __FUNCTION__, __FILE__);

	/* ***********************SEND SLAVE ADDRESS*********************** */
	if(_conf -> i2c_addressmode == I2C_ADDRESS_7BIT){
		if(Action == I2C_WRITE)_i2c -> DR = (uint8_t)((Slave_Address & 0x00FF) & ~I2C_OAR1_ADD0); // SEND 8BIT SLAVE ADDRESS WITH LSB BIT IS RESET(TRANSMIT MODE).
		else                   _i2c -> DR = (uint8_t)((Slave_Address & 0x00FF) |  I2C_OAR1_ADD0); // SEND 8BIT SLAVE ADDRESS WITH LSB BIT IS SET(RECEIVE MODE).
	}
	/* ***********************WAIT ADDR FLAG IS SET*********************** */
	res = CheckFlag_In_WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_AF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_ADDR, FLAG_SET, I2C_DEFAULT_TIMEOUT);
	if(!CheckResult(res)){
		if(res.Status == ERR)
			ACKError_Action();
		Set_Result(&res, __LINE__, __FUNCTION__, __FILE__);
		return res;
	}
	/* ***********************CLEAR ADDR FLAG*********************** */
	ClearADDR();

	return res;
}

Result_t I2C::SendStart_SlaveAddr(uint16_t Slave_Address, I2C_Action_t Action){
	Result_t res = {OKE};
	Set_Result_State(&res, OKE, 0U, __FUNCTION__, __FILE__);

	/* ************* SEND START CONDITION **************** */
	res.Status = SendStart().Status;
	if(!CheckResult(res)){
		Set_Result(&res, __LINE__, __FUNCTION__, __FILE__);
		return res;
	}
	/* ************* SEND SLAVE ADDRESS **************** */
	res.Status = SendSlaveAddr(Slave_Address, Action).Status;
	if(!CheckResult(res)){
		Set_Result(&res, __LINE__, __FUNCTION__, __FILE__);
		return res;
	}

	return res;
}

Result_t I2C::SendRepeatStart_SlaveAddr(uint16_t Slave_Address, I2C_Action_t Action){
	Result_t res = {OKE};
	Set_Result_State(&res, OKE, 0U, __FUNCTION__, __FILE__);

	/* ************* SEND REPEAT START CONDITION **************** */
	res.Status = SendRepeatStart().Status;
	if(!CheckResult(res)){
		Set_Result(&res, __LINE__, __FUNCTION__, __FILE__);
		return res;
	}
	/* ************* SEND SLAVE ADDRESS **************** */
	res.Status = SendSlaveAddr(Slave_Address, Action).Status;
	if(!CheckResult(res)){
		Set_Result(&res, __LINE__, __FUNCTION__, __FILE__);
		return res;
	}

	return res;
}

Result_t I2C::CheckDevices(uint16_t Slave_Address, uint8_t Trials, uint16_t TimeOut){
	Result_t res = {OKE};
	Set_Result_State(&res, OKE, 0U, __FUNCTION__, __FILE__);

	uint8_t I2C_Trials = 1U;
	__IO uint32_t tick = GetTick();

	res = WaitBusy();
	if(!CheckStatus(res)){
		Set_Result(&res, __LINE__, __FUNCTION__, __FILE__);
		return res;
	}

	_i2c -> CR1 |= I2C_CR1_PE;
	_i2c -> CR1 &=~ I2C_CR1_POS;

	do{
		res = SendStart();
		if(!CheckResult(res)){
			Set_Result(&res, __LINE__, __FUNCTION__, __FILE__);
			return res;
		}

		_i2c -> DR = (uint8_t)((Slave_Address & 0x00FF) & ~I2C_OAR1_ADD0);

		tick = GetTick();
		while(!(_i2c -> SR1 & I2C_SR1_ADDR) && !(_i2c -> SR1 & I2C_SR1_AF)){
			if(GetTick() - tick > TimeOut){
				break;
			}
		}
		if(_i2c -> SR1 & I2C_SR1_ADDR){
			_i2c -> CR1 |= I2C_CR1_STOP;
			_i2c -> SR1 &=~ I2C_SR1_ADDR;
			while(_i2c -> SR2 & I2C_SR2_BUSY){
				if(GetTick() - tick > TimeOut){
					Set_Result_State(&res, ERR, __LINE__, __FUNCTION__, __FILE__);
					return res;
				}
			}
			Set_Result_State(&res, OKE, __LINE__, __FUNCTION__, __FILE__);
			return res;
		}
		else{
			_i2c -> CR1 |= I2C_CR1_STOP;
			_i2c -> SR1 &=~ I2C_SR1_AF;
			while(_i2c -> SR2 & I2C_SR2_BUSY){
				if(GetTick() - tick > TimeOut){
					Set_Result_State(&res, ERR, __LINE__, __FUNCTION__, __FILE__);
					return res;
				}
			}
		}
		I2C_Trials++;
	} while(I2C_Trials < Trials);

	Set_Result_State(&res, ERR, __LINE__, __FUNCTION__, __FILE__);

	return res;
}

Result_t I2C::Transmit(uint8_t *TxData, uint16_t Size){
	Result_t res = {OKE};
	Set_Result_State(&res, OKE, 0U, __FUNCTION__, __FILE__);
	uint32_t TxCount = Size;

	/* ***********************WAIT TXE FLAG IS SET*********************** */
	res = CheckFlag_In_WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_AF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_TXE, FLAG_SET, I2C_DEFAULT_TIMEOUT);
	if(!CheckResult(res)){
		if(res.Status == ERR)
			ACKError_Action();
		Set_Result(&res, __LINE__, __FUNCTION__, __FILE__);
		return res;
	}
	/* ***********************TRANSMIT DATA*********************** */
	while(TxCount > 0U){
		/* ***********************WAIT TXE FLAG IS SET*********************** */
		res = CheckFlag_In_WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_AF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_TXE, FLAG_SET, I2C_DEFAULT_TIMEOUT);
		if(!CheckResult(res)){
			if(res.Status == ERR)
				ACKError_Action();
			Set_Result(&res, __LINE__, __FUNCTION__, __FILE__);
			return res;
		}
		/* ***********************WRITE DATA TO DR*********************** */
		_i2c -> DR = *TxData++;
		TxCount--;
		if((_i2c -> SR1 & I2C_SR1_BTF) && TxCount != 0U){
			/* ***********************WRITE DATA TO DR*********************** */
			_i2c -> DR = *TxData++;
			TxCount--;
		}
	}
	/* ***********************WAIT BTF FLAG IS SET*********************** */
	res = CheckFlag_In_WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_AF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_BTF, FLAG_SET, I2C_DEFAULT_TIMEOUT);
	if(!CheckResult(res)){
		if(res.Status == ERR)
			ACKError_Action();
		Set_Result(&res, __LINE__, __FUNCTION__, __FILE__);
		return res;
	}

	return res;
}

Result_t I2C::Transmit(uint8_t TxData){
	Result_t res = {OKE};
	Set_Result_State(&res, OKE, 0U, __FUNCTION__, __FILE__);

	/* ***********************WAIT TXE FLAG IS SET*********************** */
	res = CheckFlag_In_WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_AF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_TXE, FLAG_SET, I2C_DEFAULT_TIMEOUT);
	if(!CheckResult(res)){
		if(res.Status == ERR)
			ACKError_Action();
		Set_Result(&res, __LINE__, __FUNCTION__, __FILE__);
		return res;
	}
	/* ***********************WRITE DATA TO DR*********************** */
	_i2c -> DR = TxData;
	/* ***********************WAIT BTF FLAG IS SET*********************** */
	res = CheckFlag_In_WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_AF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_BTF, FLAG_SET, I2C_DEFAULT_TIMEOUT);
	if(!CheckResult(res)){
		if(res.Status == ERR)
			ACKError_Action();
		Set_Result(&res, __LINE__, __FUNCTION__, __FILE__);
		return res;
	}

	return res;
}

Result_t I2C::Receive(uint8_t *Data, uint16_t Size){
	Result_t res = {OKE};
	Set_Result_State(&res, OKE, 0U, __FUNCTION__, __FILE__);
	uint32_t RxCount = Size;

	/* ********************START READ DATA****************** */
	// SETUP BEFORE READ DATA.
	if(RxCount == 0U){
		// CLEAR ADDR FLAG.
		ClearADDR();
		// GENERATE STOP.
		_i2c -> CR1 |= I2C_CR1_STOP;
	}
	else if(RxCount == 1U){
		// DISABLE ACK.
		_i2c -> CR1 &=~ I2C_CR1_ACK;
		// CLEAR ADDR FLAG.
		ClearADDR();
		// GENERATE STOP.
		_i2c -> CR1 |= I2C_CR1_STOP;
	}
	else if(RxCount == 2U){
		// ENABLE POS.
		_i2c -> CR1 |= I2C_CR1_POS;
		// CLEAR ADDR FLAG.
		ClearADDR();
		// DISABLE ACK.
		_i2c -> CR1 &=~ I2C_CR1_ACK;
	}
	else{
		// ENABLE ACK.
		_i2c -> CR1 |= I2C_CR1_ACK;
		// CLEAR ADDR FLAG.
		ClearADDR();
	}
	// START READ DATA.
	while(RxCount > 0U){
		if(RxCount <= 3U){
			if(RxCount == 1U){
				// WAIT RXNE IS SET.
				res = CheckFlag_In_WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_STOPF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_RXNE, FLAG_SET, I2C_DEFAULT_TIMEOUT);
				if(!CheckResult(res)){
					if(res.Status == ERR) _i2c -> SR1 = ~(I2C_SR1_STOPF & 0xFFFF);
					Set_Result(&res, __LINE__, __FUNCTION__, __FILE__);
					return res;
				}
				// READ FORM DR.
				*Data++ = (uint8_t)_i2c -> DR;
				RxCount--;
			}
			else if(RxCount == 2U){
				// WAIT BTF FLAG IS SET.
				res = WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_BTF, FLAG_SET, I2C_DEFAULT_TIMEOUT);
				if(!CheckResult(res)) {
					Set_Result(&res, __LINE__, __FUNCTION__, __FILE__);
					return res;
				}
				// GENERATE STOP.
				_i2c -> CR1 |= I2C_CR1_STOP;
				// READ FORM DR.
				*Data++ = (uint8_t)_i2c -> DR;
				RxCount--;
				// READ FORM DR.
				*Data++ = (uint8_t)_i2c -> DR;
				RxCount--;

			}
			else{
				// WAIT BTF FLAG IS SET.
				res = WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_BTF, FLAG_SET, I2C_DEFAULT_TIMEOUT);
				if(!CheckResult(res)) { res.CodeLine = __LINE__; return res;}
				// DISABLE ACK.
				_i2c -> CR1 &=~ I2C_CR1_ACK;
				// READ FORM DR.
				*Data++ = (uint8_t)_i2c -> DR;
				RxCount--;
				// WAIT BTF FLAG IS SET.
				res = CheckFlag_In_WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_AF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_BTF, FLAG_SET, I2C_DEFAULT_TIMEOUT);
				if(!CheckResult(res)){
					Set_Result(&res, __LINE__, __FUNCTION__, __FILE__);
					return res;
				}
				// GENERATE STOP.
				_i2c -> CR1 |= I2C_CR1_STOP;
				// READ FORM DR.
				*Data++ = (uint8_t)_i2c -> DR;
				RxCount--;
				// READ FORM DR.
				*Data++ = (uint8_t)_i2c -> DR;
				RxCount--;
			}
		}
		else{
			// WAIT RXNE IS SET.
			res = CheckFlag_In_WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_STOPF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_RXNE, FLAG_SET, I2C_DEFAULT_TIMEOUT);
			if(!CheckResult(res)){
				if(res.Status == ERR) _i2c -> SR1 = ~(I2C_SR1_STOPF & 0xFFFF);
				Set_Result(&res, __LINE__, __FUNCTION__, __FILE__);
				return res;
			}
			// READ FORM DR.
			*Data++ = (uint8_t)_i2c -> DR;
			RxCount--;
			if(_i2c -> SR1 & I2C_SR1_BTF){
				// READ FORM DR.
				*Data++ = (uint8_t)_i2c -> DR;
				RxCount--;
			}
		}
	}
	return res;
}

Result_t I2C::Receive(uint8_t *Data){
	Result_t res = {OKE};
	Set_Result_State(&res, OKE, 0U, __FUNCTION__, __FILE__);

	// DISABLE ACK.
	_i2c -> CR1 &=~ I2C_CR1_ACK;
	// CLEAR ADDR FLAG.
	ClearADDR();
	// GENERATE STOP.
	_i2c -> CR1 |= I2C_CR1_STOP;
	// WAIT RXNE IS SET.
	res = CheckFlag_In_WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_STOPF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_RXNE, FLAG_SET, I2C_DEFAULT_TIMEOUT);
	if(!CheckResult(res)){
		if(res.Status == ERR) _i2c -> SR1 = ~(I2C_SR1_STOPF & 0xFFFF);
		Set_Result(&res, __LINE__, __FUNCTION__, __FILE__);
		return res;
	}
	// READ FORM DR.
	*Data = (uint8_t)_i2c -> DR;

	return res;
}

Result_t I2C::SendStop(void){
	// SEND STOP CONDITION.
	_i2c -> CR1 |= I2C_CR1_STOP;

	return {OKE, 0 };
}

Result_t I2C::Transmit_DMA(uint8_t *TxData, uint16_t Size){
	Result_t res = {OKE};
	Set_Result_State(&res, OKE, 0U, __FUNCTION__, __FILE__);

	// SETUP AND START DMA.
	res = _TxDma -> Start((uint32_t)TxData, (uint32_t)(&_i2c -> DR), Size);
	if(!CheckResult(res)){
		Set_Result(&res, __LINE__, __FUNCTION__, __FILE__);
		return res;
	}
	// ENABLE I2C TX DMA
	_i2c -> CR2 |= I2C_CR2_DMAEN;

	return res;
}

Result_t I2C::Receive_DMA(uint8_t *RxData, uint16_t Size){
	Result_t res = {OKE};
	Set_Result_State(&res, OKE, 0U, __FUNCTION__, __FILE__);

	// SETUP AND START DMA.
	res = _RxDma -> Start((uint32_t)(&_i2c -> DR), (uint32_t)RxData, Size);
	if(!CheckResult(res)){
		Set_Result(&res, __LINE__, __FUNCTION__, __FILE__);
		return res;
	}
	// RECEIVE ONLY 1 BYTE, NO NEED ACK.
	if(Size == 1U) _i2c -> CR1 &=~ I2C_CR1_ACK;
	// IF RECEIVE MORE THAN 1 BYTE, ENABLE DMA CHECK LAST BYTE.
	else _i2c -> CR2 |= I2C_CR2_LAST;
	// ENABLE I2C RX DMA
	_i2c -> CR2 |= I2C_CR2_DMAEN;

	return res;
}

Result_t I2C::Stop_Transmit_DMA(void){
	Result_t res = {OKE};
	Set_Result_State(&res, OKE, 0U, __FUNCTION__, __FILE__);

	if(_i2c -> CR2 & I2C_CR2_DMAEN){
		_TxDma -> Stop();
		_i2c -> CR2 &=~ I2C_CR2_DMAEN;
	}
	else{
		Set_Result_State(&res, ERR, 0U, __FUNCTION__, __FILE__);
	}

	return res;
}
Result_t I2C::Stop_Receive_DMA(void){
	Result_t res = {OKE};
	Set_Result_State(&res, OKE, 0U, __FUNCTION__, __FILE__);

	if(_i2c -> CR2 & I2C_CR2_DMAEN){
		_RxDma -> Stop();
		_i2c -> CR2 &=~ I2C_CR2_DMAEN;
	}
	else{
		Set_Result_State(&res, ERR, 0U, __FUNCTION__, __FILE__);
	}

	return res;
}

Result_t I2C::Memory_Transmit(uint16_t Slave_Address, uint16_t MemAddr, uint8_t MemAddrSize, uint8_t *Data, uint16_t Size){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};

	res = SendStart();
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }
	res = SendSlaveAddr(Slave_Address, I2C_WRITE);
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }
	if(MemAddrSize == 1U) {
		res = Transmit((uint8_t)(MemAddr & 0x00FF)); // MEMORY ADDRESS LSB.
		if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }
	}
	else{
		res = Transmit((uint8_t)((MemAddr & 0x00FF) >> 8U)); // MEMORY ADDRESS MSB.
		if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }
		res = Transmit((uint8_t)(MemAddr & 0x00FF));         // MEMORY ADDRESS LSB.
		if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }
	}
	res = Transmit(Data, Size);
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }
	SendStop();

	return res;
}

Result_t I2C::Memory_Receive(uint16_t Slave_Address, uint16_t MemAddr, uint8_t MemAddrSize, uint8_t *Data, uint16_t Size){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};


	res = SendStart();
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }
	res = SendSlaveAddr(Slave_Address, I2C_WRITE);
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }
	if(MemAddrSize == 1U) {
		res = Transmit((uint8_t)(MemAddr & 0x00FF)); // MEMORY ADDRESS LSB.
		if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }
	}
	else{
		res = Transmit((uint8_t)((MemAddr & 0x00FF) >> 8U)); // MEMORY ADDRESS MSB.
		if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }
		res = Transmit((uint8_t)(MemAddr & 0x00FF));         // MEMORY ADDRESS LSB.
		if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }
	}
	res = SendRepeatStart();
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }
	res = SendSlaveAddr(Slave_Address, I2C_READ);
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }
	res = Receive(Data, Size);
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }
	SendStop();

	return res;
}


#endif







