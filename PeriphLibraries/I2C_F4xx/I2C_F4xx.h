/*
 * I2C_F4xx.h
 *
 *  Created on: Nov 7, 2022
 *      Author: anh
 */

#ifndef I2C_F4XX_H_
#define I2C_F4XX_H_

#include "PERIPH_USED.h"

#ifdef ENABLE_I2C

#include "stdio.h"
#include "stm32f4xx.h"
#include "PERIPH_STATUS.h"
#include "DMA_F4xx.h"

#ifdef __cplusplus
extern "C"{
#endif


typedef enum{
	I2C_STANDARD_MODE = 0,
	I2C_FAST_MODE,
} I2C_Mode_t;

typedef enum{
	I2C_DUTY_2 = 0,
	I2C_DUTY_16_9,
} I2C_ClockDuty_t;

typedef enum{
	I2C_ADDRESS_7BIT = 0,
	I2C_ADDRESS_10BIT,
} I2C_AddressMode_t;

typedef enum{
	I2C_WRITE = 0,
	I2C_READ,
} I2C_Action_t;

typedef struct{
	I2C_Mode_t i2c_mode;
	uint32_t i2c_frequency;
	I2C_AddressMode_t i2c_addressmode = I2C_ADDRESS_7BIT;
	I2C_ClockDuty_t i2c_clockduty = I2C_DUTY_2;
	DMA *TxDma = NULL;
	DMA *RxDma = NULL;
	GPIO_TypeDef *SCLPort;
	uint16_t SCLPin;
	GPIO_TypeDef *SDAPort;
	uint16_t SDAPin;
} I2C_Config_t;

class I2C{
	public:
		I2C(I2C_TypeDef *i2c);
		Result_t Init(I2C_Config_t *conf);

		Result_t WaitBusy(void);

		Result_t SendStart(void);
		Result_t SendRepeatStart(void);
		Result_t SendSlaveAddr(uint16_t Slave_Address, I2C_Action_t Action);
		Result_t SendStart_SlaveAddr(uint16_t Slave_Address, I2C_Action_t Action);
		Result_t SendRepeatStart_SlaveAddr(uint16_t Slave_Address, I2C_Action_t Action);

		Result_t Transmit(uint8_t *TxData, uint16_t Size);
		Result_t Transmit(uint8_t TxData);

		Result_t Receive(uint8_t *Data, uint16_t Size);
		Result_t Receive(uint8_t *Data);

		Result_t Transmit_DMA(uint8_t *TxData, uint16_t Size);
		Result_t Receive_DMA(uint8_t *RxData, uint16_t Size);

		Result_t Stop_Transmit_DMA(void);
		Result_t Stop_Receive_DMA(void);

		Result_t SendStop(void);

		Result_t Memory_Transmit(uint16_t Slave_Address, uint16_t MemAddr, uint8_t MemAddrSize, uint8_t *Data, uint16_t Size);
		Result_t Memory_Receive(uint16_t Slave_Address, uint16_t MemAddr, uint8_t MemAddrSize, uint8_t *Data, uint16_t Size);

		Result_t CheckDevices(uint16_t Slave_Address, uint8_t Trials, uint16_t TimeOut);


		DMA *_TxDma, *_RxDma;

	private:
		I2C_TypeDef *_i2c;
		I2C_Config_t *_conf;
		void ACKError_Action(void);
		void ClearADDR(void);
};

#ifdef __cplusplus
}
#endif

#endif

#endif /* I2C_F4XX_H_ */
