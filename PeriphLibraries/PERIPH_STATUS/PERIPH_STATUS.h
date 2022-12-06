/*
 * PERIPH_STATUS.h
 *
 *  Created on: Oct 12, 2022
 *      Author: anh
 */

#ifndef PERIPH_STATUS_H_
#define PERIPH_STATUS_H_


#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f4xx.h"
#include "stdint.h"
#include "stdio.h"

typedef enum{
	FLAG_RESET = 0,
	FLAG_SET,
} FlagLevel_t;

typedef enum{
	ERR = 0,
	OKE,
	TIMEOUT,
	NOTSUPPORT,
	BUSY,
	READY
} Status_t;


typedef struct Result_t{
	Status_t Status;
	uint32_t CodeLine;
	uint32_t Time;
	uint32_t TimeOut;
	char FunctionName[30];
	char FileName[30];
} Result_t;

enum{
	NO_TIMEOUT = 0,
	DEFAULT_TIMEOUT = 1000U,

};


void Peripheral_Status_Init(uint32_t (*GetTimerCounter)(void));

Status_t CheckFlagRegister(__IO uint32_t *Register, uint32_t Flag, FlagLevel_t StatusCheck);

void WaitFlag(__IO uint32_t *Register, uint32_t Flag, FlagLevel_t Level);
Result_t WaitFlagTimeout(__IO uint32_t *Register, uint32_t Flag, FlagLevel_t Level, uint16_t TimeOut);
Result_t WaitFlagTimeoutBasic(__IO uint32_t *Register, uint32_t Flag, FlagLevel_t Level, uint16_t TimeOut);
Result_t CheckFlag_In_WaitFlagTimeout(__IO uint32_t *RegisterCheck, uint32_t FlagCheck, FlagLevel_t LevelCheck,
									  __IO uint32_t *RegisterWait, uint32_t FlagWait, FlagLevel_t LevelWait,uint16_t TimeOut);

void Set_Line(Result_t *res, uint16_t line);
void Set_Status_Line(Result_t *res, Status_t status, uint16_t line);
void Result_Init(Result_t *res, Status_t Status, uint32_t CodeLine, const char *FunctionName, const char *FileName);

bool CheckResult(Result_t res);
bool CheckStatus(Result_t res);



#ifdef __cplusplus
}
#endif

#endif /* PERIPH_STATUS_H_ */
