/*
 * PERIPH_STATUS.cpp
 *
 *  Created on: Oct 12, 2022
 *      Author: anh
 */


#include "PERIPH_STATUS.h"
#include "stdlib.h"
#include "string.h"

uint32_t (*GetCounterFunction)(void);

void Peripheral_Status_Init(uint32_t (*GetTimerCounter)(void)){
	GetCounterFunction = GetTimerCounter;
}

Status_t CheckFlagRegister(__IO uint32_t *Register, uint32_t Flag, FlagLevel_t StatusCheck){
	if((StatusCheck == FLAG_SET)? ((*Register & Flag) != 0U) : ((*Register & Flag) == 0U)) return OKE;
	return ERR;
}

void WaitFlag(__IO uint32_t *Register, uint32_t Flag, FlagLevel_t Level){
	while((Level == FLAG_RESET)?((*Register & Flag)) : (!(*Register & Flag)));
}

Result_t WaitFlagTimeout(__IO uint32_t *Register, uint32_t Flag, FlagLevel_t Level, uint16_t TimeOut){
	Result_t res = {OKE};

	__IO uint32_t time = GetCounterFunction();
	while((Level == FLAG_RESET)?((*Register & Flag)) : (!(*Register & Flag))){
		res.Time = GetCounterFunction() - time;
		if(TimeOut != 0U){
			if(res.Time >= TimeOut) {
				res.Status  = TIMEOUT;
				res.TimeOut = res.Time;
				res.Time = 0U;
				return res;
			}
		}
	}
	return res;
}

Result_t WaitFlagTimeoutBasic(__IO uint32_t *Register, uint32_t Flag, FlagLevel_t Level, uint16_t TimeOut){
	Result_t res = {OKE};
	__IO uint64_t ovrtick = (uint64_t)(TimeOut * ((SystemCoreClock/1000000) * 55));
	__IO uint64_t time = 0UL;

	while((Level == FLAG_RESET)?(*Register & Flag) : (!(*Register & Flag))){
		time++;
		res.Time = time;
		if(ovrtick != 0U){
			if(res.Time >= ovrtick) {
				res.Status  = TIMEOUT;
				res.TimeOut = res.Time;
				res.Time = 0U;
				return res;
			}
		}
	}
	return res;
}

Result_t CheckFlag_In_WaitFlagTimeout(__IO uint32_t *RegisterCheck, uint32_t FlagCheck, FlagLevel_t LevelCheck,
									  __IO uint32_t *RegisterWait, uint32_t FlagWait, FlagLevel_t LevelWait,uint16_t TimeOut){
	Result_t res = {OKE};

	__IO uint32_t tick = GetCounterFunction();
	while((LevelWait == FLAG_RESET)? (*RegisterWait & FlagWait) : (!(*RegisterWait & FlagWait))){
		res.Time = GetCounterFunction() - tick;
		if((LevelCheck == FLAG_RESET)? (!(*RegisterCheck & FlagCheck)) : (*RegisterCheck & FlagCheck)) {
			res.Status = ERR;
			return res;
		}
		if(TimeOut != NO_TIMEOUT){
			if(res.Time > TimeOut) {
				res.Status = TIMEOUT;
				res.Time   = TimeOut;
				return res;
			}
		}
	}

	return res;
}

void Result_Init(Result_t *res, Status_t Status, uint32_t CodeLine, const char *FunctionName, const char *FileName){
	res -> Status = Status;
	res -> CodeLine = CodeLine;
	sprintf(res -> FunctionName, "%s", FunctionName);
	sprintf(res -> FileName, "%s", strrchr(FileName, '/') ? strrchr(FileName, '/') + 1 : FileName);
}

void Set_Line(Result_t *res, uint16_t line){
	res -> CodeLine = line;
}

void Set_Status_Line(Result_t *res, Status_t status, uint16_t line){
	res -> Status = status;
	res -> CodeLine = line;
}

bool CheckResult(Result_t res){
	if(res.Status != OKE) return false;
	return true;
}

bool CheckStatus(Result_t res){
	if(res.Status != READY) return false;
	return true;
}





















