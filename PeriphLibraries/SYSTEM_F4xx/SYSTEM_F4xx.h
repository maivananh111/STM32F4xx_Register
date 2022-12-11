/*
 * SYSTEM_F4x579.h
 *
 *  Created on: Oct 12, 2022
 *      Author: anh
 */

#ifndef SYSTEM_F4X579_H_
#define SYSTEM_F4X579_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdio.h"
#include "stm32f4xx.h"
#include "stdbool.h"

typedef struct{
	uint32_t System_Clock;
	uint32_t Latency;
	bool Prefetch;
	bool Instruction_Cache;
	bool Data_Cache;
} FlashMemConfig_t;

typedef struct{
	bool VoltScale2;
	bool ClockEnable;
} PowerConfig_t;

typedef enum{
	No_Debug,
	SWD,
	JTAG,
	SWD_JTAG,
} DebugMode_t;

typedef struct{
	uint32_t heap_ram_used;
	uint32_t prog_ram_used;
	uint32_t stack_ram_used;
	uint32_t free_ram;
	uint32_t total_free_ram;
} Memory_t;

void Power_Configuration(PowerConfig_t *pwr_conf);
void Update_Latency(void);
void FlashMem_Configuration(FlashMemConfig_t *flh_conf);

Memory_t Get_MemorySize(void);



#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_F4X579_H_ */
