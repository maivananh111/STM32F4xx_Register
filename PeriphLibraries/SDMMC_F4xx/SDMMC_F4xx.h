/*
 * SDIOMMC_F4xx.h
 *
 *  Created on: Dec 11, 2022
 *      Author: anh
 */

#ifndef SDMMC_F4XX_H_
#define SDMMC_F4XX_H_

#include "PERIPH_USED.h"
#ifdef ENABLE_SDMMC


#include "stm32f4xx.h"
#include "stdio.h"
#include "PERIPH_STATUS.h"


#ifdef __cplusplus
extern "C"{
#endif

typedef enum{
	SDMMC_CLK_RISING = 0U,
	SDMMC_CLK_FALLING = SDIO_CLKCR_NEGEDGE,
} SDMMC_ClockEdge_t;

typedef enum{
	SDMMC_1WideBus = 0U,
	SDMMC_4WideBus = SDIO_CLKCR_WIDBUS_0,
	SDMMC_8WideBus = SDIO_CLKCR_WIDBUS_0,
} SDMMC_WideBus_t;

typedef struct{
	SDMMC_ClockEdge_t sdmmc_clockedge = SDMMC_CLK_RISING;
	SDMMC_WideBus_t sdmmc_widebus = SDMMC_4WideBus;
	uint32_t sdmmc_clockdiv = 0U;

} SDMMC_Config_t;



Result_t SDMMC_Init(SDMMC_Config_t *conf);






#ifdef __cplusplus
}
#endif

#endif

#endif /* SDMMC_F4XX_H_ */
