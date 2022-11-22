/*
 * IT_F4xx.h
 *
 *  Created on: Oct 18, 2022
 *      Author: anh
 */

#ifndef IT_F4XX_H_
#define IT_F4XX_H_




#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include "PERIPH_USED.h"

#ifdef ENABLE_SYSTEMTICK
void SysTick_Handler(void);
#endif






#ifdef __cplusplus
}
#endif

#endif /* IT_F4XX_H_ */
