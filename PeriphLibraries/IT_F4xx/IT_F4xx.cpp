/*
 * IT_F4xx.cpp
 *
 *  Created on: Oct 18, 2022
 *      Author: anh
 */

#include "IT_F4xx.h"
#include "PERIPH_USED.h"
// USER INCLUDE


// SYSTEM TICK TIMER INTERRUPT HANDLER.
#ifdef ENABLE_SYSTEMTICK
#include "RCC_F4xx.h"
void SysTick_Handler(void){
	Tick++;
	// USER SYSTEM TICK TIMER INTERRUPT HANDLER CODE:

}
#endif
