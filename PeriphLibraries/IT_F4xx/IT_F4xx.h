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


/* EXTERNAL INTERRUPT */
typedef enum{
	EXTI_RISING_EDGE  = EXTI_RTSR_TR0,
	EXTI_FALLING_EDGE = EXTI_FTSR_TR0,
	EXTI_RISING_FALLING_EDGE  = EXTI_RTSR_TR0 | EXTI_FTSR_TR0,
} EXTI_EdgeDetect_t;

void EXTI_Init(GPIO_TypeDef *Port, uint16_t Pin, EXTI_EdgeDetect_t Edge, uint32_t Priority);
void EXTI_Callback(uint16_t Pin);

#ifdef ENABLE_EXTI0
void EXTI0_IRQHandler(void);             			/* EXTI Line0 interrupt */
#endif
#ifdef ENABLE_EXTI1
void EXTI1_IRQHandler(void);              			/* EXTI Line1 interrupt */
#endif
#ifdef ENABLE_EXTI2
void EXTI2_IRQHandler(void);              			/* EXTI Line2 interrupt */
#endif
#ifdef ENABLE_EXTI3
void EXTI3_IRQHandler(void);              			/* EXTI Line3 interrupt */
#endif
#ifdef ENABLE_EXTI4
void EXTI4_IRQHandler(void);              			/* EXTI Line4 interrupt */
#endif
#ifdef ENABLE_EXTI9_5
void EXTI9_5_IRQHandler(void);             			/* EXTI Line[9:5] interrupts */
#endif
#ifdef ENABLE_EXTI15_10
void EXTI15_10_IRQHandler(void);         			/* EXTI Line[15:10] interrupts */
#endif




#ifdef ENABLE_SYSTEMTICK
void SysTick_Handler(void);
#endif


#ifdef __cplusplus
}
#endif

#endif /* IT_F4XX_H_ */
