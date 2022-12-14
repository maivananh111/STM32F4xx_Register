/*
 * TIM_F4xx.h
 *
 *  Created on: Dec 13, 2022
 *      Author: anh
 */

#ifndef TIM_F4XX_H_
#define TIM_F4XX_H_


#include "PERIPH_USED.h"
#ifdef ENABLE_TIM


#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f4xx.h"
#include "PERIPH_STATUS.h"
#include "DMA_F4xx.h"


typedef enum{
	TIM_BASIC,
	TIM_PWM,
	TIM_ENCODER,
	TIM_IOCOMPARE,
} TIM_Mode_t;

typedef enum{
	TIM_CHANNEL1 = 0,
	TIM_CHANNEL2,
	TIM_CHANNEL3,
	TIM_CHANNEL4,
	TIM_NOCHANNEL,
} TIM_Channel_t;

typedef enum{
	TIM_COUNTER_UP = 0,
	TIM_COUNTER_DOWN,
} TIM_Direction_t;

typedef enum{
	TIM_ARP_DISABLE = 0,
	TIM_ARP_ENABLE,
} TIM_AutoReloadPreload_t;

typedef enum{
	TIM_INTERRUPT_DISABLE = 0,
	TIM_INTERRUPT_ENABLE = 1,
} TIM_Interrupt_t;

typedef enum{
	TIM_PWM_NOINVERT = 6,
	TIM_PWM_INVERT = 7,
} TIM_PWMMode_t;

typedef enum{
	TIM_Update_Event,
	TIM_Break_Event,
	TIM_Triger_Event,
	TIM_CaptureCompare1_Event,
	TIM_CaptureCompare2_Event,
	TIM_CaptureCompare3_Event,
	TIM_CaptureCompare4_Event,
	TIM_NoEvent,
}TIM_Event_t;

typedef struct{
	TIM_Mode_t    			tim_mode = TIM_BASIC;
	uint32_t 				tim_prescaler = 0U;
	uint32_t 				tim_reloadvalue = 0U;
	TIM_Channel_t 			tim_channel = TIM_NOCHANNEL;
	TIM_Direction_t         tim_direction =TIM_COUNTER_UP;
	TIM_AutoReloadPreload_t tim_autoreloadpreload = TIM_ARP_DISABLE;
	TIM_Interrupt_t         tim_interrupt = TIM_INTERRUPT_DISABLE;
	uint32_t                tim_interruptpriority = 0U;
	DMA                     *Dma = NULL;
	GPIO_TypeDef            *tim_pwmport = NULL;
	uint16_t 				tim_pwmpin = 0;
	GPIO_TypeDef            *tim_enct1port = NULL;
	uint16_t 				tim_enct1pin = 0;
	GPIO_TypeDef            *tim_enct2port = NULL;
	uint16_t 				tim_enct2pin = 0;
	TIM_PWMMode_t           tim_pwmmode = TIM_PWM_NOINVERT;
} TIM_Config_t;

class TIM{
	public:
		TIM(TIM_TypeDef *Timer);

		Result_t Init(TIM_Config_t *conf);
		Result_t Event_Register_Handler(void(*event_handler)(void *parameter, TIM_Event_t event), void *parameter = NULL);
		void Set_Prescaler (uint32_t psc);
		void Set_AutoReload(uint32_t arl);
		/* Basic TIMER */
		Result_t Start_DMA(uint16_t *count_ptr, uint16_t size);
		Result_t Stop_DMA(void);
		void ResetCounter(void);
		uint16_t GetCounter(void);
		void Delay_us(uint32_t us);
		void Delay_ms(uint32_t ms);

		/* TIMER PWM Mode */
		Result_t PWM_SetDuty(uint16_t pwm);
		Result_t PWM_Start_DMA(uint16_t *pwm, uint16_t size);
		Result_t PWM_Stop_DMA(void);

		/* TIMER Encoder Mode */
		int16_t Encoder_GetValue(void);
		void Encoder_ISR(void);

		void Clear_UpdateINTR(void);

		void (*Event_Callback)(void *parameter, TIM_Event_t event);
		TIM_TypeDef  *_tim;
		void *Param;
		volatile uint32_t counter = 0;

	private:
		TIM_Config_t *_conf;

		DMA *_dma = NULL;

};

void TIM_IRQHandler(TIM *tim);


#ifdef ENABLE_TIM1
extern TIM tim1;
#ifndef ENABLE_TIM9
void TIM1_BRK_TIM9_IRQHandler(void);
#endif
#ifndef ENABLE_TIM10
void TIM1_UP_TIM10_IRQHandler(void);
#endif
#ifndef ENABLE_TIM11
void TIM1_TRG_COM_TIM11_IRQHandler(void);
#endif
void TIM1_CC_IRQHandler(void);
#endif

#ifdef ENABLE_TIM2
extern TIM tim2;
void TIM2_IRQHandler(void);
#endif

#ifdef ENABLE_TIM3
extern TIM tim3;
void TIM3_IRQHandler(void);
#endif

#ifdef ENABLE_TIM4
extern TIM tim4;
void TIM4_IRQHandler(void);
#endif

#ifdef ENABLE_TIM5
extern TIM tim5;
void TIM5_IRQHandler(void);
#endif

#ifdef ENABLE_TIM6
extern TIM tim6;
void TIM6_DAC_IRQHandler(void);
#endif

#ifdef ENABLE_TIM7
extern TIM tim7;
void TIM7_IRQHandler(void);
#endif

#ifdef ENABLE_TIM8
extern TIM tim8;
#ifndef ENABLE_TIM12
void TIM8_BRK_TIM12_IRQHandler(void);
#endif
#ifndef ENABLE_TIM13
void TIM8_UP_TIM13_IRQHandler(void);
#endif
#ifndef ENABLE_TIM14
void TIM8_TRG_COM_TIM14_IRQHandler(void);
#endif
void TIM8_CC_IRQHandler(void);
#endif

#ifdef ENABLE_TIM9
extern TIM tim9;
void TIM1_BRK_TIM9_IRQHandler(void);
#endif

#ifdef ENABLE_TIM10
extern TIM tim10;
void TIM1_UP_TIM10_IRQHandler(void);
#endif

#ifdef ENABLE_TIM11
extern TIM tim11;
void TIM1_TRG_COM_TIM11_IRQHandler(void);
#endif

#ifdef ENABLE_TIM12
extern TIM tim12;
void TIM8_BRK_TIM12_IRQHandler(void);
#endif

#ifdef ENABLE_TIM13
extern TIM tim13;
void TIM8_UP_TIM13_IRQHandler(void);
#endif

#ifdef ENABLE_TIM14
extern TIM tim14;
void TIM8_TRG_COM_TIM14_IRQHandler(void);
#endif

#ifdef __cplusplus
}
#endif

#endif

#endif /* TIM_F4XX_H_ */
