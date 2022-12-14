/*
 * TIM_F4xx.cpp
 *
 *  Created on: Dec 13, 2022
 *      Author: anh
 */

#include "PERIPH_USED.h"
#ifdef ENABLE_TIM

#include "RCC_F4xx.h"
#include "GPIO_F4xx.h"
#include "TIM_F4xx.h"

#define UIF_TIMEOUT 100U


TIM::TIM(TIM_TypeDef *Timer){
	_tim = Timer;
}

/* TIM Basic */
Result_t TIM::Init(TIM_Config_t *conf){
	Result_t res = {OKE};
	Result_Init(&res, OKE, 0U, __FUNCTION__, __FILE__);
	IRQn_Type IRQn;

	_conf = conf;
	_dma = _conf -> Dma;

	if     (_tim == TIM1)  RCC -> APB2ENR |= RCC_APB2ENR_TIM1EN;
	else if(_tim == TIM2)  RCC -> APB1ENR |= RCC_APB1ENR_TIM2EN;
	else if(_tim == TIM3)  RCC -> APB1ENR |= RCC_APB1ENR_TIM3EN;
	else if(_tim == TIM4)  RCC -> APB1ENR |= RCC_APB1ENR_TIM4EN;
	else if(_tim == TIM5)  RCC -> APB1ENR |= RCC_APB1ENR_TIM5EN;
	else if(_tim == TIM6)  RCC -> APB1ENR |= RCC_APB1ENR_TIM6EN;
	else if(_tim == TIM7)  RCC -> APB1ENR |= RCC_APB1ENR_TIM7EN;
	else if(_tim == TIM8)  RCC -> APB2ENR |= RCC_APB2ENR_TIM8EN;
	else if(_tim == TIM9)  RCC -> APB2ENR |= RCC_APB2ENR_TIM9EN;
	else if(_tim == TIM10) RCC -> APB2ENR |= RCC_APB2ENR_TIM10EN;
	else if(_tim == TIM11) RCC -> APB2ENR |= RCC_APB2ENR_TIM11EN;
	else if(_tim == TIM12) RCC -> APB1ENR |= RCC_APB1ENR_TIM12EN;
	else if(_tim == TIM13) RCC -> APB1ENR |= RCC_APB1ENR_TIM13EN;
	else   				   RCC -> APB1ENR |= RCC_APB1ENR_TIM14EN;

	/* BASIC TIMER */
	_tim -> CR1 |= (_conf -> tim_direction << TIM_CR1_DIR_Pos) | (_conf -> tim_autoreloadpreload << TIM_CR1_ARPE_Pos);

	_tim -> ARR = _conf -> tim_reloadvalue - 1;
	_tim -> PSC = _conf -> tim_prescaler - 1;

	_tim -> EGR = TIM_EGR_UG;
	/* TIMER PWM MODE */
	if(_conf -> tim_mode == TIM_PWM){
		GPIO_PortClockEnable(_conf -> tim_pwmport);

		GPIOAlternateFunction_t func;
		if(_tim == TIM1 || _tim == TIM2) func = AF1_TIM1_2;
		else if(_tim == TIM3 || _tim == TIM4 || _tim == TIM5) func = AF2_TIM3_5;
		else if(_tim == TIM8 || _tim == TIM9 || _tim == TIM10 || _tim == TIM11) func = AF3_TIM8_11;
		else if(_tim == TIM12 || _tim == TIM13 || _tim == TIM14) func = AF9_CAN1_2_TIM12_14;
		else {
			Set_Status_Line(&res, NOTSUPPORT, __LINE__);
			return res;
		}
		GPIO_AlternateFunction(_conf -> tim_pwmport, _conf -> tim_pwmpin, func);

		if(_conf -> tim_channel < TIM_CHANNEL3){ // Channel 1-2
			_tim -> CCMR1 |=  (TIM_CCMR1_OC1PE << (_conf -> tim_channel*8)); // Set PE.
			_tim -> CCMR1 &=~ (TIM_CCMR1_OC1FE << (_conf -> tim_channel*8)); // Clear FE.
			_tim -> CCMR1 |=  ((_conf -> tim_pwmmode << TIM_CCMR1_OC1M_Pos) << (_conf -> tim_channel*8)); // Set 6UL to OCxM.
		}
		else{ // Channel 3-4
			_tim -> CCMR2 |=  (TIM_CCMR2_OC3PE << ((_conf -> tim_channel - 2)*8)); // Set PE.
			_tim -> CCMR2 &=~ (TIM_CCMR2_OC3FE << ((_conf -> tim_channel - 2)*8)); // Clear FE.
			_tim -> CCMR2 |=  ((_conf -> tim_pwmmode << TIM_CCMR2_OC3M_Pos) << ((_conf -> tim_channel - 2)*8)); // Set PWM mode1 or mode2 (mode1 notinvert: 6, mode2 invert: 7) to OCxM.
		}

		_tim -> CCER |= (TIM_CCER_CC1E << (_conf -> tim_channel*4));
	}

	/* TIMER ENCODER MODE */
	if(_conf -> tim_mode == TIM_ENCODER){
		GPIO_PortClockEnable(_conf -> tim_enct1port);
		GPIO_PortClockEnable(_conf -> tim_enct2port);

		GPIO_Init(_conf -> tim_enct1port, _conf -> tim_enct1pin, GPIO_INPUT_PULLUP);
		GPIO_Init(_conf -> tim_enct2port, _conf -> tim_enct2pin, GPIO_INPUT_PULLUP);

		_tim -> ARR = 0xFFFE;
		_tim -> PSC = 0;

		_tim -> SMCR |= (3UL << TIM_SMCR_SMS_Pos);
		_tim -> CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
		_tim -> CCMR1 &=~ (TIM_CCMR1_IC1PSC | TIM_CCMR1_IC2PSC);
		_tim -> CCMR1 &=~ (TIM_CCMR1_IC1F | TIM_CCMR1_IC2F);

		_tim -> CCER |= TIM_CCER_CC1P | TIM_CCER_CC2P;
		_tim -> CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;

	}

	/* TIMER IO COMPARE MODE */
	if(_conf -> tim_mode == TIM_IOCOMPARE){
		/* NOTHING TO DO */
	}


	_tim -> CR1 |= TIM_CR1_CEN;

	if(_conf -> tim_mode != TIM_ENCODER){
		res = WaitFlagTimeout(&(_tim -> SR), TIM_SR_UIF, FLAG_SET, UIF_TIMEOUT);
		if(!CheckResult(res)) Set_Line(&res, __LINE__);
	}
	while(!(_tim -> SR & TIM_SR_UIF));
	_tim -> SR = 0x00U;


	if(_conf -> tim_interrupt == TIM_INTERRUPT_ENABLE){
		if(_conf -> tim_mode == TIM_ENCODER) _tim -> DIER |= TIM_DIER_CC1IE | TIM_DIER_CC2IE;
		else _tim -> DIER |= TIM_DIER_UIE;

		if	   (_tim == TIM1)  IRQn = TIM1_UP_TIM10_IRQn;
		else if(_tim == TIM2)  IRQn = TIM2_IRQn;
		else if(_tim == TIM3)  IRQn = TIM3_IRQn;
		else if(_tim == TIM4)  IRQn = TIM4_IRQn;
		else if(_tim == TIM5)  IRQn = TIM5_IRQn;
		else if(_tim == TIM6)  IRQn = TIM6_DAC_IRQn;
		else if(_tim == TIM7)  IRQn = TIM7_IRQn;
		else if(_tim == TIM8)  IRQn = TIM8_UP_TIM13_IRQn;
		else if(_tim == TIM9)  IRQn = TIM1_BRK_TIM9_IRQn;
		else if(_tim == TIM10) IRQn = TIM1_UP_TIM10_IRQn;
		else if(_tim == TIM11) IRQn = TIM1_TRG_COM_TIM11_IRQn;
		else if(_tim == TIM12) IRQn = TIM8_BRK_TIM12_IRQn;
		else if(_tim == TIM13) IRQn = TIM8_UP_TIM13_IRQn;
		else 				   IRQn = TIM8_TRG_COM_TIM14_IRQn;

		__NVIC_SetPriority(IRQn, _conf -> tim_interruptpriority);
		__NVIC_EnableIRQ(IRQn);

		/* TIMER ENCODER NEED TO ENABLE CAPTURE COMPARE INTERRUPT */
		if(_conf -> tim_mode == TIM_ENCODER){
			if	   (_tim == TIM1)  IRQn = TIM1_CC_IRQn;
			else if(_tim == TIM8)  IRQn = TIM8_CC_IRQn;

			__NVIC_SetPriority(IRQn, _conf -> tim_interruptpriority);
			__NVIC_EnableIRQ(IRQn);
		}
	}

	return res;
}

Result_t TIM::Event_Register_Handler(void(*event_handler)(void *parameter, TIM_Event_t event), void *parameter){
	Result_t res = {OKE};
	Result_Init(&res, OKE, 0U, __FUNCTION__, __FILE__);

	if(_conf -> tim_interrupt != TIM_INTERRUPT_ENABLE) {
		Set_Status_Line(&res, NOTSUPPORT, __LINE__);
		return res;
	}

	Param = parameter;
	Event_Callback = event_handler;

	return res;
}

void TIM::Set_Prescaler (uint32_t psc){
	_conf -> tim_prescaler = psc;
	_tim -> PSC = psc;
}
void TIM::Set_AutoReload(uint32_t arl){
	_conf -> tim_reloadvalue = arl;
	_tim -> ARR = arl;
}

void TIM::Clear_UpdateINTR(void){
	_tim -> SR &=~ TIM_DIER_UIE;
}

void TIM::ResetCounter(void){
	_tim -> CNT = 0;
}

uint16_t TIM::GetCounter(void){
	return _tim -> CNT;
}

void TIM::Delay_us(uint32_t us){
	_tim -> CNT = 0;
	while(_tim -> CNT < us);
}

void TIM::Delay_ms(uint32_t ms){
	for (uint32_t i=0; i<ms; i++)
		Delay_us(1000);
}

Result_t TIM::Start_DMA(uint16_t *count_buf, uint16_t size){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	_tim -> CR1  &=~ TIM_CR1_CEN;
	_tim -> DIER &=~ TIM_DIER_CC1DE;

	res = _dma -> Start((uint32_t)count_buf, (uint32_t)&_tim -> CNT, size);
	if(res.Status != OKE) {
		res.CodeLine = __LINE__;
		return res;
	}

	_tim -> DIER |= TIM_DIER_CC1DE;
	_tim -> CR1  |= TIM_CR1_CEN;

	return res;
}

Result_t TIM::Stop_DMA(void){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	if(_tim -> DIER & TIM_DIER_CC1DE){
		res = _dma -> Stop();
		if(res.Status != OKE) {
			res.CodeLine = __LINE__;
			return res;
		}
		_tim -> DIER &=~ TIM_DIER_CC1DE;
		_tim -> CR1  &=~ TIM_CR1_CEN;
	}
	else{
		res.CodeLine = __LINE__;
		res.Status = ERR;
	}

	return res;
}


/* TIMER PWM MODE */
Result_t TIM::PWM_SetDuty(uint16_t pwm){
	Result_t res = {OKE};
	Result_Init(&res, OKE, 0U, __FUNCTION__, __FILE__);

	if(_conf -> tim_mode != TIM_PWM) {
		Set_Status_Line(&res, NOTSUPPORT, __LINE__);
		return res;
	}

	switch(_conf -> tim_channel){
		case TIM_CHANNEL1:
			_tim -> CCR1 = pwm;
		break;
		case TIM_CHANNEL2:
			_tim -> CCR2 = pwm;
		break;
		case TIM_CHANNEL3:
			_tim -> CCR3 = pwm;
		break;
		case TIM_CHANNEL4:
			_tim -> CCR4 = pwm;
		break;
		default:
		break;
	};

	return res;
}

Result_t TIM::PWM_Start_DMA(uint16_t *pwm, uint16_t size){
	Result_t res = {OKE};
	Result_Init(&res, OKE, 0U, __FUNCTION__, __FILE__);

	_tim -> CCER &=~ (TIM_CCER_CC1E << (_conf -> tim_channel*4));
	_tim -> DIER &=~ TIM_DIER_CC1DE << _conf -> tim_channel;
	_tim -> CR1  &=~ TIM_CR1_CEN;

	uint32_t CCRx_addr = 0x00U;
	switch(_conf -> tim_channel){
		case TIM_CHANNEL1:
			CCRx_addr = (uint32_t)&_tim -> CCR1;
		break;
		case TIM_CHANNEL2:
			CCRx_addr = (uint32_t)&_tim -> CCR2;
		break;
		case TIM_CHANNEL3:
			CCRx_addr = (uint32_t)&_tim -> CCR3;
		break;
		case TIM_CHANNEL4:
			CCRx_addr = (uint32_t)&_tim -> CCR4;
		break;
		default:
		break;
	};
	res = _dma -> Start((uint32_t)pwm, CCRx_addr, size);
	if(!CheckResult(res)) {
		Set_Line(&res, __LINE__);
		return res;
	}
	_tim -> DIER |= TIM_DIER_CC1DE << _conf -> tim_channel;

	_tim -> CCER |= (TIM_CCER_CC1E << (_conf -> tim_channel*4));
	_tim -> CR1  |= TIM_CR1_CEN;

	return res;
}

Result_t TIM::PWM_Stop_DMA(void){
	Result_t res = {OKE};
	Result_Init(&res, OKE, 0U, __FUNCTION__, __FILE__);

	if(_tim -> DIER & TIM_DIER_CC1DE << _conf -> tim_channel){
		_tim -> DIER &=~ TIM_DIER_CC1DE << _conf -> tim_channel;
		res = _dma -> Stop();
		if(!CheckResult(res)) {
			Set_Line(&res, __LINE__);
			return res;
		}
		_tim -> CCER &=~ (TIM_CCER_CC1E << (_conf -> tim_channel*4));
		_tim -> CR1  &=~ TIM_CR1_CEN;
	}
	else{
		Set_Status_Line(&res, ERR, __LINE__);
	}

	return res;
}

/* TIMER ENCODER MODE */
void TIM::Encoder_ISR(void){
	if(_tim -> SR & TIM_SR_CC1IF){
		_tim -> SR =~ TIM_SR_CC1IF;
		counter = _tim -> CNT;
	}
	if(_tim -> SR & TIM_SR_CC2IF){
		_tim -> SR =~ TIM_SR_CC2IF;
		counter = _tim -> CNT;
	}
}

int16_t TIM::Encoder_GetValue(void){
	return (int16_t)((int16_t)counter/4);
}


void TIM_IRQHandler(TIM *tim){
	TIM_Event_t event = TIM_NoEvent;

	tim -> counter = tim -> _tim -> CNT;

	/* CHƯA HOÀN THIỆN PHẦN INPUT-OUTPUT CAPTURE */
	/* TIMER CAPTURE-COMPARE 1 INTERRUPT */
	if(tim -> _tim -> SR & TIM_SR_CC1IF && tim -> _tim -> DIER & TIM_DIER_CC1DE){
		tim -> _tim -> SR =~ TIM_SR_CC1IF;
		event = TIM_CaptureCompare1_Event;
		goto EventCB;
	}

	/* TIMER CAPTURE-COMPARE 2 INTERRUPT */
	if(tim -> _tim -> SR & TIM_SR_CC2IF && tim -> _tim -> DIER & TIM_DIER_CC2DE){
		tim -> _tim -> SR =~ TIM_SR_CC2IF;
		event = TIM_CaptureCompare2_Event;
		goto EventCB;
	}

	/* TIMER CAPTURE-COMPARE 3 INTERRUPT */
	if(tim -> _tim -> SR & TIM_SR_CC3IF && tim -> _tim -> DIER & TIM_DIER_CC3DE){
		tim -> _tim -> SR =~ TIM_SR_CC3IF;
		event = TIM_CaptureCompare3_Event;
		goto EventCB;
	}

	/* TIMER CAPTURE-COMPARE 4 INTERRUPT */
	if(tim -> _tim -> SR & TIM_SR_CC4IF && tim -> _tim -> DIER & TIM_DIER_CC4DE){
		tim -> _tim -> SR =~ TIM_SR_CC4IF;
		event = TIM_CaptureCompare4_Event;
		goto EventCB;
	}

	/* TIMER UPDATE INTERRUPT */
	if(tim -> _tim -> SR & TIM_SR_UIF && tim -> _tim -> DIER & TIM_DIER_UIE){
		tim -> _tim -> SR =~ TIM_SR_UIF;
		event = TIM_Update_Event;
		goto EventCB;
	}

	/* TIMER BREAK INTERRUPT */
	if(tim -> _tim -> SR & TIM_SR_BIF && tim -> _tim -> DIER & TIM_DIER_BIE){
		tim -> _tim -> SR =~ TIM_SR_BIF;
		event = TIM_Break_Event;
		goto EventCB;
	}

	/* TIMER TRIGER INTERRUPT */
	if(tim -> _tim -> SR & TIM_SR_TIF && tim -> _tim -> DIER & TIM_DIER_TIE){
		tim -> _tim -> SR =~ TIM_SR_TIF;
		event = TIM_Triger_Event;
		goto EventCB;
	}

	EventCB:
	tim -> Event_Callback(tim -> Param, event);
}

#ifdef ENABLE_TIM1
TIM tim1(TIM1);
#endif

#ifdef ENABLE_TIM2
TIM tim2(TIM2);
#endif

#ifdef ENABLE_TIM3
TIM tim3(TIM3);
#endif

#ifdef ENABLE_TIM4
TIM tim4(TIM4);
#endif

#ifdef ENABLE_TIM5
TIM tim5(TIM5);
#endif

#ifdef ENABLE_TIM6
TIM tim6(TIM6);
#endif

#ifdef ENABLE_TIM7
TIM tim7(TIM7);
#endif

#ifdef ENABLE_TIM8
TIM tim8(TIM8);
#endif

#ifdef ENABLE_TIM9
TIM tim9(TIM9);
#endif

#ifdef ENABLE_TIM10
TIM tim10(TIM10);
#endif

#ifdef ENABLE_TIM11
TIM tim11(TIM11);
#endif

#ifdef ENABLE_TIM12
TIM tim12(TIM12);
#endif

#ifdef ENABLE_TIM13
TIM tim13(TIM13);
#endif

#ifdef ENABLE_TIM14
TIM tim14(TIM14);
#endif


#endif
















