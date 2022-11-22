/*
 * DMA_F4xx.cpp
 *
 *  Created on: Nov 11, 2022
 *      Author: anh
 */

#include "PERIPH_USED.h"

#ifdef ENABLE_DMA

#include "DMA_F4xx.h"
#include "RCC_F4xx.h"
#include "STM_LOG.h"


static const uint8_t Channel_Index[8U] = {0U, 6U, 16U, 22U, 0U, 6U, 16U, 22U};

DMA::DMA(DMA_TypeDef *dma){
	_dma = dma;
}

void DMA::ClearIFCR(__IO uint32_t Value){
	(Stream < 4)? (_dma -> LIFCR = Value) : (_dma -> HIFCR = Value);
}
void DMA::ClearAllIntrFlag(void){
	ClearIFCR((0x3FU << _Intr_Index));
}

__IO uint32_t DMA::GetISR(void){
	__IO uint32_t isr = 0;
	(Stream < 4)? (isr = _dma -> LISR) : (isr = _dma -> HISR);
	return isr;
}

Result_t DMA::Init(DMA_Config_t *conf){
	Result_t res = {OKE};
	__IO uint32_t tmpreg;
	_conf = conf;
	Set_Result_State(&res, OKE, 0U, __FUNCTION__, __FILE__);

	RCC -> AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;

	_conf -> dma_stream -> CR &=~ DMA_SxCR_EN;
	res = WaitFlagTimeout(&(_conf -> dma_stream -> CR), DMA_SxCR_EN, FLAG_RESET, 500U);
	if(!CheckResult(res)){Set_Result(&res, __LINE__, __FUNCTION__, __FILE__);
//		return res;
	}

	tmpreg = (_conf -> dma_stream) -> CR;
	tmpreg &= ~(DMA_SxCR_CHSEL | DMA_SxCR_MBURST | DMA_SxCR_PBURST |
				DMA_SxCR_CT    | DMA_SxCR_DBM    | DMA_SxCR_PL     |
			    DMA_SxCR_MSIZE | DMA_SxCR_PSIZE  | DMA_SxCR_MINC   |
			    DMA_SxCR_PINC  | DMA_SxCR_CIRC   | DMA_SxCR_DIR    |
			    DMA_SxCR_PFCTRL| DMA_SxCR_TCIE   | DMA_SxCR_HTIE   |
				DMA_SxCR_TEIE  | DMA_SxCR_DMEIE  | DMA_SxCR_EN     );

	tmpreg |= (uint32_t)((_conf->dma_channel << DMA_SxCR_CHSEL_Pos) | (_conf->dma_channel_priority << DMA_SxCR_PL_Pos) |
						 (_conf-> dma_data_size << DMA_SxCR_PSIZE_Pos) | (_conf->dma_data_size << DMA_SxCR_MSIZE_Pos)  |
						 DMA_SxCR_MINC | (_conf->dma_mode << DMA_SxCR_CIRC_Pos) | (_conf->dma_direction << DMA_SxCR_DIR_Pos));

	if(_conf->dma_fifo == DMA_FIF0) {tmpreg |= (_conf->dma_burst << DMA_SxCR_MBURST_Pos) | (_conf->dma_burst << DMA_SxCR_PBURST_Pos);}
	(_conf -> dma_stream) -> CR = tmpreg;

	tmpreg = (_conf -> dma_stream) -> FCR;
	tmpreg &=~ (DMA_SxFCR_DMDIS | DMA_SxFCR_FTH);
	tmpreg |= _conf -> dma_fifo;
	if(_conf -> dma_fifo == DMA_FIF0) tmpreg |= DMA_SxFCR_FTH;
	_conf -> dma_stream -> FCR = tmpreg;

	Stream = (((uint32_t)_conf -> dma_stream & 0xFFU) - 16U) / 24U;
	_Intr_Index = Channel_Index[Stream];


	ClearAllIntrFlag();

	if(_conf->dma == DMA1){
		if(Stream == 7U) _IRQn = DMA1_Stream7_IRQn;
		else _IRQn = (IRQn_Type)(Stream + 11U);
	}
	else{
		if(Stream > 4U) _IRQn = (IRQn_Type)(Stream + 68U);
		else _IRQn = (IRQn_Type)(Stream + 56U);
	}

	__NVIC_SetPriority(_IRQn, _conf -> dma_interrupt_priority);
	__NVIC_EnableIRQ(_IRQn);

//	STM_LOG_REG((char *)"DMA_CR", _conf -> dma_stream -> CR);
//	STM_LOG_REG((char *)"DMA_FCR", _conf -> dma_stream -> FCR);
	STM_LOG(BOLD_YELLOW, "DMA", "CH: %d",  _conf -> dma_channel);
	STM_LOG(BOLD_YELLOW, "DMA", "DIR: %d",  _conf -> dma_direction);
	STM_LOG(BOLD_YELLOW, "DMA", "Mode: %d", _conf -> dma_mode);
	STM_LOG(BOLD_YELLOW, "DMA", "Size: %d",  _conf -> dma_data_size);
	STM_LOG(BOLD_YELLOW, "DMA", "fifo: %d", _conf -> dma_fifo);
	STM_LOG(BOLD_YELLOW, "DMA", "priority: %d",  _conf -> dma_channel_priority);
	STM_LOG(BOLD_YELLOW, "DMA", "Intr Select: %d", _conf -> dma_interrupt_select);
	STM_LOG(BOLD_YELLOW, "DMA", "Intr priorty: %d", _conf -> dma_interrupt_priority);
	STM_LOG_RES(res);

	_state = READY;

	return res;
}

Result_t DMA::Start(uint32_t Src_Address, uint32_t Dest_Address, uint32_t Number_Data){
	Result_t res = {OKE};
	Set_Result_State(&res, OKE, 0U, __FUNCTION__, __FILE__);

	if(_state == READY){
		STM_LOG(BOLD_YELLOW, "DMA", "CH: %d",  _conf -> dma_channel);
		STM_LOG(BOLD_RED, "DMA", "DIR: %d",  _conf -> dma_direction);
		STM_LOG(BOLD_RED, "DMA", "Mode: %d", _conf -> dma_mode);
		STM_LOG(BOLD_RED, "DMA", "Size: %d",  _conf -> dma_data_size);
		STM_LOG(BOLD_RED, "DMA", "fifo: %d", _conf -> dma_fifo);
		STM_LOG(BOLD_RED, "DMA", "priority: %d",  _conf -> dma_channel_priority);
		STM_LOG(BOLD_RED, "DMA", "Intr Select: %d", _conf -> dma_interrupt_select);
		STM_LOG(BOLD_RED, "DMA", "Intr priorty: %d", _conf -> dma_interrupt_priority);

		(_conf -> dma_stream) -> CR &=~ (DMA_SxCR_EN);
		res = WaitFlagTimeout(&(_conf -> dma_stream -> CR), DMA_SxCR_EN, FLAG_RESET, 500U);
		if(!CheckResult(res)){Set_Result(&res, __LINE__, __FUNCTION__, __FILE__);
	//		return res;
		}


		_conf -> dma_stream -> CR &=~ DMA_SxCR_DBM;
		_conf -> dma_stream -> NDTR = Number_Data;
		if((_conf -> dma_direction) == DMA_MEM_TO_PERIPH){
			_conf -> dma_stream -> PAR = Dest_Address;
			_conf -> dma_stream -> M0AR = Src_Address;
			STM_LOG(BOLD_YELLOW, "DMA", "CH: %d",  _conf -> dma_channel);
			STM_LOG(BOLD_CYAN, "DMA", "DIR: %d",  _conf -> dma_direction);
			STM_LOG(BOLD_CYAN, "DMA", "Mode: %d", _conf -> dma_mode);
			STM_LOG(BOLD_CYAN, "DMA", "Size: %d",  _conf -> dma_data_size);
			STM_LOG(BOLD_CYAN, "DMA", "fifo: %d", _conf -> dma_fifo);
			STM_LOG(BOLD_CYAN, "DMA", "priority: %d",  _conf -> dma_channel_priority);
			STM_LOG(BOLD_CYAN, "DMA", "Intr Select: %d", _conf -> dma_interrupt_select);
			STM_LOG(BOLD_CYAN, "DMA", "Intr priorty: %d", _conf -> dma_interrupt_priority);
		}
		else {
			_conf -> dma_stream -> PAR = Src_Address;
			_conf -> dma_stream -> M0AR = Dest_Address;

		}
		ClearAllIntrFlag();

		_conf -> dma_stream -> CR  |= _conf -> dma_interrupt_select | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;
		_conf -> dma_stream -> CR |= DMA_SxCR_EN;
		_state = BUSY;
	}
	else{
		Set_Result_State(&res, BUSY, __LINE__, __FUNCTION__, __FILE__);
		return res;
	}

	return res;
}

Result_t DMA::Stop(void){
	Result_t res = {OKE};
	Set_Result_State(&res, OKE, 0U, __FUNCTION__, __FILE__);

	if(_state == BUSY){
		_conf -> dma_stream -> CR  &= ~(DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE | DMA_SxCR_HTIE);
		_conf -> dma_stream -> FCR &=~ DMA_SxFCR_FEIE;
		_conf -> dma_stream -> CR  &=~ DMA_SxCR_EN;

		res = WaitFlagTimeout(&(_conf -> dma_stream -> CR), DMA_SxCR_EN, FLAG_RESET, 5U);
		if(!CheckResult(res)){Set_Result(&res, __LINE__, __FUNCTION__, __FILE__);
			return res;
		}

		ClearAllIntrFlag();
		_state = READY;
	}
	else{
		Set_Result_State(&res, ERR, __LINE__, __FUNCTION__, __FILE__);
		return res;
	}

	return res;
}
/*
Result_t DMA::PollForTranfer(DMA_InterruptSelect_t PollLevel, uint32_t TimeOut){
	Result_t res = {OKE};
	__IO uint32_t PollValue = 0U, tick, isr;

	Set_Result(&res, 0U, __FUNCTION__, __FILE__);

	if(_state != BUSY){
		Set_Result_State(&res, BUSY, __LINE__, __FUNCTION__, __FILE__);

		return res;
	}

	if(_conf -> dma_stream -> CR & DMA_SxCR_CIRC){
		Set_Result_State(&res, NOTSUPPORT, __LINE__, __FUNCTION__, __FILE__);

		return res;
	}

	if(PollLevel == DMA_TRANSFER_COMPLETE_INTERRUPT){
		PollValue = (uint32_t)(0x20U << _Intr_Index);
	}
	else if(PollLevel == DMA_HALF_TRANSFER_INTERRUPT){
		PollValue = (uint32_t)(0x20U << _Intr_Index);
	}
	else{
		Set_Result_State(&res, ERR, __LINE__, __FUNCTION__, __FILE__);

		return res;
	}

	tick = GetTick();
	isr = GetISR();
	while(!(isr & PollValue)){
		if(TimeOut != NO_TIMEOUT){
			if(GetTick() - tick > TimeOut){
				Set_Result_State(&res, TIMEOUT, __LINE__, __FUNCTION__, __FILE__);

				return res;
			}
		}
		// CHECK TRANFER ERROR.
		isr = GetISR();
		if(isr & (0x01U << _Intr_Index)){ // FEIE.
			ClearIFCR(0x01U << _Intr_Index);
			break;
		}
		if(isr & (0x04U << _Intr_Index)){ // DMEIE.
			ClearIFCR(0x04U << _Intr_Index);
			break;
		}
		if(isr & (0x08U << _Intr_Index)){ // TEIE.
			ClearIFCR(0x08U << _Intr_Index);
			break;
		}
	}

	if(isr & (0x08U << _Intr_Index)){
		Stop();
		ClearIFCR((DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0) << _Intr_Index);
		Set_Result_State(&res, ERR, __LINE__, __FUNCTION__, __FILE__);

		return res;
	}
	if(PollLevel == DMA_TRANSFER_COMPLETE_INTERRUPT){
		ClearIFCR((DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0) << _Intr_Index);
		_state = READY;
	}
	else{
		ClearIFCR(DMA_LIFCR_CHTIF0 << _Intr_Index);
	}

	return res;
}
*/

/* DMA1 IRQ HANDLER */
#ifdef ENABLE_DMA1_STREAM0
void DMA1_Stream0_IRQHandler(void){
	if((DMA1_Stream0 -> CR & DMA_SxCR_HTIE) && (DMA1 -> LISR & DMA_LISR_HTIF0)){
		if(!(DMA1_Stream0 -> CR & DMA_SxCR_CIRC)) DMA1_Stream0 -> CR &=~ DMA_SxCR_HTIE;
		DMA1 -> LIFCR = DMA_LIFCR_CHTIF0;
		DMA1_Stream0_HalfTranfer_CallBack();
	}
	else if((DMA1_Stream0 -> CR & DMA_SxCR_TCIE) && (DMA1 -> LISR & DMA_LISR_TCIF0)){
		if(!(DMA1_Stream0 -> CR & DMA_SxCR_CIRC)) DMA1_Stream0 -> CR &=~ (DMA_SxCR_TEIE | DMA_SxCR_TCIE);
		DMA1 -> LIFCR = DMA_LIFCR_CTCIF0;
		DMA1_Stream0_TranferComplete_CallBack();
	}
	else if((DMA1_Stream0 -> CR & DMA_SxCR_TEIE) && (DMA1 -> LISR & DMA_LISR_TEIF0)){
		DMA1_Stream0 -> CR &=~ (DMA_SxCR_TEIE | DMA_SxCR_TCIE | DMA_SxCR_HTIE);
		DMA1 -> LIFCR = (0x3FU << 0U);
		DMA1_Stream0_TranferError_CallBack();
	}
}
__WEAK void DMA1_Stream0_TranferComplete_CallBack(void){}
__WEAK void DMA1_Stream0_HalfTranfer_CallBack(void){}
__WEAK void DMA1_Stream0_TranferError_CallBack(void){}
#endif
#ifdef ENABLE_DMA1_STREAM1
void DMA1_Stream1_IRQHandler(void){
	if((DMA1_Stream1 -> CR & DMA_SxCR_HTIE) && (DMA1 -> LISR & DMA_LISR_HTIF1)){
		if(!(DMA1_Stream1 -> CR & DMA_SxCR_CIRC)) DMA1_Stream1 -> CR &=~ DMA_SxCR_HTIE;
		DMA1 -> LIFCR = DMA_LIFCR_CHTIF1;
		DMA1_Stream1_HalfTranfer_CallBack();
	}
	else if((DMA1_Stream1 -> CR & DMA_SxCR_TCIE) && (DMA1 -> LISR & DMA_LISR_TCIF1)){
		if(!(DMA1_Stream1 -> CR & DMA_SxCR_CIRC)) DMA1_Stream1 -> CR &=~ (DMA_SxCR_TEIE | DMA_SxCR_TCIE);
		DMA1 -> LIFCR = DMA_LIFCR_CTCIF1;
		DMA1_Stream1_TranferComplete_CallBack();
	}
	else if((DMA1_Stream1 -> CR & DMA_SxCR_TEIE) && (DMA1 -> LISR & DMA_LISR_TEIF1)){
		DMA1_Stream1 -> CR &=~ (DMA_SxCR_TEIE | DMA_SxCR_TCIE | DMA_SxCR_HTIE);
		DMA1 -> LIFCR = (0x3FU << 6U);
		DMA1_Stream1_TranferError_CallBack();
	}
}
__WEAK void DMA1_Stream1_TranferComplete_CallBack(void){}
__WEAK void DMA1_Stream1_HalfTranfer_CallBack(void){}
__WEAK void DMA1_Stream1_TranferError_CallBack(void){}
#endif
#ifdef ENABLE_DMA1_STREAM2
void DMA1_Stream2_IRQHandler(void){

}
__WEAK void DMA1_Stream2_TranferComplete_CallBack(void){}
__WEAK void DMA1_Stream2_HalfTranfer_CallBack(void){}
__WEAK void DMA1_Stream2_TranferError_CallBack(void){}
#endif
#ifdef ENABLE_DMA1_STREAM3
void DMA1_Stream3_IRQHandler(void){

}
__WEAK void DMA1_Stream3_TranferComplete_CallBack(void){}
__WEAK void DMA1_Stream3_HalfTranfer_CallBack(void){}
__WEAK void DMA1_Stream3_TranferError_CallBack(void){}
#endif
#ifdef ENABLE_DMA1_STREAM4
void DMA1_Stream4_IRQHandler(void){
	if(DMA1 -> HISR & DMA_HISR_HTIF4){
		if(DMA1_Stream4 -> CR & DMA_SxCR_HTIE){
			DMA1 -> HIFCR = DMA_HIFCR_CHTIF4;
			if(!(DMA1_Stream4 -> CR & DMA_SxCR_CIRC)){
				DMA1_Stream4 -> CR &=~ DMA_SxCR_HTIE;
				DMA1_Stream4_HalfTranfer_CallBack();
			}
		}
	}
	if(DMA1 -> HISR & DMA_HISR_TCIF4){
		DMA1_Stream4 -> CR &=~ (DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE | DMA_SxCR_HTIE);
		DMA1_Stream4 -> FCR &=~ (DMA_SxFCR_FEIE);
		DMA1 -> HIFCR = (0x3F << 0U);
		if(!(DMA1_Stream4 -> CR & DMA_SxCR_CIRC)){
			DMA1_Stream4 -> CR &=~ DMA_SxCR_TCIE;
			DMA1_Stream4_TranferComplete_CallBack();
		}
	}
	if(DMA1 -> HISR & DMA_HISR_TEIF4){
		DMA1_Stream4 -> CR &=~ DMA_SxCR_TEIE;
		DMA1 -> HIFCR = DMA_HIFCR_CTEIF4;
		DMA1_Stream4_TranferError_CallBack();
	}
}
__WEAK void DMA1_Stream4_TranferComplete_CallBack(void){}
__WEAK void DMA1_Stream4_HalfTranfer_CallBack(void){}
__WEAK void DMA1_Stream4_TranferError_CallBack(void){}
#endif
#ifdef ENABLE_DMA1_STREAM5
void DMA1_Stream5_IRQHandler(void){

}
__WEAK void DMA1_Stream5_TranferComplete_CallBack(void){}
__WEAK void DMA1_Stream5_HalfTranfer_CallBack(void){}
__WEAK void DMA1_Stream5_TranferError_CallBack(void){}
#endif
#ifdef ENABLE_DMA1_STREAM6
void DMA1_Stream6_IRQHandler(void){
	if(DMA1 -> HISR & DMA_HISR_HTIF6){
		if(DMA1_Stream6 -> CR & DMA_SxCR_HTIE){
			DMA1 -> HIFCR = DMA_HIFCR_CHTIF6;
			if(!(DMA1_Stream6 -> CR & DMA_SxCR_CIRC)){
				DMA1_Stream6 -> CR &=~ DMA_SxCR_HTIE;
				DMA1_Stream6_HalfTranfer_CallBack();
			}
		}
	}
	if(DMA1 -> HISR & DMA_HISR_TCIF6){
		DMA1_Stream6 -> CR &=~ (DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE | DMA_SxCR_HTIE);
		DMA1_Stream6 -> FCR &=~ (DMA_SxFCR_FEIE);
		DMA1 -> HIFCR = (0x3F << 16U);
		if(!(DMA1_Stream6 -> CR & DMA_SxCR_CIRC)){
			DMA1_Stream6 -> CR &=~ DMA_SxCR_TCIE;
			DMA1_Stream6_TranferComplete_CallBack();
		}
	}
	if(DMA1 -> HISR & DMA_HISR_TEIF6){
		DMA1_Stream6 -> CR &=~ DMA_SxCR_TEIE;
		DMA1 -> HIFCR = DMA_HIFCR_CTEIF6;
		DMA1_Stream6_TranferError_CallBack();
	}
}
__WEAK void DMA1_Stream6_TranferComplete_CallBack(void){}
__WEAK void DMA1_Stream6_HalfTranfer_CallBack(void){}
__WEAK void DMA1_Stream6_TranferError_CallBack(void){}
#endif
#ifdef ENABLE_DMA1_STREAM7
void DMA1_Stream7_IRQHandler(void){

}
__WEAK void DMA1_Stream7_TranferComplete_CallBack(void){}
__WEAK void DMA1_Stream7_HalfTranfer_CallBack(void){}
__WEAK void DMA1_Stream7_TranferError_CallBack(void){}
#endif

/* DMA2 IRQ HANDLER */
#ifdef ENABLE_DMA2_STREAM0
void DMA2_Stream0_IRQHandler(void){

}
__WEAK void DMA2_Stream0_TranferComplete_CallBack(void){}
__WEAK void DMA2_Stream0_HalfTranfer_CallBack(void){}
__WEAK void DMA2_Stream0_TranferError_CallBack(void){}
#endif
#ifdef ENABLE_DMA2_STREAM1
void DMA2_Stream1_IRQHandler(void){

}
__WEAK void DMA2_Stream1_TranferComplete_CallBack(void){}
__WEAK void DMA2_Stream1_HalfTranfer_CallBack(void){}
__WEAK void DMA2_Stream1_TranferError_CallBack(void){}
#endif
#ifdef ENABLE_DMA2_STREAM2
void DMA2_Stream2_IRQHandler(void){

}
__WEAK void DMA2_Stream2_TranferComplete_CallBack(void){}
__WEAK void DMA2_Stream2_HalfTranfer_CallBack(void){}
__WEAK void DMA2_Stream2_TranferError_CallBack(void){}
#endif
#ifdef ENABLE_DMA2_STREAM3
void DMA2_Stream3_IRQHandler(void){

}
__WEAK void DMA2_Stream3_TranferComplete_CallBack(void){}
__WEAK void DMA2_Stream3_HalfTranfer_CallBack(void){}
__WEAK void DMA2_Stream3_TranferError_CallBack(void){}
#endif
#ifdef ENABLE_DMA2_STREAM4
void DMA2_Stream4_IRQHandler(void){

}
__WEAK void DMA2_Stream4_TranferComplete_CallBack(void){}
__WEAK void DMA2_Stream4_HalfTranfer_CallBack(void){}
__WEAK void DMA2_Stream4_TranferError_CallBack(void){}
#endif
#ifdef ENABLE_DMA2_STREAM5
void DMA2_Stream5_IRQHandler(void){

}
__WEAK void DMA2_Stream5_TranferComplete_CallBack(void){}
__WEAK void DMA2_Stream5_HalfTranfer_CallBack(void){}
__WEAK void DMA2_Stream5_TranferError_CallBack(void){}
#endif
#ifdef ENABLE_DMA2_STREAM6
void DMA2_Stream6_IRQHandler(void){

}
__WEAK void DMA2_Stream6_TranferComplete_CallBack(void){}
__WEAK void DMA2_Stream6_HalfTranfer_CallBack(void){}
__WEAK void DMA2_Stream6_TranferError_CallBack(void){}
#endif
#ifdef ENABLE_DMA2_STREAM7
void DMA2_Stream7_IRQHandler(void){

}
__WEAK void DMA2_Stream7_TranferComplete_CallBack(void){}
__WEAK void DMA2_Stream7_HalfTranfer_CallBack(void){}
__WEAK void DMA2_Stream7_TranferError_CallBack(void){}
#endif

#endif













