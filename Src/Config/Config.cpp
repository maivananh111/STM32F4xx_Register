/*
 * Config.cpp
 *
 *  Created on: Nov 7, 2022
 *      Author: anh
 */

#include "Config.h"
#include "string.h"
#include "stdio.h"


PowerConfig_t pwr = {
	.VoltScale2 = true,
	.ClockEnable = true,
};

FlashMemConfig_t flash = {
	.System_Clock = 168000000U,
	.Latency = 5U,
	.Prefetch = true,
	.Instruction_Cache = true,
	.Data_Cache  =true,
};

RCC_Config_t rcc = {
	.CLOCK_SOURCE          = HSE_CRYSTAL,
	.HSI_CRYSTAL_FREQUENCY = HSI_VALUE,
	.HSI_TRIM_VALUE        = 16U,
	.HSE_CRYSTAL_FREQUENCY = HSE_VALUE,
	.CLOCK_MUX             = PLLCLK,
	.SYSTEM_FREQUENCY      = 168000000U,
	.AHB_PRESCALER  	   = RCC_CFGR_HPRE_DIV1,
	.APB1_PRESCALER 	   = RCC_CFGR_PPRE1_DIV4,
	.APB2_PRESCALER 	   = RCC_CFGR_PPRE2_DIV2,
	rcc.PLL.PLLM = 25U,
	rcc.PLL.PLLN = 336U,
	rcc.PLL.PLLP = 2U,
	rcc.PLL.PLLQ = 4U,
};


DMA_Config_t uart_log_TxDma_conf = {
	.dma = DMA2,
	.dma_stream = DMA2_Stream7,
	.dma_channel = DMA_Channel4,
	.dma_direction = DMA_MEM_TO_PERIPH,
	.dma_mode = DMA_NORMAL,
	.dma_data_size = DMA_DATA8BIT,
	.dma_fifo = DMA_NOFIFO,
	.dma_channel_priority = DMA_CHANNEL_PRIORITY_HIGH,
	.dma_interrupt_select = DMA_TRANSFER_COMPLETE_INTERRUPT,
	.dma_interrupt_priority = 0,
};

DMA uart_log_TxDma(DMA2);
USART_Config_t uart_log_conf = {
	.Baudrate = 115200,
	.Type = USART_INTERRUPT_DMA,
	.InterruptSelect = USART_INTR_RX,
	.InterruptPriority = 1,
	.Port = GPIOB,
	.TxPin = 6,
	.RxPin = 7,
	.TxDma = &uart_log_TxDma,
};
USART *uart_log = &usart1;
volatile uint8_t dma_uart_log_tx_flag = 0;


DMA_Config_t fls_spi_RxDma_conf = {
	.dma = DMA2,
	.dma_stream = DMA2_Stream0,
	.dma_channel = DMA_Channel3,
	.dma_direction = DMA_PERIH_TO_MEM,
	.dma_mode = DMA_NORMAL,
	.dma_data_size = DMA_DATA8BIT,
	.dma_fifo = DMA_NOFIFO,
	.dma_channel_priority = DMA_CHANNEL_PRIORITY_LOW,
	.dma_interrupt_select = DMA_TRANSFER_COMPLETE_INTERRUPT,
	.dma_interrupt_priority = 0,
};

DMA fls_spi_RxDma(DMA2);
SPI_Config_t fls_spi_conf = {
	.spi_mode = SPI_FULLDUPLEXMASTER,
	.spi_type = SPI_NORMAL_DMA,
	.spi_datasize = SPI_DATA8BIT,
	.spi_dataformat = SPI_DATAMSB,
	.spi_clockdivision = SPI_CLOCKDIV4,
	.spi_clocksample = SPI_CPOL0_CPHA0,
	.spi_interruptpriority = 0,
	.spi_irq = SPI1_IRQn,
	.CLKPort = GPIOA,
	.CLKPin = 5,
	.MISOPort = GPIOA,
	.MISOPin = 6,
	.MOSIPort = GPIOA,
	.MOSIPin = 7,
	.RxDma = &fls_spi_RxDma,
};
SPI<uint8_t> fls_spi(SPI1);
SPIFLASH spiflash(GPIOE, 1);



I2C_Config_t i2c_conf = {
	.i2c_mode = I2C_FAST_MODE,
	.i2c_frequency = 400000UL,
	.SCLPort = GPIOB,
	.SCLPin = 6,
	.SDAPort = GPIOB,
	.SDAPin = 7,
};
I2C i2c(I2C1);

static Result_t res;
static const char * TAG = "Initialize";

uint8_t *Rxbuf;

static void STM_LOGSET(char *Buffer);


void Periph_Initialize(void){

	Result_Init(&res, OKE, __LINE__, __FUNCTION__, __FILE__);

	Power_Configuration(&pwr);
	FlashMem_Configuration(&flash);
	HSClock_Init(&rcc);

	Peripheral_Status_Init(&GetTick);

	GPIO_CLOCKENABLE();
	GPIO_Init(GPIOC, 13, GPIO_OUTPUT_PUSHPULL);

	uart_log_TxDma.Init(&uart_log_TxDma_conf);
	uart_log -> Init(&uart_log_conf);
	STM_LOG_Init(&STM_LOGSET);

	fls_spi_RxDma.Init(&fls_spi_RxDma_conf);
	fls_spi.Init(&fls_spi_conf);
//
//	i2c.Init(&i2c_conf);

}

void AppLayer_Initialize(void){

	STM_LOG(BOLD_GREEN, TAG, "STM LOG OKE");

	TickDelay_ms(1000);
	uint32_t ID = spiflash.Init(&fls_spi);
	STM_LOG(BOLD_GREEN, TAG, "Flash Initialize finish, ID: 0x%06x.", ID);

	uart_log -> ReceiveUntilIdleIT(50);

//	STM_LOG(BOLD_GREEN, TAG, "I2C1 Start scan.");
//	for(uint8_t i=0; i<128; i++){
//		res = i2c.CheckDevices((uint16_t)(i<<1), 3, 5);
//		if(CheckResult(res)){
//			STM_LOG(BOLD_CYAN, TAG, "Finded devices at 0x%02x", i);
//		}
//	}
//	STM_LOG(BOLD_YELLOW, TAG, "I2C1 End scan.");


}

static void STM_LOGSET(char *Buffer){
//	uart_log -> SendString(Buffer);
	uart_log -> TransmitDMA((uint8_t *)Buffer, (uint16_t)strlen(Buffer));
	while(!dma_uart_log_tx_flag);
	uart_log -> Stop_DMA();
	dma_uart_log_tx_flag = 0;
}

void DMA2_Stream0_TranferComplete_CallBack(void){
	dma_flash_rx_flag = 1;
}

void DMA2_Stream7_TranferComplete_CallBack(void){
	dma_uart_log_tx_flag = 1;
}

void USART1_Event_Callback(USART_Event_t event){

	switch(event){
		case USART_Event_ReceiveComplete:
			GPIO_Set(GPIOC, 13);
		break;

		case USART_Event_ReceiveEndChar:


		break;

		case USART_Event_IdleDetected:
			uart_log -> GetBuffer(&Rxbuf);
			STM_LOG(BOLD_YELLOW, TAG, "Idle, Receive: %s", Rxbuf);
			uart_log -> ReceiveUntilIdleIT(50);
//			free(Rxbuf);
		break;

		case USART_Event_TransmitComplete:

		break;

		default:

		break;

	}
}



