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
	rcc.PLL.PLLM = 12U,
	rcc.PLL.PLLN = 168U,
	rcc.PLL.PLLP = 2U,
	rcc.PLL.PLLQ = 7U,
};

USART_Config_t uart_log_conf = {
	.Baudrate = 115200U,
	.Type = USART_NORMAL_DMA,
	.InterruptSelect = USART_INTR_RX,
	.InterruptPriority = 0,
	.Port = GPIOA,
	.TxPin = 2,
	.RxPin = 3,
};
USART uart_log(USART2);


DMA_Config_t fls_spi_TxDma_conf = {
	.dma = DMA1,
	.dma_stream = DMA1_Stream4,
	.dma_channel = DMA_Channel0,
	.dma_direction = DMA_MEM_TO_PERIPH,
	.dma_mode = DMA_NORMAL,
	.dma_data_size = DMA_DATA8BIT,
	.dma_fifo = DMA_NOFIFO,
	.dma_channel_priority = DMA_CHANNEL_PRIORITY_LOW,
	.dma_interrupt_select = DMA_TRANSFER_COMPLETE_INTERRUPT,
	.dma_interrupt_priority = 1,
};

DMA fls_spi_TxDma(DMA1);
SPI_Config_t fls_spi_conf = {
	.spi_mode = SPI_FULLDUPLEXMASTER,
	.spi_type = SPI_NORMAL_DMA,
	.spi_datasize = SPI_DATA8BIT,
	.spi_dataformat = SPI_DATAMSB,
	.spi_clockdivision = SPI_CLOCKDIV2,
	.spi_clocksample = SPI_CPOL0_CPHA0,
	.spi_interruptpriority = 0,
	.spi_irq = SPI2_IRQn,
	.CLKPort = GPIOB,
	.CLKPin = 10,
	.MISOPort = GPIOC,
	.MISOPin = 2,
	.MOSIPort = GPIOC,
	.MOSIPin = 3,
	.TxDma = &fls_spi_TxDma,
};
SPI<uint8_t> fls_spi(SPI2);
SPIFLASH spiflash(GPIOA, 8);



static const char * TAG = "Initialize";
static void STM_LOGSET(char *Buffer);


void Periph_Initialize(void){
	Power_Configuration(&pwr);
	FlashMem_Configuration(&flash);
	HSClock_Init(&rcc);

	Peripheral_Status_Init(&GetTick);

	GPIO_CLOCKENABLE();
	GPIO_Init(GPIOC, 13, GPIO_OUTPUT_PUSHPULL);

	uart_log.Init(&uart_log_conf);
	STM_LOG_Init(&STM_LOGSET);

	fls_spi_TxDma.Init(&fls_spi_TxDma_conf);
	fls_spi.Init(&fls_spi_conf);

}

void AppLayer_Initialize(void){

	STM_LOG(BOLD_GREEN, TAG, "STM LOG OKE");

	TickDelay_ms(1000);
	uint32_t ID = spiflash.Init(&fls_spi);
	spiflash.UnProtectedChip();
	ID = spiflash.Init(&fls_spi);
	STM_LOG(BOLD_GREEN, TAG, "Flash Initialize finish, ID: 0x%06x.", ID);




}

static void STM_LOGSET(char *Buffer){
	uart_log.SendString(Buffer);
}

void DMA1_Stream4_TranferComplete_CallBack(void){
	dma_flash_tx_flag = 1;
}



