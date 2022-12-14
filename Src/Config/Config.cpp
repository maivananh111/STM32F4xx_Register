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
	.power_voltscale2 = true,
	.power_clockenable = true,
};

FlashMemConfig_t flash = {
	.rcc_systemclock = 168000000U,
	.flash_latency = 5U,
	.flash_prefetch = true,
	.flash_instructioncache = true,
	.flash_datacache  =true,
};

RCC_Config_t rcc = {
	.rcc_clocksource          = HSE_CRYSTAL,
	.rcc_hsifrequency = HSI_VALUE,
	.rcc_hsitrim        = 16U,
	.rcc_hsefrequency = HSE_VALUE,
	.rcc_clockmux             = PLLCLK,
	.rcc_systemfrequency      = 168000000U,
	.rcc_ahbprescaler  	   = RCC_CFGR_HPRE_DIV1,
	.rcc_apb1prescaler 	   = RCC_CFGR_PPRE1_DIV4,
	.rcc_apb2prescaler 	   = RCC_CFGR_PPRE2_DIV2,
	rcc.rcc_pll.rcc_pllm = 25U,
	rcc.rcc_pll.rcc_plln = 336U,
	rcc.rcc_pll.rcc_pllp = 2U,
	rcc.rcc_pll.rcc_pllq = 7U,
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
	.dma_interrupt_priority = UART_TXDMA_PRIORITY,
};
DMA *uart_log_TxDma = &dma2_stream7;

USART_Config_t uart_log_conf = {
	.usart_baudrate = 115200,
	.usart_type = USART_INTERRUPT_DMA,
	.usart_interruptselect = USART_INTR_RX,
	.usart_interruptpriority = UART_PRIORITY,
	.TxPort = GPIOB,
	.TxPin = 6,
	.RxPort = GPIOB,
	.RxPin = 7,
	.TxDma = uart_log_TxDma,
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
	.dma_interrupt_priority = SPIFLS_RXDMA_PRIORITY,
};
DMA *fls_spi_RxDma = &dma2_stream0;

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
	.RxDma = fls_spi_RxDma,
};
SPI<uint8_t> fls_spi(SPI1);

SPIFLASH spiflash(GPIOE, 1);






TIM_Config_t lora_tim_conf = {
	.tim_mode = TIM_BASIC,
	.tim_prescaler = 42000U,
	.tim_reloadvalue = 4000U,
	.tim_direction =TIM_COUNTER_UP,
	.tim_autoreloadpreload = TIM_ARP_DISABLE,
	.tim_interrupt = TIM_INTERRUPT_ENABLE,
	.tim_interruptpriority = LORA_TIM_INTERRUPT_PRIORITY,
};
TIM *lora_tim = &tim2;

SPI_Config_t lora_spi_conf = {
	.spi_mode = SPI_FULLDUPLEXMASTER,
	.spi_type = SPI_NORMAL_DMA,
	.spi_datasize = SPI_DATA8BIT,
	.spi_dataformat = SPI_DATAMSB,
	.spi_clockdivision = SPI_CLOCKDIV4,
	.spi_clocksample = SPI_CPOL0_CPHA0,
	.spi_interruptpriority = 0,
	.spi_irq = SPI3_IRQn,
	.CLKPort = GPIOB,
	.CLKPin = 3,
	.MISOPort = GPIOB,
	.MISOPin = 4,
	.MOSIPort = GPIOB,
	.MOSIPin = 5,
};
SPI<uint8_t> lora_spi(SPI3);

LoRa lora(GPIOA, 8, GPIOA, 10, GPIOD, 3);

TIM_Config_t tim3_conf = {
	.tim_mode = TIM_BASIC,
	.tim_prescaler = 42000U,
	.tim_reloadvalue = 8000U,
	.tim_direction =TIM_COUNTER_UP,
	.tim_autoreloadpreload = TIM_ARP_DISABLE,
	.tim_interrupt = TIM_INTERRUPT_ENABLE,
	.tim_interruptpriority = LORA_TIM_INTERRUPT_PRIORITY,
};
TIM *timer3 = &tim3;


I2C_Config_t i2c1_conf = {
	.i2c_mode = I2C_FAST_MODE,
	.i2c_frequency = 400000UL,
	.SCLPort = GPIOB,
	.SCLPin = 8,
	.SDAPort = GPIOB,
	.SDAPin = 9,
};
I2C i2c1(I2C1);

DS3231_Time_t time = {
	.seconds = 0,
	.minutes = 37,
	.hour = 21,
	.dayofweek = 3,
	.dayofmonth = 13,
	.month = 12,
	.year = 22,
};


static Result_t res;
static const char * TAG = "LOG";
uint8_t *Rxbuf;
uint32_t count = 0, tick = 0;;

bool esp_ok = false;
bool RxFromEsp_Flag = false;

static void STM_LOGSET(char *Buffer);


void Periph_Initialize(void){
	Result_Init(&res, OKE, __LINE__, __FUNCTION__, __FILE__);

	Power_Configuration(&pwr);
	FlashMem_Configuration(&flash);
	RCC_SystemClock_Init(&rcc);


	gettick_Init(RCC_GetTick);
	delay_ms_Init(RCC_Delay_ms);
	Peripheral_Status_Init(gettick);


	GPIO_CLOCKENABLE();
	GPIO_Init(GPIOC, 13, GPIO_OUTPUT_PUSHPULL);
	GPIO_Init(GPIOC,  6, GPIO_OUTPUT_PUSHPULL);
	GPIO_Set(GPIOC, 6);


	uart_log_TxDma -> Init(&uart_log_TxDma_conf);
	uart_log_TxDma -> Event_Register_Handler(DMA2_Stream7_TranferComplete_CallBack, NULL);
	uart_log -> Init(&uart_log_conf);
	uart_log -> Event_Register_Handler(USART1_Event_Callback, NULL);
	STM_LOG_Init(STM_LOGSET);
	STM_LOG(BOLD_GREEN, TAG, "STM LOG OKE");


	ESP32_API_Init(uart_log, GPIOC, 4, GPIOC, 5);
	uart_log -> ReceiveUntilIdleIT(50);
	while(esp_ok == false){
		GPIO_Set(GPIOC, 13);
		ESP32_Hardware_Reset();
		delay_ms(1000);
	}
	GPIO_Reset(GPIOC, 13);


	fls_spi_RxDma -> Init(&fls_spi_RxDma_conf);
	fls_spi_RxDma -> Event_Register_Handler(DMA2_Stream0_TranferComplete_CallBack, NULL);
	fls_spi.Init(&fls_spi_conf);
	uint32_t ID = spiflash.Init(&fls_spi);
	STM_LOG(BOLD_GREEN, TAG, "Flash Initialize finish, ID: 0x%06x.", ID);


	timer3 -> Init(&tim3_conf);
	timer3 -> Event_Register_Handler(Timer3Callback, NULL);
	lora_spi.Init(&lora_spi_conf);
	if(lora.Init(&lora_spi, 433E6, 17, LORA_PRIORITY)) {
		STM_LOG(BOLD_GREEN, TAG, "Lora Initialize OKE.");
	}
	else STM_LOG(BOLD_RED, TAG, "Lora Initialize Failed.");
	lora.setSyncWord(0xFF);
	lora.Event_Register_Handler(NULL, LoRaCallback);
	lora.Receive(0);
	lora_tim -> Init(&lora_tim_conf);
	lora_tim -> Event_Register_Handler(LoRa_TimerCallback, NULL);
	LoRa_AP_Init(&lora);


	i2c1.Init(&i2c1_conf);
	STM_LOG(BOLD_GREEN, TAG, "I2C1 Start scan.");
	for(uint8_t i=0; i<128; i++){
		res = i2c1.CheckDevices((uint16_t)(i<<1), 3, 5);
		if(CheckResult(res)){
			STM_LOG(BOLD_CYAN, TAG, "Finded devices at 0x%02x", i);
		}
	}
	STM_LOG(BOLD_YELLOW, TAG, "I2C1 End scan.");

	DS3231_Init(&i2c1);
	DS3231_SetTime(time);

//	ESP32_WiFi_On("Redmi Note 9S", "khoavantue");
}

static void STM_LOGSET(char *Buffer){
	uart_log -> SendString(Buffer);
//	uart_log -> TransmitDMA((uint8_t *)Buffer, (uint16_t)strlen(Buffer));
//	while(!dma_uart_log_tx_flag);
//	dma_uart_log_tx_flag = 0;
}

void DMA2_Stream0_TranferComplete_CallBack(void *Paramemter, DMA_Event_t event){
	if(event == DMA_Event_Tranfer_Complete)
		dma_flash_rx_flag = 1;
}

void DMA2_Stream7_TranferComplete_CallBack(void *Paramemter, DMA_Event_t event){
	if(event == DMA_Event_Tranfer_Complete){
		dma_uart_log_tx_flag = 1;
		uart_log -> Stop_DMA();
	}
}

void USART1_Event_Callback(void *Parameter, USART_Event_t event){

	switch(event){
		case USART_Event_ReceiveComplete:

		break;

		case USART_Event_ReceiveEndChar:

		break;

		case USART_Event_IdleDetected:
			GPIO_Set(GPIOC, 13);
			uart_log -> GetBuffer(&Rxbuf);
			if(strstr((const char *)Rxbuf, "ESP OK") != NULL) esp_ok = true;
			STM_LOG(BOLD_RED, TAG, "STM32 Receive \"%s\" from ESP32.", (char *)Rxbuf);
			uart_log -> ReceiveUntilIdleIT(50);
		break;

		case USART_Event_TransmitComplete:

		break;

		default:

		break;

	}
}

void EXTI_Callback(uint16_t Pin){
	switch(Pin){
		case 3:
			GPIO_Set(GPIOC, 13);
			lora.IRQHandler();

		break;

		default:

		break;
	}

}

void LoRaCallback(void *arg, uint8_t len){
	LoRa *lr = (LoRa *)arg;
	uint8_t packetSize = len;
	if(packetSize){
		char *msg = (char *)malloc(packetSize+1);
		for (int i = 0; i < packetSize; i++) msg[i] = lr -> read();
		msg[packetSize] = '\0';
		STM_LOG(BOLD_YELLOW, TAG, "Receive: %s", msg
				);
		Packet_t pack = LoRa_ParseData(msg);
		switch(pack.command){
			case Node_Command_GetID:

//				if(!strcmp(pack.data, "*")){
//					Init_Node();
//				}
//				if(strcmp(pack.data, Init_id)==0){
//					Node_ID[atoi(Init_id)] = gettick() - Timing_init_node;
//				}

			break;
			case Node_Command_Data:
//				Node_Recieved[GetID_in_data(pack.data)]=1;
			break;
			case Node_Command_Control:

			break;


		}
		free(msg);
	}
}

void LoRa_TimerCallback(void *Parameter, TIM_Event_t event){
	switch(event){
		case TIM_Update_Event:
			LoRaTransmit(LORA_CMD_GETDATA, LORA_CMD_NODATA);
			STM_LOG(BOLD_CYAN, TAG, "LoRa Transmited");
//			Get_Data();
		break;
		default:

		break;
	}
}

void Timer3Callback(void *Parameter, TIM_Event_t event){
	switch(event){
		case TIM_Update_Event:
			GPIO_Toggle(GPIOC, 13);
//			uint32_t max_time_respone = 0;
//			char* data;
//			for(int i = 1; i < COUNT_NODE; i++){
//				if(!Node_Recieved[i]){
//
//				}
//			}
		break;
		default:

		break;
	}
}





