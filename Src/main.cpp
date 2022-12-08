


#include <stdint.h>
#include "stm32f4xx.h"
#include "Config.h"
#include "string.h"

const char *TAG = "STM32";
// CS = PA8
uint8_t FlsRxbuf[256];
uint8_t FlsTxbuf[256];
uint32_t FlsAddr = 0x5100UL;
char buf[17];

void ShowFlashData(uint32_t addr);

uint32_t count = 0;

int main (void){
	Periph_Initialize();
	AppLayer_Initialize();

	spiflash.EraseSector(FlsAddr/4096);
	ShowFlashData(FlsAddr);

	for(uint16_t i=0; i<256; i++) FlsTxbuf[i] = i;
	spiflash.WriteBytes(FlsAddr, FlsTxbuf, 256);
	ShowFlashData(FlsAddr);


	while(1){
//		if(RxFlag){
		GPIO_Reset(GPIOC, 13);
		TickDelay_ms(10);
//			RxFlag = false;
//		}
//		GPIO_Toggle(GPIOC, 13);

	}
}

void ShowFlashData(uint32_t addr){
	STM_LOG(BOLD_PURPLE, TAG, "Read from ID: 0x%08x.", addr);
	spiflash.ReadBytes(FlsAddr, FlsRxbuf, 256);
	for(uint8_t i=0; i<16; i++){
		STM_LOG(BOLD_WHITE, TAG, "0x%08x: 0x%02x|0x%02x|0x%02x|0x%02x|0x%02x|0x%02x|0x%02x|0x%02x|0x%02x|0x%02x|0x%02x|0x%02x|0x%02x|0x%02x|0x%02x|0x%02x", addr + i*16,
				FlsRxbuf[0+i*16], FlsRxbuf[1+i*16], FlsRxbuf[2+i*16],  FlsRxbuf[3+i*16],  FlsRxbuf[4+i*16],  FlsRxbuf[5+i*16],  FlsRxbuf[6+i*16],  FlsRxbuf[7+i*16],
				FlsRxbuf[8+i*16], FlsRxbuf[9+i*16], FlsRxbuf[10+i*16], FlsRxbuf[11+i*16], FlsRxbuf[12+i*16], FlsRxbuf[13+i*16], FlsRxbuf[14+i*16], FlsRxbuf[15+i*16]);
	}
}




















