


#include <stdint.h>
#include "stm32f4xx.h"
#include "Config.h"
#include "string.h"

const char *TAG = "MAIN";
// CS = PA8
uint8_t Rxbuf[256];
uint8_t Txbuf[256];
uint32_t addr = 0x200UL;
char buf[17];

void ShowFlashData(uint32_t addr);


uint32_t count = 0;
int main (void){
	Periph_Initialize();
	AppLayer_Initialize();

	spiflash.EraseSector(addr/4096);
	ShowFlashData(addr);

	for(uint16_t i=0; i<256; i++) Txbuf[i] = i;
	spiflash.WriteBytes(addr, Txbuf, 256);
	ShowFlashData(addr);

	spiflash.EraseSector(addr/4096);
	ShowFlashData(addr);

	for(uint16_t i=0; i<256; i++) Txbuf[i] = 255-i;
	spiflash.WriteBytes(addr, Txbuf, 256);
	ShowFlashData(addr);

	while(1){
		GPIO_Toggle(GPIOC, 13);
		TickDelay_ms(500);
//		STM_LOG(BOLD_GREEN, TAG, "Count = %d", count++);
	}
}

void ShowFlashData(uint32_t addr){
	STM_LOG(BOLD_PURPLE, TAG, "Read from ID: 0x%08x.", addr);
	spiflash.ReadBytes(addr, Rxbuf, 256);
	for(uint8_t i=0; i<16; i++){
		STM_LOG(BOLD_WHITE, TAG, "0x%08x: 0x%02x|0x%02x|0x%02x|0x%02x|0x%02x|0x%02x|0x%02x|0x%02x|0x%02x|0x%02x|0x%02x|0x%02x|0x%02x|0x%02x|0x%02x|0x%02x", addr + i*16,
				Rxbuf[0+i*16], Rxbuf[1+i*16], Rxbuf[2+i*16],  Rxbuf[3+i*16],  Rxbuf[4+i*16],  Rxbuf[5+i*16],  Rxbuf[6+i*16],  Rxbuf[7+i*16],
				Rxbuf[8+i*16], Rxbuf[9+i*16], Rxbuf[10+i*16], Rxbuf[11+i*16], Rxbuf[12+i*16], Rxbuf[13+i*16], Rxbuf[14+i*16], Rxbuf[15+i*16]);
	}
}



























