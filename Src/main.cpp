


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

#define MPU6050_ADDR 0xD0

#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75


int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

float Ax, Ay, Az, Gx, Gy, Gz;

void ShowFlashData(uint32_t addr);
void MPU6050_Init(void);
void MPU6050_Read_Accel(void);
void MPU6050_Read_Gyro(void);

int main (void){
	Periph_Initialize();
	AppLayer_Initialize();

//	spiflash.EraseSector(addr/4096);
//	ShowFlashData(addr);
//
//	for(uint16_t i=0; i<256; i++) Txbuf[i] = i;
//	spiflash.WriteBytes(addr, Txbuf, 256);
//	ShowFlashData(addr);

	MPU6050_Init();

	while(1){
		MPU6050_Read_Accel();
		MPU6050_Read_Gyro();

		STM_LOG(BOLD_YELLOW, TAG, "Accel: Ax = %f, Ax = %f, Az = %f", Ax, Ay, Az);
		STM_LOG(BOLD_CYAN  , TAG, "Gyro:  Gx = %f, Gx = %f, Gz = %f", Gx, Gy, Gz);

//		TickDelay_ms(1000);
//		GPIO_Toggle(GPIOC, 13);

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


void MPU6050_Init(void){
	uint8_t check;
	uint8_t Data;

	Result_t res = i2c.Memory_Receive(MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1);
	STM_LOG_RES(res);
	STM_LOG(BOLD_RED, TAG, "MPU6050 WHO_AM_I_REG = 0x%02x", check);
	if (check == 0x68) {
		Data = 0;
		res = i2c.Memory_Transmit(MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1);
		if(!CheckResult(res)) {
			STM_LOG_RES(res);
		}
		Data = 0x07;
		res = i2c.Memory_Transmit(MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1);
		if(!CheckResult(res)) {
			STM_LOG_RES(res);
		}
		Data = 0x00;
		res = i2c.Memory_Transmit(MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1);
		if(!CheckResult(res)) {
			STM_LOG_RES(res);
		}
		Data = 0x00;
		res = i2c.Memory_Transmit(MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1);
		if(!CheckResult(res)) {
			STM_LOG_RES(res);
		}
	}
}


void MPU6050_Read_Accel(void){
	uint8_t Rec_Data[6];

	Result_t res = i2c.Memory_Receive(MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6);
	if(!CheckResult(res)) {
		STM_LOG_RES(res);
	}

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	Ax = Accel_X_RAW/16384.0;
	Ay = Accel_Y_RAW/16384.0;
	Az = Accel_Z_RAW/16384.0;
}


void MPU6050_Read_Gyro(void){
	uint8_t Rec_Data[6];

	Result_t res = i2c.Memory_Receive(MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6);
	if(!CheckResult(res)) {
		STM_LOG_RES(res);
	}

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	Gx = Gyro_X_RAW/131.0;
	Gy = Gyro_Y_RAW/131.0;
	Gz = Gyro_Z_RAW/131.0;
}





















