/*
 * DS3231.h
 *
 *  Created on: Nov 29, 2022
 *      Author: anh
 */

#ifndef DS3231_H_
#define DS3231_H_



#include "stm32f4xx.h"
#include "I2C_F4xx.h"

#ifdef __cplusplus
extern "C"{
#endif

#define DS3231_ADDRESS 0xD0



typedef struct {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hour;
	uint8_t dayofweek;
	uint8_t dayofmonth;
	uint8_t month;
	uint8_t year;
} DS3231_Time_t;


void DS3231_Init(I2C *i2c);

void DS3231_SetTime(DS3231_Time_t time);
void DS3231_GetTime(DS3231_Time_t *time);

float DS3231_GetTemp(void);


#ifdef __cplusplus
}
#endif


#endif /* DS3231_H_ */
