/*
 * GPIO_F4xx.h
 *
 *  Created on: Nov 9, 2022
 *      Author: anh
 */

#ifndef GPIO_F4XX_H_
#define GPIO_F4XX_H_

#include "PERIPH_USED.h"

#ifdef ENABLE_GPIO

#include "stdio.h"
#include "stm32f4xx.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GPIO_OUTPUTSPEED_DEFAULT 3U

typedef enum{
	GPIO_INPUT,
	GPIO_INPUT_PULLUP,
	GPIO_INPUT_PULLDOWN,

	GPIO_OUTPUT_OPENDRAIN,
	GPIO_OUTPUT_OPENDRAIN_PULLUP,
	GPIO_OUTPUT_OPENDRAIN_PULLDOWN,
	GPIO_OUTPUT_PUSHPULL,
	GPIO_OUTPUT_PUSHPULL_PULLUP,
	GPIO_OUTPUT_PUSHPULL_PULLDOWN,

	GPIO_ANALOGOUTPUT_OD
} GPIOMode_t;

typedef enum{
	AF0_SYSTEM,
	AF1_TIM1_2,
	AF2_TIM3_5,
	AF3_TIM8_11,
	AF4_I2C1_3,
	AF5_SPI1_2,
	AF6_SPI3,
	AF7_USART1_3,
	AF8_USART4_6,
	AF9_CAN1_2_TIM12_14,
	AF10_USB,
	AF11_ETH,
	AF12_FSMC_SDIO_USB,
	AF13_DCMI,
	AF14,
	AF15_EVENTOUT,
} GPIOAlternateFunction_t;


void GPIO_CLOCKENABLE(void);
void GPIO_PortClockEnable(GPIO_TypeDef *port);
void GPIO_Init(GPIO_TypeDef *port, uint16_t pin, GPIOMode_t mode);
void GPIO_AlternateFunction(GPIO_TypeDef *port, uint16_t pin, GPIOAlternateFunction_t function);
void GPIO_AF_Type(GPIO_TypeDef *port, uint16_t pin, GPIOMode_t mode);

void GPIO_Pullup(GPIO_TypeDef *port, uint16_t pin);
void GPIO_Pulldown(GPIO_TypeDef *port, uint16_t pin);

void GPIO_Set(GPIO_TypeDef *port, uint16_t pin);
void GPIO_Reset(GPIO_TypeDef *port, uint16_t pin);
void GPIO_Toggle(GPIO_TypeDef *port, uint16_t pin);

int GPIO_Read(GPIO_TypeDef *port, uint16_t pin);

#define GPIOA_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN
#define GPIOB_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOBEN
#define GPIOC_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOCEN
#define GPIOD_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIODEN
#define GPIOE_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOEEN
#define GPIOF_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOFEN
#define GPIOG_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOGEN
#define GPIOH_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOHEN
#define GPIOI_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOIEN

#ifdef __cplusplus
}
#endif

#endif

#endif /* GPIO_F4XX_H_ */
