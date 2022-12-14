/*
 * GPIO_F4xx.cpp
 *
 *  Created on: Nov 9, 2022
 *      Author: anh
 */
#include "PERIPH_USED.h"

#ifdef ENABLE_GPIO

#include "GPIO_F4xx.h"


void GPIO_CLOCKENABLE(void){
	/* ENABLE GPIO CLOCK */
	RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN
					| RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOFEN | RCC_AHB1ENR_GPIOGEN | RCC_AHB1ENR_GPIOHEN | RCC_AHB1ENR_GPIOIEN;
}

void GPIO_PortClockEnable(GPIO_TypeDef *port){
	if(port == GPIOA) {GPIOA_CLOCKENABLE(); return;}
	if(port == GPIOB) {GPIOB_CLOCKENABLE(); return;}
	if(port == GPIOC) {GPIOC_CLOCKENABLE(); return;}
	if(port == GPIOD) {GPIOD_CLOCKENABLE(); return;}
	if(port == GPIOE) {GPIOE_CLOCKENABLE(); return;}
	if(port == GPIOF) {GPIOF_CLOCKENABLE(); return;}
	if(port == GPIOG) {GPIOG_CLOCKENABLE(); return;}
	if(port == GPIOH) {GPIOH_CLOCKENABLE(); return;}
	if(port == GPIOI) {GPIOI_CLOCKENABLE(); return;}

}

void GPIO_Init(GPIO_TypeDef *port, uint16_t pin, GPIOMode_t mode){
	__IO uint32_t tmpreg = 0U;
	/* *************************************************** */
	if(mode <=  GPIO_INPUT_PULLDOWN){ // GPIO_INPUT.
		port -> MODER &=~ (3U << (pin * 2));

		tmpreg = port -> PUPDR;
		tmpreg &=~ (3U << (pin * 2));
		switch(mode){
			case GPIO_INPUT_PULLUP:
				tmpreg |=  (1U << (pin * 2));
			break;
			case GPIO_INPUT_PULLDOWN:
				tmpreg |=  (2U << (pin * 2));
			break;
			default:
			break;
		}
		port -> PUPDR |=tmpreg;
	}
	/* *************************************************** */
	else if(mode >= GPIO_OUTPUT_OPENDRAIN && mode <=  GPIO_OUTPUT_PUSHPULL_PULLDOWN){ // GPIO_OUTPUT.
		port -> MODER &=~ (3U << (pin * 2));
		port -> MODER |=  (1U << (pin * 2));

		if(mode >= GPIO_OUTPUT_OPENDRAIN && mode <=  GPIO_OUTPUT_OPENDRAIN_PULLDOWN) port -> OTYPER |= (1U << pin);
		else port -> OTYPER &=~ (1U << pin);

		port -> OSPEEDR &=~ (3U << (pin * 2));
		port -> OSPEEDR |=  (GPIO_OUTPUTSPEED_DEFAULT << (pin * 2));

		tmpreg = port -> PUPDR;
		tmpreg &=~ (3U << (pin * 2));
		if(mode == GPIO_OUTPUT_OPENDRAIN_PULLUP || mode == GPIO_OUTPUT_PUSHPULL_PULLUP) tmpreg |=  (1U << (pin * 2));
		else if(mode == GPIO_OUTPUT_OPENDRAIN_PULLDOWN || mode == GPIO_OUTPUT_PUSHPULL_PULLDOWN) tmpreg |=  (2U << (pin * 2));
		port -> PUPDR |=tmpreg;

	}
	/* *************************************************** */
	else{ // GPIO_ANALOG.
		port -> MODER |= (3U << (pin * 2));
	}
}

void GPIO_AlternateFunction(GPIO_TypeDef *port, uint16_t pin, GPIOAlternateFunction_t function){
	port -> MODER &=~ (3U << (pin*2));
	port -> MODER |=  (2U << (pin*2));

	port -> OSPEEDR &=~ (3U << (pin * 2));
	port -> OSPEEDR |=  (GPIO_OUTPUTSPEED_DEFAULT << (pin * 2));

	if(pin < 8){
		port -> AFR[0] &=~ (0x0FU << (pin*4));
		port -> AFR[0] |=  (function  << (pin*4));
	}
	else{
		port -> AFR[1] &=~ (0x0FU << ((pin-8)*4));
		port -> AFR[1] |=  (function  << ((pin-8)*4));
	}
}

void GPIO_AF_Type(GPIO_TypeDef *port, uint16_t pin, GPIOMode_t mode){
	if(mode == GPIO_OUTPUT_OPENDRAIN) port -> OTYPER |= (1U<<pin);
	else if(mode == GPIO_OUTPUT_PUSHPULL) port -> OTYPER &=~ (1U<<pin);
}

void GPIO_Pullup(GPIO_TypeDef *port, uint16_t pin){
	port ->PUPDR &=~ (3U << (pin*2));
	port ->PUPDR |= (1U << (pin*2));
}

void GPIO_Pulldown(GPIO_TypeDef *port, uint16_t pin){
	port ->PUPDR &=~ (3U << (pin*2));
	port ->PUPDR |= (2U << (pin*2));
}

void GPIO_Set(GPIO_TypeDef *port, uint16_t pin){
	port -> BSRR |= (1 << pin);
}

void GPIO_Reset(GPIO_TypeDef *port, uint16_t pin){
	port -> BSRR |= (1 << (pin + 16));
}

void GPIO_Toggle(GPIO_TypeDef *port, uint16_t pin){
	if(port -> ODR & (1<<pin)) GPIO_Reset(port, pin);
	else GPIO_Set(port, pin);

}

int GPIO_Read(GPIO_TypeDef *port, uint16_t pin){
	return (port -> IDR >> pin) & 1UL;
}


#endif





















