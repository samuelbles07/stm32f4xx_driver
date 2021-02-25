/*
 * 002_interrupt.c
 *
 *  Created on: Feb 25, 2021
 *      Author: bles
 */

#include <string.h>
#include "stm32f407xx.h"

void delay(void) {
	for (uint32_t i = 0; i < 500000/2; i++); // around 200ms
}

int main(void) {

	// Init Led
	GPIO_Handle_t gpioLed;
	gpioLed.pGPIOx = GPIOD;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_DEFAULT;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_DEFAULT;
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_DEFAULT;

	GPIO_PClkControl(GPIOD, ENABLE);
	GPIO_Init(&gpioLed);

	// Init button
	GPIO_Handle_t gpioButton;
	memset(&gpioButton, 0, sizeof(gpioButton)); // Set all default value to 0
	gpioButton.pGPIOx = GPIOA;
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ISR_RISE;

	GPIO_PClkControl(GPIOA, ENABLE);
	GPIO_Init(&gpioButton);

	// Init interrupt
	GPIO_IRQConfig(IRQ_NUM_EXTI0, ENABLE, 15);


	while(1);
}

void EXTI0_IRQHandler(void) {
	delay();
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
	GPIO_IRQClear(GPIO_PIN_0);
}

