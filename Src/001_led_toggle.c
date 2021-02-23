/*
 * 001_led_toggle.c
 *
 *  Created on: Feb 24, 2021
 *      Author: bles
 */

#include "stm32f407xx.h"

void delay(void) {
	for (uint32_t i = 0; i < 500000/2; i++);
}

int main(void) {

	// Init Led
	GPIO_Handle_t gpioLed;
	gpioLed.pGPIOx 								= GPIOD;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_12;
	gpioLed.GPIO_PinConfig.GPIO_PinMode 		= GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_SPEED_DEFAULT;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OP_TYPE_DEFAULT;
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_PUPD_DEFAULT;

	GPIO_PClkControl(GPIOD, ENABLE);
	GPIO_Init(&gpioLed);

	// Init button
	GPIO_Handle_t gpioButton;
	gpioButton.pGPIOx	= GPIOA;
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	gpioButton.GPIO_PinConfig.GPIO_PinMode	 = GPIO_MODE_IN;

	GPIO_PClkControl(GPIOA, ENABLE);
	GPIO_Init(&gpioButton);

//	uint8_t lastState = 0;
	while (1) {
		// Test 1
//		if (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0) != lastState) {
//			lastState = !lastState;
//			GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_12, lastState);
//		}

		// Test 2
		if (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0) == 1) {
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
			delay(); // Wait for debounce
		}

	}

	return 0;
}
