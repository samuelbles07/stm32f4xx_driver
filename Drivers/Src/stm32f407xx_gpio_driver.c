/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Feb 23, 2021
 *      Author: bles
 */


#include "stm32f407xx_gpio_driver.h"

/*
 * Desc		- Enables or disable peripheral clock for given GPIO port
 * Param1	- Base address of the GPIO peripheral
 * Param2	- ENABLE or DISABLE Macro
 */
void GPIO_PClkControl(GPIO_RegDef_t *pGPIOx, uint8_t status) {

	if (status == ENABLE) {

		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();

		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();

		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();

		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();

		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();

		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();

		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();

		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();

		} else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_EN();

		}
	} else {

		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();

		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();

		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();

		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();

		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();

		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();

		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();

		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();

		} else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_DI();

		}
	}
}


/*
 * Desc		- Initialize given GPIO configuration
 * Param1	- GPIO Struct handler
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {

	// Temporary variable to store config
	uint32_t tmp = 0;

	// Check if GPIO mode either normal or IRQ
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		// Set GPIO mode
		tmp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ); // Clear register bit
		pGPIOHandle->pGPIOx->MODER |= tmp; // Then set

	} else {
		// Set IRQ mode

		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ISR_FALL) {
			// Configure interrupt fall event
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			// Just to make sure provided pin not in rise event
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ISR_RISE) {
			// Configure interrupt rise event
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			// Just to make sure provided pin not in fall event
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ISR_BOTH) {
			// Configure interrupt for both rise and fall event
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}

		// Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t tmp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4; // Get index position of EXTICR
		uint8_t tmp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4; // Get multiplier of bit position
		uint8_t bits = GPIO_BASEADDR_TO_EXTICR_BIT(pGPIOHandle->pGPIOx);
		SYSCFG->EXTICR[tmp1] = bits << (tmp2 * 4); // Set bit of coresponding port

		// Enable SYSCFG peripheral clock
		SYSCFG_PCLK_EN();
		// Enable EXTI interrupt delivery using IMR
		EXTI->IMR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	}

	// Clear tmp variable
	tmp = 0;

	// Set GPIO Speed
	tmp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ); // Clear register bit
	pGPIOHandle->pGPIOx->OSPEEDR |= tmp;

	tmp = 0;

	// Set GPIO pull up pull down
	tmp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ); // Clear register bit
	pGPIOHandle->pGPIOx->PUPDR |= tmp;

	tmp = 0;

	// Set GPIO Output type
	tmp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ); // Clear register bit
	pGPIOHandle->pGPIOx->OTYPER |= tmp;

	// alt_fn
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALT_FN) {
		// Declare temp variable
		uint8_t tmp1, tmp2;

		tmp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8; // Find if it's high register or low register
		tmp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8; // Find bit position multiplier that gonna be multiply by 4 (because value selection is 4 bit)
		pGPIOHandle->pGPIOx->AFR[tmp1] &= ~( 0xF << (4 * tmp2) );
		pGPIOHandle->pGPIOx->AFR[tmp1] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * tmp2 ); // Set
	}


}

/*
 * Desc		- De-Initialize given GPIO port
 * Param1	- GPIO Register port
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {

	if (pGPIOx == GPIOA) {
		GPIOA_REG_RESET();

	} else if (pGPIOx == GPIOB) {
		GPIOB_REG_RESET();

	} else if (pGPIOx == GPIOC) {
		GPIOC_REG_RESET();

	} else if (pGPIOx == GPIOD) {
		GPIOD_REG_RESET();

	} else if (pGPIOx == GPIOE) {
		GPIOE_REG_RESET();

	} else if (pGPIOx == GPIOF) {
		GPIOF_REG_RESET();

	} else if (pGPIOx == GPIOG) {
		GPIOG_REG_RESET();

	} else if (pGPIOx == GPIOH) {
		GPIOH_REG_RESET();

	} else if (pGPIOx == GPIOI) {
		GPIOI_REG_RESET();

	}
}


/*
 * Desc		- Read data from input pin
 * Param1	- GPIO register port
 * Param2	- GPIO pin number
 * Return	- Data in 1 or 0 (HIGH or LOW))
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {

	uint8_t value;
	value = (uint8_t)( (pGPIOx->IDR >> pinNumber) & 0x00000001);

	return value;
}

/*
 * Desc		- Read data from input port
 * Param1	- GPIO register port
 * Return	- Data in 1 or 0 for all 16 pins
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {

	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;

	return value;
}

/*
 * Desc		- Write data to output pin
 * Param1	- GPIO register port
 * Param2	- Pin to write
 * Param3	- Data to write (GPIO_PIN_SET or GPIO_PIN_RESET macro)
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value) {

	if (value == GPIO_PIN_SET) {
		pGPIOx->ODR |= (1 << pinNumber);
	} else {
		pGPIOx->ODR &= ~(1 << pinNumber);
	}
}

/*
 * Desc		- Write data to output port
 * Param1	- GPIO register port
 * Param2	- Data to write
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value) {

	pGPIOx->ODR = value;
}

/*
 * Desc		- Toggle given pin number to !state
 * Param1	- GPIO register port
 * Param2	- Pin to write
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {

	pGPIOx->ODR ^= ( 1 << pinNumber );
}


/*
 * Desc		- Configure given IRQ number
 * Param1	- IRQ number to configure
 * Param2	- ENABLE or DISABLE IRQ
 * Param3	- Set priority (if default just set -1)
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t status, uint32_t IRQPriority) {

	if (status == ENABLE) {
		// Check IRQ Number
		if (IRQNumber <= 31) { // 0 - 31
			// Enable ISER0 Register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		} else if (IRQNumber > 31 && IRQNumber < 64) { // 32 - 63
			// Enable ISER1 Register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) ); // map to 0 - 31 bit of ISER1

		} else if (IRQNumber >= 64 && IRQNumber < 94) {	// 64 - 93
			// Enable ISER2 Register
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) ); // map to 0 - 31 bit of ISER2
		}
	} else {
		// Check IRQ Number
		if (IRQNumber <= 31) {
			// Disable interrupt by set ICER0 Register to 1
			*NVIC_ICER0 |= (1 << IRQNumber);

		} else if (IRQNumber > 31 && IRQNumber < 64) { // 32 - 63
		// Disable ICER1 Register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));

		} else if (IRQNumber >= 64 && IRQNumber < 94) {	// 64 - 93
		// Disable ICER2 Register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}

	/*
	 * Check 4.2.7 Interrupt Priority Registers of Cortex-M4 Generic User Guide
	 */
	if ((int)IRQPriority > -1) {
		// First, find address that have to be set (Each IPRx have 4 section)
		// Then, set Priority value by multiply multiplier(iprx_section) by 8 because each section is 8 bit
		// And because of this "Register priority value fields are eight bits wide, and non-implemented low-order bits read as zero and ignore writes."
		// So we have to shift Priority value by 4 then we get the shift_amount

		uint8_t iprx = IRQNumber / 4; //1
		uint8_t iprx_section = IRQNumber % 4; //2
		uint8_t shift_amount = ( iprx_section * 8 ) + ( 8 - NO_PR_BITS_IMPLEMENTED );

		*(NVIC_PR_BASE_ADDR + iprx) |= ( IRQPriority << shift_amount );
	}
}

void GPIO_IRQClear(uint8_t pinNumber) {
	// Clear EXTI PR register for given pin number
	if (EXTI->PR & ( 1 << pinNumber)) {
		EXTI->PR |= ( 1 << pinNumber);
	}
}
