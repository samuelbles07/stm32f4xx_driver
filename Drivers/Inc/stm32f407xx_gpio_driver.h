/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Feb 23, 2021
 *      Author: bles
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/*
 * This is a Configuration structure for a GPIO pin
 */

typedef struct {
	uint8_t GPIO_PinNumber;			// Values from @GPIO_PINS
	uint8_t GPIO_PinMode;			// Values from @GPIO_PIN_MODE
	uint8_t GPIO_PinSpeed;			// Values from @GPIO_PIN_OUTPUT_SPEED
	uint8_t GPIO_PinPuPdControl;	// Values from @GPIO_PIN_PUPD
	uint8_t GPIO_PinOPType;			// Values from @GPIO_PIN_OUTPUT_TYPES
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

/*
 * This is a Handle structure for a GPIO pin
 */
typedef struct {
	GPIO_RegDef_t *pGPIOx;	// This holds the base address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig; // This holds GPIO pin configuration settings
} GPIO_Handle_t;

/*
 * @GPIO_PINS
 * GPIO Pin list
 */
#define GPIO_PIN_0				0
#define GPIO_PIN_1				1
#define GPIO_PIN_2				2
#define GPIO_PIN_3				3
#define GPIO_PIN_4				4
#define GPIO_PIN_5				5
#define GPIO_PIN_6				6
#define GPIO_PIN_7				7
#define GPIO_PIN_8				8
#define GPIO_PIN_9				9
#define GPIO_PIN_10				10
#define GPIO_PIN_11				11
#define GPIO_PIN_12				12
#define GPIO_PIN_13				13
#define GPIO_PIN_14				14
#define GPIO_PIN_15				15

/*
 * @GPIO_PIN_MODE
 * GPIO pin possible mode
 */
#define GPIO_MODE_IN			0
#define GPIO_MODE_OUT			1
#define GPIO_MODE_ALT_FN		2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_ISR_FALL		4
#define GPIO_MODE_ISR_RISE		5
#define GPIO_MODE_ISR_BOTH		6

/*
 * @GPIO_PIN_OUTPUT_TYPES
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP			0
#define GPIO_OP_TYPE_OD			1
#define GPIO_OP_TYPE_DEFAULT	GPIO_OP_TYPE_PP

/*
 * @GPIO_PIN_OUTPUT_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3
#define GPIO_SPEED_DEFAULT		GPIO_SPEED_MEDIUM

/*
 * @GPIO_PIN_PUPD
 * GPIO pin pull up pull down configuration macros
 */
#define GPIO_NO_PUPD			0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2
#define GPIO_PUPD_DEFAULT		GPIO_NO_PUPD

/*
 * Drivers API
 */

// Peripheral Clock Setup
void GPIO_PClkControl(GPIO_RegDef_t *pGPIOx, uint8_t status);


// Init and De-Init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


// Data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);


// IRQ Config and handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t status, uint32_t IRQPriority);
void GPIO_IRQClear(uint8_t pinNumber);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
