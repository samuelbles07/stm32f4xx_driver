/*
 * stm32f407xx.h
 *
 *  Created on: Feb 22, 2021
 *      Author: bles
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo			volatile

/*
 * Base address of Flash and SRAM memories
 */
#define FLASH_BASEADDR					0x08000000U
#define SRAM1_BASEADDR					0x20000000U
#define SRAM2_BASEADDR					0x2001C000U
#define ROM_BASEADDR					0x1FFF0000U
#define SRAM							SRAM1_BASEADDR

/*
 * AHBx and APBx peripheral base address
 */
#define PERIPH_BASEADDR					0x40000000U
#define APB1PERIPH_BASEADDR				PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR				0x40010000U
#define AHB1PERIPH_BASEADDR				0x40020000U
#define AHB2PERIPH_BASEADDR				0x50000000U

/*
 * Base address of peripheral connected to AHB1 Bus
 * AHB1PERIPH_BASEADDR + PERIPHERAL_OFFSET
 */
#define GPIOA_BASEADDR					0x40020000U
#define GPIOB_BASEADDR					0x40020400U
#define GPIOC_BASEADDR					0x40020800U
#define GPIOD_BASEADDR					0x40020C00U
#define GPIOE_BASEADDR					0x40021000U
#define GPIOF_BASEADDR					0x40021400U
#define GPIOG_BASEADDR					0x40021800U
#define GPIOH_BASEADDR					0x40021C00U
#define GPIOI_BASEADDR					0x40022000U
#define RCC_BASEADDR					0x40023800U

/*
 * Base address of peripheral connected to APB1 Bus
 * APB1PERIPH_BASEADDR + PERIPHERAL_OFFSET
 */
#define I2C1_BASEADDR					0x40005400U
#define I2C2_BASEADDR					0x40005800U
#define I2C3_BASEADDR					0x40005C00U
#define SPI2_BASEADDR					0x40003800U
#define SPI3_BASEADDR					0x40003C00U
#define USART2_BASEADDR					0x40004400U
#define USART3_BASEADDR					0x40004800U
#define UART4_BASEADDR					0x40004C00U
#define UART5_BASEADDR					0x40005000U

/*
 * Base address of peripheral connected to APB2 Bus
 * APB2PERIPH_BASEADDR + PERIPHERAL_OFFSET
 */
#define SPI1_BASEADDR					0x40013000U
#define USART1_BASEADDR					0x40011000U
#define USART6_BASEADDR					0x40011400U
#define EXTI_BASEADDR					0x40013C00U
#define SYSCFG_BASEADDR					0x40013800U


/*******************PERIPHERAL REGISTER DEFINITION STRUCTURE*****************/

/*
 * GPIO Registers Definition
 * 4 bytes between register definition (32 bit)
 */
typedef struct {
	__vo uint32_t MODER;		// GPIO port mode register (GPIOx_MODER) (x = A..I/J/K)
	__vo uint32_t OTYPER;		// GPIO port output type register (GPIOx_OTYPER)
	__vo uint32_t OSPEEDR;		// GPIO port output speed register (GPIOx_OSPEEDR)
	__vo uint32_t PUPDR;		// GPIO port pull-up/pull-down register (GPIOx_PUPDR)
	__vo uint32_t IDR;			// GPIO port input data register (GPIOx_IDR) (x = A..I/J/K)
	__vo uint32_t ODR;			// GPIO port output data register (GPIOx_ODR) (x = A..I/J/K)
	__vo uint32_t BSRR;			// GPIO port bit set/reset register (GPIOx_BSRR) (x = A..I/J/K)
	__vo uint32_t LCKR;			// GPIO port configuration lock register (GPIOx_LCKR) (x = A..I/J/K)
	__vo uint32_t AFR[2];		// GPIO alternate function [0] = GPIOx_AFRL (LOW); [1] = GPIOx_AFRH (HIGH)
} GPIO_RegDef_t;

/*
 * Peripheral register definition structure for RCC
 */
typedef struct {
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	__vo uint32_t RESERVED3;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t RESERVED4[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	uint32_t RESERVED5;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t RESERVED6[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t RESERVED8[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
	__vo uint32_t CKGATENR;
	__vo uint32_t DCKCFGR2;
} RCC_RegDef_t;

/*
 * Peripheral definition (PERIPHERAL BASE ADDRESS TYPECASTED TO xxx_RegDef_t)
 */
#define GPIOA							((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB							((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC							((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD							((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE							((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF							((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG							((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH							((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI							((GPIO_RegDef_t*) GPIOI_BASEADDR)

#define RCC								((RCC_RegDef_t*) RCC_BASEADDR)

/*
 * Clock ENABLE Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()					( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN()					( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN()					( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN()					( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN()					( RCC->AHB1ENR |= (1 << 4) )
#define GPIOF_PCLK_EN()					( RCC->AHB1ENR |= (1 << 5) )
#define GPIOG_PCLK_EN()					( RCC->AHB1ENR |= (1 << 6) )
#define GPIOH_PCLK_EN()					( RCC->AHB1ENR |= (1 << 7) )
#define GPIOI_PCLK_EN()					( RCC->AHB1ENR |= (1 << 8) )

/*
 * Clock ENABLE Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()					( RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN()					( RCC->APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN()					( RCC->APB1ENR |= (1 << 23) )

/*
 * Clock ENABLE Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()					( RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN()					( RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN()					( RCC->APB1ENR |= (1 << 15) )

/*
 * Clock ENABLE Macros for USARTx peripherals
 */
#define USART1_PCLK_EN()					( RCC->APB2ENR |= (1 << 4) )
#define USART2_PCLK_EN()					( RCC->APB1ENR |= (1 << 17) )
#define USART3_PCLK_EN()					( RCC->APB1ENR |= (1 << 18) )
#define UART4_PCLK_EN()						( RCC->APB1ENR |= (1 << 19) )
#define UART5_PCLK_EN()						( RCC->APB1ENR |= (1 << 20) )
#define USART6_PCLK_EN()					( RCC->APB2ENR |= (1 << 5) )

/*
 * Clock ENABLE Macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_EN()					( RCC->APB2ENR |= (1 << 14) )

/*
 * Clock DISABLE Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()					( RCC->AHB1ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DI()					( RCC->AHB1ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI()					( RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI()					( RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI()					( RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOF_PCLK_DI()					( RCC->AHB1ENR &= ~(1 << 5) )
#define GPIOG_PCLK_DI()					( RCC->AHB1ENR &= ~(1 << 6) )
#define GPIOH_PCLK_DI()					( RCC->AHB1ENR &= ~(1 << 7) )
#define GPIOI_PCLK_DI()					( RCC->AHB1ENR &= ~(1 << 8) )

/*
 * Clock DISABLE Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()					( RCC->APB1ENR &= ~(1 << 21) )
#define I2C2_PCLK_DI()					( RCC->APB1ENR &= ~(1 << 22) )
#define I2C3_PCLK_DI()					( RCC->APB1ENR &= ~(1 << 23) )

/*
 * Clock DISABLE Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()					( RCC->APB2ENR &= ~(1 << 12) )
#define SPI2_PCLK_DI()					( RCC->APB1ENR &= ~(1 << 14) )
#define SPI3_PCLK_DI()					( RCC->APB1ENR &= ~(1 << 15) )

/*
 * Clock DISABLE Macros for USARTx peripherals
 */
#define USART1_PCLK_DI()					( RCC->APB2ENR &= ~(1 << 4) )
#define USART2_PCLK_DI()					( RCC->APB1ENR &= ~(1 << 17) )
#define USART3_PCLK_DI()					( RCC->APB1ENR &= ~(1 << 18) )
#define UART4_PCLK_DI()						( RCC->APB1ENR &= ~(1 << 19) )
#define UART5_PCLK_DI()						( RCC->APB1ENR &= ~(1 << 20) )
#define USART6_PCLK_DI()					( RCC->APB2ENR &= ~(1 << 5) )

/*
 * Clock DISABLE Macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_DI()					( RCC->APB2ENR &= ~(1 << 14) )

/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()					do { (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= !(1 << 0)); }while(0)
#define GPIOB_REG_RESET()					do { (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= !(1 << 1)); }while(0)
#define GPIOC_REG_RESET()					do { (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= !(1 << 2)); }while(0)
#define GPIOD_REG_RESET()					do { (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= !(1 << 3)); }while(0)
#define GPIOE_REG_RESET()					do { (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= !(1 << 4)); }while(0)
#define GPIOF_REG_RESET()					do { (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= !(1 << 5)); }while(0)
#define GPIOG_REG_RESET()					do { (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= !(1 << 6)); }while(0)
#define GPIOH_REG_RESET()					do { (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= !(1 << 7)); }while(0)
#define GPIOI_REG_RESET()					do { (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= !(1 << 8)); }while(0)


// Generic definition macro
#define ENABLE								1
#define DISABLE								0
#define SET									ENABLE
#define RESET								DISABLE
#define GPIO_PIN_SET						SET
#define GPIO_PIN_RESET						RESET


#include "stm32f407xx_gpio_driver.h"

#endif /* INC_STM32F407XX_H_ */
