
#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

// base addresses of Flash, ROM and SRAM

#define FLASH_BASEADDR				0X08000000U
#define SRAM1_BASEADDR				0x20000000U // 112kb
#define SRAM2_BASEADDR				0x20001C00U
#define ROM_BASEADDR				0x1FFF0000U
#define SRAM 						SRAM1_BASEADDR


// AHBx and APBx bus peripheral base addresses

#define PERIPH_BASEADDR				0x40000000U
#define APB1PERIPH_BASEADDR			PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR			0x40010000U
#define AHB1PERIPH_BASEADDR			0x40020000U
#define AHB2PERIPH_BASEADDR			0x50000000U


// AHB1 bus peripherals

#define GPIOA_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR				(AHB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3800)

// APB1 bus peripherals

#define I2C1_BASEADDR				(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR				(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR				(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR				(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR				(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR				(APB1PERIPH_BASEADDR + 0x5000)

// APB2 bus peripherals

#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3000)

#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR				(APB2PERIPH_BASEADDR + 0x1400)

#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR				(APB2PERIPH_BASEADDR + 0x3800)

// peripheral register definition structures
// volatile as registers are updated every AHB clock cycle

typedef struct
{
	volatile uint32_t MODER;				// GPIO port mode register
	volatile uint32_t OTYPER;				// GPIO port output type register
	volatile uint32_t OSPEEDR;				// GPIO port output speed register
	volatile uint32_t PUPDR;				// GPIO port pull-up/pull-down register
	volatile uint32_t IDR;					// GPIO port input data register
	volatile uint32_t ODR;					// GPIO port output data register
	volatile uint32_t BSRR;					// GPIO port bit set/reset register
	volatile uint32_t LCKR;					// GPIO port configuration lock register
	volatile uint32_t AFR[2];				// AFR[0] alternate function low register / AFR[1] alternate function high register
} GPIO_RegDef_t;

typedef struct
{
	volatile uint32_t CR;					// RCC clock control register
	volatile uint32_t PLLCFGR;				// RCC PLL configuration register
	volatile uint32_t CFGR;					// RCC clock configuration register
	volatile uint32_t CIR;					// RCC clock interrupt register

	volatile uint32_t AHB1RSTR;				// RCC AHB1 peripheral reset register
	volatile uint32_t AHB2RSTR;				// RCC AHB2 peripheral reset register
	volatile uint32_t AHB3RSTR;				// RCC AHB3 peripheral reset register
	uint32_t RESERVED00;					// reserved

	volatile uint32_t APB1RSTR;				// RCC APB1 peripheral reset register
	volatile uint32_t APB2RSTR;				// RCC APB2 peripheral reset register
	uint32_t RESERVED01[2];					// reserved

	volatile uint32_t AHB1ENR;				// RCC AHB1 peripheral clock enable register
	volatile uint32_t AHB2ENR;				// RCC AHB2 peripheral clock enable register
	volatile uint32_t AHB3ENR;				// RCC AHB3 peripheral clock enable register
	uint32_t RESERVED02;					// reserved

	volatile uint32_t APB1ENR;				// RCC APB1 peripheral clock enable register
	volatile uint32_t APB2ENR;				// RCC APB2 peripheral clock enable register
	uint32_t RESERVED03[2];					// reserved

	volatile uint32_t AHB1LPENR;			// RCC AHB1 peripheral clock enable in low power mode register
	volatile uint32_t AHB2LPENR;			// RCC AHB2 peripheral clock enable in low power mode register
	volatile uint32_t AHB3LPENR;			// RCC AHB3 peripheral clock enable in low power mode register
	uint32_t RESERVED04;					// reserved

	volatile uint32_t APB1LPENR;			// RCC APB1 peripheral clock enable in low power mode register
	volatile uint32_t APB2LPENR;			// RCC APB2 peripheral clock enable in low power mode register
	uint32_t RESERVED05[2];					// reserved

	volatile uint32_t BDCR;					// RCC Backup domain control register
	volatile uint32_t CSR;					// RCC clock control & status register
	uint32_t RESERVED06[2];					// reserved

	volatile uint32_t SSCGR;				// RCC spread spectrum clock generation register
	volatile uint32_t PLLI2SCFGR;			// RCC PLLI2S configuration register
//	volatile uint32_t PLLSAICFGR;			//
//	volatile uint32_t DCKCFGR;
//	volatile uint32_t CKGATENR;
//	volatile uint32_t DCKCFGR2;				//
} RCC_RegDef_t;


// peripheral definitions

#define GPIOA						((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB						((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC						((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD						((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE						((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF						((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG						((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH						((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI						((GPIO_RegDef_t*) GPIOI_BASEADDR)

#define RCC							((RCC_RegDef_t*) RCC_BASEADDR)

// Clock enable macros for GPIOx peripherals
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1 << 8))

// Clock enable macros for I2Cx peripherals
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 23))

// Clock enable macros for SPIx peripherals
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))

// Clock enable macros for USARTx peripherals 1 - 6, 4 & 5 UART
#define USART1_PLCK_EN()	(RCC->APB2ENR |= (1 << 4))
#define USART2_PLCK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PLCK_EN()	(RCC->APB1ENR |= (1 << 18))
#define UART4_PLCK_EN()		(RCC->APB1ENR |= (1 << 19))
#define UART5_PLCK_EN()		(RCC->APB1ENR |= (1 << 20))
#define USART6_PLCK_EN()	(RCC->APB2ENR |= (1 << 5))

// Clock enable macros for SYSCFG peripheral
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))

// Clock disable macros for GPIOx peripherals
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 8))

// Clock disable macros for I2Cx peripherals
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 23))

// Clock disable macros for SPIx peripherals
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 15))

// Clock disable macros for USARTx peripherals 1 - 6, 4 & 5 UART
#define USART1_PLCK_DI()	(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PLCK_DI()	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PLCK_DI()	(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PLCK_DI()		(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PLCK_DI()		(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PLCK_DI()	(RCC->APB2ENR &= ~(1 << 5))

// Clock disable macros for SYSCFG peripheral
#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 14))


#endif /* INC_STM32F407XX_H_ */
