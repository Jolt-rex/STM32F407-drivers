/*
 * stm32f407xx_gpio_driver.c
 */


#include "stm32f407xx_gpio_driver.h"


// Peripheral clock setup
/****************************************************
 * @fn				- GPIO_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		- base address of the GPIO peripheral
 * @param[in]		- ENABLE or DISABLE macro
 *
 * return			- none
 *
 * @note			- none
 *
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		} else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		} else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		} else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		} else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		} else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		} else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		} else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		} else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		} else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		} else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		} else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		} else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		} else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		} else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		} else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		} else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}
}

// Init and de-init
/****************************************************
 * @fn				- GPIO_Init
 *
 * @brief			- This function initializes a GPIO pin
 *
 * @param[in]		- GPIO PIN handler
 *
 * return			- none
 *
 * @note			- none
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;
	// configure mode of GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// a non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		// clear the 2 bit's first (0b11)
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		// set the MODE bits
		pGPIOHandle->pGPIOx->MODER |= temp;
	} else
	{
		// TODO: an interrupt mode
	}

	temp = 0;

	// configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp; // setting

	temp = 0;

	// configure the PUPD settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp; // setting

	temp = 0;

	// configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp; // setting

	temp = 0;

	// configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		// configure the alt function registers
		uint8_t altFunRegIndex = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		uint8_t pinOffset = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[altFunRegIndex] &= ~(0xF << (4 * pinOffset)); // clearing
		pGPIOHandle->pGPIOx->AFR[altFunRegIndex] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * pinOffset)); // setting

	}

}


void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	} else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	} else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	} else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	} else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	} else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	} else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	} else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	} else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
}


// Data read and write
/****************************************************
 * @fn				- GPIO_ReadFromInputPin
 *
 * @brief			- This function reads the value from a given input pin
 *
 * @param[in]		- base address of the GPIO peripheral
 * @param[in]		- pin number
 *
 * return			- 0 or 1 value
 *
 * @note			- none
 *
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/****************************************************
 * @fn				- GPIO_ReadFromInputPin
 *
 * @brief			- This function reads the value from a given input pin
 *
 * @param[in]		- base address of the GPIO peripheral
 *
 * return			- 16bit value of the entire GPIO port
 *
 * @note			- none
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	return (uint16_t)pGPIOx->IDR;
}

/****************************************************
 * @fn				- GPIO_WriteToOutputPin
 *
 * @brief			- This function writes the given value to the pin number
 *
 * @param[in]		- base address of the GPIO peripheral
 * @param[in]		- pin number to write to
 * @param[in]		- value to be written 0 or 1
 *
 * return			- None
 *
 * @note			- none
 *
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		// write 1 to the output data register at the bit field of the pin number
		pGPIOx->ODR |= (1 << PinNumber);
	} else
	{
		// write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/****************************************************
 * @fn				- GPIO_WriteToOutputPort
 *
 * @brief			- This function writes the 16bit value to the GPIO port
 *
 * @param[in]		- base address of the GPIO peripheral
 * @param[in]		- 16bit value to be written
 *
 * return			- none
 *
 * @note			- none
 *
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/****************************************************
 * @fn				- GPIO_ToggleOutputPin
 *
 * @brief			- This function toggles the output pin state between 0 and 1
 *
 * @param[in]		- base address of the GPIO peripheral
 * @param[in]		- pin number
 *
 * return			- none
 *
 * @note			- none
 *
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}


// Interrupt configuration and ISR handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{

}

void GPIO_IRQHandling(uint8_t PinNumber)
{

}
