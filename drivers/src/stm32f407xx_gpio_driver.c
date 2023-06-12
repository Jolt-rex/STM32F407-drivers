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
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNUMBER));
		pGPIOHandle->pGPIOx->MODER = temp;
	} else
	{
		// TODO: an interrupt mode
	}

	temp = 0;

	// configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNUMBER));
	pGPIOHandle->pGPIOx->OSPEEDR = temp;

	temp = 0;

	// configure the PUPD settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR = temp;

	temp = 0;

	// configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER = temp;

	temp = 0;

	// configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		// configure the alt function registers
	}

}


void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

}


// Data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

}


uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{

}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{

}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{

}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

}


// Interrupt configuration and ISR handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{

}

void GPIO_IRQHandling(uint8_t PinNumber)
{

}
