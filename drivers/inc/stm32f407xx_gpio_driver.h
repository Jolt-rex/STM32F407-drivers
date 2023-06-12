/*
 * stm32f407xx_gpio_driver.h
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"


typedef struct
{
	// pointer to hold the base address of the GPIO peripheral
	GPIO_RegDef_t *pGPIOx;	// this holds the base address of the GPIO port to which the pin belongs



} GPIO_Handle_t;


#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
