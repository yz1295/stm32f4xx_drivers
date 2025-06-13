/*
 * stm32f407x_gpio_driver.c
 *
 *  Created on: Jun 4, 2025
 *      Author: z1400
 */

#include "stm32f407x_gpio_driver.h"
#include <assert.h>

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
				{
					GPIOA_PCLK_DI();
				}else if (pGPIOx == GPIOB)
				{
					GPIOB_PCLK_DI();
				}else if (pGPIOx == GPIOC)
				{
					GPIOC_PCLK_DI();
				}else if (pGPIOx == GPIOD)
				{
					GPIOD_PCLK_DI();
				}else if (pGPIOx == GPIOE)
				{
					GPIOE_PCLK_DI();
				}else if (pGPIOx == GPIOF)
				{
					GPIOF_PCLK_DI();
				}else if (pGPIOx == GPIOG)
				{
					GPIOG_PCLK_DI();
				}else if (pGPIOx == GPIOH)
				{
					GPIOH_PCLK_DI();
				}else if (pGPIOx == GPIOI)
				{
					GPIOI_PCLK_DI();
				}


	}

}





/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;
	//1.configure the mode of GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG )
	{
		//non-intertupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0;

		 // GPT: safe practice of modifying only the targeted pin configuration, rather than overwriting the entire register.
		/*if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
{
			uint32_t temp = 0;
			uint32_t pinNumber = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
			uint32_t pinMode   = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode;

			// Clear existing configuration for the pin
			pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pinNumber));

			// Set new mode for the pin
			temp = (pinMode << (2 * pinNumber));
			pGPIOHandle->pGPIOx->MODER |= temp;
}
		 * */


	}
	else
	{
		//interrupt mode
	}


    temp = 0;
	//2.configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;//calibrate temp



	//3.configure the pull-up/ pull-down setting

	uint8_t upANDdown = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl;
	uint8_t pinNumber = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;

	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2*pinNumber));

	temp = (upANDdown << (2*pinNumber));

	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;



	//4.configure the output type
    int8_t oType = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType;
    pinNumber = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;

    pGPIOHandle->pGPIOx->OTYPER &= ~(1 << pinNumber);
    temp = (oType << pinNumber);
    pGPIOHandle->pGPIOx->OTYPER |= temp;

    temp = 0;



	//5.configure the alternate functionality
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN )
    {
    	//configure the alternate functionality registers
    	uint8_t temp1=0;//temp1: AFRL or AFRH ;
    	uint8_t temp2 = 0;//temp2: which pin;

    	temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
    	temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

    	pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4*temp2));
    	pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));





    }



}
/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if (pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}

}


/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;

	value = (uint8_t)((pGPIOx->IDR >> PinNumber)&(0x1));

	return value;



}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	return (uint16_t)(pGPIOx->IDR);


}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
    if (Value == GPIO_PIN_SET)
    {
        pGPIOx->ODR |= (1 << PinNumber);
    }
    else if (Value == GPIO_PIN_RESET)
    {
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
    else
    {
        // Trigger a runtime error during development
        assert(!"Invalid Value passed to GPIO_WriteToOutputPin. Expected 0 or 1.");
    }
}


void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
    pGPIOx->ODR = Value;
    //optional safety mask:guarantee only lower 16 bits are written
    //pGPIOx->ODR = (Value & 0xFFFF);

}


void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1<<PinNumber);

}


/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}
void GPIO_IRQHandling(uint8_t PinNumber)
{

}


