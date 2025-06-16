/*
 * externalButtonLED.c
 *
 *  Created on: Jun 13, 2025
 *      Author: z1400
 */


/*
 * 002ledButton.c
 *
 *  Created on: Jun 12, 2025
 *      Author: z1400
 *
 *     Test the driver I writ: use User button to trigger the GPIO toggle
 */

/*
 * 001led_toggle.c
 *
 */

#include "stm32f407xx.h"

#define high           1
#define low            0
#define pressButton   low


void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}


int main(void)
{

	//initialize LED: PD12
	GPIO_Handle_t GpioLed;
	GPIO_Handle_t GPIOBtn;

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	//GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;


	GPIO_PeriClockControl(GPIOA,ENABLE);

	GPIO_Init(&GpioLed);

	//
	GPIOBtn.pGPIOx = GPIOB;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
		//GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	//GPIOBtn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;


	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;



		//GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOB,ENABLE);

	GPIO_Init(&GPIOBtn);

	while(1)
	{
		if (GPIO_ReadFromInputPin(GPIOB,GPIO_PIN_NO_12)== pressButton)
		{
			delay();
			//delay();
			GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_8);


		}


	}
	return 0;
}

