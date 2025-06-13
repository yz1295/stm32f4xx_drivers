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
#define pressButton   high


void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}


int main(void)
{

	//initialize LED: PD12
	GPIO_Handle_t GpioLed;
	GPIO_Handle_t GPIOBtn;

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	//GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;


	GPIO_PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&GpioLed);

	//initialize User Button: PA0
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
		//GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	//GPIOBtn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;


	/*
	 * R39 (220kΩ) is already present between the switch node and GND.

This acts as a hardware pull-down resistor.

So in theory, you do not need to enable an internal pull-down in code — the external one should be enough.
So why did enabling the internal pull-down fix your issue?
Likely reasons:
High resistor value (220kΩ)

It's quite weak. With such a high resistance, noise or leakage may still cause the pin to float slightly above logic LOW.

STM32 inputs are high impedance, so even a tiny stray current can make the pin read as HIGH.

PCB layout and leakage paths

Sometimes on real boards (especially dev boards), capacitive coupling, board trace length, or nearby signals can inject noise.

Startup state

Before the code configures the GPIO pin, it may momentarily float.

The internal pull-down ensures a clean LOW level from the start of execution.
	 */
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PD;



		//GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOA,ENABLE);

	GPIO_Init(&GPIOBtn);

	while(1)
	{
		if (GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0)== pressButton)
		{
			delay();
			//delay();
			GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);


		}


	}
	return 0;
}

