/*
 * main.c
 *
 *  Created on: Jun 4, 2025
 *      Author: z1400
 */


#include "stm32f407xx.h"

int main(void)
{


	return 0;



}

void EXTI0_IRQHandler(void)
{
	GPIO_IRQHandling(0);



}
