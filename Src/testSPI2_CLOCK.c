/*
 * testSPI2_CLOCK.c
 *
 *  Created on: Jun 25, 2025
 *      Author: z1400
 */


#include "stm32f407xx.h"

void delay(void)
{
    for (volatile uint32_t i = 0; i < 100000; i++);
}

int main(void)
{
    // Enable SPI1 clock (APB2, bit 12)
    RCC->APB2ENR |= (1 << 12);
    delay(); // Place a breakpoint here and check RCC->APB2ENR

    // Enable SPI2 clock (APB1, bit 14)
    RCC->APB1ENR |= (1 << 14);
    delay(); // Place a breakpoint here and check RCC->APB1ENR

    while (1)
    {
        // Optional: Toggle a dummy flag or breakpoint here to keep MCU alive
    }
}
