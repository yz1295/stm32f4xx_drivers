/*
 * stm32f407xx_spi_driver.c
 *
 *      Author: z1400
 */
#include "stm32f407xx_spi_driver.h"

/***********************************************************
 * @fn      		  - SPI_PeriClockControl
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
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
	else
	{
		if (pSPIx == SPI1)
				{
					SPI1_PCLK_DI();
				}
		else if (pSPIx == SPI2)
				{
					SPI2_PCLK_DI();
				}
		else if (pSPIx == SPI3)
				{
					SPI3_PCLK_DI();
				}
	}
}



/*********************************************************************
 * @fn      		  - SPI_Init
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
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//Enable the SPI clock
	SPI_PeriClockControl(pSPIHandle->pSPIx,ENABLE);

	//peripheral clock enable

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//first lets configure the SPI_CR1 register

	uint32_t tempreg = 0;

	//1. configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR ;

	//2. Configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE);

	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= ( 1 << SPI_CR1_BIDIMODE);
	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI mode should be cleared
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempreg |= ( 1 << SPI_CR1_RXONLY);
	}

	// 3. Configure the spi serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4.  Configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6 . configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;

}

/*********************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             - Resets the SPI peripheral by asserting and deasserting
 *                      its reset bit in the RCC reset register.
 *
 * @param[in]         - pSPIx: Pointer to the SPI peripheral (SPI1, SPI2, or SPI3)
 *
 * @return            - None
 *
 * @Note              - Only SPI1, SPI2, and SPI3 are supported in this function.
 *                      This does not disable SPI clocks or reset GPIO pins.
 *********************************************************************/

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
    if (pSPIx == SPI1)
    {
        SPI1_RESET();
    }
    else if (pSPIx == SPI2)
    {
        SPI2_RESET();
    }
    else if (pSPIx == SPI3)
    {
        SPI3_RESET();
    }
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;

}
/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             - Sends data over SPI in a non-blocking (polling-based) manner.
 *
 * @param[in]         - pSPIx: Pointer to SPI peripheral base address (SPI1, SPI2, etc.)
 * @param[in]         - pTxBuffer: Pointer to the data buffer to be transmitted
 * @param[in]         - Len: Number of bytes to be transmitted
 *
 * @return            - None
 *
 * @Note              - This function uses polling, but still blocks the CPU until
 *                      all data is sent. It supports both 8-bit and 16-bit DFF modes.
 *                      For true non-blocking behavior, use interrupt- or DMA-based send.
 *********************************************************************/

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while (Len > 0)
	{
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		if (pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			// 16-bit DFF
			pSPIx->DR = *((uint16_t*)pTxBuffer);  // Correct pointer cast
			Len -= 2;
			pTxBuffer += 2;  // Move 2 bytes forward
		}
		else
		{
			// 8-bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;  // Move 1 byte forward
		}
	}
}
/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
 *
 * @brief             - Enables or disables the SPI peripheral.
 *
 * @param[in]         - pSPIx: Pointer to the SPI peripheral (SPI1, SPI2, or SPI3)
 * @param[in]         - EnOrDi: ENABLE to turn on the peripheral, DISABLE to turn it off
 *
 * @return            - None
 *
 * @Note              - This function sets or clears the SPE (SPI Enable) bit in the CR1 register.
 *                      The SPI must be disabled (SPE=0) before configuring certain registers.
 *********************************************************************/

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |=  (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &=  ~(1 << SPI_CR1_SPE);
	}


}
/*********************************************************************
 * @fn      		  - SPI_SSIConfig
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
void  SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |=  (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &=  ~(1 << SPI_CR1_SSI);
	}


}
/*********************************************************************
 * @fn      		  - SPI_SSOEConfig
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
void  SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |=  (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &=  ~(1 << SPI_CR2_SSOE);
	}


}

