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
		return Flag_SET;
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

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t Len)
{
	while(Len >0)
	{
		//1.wait till Tx buffer is  empty
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG)== FLAG_RESET);
		//while(!(pSPIx->SR &(1<<1)));

		//check the DFF bit in CR1
		if(pSPIx->CR1&(1<<SPI_CR1_DFF))
		{
			// 16 bit DFF
			//load the data into the DR
			pSPI->DR = *((uint16_t*)pTxBuffer);
			Len-=2;// 16 bit, I send 2 bytes so decrease 2

			(uint16_t*)pTxBuffer++;


		}
		else
		{
			//8 bit DFF
			pSPI->DR = *pTxBuffer;
			Len--;
			*pTxBuffer++;
		}


	}

}
