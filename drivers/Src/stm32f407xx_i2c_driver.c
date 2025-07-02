/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Jun 30, 2025
 *      Author: z1400
 */
#include "stm32f407xx_i2c_driver.h"
/*********************************************************************
 * @fn      		  - RCC_GetPCLK1Value
 *
 * @brief             - Returns the current PCLK1 (APB1 peripheral clock) frequency
 *
 * @param[in]         - None
 *
 * @return            - The frequency of the APB1 peripheral clock in Hz
 *
 * @Note              - This function reads system clock source and prescaler values
 *                      from the RCC_CFGR register, and calculates the effective
 *                      APB1 clock frequency (PCLK1). The function assumes that
 *                      the PLL output frequency (if used) is returned by an external
 *                      function `RCC_GetPLLOutputClock()` which must be implemented.
 ********************************************************************/
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, system_clk;
	uint8_t clk_src, ahb_pre, apb1_pre;

	uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
	uint8_t  APB1_PreScaler[4] = {2, 4, 8, 16};

	clk_src = (RCC->CFGR >> 2) & 0x3;

	if(clk_src == 0)
	{
		system_clk = 16000000; // HSI
	}
	else if(clk_src == 1)
	{
		system_clk = 8000000; // HSE
	}
	else if(clk_src == 2)
	{
		// Assume PLL is set to 84MHz for STM32F407 as per standard config
		system_clk = 84000000;
	}

	// AHB prescaler
	ahb_pre = (RCC->CFGR >> 4) & 0xF;
	uint32_t ahb_div = (ahb_pre < 8) ? 1 : AHB_PreScaler[ahb_pre - 8];

	// APB1 prescaler
	apb1_pre = (RCC->CFGR >> 10) & 0x7;
	uint32_t apb1_div = (apb1_pre < 4) ? 1 : APB1_PreScaler[apb1_pre - 4];

	pclk1 = (system_clk / ahb_div) / apb1_div;

	return pclk1;
}




/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
 *
 * @brief             - Enables or disables peripheral clock for the given I2C module
 *
 * @param[in]         - pI2Cx: Base address of the I2C peripheral (I2C1, I2C2, I2C3)
 * @param[in]         - EnorDi: ENABLE or DISABLE macro to turn the clock ON or OFF
 *
 * @return            - None
 *
 * @Note              - This function must be called before using the I2C peripheral
 *                      to ensure its clock is enabled. Disabling the clock can save
 *                      power when the peripheral is not in use.
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}



/*********************************************************************
 * @fn      		  - I2C_PeripheralControl
 *
 * @brief             - Enables or disables the I2C peripheral
 *
 * @param[in]         - pI2Cx : Pointer to the I2C peripheral base address
 * @param[in]         - EnOrDi : ENABLE to enable the peripheral, DISABLE to disable it
 *
 * @return            - None
 *
 * @Note              - This function sets or clears the PE (Peripheral Enable) bit
 *                      in the I2C_CR1 register to turn the I2C peripheral ON or OFF.
 *                      It must be enabled before starting any communication.
 *********************************************************************/

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
		//pI2cBaseAddress->CR1 |= I2C_CR1_PE_Bit_Mask;
	}else
	{
		pI2Cx->CR1 &= ~(1 << 0);
	}

}
/*********************************************************************
 * @fn      		  - I2C_Init
 *
 * @brief             - Initializes the I2C peripheral with the specified configuration
 *
 * @param[in]         - pI2CHandle : Pointer to the I2C handle structure containing
 *                                   the peripheral base address and configuration settings
 *
 * @return            - None
 *
 * @Note              - This function must be called before using the I2C peripheral.
 *                      It performs the following configuration steps:
 *                      1. Enables the peripheral clock
 *                      2. Configures ACK control
 *                      3. Sets the peripheral clock frequency in CR2
 *                      4. Programs the device own address in OAR1
 *                      5. Calculates and sets the CCR (Clock Control Register) for SCL
 *                      6. Configures the maximum rise time (TRISE)
 *                      Assumes that `RCC_GetPCLK1Value()` returns the correct APB1 clock.
 *********************************************************************/

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0 ;

	//enable the clock for the i2cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx,ENABLE);

	//ack control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_AckControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//configure the FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() /1000000U ;
	pI2CHandle->pI2Cx->CR2 =  (tempreg & 0x3F);

   //program the device own address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= ( 1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		ccr_value = (RCC_GetPCLK1Value() / ( 2 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		tempreg |= (ccr_value & 0xFFF);
	}else
	{
		//mode is fast mode
		tempreg |= ( 1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		}else
		{
			ccr_value = (RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE Configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode

		tempreg = (RCC_GetPCLK1Value() /1000000U) + 1 ;

	}else
	{
		//mode is fast mode
		tempreg = ( (RCC_GetPCLK1Value() * 300) / 1000000000U ) + 1;

	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);

}
/*********************************************************************
 * @fn      		  - I2C_DeInit
 *
 * @brief             - Resets the given I2C peripheral registers to their default reset values
 *
 * @param[in]         - pI2Cx : Pointer to the I2C peripheral base address (I2C1, I2C2, or I2C3)
 *
 * @return            - None
 *
 * @Note              - This function uses RCC reset macros to reset the I2C peripheral.
 *                      It should only be used when the peripheral is no longer needed or
 *                      before re-initializing it.
 *********************************************************************/
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if (pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}
	else if (pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}
	else if (pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
}
