/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: Jul 6, 2025
 *      Author: z1400
 */

#include "stm32f407xx_usart_driver.h"

/*********************************************************************
 * @fn      		  - USART_PeriClockControl
 *
 * @brief             - Enables or disables peripheral clock for given USARTx
 *
 * @param[in]         - pUSARTx: pointer to USART peripheral base address
 * @param[in]         - EnorDi : ENABLE or DISABLE macro
 *
 * @return            - None
 *
 * @Note              - None
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCCK_EN();
		}
		else if (pUSARTx == USART2)
		{
			USART2_PCCK_EN();
		}
		else if (pUSARTx == USART3)
		{
			USART3_PCCK_EN();
		}
		else if (pUSARTx == UART4)
		{
			UART4_PCCK_EN();
		}
		else if (pUSARTx == UART5)
		{
			UART5_PCCK_EN();
		}
		else if (pUSARTx == USART6)
		{
			USART6_PCCK_EN();
		}
	}
	else
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}
		else if (pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}
		else if (pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}
		else if (pUSARTx == UART4)
		{
			UART4_PCLK_DI();
		}
		else if (pUSARTx == UART5)
		{
			UART5_PCLK_DI();
		}
		else if (pUSARTx == USART6)
		{
			USART6_PCLK_DI();
		}
	}
}
/*********************************************************************
 * @fn      		  - USART_PeripheralControl
 *
 * @brief             - Enables or disables the USART peripheral
 *
 * @param[in]         - pUSARTx : Pointer to USART peripheral base address
 * @param[in]         - Cmd     : ENABLE or DISABLE macro
 *
 * @return            - None
 *
 * @Note              - This only controls the UE (USART Enable) bit in CR1.
 *                      Other configurations like baud rate, data bits, etc.
 *                      should be done before enabling the peripheral.
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t Cmd)
{
	if(Cmd == ENABLE)
	{
		pUSARTx->CR1 |= (1 << 13);  // Set UE bit to enable USART
	}
	else
	{
		pUSARTx->CR1 &= ~(1 << 13); // Clear UE bit to disable USART
	}
}
/*********************************************************************
 * @fn      		  - USART_GetFlagStatus
 *
 * @brief             - Checks whether the specified USART flag is set or not
 *
 * @param[in]         - pUSARTx         : Pointer to the USART peripheral base address
 * @param[in]         - StatusFlagName : Name of the flag to check (e.g., USART_SR_TXE)
 *
 * @return            - Flag status (SET or RESET)
 *
 * @Note              - The flag should be one of the macros defined for USART status register (SR)
 *                      e.g., (1 << 7) for TXE, (1 << 5) for RXNE, etc.
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName)
{
    if(pUSARTx->SR & StatusFlagName)
    {
    	return SET;
    }

    return RESET;
}
/*********************************************************************
 * @fn      		  - USART_ClearFlag
 *
 * @brief             - Clears specific USART status flags
 *
 * @param[in]         - pUSARTx         : Pointer to USART peripheral base address
 * @param[in]         - StatusFlagName : One of USART_FLAG_TC, USART_FLAG_CTS, USART_FLAG_LBD
 *
 * @return            - None
 *
 * @Note              - Only valid for bits that can be cleared by software:
 *                      TC, CTS, LBD. Others are read-only or cleared by reading DR.
 */
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
    // Only bits like TC, CTS, LBD are allowed to be cleared this way
    pUSARTx->SR &= ~(StatusFlagName);
}
/*********************************************************************
 * @fn      		  - USART_IRQInterruptConfig
 *
 * @brief             - Enables or disables a given IRQ number in NVIC
 *
 * @param[in]         - IRQNumber : IRQ number from the vector table
 * @param[in]         - EnorDi    : ENABLE or DISABLE macro
 *
 * @return            - None
 *
 * @Note              - Valid only for IRQ numbers 0–95
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber < 64)
		{
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber < 96)
		{
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));  // fixed: ISER3 → ISER2
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber < 64)
		{
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber < 96)
		{
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));  // fixed: ICER3 → ICER2
		}
	}
}
/*********************************************************************
 * @fn      		  - USART_IRQPriorityConfig
 *
 * @brief             - Sets priority for a given IRQ number
 *
 * @param[in]         - IRQNumber   : IRQ number from the vector table
 * @param[in]         - IRQPriority : Priority level (usually 0–15)
 *
 * @return            - None
 *
 * @Note              - Only upper bits [7:4] of each IPR byte are implemented (for STM32F4)
 */
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}
/*********************************************************************
 * @fn      		  - USART_Init
 *
 * @brief             - Initializes the given USART peripheral with the configuration
 *                      provided in USART_Handle_t structure.
 *
 * @param[in]         - pUSARTHandle : Pointer to USART handle structure
 *
 * @return            - None
 *
 * @Note              - This function must be called before enabling the USART peripheral.
 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	uint32_t tempreg = 0;

	/******************************** Enable USART Peripheral Clock ******************************************/
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	/******************************** Configuration of CR1 ******************************************/

	// 1. Configure USART mode (Tx/Rx)
	if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		tempreg |= (1 << USART_CR1_RE);  // Enable receiver
	}
	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		tempreg |= (1 << USART_CR1_TE);  // Enable transmitter
	}
	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		tempreg |= ((1 << USART_CR1_RE) | (1 << USART_CR1_TE));  // Enable both
	}

	// 2. Configure word length
	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M;

	// 3. Configure parity control
	if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		tempreg |= (1 << USART_CR1_PCE);  // Enable parity (even by default)
	}
	else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD)
	{
		tempreg |= (1 << USART_CR1_PCE);  // Enable parity
		tempreg |= (1 << USART_CR1_PS);   // Odd parity
	}

	// Write to CR1
	pUSARTHandle->pUSARTx->CR1 = tempreg;

	/******************************** Configuration of CR2 ******************************************/
	tempreg = 0;

	// Configure number of stop bits
	tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

	// Write to CR2
	pUSARTHandle->pUSARTx->CR2 = tempreg;

	/******************************** Configuration of CR3 ******************************************/
	tempreg = 0;

	// Configure hardware flow control
	if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		tempreg |= (1 << USART_CR3_CTSE);  // Enable CTS
	}
	else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		tempreg |= (1 << USART_CR3_RTSE);  // Enable RTS
	}
	else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		tempreg |= (1 << USART_CR3_CTSE);
		tempreg |= (1 << USART_CR3_RTSE);
	}

	// Write to CR3
	pUSARTHandle->pUSARTx->CR3 = tempreg;

	/******************************** Configuration of BRR (Baud Rate) ******************************************/
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}
/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             - Sends data using blocking mode
 *
 * @param[in]         - pUSARTHandle : Pointer to USART handle
 * @param[in]         - pTxBuffer    : Pointer to transmit buffer
 * @param[in]         - Len          : Number of bytes to send
 *
 * @return            - None
 *
 * @Note              - Supports 8-bit and 9-bit data length with or without parity
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *pdata;

	for(uint32_t i = 0 ; i < Len; i++)
	{
		// Wait until TXE (Transmit Data Register Empty) flag is set
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE));

		// Check word length: 9 bits
		if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				// 9 bits = 2 bytes
				pTxBuffer += 2;
			}
			else
			{
				// 8 bits data + 1 parity bit handled by hardware
				pTxBuffer++;
			}
		}
		else // Word length = 8 bits
		{
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t)0xFF);
			pTxBuffer++;
		}
	}

	// Wait for TC (Transmission Complete) flag
	while (! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));
}
/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             - Receives data using blocking mode
 *
 * @param[in]         - pUSARTHandle : Pointer to USART handle
 * @param[in]         - pRxBuffer    : Pointer to receive buffer
 * @param[in]         - Len          : Number of bytes to receive
 *
 * @return            - None
 *
 * @Note              - Supports 8-bit and 9-bit data length with or without parity
 */
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	for(uint32_t i = 0 ; i < Len; i++)
	{
		// Wait until RXNE (Read Data Register Not Empty) flag is set
		while (! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE));

		// Check word length: 9 bits
		if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x01FF);
				pRxBuffer += 2;
			}
			else
			{
				*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & 0xFF);
				pRxBuffer++;
			}
		}
		else // Word length = 8 bits
		{
			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & 0xFF);
			}
			else
			{
				*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & 0x7F); // 7 bits data
			}
			pRxBuffer++;
		}
	}
}
/*********************************************************************
 * @fn      		  - USART_SendDataIT
 *
 * @brief             - Begins transmitting data via interrupt (non-blocking)
 *
 * @param[in]         - pUSARTHandle : Pointer to USART handle
 * @param[in]         - pTxBuffer    : Pointer to transmit buffer
 * @param[in]         - Len          : Number of bytes to transmit
 *
 * @return            - Current Tx state (USART_BUSY_IN_TX or USART_READY)
 *
 * @Note              - Actual transmission is handled by ISR (IRQ handler)
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->TxBusyState;

	if (txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		// Enable TXE interrupt (triggers when DR is empty)
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

		// Enable TC interrupt (optional, for post-transmission notification)
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
	}

	return txstate;
}
/*********************************************************************
 * @fn      		  - USART_ReceiveDataIT
 *
 * @brief             - Begins receiving data via interrupt (non-blocking)
 *
 * @param[in]         - pUSARTHandle : Pointer to USART handle
 * @param[in]         - pRxBuffer    : Pointer to receive buffer
 * @param[in]         - Len          : Number of bytes to receive
 *
 * @return            - Current Rx state (USART_BUSY_IN_RX or USART_READY)
 *
 * @Note              - Actual reception is handled by ISR (IRQ handler)
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if (rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		// Dummy read of DR (optional sync step for some MCUs)
		(void)pUSARTHandle->pUSARTx->DR;

		// Enable RXNE interrupt (triggers when data is received)
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
	}

	return rxstate;
}

