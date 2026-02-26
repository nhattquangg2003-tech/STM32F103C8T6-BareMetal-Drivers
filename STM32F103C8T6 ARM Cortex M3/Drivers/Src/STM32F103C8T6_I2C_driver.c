/*
 * STM32F103C8T6_I2C_driver.c
 *
 *  Created on: Feb 16, 2026
 *      Author: Owner
 */

#include <stdint.h>
#include <STM32F103C8T6.h>
#include <STM32F103C8T6_I2C_driver.h>

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
    // Shift địa chỉ sang trái 1 để dành chỗ cho bit R/W
    SlaveAddr = SlaveAddr << 1;
    // Clear bit0 để chọn Write (R/W = 0)
    SlaveAddr &= ~(1);
    // Ghi vào thanh ghi dữ liệu
    pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
    // Shift địa chỉ sang trái 1 để dành chỗ cho bit R/W
    SlaveAddr = SlaveAddr << 1;
    // Set bit0 để chọn Read (R/W = 1)
    SlaveAddr |= 1;
    // Ghi vào thanh ghi dữ liệu
    pI2Cx->DR = SlaveAddr;
}


static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummy_read;
	// check for  the device mode
	if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
			{
			// master mode
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if (pI2CHandle->RxSize == 1)
			{
				// disable the ack
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
				// clear the addr flag
				// 1. read SR1
				dummy_read = pI2CHandle->pI2Cx->SR1;
				// 2. read SR2
				dummy_read = pI2CHandle->pI2Cx->SR2;
				// cast to void
				(void)dummy_read;
			}
		}
		else
		{
			// clear the addr flag
						// 1. read SR1
						dummy_read = pI2CHandle->pI2Cx->SR1;
						// 2. read SR2
						dummy_read = pI2CHandle->pI2Cx->SR2;
						// cast to void
						(void)dummy_read;
		}
			}
	else
			{
				// slave mode
		// clear the addr flag
								// 1. read SR1
								dummy_read = pI2CHandle->pI2Cx->SR1;
								// 2. read SR2
								dummy_read = pI2CHandle->pI2Cx->SR2;
								// cast to void
								(void)dummy_read;
			}

}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_STOP);
}

 void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        // Set ACK bit
        pI2Cx->CR1 |= (1U << 10);
    }
    else
    {
        // Clear ACK bit
        pI2Cx->CR1 &= ~(1U << 10);
    }
}


void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		    {
		        if(pI2Cx == I2C1)
		        {
		        	I2C1_PCLK_EN();
		        }
		        else if(pI2Cx == I2C2)
		        {
		        	I2C2_PCLK_EN();
		        }
		    }
		    else  // DISABLE
		    {
		    	if(pI2Cx == I2C1)
		    		        {
		    		I2C1_PCLK_DI();
		    		        }
		    		        else if(pI2Cx == I2C2)
		    		        {
		    		        	I2C2_PCLK_DI();
		    		        }
		    }
}
uint32_t RCC_GetPCLK1Value(void)
{
    uint32_t SystemClk = 0, pclk1;
    uint32_t clksrc, temp, ahbp, apb1p;

    clksrc = ((RCC->CFGR >> 2) & 0x3);

    if(clksrc == 0)
    {
        SystemClk = 16000000; // HSI = 16 MHz
    }
    else if(clksrc == 1)
    {
        SystemClk = 8000000;  // HSE = 8 MHz (thường dùng trên STM32F103C8T6)
    }
    else if(clksrc == 2)
    {
        // PLL selected
        uint32_t pll_src, pll_mul;
        pll_src = (RCC->CFGR >> 16) & 0x1;   // PLL source
        pll_mul = ((RCC->CFGR >> 18) & 0xF) + 2; // PLL multiplier (x2..x16)

        if(pll_src == 0)
        {
            // HSI/2 as PLL input
            SystemClk = (16000000 / 2) * pll_mul;
        }
        else
        {
            // HSE as PLL input
            SystemClk = 8000000 * pll_mul;
        }
    }

    // AHB prescaler
    temp = (RCC->CFGR >> 4) & 0xF;
    if(temp < 8)
    {
        ahbp = 1;
    }
    else
    {
        static const uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
        ahbp = AHB_PreScaler[temp - 8];
    }
    uint32_t Hclk = SystemClk / ahbp;

    // APB1 prescaler
    temp = (RCC->CFGR >> 8) & 0x7;
    if(temp < 4)
    {
        apb1p = 1;
    }
    else
    {
        static const uint8_t APB1_PreScaler[4] = {2,4,8,16};
        apb1p = APB1_PreScaler[temp - 4];
    }

    pclk1 = Hclk / apb1p;

    return pclk1;
}

//
//
//
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
    uint32_t tempreg = 0;

    // 1. ACK control
    tempreg = 0;
    tempreg |= pI2CHandle->I2C_Config.I2C_AckControl << 10;
    pI2CHandle->pI2Cx->CR1 = tempreg;

    // 2. Configure FREQ field of CR2
    tempreg = 0;
    tempreg |= (RCC_GetPCLK1Value() / 1000000U);
    pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

    // 3. Program device own address in OAR1
    tempreg = 0;
    tempreg |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1);
    tempreg |= (1 << 14); // bit ADDMODE = 1 (7-bit addressing)
    pI2CHandle->pI2Cx->OAR1 = tempreg;

    // 4. CCR calculation
    uint16_t ccr_value = 0;
    tempreg = 0;
    if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
    {
        // Standard mode
        ccr_value = RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
        tempreg |= (ccr_value & 0xFFF);
    }
    else
    {
        // Fast mode
        tempreg |= (1 << 15); // FS = 1
        tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);

        if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
        {
            ccr_value = RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
        }
        else
        {
            ccr_value = RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
        }
        tempreg |= (ccr_value & 0xFFF);
    }
    pI2CHandle->pI2Cx->CCR = tempreg;

    // 5. TRISE configuration
    if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
    {
        // Standard mode: TRISE = FREQ + 1
        pI2CHandle->pI2Cx->TRISE = ((RCC_GetPCLK1Value() / 1000000U) + 1);
    }
    else
    {
        // Fast mode: TRISE = (FREQ * 300ns) + 1
        pI2CHandle->pI2Cx->TRISE = (((RCC_GetPCLK1Value() / 1000000U) * 300) / 1000) + 1;
    }
}
//
//
//
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
			        {
		I2C1_REG_RESET();
			        }
			        else if(pI2Cx == I2C2)
			        {
			        	I2C2_REG_RESET();
			        }
}
//
//
//
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
			    {
			        if(IRQNumber <= 31)
			        {
			            // program ISER0 register
			        	*NVIC_ISER0 |= (1 << IRQNumber);

			        }
			        else if( IRQNumber > 31 && IRQNumber < 64 ) // 32 to 63
			        {
			        	// program ISER1 register
			        	*NVIC_ISER1 |= (1 << IRQNumber % 32);
			        }
			        else if(IRQNumber >= 64 && IRQNumber < 96 ) // 64 to 95
			        {
			        	// program ISER2 register
			        	*NVIC_ISER2 |= (1 << IRQNumber % 64);
			        }
			    }
			    else  // DISABLE
			    {
			        if(IRQNumber <= 31)
			    	        {
			    	            // program ISER0 register
			        	      *NVIC_ICER0  |= (1 << IRQNumber);

			    	        }
			    	        else if( IRQNumber > 31 && IRQNumber < 64 ) // 32 to 63
			    	        {
			    	        	// program ISER1 register
			    	        	*NVIC_ICER1 |= (1 << IRQNumber % 32);
			    	        }
			    	        else if(IRQNumber >= 64 && IRQNumber < 96 )
			    	        {
			    	        	// program ISER2 register
			    	        	*NVIC_ICER2 |= (1 << IRQNumber % 64);
			    	        }

			    }
}
//
//
//
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	// 1. first lets find out the ipr register
				uint8_t iprx = IRQNumber / 4;
				uint8_t iprx_section = IRQNumber % 4 ;
				uint8_t shift_amount = ( iprx_section*8 ) - ( 8- NO_PR_BITS_IMPLEMENT );
				*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (IRQPriority << shift_amount );
}
//
//
//
// // enable the I2C peripheral
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if(EnorDi == ENABLE)
			        {
		pI2Cx->CR1 |= (1 << I2C_CR1_PE );
			        }
			        else
			        {
			        	pI2Cx->CR1 &=  ~(1 << I2C_CR1_PE );
			        }
}
//
//
//
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
	         return FLAG_SET;
	}
	return FLAG_RESET;
}
//
//
//
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	// 1. generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	// 2. confirm that the start generation is complete by cheking the SB flag in the SR1
	// note: until SB is cleared SCL will be stretched (pulled to low)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));

	// 3. send the address of the slave with r/nw bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);

	// 4. confirm that address phase is completed by checking the ADDR flag in the SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR));

	// 5. clear the ADDR flag according to its software sequence
	// note: until ADDR is cleared SCL will be stretched (pulled to low)
	I2C_ClearADDRFlag(pI2CHandle);
	// 6. send the data until Len becomes 0
	while(Len > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE)); // wait till TXE set
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}
	// 7. when len becomes zero wait for txe = 1 and btf = 1 before generating the stop condition
	// note: txe = 1, btf = 1, means that both sr and dr are empty and next transmission should begin
	// when btf = 1 SCL will be stretched (pulled to low)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE)); // wait till TXE set
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_BTF)); // wait till BTF set
	// 8. generate STOP condition and master need not to wait for the completion of stop condition
		// note: generating stop, automatically clears the btf bit
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}
//
//
//
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,
                           uint8_t *pRxBuffer,
                           uint32_t Len,
                           uint8_t SlaveAddr)
{
    // 1. Generate the START condition
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

    // 2. Wait until SB flag is set
    while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

    // 3. Send slave address with read bit (1)
    I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,SlaveAddr);

    // 4. Wait until ADDR flag is set
    while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

    if(Len == 1)
    {
        // Disable ACK
        I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

        // Clear ADDR flag by reading SR1 and SR2
        I2C_ClearADDRFlag(pI2CHandle);

        // Wait until RXNE = 1
        while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

        // Generate STOP condition
        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

        // Read data into buffer
        *pRxBuffer = pI2CHandle->pI2Cx->DR;

        return;
    }

    if(Len > 1)
    {
        // Clear ADDR flag
    	I2C_ClearADDRFlag(pI2CHandle);

        for(uint32_t i = Len; i > 0; i--)
        {
            // Wait until RXNE = 1
            while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

            if(i == 2)
            {
                // Disable ACK
                I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

                // Generate STOP condition
                I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
            }

            // Read data into buffer
            *pRxBuffer = pI2CHandle->pI2Cx->DR;

            // Increment buffer pointer
            pRxBuffer++;
        }
    }

    // Re-enable ACK for future receptions
    if(pI2CHandle->I2C_Config.I2C_AckControl == ENABLE)
    {
        I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
    }

}
//
//
//
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,
                             uint8_t *pTxBuffer,
                             uint32_t Len,
                             uint8_t SlaveAddr,
                             uint8_t Sr)
{
    uint8_t busystate = pI2CHandle->TxRxState;

    if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
    {
        // 1. Lưu thông tin truyền vào handle
        pI2CHandle->pTxBuffer = pTxBuffer;
        pI2CHandle->TxLen     = Len;
        pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
        pI2CHandle->DevAddr   = SlaveAddr;
        pI2CHandle->Sr        = Sr;

        // 2. Tạo START condition
        pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);

        // 3. Enable interrupt cho event và error
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
    }

    return busystate;
}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,
                                uint8_t *pRxBuffer,
                                uint32_t Len,
                                uint8_t SlaveAddr,
                                uint8_t Sr)
{
    uint8_t busystate = pI2CHandle->TxRxState;

    if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
    {
        // 1. Lưu thông tin nhận vào handle
        pI2CHandle->pRxBuffer = pRxBuffer;
        pI2CHandle->RxLen     = Len;
        pI2CHandle->RxSize    = Len;
        pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
        pI2CHandle->DevAddr   = SlaveAddr;
        pI2CHandle->Sr        = Sr;

        // 2. Tạo START condition
        pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);

        // 3. Enable interrupt cho event và error
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
    }

    return busystate;
}
//
//
//
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->TxLen > 0) {

				// 1. load the data into DR
				pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
				// 2. decrement the TxLen
				pI2CHandle->TxLen--;
				// 3. Increment the buffet adddress
				pI2CHandle->pTxBuffer++;

}
}
//
//
//
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if (pI2CHandle->RxSize == 1)
				{
					pI2CHandle->pI2Cx->DR = *(pI2CHandle->pRxBuffer);
					pI2CHandle->RxLen--;
				}

				if (pI2CHandle->RxSize > 1)
						{
							if (pI2CHandle->RxLen == 2)
							{
								// clear the ack bit
								I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
							}

							// read DR
							*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
							pI2CHandle->pRxBuffer++;
										pI2CHandle->RxLen--;

						}

				if (pI2CHandle->RxLen == 0)
						{

	// close the i2c reception and notify the application

					// generate the stop condition
					 if (pI2CHandle->Sr == I2C_DISABLE_SR)
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					// close the i2c rx
					 I2C_CloseReceiveData(pI2CHandle);

					// notify the application
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
						}
}
//
//
//
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	// interupt handling for both master and slave mode of a device
	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITBUFEN);
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_SB);

	// 1. Handle for interupt generate by SB event
	// note : SB flag is only applicable in Master mode
	if (temp1 && temp3)
	{
		// SB flag is set
		// the interupt is generated by SB event
		// this block will not be executed in slave mode because for slave SB is always 0
		// in this block lets executed the address phase
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		} else 	if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_ADDR);
	// 2. Handle for interupt generate by ADDR event
	// note : when master mode: ADDRESS is sent
	// when slave mode: ADDRESS matched with own address
	if (temp1 && temp3)
		{
			// ADDR flag is set
		I2C_ClearADDRFlag(pI2CHandle);
		}


	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_BTF);
	// 3. Handle for interupt generate by BTF (Byte Transfer Finished) event
	if (temp1 && temp3)
	{
		// BTF flag is set
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				// make sure that txe is also set
			 if(pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE)){

				 // btf, txe = 1 => closse the transmission
				 // check the length = 0
				 // generate the stop
				 if(pI2CHandle->TxLen == 0){
				 if (pI2CHandle->Sr == I2C_DISABLE_SR)
				 I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				 // reset all the member elements of the handle structure
				 I2C_CloseSendData(pI2CHandle);
				 // 3. notify the application about transmission complete
				 I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);

			 }
			 }
			} else 	if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{

			}
	}


	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_STOPF);
	// 4. Handle for interupt generate by STOPF event
	// note : Stop dêtction flag is applicable only slave mode
	if (temp1 && temp3)
	{
		// STOPF flag is set
		// clear the STOPF flag > read SR1 > Write to CR1

		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		// notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP_CMPLT);

	}


	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE);
	// 5. Handle for interupt generate by TXE event
	if (temp1 && temp3 && temp2)
	{   // check for device mode
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
		{
		// TXE flag is set
		// we have to do the data transmission
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{

			I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		} else {
			// slave
			// make sure that the slave is really in transmitter mode
			if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA))
			{
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}



	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_RXNE);
	// 6. Handle for interupt generate by RXNE event
	if (temp1 && temp3 && temp2)
	{
		// check for device mode
				if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
				{

					// the device is master
		// RXNE flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_MasterHandleRXNEInterrupt(pI2CHandle);
		}

	} else
	{
		// slave
					// make sure that the slave is really in receiver mode
					if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA))
					{
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
					}
	}
	}
}
//
//
//
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
    // Disable interrupt cho buffer và event
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

    // Reset buffer và độ dài
    pI2CHandle->pRxBuffer = NULL;
    pI2CHandle->RxLen     = 0;
    pI2CHandle->RxSize    = 0;

    // Reset trạng thái
    pI2CHandle->TxRxState = I2C_READY;

    // Reset các trường khác
    pI2CHandle->DevAddr   = 0;
    pI2CHandle->Sr        = I2C_DISABLE_SR;
}
//
//
//
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
    // Disable interrupt cho buffer và event
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

    // Reset buffer và độ dài
    pI2CHandle->pTxBuffer = NULL;
    pI2CHandle->TxLen     = 0;

    // Reset trạng thái
    pI2CHandle->TxRxState = I2C_READY;

    // Reset các trường khác
    pI2CHandle->DevAddr   = 0;
    pI2CHandle->Sr        = I2C_DISABLE_SR;
}
//
//
//
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    uint32_t temp1, temp2;

    // Kiểm tra trạng thái bit ITERREN trong CR2
    temp2 = (pI2CHandle->pI2Cx->CR2) & (1 << I2C_CR2_ITERREN);

    /*********************** Bus Error ************************************/
    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_BERR);
    if(temp1 && temp2)
    {
        // Clear flag
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);
        // Notify application
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
    }

    /*********************** Arbitration Lost Error ************************************/
    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_ARLO);
    if(temp1 && temp2)
    {
        // Clear flag
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);
        // Notify application
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
    }

    /*********************** ACK Failure Error ************************************/
    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_AF);
    if(temp1 && temp2)
    {
        // Clear flag
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);
        // Notify application
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
    }

    /*********************** Overrun/Underrun Error ************************************/
    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_OVR);
    if(temp1 && temp2)
    {
        // Clear flag
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);
        // Notify application
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
    }

    /*********************** Timeout Error ************************************/
    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_TIMEOUT);
    if(temp1 && temp2)
    {
        // Clear flag
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);
        // Notify application
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
    }
}
//
//
//
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
	pI2Cx->DR = data;
}
//
//
//
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	return (uint8_t) pI2Cx->DR;
}


