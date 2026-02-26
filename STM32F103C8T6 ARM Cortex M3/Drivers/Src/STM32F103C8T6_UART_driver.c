/*
 * STM32F103C8T6_UART_driver.c
 *
 *  Created on: Feb 25, 2026
 *      Author: Owner
 */
#include "STM32F103C8T6_UART_driver.h"
#include "STM32F103C8T6.h"
static uint32_t RCC_GetPCLK1Value(void);
static uint32_t RCC_GetPCLK2Value(void);
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pUSARTx->CR1 |= ( 1 << USART_CR1_UE );
	} else
	{
		pUSARTx->CR1 &= ~( 1 << USART_CR1_UE );
	}
}
//
//
//
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        if(pUSARTx == USART1)
        {
            USART1_PCLK_EN();   // macro bật clock USART1 (APB2)
        }
        else if(pUSARTx == USART2)
        {
            USART2_PCLK_EN();   // macro bật clock USART2 (APB1)
        }
        else if(pUSARTx == USART3)
        {
            USART3_PCLK_EN();   // macro bật clock USART3 (APB1)
        }
        else if(pUSARTx == UART4)
        {
            UART4_PCLK_EN();    // macro bật clock UART4 (APB1)
        }
        else if(pUSARTx == UART5)
        {
            UART5_PCLK_EN();    // macro bật clock UART5 (APB1)
        }
    }
    else  // DISABLE
    {
        if(pUSARTx == USART1)
        {
            USART1_PCLK_DI();   // macro tắt clock USART1
        }
        else if(pUSARTx == USART2)
        {
            USART2_PCLK_DI();   // macro tắt clock USART2
        }
        else if(pUSARTx == USART3)
        {
            USART3_PCLK_DI();   // macro tắt clock USART3
        }
        else if(pUSARTx == UART4)
        {
            UART4_PCLK_DI();    // macro tắt clock UART4
        }
        else if(pUSARTx == UART5)
        {
            UART5_PCLK_DI();    // macro tắt clock UART5
        }
    }
}
//
//
//
/**
 ******************************************************************************
 * @fn      uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName)
 * @brief   Kiểm tra trạng thái của một cờ trong SR.
 *
 * @param   [in] pUSARTx   Địa chỉ cơ sở của USART/UART.
 * @param   [in] FlagName  Tên cờ (ví dụ USART_SR_RXNE, USART_SR_TXE).
 *
 * @return  FLAG_SET hoặc FLAG_RESET.
 ******************************************************************************
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName)
{
    if(pUSARTx->SR & FlagName)
    {
        return FLAG_SET;
    }
    return FLAG_RESET;
}

/**
 ******************************************************************************
 * @fn      void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
 * @brief   Xóa một cờ trạng thái trong SR.
 *
 * @param   [in] pUSARTx          Địa chỉ cơ sở của USART/UART.
 * @param   [in] StatusFlagName   Tên cờ cần xóa (ví dụ USART_SR_TC, USART_SR_RXNE).
 *
 * @return  None
 *
 * @note    Một số cờ lỗi (ORE, NE, FE, PE) được xóa bằng cách đọc SR rồi đọc DR.
 *          Các cờ khác có thể xóa bằng cách ghi 0 vào bit tương ứng trong SR.
 ******************************************************************************
 */
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
    // Với STM32F1, hầu hết các cờ trong SR có thể xóa bằng cách ghi 0
    pUSARTx->SR &= ~(StatusFlagName);
}
/*********************************************************************
 * @fn      		  - USART_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{
    uint32_t tempreg = 0;

    /******************************** Configuration of CR1 ******************************************/

    // Enable the Clock for given USART peripheral
    USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

    // Enable USART Tx and Rx engines according to the USART_Mode configuration item
    if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
    {
        tempreg |= (1 << USART_CR1_RE);   // Receiver enable
    }
    else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
    {
        tempreg |= (1 << USART_CR1_TE);   // Transmitter enable
    }
    else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
    {
        tempreg |= (1 << USART_CR1_RE);   // Receiver enable
        tempreg |= (1 << USART_CR1_TE);   // Transmitter enable
    }

    // Configure the Word length
    tempreg |= (pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M);

    // Configuration of parity control bit fields
    if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
    {
        tempreg |= (1 << USART_CR1_PCE);  // Parity control enable
        // Even parity is default
    }
    else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD)
    {
        tempreg |= (1 << USART_CR1_PCE);  // Parity control enable
        tempreg |= (1 << USART_CR1_PS);   // Odd parity
    }

    // Program the CR1 register
    pUSARTHandle->pUSARTx->CR1 = tempreg;

    /******************************** Configuration of CR2 ******************************************/

    tempreg = 0;

    // Configure the number of stop bits
    tempreg |= (pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP);

    // Program the CR2 register
    pUSARTHandle->pUSARTx->CR2 = tempreg;

    /******************************** Configuration of CR3 ******************************************/

    tempreg = 0;

    // Configuration of USART hardware flow control
    if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
    {
        tempreg |= (1 << USART_CR3_CTSE);   // CTS enable
    }
    else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
    {
        tempreg |= (1 << USART_CR3_RTSE);   // RTS enable
    }
    else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
    {
        tempreg |= (1 << USART_CR3_CTSE);   // CTS enable
        tempreg |= (1 << USART_CR3_RTSE);   // RTS enable
    }

    // Program the CR3 register
    pUSARTHandle->pUSARTx->CR3 = tempreg;

    /******************************** Configuration of BRR (Baudrate register) ******************************************/

    // Configure the baud rate
    USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}
/**
 ******************************************************************************
 * @fn      void USART_DeInit(USART_RegDef_t *pUSARTx)
 * @brief   Reset peripheral USART/UART về trạng thái mặc định.
 *
 * @param   [in] pUSARTx   Địa chỉ cơ sở của USART/UART (USART1, USART2, USART3, UART4, UART5).
 *
 * @return  None
 *
 * @note    Hàm này sẽ gọi macro reset tương ứng trong RCC.
 *          Sau khi reset, tất cả các thanh ghi cấu hình của USART/UART sẽ trở về giá trị mặc định.
 ******************************************************************************
 */
void USART_DeInit(USART_RegDef_t *pUSARTx)
{
    if(pUSARTx == USART1)
    {
        USART1_REG_RESET();
    }
    else if(pUSARTx == USART2)
    {
        USART2_REG_RESET();
    }
    else if(pUSARTx == USART3)
    {
        USART3_REG_RESET();
    }
    else if(pUSARTx == UART4)
    {
        UART4_REG_RESET();
    }
    else if(pUSARTx == UART5)
    {
        UART5_REG_RESET();
    }
}


/*********************************************************************
 * @fn                - USART_SendData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{

    uint16_t *pdata;
   //Loop over until "Len" number of bytes are transferred
    for(uint32_t i = 0 ; i < Len; i++)
    {
        //Implement the code to wait until TXE flag is set in the SR
        while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx, (1 << USART_SR_TXE)));

         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
        if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
        {
            //if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
            pdata = (uint16_t*) pTxBuffer;
            pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

            //check for USART_ParityControl
            if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
            {
                //No parity is used in this transfer. so, 9bits of user data will be sent
                //Implement the code to increment pTxBuffer twice
                pTxBuffer++;
                pTxBuffer++;
            }
            else
            {
                //Parity bit is used in this transfer . so , 8bits of user data will be sent
                //The 9th bit will be replaced by parity bit by the hardware
                pTxBuffer++;
            }
        }
        else
        {
            //This is 8bit data transfer
            pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);

            //Implement the code to increment the buffer address
            pTxBuffer++;
        }
    }

    //Implement the code to wait till TC flag is set in the SR
    while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx, (1 << USART_SR_TC)));
}


/*********************************************************************
 * @fn                - USART_ReceiveData
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

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
   //Loop over until "Len" number of bytes are transferred
    for(uint32_t i = 0 ; i < Len; i++)
    {
        //Implement the code to wait until RXNE flag is set in the SR
        while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx, (1 << USART_SR_RXNE)));

        //Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
        if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
        {
            //We are going to receive 9bit data in a frame

            //check are we using USART_ParityControl control or not
            if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
            {
                //No parity is used. so, all 9bits will be of user data

                //read only first 9 bits. so, mask the DR with 0x01FF
                *((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

                //Now increment the pRxBuffer two times
                pRxBuffer++;
                pRxBuffer++;
            }
            else
            {
                //Parity is used, so, 8bits will be of user data and 1 bit is parity
                 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

                 //Increment the pRxBuffer
                pRxBuffer++;
            }
        }
        else
        {
            //We are going to receive 8bit data in a frame

            //check are we using USART_ParityControl control or not
            if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
            {
                //No parity is used , so all 8bits will be of user data

                //read 8 bits from DR
                 *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & 0xFF);
            }

            else
            {
                //Parity is used, so , 7 bits will be of user data and 1 bit is parity

                //read only 7 bits , hence mask the DR with 0X7F
                 *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & 0x7F);
            }

            //increment the pRxBuffer
            pRxBuffer++;
        }
    }

}

/*********************************************************************
 * @fn                - USART_SendDataWithIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
    uint8_t txstate = pUSARTHandle->TxBusyState;

    if(txstate != USART_BUSY_IN_TX)
    {
        pUSARTHandle->TxLen = Len;
        pUSARTHandle->pTxBuffer = pTxBuffer;
        pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

        //Implement the code to enable interrupt for TXE
        pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);


        //Implement the code to enable interrupt for TC
        pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);


    }

    return txstate;

}

/*********************************************************************
 * @fn                - USART_ReceiveDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 *********************************************************************/
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len)
{
    uint8_t rxstate = pUSARTHandle->RxBusyState;

    if(rxstate != USART_BUSY_IN_RX)
    {
        pUSARTHandle->RxLen = Len;
        pUSARTHandle->pRxBuffer = pRxBuffer;
        pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

        //Implement the code to enable interrupt for RXNE
        pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
    }

    return rxstate;
}
/*********************************************************************
 * @fn                - USART_SetBaudRate
 *
 * @brief             Cấu hình baud rate cho USART
 *
 * @param[in]         pUSARTx   : Peripheral USART cần cấu hình
 * @param[in]         BaudRate  : Giá trị baud rate mong muốn
 *
 * @return            None
 *
 * @Note              - Resolve all the TODOs
 *********************************************************************/
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
    //Variable to hold the APB clock
    uint32_t PCLKx;

    uint32_t usartdiv;

    //variables to hold Mantissa and Fraction values
    uint32_t M_part,F_part;

    uint32_t tempreg=0;

    //Get the value of APB bus clock in to the variable PCLKx
    if(pUSARTx == USART1)
    {
       //USART1 is hanging on APB2 bus
       PCLKx = RCC_GetPCLK2Value();
    }
    else
    {
       //USART2,3,4,5 are on APB1 bus
       PCLKx = RCC_GetPCLK1Value();
    }

    //Check for OVER8 configuration bit
    if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
    {
       //OVER8 = 1 , oversampling by 8
       usartdiv = ((25 * PCLKx) / (2 * BaudRate));
    }
    else
    {
       //OVER8 = 0 , oversampling by 16
       usartdiv = ((25 * PCLKx) / (4 * BaudRate));
    }

    //Calculate the Mantissa part
    M_part = usartdiv / 100;

    //Place the Mantissa part in appropriate bit position . refer USART_BRR
    tempreg |= (M_part << 4);

    //Extract the fraction part
    F_part = (usartdiv - (M_part * 100));

    //Calculate the final fractional
    if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
    {
      //OVER8 = 1 , oversampling by 8
      F_part = (((F_part * 8) + 50) / 100) & ((uint8_t)0x07);
    }
    else
    {
      //OVER8 = 0 , oversampling by 16
      F_part = (((F_part * 16) + 50) / 100) & ((uint8_t)0x0F);
    }

    //Place the fractional part in appropriate bit position . refer USART_BRR
    tempreg |= F_part;

    //copy the value of tempreg in to BRR register
    pUSARTx->BRR = tempreg;
}
//
//
//
static uint32_t RCC_GetPCLK1Value(void)
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
static uint32_t RCC_GetPCLK2Value(void)
{
    uint32_t SystemClk = 0, pclk2;
    uint32_t clksrc, temp, ahbp, apb2p;

    // Lấy nguồn clock hệ thống
    clksrc = ((RCC->CFGR >> 2) & 0x3);

    if(clksrc == 0)
    {
        SystemClk = 16000000; // HSI = 16 MHz
    }
    else if(clksrc == 1)
    {
        SystemClk = 8000000;  // HSE = 8 MHz
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

    // APB2 prescaler
    temp = (RCC->CFGR >> 11) & 0x7;
    if(temp < 4)
    {
        apb2p = 1;
    }
    else
    {
        static const uint8_t APB2_PreScaler[4] = {2,4,8,16};
        apb2p = APB2_PreScaler[temp - 4];
    }

    pclk2 = Hclk / apb2p;

    return pclk2;
}
//
//
//
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
///
//
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
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
/*********************************************************************
 * @fn                - USART_IRQHandler
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{

    uint32_t temp1 , temp2, temp3;
    uint16_t *pdata;
    // Lấy buffer từ handle
    uint8_t *pTxBuffer = pUSARTHandle->pTxBuffer;
    uint8_t *pRxBuffer = pUSARTHandle->pRxBuffer;
/*************************Check for TC flag ********************************************/

    //Implement the code to check the state of TC bit in the SR
    temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TC);

     //Implement the code to check the state of TCEIE bit
    temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TCIE);

    if(temp1 && temp2 )
    {
        //this interrupt is because of TC

        //close transmission and call application callback if TxLen is zero
        if ( pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
        {
            //Check the TxLen . If it is zero then close the data transmission
            if(! pUSARTHandle->TxLen )
            {
                //Implement the code to clear the TC flag
                pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_TC);

                //Implement the code to clear the TCIE control bit
                pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TCIE);

                //Reset the application state
                pUSARTHandle->TxBusyState = USART_READY;

                //Reset Buffer address to NULL
                pUSARTHandle->pTxBuffer = NULL;

                //Reset the length to zero
                pUSARTHandle->TxLen = 0;

                //Call the applicaton call back with event USART_EVENT_TX_CMPLT
                USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_TX_CMPLT);
            }
        }
    }

/*************************Check for TXE flag ********************************************/

    //Implement the code to check the state of TXE bit in the SR
    temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TXE);

    //Implement the code to check the state of TXEIE bit in CR1
    temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TXEIE);


    if(temp1 && temp2 )
    {
        //this interrupt is because of TXE

        if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
        {
            //Keep sending data until Txlen reaches to zero
            if(pUSARTHandle->TxLen > 0)
            {
                //Check the USART_WordLength item for 9BIT or 8BIT in a frame
                if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
                {
                    //if 9BIT , load the DR with 2bytes masking the bits other than first 9 bits
                	pdata = (uint16_t*) pTxBuffer;

                    //loading only first 9 bits , so we have to mask with the value 0x01FF
                    pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

                    //check for USART_ParityControl
                    if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
                    {
                        //No parity is used in this transfer , so, 9bits of user data will be sent
                        //Implement the code to increment pTxBuffer twice
                        pTxBuffer++;
                        pTxBuffer++;

                        //Implement the code to decrement the length
                        pUSARTHandle->TxLen -= 2;
                    }
                    else
                    {
                        //Parity bit is used in this transfer . so , 8bits of user data will be sent
                        //The 9th bit will be replaced by parity bit by the hardware
                        pTxBuffer++;

                        //Implement the code to decrement the length
                        pUSARTHandle->TxLen -= 1;
                    }
                }
                else
                {
                    //This is 8bit data transfer
                    pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);

                    //Implement the code to increment the buffer address
                    pTxBuffer++;

                    //Implement the code to decrement the length
                    pUSARTHandle->TxLen -= 1;
                }

            }
            if (pUSARTHandle->TxLen == 0 )
            {
                //TxLen is zero
                //Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
                pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TXEIE );
            }
        }
    }

/*************************Check for RXNE flag ********************************************/

    temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_RXNE);
    temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE);


    if(temp1 && temp2 )
    {
        //this interrupt is because of rxne
        //this interrupt is because of txe
        if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX)
        {
            //TXE is set so send data
            if(pUSARTHandle->RxLen > 0)
            {
                //Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
                if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
                {
                    //We are going to receive 9bit data in a frame

                    //Now, check are we using USART_ParityControl control or not
                    if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
                    {
                        //No parity is used. so, all 9bits will be of user data

                        //read only first 9 bits so mask the DR with 0x01FF
                        *((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

                        //Now increment the pRxBuffer two times
                        pRxBuffer++;
                        pRxBuffer++;

                        //Implement the code to decrement the length
                        pUSARTHandle->RxLen -= 2;
                    }
                    else
                    {
                        //Parity is used. so, 8bits will be of user data and 1 bit is parity
                         *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

                         //Now increment the pRxBuffer
                         pRxBuffer++;

                         //Implement the code to decrement the length
                         pUSARTHandle->RxLen -= 1;
                    }
                }
                else
                {
                    //We are going to receive 8bit data in a frame

                    //Now, check are we using USART_ParityControl control or not
                    if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
                    {
                        //No parity is used , so all 8bits will be of user data

                        //read 8 bits from DR
                         *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
                    }

                    else
                    {
                        //Parity is used, so , 7 bits will be of user data and 1 bit is parity

                        //read only 7 bits , hence mask the DR with 0X7F
                         *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

                    }

                    //Now , increment the pRxBuffer
                    pRxBuffer++;

                    //Implement the code to decrement the length
                    pUSARTHandle->RxLen -= 1;
                }


            }//if of >0

            if(! pUSARTHandle->RxLen)
            {
                //disable the rxne
                pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_RXNEIE );
                pUSARTHandle->RxBusyState = USART_READY;
                USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_RX_CMPLT);
            }
        }
    }


    /*************************Check for CTS flag ********************************************/
    //Note : CTS feature is not applicable for UART4 and UART5

        //Implement the code to check the status of CTS bit in the SR
        temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_CTS);

        //Implement the code to check the state of CTSE bit in CR1
        temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE);

        //Implement the code to check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)
        temp3 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSIE);

        if(temp1  && temp2 && temp3 )
        {
            //Implement the code to clear the CTS flag in SR
            pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_CTS);

            //this interrupt is because of cts
            USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_CTS);
        }

    /*************************Check for IDLE detection flag ********************************************/

        //Implement the code to check the status of IDLE flag bit in the SR
        temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_IDLE);

        //Implement the code to check the state of IDLEIE bit in CR1
        temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_IDLEIE);

        if(temp1 && temp2)
        {
            //Implement the code to clear the IDLE flag. Refer to the RM to understand the clear sequence
            volatile uint32_t dummy = pUSARTHandle->pUSARTx->SR;
            dummy = pUSARTHandle->pUSARTx->DR;
            (void)dummy;

            //this interrupt is because of idle
            USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_IDLE);
        }

    /*************************Check for Overrun detection flag ********************************************/

        //Implement the code to check the status of ORE flag  in the SR
        temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_ORE);

        //Implement the code to check the status of RXNEIE  bit in the CR1
        temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE);

        if(temp1  && temp2 )
        {
            //Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag .

            //this interrupt is because of Overrun error
            USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_ORE);
        }

    /*************************Check for Error Flag ********************************************/

    //Noise Flag, Overrun error and Framing Error in multibuffer communication
    //We dont discuss multibuffer communication in this course. please refer to the RM
    //The blow code will get executed in only if multibuffer mode is used.

        temp2 =  pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE) ;

        if(temp2 )
        {
            temp1 = pUSARTHandle->pUSARTx->SR;
            if(temp1 & ( 1 << USART_SR_FE))
            {
                /*
                    This bit is set by hardware when a de-synchronization, excessive noise or a break character
                    is detected. It is cleared by a software sequence (an read to the USART_SR register
                    followed by a read to the USART_DR register).
                */
                USART_ApplicationEventCallback(pUSARTHandle,USART_ERREVENT_FE);
            }

            if(temp1 & ( 1 << USART_SR_NE) )
            {
                /*
                    This bit is set by hardware when noise is detected on a received frame. It is cleared by a
                    software sequence (an read to the USART_SR register followed by a read to the
                    USART_DR register).
                */
                USART_ApplicationEventCallback(pUSARTHandle,USART_ERREVENT_NE);
            }

            if(temp1 & ( 1 << USART_SR_ORE) )
            {
                USART_ApplicationEventCallback(pUSARTHandle,USART_ERREVENT_ORE);
            }
        }
}





