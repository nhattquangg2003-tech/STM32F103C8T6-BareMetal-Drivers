/*
 * STM32F103C8T6_SPI_driver.c
 *
 *  Created on: Feb 15, 2026
 *      Author: Owner
 */

#include "STM32F103C8T6_SPI_driver.h"
#include <stdint.h>
#include <STM32F103C8T6.h>

static void SPI_TXE_INTERRUPT_HANDLE(SPI_Handle_t *pSPIHandle);
static void SPI_RXNE_INTERRUPT_HANDLE(SPI_Handle_t *pSPIHandle);
static void SPI_OVR_ERR_INTERRUPT_HANDLE(SPI_Handle_t *pSPIHandle);

/**
 ******************************************************************************
 * @fn      void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
 * @brief   Bật hoặc tắt xung clock cho peripheral SPI được chọn.
 *
 * @param   [in] pSPIx   Địa chỉ cơ sở của peripheral SPI (SPI1, SPI2)
 * @param   [in] EnOrDi  Macro ENABLE hoặc DISABLE
 *
 * @return  Không có (void)
 *
 * @note    Hàm này phải được gọi trước khi sử dụng bất kỳ API nào của SPI.
 *          Nếu không bật clock, việc truy cập thanh ghi SPI sẽ không hoạt động.
 ******************************************************************************
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	 if(EnorDi == ENABLE)
	    {
	        if(pSPIx == SPI1)
	        {
	        	SPI1_PCLK_EN();
	        }
	        else if(pSPIx == SPI2)
	        {
	        	SPI2_PCLK_EN();
	        }
	    }
	    else  // DISABLE
	    {
	    	if(pSPIx == SPI1)
	    		        {
	    		        	SPI1_PCLK_DI();
	    		        }
	    		        else if(pSPIx == SPI2)
	    		        {
	    		        	SPI2_PCLK_DI();
	    		        }
	    }
};

/*
 * INIT AND DEINIT
 */
/**
 ******************************************************************************
 * @fn      void SPI_Init(SPI_Handle_t *pSPIHandle)
 * @brief   Khởi tạo và cấu hình peripheral SPI theo thông số trong SPI_Config_t.
 *
 * @param   [in] pSPIHandle   Con trỏ tới SPI handle, chứa địa chỉ SPIx và
 *                            cấu hình SPI (DeviceMode, BusConfig, SclkSpeed,
 *                            DFF, CPOL, CPHA, SSM).
 *
 * @return  Không có (void)
 *
 * @note    Hàm này sẽ ghi cấu hình vào thanh ghi CR1 của SPI.
 *          Trước khi gọi hàm, cần bật clock cho SPI bằng SPI_PeriClockControl().
 *          Nếu chưa enable clock, việc ghi thanh ghi sẽ không có hiệu lực.
 ******************************************************************************
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	// peripheral clock enable

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);



	// first lets configure the SPI_CR1 register

	uint32_t tempreg = 0;

	// 1. config the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	// 2. config the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		// bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
		// bidi mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
	// bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		// rxonly bit must be set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}
	// 3. config the spi serial clock speed
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;        // BIT THU 3
	// 4. CONFIG THE DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;    // BIT THU 11
	// 5. config the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;     // BIT THU 1
	// 6. CONFIG THE CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;    // BIT THU 0

	pSPIHandle->pSPIx->CR1 = tempreg;
}
/**
 ******************************************************************************
 * @fn      void SPI_DeInit(SPI_RegDef_t *pSPIx)
 * @brief   Reset peripheral SPI về trạng thái mặc định bằng cách sử dụng
 *          macro reset tương ứng trong RCC.
 *
 * @param   [in] pSPIx   Địa chỉ cơ sở của peripheral SPI (SPI1, SPI2)
 *
 * @return  Không có (void)
 *
 * @note    Hàm này sẽ gọi macro SPIx_REG_RESET() để thực hiện reset.
 *          Sau khi reset, tất cả các thanh ghi cấu hình của SPI sẽ trở về
 *          giá trị mặc định như lúc khởi động hệ thống.
 ******************************************************************************
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
		        {
		SPI1_REG_RESET();
		        }
		        else if(pSPIx == SPI2)
		        {
		        	SPI2_REG_RESET();
		        }
}
// // enable the spi peripheral
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE)
			        {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
			        }
			        else
			        {
			        	pSPIx->CR1 &=  ~(1 << SPI_CR1_SPE);
			        }
}
// enable the spi ssi, khi chọn chế độ ssm thì phải config luôn cả bit ssi nếu không sẽ tự reset về slave mà không chọn master
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
				        {
			pSPIx->CR1 |= (1 << SPI_CR1_SSI);
				        }
				        else
				        {
				        	pSPIx->CR1 &=  ~(1 << SPI_CR1_SSI);
				        }
}
/*
 * DATA SEND AND RECEIVE
 */
/**
 ******************************************************************************
 * @fn      void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
 * @brief   Gửi dữ liệu qua SPI ở chế độ blocking.
 *
 * @param   [in] pSPIx      Địa chỉ cơ sở của SPI.
 * @param   [in] pTxBuffer  Con trỏ tới buffer dữ liệu cần gửi.
 * @param   [in] Len        Độ dài dữ liệu.
 *
 * @return  None
 *
 * @note    Hàm này chờ TXE flag trước khi ghi dữ liệu vào DR.
 ******************************************************************************
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
	         return FLAG_SET;
	}
	return FLAG_RESET;
}
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{

	    while(Len > 0) {
	        // 1. chờ TXE = 1
	        while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

	        // 2. kiểm tra DFF
	        if( (pSPIx->CR1 & (1 << SPI_CR1_DFF)) ) {
	            // 16-bit DFF
	            pSPIx->DR = *((uint16_t*)pTxBuffer);
	            Len -= 2;
	            pTxBuffer += 2;   // tăng con trỏ đúng 2 byte
	        } else {
	            // 8-bit DFF
	            pSPIx->DR = *pTxBuffer;
	            Len--;
	            pTxBuffer++;
	        }
	    }
	}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
    while(Len > 0) {
        // 1. chờ RXNE = 1
        while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

        // 2. kiểm tra DFF
        if( (pSPIx->CR1 & (1 << SPI_CR1_DFF)) ) {
            // 16-bit DFF
            *((uint16_t*)pRxBuffer) = pSPIx->DR;
            Len -= 2;
            pRxBuffer += 2;   // tăng con trỏ đúng 2 byte
        } else {
            // 8-bit DFF
            *pRxBuffer = pSPIx->DR;
            Len--;
            pRxBuffer++;
        }
    }
}
/*
 * IRQ CONFIGURATION AND ISR HANDLING
 */
void SPI_IRQ_Interupt_Config(uint8_t IRQNumber,  uint8_t EnorDi)
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


void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;
// first lets check for TXE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE);

	if ( temp1 && temp2) {

		// handle TXE
		SPI_TXE_INTERRUPT_HANDLE(pHandle);

	}

	// 2nd lets check for RXNE
		temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE);
		temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);

		if ( temp1 && temp2) {

				// handle RXNE
				SPI_RXNE_INTERRUPT_HANDLE(pHandle);

			}

		// 3nd lets check for OVER flag
			temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
			temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

			if ( temp1 && temp2) {

					// handle ovr error
					SPI_OVR_ERR_INTERRUPT_HANDLE(pHandle);

				}

}

void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
	// 1. first lets find out the ipr register
			uint8_t iprx = IRQNumber / 4;
			uint8_t iprx_section = IRQNumber % 4 ;
			uint8_t shift_amount = ( iprx_section*8 ) - ( 8- NO_PR_BITS_IMPLEMENT );
			*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (IRQPriority << shift_amount );
}



uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX) {
	// 1. Save the Tx buffer address and Len information in some global vairables
	pSPIHandle->pTxBuffet = pTxBuffer;
	pSPIHandle->TxLen = Len;
	// 2. Mark the SPI state as busy in transmission so that
	// no other code can take over same SPI peripheral until transmission is over
	pSPIHandle->TxState = SPI_BUSY_IN_TX;
	// 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
    pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE );
	// 4. Data transmission will be handled by the ISR code ( will implement later )
	}
	return state;
}
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;
		if(state != SPI_BUSY_IN_RX) {
		// 1. Save the Tx buffer address and Len information in some global vairables
		pSPIHandle->pRxBuffet = pRxBuffer;
		pSPIHandle->RxLen = Len;
		// 2. Mark the SPI state as busy in transmission so that
		// no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		// 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
	    pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE );
		// 4. Data transmission will be handled by the ISR code ( will implement later )
		}
		return state;
}

// some helper function implementations

static void SPI_TXE_INTERRUPT_HANDLE(SPI_Handle_t *pSPIHandle)
{
	        // 2. kiểm tra DFF
	        if( (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) ) {
	            // 16-bit DFF
	        	pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffet);
	        	pSPIHandle->TxLen -= 2;
	        	pSPIHandle->pTxBuffet += 2;   // tăng con trỏ đúng 2 byte
	        } else {
	            // 8-bit DFF
	        	pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffet);
	        	pSPIHandle->TxLen--;
	        	pSPIHandle->pTxBuffet++;
	        }
	        if(! pSPIHandle->TxLen)
	        {
	        	// txlen is zero, so closse the spi transmission and inform the application that
	        	// tx is over.
	        	// this prevents interrupt from setting up of txe flag
	        	SPI_CloseTransmisson(pSPIHandle);
	        	SPI_ApplicationCallback(pSPIHandle,SPI_EVENT_TX_COMPLT);

	        }

}
static void SPI_RXNE_INTERRUPT_HANDLE(SPI_Handle_t *pSPIHandle)
{
	// 2. kiểm tra DFF
	        if( (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) ) {
	            // 16-bit DFF
	            *((uint16_t*)pSPIHandle->pRxBuffet) = (uint16_t)pSPIHandle->pSPIx->DR;
	            pSPIHandle->RxLen -= 2;
	            pSPIHandle->pRxBuffet += 2;   // tăng con trỏ đúng 2 byte
	        } else {
	            // 8-bit DFF
	            *(pSPIHandle->pRxBuffet) = (uint8_t)pSPIHandle->pSPIx->DR;
	            pSPIHandle->RxLen--;
	            pSPIHandle->pRxBuffet++;
	        }
	        if(! pSPIHandle->RxLen)
	     	        {
	     	        	// rxlen is zero, so closse the spi transmission and inform the application that
	     	        	// rx is over.
	     	        	// this prevents interrupt from setting up of rxne flag
	        	SPI_CloseReception(pSPIHandle);
	     	        	SPI_ApplicationCallback(pSPIHandle,SPI_EVENT_RX_COMPLT);

	     	        }
}
static void SPI_OVR_ERR_INTERRUPT_HANDLE(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	// 1. clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
	temp = pSPIHandle->pSPIx->DR;
	temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	// 2, infrom the application
	SPI_ApplicationCallback(pSPIHandle,SPI_EVENT_OVR_ERR);

}
void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pRxBuffet = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffet = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}


__weak void SPI_ApplicationCallback(SPI_Handle_t *pHandle,uint8_t AppEv)
{

	// this is a week implementation . the application may override this function

}


