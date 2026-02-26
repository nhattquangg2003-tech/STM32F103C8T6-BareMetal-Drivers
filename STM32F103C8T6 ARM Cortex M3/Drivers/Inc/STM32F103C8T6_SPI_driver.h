/*
 * STM32F103C8T6_SPI_driver.h
 *
 *  Created on: Feb 15, 2026
 *      Author: Owner
 */

#ifndef INC_STM32F103C8T6_SPI_DRIVER_H_
#define INC_STM32F103C8T6_SPI_DRIVER_H_
#include "STM32F103C8T6_SPI_driver.h"
#include <stdint.h>
#include <STM32F103C8T6.h>
/*
 * CONFIGURATION STRUCTURE FOR SPIx PERIPHERAL
 */
typedef struct
{
    uint8_t SPI_DeviceMode;     // Master hoặc Slave
    uint8_t SPI_BusConfig;      // Full-duplex, Half-duplex, Simplex RX-only
    uint8_t SPI_SclkSpeed;      // Baud rate (fPCLK/2, fPCLK/4, ...)
    uint8_t SPI_DFF;            // Data Frame Format: 8-bit hoặc 16-bit
    uint8_t SPI_CPOL;           // Clock Polarity
    uint8_t SPI_CPHA;           // Clock Phase
    uint8_t SPI_SSM;            // Software slave management (enable/disable)
} SPI_Config_t;


/*
 * HANDLE STRUCTURE FOR SPIx PERIPHERAL
 */
typedef struct
{
	SPI_RegDef_t  *pSPIx;
	SPI_Config_t SPIConfig;
	uint8_t *pTxBuffet;
	uint8_t *pRxBuffet;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxState;
	uint8_t RxState;
}SPI_Handle_t;
/************************************************************************************************************
 *                                              APIs SUPPORTED BY THIS DRIVER
 *                       FOR MORE INFOMATION ABOUT THE APIs CHECK THE FUNCTION DEFINITIONS
 *************************************************************************************************************/
// DeviceMode
#define SPI_DEVICE_MODE_MASTER    1
#define SPI_DEVICE_MODE_SLAVE     0

// BusConfig
#define SPI_BUS_CONFIG_FD         				1
#define SPI_BUS_CONFIG_HD         				2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY 			3
// SclkSpeed (Baud rate control bits BR[2:0])
#define SPI_SCLK_SPEED_DIV2       0
#define SPI_SCLK_SPEED_DIV4       1
#define SPI_SCLK_SPEED_DIV8       2
#define SPI_SCLK_SPEED_DIV16      3
#define SPI_SCLK_SPEED_DIV32      4
#define SPI_SCLK_SPEED_DIV64      5
#define SPI_SCLK_SPEED_DIV128     6
#define SPI_SCLK_SPEED_DIV256     7

// DFF
#define SPI_DFF_8BITS             0
#define SPI_DFF_16BITS            1

// CPOL
#define SPI_CPOL_LOW              0
#define SPI_CPOL_HIGH             1

// CPHA
#define SPI_CPHA_LOW              0
#define SPI_CPHA_HIGH             1

// SSM
#define SPI_SSM_EN                1
#define SPI_SSM_DI                0



#define SPI_TXE_FLAG ( 1 << SPI_SR_TXE )
#define SPI_RXNE_FLAG ( 1 << SPI_SR_RXNE )
#define SPI_BUSY_FLAG ( 1 << SPI_SR_BSY )

// define SPI Aplication states
#define SPI_READY 				0
#define SPI_BUSY_IN_RX 			1
#define SPI_BUSY_IN_TX 			2
/*
 *
 * possible SPI application events
 */
#define SPI_EVENT_TX_COMPLT 1
#define SPI_EVENT_RX_COMPLT 2
#define SPI_EVENT_OVR_ERR 	3
#define SPI_EVENT_CRC_ERR 	4



/*
 *
 *
 * PERIPHERAL CLOCK SET UP
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * INIT AND DEINIT
 */

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * DATA SEND AND RECEIVE
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);
/*
 * IRQ CONFIGURATION AND ISR HANDLING
 */
void SPI_IRQ_Interupt_Config(uint8_t IRQNumber,  uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);
/*
 * OTHER PERIPHERAL
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmisson(SPI_Handle_t *pHandle);
void SPI_CloseReception(SPI_Handle_t *pHandle);
/*
 * application callback
 */
void SPI_ApplicationCallback(SPI_Handle_t *pHandle,uint8_t AppEv);
#endif /* INC_STM32F103C8T6_SPI_DRIVER_H_ */
