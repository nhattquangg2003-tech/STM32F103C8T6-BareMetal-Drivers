/*
 * STM32F103C8T6_UART_driver.h
 *
 *  Created on: Feb 25, 2026
 *      Author: Owner
 */

#ifndef INC_STM32F103C8T6_UART_DRIVER_H_
#define INC_STM32F103C8T6_UART_DRIVER_H_
#include "STM32F103C8T6_UART_driver.h"
#include <stdint.h>
#include <STM32F103C8T6.h>



/*
 * Configuration structure for USART peripheral
 */
typedef struct
{
    uint8_t USART_Mode;              // Tx/Rx mode
    uint32_t USART_Baud;             // Baud rate
    uint8_t USART_NoOfStopBits;      // Stop bits
    uint8_t USART_WordLength;        // Word length (8/9 bits)
    uint8_t USART_ParityControl;     // Parity control
    uint8_t USART_HWFlowControl;     // Hardware flow control
} USART_Config_t;

/*
 * Handle structure for USART peripheral
 */
typedef struct
{
    USART_RegDef_t *pUSARTx;         // Base address of USART peripheral
    USART_Config_t USART_Config;     // USART configuration settings

    uint8_t *pTxBuffer;              // Tx buffer pointer
    uint8_t *pRxBuffer;              // Rx buffer pointer
    uint32_t TxLen;                  // Tx length
    uint32_t RxLen;                  // Rx length
    uint8_t TxBusyState;             // Tx state
    uint8_t RxBusyState;             // Rx state
} USART_Handle_t;




/*
 * USART application states
 */
#define USART_READY          0
#define USART_BUSY_IN_RX     1
#define USART_BUSY_IN_TX     2

/*
 * USART word length options
 */
#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1

/*
 * USART parity control options
 */
#define USART_PARITY_DISABLE   0
#define USART_PARITY_EN_EVEN   1
#define USART_PARITY_EN_ODD    2

/*
 * USART mode options
 */
#define USART_MODE_ONLY_TX     0
#define USART_MODE_ONLY_RX     1
#define USART_MODE_TXRX        2

/*
 * USART hardware flow control options
 */
#define USART_HW_FLOW_CTRL_NONE     0
#define USART_HW_FLOW_CTRL_RTS      1
#define USART_HW_FLOW_CTRL_CTS      2
#define USART_HW_FLOW_CTRL_CTS_RTS  3

/*
 * Flag macros for SR register
 */
#define USART_FLAG_TXE   (1 << USART_SR_TXE)   // Transmit data register empty
#define USART_FLAG_RXNE  (1 << USART_SR_RXNE)  // Read data register not empty
#define USART_FLAG_TC    (1 << USART_SR_TC)    // Transmission complete
/*********************** USART Application Events ********************/
#define USART_EVENT_TX_CMPLT     0   // Transmission Complete
#define USART_EVENT_RX_CMPLT     1   // Reception Complete
#define USART_EVENT_IDLE         2   // IDLE line detected
#define USART_EVENT_CTS          3   // CTS event
#define USART_EVENT_ORE          4   // Overrun Error

/*********************** USART Error Events **************************/
#define USART_ERREVENT_FE        5   // Framing Error
#define USART_ERREVENT_NE        6   // Noise Error
#define USART_ERREVENT_ORE       7   // Overrun Error (error context)



/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);


/*
 * Data Send and Receive
 */
void USART_SendData(USART_Handle_t *pUSARTHandle ,uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle , uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv);
/*
 * set baudrate
 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

#endif /* INC_STM32F103C8T6_UART_DRIVER_H_ */
