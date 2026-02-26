/*
 * STM32F103C8T6_I2C_driver.h
 *
 *  Created on: Feb 16, 2026
 *      Author: Owner
 */

#ifndef INC_STM32F103C8T6_I2C_DRIVER_H_
#define INC_STM32F103C8T6_I2C_DRIVER_H_
#include <stdint.h>
#include <STM32F103C8T6.h>
#include <STM32F103C8T6_I2C_driver.h>
/*
 * CONFIGURATION STRUCTURE FOR I2Cx PERIPHERAL
 */
typedef struct
{
    uint32_t I2C_SCLSpeed;       // Clock speed (Standard 100kHz, Fast 400kHz)
    uint8_t  I2C_DeviceAddress;  // Own device address (7-bit or 10-bit)
    uint8_t  I2C_AckControl;     // ACK enable/disable
    uint8_t  I2C_FMDutyCycle;    // Duty cycle in fast mode (2 or 16/9)
} I2C_Config_t;
/*
 * HANDLE STRUCTURE FOR I2Cx PERIPHERAL
 */
typedef struct
{
    I2C_RegDef_t *pI2Cx;         // Base address of I2C peripheral
    I2C_Config_t I2C_Config;     // I2C configuration settings
    uint8_t *pTxBuffer;          // Pointer to Tx buffer
    uint8_t *pRxBuffer;          // Pointer to Rx buffer
    uint32_t TxLen;              // Length of Tx data
    uint32_t RxLen;              // Length of Rx data
    uint8_t TxRxState;           // Communication state (busy in Tx/Rx)
    uint8_t DevAddr;             // Slave/device address for communication
    uint32_t RxSize;             // Size of Rx data
    uint8_t Sr;                  // Repeated start condition flag
} I2C_Handle_t;
/*
 * I2C SCL Speed macros
 */
#define I2C_SCL_SPEED_SM     100000     // Standard mode (100 kHz)
#define I2C_SCL_SPEED_FM     400000     // Fast mode (400 kHz)

/*
 * I2C ACK Control macros
 */
#define I2C_ACK_ENABLE       1
#define I2C_ACK_DISABLE      0
//
//
//
#define I2C_DISABLE_SR   RESET
#define I2C_ENABLE_SR    SET
//
/*
 * I2C Fast Mode Duty Cycle macros
 */
#define I2C_FM_DUTY_2        0          // Duty cycle = 2
#define I2C_FM_DUTY_16_9     1          // Duty cycle = 16/9
/*
 * I2C related status flags definitions
 */
/* Flags using bit positions */
#define I2C_FLAG_SB        (1U << I2C_SR1_SB)
#define I2C_FLAG_ADDR      (1U << I2C_SR1_ADDR)
#define I2C_FLAG_BTF       (1U << I2C_SR1_BTF)
#define I2C_FLAG_ADD10     (1U << I2C_SR1_ADD10)
#define I2C_FLAG_STOPF     (1U << I2C_SR1_STOPF)
#define I2C_FLAG_RXNE      (1U << I2C_SR1_RXNE)
#define I2C_FLAG_TXE       (1U << I2C_SR1_TXE)
#define I2C_FLAG_BERR      (1U << I2C_SR1_BERR)
#define I2C_FLAG_ARLO      (1U << I2C_SR1_ARLO)
#define I2C_FLAG_AF        (1U << I2C_SR1_AF)
#define I2C_FLAG_OVR       (1U << I2C_SR1_OVR)
#define I2C_FLAG_PECERR    (1U << I2C_SR1_PECERR)
#define I2C_FLAG_TIMEOUT   (1U << I2C_SR1_TIMEOUT)
#define I2C_FLAG_SMBALERT  (1U << I2C_SR1_SMBALERT)
/*
 * I2C Application states
 */
#define I2C_READY	0
#define I2C_BUSY_IN_RX	1
#define I2C_BUSY_IN_TX	2
/*
 * I2C APPLICATION MACROS
 */
#define I2C_EV_TX_CMPLT				0
#define I2C_EV_STOP_CMPLT			1
#define I2C_EV_RX_CMPLT				2
#define I2C_ERROR_BERR  			3
#define I2C_ERROR_ARLO  			4
#define I2C_ERROR_AF    			5
#define I2C_ERROR_OVR   			6
#define I2C_ERROR_TIMEOUT 			7
#define I2C_EV_DATA_REQ				8
#define I2C_EV_DATA_RCV				9
/************************************************************************************************************
 *                                              APIs SUPPORTED BY THIS DRIVER
 *                       FOR MORE INFOMATION ABOUT THE APIs CHECK THE FUNCTION DEFINITIONS
 *************************************************************************************************************/
/*
 *
 * PERIPHERAL CLOCK SET UP
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * INIT AND DEINIT
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * DATA SEND AND RECEIVE
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr);

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

/*
 * IRQ CONFIGURATION AND ISR HANDLING
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
/*
 * OTHER PERIPHERAL CONTROL
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * APPLICATION CALLBACK
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);


#endif /* INC_STM32F103C8T6_I2C_DRIVER_H_ */
