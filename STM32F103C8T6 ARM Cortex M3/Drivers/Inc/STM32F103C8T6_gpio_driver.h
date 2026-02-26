/*
 * STM32F103C8T6_gpio_driver.h
 *
 *  Created on: Feb 10, 2026
 *      Author: Owner
 */

#ifndef INC_STM32F103C8T6_GPIO_DRIVER_H_
#define INC_STM32F103C8T6_GPIO_DRIVER_H_

#include "STM32F103C8T6.h"
#include <stdint.h>


/*
 * THIS IS A HANDLE STRUCTURE FOR A GPIO PIN
 */
/* typedef struct
{
    uint8_t GPIO_PinNumber;        // the number of pin (0–15)
    uint8_t GPIO_PinMode;          // function: input, output, alternate function, analog
    uint8_t GPIO_PinSpeed;         // speed output: 2 MHz, 10 MHz, 50 MHz
    uint8_t GPIO_PinPuPdControl;   // type pull-up/pull-down or floating
    uint8_t GPIO_PinOPType;        // Output type: push-pull or open-drain
    uint8_t GPIO_PinAltFunMode;    // Alternate function mode (if use AF)
} GPIO_PinConfig_t;
*/

  //THIS IS A HANDLE STRUCTURE FOR A GPIO PIN


typedef struct
{
	GPIO_RegDef_t *pGPIOx; /* < this holds the base address of the GPIO port to which the pin belongs > */
	//GPIO_PinConfig_t GPIO_PinConfig; /* < this holds GPIO pin configuration settings > */
	   uint8_t PinNumber;     // 0..15
	    uint8_t Mode;          // MODE bits
	    uint8_t CNF;           // CNF bits
}GPIO_Handle_t;

/************************************************************************************************************
 *                                              APIs SUPPORTED BY THIS DRIVER
 *                       FOR MORE INFOMATION ABOUT THE APIs CHECK THE FUNCTION DEFINITIONS
 *************************************************************************************************************/

/*
 * PERIPHERAL CLOCK SET UP
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * INIT AND DEINIT
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * DATA READ AND WRITE
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ CONFIGURATION AND ISR HANDLING
 */
void GPIO_IRQ_Interupt_Config(uint8_t IRQNumber,  uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);






#endif /* INC_STM32F103C8T6_GPIO_DRIVER_H_ */
