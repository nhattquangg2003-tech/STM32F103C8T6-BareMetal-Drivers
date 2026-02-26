/*
 * 002ButtonToggle.c
 *
 *  Created on: Feb 12, 2026
 *      Author: Owner
 */
#include "STM32F103C8T6.h"
#include <stdint.h>

void delay(uint32_t time)
{
	for (volatile uint32_t i = 0; i < time; i++);
}
int main(void)
{
	// config chân led pb2
	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOB;
	GpioLed.PinNumber = GPIO_PIN_2;
	GpioLed.Mode = GPIO_MODE_OUTPUT_10MHZ;
	GpioLed.CNF = GPIO_CNF_OUTPUT_PP;

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&GpioLed);

	// config button trên board là pa0
	GPIO_Handle_t GpioBtn;
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.PinNumber = GPIO_PIN_0;
	GpioBtn.Mode = GPIO_MODE_INPUT;
	GpioBtn.CNF = GPIO_CNF_INPUT_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioBtn);

	while(1)
	{
		/* if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0) == ENABLE) {
			delay(500000/2);
	    	GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_2);
	}*/
	}
	void EXTI0_IRQHandler(void) {
		if(EXTI->PR & (1 << 0))
	{
		EXTI->PR |= (1 << 0); // clear pending bit
	 //Toggle LED PB2
		GPIOB->ODR ^= (1 << 2); }
	}
	return 0;
}


