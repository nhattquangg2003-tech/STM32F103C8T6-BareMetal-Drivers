/*
 * 002ButtonToggle.c
 *
 *  Created on: Feb 12, 2026
 *      Author: Owner
 */
#include "STM32F103C8T6.h"
#include <stdint.h>
#include <string.h>



volatile uint8_t led_state = 0;
void delay(uint32_t time)
{
	for (volatile uint32_t i = 0; i < time; i++);
}
int main(void)
{

	GPIO_Handle_t GpioLed,GpioBtn;

	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GpioBtn,0,sizeof(GpioBtn));


	// config chân led pa12
	GpioLed.pGPIOx = GPIOA;
	GpioLed.PinNumber = GPIO_PIN_12;
	GpioLed.Mode = GPIO_MODE_OUTPUT_10MHZ;
	GpioLed.CNF = GPIO_CNF_OUTPUT_PP;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);

	// config button chan pa5
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.PinNumber = GPIO_PIN_0;
	GpioBtn.Mode = GPIO_MODE_IT_FT;
	GpioBtn.CNF = GPIO_CNF_INPUT_PUPD;

	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_Init(&GpioBtn);

	// IRQ CONFIGURATIONS
	 GPIO_IRQ_Interupt_Config(EXTI0_IRQn, ENABLE);
	 GPIO_IRQPriorityConfig(EXTI0_IRQn, NVIC_IRQ_PRI15);

	while(1)
	{
		if(led_state == 1){
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_12);
			delay(500000/2);
		} else {
			GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_12, 0);
		}
	}

	return 0;
}
void EXTI0_IRQHandler(void)
{
	delay(500000/2);
	GPIO_IRQHandling(GPIO_PIN_0);
	led_state ^= 1;
}

