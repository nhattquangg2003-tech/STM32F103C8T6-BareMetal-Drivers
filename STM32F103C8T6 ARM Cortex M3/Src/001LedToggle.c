/*
 * 001LedToggle.c
 *
 *  Created on: Feb 11, 2026
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
	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOB;
	GpioLed.PinNumber = GPIO_PIN_2;
	GpioLed.Mode = GPIO_MODE_OUTPUT_10MHZ;
	GpioLed.CNF = GPIO_CNF_OUTPUT_PP;

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&GpioLed);

	while(1)
	{
	    GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_2);
	    delay(5000000);
	}




	/*
	 * Đối với xài mode open drain thì cần kết nối 1 điện trở ngoài, cần 1 nguồn ngoài và 1 trở ngoài để pull up vì mode open drain chỉ
	 * có thể kéo xuống gnd nếu write 0 còn khi write 1 thì nó không tự pull up giống như push pull phải nhờ vào nguồn ngoài và trở để kéo
	 * vậy ta phải nối chân vcc với trở sau đó nối lại với chân pb2 để có thể toggle được chân pb2 cũng như là làm led nhấp nháy
	 */

	return 0;
}

