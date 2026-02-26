/*
 * 006SPI_tx_testing.c
 *
 *  Created on: Feb 15, 2026
 *      Author: Owner
 */
// pb 15 SPI2_mosi afpp
// pb 14 SPI2_miso   input push pull
// pb 13 SPI2_sclk   afpp
// pb 12 SPI2_nss    afpp
#include "STM32F103C8T6.h"
#include <string.h>



void SPI2_GPIOInits(void)
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;   // generate sclk
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN; // sofware slave mangagement enabled for NSS pin

	SPI_Init(&SPI2handle);
}
int main(void)
{
      // config chân pb 15
	GPIO_Handle_t SPI2_mosi;
	SPI2_mosi.CNF = GPIO_CNF_AF_PP;
	SPI2_mosi.pGPIOx = GPIOB;
	SPI2_mosi.Mode = GPIO_MODE_OUTPUT_50MHZ;
	SPI2_mosi.PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPI2_mosi);
	// config chân pb 14
	GPIO_Handle_t SPI2_miso;
	SPI2_miso.CNF = GPIO_CNF_INPUT_PUPD;
	SPI2_miso.pGPIOx = GPIOB;
	SPI2_miso.Mode = GPIO_MODE_INPUT;
	SPI2_miso.PinNumber = GPIO_PIN_14;
		GPIO_Init(&SPI2_miso);
		// config chân pb 13
		GPIO_Handle_t SPI2_sclk;
		SPI2_sclk.CNF = GPIO_CNF_AF_PP;
		SPI2_sclk.pGPIOx = GPIOB;
		SPI2_sclk.Mode = GPIO_MODE_OUTPUT_50MHZ;
		SPI2_sclk.PinNumber = GPIO_PIN_13;
			GPIO_Init(&SPI2_sclk);
			// config chân pb 12
			GPIO_Handle_t SPI2_nss;
			SPI2_nss.CNF = GPIO_CNF_AF_PP;
			SPI2_nss.pGPIOx = GPIOB;
			SPI2_nss.Mode = GPIO_MODE_OUTPUT_50MHZ;
			SPI2_nss.PinNumber = GPIO_PIN_12;
				GPIO_Init(&SPI2_nss);

	SPI2_GPIOInits();

    // this make NSS signal internally high and avoid MODF error
	SPI_SSIConfig( SPI2,ENABLE);
	// enable the spi2 peripheral
	SPI_PeripheralControl(SPI2,ENABLE);

	char user_data[] = "Hello World";
    // to send data
	SPI_SendData(SPI2, (uint8_t*)user_data,strlen(user_data));
	SPI_PeripheralControl(SPI2,DISABLE);


   while(1);






	return 0;
}
