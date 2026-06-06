// GPIOG->ODR^=GPIO_ODR_OD6; //toggle PG6 (green)
// turns off: GPIOG->BSRR = GPIO_BSRR_BS6;
// GPIOD->ODR^=GPIO_ODR_OD4; // orange
//GPIOD->BSRR = GPIO_BSRR_BS4; // turns off oarange

#include <spi1.hpp>
#include "stm32f469xx.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stddef.h>
#include <span>
#include "BareM_Def.h"
//#include "main.h"
#include "myConfig.h"
#include "spi1.hpp"
#include "timers.h"
#include "Flash_W25Q.h"
uint8_t buf[10] = {0};

int main (void)
{
	// Initialization functions
	activateFPU();
	SysClockConfig();
	SysTick_Init();

	GPIO_Config();
	InterruptGPIO_Config();

	//GPIOD->ODR^=GPIO_ODR_OD4; // turns off orange led
	NBdelay_ms(100);

	// Simply call initialization with your desired runtime prescaler speed division
	spi1.Init(BaudRatePrescaler::DIV_8); // DIV_8 => 11.25 MHz

//	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
//	DWT->CYCCNT = 0;
//	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;


	uint8_t bufferTx[] = {0x9F, 0x00, 0x00, 0x00, 0x00};

	NBdelay_ms(200);

	auto voue = std::span(bufferTx).subspan(0, 1);
    auto voue2 = std::span(buf).subspan(0, 3);

	auto vue = std::span(bufferTx).subspan(0, 4);
	auto vue2 = std::span(buf).subspan(0, 4);

	while(1) {

//		spi1.CS1_Low();
//		spi1.Transmit(voue, 10);
//		spi1.Receive(voue2, 10);
//		while(spi1.GetState() != SpiState::READY);
//		spi1.CS1_High();
//
//		NBdelay_ms(1);
//
//		spi1.CS1_Low();
//		spi1.TransmitReceive(vue, vue2, 10);
//		while(spi1.GetState() != SpiState::READY);
//		spi1.CS1_High();
//
//		NBdelay_ms(1);

		spi1.CS1_Low();
		spi1.Transmit_DMA(voue);
		spi1.Receive_DMA(voue2);
		while(spi1.GetState() != SpiState::READY);
		spi1.CS1_High();

//		NBdelay_ms(1);
//
//		spi1.CS1_Low();
//		spi1.TransmitReceive_DMA(vue, vue2);
//		while(spi1.GetState() != SpiState::READY);
//		spi1.CS1_High();

		NBdelay_ms(1000);

	}



//	BareM_StatusTypeDef Sp_s = SPI1_Init();
//	while(Sp_s != Bare_OK);
//
//
//	uint32_t ID = 0;
//	while(GetState_SPI1() != SPI_READY);
//	ID = W25Q_ReadID(); // no result bec the CS pin is not driven !!
//	uint8_t idW25Q[3];
//	idW25Q[0] = (ID >> 16) & 0xFF;
//	idW25Q[1] = (ID >> 8) & 0xFF;
//	idW25Q[2] = ID & 0xFF;
//	char hex_string[6]; // 2 characters for 'FE' + 1 for '\0'
//	// display the hex characters of JEDEC
//	for(int i=0; i<3; i++)	{
//		snprintf(&hex_string[i*2], sizeof(hex_string), "%02X", idW25Q[i]); 	// %02X prints the value as 2 uppercase hex digits
//	}




}
