// ******* 		SPI - Polling & DMA-based driver 	 ***** //
// **** with MISO = PB4; MOSI = PB5; SCK = PA5; NSS = PA15 **** //

#include "SPI1.h"
#include "stm32f4xx.h"
#include "myConfig.h"


volatile uint8_t tx_finished, rx_finished; // flags modified in interrupts

SPI_Status State_SPI1 = SPI_READY; // global variable of SPI state
uint8_t dummy_tx = 0x00; // Global dummy byte to send during reception
BareM_StatusTypeDef BareM_Status_SPI1; // variable for return value of functions

BareM_StatusTypeDef SPI1_Init(void)
{
	    // 1. Enable Clocks for GPIOA, GPIOB and SPI1
	    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
	    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	    // 2. Configure GPIO Pins (PA5: SCK, PB4: MISO, PB5: MOSI) to AF5
	    // PA5 (SCK)
	    GPIOA->MODER   |= (2 << GPIO_MODER_MODER5_Pos);
	    GPIOA->AFR[0]  |= (5 << GPIO_AFRL_AFSEL5_Pos);
	    GPIOA->OSPEEDR &= ~(3 << GPIO_OSPEEDR_OSPEED5_Pos); // Clear speed bits
	    GPIOA->OSPEEDR |=  (2 << GPIO_OSPEEDR_OSPEED5_Pos); // Set to High Speed (10)
	    // PB4 (MISO) and PB5 (MOSI)
	    GPIOB->MODER   |= (2 << GPIO_MODER_MODER4_Pos) | (2 << GPIO_MODER_MODER5_Pos);
	    GPIOB->AFR[0]  |= (5 << GPIO_AFRL_AFSEL4_Pos) | (5 << GPIO_AFRL_AFSEL5_Pos);
	    GPIOB->OSPEEDR &= ~(3 << GPIO_OSPEEDR_OSPEED4_Pos); // Clear speed bits
	    GPIOB->OSPEEDR |=  (2 << GPIO_OSPEEDR_OSPEED4_Pos); // Set to High Speed (10)
	    GPIOB->OSPEEDR &= ~(3 << GPIO_OSPEEDR_OSPEED5_Pos); // Clear speed bits
	    GPIOB->OSPEEDR |=  (2 << GPIO_OSPEEDR_OSPEED5_Pos); // Set to High Speed (10)

	    // 3. Configure PA15 and PC13 as General Purpose Output for Chip Select
//	    GPIOA->MODER &=  ~(3 << GPIO_MODER_MODER15_Pos); // PA15
//	    GPIOA->MODER |= (1 << GPIO_MODER_MODER15_Pos);
	    //	    GPIOA->OSPEEDR &= ~(3 << GPIO_OSPEEDR_OSPEED15_Pos); // Clear existing bits
	    //	    GPIOA->OSPEEDR |=  (2 << GPIO_OSPEEDR_OSPEED15_Pos); // Set to High Speed
	    //	    GPIOA->BSRR |= (1<<15); // Set PA15 high as initial state

	    GPIOA->MODER  &= ~(3 << GPIO_MODER_MODER15_Pos);
	    GPIOA->MODER  |=  (2 << GPIO_MODER_MODER15_Pos); // Alternate Function Mode
	    GPIOA->AFR[1] &= ~(0xF << ((15 - 8) * 4));
	    GPIOA->AFR[1] |=  (5   << ((15 - 8) * 4));        // AF5 = SPI1

	    GPIOC->MODER &=  ~(3 << GPIO_MODER_MODER13_Pos); // PC13
	    GPIOC->MODER |= (1 << GPIO_MODER_MODER13_Pos);
	    GPIOC->OSPEEDR &= ~(3 << GPIO_OSPEEDR_OSPEED13_Pos); // Clear existing bits
	    GPIOC->OSPEEDR |=  (2 << GPIO_OSPEEDR_OSPEED13_Pos); // Set to High Speed
	    GPIOC->BSRR |= (1<<13); // Set PC13 high as initial state


	    // 4. SPI Configuration  // Master mode, BaudRate = PCLK2(90Mhz)/8 (11.25Mhz), CPOL=0, CPHA=0, 8-bit data, MSB First / SW Slave Management (SSM=1, SSI=1)
	    // Work well up to 25Mhz with ST7796, for tests by precaution set to 11.25Mhz
		SPI1->CR1 |= SPI_CR1_MSTR | SPI_CR1_BR_2 |  SPI_CR1_BR_1 | /* SPI_CR1_BR_0 |*/ SPI_CR1_SSM | SPI_CR1_SSI; //
	    SPI1->CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN; // TXDMAEN: Tx buffer DMA enable, RXDMAEN Rx buffer too (perdu du tps!)

	    // CRITICAL LINE: Enable TI Frame Format (FRF = 1)
	    SPI1->CR2 |= SPI_CR2_FRF;

	    SPI1->CR1 |= SPI_CR1_SPE; // Enable SPI

	    // Configuring DMA
		RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN; // Enable the DMA2 clock
		// Configuring SPI TX Stream
		DMA2_Stream3->CR &= ~DMA_SxCR_EN; // Disable the stream
		while((DMA2_Stream3->CR) & DMA_SxCR_EN); // And make sure it is disabled
		// Ch. 3| Mem. pointer Incremented after each data transfer | Direction Mem to periph. | Transfer Cmplt Int. | Half transfer Int. | Transfer error Int. | Direct Mode error Int.
		DMA2_Stream3->CR = (3<<25) | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE	| DMA_SxCR_HTIE | DMA_SxCR_TEIE |DMA_SxCR_DMEIE;
		//DMA2_Stream3->FCR=0;
		DMA2_Stream3->FCR |= DMA_SxFCR_DMDIS; // Direct mode not enabled
		NVIC_EnableIRQ(DMA2_Stream3_IRQn);

		// Configuring SPI_RX Stream
		DMA2_Stream2->CR &= ~DMA_SxCR_EN;
		while((DMA2_Stream2->CR) & DMA_SxCR_EN);
		// Ch. 3 | Memory and Peripheral size to 8-bit | Memory increment | Direction: Peripheral to memory
		DMA2_Stream2->CR = (3<<25) | DMA_SxCR_MINC | DMA_SxCR_TCIE | DMA_SxCR_HTIE |DMA_SxCR_TEIE | DMA_SxCR_DMEIE; // interrupt T useful ??
		DMA2_Stream2->CR &= ~(3<<6); // Direction 00: Peripheral-to-memory
		DMA2_Stream2->FCR |= DMA_SxFCR_DMDIS; // Direct mode not enabled
		DMA2_Stream2->FCR |= (DMA_SxFCR_FTH_0 | DMA_SxFCR_FTH_1);
		NVIC_SetPriority(DMA2_Stream2_IRQn, 2);
		NVIC_EnableIRQ(DMA2_Stream2_IRQn);

		return BareM_Status_SPI1 = Bare_OK;
}

// PA15 for first slave device CS
void csLOW1(void) {	GPIOA->BSRR = GPIO_BSRR_BR15; } // Pull down to chip enable
void csHIGH1(void) { GPIOA->BSRR = GPIO_BSRR_BS15; } // Pull high to disable
// PC13 for 2nd slave device CS
void csLOW2(void) {	GPIOC->BSRR = GPIO_BSRR_BR13; } // Pull down to chip enable
void csHIGH2(void) { GPIOC->BSRR = GPIO_BSRR_BS13; } // Pull high to disable

SPI_Status GetState_SPI1() { return State_SPI1; } // better: GetState_SPI(SPI_HandleTypeDef *hspi);
/*
 * Asynchronous Wait: Example of while loop with timeout
	uint32_t timeout_counter = GetSysTick();
	while(GetState_SPI1() != SPI_READY) {if (GetSysTick() - timeout_counter > 2000) return;}
*/

/***************************** SPI FUNCTIONS DMA MODE *****************************/

BareM_StatusTypeDef SPI1_Transmit_DMA(uint8_t *pData, uint32_t len) // (uint8_t *data, uint32_t size, uint32_t timeout)
{
	if(len == 0) return BareM_Status_SPI1 = Bare_ERROR;
	while(State_SPI1 == SPI_BUSY_TX); // in case
	State_SPI1 = SPI_BUSY_TX;
	DMA2_Stream3->CR &= ~DMA_SxCR_EN; 		// (it is forbidden to write those registers when the EN bit is read as 1)
	while(DMA2_Stream3->CR & DMA_SxCR_EN);		// Stream enable/flag stream ready when read low
	//uint32_t timeout_counter = GetSysTick();
	DMA2->LIFCR |= DMA_LIFCR_CTCIF3|DMA_LIFCR_CHTIF3|DMA_LIFCR_CTEIF3|DMA_LIFCR_CDMEIF3|DMA_LIFCR_CFEIF3; // clear all pending interrupts
	DMA2_Stream3->PAR = (uint32_t)&SPI1->DR; // set the peripheral address to SPI->DR
	DMA2_Stream3->M0AR = (uint32_t)pData; // Set memory address to be the byte(s) to be sent
	DMA2_Stream3->NDTR = len; //
	DMA2_Stream3->CR |= DMA_SxCR_EN; // Enable the DMA stream

	while(State_SPI1 != SPI_READY);/*;{
		if (GetSysTick() - timeout_counter > 1) {

			return BareM_Status_SPI1 = Bare_OK;
		}
	}
*/
	return BareM_Status_SPI1 = Bare_OK;
}

BareM_StatusTypeDef SPI1_Receive_DMA(uint8_t *pData, uint32_t len)
{
	if(len == 0) return BareM_Status_SPI1 = Bare_ERROR;
	while(State_SPI1 == SPI_BUSY_RX); // in case
	State_SPI1 = SPI_BUSY_RX;
	DMA2_Stream2->CR &= ~DMA_SxCR_EN; // Disable to configure
	DMA2_Stream3->CR &= ~DMA_SxCR_EN;
	while((DMA2_Stream3->CR & DMA_SxCR_EN) || (DMA2_Stream2->CR & DMA_SxCR_EN)); // Wait for disable

	DMA2->LIFCR |= DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3; // clear all pending interrupts Tx
	DMA2->LIFCR |= DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTEIF2 | DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CFEIF2; // Rx

	// Configure Rx stream
	DMA2_Stream2->PAR = (uint32_t)&(SPI1->DR);
	DMA2_Stream2->M0AR = (uint32_t)pData;
	DMA2_Stream2->NDTR = len;
	//Configure Tx stream (dummy datas)
	DMA2_Stream3->PAR = (uint32_t)&(SPI1->DR); // set the peripheral address to SPI->DR
	DMA2_Stream3->M0AR = (uint32_t)&dummy_tx; // Set memory address to be the byte(s) to be sent
	DMA2_Stream3->NDTR = len;

	// Clear any stale data - avoid that the first dummy byte (0xFF) fills the Rx buffer and increment detrimentally the counter NDTR // 6h pour trouver !
	volatile uint32_t tmpreg = SPI1->DR; (void)tmpreg; // Must be volatile, and positioned after Configure Tx Stream ! Why this trick has solved the issue ?

	DMA2_Stream2->CR |= DMA_SxCR_EN;
	DMA2_Stream3->CR |= DMA_SxCR_EN; // Enable the TX DMA stream after enabling the Rx

	while(State_SPI1 != SPI_READY);
	return BareM_Status_SPI1 = Bare_OK;
}


void DMA2_Stream2_IRQHandler(void) 		// DMA interrupt handling Rx
{
	if(DMA2->LISR & (DMA_LISR_TCIF2))
	{
		DMA2->LIFCR |= DMA_LIFCR_CTCIF2; // Clear Transfer Complete Interrupt flag of Stream 2
		DMA2_Stream2->CR&=~DMA_SxCR_EN;
		//while(SPI1->SR & SPI_SR_BSY); // Double-Confirms the end of transfer - necessary ?
		State_SPI1 = SPI_READY;
	}

	if(DMA2->LISR&(DMA_LISR_HTIF2))
	{

		DMA2->LIFCR |=DMA_LIFCR_CHTIF2;
	}

	if(DMA2->LISR&(DMA_LISR_TEIF2))
	{

		DMA2->LIFCR|=(DMA_LIFCR_CTEIF2);
	}

	if(DMA2->LISR&(DMA_LISR_DMEIF2))
	{

		DMA2->LIFCR|=(DMA_LIFCR_CDMEIF2);
	}

	if(DMA2->LISR&(DMA_LISR_FEIF2))
	{

		DMA2->LIFCR|=(DMA_LIFCR_CFEIF2);
	}

	NVIC_ClearPendingIRQ(DMA2_Stream2_IRQn);
}


void DMA2_Stream3_IRQHandler(void) 		// DMA interrupt handling Tx
{
	if(DMA2->LISR&(DMA_LISR_TCIF3)) // Stream 3 transfer complete interrupt flag
	{
		DMA2->LIFCR |= DMA_LIFCR_CTCIF3; // Clear Transfer Complete Interrupt flag of Stream 3
		DMA2_Stream3->CR &= ~DMA_SxCR_EN;
		while(SPI1->SR & SPI_SR_BSY);
		State_SPI1 = SPI_READY;
		/*
		 * In DMA, essential to wait BSY=0 before raising the SPI_READY flag, since TCIF seems fire too early (in the middle of the Tx byte, not at the last),
		 the subsequent function (ex: Receive) will end also too early, causing missing bits while the clock is still running. */
	}

	if(DMA2->LISR&(DMA_LISR_HTIF3)) // Stream 3 transfer half-complete interrupt flag
	{

		DMA2->LIFCR |= DMA_LIFCR_CHTIF3; // clear flag
	}

	if(DMA2->LISR&(DMA_LISR_TEIF3)) // Stream 3 transfer error interrupt flag
	{
		DMA2->LIFCR|=(DMA_LIFCR_CTEIF3);
	}

	if(DMA2->LISR&(DMA_LISR_DMEIF3)) // Stream 3 direct mode error interrupt flag
	{
		DMA2->LIFCR|=(DMA_LIFCR_CDMEIF3);
	}

	if(DMA2->LISR&(DMA_LISR_FEIF3)) // Stream 3 FIFO error interrupt flag
	{
		DMA2->LIFCR|=(DMA_LIFCR_CFEIF3);
	}

	NVIC_ClearPendingIRQ(DMA2_Stream3_IRQn);
}



/****************************** SPI FUNCTIONS POLLING MODE ********************************/

BareM_StatusTypeDef SPI1_Receive(uint8_t *data, uint32_t size, uint32_t timeout)
{
	if(size == 0) return BareM_Status_SPI1 = Bare_ERROR;
	while(State_SPI1 == SPI_BUSY_RX);

	State_SPI1 = SPI_BUSY_RX;

	uint32_t timeout_ct = GetSysTick() + timeout;

	while(size)
	{
		while(!(SPI1->SR & (SPI_SR_TXE)))
		{
			if (GetSysTick() > timeout_ct ) {
				State_SPI1 = SPI_TIMEOUT;
				return BareM_Status_SPI1 = Bare_TIMEOUT; // Wait until TXE is set or timeout
			}
		}

		SPI1->DR = 0; // Send dummy data

		while(!(SPI1->SR & (SPI_SR_RXNE)))
		{
			if (GetSysTick() > timeout_ct ) {
				State_SPI1 = SPI_TIMEOUT;
				return BareM_Status_SPI1 = Bare_TIMEOUT; // Wait for RXNE flag to be set or timeout
			}
		}

		*data++ = (SPI1->DR); // Read data from data register
		size--;
	}
	State_SPI1 = SPI_READY;

	return BareM_Status_SPI1 = Bare_OK;
}

BareM_StatusTypeDef SPI1_Transmit(uint8_t *data, uint32_t size, uint32_t timeout)
{
	if(size == 0) return BareM_Status_SPI1 = Bare_ERROR;
	while(State_SPI1 == SPI_BUSY_TX);

	State_SPI1 = SPI_BUSY_TX;
	uint32_t timeout_ct = GetSysTick() + timeout;
	uint32_t i=1;

	SPI1->DR = data[0]; // Write the data to the data register
	csLOW1();

	while(i < size)
	{
		while(!(SPI1->SR & (SPI_SR_TXE))) { if (GetSysTick() > timeout_ct ) return SPI_TIMEOUT; } // Wait until TXE is set or timeout

		SPI1->DR = data[i]; // Write the data to the data register
		i++;
	}

	while(!(SPI1->SR & (SPI_SR_TXE))); // Wait until TXE is set
	while((SPI1->SR & (SPI_SR_BSY))) { if (GetSysTick() > timeout_ct ) return SPI_TIMEOUT; } // Wait for BUSY flag to reset or timeout
	(void)SPI1->DR; (void)SPI1->SR; // Clear OVR (Overrun Error) flag
	State_SPI1 = SPI_READY;

	return BareM_Status_SPI1 = Bare_OK;
}
