/*
*	***  SPI bus - Polling & DMA modes bare-metal driver	---	  Nicolas PRATA - 06/2026  ***
*
*	Notes and instructions:
*
*   About the combined TransmitReceive_DMA() function and the rule of symmetrical DMA drivers:
	   - txData.size() must equal rxData.size().
	   - That size must represent the TOTAL number of clock pulses needed for the entire conversation.
	   - The real incoming data will always be offset in your receive buffer by the length of your command header !
	   - Allocating a massive array to send only a few command bytes (ex: reading a Flash sector) is a massive waste of precious RAM.
	   -> Use TransmitReceive_DMA only for small control messages (like reading a 3-byte Unique ID) where creating a tiny matching array is trivial
	  	  and don't forget to subtract the first garbage bytes (ex: std::span<uint8_t> clean_payload = std::span(raw_buffer).subspan(3);)
*	- Use Transmit_DMA followed by Receive_DMA for heavy payload operations (like 4KB sector reads) to keep the RAM completely clean.
*	- Polling is slightly faster (1µs less latency than equivalent DMA functions at 11.25Mhz) - best for small transfers if blocking is not a problem
*	- Using Polling TransmitReceive() is actually slightly slower than combining the separate Transmit() + Receive() functions !
*	   But unlike DMA you don't need to have equal sizes of buffers for Rx and Tx.
*	- DMA functions have been kept non-blocking, so check while(spi1.GetState() != SpiState::READY); before pulling CS low or high again
*	- Weak Callback functions available for Tx/Rx/TxRx complete and Errors
*	- GPIO used (STM32F446): MISO = ; MOSI = ; SCK = ; NSS =
*/

#include <spi.hpp>

// Constructor definition
SpiDriver::SpiDriver(const SpiHardwareConfig& hardwareConfig) : config(hardwareConfig) {}

BareM_Status SpiDriver::Init(BaudRatePrescaler prescaler) {
    if (config.lowLevelInit != nullptr) {
        config.lowLevelInit();
    }

    config.spi->CR1 &= ~SPI_CR1_SPE;
    config.spi->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | static_cast<uint32_t>(prescaler);
    config.spi->CR2 = SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;
    config.spi->CR1 |= SPI_CR1_SPE;

    ConfigureDma();
    m_state = SpiState::READY;
    return BareM_Status::OK;
}

void SpiDriver::ConfigureDma() {
    if (m_isDmaInitialized) return;

    if (config.dmaBase == DMA1)
           RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
       else if (config.dmaBase == DMA2)
           RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    // Configure Rx Stream
    config.rxStream->CR &= ~DMA_SxCR_EN;
    while (config.rxStream->CR & DMA_SxCR_EN);
    *config.rxFcrReg = config.rxClearMask; // Direct Clear

    config.rxStream->CR = (config.dmaChannel << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC
                        | DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;
    config.rxStream->CR &= ~(3 << DMA_SxCR_DIR_Pos);
    config.rxStream->FCR |= DMA_SxFCR_DMDIS;
    config.rxStream->FCR |= (DMA_SxFCR_FTH_0 | DMA_SxFCR_FTH_1);
    config.rxStream->PAR = reinterpret_cast<uint32_t>(&config.spi->DR); // Set the static peripheral destinations

    NVIC_SetPriority(config.rxDmaIrq, 2);
    NVIC_EnableIRQ(config.rxDmaIrq);

    // Configure Tx Stream
    config.txStream->CR &= ~DMA_SxCR_EN;
    while (config.txStream->CR & DMA_SxCR_EN);
    *config.txFcrReg = config.txClearMask; // Direct Clear

    config.txStream->CR = (config.dmaChannel << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_DIR_0
                        | DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;
    config.txStream->FCR |= DMA_SxFCR_DMDIS;
    config.txStream->PAR = reinterpret_cast<uint32_t>(&config.spi->DR);

    NVIC_SetPriority(config.txDmaIrq, 2);
    NVIC_EnableIRQ(config.txDmaIrq);

    m_isDmaInitialized = true;
}

/*=================================================================
/=============    SPI FUNCTIONS POLLING MODE   ====================
/================================================================== */

BareM_Status SpiDriver::Receive_MainBody(std::span<uint8_t> rxData, uint32_t timeoutMs) {

    uint32_t startTick = GetSysTick();

    uint32_t rxCounter = rxData.size();
    // CRITICAL: We already kicked off the first byte in the inline header!
    // So the transmit pipeline counter starts at size - 1.
    uint32_t txCounter = rxData.size() - 1;
    uint8_t* destPtr = rxData.data();

    // HIGH-SPEED PIPELINED LOOP
    while (rxCounter > 0) {
        // 1. Keep the TX pipeline full to maintain continuous SCK cycles
        if (txCounter > 0 && (config.spi->SR & SPI_SR_TXE)) {
            config.spi->DR = 0x00;
            txCounter--;
        }
        // 2. Extract incoming data bytes as they land
        if (config.spi->SR & SPI_SR_RXNE) {
            *destPtr++ = static_cast<uint8_t>(config.spi->DR);
            rxCounter--;
        }
        // 3. Safety Gate
        if ((GetSysTick() - startTick) > timeoutMs) {
            m_state = SpiState::READY;
            return BareM_Status::TIMEOUT;
        }
    }

    // Allow physical shift registers to completely settle
    while (config.spi->SR & SPI_SR_BSY);

    m_state = SpiState::READY;
    return BareM_Status::OK;
}


BareM_Status SpiDriver::Transmit_MainBody(std::span<const uint8_t> txData, uint32_t timeoutMs) {
	// Regular, non-inlined function that does the heavy lifting

	uint32_t startTick = GetSysTick();

    // Check if the peripheral is ready
    while (m_state != SpiState::READY) {
    	if ((GetSysTick() - startTick) > timeoutMs) {
            m_state = SpiState::READY;
            return BareM_Status::TIMEOUT;
        }
    }
    m_state = SpiState::BUSY_TX;

    const uint8_t* txPtr = txData.data() + 1; // added +1 since the wrapper already sends txData[0]
    uint32_t totalBytes  = txData.size() - 1;
    uint32_t bytesSent   = 0;
    uint32_t bytesRead   = 0;

    // Hyper-fast pipelined loop (No nested blocking loops!)
    while (bytesRead < totalBytes) {

        // 1. Keep the Transmit Mailbox full whenever TXE is ready
        if (bytesSent < totalBytes && (config.spi->SR & SPI_SR_TXE)) {
            config.spi->DR = *txPtr++;
            bytesSent++;
        }
        // 2. Clear out RX bytes instantly as they arrive, without blocking
        if (config.spi->SR & SPI_SR_RXNE) {
            [[maybe_unused]] volatile uint8_t dummySink = static_cast<uint8_t>(config.spi->DR);
            bytesRead++;
        }
        // 3. Single safety timeout gate for the entire loop execution
        if ((GetSysTick() - startTick) > timeoutMs) {
            m_state = SpiState::READY;
            return BareM_Status::TIMEOUT;
        }
    }

    // Wait to allow Chip Select (CS) to be pulled high again
    while (config.spi->SR & SPI_SR_BSY) {
    	if ((GetSysTick() - startTick) > timeoutMs) {
            m_state = SpiState::READY;
            return BareM_Status::TIMEOUT;
        }
    }

    m_state = SpiState::READY;
    return BareM_Status::OK;
}



BareM_Status SpiDriver::TransmitReceive_MainBody(std::span<const uint8_t> txData, std::span<uint8_t> rxData, uint32_t timeoutMs) {
	// Regular, non-inlined function that does the heavy lifting

	uint32_t totalSize = 0;
    uint32_t skipRxBytes = 0;

    if (txData.size() > 0 && rxData.size() > 0 && txData.size() != rxData.size()) {
        totalSize = txData.size() + rxData.size();
        skipRxBytes = txData.size();
    } else {
        totalSize = (txData.size() > rxData.size()) ? txData.size() : rxData.size();
    }

    if (totalSize == 0) return BareM_Status::ERROR;

    m_state = SpiState::BUSY_TX_RX;

    uint32_t startTick = GetSysTick();

    // HW Trackers adjusted for the byte we just pushed
    uint32_t txBytesRemaining = totalSize - 1;
    uint32_t realTxLeft       = (txData.size() > 0) ? (txData.size() - 1) : 0;
    uint32_t rxClocksCounted  = 0;
    uint32_t realRxLeft       = rxData.size();

    // Safe pointer setups without sacrificing setup-latency speed
    uint8_t RxDummy = 0x00;
    const uint8_t* txPtr = txData.empty() ? &RxDummy : (txData.data() + 1);
    uint32_t txPointerIncrement = txData.empty() ? 0 : 1;

    uint8_t dummySink = 0;
    uint8_t* destPtr = rxData.data();
    uint32_t rxPointerIncrement = 1;

    if (rxData.empty()) {
        destPtr = &dummySink;
        rxPointerIncrement = 0;
    }

    // High-speed interleaved processing engine
    while (rxClocksCounted < totalSize) {

        // --- TRANSMIT HANDLER ---
        if (txBytesRemaining > 0 && (config.spi->SR & SPI_SR_TXE)) {
            if (realTxLeft > 0) {
                config.spi->DR = *txPtr;
                txPtr += txPointerIncrement;
                realTxLeft--;
            } else {
                config.spi->DR = 0x00;
            }
            txBytesRemaining--;
        }
        // --- RECEIVE HANDLER ---
        if (config.spi->SR & SPI_SR_RXNE) {
            uint8_t receivedByte = static_cast<uint8_t>(config.spi->DR);
            if (skipRxBytes > 0) {
                skipRxBytes--;
            } else if (realRxLeft > 0) {
                *destPtr = receivedByte;
                destPtr += rxPointerIncrement;
                realRxLeft--;
            }
            rxClocksCounted++;
        }
        if ((GetSysTick() - startTick) > timeoutMs) {
            m_state = SpiState::READY;
            return BareM_Status::TIMEOUT;
        }
    }
    while (config.spi->SR & SPI_SR_BSY);
    m_state = SpiState::READY;
    return BareM_Status::OK;
}


void SpiDriver::Handle_DMA_RX_IRQ()
{
    // Determine the status register and flag position from the configured
    // clear-register/mask, so this remains valid for any DMA stream.
    volatile uint32_t* statusReg =
        (config.rxFcrReg == &config.dmaBase->HIFCR)
            ? &config.dmaBase->HISR
            : &config.dmaBase->LISR;

    const uint32_t flagShift = __builtin_ctz(config.rxClearMask);
    const uint32_t tcMask    = (1U << (flagShift + 5U));
    const uint32_t errorMask =
        (1U << (flagShift + 3U)) |   // TEIF
        (1U << (flagShift + 2U)) |   // DMEIF
        (1U << (flagShift + 0U));     // FEIF

    const uint32_t flags = *statusReg;
    *config.rxFcrReg = config.rxClearMask;

    if (flags & tcMask)
    {
        // RX stream is automatically disabled when NDTR reaches zero.
    	m_state = SpiState::READY; // Reset state before user callback executes
        RxCpltCallback();
    }
    else if (flags & errorMask)
    {
        config.rxStream->CR &= ~DMA_SxCR_EN;
        m_state = SpiState::READY;
        ErrorCallback();
    }
}

void SpiDriver::Handle_DMA_TX_IRQ()
{
    // Stream 5 is on HISR for SPI3, but derive the correct status register
    // from the configured HIFCR/LIFCR pointer so the driver stays generic.
    volatile uint32_t* statusReg =
        (config.txFcrReg == &config.dmaBase->HIFCR)
            ? &config.dmaBase->HISR
            : &config.dmaBase->LISR;

    const uint32_t flagShift = __builtin_ctz(config.txClearMask);
    const uint32_t tcMask    = (1U << (flagShift + 5U));
    const uint32_t errorMask =
        (1U << (flagShift + 3U)) |   // TEIF
        (1U << (flagShift + 2U)) |   // DMEIF
        (1U << (flagShift + 0U));     // FEIF

    const uint32_t flags = *statusReg;
    *config.txFcrReg = config.txClearMask;

    if (flags & tcMask)
    {
        config.txStream->CR &= ~DMA_SxCR_EN;

        SpiState previousState = m_state;

        // DMA TC can occur while the final SPI bit is still shifting.
        while (config.spi->SR & SPI_SR_BSY);

        m_state = SpiState::READY;

        if (previousState == SpiState::BUSY_TX_RX)
        {
            // RX DMA is active in BUSY_TX_RX, so it drains SPI->DR.
            TxRxCpltCallback();
        }
        else if (previousState == SpiState::BUSY_TX)
        {
            TxCpltCallback();
        }
    }
    else if (flags & errorMask)
    {
        config.txStream->CR &= ~DMA_SxCR_EN;
        config.rxStream->CR &= ~DMA_SxCR_EN;
        *config.rxFcrReg = config.rxClearMask;
        m_state = SpiState::READY;
        ErrorCallback();
    }
}


// ==============================================================================
// WEAK DEFAULT CALLBACKS: The compiler will use these if they are not overriden
// ==============================================================================

__attribute__((weak)) void SpiDriver::TxCpltCallback() {
	// Example :
	/* 		if (this == &spi1) {
				// Pull Flash Chip Select high instantly!
			} else if (this == &spi2) {
				// Pull Display Chip Select high!
			}										*/
}

__attribute__((weak)) void SpiDriver::RxCpltCallback() {
    // Default: Do nothing safely

}

__attribute__((weak)) void SpiDriver::TxRxCpltCallback() {
    // Default: Do nothing safely
}

__attribute__((weak)) void SpiDriver::ErrorCallback() {
    // Default: Do nothing safely
	//if (this == &spi1) GPIOD->ODR^=GPIO_ODR_OD4; // Toggle orange led
}


/*================================================================
/==============    SPI1 INSTANCE CONFIGURATION   =================
/================================================================= */

// Forward declaration of local low-level pin mapping function
void Spi1_LowLevelInit(void);

// Compile-time hardware parameters (internal to this source file)
constexpr SpiHardwareConfig Spi1Config {
    SPI1,               // .spi
    DMA2_Stream2,       // .rxStream
    DMA2_Stream3,       // .txStream
    3,                  // .dmaChannel
    DMA2,               // .dmaBase
    SPI1_IRQn,          // .spiIrq
    DMA2_Stream2_IRQn,  // .rxDmaIrq
    DMA2_Stream3_IRQn,  // .txDmaIrq
    Spi1_LowLevelInit,   // .lowLevelInit
	&DMA2->LIFCR, (0x3DU << 22), // TX: Stream 3 is at bit 22 in LIFCR
	&DMA2->LIFCR, (0x3DU << 16)  // RX: Stream 2 is at bit 16 in LIFCR
};

// Global Instance Allocation (Instantiates the extern declared in the header)
SpiDriver spi1(Spi1Config);

// Concrete execution block mapping GPIO, AF5 configurations, and CS pins
void Spi1_LowLevelInit(void) {
    // 1. Enable Clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // 2. PA5 = SPI1_SCK (Alternate Function)
    GPIOA->MODER &= ~(3 << GPIO_MODER_MODER5_Pos);
    GPIOA->MODER |=  (2 << GPIO_MODER_MODER5_Pos);
    GPIOA->AFR[0] &= ~(0xF << GPIO_AFRL_AFSEL5_Pos);
    GPIOA->AFR[0] |=  (5   << GPIO_AFRL_AFSEL5_Pos);

    // 3. PB4 = MISO, PB5 = MOSI (Alternate Function)
    GPIOB->MODER &= ~((3 << GPIO_MODER_MODER4_Pos) | (3 << GPIO_MODER_MODER5_Pos));
    GPIOB->MODER |=  ((2 << GPIO_MODER_MODER4_Pos) | (2 << GPIO_MODER_MODER5_Pos));
    GPIOB->AFR[0] &= ~((0xF << GPIO_AFRL_AFSEL4_Pos) | (0xF << GPIO_AFRL_AFSEL5_Pos));
    GPIOB->AFR[0] |=  ((5   << GPIO_AFRL_AFSEL4_Pos) | (5   << GPIO_AFRL_AFSEL5_Pos));
    GPIOB->PUPDR &= ~(3 << GPIO_PUPDR_PUPD4_Pos); // Helps to gently pull (40Kohm) the MISO line back up to the VCC rail whenever the slave releases the bus
    GPIOB->PUPDR |=  (1 << GPIO_PUPDR_PUPD4_Pos); // 1 = Pull-up, 2 = Pull-down

    // CRITICAL: Force SPI Clock and Data lines to Very High Speed (30 MHz - 100 MHz+)
    GPIOA->OSPEEDR |= (3 << GPIO_OSPEEDR_OSPEED5_Pos);
    GPIOB->OSPEEDR |= (3 << GPIO_OSPEEDR_OSPEED4_Pos) | (3 << GPIO_OSPEEDR_OSPEED5_Pos);

    // 4. PA15 = CS1 (Standard GPIO Output, Low Speed for natural hardware padding)
    GPIOA->MODER   &= ~(3 << GPIO_MODER_MODER15_Pos);
    GPIOA->MODER   |=  (1 << GPIO_MODER_MODER15_Pos);
    GPIOA->OSPEEDR |= (2 << GPIO_OSPEEDR_OSPEED15_Pos); // 2 = High Speed
    GPIOA->BSRR     =  GPIO_BSRR_BS15;                    // Start High

    // 5. PC13 = CS2 (Standard GPIO Output, Low Speed)
    GPIOC->MODER   &= ~(3 << GPIO_MODER_MODER13_Pos);
    GPIOC->MODER   |=  (1 << GPIO_MODER_MODER13_Pos);
    GPIOC->OSPEEDR &= ~(3 << GPIO_OSPEEDR_OSPEED13_Pos);
    GPIOC->OSPEEDR |=  (2 << GPIO_OSPEEDR_OSPEED13_Pos); // 2 = High Speed, 1 = Mid Speed
    GPIOC->BSRR     =  GPIO_BSRR_BS13;                    // Start High
}


// C-Compatible Hardware Interrupt Vector Table Routing
extern "C" {
    void DMA2_Stream2_IRQHandler(void) {
        spi1.Handle_DMA_RX_IRQ();
    }

    void DMA2_Stream3_IRQHandler(void) {
        spi1.Handle_DMA_TX_IRQ();
    }
}



/*================================================================
/==============    SPI3 INSTANCE CONFIGURATION   =================
/================================================================= */

// Forward declaration of local low-level pin mapping function
void Spi3_LowLevelInit(void);

// Compile-time hardware parameters (internal to this source file)
constexpr SpiHardwareConfig Spi3Config {
    SPI3,                    // .spi
    DMA1_Stream0,            // .rxStream
    DMA1_Stream5,            // .txStream
    0,                       // .dmaChannel
    DMA1,                    // .dmaBase
    SPI3_IRQn,               // .spiIrq
    DMA1_Stream0_IRQn,       // .rxDmaIrq
    DMA1_Stream5_IRQn,       // .txDmaIrq
    Spi3_LowLevelInit,       // .lowLevelInit

    &DMA1->HIFCR, (0x3DU << 6), // TX: Stream 5
    &DMA1->LIFCR, (0x3DU << 0)   // RX: Stream 0
};


// Global Instance Allocation (Instantiates the extern declared in the header)
SpiDriver spi3(Spi3Config);

void Spi3_LowLevelInit(void)
{
    // Enable GPIO and SPI3 clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;

    // PC10 = SCK, PC11 = MISO, PC12 = MOSI, AF6
    GPIOC->MODER &= ~((3 << GPIO_MODER_MODER10_Pos) |
                      (3 << GPIO_MODER_MODER11_Pos) |
                      (3 << GPIO_MODER_MODER12_Pos));

    GPIOC->MODER |=  ((2 << GPIO_MODER_MODER10_Pos) |
                      (2 << GPIO_MODER_MODER11_Pos) |
                      (2 << GPIO_MODER_MODER12_Pos));

    GPIOC->AFR[1] &= ~((0xF << GPIO_AFRH_AFSEL10_Pos) |
                       (0xF << GPIO_AFRH_AFSEL11_Pos) |
                       (0xF << GPIO_AFRH_AFSEL12_Pos));

    GPIOC->AFR[1] |=  ((6 << GPIO_AFRH_AFSEL10_Pos) |
                       (6 << GPIO_AFRH_AFSEL11_Pos) |
                       (6 << GPIO_AFRH_AFSEL12_Pos));

    GPIOC->OSPEEDR |= (3 << GPIO_OSPEEDR_OSPEED10_Pos) |
                      (3 << GPIO_OSPEEDR_OSPEED11_Pos) |
                      (3 << GPIO_OSPEEDR_OSPEED12_Pos);

    // PD0 = W5500 CS
    GPIOD->MODER &= ~(3 << GPIO_MODER_MODER0_Pos);
    GPIOD->MODER |=  (1 << GPIO_MODER_MODER0_Pos);
    GPIOD->BSRR = GPIO_BSRR_BS0;

    // PD1 = W5500 RESET
    GPIOD->MODER &= ~(3 << GPIO_MODER_MODER1_Pos);
    GPIOD->MODER |=  (1 << GPIO_MODER_MODER1_Pos);
    GPIOD->BSRR = GPIO_BSRR_BR1;
}


// C-Compatible Hardware Interrupt Vector Table Routing
extern "C" {
    void DMA1_Stream0_IRQHandler(void) {
        spi3.Handle_DMA_RX_IRQ();
    }

    void DMA1_Stream5_IRQHandler(void) {
        spi3.Handle_DMA_TX_IRQ();
    }
}
