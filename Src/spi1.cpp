/*
*		SPI - Polling & DMA-based driver
* 		with MISO = PB4; MOSI = PB5; SCK = PA5; NSS = PA15
*
*   About the combined TransmitReceive_DMA() function and the rule of symmetrical DMA drivers:
*   - txData.size() must equal rxData.size().
*   - That size must represent the TOTAL number of clock pulses needed for the entire conversation.
*   - The real incoming data will always be offset in your receive buffer by the length of your command header !
*   - Allocating a massive array to send only a few command bytes (ex: reading a Flash sector) is a massive waste of precious RAM.
*   -> Use TransmitReceive_DMA (Symmetrical) only for tiny, fixed-size control messages (like reading the 3-byte Unique ID) where creating a tiny matching array is trivial.
*	-> Use Transmit_DMA followed by Receive_DMA for heavy payload operations (like 4KB sector reads or 256-byte page reads) to keep your RAM completely clean.
*	-> Polling is slightly faster (1µs less latency than equivalent DMA functions at 11.25Mhz) - best for small transfers if blocking is not a problem
*	-> Using Polling TransmitReceive() is actually slightly slower than combining the separate Transmit() + Receive() functions
*/


#include "spi1.hpp"
#include <span>



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
    uint32_t timeout_ct = GetSysTick() + timeoutMs;

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
        if (GetSysTick() > timeout_ct) {
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

    uint32_t timeout_ct = GetSysTick() + timeoutMs;

    // Check if the peripheral is ready
    while (m_state != SpiState::READY) {
        if (GetSysTick() > timeout_ct) {
            m_state = SpiState::READY;
            return BareM_Status::TIMEOUT;
        }
    }
    m_state = SpiState::BUSY_TX;

    const uint8_t* txPtr = txData.data();
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
        if (GetSysTick() > timeout_ct) {
            m_state = SpiState::READY;
            return BareM_Status::TIMEOUT;
        }
    }

    // Wait to allow Chip Select (CS) to be pulled high again
    while (config.spi->SR & SPI_SR_BSY) {
        if (GetSysTick() > timeout_ct) {
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
    uint32_t timeout_ct = GetSysTick() + timeoutMs;

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
        if (GetSysTick() > timeout_ct) {
            m_state = SpiState::READY;
            return BareM_Status::TIMEOUT;
        }
    }
    while (config.spi->SR & SPI_SR_BSY);
    m_state = SpiState::READY;
    return BareM_Status::OK;
}







void SpiDriver::Handle_DMA_RX_IRQ() {
    // RX is a pure data worker. We clear flags and leave CR alone.
    // The STM32 hardware automatically disables the stream when NDTR hits 0.
	*config.rxFcrReg = config.rxClearMask; // Instant hardware clear
}

void SpiDriver::Handle_DMA_TX_IRQ() {
    uint32_t lisr = config.dmaBase->LISR;
    *config.txFcrReg = config.txClearMask; // Instant hardware clear

    if (__builtin_expect(lisr & DMA_LISR_TCIF3, 1)) {
        config.txStream->CR &= ~DMA_SxCR_EN; // Explicit shutdown

        // CRITICAL: Ensure the hardware shift registers are 100% empty
        // and the physical pins have stopped toggling before freeing the driver
        while (config.spi->SR & SPI_SR_BSY);
        m_state = SpiState::READY;
    } else {
        // Fallback for DMA Errors (TEIF, DMEIF)
        config.txStream->CR &= ~DMA_SxCR_EN;
        config.rxStream->CR &= ~DMA_SxCR_EN;
        *config.rxFcrReg = config.rxClearMask; // Clean RX stream on unexpected fault
        m_state = SpiState::READY;
    }
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
