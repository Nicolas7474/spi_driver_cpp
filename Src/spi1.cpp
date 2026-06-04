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

    // 1. Configure Rx Stream
    config.rxStream->CR &= ~DMA_SxCR_EN;
    while (config.rxStream->CR & DMA_SxCR_EN);
    ClearDmaFlags(config.rxStream);

    config.rxStream->CR = (config.dmaChannel << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC
                        | DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;
    config.rxStream->CR &= ~(3 << DMA_SxCR_DIR_Pos);
    config.rxStream->FCR |= DMA_SxFCR_DMDIS;
    config.rxStream->FCR |= (DMA_SxFCR_FTH_0 | DMA_SxFCR_FTH_1);

    NVIC_SetPriority(config.rxDmaIrq, 2);
    NVIC_EnableIRQ(config.rxDmaIrq);

    // 2. Configure Tx Stream
    config.txStream->CR &= ~DMA_SxCR_EN;
    while (config.txStream->CR & DMA_SxCR_EN);
    ClearDmaFlags(config.txStream);

    config.txStream->CR = (config.dmaChannel << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_DIR_0
                        | DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;
    config.txStream->FCR |= DMA_SxFCR_DMDIS;

    NVIC_SetPriority(config.txDmaIrq, 2);
    NVIC_EnableIRQ(config.txDmaIrq);

    m_isDmaInitialized = true;
}

void SpiDriver::ClearDmaFlags(DMA_Stream_TypeDef* stream) {
    uint32_t streamIndex = 0;
    if (stream == DMA2_Stream0 || stream == DMA1_Stream0) streamIndex = 0;
    else if (stream == DMA2_Stream1 || stream == DMA1_Stream1) streamIndex = 6;
    else if (stream == DMA2_Stream2 || stream == DMA1_Stream2) streamIndex = 16;
    else if (stream == DMA2_Stream3 || stream == DMA1_Stream3) streamIndex = 22;
    else if (stream == DMA2_Stream4 || stream == DMA1_Stream4) streamIndex = 0;
    else if (stream == DMA2_Stream5 || stream == DMA1_Stream5) streamIndex = 6;
    else if (stream == DMA2_Stream6 || stream == DMA1_Stream6) streamIndex = 16;
    else if (stream == DMA2_Stream7 || stream == DMA1_Stream7) streamIndex = 22;

    uint32_t mask = (0x3DU << streamIndex);
    if (stream < DMA2_Stream4) {
        config.dmaBase->LIFCR = mask;
    } else {
        config.dmaBase->HIFCR = mask;
    }
}





BareM_Status SpiDriver::Receive(std::span<uint8_t> rxData, uint32_t timeoutMs) {
    if (rxData.empty()) return BareM_Status::ERROR;

    uint32_t timeout_ct = GetSysTick() + timeoutMs;

    // Check if the peripheral is ready
    while (m_state != SpiState::READY) {
    	if (GetSysTick() > timeout_ct) {
    		m_state = SpiState::READY;
    		return BareM_Status::TIMEOUT;
    	}
    }
    m_state = SpiState::BUSY_RX;

    // Clear any residual garbage flag
    [[maybe_unused]] volatile uint32_t dummyRead = config.spi->DR;

    uint32_t txCounter = rxData.size();
    uint32_t rxCounter = rxData.size();
    uint8_t* destPtr = rxData.data();

    // PIPELINED LOOP: Separate the TX pushes from the RX pulls
    while (rxCounter > 0) {
        // 1. Keep the TX pipeline full (Up to 2 bytes can be safely queued in hardware)
        if (txCounter > 0 && (config.spi->SR & SPI_SR_TXE)) {
            config.spi->DR = 0x00; // Drop a dummy byte into the hardware buffer
            txCounter--;
        }
        // 2. Read bytes as soon as they arrive
        if (config.spi->SR & SPI_SR_RXNE) {
            *destPtr++ = static_cast<uint8_t>(config.spi->DR);
            rxCounter--;
        }
        // 3. Optional: Add your timeout check here if needed
        if (GetSysTick() > timeout_ct) {
            m_state = SpiState::READY;
            return BareM_Status::TIMEOUT;
        }
    }
    // Ensure the final bits completely clear the physical shifts before exiting
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



BareM_Status SpiDriver::Transmit_DMA(std::span<const uint8_t> payload) {

    if (payload.empty()) return BareM_Status::ERROR;
    while (m_state == SpiState::BUSY_TX);

    m_state = SpiState::BUSY_TX;
    config.txStream->CR &= ~DMA_SxCR_EN;
    while (config.txStream->CR & DMA_SxCR_EN);

    ClearDmaFlags(config.txStream);

    config.txStream->PAR  = reinterpret_cast<uint32_t>(&config.spi->DR);
    config.txStream->M0AR = reinterpret_cast<uint32_t>(payload.data());
    config.txStream->NDTR = static_cast<uint16_t>(payload.size()); // NDTR is on 16 bits

    config.txStream->CR  |= DMA_SxCR_EN;

    //while (m_state != SpiState::READY);
    return BareM_Status::OK;
}

BareM_Status SpiDriver::Receive_DMA(uint8_t* pData, uint32_t len) {
    if (len == 0) return BareM_Status::ERROR;
    //while (m_state == SpiState::BUSY_RX);

    m_state = SpiState::BUSY_RX;
    config.rxStream->CR &= ~DMA_SxCR_EN;
    while ((config.txStream->CR & DMA_SxCR_EN) || (config.rxStream->CR & DMA_SxCR_EN));

    ClearDmaFlags(config.txStream);
    ClearDmaFlags(config.rxStream);

    config.rxStream->PAR  = reinterpret_cast<uint32_t>(&config.spi->DR);
    config.rxStream->M0AR = reinterpret_cast<uint32_t>(pData);
    config.rxStream->NDTR = len;

    config.txStream->PAR  = reinterpret_cast<uint32_t>(&config.spi->DR);
    config.txStream->M0AR = reinterpret_cast<uint32_t>(&m_dummyTx);
    config.txStream->NDTR = len;

    [[maybe_unused]] volatile uint32_t tmpreg = config.spi->DR;

    config.rxStream->CR |= DMA_SxCR_EN;
    config.txStream->CR |= DMA_SxCR_EN;

    while (m_state != SpiState::READY);
    return BareM_Status::OK;
}

void SpiDriver::Handle_DMA_RX_IRQ() {
    uint32_t lisr = config.dmaBase->LISR;
    if (lisr & DMA_LISR_TCIF2) {
        ClearDmaFlags(config.rxStream);
        config.rxStream->CR &= ~DMA_SxCR_EN;
        m_state = SpiState::READY;
    }
}

void SpiDriver::Handle_DMA_TX_IRQ() {
    uint32_t lisr = config.dmaBase->LISR;
    if (lisr & DMA_LISR_TCIF3) {
        ClearDmaFlags(config.txStream);
        config.txStream->CR &= ~DMA_SxCR_EN;
        while (config.spi->SR & SPI_SR_BSY);
        /* Trick to reduce drastically the delay between the last edge of the clock and the raising edge of CS:
           if CS_High() command was written, the CS signal is triggered directly in the ISR. No wait for status Ready */
        m_state = SpiState::READY;
    }
}


/***************************** SPI1 INSTANCE CONFIGURATION ********************************/

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
    Spi1_LowLevelInit   // .lowLevelInit
};

// Global Instance Allocation (Instantiates the extern declared in the header)
SpiDriver spi1(Spi1Config);

// C-Compatible Hardware Interrupt Vector Table Routing
extern "C" {
    void DMA2_Stream2_IRQHandler(void) {
        spi1.Handle_DMA_RX_IRQ();
        NVIC_ClearPendingIRQ(DMA2_Stream2_IRQn);
    }

    void DMA2_Stream3_IRQHandler(void) {
        spi1.Handle_DMA_TX_IRQ();
        NVIC_ClearPendingIRQ(DMA2_Stream3_IRQn);
    }
}

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
    GPIOB->PUPDR &= ~(3 << GPIO_PUPDR_PUPD14_Pos); // gently pulls (40Kohm) the MISO line back up to the VCC rail whenever the slave releases the bus
    GPIOB->PUPDR |=  (1 << GPIO_PUPDR_PUPD14_Pos); // 1 = Pull-up, 2 = Pull-down

    // CRITICAL: Force SPI Clock and Data lines to Very High Speed (30 MHz - 100 MHz+)
    GPIOA->OSPEEDR |= (3 << GPIO_OSPEEDR_OSPEED5_Pos);
    GPIOB->OSPEEDR |= (3 << GPIO_OSPEEDR_OSPEED4_Pos) | (3 << GPIO_OSPEEDR_OSPEED5_Pos);

    // 4. PA15 = CS1 (Standard GPIO Output, Low Speed for natural hardware padding)
    GPIOA->MODER   &= ~(3 << GPIO_MODER_MODER15_Pos);
    GPIOA->MODER   |=  (1 << GPIO_MODER_MODER15_Pos);
    GPIOA->OSPEEDR |= (1 << GPIO_OSPEEDR_OSPEED15_Pos); // 01 = Mid Speed
    GPIOA->BSRR     =  GPIO_BSRR_BS15;                    // Start High

    // 5. PC13 = CS2 (Standard GPIO Output, Low Speed)
    GPIOC->MODER   &= ~(3 << GPIO_MODER_MODER13_Pos);
    GPIOC->MODER   |=  (1 << GPIO_MODER_MODER13_Pos);
    GPIOC->OSPEEDR |= ~(3 << GPIO_OSPEEDR_OSPEED13_Pos); //  Speed ??
    GPIOC->BSRR     =  GPIO_BSRR_BS13;                    // Start High
}
