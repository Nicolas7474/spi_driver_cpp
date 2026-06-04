#pragma once

#include <stdint.h>
#include <span>
#include "stm32f4xx.h"
#include "timers.h"


// Scoped enums for stricter type checking
enum class BareM_Status : uint8_t {
    OK      = 0x00U,
    ERROR   = 0x01U,
    BUSY    = 0x02U,
    TIMEOUT = 0x03U
};

enum class SpiState : uint8_t {
    RESET = 0,
    READY,
    BUSY_RX,
    BUSY_TX,
	BUSY_TX_RX
};

// Strongly-typed clock prescaler options
enum class BaudRatePrescaler : uint32_t {
    DIV_2   = 0,
    DIV_4   = (1 << SPI_CR1_BR_Pos),
    DIV_8   = (2 << SPI_CR1_BR_Pos),
    DIV_16  = (3 << SPI_CR1_BR_Pos),
    DIV_32  = (4 << SPI_CR1_BR_Pos),
    DIV_64  = (5 << SPI_CR1_BR_Pos),
    DIV_128 = (6 << SPI_CR1_BR_Pos),
    DIV_256 = (7 << SPI_CR1_BR_Pos)
};

using SpiLowLevelInitFn = void(*)(void);

// Compile-time hardware manifest structure
struct SpiHardwareConfig {
    SPI_TypeDef* spi;
    DMA_Stream_TypeDef* rxStream;
    DMA_Stream_TypeDef* txStream;
    uint32_t            dmaChannel;
    DMA_TypeDef* dmaBase;
    IRQn_Type           spiIrq;
    IRQn_Type           rxDmaIrq;
    IRQn_Type           txDmaIrq;
    SpiLowLevelInitFn   lowLevelInit;
};



class SpiDriver {
public:
    explicit SpiDriver(const SpiHardwareConfig& hardwareConfig);
    ~SpiDriver() = default;

    BareM_Status Init(BaudRatePrescaler prescaler);
    BareM_Status Transmit(const uint8_t* data, uint32_t size, uint32_t timeoutMs);
    BareM_Status Receive(uint8_t* data, uint32_t size, uint32_t timeoutMs);
    BareM_Status Transmit_DMA(std::span<const uint8_t> payload);
    BareM_Status Receive_DMA(uint8_t* pData, uint32_t len);

    // This stays completely hidden in your .cpp file, drawing zero flash bloat
    BareM_Status TransmitReceive_PollingLoop(std::span<const uint8_t> txData, std::span<uint8_t> rxData, uint32_t timeoutMs);
    // Inline function - he compiler literally erases the concept of the function call
    // Reduce drastically the delay between falling edge of CS and first edge of CLK (420ns->165ns)
    BareM_Status TransmitReceive(std::span<const uint8_t> txData, std::span<uint8_t> rxData, uint32_t timeoutMs);

    SpiState GetState() const { return m_state; }
    void Handle_DMA_RX_IRQ();
    void Handle_DMA_TX_IRQ();

    void CS1_Low()  { GPIOA->BSRR = GPIO_BSRR_BR15; }
    void CS1_High() { GPIOA->BSRR = GPIO_BSRR_BS15; }
    static void ChipSelect2_Low()  { GPIOC->BSRR = GPIO_BSRR_BR13; }
    static void ChipSelect2_High() { GPIOC->BSRR = GPIO_BSRR_BS13; }

private:
    const SpiHardwareConfig& config;
    volatile SpiState        m_state{SpiState::RESET};
    volatile uint8_t         m_dummyTx{0x00};
    bool                     m_isDmaInitialized{false};

    void ConfigureDma();
    void ClearDmaFlags(DMA_Stream_TypeDef* stream);
};


/* SEPARATE HIGH-SPEED INLINE DEFINITION
  Split the function into a fast inline header and a heavy static worker avoids Code Bloat (Flash Memory Expansion).
  If you call a big inline function in 20 different places throughout the codebase, the complete, complex while loop engine
  will be copied into the flash memory 20 distinct times. For a large bare-metal application running on a memory-constrained MCU,
  this can quickly deplete the available flash space. __attribute__((always_inline)) should be reserved for cases where timing is critical.
 */
__attribute__((always_inline)) inline BareM_Status SpiDriver::TransmitReceive(std::span<const uint8_t> txData, std::span<uint8_t> rxData, uint32_t timeoutMs) {
	// This is the clean, hyper-optimized entry point the user calls

    // FAST-PATH KICK: Compiles to an ultra-fast conditional or direct store
    if (!txData.empty()) {				// Kick the hardware immediately (the first byte)
        config.spi->DR = txData[0]; 	// Reduce drastically the delay between falling edge of CS and first edge of CLK (780ns->420ns)
    } else {
        config.spi->DR = 0x00; // Safe dummy kick for pure Receive operations
    }

    return TransmitReceive_PollingLoop(txData, rxData, timeoutMs);  // Hand off the heavy lifting to the single, shared function in Flash
}


extern SpiDriver spi1;



