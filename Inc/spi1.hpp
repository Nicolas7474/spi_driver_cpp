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
    // Polling functions declarations
    BareM_Status Transmit(std::span<const uint8_t> txData, uint32_t timeoutMs);
    BareM_Status Transmit_MainBody(std::span<const uint8_t> txData, uint32_t timeoutMs);
    BareM_Status Receive(std::span<uint8_t> rxData, uint32_t timeoutMs);
    BareM_Status TransmitReceive(std::span<const uint8_t> txData, std::span<uint8_t> rxData, uint32_t timeoutMs);
    BareM_Status TransmitReceive_MainBody(std::span<const uint8_t> txData, std::span<uint8_t> rxData, uint32_t timeoutMs);
    // DMA functions declarations
    BareM_Status Receive_DMA(std::span<uint8_t> rxData);
    BareM_Status Transmit_DMA(std::span<const uint8_t> txdata);


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


/* =======================================================================================================================
  SEPARATE HIGH-SPEED INLINE DEFINITIONS : the compiler literally erases the concept of the function call.
  Reduces drastically the delay between falling edge of CS and first edge of MOSI (more than factor 2).
  Used with polling transfers only, since DMA introduces a much larger latency and inlining has only little effect.
  Split the function into a fast inline header and a heavy static worker, which avoids code bloat: if a
  large inline function is called in 20 different places throughout the codebase, the complete function code
  will be copied into the flash memory 20 distinct times and can quickly deplete the available flash space.
  __attribute__((always_inline)) should be reserved when timing is critical
======================================================================================================================= */


__attribute__((always_inline)) inline
BareM_Status SpiDriver::Transmit(std::span<const uint8_t> txData, uint32_t timeoutMs) {
	// FAST-PATH KICK: Compiles to an ultra-fast conditional or direct store
	if (__builtin_expect(m_state == SpiState::READY, 1)) {
		if (!txData.empty()) {				// Kick the hardware immediately (the first byte)
			config.spi->DR = txData[0]; 	// Reduce drastically the delay between falling edge of CS and that of CLK (780ns -> 420ns)
		} else {
			return BareM_Status::ERROR;
		}
	} else {
		// FALLBACK: The programmer made a mistake or a background task is running
		uint32_t timeout_ct = GetSysTick() + timeoutMs;
		// We wait/block right here until the previous transaction clears up or times out
		while (m_state != SpiState::READY) {
			if (GetSysTick() > timeout_ct) {
				return BareM_Status::TIMEOUT;
			}
		}
		// The bus finally cleared, fire the delayed first byte
		if (!txData.empty()) {
			config.spi->DR = txData[0];
		} else {
			return BareM_Status::ERROR;
		}
	}
	return Transmit_MainBody(txData, timeoutMs);  // Hand off the heavy lifting to the single, shared function in Flash
}


__attribute__((always_inline)) inline
BareM_Status SpiDriver::TransmitReceive(std::span<const uint8_t> txData, std::span<uint8_t> rxData, uint32_t timeoutMs) {
	// Clean, optimized entry user point - fast safety check
	if (__builtin_expect(m_state == SpiState::READY, 1)) {
		// FAST-PATH KICK: Compiles to an ultra-fast conditional or direct store
		if (!txData.empty()) {				// Kick the hardware immediately (the first byte)
			config.spi->DR = txData[0]; 	// Reduce drastically the delay between falling edge of CS and that of CLK (780ns -> 420ns)
		} else {
			config.spi->DR = 0x00; // Safe dummy kick for pure Receive operations
		}
	} else {
        // FALLBACK: The programmer made a mistake or a background task is running
        uint32_t timeout_ct = GetSysTick() + timeoutMs;
        while (m_state != SpiState::READY) {
            if (GetSysTick() > timeout_ct) {
                return BareM_Status::TIMEOUT;
            }
        }
        if (!txData.empty()) {
            config.spi->DR = txData[0];
        } else {
            config.spi->DR = 0x00;
        }
    }
    return TransmitReceive_MainBody(txData, rxData, timeoutMs);  // Hand off the heavy lifting to the single, shared function in Flash
}



extern SpiDriver spi1;



