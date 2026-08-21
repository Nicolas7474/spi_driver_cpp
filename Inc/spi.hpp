#pragma once

#include <stdint.h>
#include <span>
#include "stm32f4xx.h"
#include <stm32f469xx.h>
#include "myConfig.h"
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
    // Added these fields for zero-latency flag clearing
    volatile uint32_t* txFcrReg; // Pointer to LIFCR or HIFCR
    uint32_t txClearMask;        // Exact 0x3D bitmask shifted to correct stream position
    volatile uint32_t* rxFcrReg;
    uint32_t rxClearMask;
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
    BareM_Status Receive_MainBody(std::span<uint8_t> rxData, uint32_t timeoutMs);
    BareM_Status TransmitReceive(std::span<const uint8_t> txData, std::span<uint8_t> rxData, uint32_t timeoutMs);
    // DMA functions declarations
    BareM_Status Receive_DMA(std::span<uint8_t> rxData);
    BareM_Status Transmit_DMA(std::span<const uint8_t> txdata);
    BareM_Status TransmitReceive_DMA(std::span<const uint8_t> txData, std::span<uint8_t> rxData);

    static void CS1_Low()  { GPIOA->BSRR = GPIO_BSRR_BR15; } // "static" strips away the hidden this pointer requirement
    static void CS1_High() { GPIOA->BSRR = GPIO_BSRR_BS15; } // Enforces the SpiDriver:: or spi. prefix scoping
    static void CS2_Low()  { GPIOC->BSRR = GPIO_BSRR_BR13; } // The function becomes a regular, global function
    static void CS2_High() { GPIOC->BSRR = GPIO_BSRR_BS13; }

    SpiState GetState() const { return m_state; }

    void Handle_DMA_RX_IRQ();
    void Handle_DMA_TX_IRQ();

    void TxCpltCallback(); 	// Callbacks declared inside the class
    void RxCpltCallback(); 	// to enforce the spi1 prefix scoping
    void TxRxCpltCallback(); // Hard-coded (no function pointers)
    void ErrorCallback();	// for less time penalty

private:
    const SpiHardwareConfig& config;
    volatile SpiState        m_state{SpiState::RESET};
    volatile uint8_t         m_dummyTx{0x00};
    volatile uint8_t 		 m_dummyRx{0x00}; // Dedicated garbage bin for unwanted incoming data
    bool                     m_isDmaInitialized{false};

    void ConfigureDma();
};


/* =======================================================================================================================
  SEPARATE HIGH-SPEED INLINE (time-critical) DEFINITIONS
  More prominent with polling transfers, since DMA preparations introduce a big latency and inlining has a lesser impact.
  The compiler literally erases the concept of the function call, reducing drastically the delay between the
  falling edge of CS and first MOSI data (more than factor 2).
  Split the function into a fast inline header and a heavy static worker, which avoids code bloat: if a
  large inline function is called in 20 different places throughout the codebase, the complete function code
  will be copied into the flash memory 20 distinct times and can quickly deplete the available flash space.
======================================================================================================================= */

/*================================================================
/ ==============    SPI FUNCTIONS DMA MODE   =====================
/ ================================================================ */


__attribute__((always_inline)) inline
BareM_Status SpiDriver::Receive_DMA(std::span<uint8_t> rxData) {
	// First CLK edge delay reduced -200ns with function inlining

	if (rxData.empty()) return BareM_Status::ERROR;

	// Fast guard: Compiler assumes this if statement is false
	// and places the hot registration blitting directly in the pipeline stream.
	if (__builtin_expect(m_state != SpiState::READY, 0)) {
		const uint32_t startTick = GetSysTick();
		while (m_state != SpiState::READY) {
			 if ((GetSysTick() - startTick) >= 10) {
				m_state = SpiState::READY; // Force reset driver state
				return BareM_Status::TIMEOUT;
			}
		}
	}
	m_state = SpiState::BUSY_RX;

	[[maybe_unused]] volatile uint32_t tmpreg = config.spi->DR;

	config.rxStream->CR &= ~DMA_SxCR_EN; // Disable DMA stream to allow configuration
	config.txStream->CR &= ~DMA_SxCR_EN;
	while ((config.txStream->CR & DMA_SxCR_EN) || (config.rxStream->CR & DMA_SxCR_EN));
	// ClearDmaFlags() already called in the ISR, no need again here

	config.rxStream->CR |= DMA_SxCR_MINC; // Re-enable in case it was disabled in Transmit_DMA
	config.rxStream->M0AR = reinterpret_cast<uintptr_t>(rxData.data());
	config.rxStream->NDTR = static_cast<uint16_t>(rxData.size());

	config.txStream->M0AR = reinterpret_cast<uintptr_t>(&m_dummyTx);
	config.txStream->NDTR = rxData.size(); // Dummy bytes to generate the CLK required for the Slave to shift out its data

	config.txStream->CR &= ~DMA_SxCR_MINC; // Disable Memory Increment on TX so it locks onto the single dummy variable address
	config.rxStream->CR |= DMA_SxCR_EN;
	config.txStream->CR |= DMA_SxCR_EN;

	return BareM_Status::OK;  // while(m_state!=SpiState::READY) -> no blocking before returning
}


__attribute__((always_inline)) inline
BareM_Status SpiDriver::Transmit_DMA(std::span<const uint8_t> txdata)
{
    if (txdata.empty() || txdata.size() > UINT16_MAX)
        return BareM_Status::ERROR;

    // Fast guard: READY is the normal case
    if (__builtin_expect(m_state != SpiState::READY, 0))
    {
        uint32_t timeout_ct = GetSysTick() + 10;

        while (m_state != SpiState::READY)
        {
            if (GetSysTick() > timeout_ct)
            {
                m_state = SpiState::READY;
                return BareM_Status::TIMEOUT;
            }
        }
    }

    m_state = SpiState::BUSY_TX;

    // --- Configure RX DMA stream (dummy drain) ---
    //  Every byte clocked out by TX has a corresponding RX byte consumed by DMA.
    // No RX overrun should occur simply because Transmit_DMA() is TX-only from the API perspective.
    config.rxStream->CR &= ~DMA_SxCR_EN;
    while (config.rxStream->CR & DMA_SxCR_EN);

    *config.rxFcrReg = config.rxClearMask;
    config.rxStream->CR &= ~DMA_SxCR_MINC; // Disable memory increment: all received bytes go to m_dummyRx.
    config.rxStream->PAR = reinterpret_cast<uint32_t>(&config.spi->DR);
    config.rxStream->M0AR = reinterpret_cast<uint32_t>(&m_dummyRx);
    config.rxStream->NDTR = static_cast<uint16_t>(txdata.size());

    // ---  Configure TX DMA stream ---
    config.txStream->CR &= ~DMA_SxCR_EN;
    while (config.txStream->CR & DMA_SxCR_EN);

    *config.txFcrReg = config.txClearMask;
    config.txStream->CR |= DMA_SxCR_MINC;
    config.txStream->PAR = reinterpret_cast<uint32_t>(&config.spi->DR);
    config.txStream->M0AR = reinterpret_cast<uintptr_t>(txdata.data());
    config.txStream->NDTR = static_cast<uint16_t>(txdata.size());

    // RX first: make sure every received byte has somewhere to go before TX starts generating SPI clocks.
    config.rxStream->CR |= DMA_SxCR_EN;
    config.txStream->CR |= DMA_SxCR_EN;

    return BareM_Status::OK;
}



__attribute__((always_inline)) inline
BareM_Status SpiDriver::TransmitReceive_DMA(std::span<const uint8_t> txData, std::span<uint8_t> rxData) {
	// First MOSI byte delay reduced -250ns with function inlining

	if (__builtin_expect(txData.empty() || txData.size() != rxData.size(), 0)) {
		return BareM_Status::ERROR;
	}

	// Fast guard: Compiler assumes this if statement is false
	// and places the hot registration blitting directly in the pipeline stream.
	if (__builtin_expect(m_state != SpiState::READY, 0)) {
		const uint32_t startTick = GetSysTick();
		while (m_state != SpiState::READY) {
			 if ((GetSysTick() - startTick) >= 10) {
				m_state = SpiState::READY; // Force reset driver state
				return BareM_Status::TIMEOUT;
			}
		}
	}
	m_state = SpiState::BUSY_TX_RX;

	[[maybe_unused]] volatile uint32_t tmpreg = config.spi->DR;

	// Optimization (delay -250ns ): only disable and wait if a stream is actually still running
	if (__builtin_expect((config.txStream->CR & DMA_SxCR_EN) || (config.rxStream->CR & DMA_SxCR_EN), 0)) {
		config.rxStream->CR &= ~DMA_SxCR_EN;
    	config.txStream->CR &= ~DMA_SxCR_EN;
    	while ((config.txStream->CR & DMA_SxCR_EN) || (config.rxStream->CR & DMA_SxCR_EN));
    }
    // ClearDmaFlags() already called in the ISR, no need again here

    uint32_t transferSize = txData.size();

    // --- RX Stream Configuration ---
    config.rxStream->M0AR = reinterpret_cast<uintptr_t>(rxData.data());
    config.rxStream->NDTR = transferSize; // Corrected: Removed the "+ 1" overread trap
    config.rxStream->CR  |= DMA_SxCR_MINC; // Re-enable the RX increments memory in case it was disabled in Transmit_DMA()

    // --- TX Stream Configuration ---
    config.txStream->M0AR = reinterpret_cast<uintptr_t>(txData.data());
    config.txStream->NDTR = transferSize; // Corrected: Removed the "+ 1" overread trap
    config.txStream->CR  |= DMA_SxCR_MINC; // Keep memory increment enabled

    // --- Synchronous Engine Launch ---
    config.rxStream->CR |= DMA_SxCR_EN; // Corrected: Enable RX first so it waits for incoming data
    config.txStream->CR |= DMA_SxCR_EN; // Enable TX second; clock pulses begin instantly

    return BareM_Status::OK;  // No blocking before returning
}


/*================================================================
/ ==============    SPI FUNCTIONS POLLING MODE   =================
/ ================================================================ */

__attribute__((always_inline)) inline
BareM_Status SpiDriver::Receive(std::span<uint8_t> rxData, uint32_t timeoutMs) {
	// Delay reduced ca. 150ns with function inlining

    if (__builtin_expect(rxData.empty(), 0)) return BareM_Status::ERROR;

    // First CLK edge delay reduced -210ns with call the hardware immediately
    if (__builtin_expect(m_state == SpiState::READY, 1)) {
        m_state = SpiState::BUSY_RX;
        [[maybe_unused]] volatile uint32_t dummyRead = config.spi->DR; // Clear residual flags

        config.spi->DR = 0x00; // FAST KICK: Instantly start generating clock pulses!
    } else {
        // FALLBACK: Wait for a background task or prior transfer to finish
        uint32_t timeout_ct = GetSysTick() + timeoutMs;
        while (m_state != SpiState::READY) {
            if (GetSysTick() > timeout_ct) {
                return BareM_Status::TIMEOUT;
            }
        }
        m_state = SpiState::BUSY_RX;
        [[maybe_unused]] volatile uint32_t dummyRead = config.spi->DR;

        config.spi->DR = 0x00; // Delayed kick
    }

    // Hand off the rest of the extraction loop to the Flash-resident main body
    return Receive_MainBody(rxData, timeoutMs);
}


__attribute__((always_inline)) inline
BareM_Status SpiDriver::Transmit(std::span<const uint8_t> txData, uint32_t timeoutMs) {
	// Fast path kick: Compiles to an ultra-fast conditional or direct store
	if (__builtin_expect(m_state == SpiState::READY, 1)) {
		if (!txData.empty()) {				// Kick the hardware immediately (the first byte)
			config.spi->DR = txData[0]; 	// Reduce drastically the delay between falling edge of CS and that of CLK (780ns -> 420ns)
		} else {
			return BareM_Status::ERROR;
		}
	} else {
		// Fallback: The programmer made a mistake or a background task is running
		const uint32_t startTick = GetSysTick();
		// We wait/block right here until the previous transaction clears up or times out
		while (m_state != SpiState::READY) {
			if ((GetSysTick() - startTick) >= timeoutMs) {
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
BareM_Status SpiDriver::TransmitReceive(std::span<const uint8_t> txData, std::span<uint8_t> rxData, uint32_t timeoutMs)
{

    // VALIDATION
    if (__builtin_expect(txData.empty() && rxData.empty(), 0))
        return BareM_Status::ERROR;

    // WAIT FOR SPI DRIVER TO BECOME AVAILABLE
    if (__builtin_expect(m_state != SpiState::READY, 0))
    {
        const uint32_t waitStart = GetSysTick();

        while (m_state != SpiState::READY)
        {
            if ((GetSysTick() - waitStart) >= timeoutMs)
                return BareM_Status::TIMEOUT;
        }
    }

    // ================================================================
    // DETERMINE TOTAL NUMBER OF SPI CLOCKED BYTES    //
    // Equal TX/RX sizes:
    //     TX and RX are simultaneous for the whole transfer.
    // Different TX/RX sizes:
    //     TX bytes are sent first, then dummy bytes generate the
    //     remaining clocks needed to receive the RX payload.
    // Example:
    //     TX = 3 bytes, RX = 1 byte
    //     SPI clocks = 4 bytes
    //     RX byte #1, #2, #3 = command/header garbage
    //     RX byte #4         = actual payload
    // ================================================================

    uint32_t totalSize;
    uint32_t skipRxBytes = 0;

    if (!txData.empty() && !rxData.empty() && txData.size() != rxData.size())
    {
        totalSize = static_cast<uint32_t>(txData.size() + rxData.size());
        skipRxBytes = static_cast<uint32_t>(txData.size());
    }
    else
    {
        totalSize = static_cast<uint32_t>((txData.size() > rxData.size()) ? txData.size() : rxData.size());
    }

    // START TRANSFER
    m_state = SpiState::BUSY_TX_RX;
    const uint32_t startTick = GetSysTick();
    // Number of bytes already transmitted.
    uint32_t txBytesSent = 0;

    // Number of SPI bytes already received.
    uint32_t rxClocksCounted = 0;

    // Number of actual RX payload bytes still wanted.
    uint32_t realRxLeft = static_cast<uint32_t>(rxData.size());

    // Number of initial RX bytes to discard.
    uint32_t rxBytesToSkip = skipRxBytes;

    // Dummy destination for transfers where the caller doesn't request RX data.
    uint8_t dummyRx = 0;
    uint8_t* rxPtr = rxData.empty() ? &dummyRx : rxData.data();

    // ================================================================
    // STANDARD FULL-DUPLEX POLLING LOOP
    // No special first-byte kick.
    // TX and RX are serviced independently. As soon as TXE becomes
    // available, the next byte is written to DR.
    // This is the important part for eliminating the artificial gap
    // between byte #1 and byte #2.
    // ================================================================

    while (rxClocksCounted < totalSize)
    {

        // TRANSMIT
        // Keep the SPI TX register supplied as soon as possible.
        if (txBytesSent < totalSize && (config.spi->SR & SPI_SR_TXE))
        {
            if (txBytesSent < txData.size())
            {
                // Send actual application data.
                config.spi->DR = txData[txBytesSent];
            }
            else
            {
                // TX data exhausted. Continue generating clocks for RX.
                config.spi->DR = 0x00;
            }
            ++txBytesSent;
        }

        // RECEIVE
        // Every RXNE corresponds to one completed SPI byte.
        if (config.spi->SR & SPI_SR_RXNE)
        {
            const uint8_t receivedByte = static_cast<uint8_t>(config.spi->DR);

            // Discard command/header bytes when TX and RX lengths
            // represent different phases of one SPI transaction.
            if (rxBytesToSkip > 0)
            {
                --rxBytesToSkip;
            }
            else if (realRxLeft > 0)
            {
                *rxPtr++ = receivedByte;
                --realRxLeft;
            }
            ++rxClocksCounted;
        }

        // TIMEOUT
        if ((GetSysTick() - startTick) > timeoutMs)
        {
            m_state = SpiState::READY;
            return BareM_Status::TIMEOUT;
        }
    }

    // WAIT UNTIL THE PHYSICAL SPI SHIFT REGISTER IS FINISHED
    while (config.spi->SR & SPI_SR_BSY)
    {
        if ((GetSysTick() - startTick) > timeoutMs)
        {
            m_state = SpiState::READY;
            return BareM_Status::TIMEOUT;
        }
    }

    // TRANSFER COMPLETE
    m_state = SpiState::READY;
    return BareM_Status::OK;
}

extern SpiDriver spi1; // declaration of global instance of an SpiDriver object named spi1
// extern SpiDriver spi3; // declaration of global instance of an SpiDriver object named spi3


