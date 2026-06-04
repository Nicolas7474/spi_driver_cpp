#ifndef SYS_TICK_H_
#define SYS_TICK_H_

#include <stm32f4xx.h>
#include <stm32f469xx.h>

// If this header is being read by a C++ compiler, wrap the functions in extern "C"
#ifdef __cplusplus
extern "C" {
#endif

void SysTickDelayMs(int delay);

void Delay_us_TIM7(uint16_t us);

void NBdelay_ms(uint32_t ms);

extern volatile uint32_t msTicks;

extern unsigned int countWakeUp;
extern int flagmsTicks;  // extern


void SysTick_Init();
// void SysTick_Handler(void); // defined in CMSIS
uint32_t GetSysTick(void);

#ifdef __cplusplus
}
#endif

#endif /* SYS_TICK_H_ */
