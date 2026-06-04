#include <stm32f469xx.h>

// If this header is being read by a C++ compiler, wrap the functions in extern "C"
#ifdef __cplusplus
extern "C" {
#endif

void SysClockConfig (void);

void activateFPU(void);

void GPIO_Config (void);

void InterruptGPIO_Config (void);

void USB_OTG_FS_Init(void);

#ifdef __cplusplus
}
#endif
