################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Src/main.cpp \
../Src/spi1.cpp 

C_SRCS += \
../Src/myConfig.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/timers.c 

C_DEPS += \
./Src/myConfig.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/timers.d 

OBJS += \
./Src/main.o \
./Src/myConfig.o \
./Src/spi1.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/timers.o 

CPP_DEPS += \
./Src/main.d \
./Src/spi1.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.cpp Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++20 -g3 -DDEBUG -DSTM32 -DSTM32F469NIHx -DSTM32F4 -c -I../Inc -O2 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F469NIHx -DSTM32F4 -c -I../Inc -O2 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/myConfig.cyclo ./Src/myConfig.d ./Src/myConfig.o ./Src/myConfig.su ./Src/spi1.cyclo ./Src/spi1.d ./Src/spi1.o ./Src/spi1.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/timers.cyclo ./Src/timers.d ./Src/timers.o ./Src/timers.su

.PHONY: clean-Src

