cmake_minimum_required(VERSION 3.20)

# The project definition (C, C++ and Assembly)
project(spi_driver_cpp C CXX ASM)

# Enable modern C++
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Extra bare-metal C++ optimizations (removes heavy desktop memory overhead)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-rtti -fno-exceptions -Og -g")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Og -g")

# Linker flags pointing to your exact .ld file inside your project folder
# Make sure this filename matches your physical STM32F469 Linker Script!
set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/STM32F469NIHX_FLASH.ld") 
set(CMAKE_EXE_LINKER_FLAGS "${MCU_FLAGS} -T\"${LINKER_SCRIPT}\" -Wl,-Map=${PROJECT_NAME}.map -Wl,--gc-sections --specs=nano.specs")

# Collect Source Files (recursively find CubeIDE files)
file(GLOB_RECURSE SOURCES 
    "Src/*.cpp"
    "Src/*.c" 
    "Startup/*.s"
)

list(REMOVE_ITEM SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/Src/Flash_W25Q.cpp")

# Define Executable
add_executable(${PROJECT_NAME} ${SOURCES})

# FORCE the output file to have the .elf extension!
set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX ".elf")

# Specify Include Directories (-I) so compiler finds main.h
target_include_directories(${PROJECT_NAME} PRIVATE
    "Inc"
    "drivers/CMSIS"            # The new home for core_cm4.h, etc.
    "drivers/CMSIS_Device"     # The new home for stm32f4xx.h, etc.
)

# Flash & Reset Automations via Ninja
add_custom_target(flash
    COMMAND "C:/Program Files/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe" -c port=SWD -w ${PROJECT_NAME}.elf -v -rst
    DEPENDS ${PROJECT_NAME}
    USES_TERMINAL
)

add_custom_target(reset
    COMMAND "C:/Program Files/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe" -c port=SWD -rst
    USES_TERMINAL
)
