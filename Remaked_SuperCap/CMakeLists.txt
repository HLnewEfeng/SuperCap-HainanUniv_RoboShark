# THIS FILE IS AUTO GENERATED FROM THE TEMPLATE! DO NOT CHANGE!
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.22)

# ======================
# Toolchain
# ======================
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# ======================
# Project
# ======================
project(Remaked_SuperCap C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# ======================
# CPU
# ======================
add_compile_options(
        -mcpu=cortex-m4
        -mthumb
        -mthumb-interwork
)

add_link_options(
        -mcpu=cortex-m4
        -mthumb
        -mthumb-interwork
)

# ======================
# Optimization（核心）
# ======================
if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Release: balance performance + debuggability")
    add_compile_options(-O2 -g1)

elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "RelWithDebInfo: recommended mode")
    add_compile_options(-Og -g3 -ggdb -gdwarf-4)

elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "MinSizeRel: size optimized")
    add_compile_options(-Os -g1)

else ()
    message(STATUS "Debug: full debug")
    add_compile_options(-Og -g3 -ggdb -gdwarf-4)
endif ()

# 🚨 绝对不要用 Ofast（会破坏 LinkScope + CAN稳定性）
# add_compile_options(-Ofast)  ← 禁止

# ======================
# Code size optimization
# ======================
add_compile_options(
        -ffunction-sections
        -fdata-sections
        -fno-common
        -fmessage-length=0
)

add_link_options(
        -Wl,-gc-sections
)

# ======================
# ASM support
# ======================
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

# ======================
# Defines
# ======================
add_definitions(
        -DUSE_HAL_DRIVER
        -DSTM32G431xx
        -DDEBUG
)

# ======================
# Includes
# ======================
include_directories(
        Core/Inc
        Drivers/STM32G4xx_HAL_Driver/Inc
        Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
        Drivers/CMSIS/Device/ST/STM32G4xx/Include
        Drivers/CMSIS/Include
)

# ======================
# Sources
# ======================
file(GLOB_RECURSE SOURCES
        "Core/*.c"
        "Core/*.cpp"
        "Core/*.s"
        "Drivers/*.c"
        "Drivers/*.cpp"
)

# ======================
# Linker script
# ======================
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32G431CBTX_FLASH.ld)

add_link_options(
        -T ${LINKER_SCRIPT}
        -Wl,--print-memory-usage
        -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map
)

# ======================
# Executable
# ======================
add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

# ======================
# Output files
# ======================
set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMAND ${SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
        COMMENT "Generating HEX/BIN and printing size"
)