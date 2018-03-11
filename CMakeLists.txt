cmake_minimum_required (VERSION 3.6)


#Add sources to the project
set(SOURCES_PREFIX ${CMAKE_SOURCE_DIR}/src)
add_subdirectory(src)
file(GLOB LDSCRIPT_SOURCES "${CMAKE_SOURCE_DIR}/ldscript/*")
set(SHARED_SOURCELIST ${SOURCES} ${LDSCRIPT_SOURCES} ${CMAKE_SOURCE_DIR}/include/stm32_arch.h ${CMAKE_SOURCE_DIR}/README.md)

if( ${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Darwin" )
  set(SOS_TOOLCHAIN_CMAKE_PATH /Applications/StratifyLabs-SDK/Tools/gcc/arm-none-eabi/cmake)
endif()
if( ${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Windows" )
  set(SOS_TOOLCHAIN_CMAKE_PATH C:/StratifyLabs-SDK/Tools/gcc/arm-none-eabi/cmake)
endif()
set(CMAKE_TOOLCHAIN_FILE ${SOS_TOOLCHAIN_CMAKE_PATH}/sos-lib-toolchain.cmake)
set(SOS_LIB_NAME sos)
project(${SOS_LIB_NAME} C ASM)

install(DIRECTORY include/ DESTINATION include/mcu/arch/stm32)

#[[
set(STM32_VARIANT stm32f446xx)
set(STM32_VARIANT_DEFINE STM32F446xx)
include(${CMAKE_SOURCE_DIR}/stm32f4.cmake)

set(STM32_VARIANT stm32f401xc)
set(STM32_VARIANT_DEFINE STM32F401xC)
include(${CMAKE_SOURCE_DIR}/stm32f4.cmake)

set(STM32_VARIANT stm32f411xe)
set(STM32_VARIANT_DEFINE STM32F411xE)
include(${CMAKE_SOURCE_DIR}/stm32f4.cmake)

set(STM32_VARIANT stm32f412zx)
set(STM32_VARIANT_DEFINE STM32F412Zx)
include(${CMAKE_SOURCE_DIR}/stm32f4.cmake)

set(STM32_VARIANT stm32f417xx)
set(STM32_VARIANT_DEFINE STM32F417xx)
include(${CMAKE_SOURCE_DIR}/stm32f4.cmake)

set(STM32_VARIANT stm32f429xx)
set(STM32_VARIANT_DEFINE STM32F429xx)
include(${CMAKE_SOURCE_DIR}/stm32f4.cmake)
]]#

set(STM32_VARIANT stm32f723xx)
set(STM32_VARIANT_DEFINE STM32F723xx)
set(STM32_ARCH_FPU v7em_f5sh)
include(${CMAKE_SOURCE_DIR}/stm32f7.cmake)

set(STM32_VARIANT stm32f746xx)
set(STM32_VARIANT_DEFINE STM32F746xx)
set(STM32_ARCH_FPU v7em_f5sh)
include(${CMAKE_SOURCE_DIR}/stm32f7.cmake)

set(STM32_VARIANT stm32f767xx)
set(STM32_VARIANT_DEFINE STM32F767xx)
set(STM32_ARCH_FPU v7em_f5dh)
include(${CMAKE_SOURCE_DIR}/stm32f7.cmake)

install(DIRECTORY ldscript/ DESTINATION lib/ldscripts)
