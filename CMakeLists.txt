cmake_minimum_required (VERSION 3.6)


#Add sources to the project
set(SOURCES_PREFIX ${CMAKE_SOURCE_DIR}/src)
add_subdirectory(src)
set(SHARED_SOURCELIST ${SOURCES})
list(APPEND SOS_LIB_SOURCELIST ${SOURCES})

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

set(SOS_LIB_INCLUDE_DIRECTORIES include src stm32f4xx include/cmsis)

file(GLOB STM32F4XX_SOURCES "${CMAKE_SOURCE_DIR}/src/stm32f4xx/*.c" "${CMAKE_SOURCE_DIR}/src/stm32f4xx/*.h")
file(GLOB STM32F446_SOURCES "${CMAKE_SOURCE_DIR}/src/stm32f446xx/*.c")
set(SOS_LIB_SOURCELIST ${SHARED_SOURCELIST} ${STM32F4XX_SOURCES} ${STM32F446_SOURCES})

set(SOS_LIB_OPTION mcu_stm32f446xx)
set(SOS_LIB_DEFINITIONS __stm32f446xx)
set(SOS_LIB_ARCH armv7e-m)
set(SOS_LIB_TYPE release)
include(${SOS_TOOLCHAIN_CMAKE_PATH}/sos-lib.cmake)
set(SOS_LIB_TYPE debug)
include(${SOS_TOOLCHAIN_CMAKE_PATH}/sos-lib.cmake)
