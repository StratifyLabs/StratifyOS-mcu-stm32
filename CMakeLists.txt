cmake_minimum_required(VERSION 3.12)
cmsdk2_minimum_required(2.1.2)

project(StratifyOS-mcu-stm32
  LANGUAGES C ASM
  VERSION 4.2.2)


cmsdk2_check_version(
  NAME StratifyOS
  VERSION ${StratifyOS_VERSION}
  MINIMUM_VERSION 4.3.1)

set(${PROJECT_NAME}_VERSION ${PROJECT_VERSION} CACHE INTERNAL "StratifyOS mcu STM32 project version")

if(CMSDK_IS_ARM)

  if(NOT STM32_CONFIG_PATH)
    message(FATAL_ERROR "STM32_CONFIG_PATH needs to be set to config directory")
  endif()

  #Add sources to the project
  cmsdk_add_subdirectory(SOURCE_LIST ${CMAKE_CURRENT_SOURCE_DIR}/src)
  set(SHARED_SOURCELIST ${SOURCE_LIST}
    ${CMAKE_CURRENT_SOURCE_DIR}/include/stm32_arch.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/stm32_config.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/stm32_state_types.h
    ${CMAKE_CURRENT_SOURCE_DIR}/README.md)

  option(BUILD_STM32_ALL "Build All configurations" ON)

  option(BUILD_STM32F2 "Build All F2 configurations" OFF)
  option(BUILD_STM32F205xx "Build F205xx configuration" OFF)
  option(BUILD_STM32F207xx "Build F207xx configuration" OFF)
  option(BUILD_STM32F215xx "Build F215xx configuration" OFF)
  option(BUILD_STM32F217xx "Build F217xx configuration" OFF)

  option(BUILD_STM32F4 "Build All F4 configurations" OFF)
  option(BUILD_STM32F401xC "Build F401xC configuration" OFF)
  option(BUILD_STM32F401xE "Build F401xE configuration" OFF)
  option(BUILD_STM32F405 "Build F405 configuration" OFF)
  option(BUILD_STM32F411 "Build F411 configuration" OFF)
  option(BUILD_STM32F412 "Build F412 configuration" OFF)
  option(BUILD_STM32F413 "Build F413 configuration" OFF)
  option(BUILD_STM32F417 "Build F417 configuration" OFF)
  option(BUILD_STM32F429 "Build F429 configuration" OFF)
  option(BUILD_STM32F446 "Build F446 configuration" OFF)

  option(BUILD_STM32F7 "Build All F7 configurations" OFF)
  option(BUILD_STM32F722 "Build F722 configuration" OFF)
  option(BUILD_STM32F723 "Build F723 configuration" OFF)
  option(BUILD_STM32F746 "Build F746 configuration" OFF)
  option(BUILD_STM32F767 "Build F767 configuration" OFF)

  option(BUILD_STM32L4 "Build All L4 configurations" OFF)
  option(BUILD_STM32L475 "Build L475 configuration" OFF)
  option(BUILD_STM32L432 "Build L432 configuration" OFF)

  option(BUILD_STM32H7 "Build All H7 configurations" OFF)
  option(BUILD_STM32H735 "Build H735 configuration" OFF)
  option(BUILD_STM32H743 "Build H743 configuration" OFF)
  option(BUILD_STM32H750 "Build H750 configuration" OFF)

  function(add_targets FAMILY VARIANT VARIANT_DEFINE ARCH)

    file(GLOB FAMILY_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/${FAMILY}xx/*)
    file(GLOB VARIANT_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/${VARIANT}/*.c")
    file(GLOB VARIANT_HEADERS
      ${CMAKE_CURRENT_SOURCE_DIR}/include/*${VARIANT}.h
      ${CMAKE_CURRENT_SOURCE_DIR}/include/cmsis/${FAMILY}.h
      ${CMAKE_CURRENT_SOURCE_DIR}/include/cmsis/${VARIANT}.h)
    file(GLOB LDSCRIPT_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/ldscript/${FAMILY}*")

    set(SOURCELIST ${SHARED_SOURCELIST} ${FAMILY_SOURCES} ${VARIANT_SOURCES} ${VARIANT_HEADERS} ${LDSCRIPT_SOURCES})
    set(DEFINITIONS __${VARIANT} ${VARIANT_DEFINE}=1 MCU_ARCH_STM32)

    cmsdk2_add_library(
      NAME StratifyOS
      OPTION ${VARIANT}
      CONFIG release
      ARCH ${ARCH}
      TARGET RELEASE_TARGET
    )
    target_compile_definitions(${RELEASE_TARGET} PUBLIC ${DEFINITIONS})
    target_sources(${RELEASE_TARGET}
      PRIVATE
      ${SOURCELIST}
      ${STM32_CONFIG_PATH}/${FAMILY}xx_hal_conf.h
      )
    target_include_directories(${RELEASE_TARGET}
      PUBLIC
      $<INSTALL_INTERFACE:include/stm32>
      $<INSTALL_INTERFACE:include/stm32/cmsis>
      $<BUILD_INTERFACE:${STM32_CONFIG_PATH}>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/${FAMILY}xx>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/cmsis>
      )
    target_compile_options(
      ${RELEASE_TARGET}
      PUBLIC -Os
      PRIVATE -Wno-strict-aliasing)

    # create debug as a copy of release
    cmsdk2_add_library(
      NAME StratifyOS
      OPTION ${VARIANT}
      CONFIG debug
      ARCH ${ARCH}
      TARGET DEBUG_TARGET
    )
    cmsdk2_copy_target(
      SOURCE ${RELEASE_TARGET}
      DESTINATION ${DEBUG_TARGET})

    cmsdk2_library_add_dependencies(
      TARGET ${RELEASE_TARGET}
      DEPENDENCIES StratifyOS_interface)

    cmsdk2_library_add_dependencies(
      TARGET ${DEBUG_TARGET}
      DEPENDENCIES StratifyOS_interface)

  endfunction()

  if(BUILD_STM32F205xx OR BUILD_STM32F2 OR BUILD_STM32_ALL)
    add_targets(stm32f2 stm32f205xx STM32F205xx v7m)
  endif()

  if(BUILD_STM32F401xC OR BUILD_STM32F4 OR BUILD_STM32_ALL)
    add_targets(stm32f4 stm32f401xc STM32F401xC v7em_f4sh)
  endif()

  if(BUILD_STM32F401xE OR BUILD_STM32F4 OR BUILD_STM32_ALL)
    add_targets(stm32f4 stm32f401xe STM32F401xE v7em_f4sh)
  endif()

  if(BUILD_STM32F405 OR BUILD_STM32F4 OR BUILD_STM32_ALL)
    add_targets(stm32f4 stm32f405xx STM32F405xx v7em_f4sh)
  endif()

  if(BUILD_STM32F411 OR BUILD_STM32F4 OR BUILD_STM32_ALL)
    add_targets(stm32f4 stm32f411xe STM32F411xE v7em_f4sh)
  endif()

  if(BUILD_STM32F412 OR BUILD_STM32F4 OR BUILD_STM32_ALL)
    add_targets(stm32f4 stm32f412zx STM32F412Zx v7em_f4sh)
  endif()

  if(BUILD_STM32F413 OR BUILD_STM32F4 OR BUILD_STM32_ALL)
    add_targets(stm32f4 stm32f413xx STM32F413xx v7em_f4sh)
  endif()

  if(BUILD_STM32F417 OR BUILD_STM32F4 OR BUILD_STM32_ALL)
    add_targets(stm32f4 stm32f417xx STM32F417xx v7em_f4sh)
  endif()

  if(BUILD_STM32F429 OR BUILD_STM32F4 OR BUILD_STM32_ALL)
    add_targets(stm32f4 stm32f429xx STM32F429xx v7em_f4sh)
  endif()

  if(BUILD_STM32F446 OR BUILD_STM32F4 OR BUILD_STM32_ALL)
    add_targets(stm32f4 stm32f446xx STM32F446xx v7em_f4sh)
  endif()

  if(BUILD_STM32F722 OR BUILD_STM32F7 OR BUILD_STM32_ALL)
    add_targets(stm32f7 stm32f722xx STM32F722xx v7em_f5sh)
  endif()

  if(BUILD_STM32F723 OR BUILD_STM32F7 OR BUILD_STM32_ALL)
    add_targets(stm32f7 stm32f723xx STM32F723xx v7em_f5sh)
  endif()

  if(BUILD_STM32F746 OR BUILD_STM32F7 OR BUILD_STM32_ALL)
    add_targets(stm32f7 stm32f746xx STM32F746xx v7em_f5sh)
  endif()

  if(BUILD_STM32F767 OR BUILD_STM32F7 OR BUILD_STM32_ALL)
    add_targets(stm32f7 stm32f767xx STM32F767xx v7em_f5dh)
  endif()

  if(BUILD_STM32L475 OR BUILD_STM32L4 OR BUILD_STM32_ALL)
    add_targets(stm32l4 stm32l475xx STM32L475xx v7em_f4sh)
  endif()

  if(BUILD_STM32L432 OR BUILD_STM32L4 OR BUILD_STM32_ALL)
    add_targets(stm32l4 stm32l432xx STM32L432xx v7em_f4sh)
  endif()

  if(BUILD_STM32H743 OR BUILD_STM32H7 OR BUILD_STM32_ALL)
    add_targets(stm32h7 stm32h743xx STM32H743xx v7em_f5dh)
  endif()

  if(BUILD_STM32H735 OR BUILD_STM32H7 OR BUILD_STM32_ALL)
    add_targets(stm32h7 stm32h735xx STM32H735xx v7em_f5dh)
  endif()

  if(BUILD_STM32H750 OR BUILD_STM32H7 OR BUILD_STM32_ALL)
    add_targets(stm32h7 stm32h750xx STM32H750xx v7em_f5dh)
  endif()

  file(GLOB STM32F2XX_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/src/stm32f2xx/*.h)
  file(GLOB STM32F3XX_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/src/stm32f3xx/*.h)
  file(GLOB STM32L4XX_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/src/stm32l4xx/*.h)
  file(GLOB STM32F4XX_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/src/stm32f4xx/*.h)
  file(GLOB STM32F7XX_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/src/stm32f7xx/*.h)
  file(GLOB STM32H7XX_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/src/stm32h7xx/*.h)

  install(FILES ${STM32F2XX_HEADERS} DESTINATION include/stm32/stm32f2xx)
  install(FILES ${STM32F3XX_HEADERS} DESTINATION include/stm32/stm32f3xx)
  install(FILES ${STM32L4XX_HEADERS} DESTINATION include/stm32/stm32l4xx)
  install(FILES ${STM32F4XX_HEADERS} DESTINATION include/stm32/stm32f4xx)
  install(FILES ${STM32F7XX_HEADERS} DESTINATION include/stm32/stm32f7xx)
  install(FILES ${STM32H7XX_HEADERS} DESTINATION include/stm32/stm32h7xx)
  install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/stm32_types.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/stm32_types_conf.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/stm32_dma.h
    DESTINATION
    include/stm32)
  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/stm32f2xx/Legacy/stm32_hal_legacy.h DESTINATION include/stm32/stm32f2xx/Legacy)
  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/stm32h7xx/Legacy/stm32_hal_legacy.h DESTINATION include/stm32/stm32h7xx/Legacy)
  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/stm32f7xx/Legacy/stm32_hal_legacy.h DESTINATION include/stm32/stm32f7xx/Legacy)
  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/stm32l4xx/Legacy/stm32_hal_legacy.h DESTINATION include/stm32/stm32l4xx/Legacy)

  install(DIRECTORY include/ DESTINATION include/stm32/)
  install(DIRECTORY ldscript/ DESTINATION lib/ldscripts)

else()
  # no targets for non arm
endif()

#/Users/tgil/gitv4/StratifyOS-Nucleo144/SDK/local/bin/arm-none-eabi-gcc -DCMSDK_BUILD_GIT_HASH=09ce5a4 -DMCU_ARCH_STM32 -DSTM32F446xx=1 -D__StratifyOS__ -D___release -D__stm32f446xx -D__v7em_f4sh -I/Users/tgil/gitv4/StratifyOS-Nucleo144/SDK/local/arm-none-eabi/include/StratifyOS -I/Users/tgil/gitv4/StratifyOS-Nucleo144/SDK/config -I/Users/tgil/gitv4/StratifyOS-Nucleo144/SDK/dependencies/StratifyOS-mcu-stm32/include -I/Users/tgil/gitv4/StratifyOS-Nucleo144/SDK/dependencies/StratifyOS-mcu-stm32/src -I/Users/tgil/gitv4/StratifyOS-Nucleo144/SDK/dependencies/StratifyOS-mcu-stm32/src/stm32f4xx -I/Users/tgil/gitv4/StratifyOS-Nucleo144/SDK/dependencies/StratifyOS-mcu-stm32/include/cmsis -mthumb -ffunction-sections -fdata-sections -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16 -U__SOFTFP__ -D__FPU_PRESENT=1 -DARM_MATH_CM4=1 -Os -Wno-strict-aliasing -MD -MT SDK/StratifyOS-mcu-stm32/CMakeFiles/StratifyOS_stm32f446xx_release_v7em_f4sh.dir/src/crc/crc_dev.c.obj -MF SDK/StratifyOS-mcu-stm32/CMakeFiles/StratifyOS_stm32f446xx_release_v7em_f4sh.dir/src/crc/crc_dev.c.obj.d -o SDK/StratifyOS-mcu-stm32/CMakeFiles/StratifyOS_stm32f446xx_release_v7em_f4sh.dir/src/crc/crc_dev.c.obj -c /Users/tgil/gitv4/StratifyOS-Nucleo144/SDK/dependencies/StratifyOS-mcu-stm32/src/crc/crc_dev.c