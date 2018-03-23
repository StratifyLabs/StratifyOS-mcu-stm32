


set(SOURCES
  ${SOURCES_PREFIX}/core_dev.c
  ${SOURCES_PREFIX}/core_setusbclock.c
  ${SOURCES_PREFIX}/core_sleep.c
  ${SOURCES_PREFIX}/core_startup.c
  ${SOURCES_PREFIX}/fault_dev.c
  ${SOURCES_PREFIX}/flash_dev.c
	${SOURCES_PREFIX}/i2c_dev.c
	${SOURCES_PREFIX}/i2s_spi_dev.c
	${SOURCES_PREFIX}/stm32_flash.c
	${SOURCES_PREFIX}/stm32_local.c
	${SOURCES_PREFIX}/mcu_dev.c
  ${SOURCES_PREFIX}/mem_dev.c
  ${SOURCES_PREFIX}/mpu_dev.c
  ${SOURCES_PREFIX}/pio_dev.c
  ${SOURCES_PREFIX}/spi_dev.c
  ${SOURCES_PREFIX}/tmr_dev.c
  ${SOURCES_PREFIX}/uart_dev.c
  ${SOURCES_PREFIX}/usb_otg_fs_dev.c
  ${SOURCES_PREFIX}/wdt.c
	${SOURCES_PREFIX}/stm32_local.h
	${SOURCES_PREFIX}/stm32_flash.h
	${SOURCES_PREFIX}/stm32_pin_local.h
	${SOURCES_PREFIX}/spi_local.h
	PARENT_SCOPE)
