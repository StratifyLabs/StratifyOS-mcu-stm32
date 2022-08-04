
cmsdk_add_subdirectory(ADC_SOURCES adc)
cmsdk_add_subdirectory(CORE_SOURCES core)
cmsdk_add_subdirectory(CRC_SOURCES crc)
cmsdk_add_subdirectory(CRYPT_SOURCES crypt)
cmsdk_add_subdirectory(DAC_SOURCES dac)
cmsdk_add_subdirectory(ETH_SOURCES eth)
cmsdk_add_subdirectory(FLASH_SOURCES flash)
cmsdk_add_subdirectory(HASH_SOURCES hash)
cmsdk_add_subdirectory(I2C_SOURCES i2c)
cmsdk_add_subdirectory(I2S_SOURCES i2s)
cmsdk_add_subdirectory(MMC_SOURCES mmc)
cmsdk_add_subdirectory(PIO_SOURCES pio)
cmsdk_add_subdirectory(QSPI_SOURCES qspi)
cmsdk_add_subdirectory(RNG_SOURCES rng)
cmsdk_add_subdirectory(RTC_SOURCES rtc)
cmsdk_add_subdirectory(SAI_SOURCES sai)
cmsdk_add_subdirectory(SPI_SOURCES spi)
cmsdk_add_subdirectory(SDIO_SOURCES sdio)
cmsdk_add_subdirectory(TMR_SOURCES tmr)
cmsdk_add_subdirectory(UART_SOURCES uart)
cmsdk_add_subdirectory(USB_SOURCES usb)

set(SOURCES
	${ADC_SOURCES}
	${CORE_SOURCES}
	${CRC_SOURCES}
	${CRYPT_SOURCES}
	${DAC_SOURCES}
	${ETH_SOURCES}
	${FLASH_SOURCES}
	${HASH_SOURCES}
	${I2C_SOURCES}
	${I2S_SOURCES}
	${MMC_SOURCES}
	${PIO_SOURCES}
	${QSPI_SOURCES}
	${RNG_SOURCES}
	${RTC_SOURCES}
	${SAI_SOURCES}
	${SPI_SOURCES}
	${SDIO_SOURCES}
	${TMR_SOURCES}
	${UART_SOURCES}
	${USB_SOURCES}
	hal_set_alternate_pin_function.c
	fault_dev.c
	stm32_flash.c
	stm32_config.c
	stm32_dma.c
	stm32_dma.h
	stm32_mdma.c
	stm32_mdma.h
	stm32_local.c
	mcu_dev.c
	mem_dev.c
	mpu_dev.c
	wdt.c
	stm32_types.h
	stm32_types_conf.h
	stm32_local.h
	stm32_flash.h
	stm32_pin_local.h

	PARENT_SCOPE)
