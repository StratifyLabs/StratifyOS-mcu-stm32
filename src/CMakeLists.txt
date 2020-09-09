
sos_sdk_add_subdirectory(ADC_SOURCES adc)
sos_sdk_add_subdirectory(CORE_SOURCES core)
sos_sdk_add_subdirectory(CRC_SOURCES crc)
sos_sdk_add_subdirectory(CRYPT_SOURCES crypt)
sos_sdk_add_subdirectory(DAC_SOURCES dac)
sos_sdk_add_subdirectory(ETH_SOURCES eth)
sos_sdk_add_subdirectory(FLASH_SOURCES flash)
sos_sdk_add_subdirectory(HASH_SOURCES hash)
sos_sdk_add_subdirectory(I2C_SOURCES i2c)
sos_sdk_add_subdirectory(I2S_SOURCES i2s)
sos_sdk_add_subdirectory(MMC_SOURCES mmc)
sos_sdk_add_subdirectory(PIO_SOURCES pio)
sos_sdk_add_subdirectory(QSPI_SOURCES qspi)
sos_sdk_add_subdirectory(RNG_SOURCES rng)
sos_sdk_add_subdirectory(RTC_SOURCES rtc)
sos_sdk_add_subdirectory(SAI_SOURCES sai)
sos_sdk_add_subdirectory(SPI_SOURCES spi)
sos_sdk_add_subdirectory(SDIO_SOURCES sdio)
sos_sdk_add_subdirectory(TMR_SOURCES tmr)
sos_sdk_add_subdirectory(UART_SOURCES uart)
sos_sdk_add_subdirectory(USB_SOURCES usb)

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
	stm32_dma.c
	stm32_dma.h
	stm32_mdma.c
	stm32_mdma.h
	stm32_local.c
	mcu_dev.c
	mem_dev.c
	mpu_dev.c
	wdt.c
	stm32_dev_local.h
	stm32_dev_local_conf.h
	stm32_local.h
	stm32_flash.h
	stm32_pin_local.h

	PARENT_SCOPE)
