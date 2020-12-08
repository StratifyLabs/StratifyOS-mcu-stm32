#ifndef MCU_ARCH_STM32_STM32_DEV_LOCAL_H
#define MCU_ARCH_STM32_STM32_DEV_LOCAL_H

#include <device/fifo.h>
#include <sos/dev/i2c.h>

#include "stm32_dev_local_conf.h"

// local types

#if MCU_ADC_PORTS > 0
typedef struct {
  ADC_HandleTypeDef hal_handle;
  devfs_transfer_handler_t transfer_handler;
  int words_read;
  u32 o_flags;
  u8 ref_count;
} adc_state_t;
#endif

#if MCU_CRC_PORTS > 0
typedef struct {
  CRC_HandleTypeDef hal_handle;
  u32 value;
  u8 ref_count;
} crc_state_t;
#endif

#if MCU_CRYPT_PORTS > 0

#define CRYPT_MAX_KEY_SIZE 32
#define CRYPT_MAX_IV_SIZE 16
#define CRYPT_MAX_HEADER_SIZE 64

typedef struct {
  CRYP_HandleTypeDef hal_handle;
  devfs_transfer_handler_t transfer_handler;
  u32 o_flags;
  u8 iv[CRYPT_MAX_IV_SIZE];
  u8 header[CRYPT_MAX_HEADER_SIZE];
  u8 ref_count;
} crypt_state_t;
#endif

#if MCU_DAC_PORTS > 0
typedef struct {
  DAC_HandleTypeDef hal_handle;
  devfs_transfer_handler_t transfer_handler;
  int words_written;
  u32 o_flags;
  u8 ref_count;
  // stm32_dma_channel_t dma_tx_channel;
} dac_state_t;
#endif

#if MCU_HASH_PORTS > 0

#define HASH_MAX_KEY_SIZE 32
#define HASH_MAX_IV_SIZE 32
#define HASH_MAX_HEADER_SIZE 64

typedef struct {
  HASH_HandleTypeDef hal_handle;
  devfs_transfer_handler_t transfer_handler;
  u32 o_flags;
  u8 key[HASH_MAX_KEY_SIZE];
  u8 iv[HASH_MAX_IV_SIZE];
  u8 header[HASH_MAX_HEADER_SIZE];
  u8 ref_count;
} hash_state_t;
#endif

#if MCU_I2C_PORTS > 0
typedef struct MCU_PACK {
  I2C_HandleTypeDef hal_handle;
  devfs_transfer_handler_t transfer_handler;
  void *slave_memory;
  u32 slave_memory_offset;
  u32 slave_memory_size;
  u32 o_flags;
  u16 slave_addr[2];
  u16 err;
  u16 ref_count;
  i2c_pin_assignment_t
    pin_assignment; // this is only needed because of the errata
} i2c_state_t;
#endif

#if MCU_SDIO_PORTS > 0 && defined MMC_TypeDef
typedef struct {
  MMC_HandleTypeDef hal_handle; // must be the first member of the struct
  devfs_transfer_handler_t transfer_handler;
  u32 o_flags;
  u8 ref_count;
} mmc_state_t;
#endif

#if MCU_QSPI_PORTS > 0
typedef struct {
  QSPI_HandleTypeDef hal_handle;
  devfs_transfer_handler_t transfer_handler;
  u32 state;
  u32 ref_count;
#if MCU_QSPI_API == 1
  MDMA_HandleTypeDef hmdma;
#endif
} qspi_state_t;
#endif

#if MCU_SAI_PORTS > 0
typedef struct {
  SAI_HandleTypeDef hal_handle;
  devfs_transfer_handler_t transfer_handler;
  mcu_event_handler_t special_event_handler;
  u16 o_flags;
  u8 ref_count;
  u8 size_mult;
} sai_state_t;
#endif

#if MCU_RNG_PORTS > 0

typedef struct {
  RNG_HandleTypeDef hal_handle;
  devfs_transfer_handler_t transfer_handler;
  u32 bytes_read;
  u8 ref_count;
} rng_state_t;
#endif

#if MCU_RTC_PORTS > 0
typedef struct {
  RTC_HandleTypeDef hal_handle; // must be the first member of the struct
  mcu_event_handler_t alarm_handler;
  mcu_event_handler_t count_handler;
  u8 ref_count;
} rtc_state_t;
#endif

#if MCU_SDIO_PORTS > 0
typedef struct {
  SD_HandleTypeDef hal_handle; // must be the first member of the struct
  devfs_transfer_handler_t transfer_handler;
  u32 o_flags;
  u8 ref_count;
} sdio_state_t;
#endif

#if MCU_SPI_PORTS > 0
typedef struct {
  union { // must be first
    SPI_HandleTypeDef hal_handle;
#if MCU_I2S_SPI_PORTS > 0
    I2S_HandleTypeDef i2s_hal_handle;
#endif
  };
  devfs_transfer_handler_t transfer_handler;
  mcu_pin_t ws_pin;
  u16 o_flags;
  u8 ref_count;
  u8 size_mult;
} spi_state_t;
#endif

#if MCU_TMR_PORTS > 0
typedef struct {
  TIM_HandleTypeDef hal_handle; // must be first
  mcu_event_handler_t handler[MCU_TMR_CHANNELS * 2];
  mcu_event_handler_t period_handler;
  u8 ref_count;
} tmr_state_t;
#endif

#if MCU_UART_PORTS > 0
typedef struct {
  UART_HandleTypeDef hal_handle;
  devfs_transfer_handler_t transfer_handler;
  fifo_state_t fifo_state;
  const fifo_config_t *fifo_config;
  u8 ref_count;
  u8 o_flags;
} uart_state_t;
#endif

typedef struct {
  u8 ref_count;
} pio_state_t;

#if MCU_USB_PORTS > 0
typedef struct {
  PCD_HandleTypeDef hal_handle;
  devfs_transfer_handler_t transfer_handler[MCU_USB_ENDPOINT_COUNT];
  mcu_event_handler_t control_handler;
  u16 rx_count[MCU_USB_ENDPOINT_COUNT];
  u16 rx_buffer_offset[MCU_USB_ENDPOINT_COUNT];
  u16 rx_buffer_used;
  volatile u32 write_pending;
  volatile u32 read_ready;
  mcu_event_handler_t special_event_handler;
  u8 ref_count;
  u8 connected;
} usb_state_t;
#endif

#endif // MCU_ARCH_STM32_STM32_DEV_LOCAL_H
