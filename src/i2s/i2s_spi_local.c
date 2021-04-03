// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md

#include <mcu/i2s.h>
#include <mcu/spi.h>
#include <sos/config.h>

#include "i2s_spi_local.h"

#if MCU_I2S_SPI_PORTS > 0

int i2s_spi_local_open(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(spi);
  state->o_flags |= SPI_LOCAL_IS_I2S;
  return spi_local_open(handle);
}

int i2s_spi_local_close(const devfs_handle_t *handle) {
  return spi_local_close(handle);
}

int i2s_spi_local_mute(const devfs_handle_t *handle, void *ctl) {
  MCU_UNUSED_ARGUMENT(handle);
  MCU_UNUSED_ARGUMENT(ctl);
  return SYSFS_SET_RETURN(ENOTSUP);
}

int i2s_spi_local_unmute(const devfs_handle_t *handle, void *ctl) {
  MCU_UNUSED_ARGUMENT(handle);
  MCU_UNUSED_ARGUMENT(ctl);
  return SYSFS_SET_RETURN(ENOTSUP);
}

static void check_full_duplex(u32 o_flags, spi_state_t * state){
#if 0
  if (o_flags & I2S_FLAG_IS_FULL_DUPLEX) {
#if defined SPI_I2S_FULLDUPLEX_SUPPORT && defined I2S_FULLDUPLEXMODE_ENABLE
    state->i2s_hal_handle.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
#endif
#if defined I2S_MODE_SLAVE_FULLDUPLEX
    state->i2s_hal_handle.Init.Mode = I2S_MODE_SLAVE_FULLDUPLEX;
#endif
#if defined SPI_I2S_FULLDUPLEX_SUPPORT || defined STM32H7
    state->o_flags |= SPI_LOCAL_IS_FULL_DUPLEX;
#endif
  }
#endif
}

int i2s_spi_local_setattr(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(spi);
  const i2s_attr_t *attr = DEVFS_ASSIGN_ATTRIBUTES(i2s, ctl);
  if (attr == NULL) {
    return SYSFS_SET_RETURN(EINVAL);
  }

  u32 o_flags = attr->o_flags;

  // set I2S Flags

  if (o_flags & (I2S_FLAG_SET_MASTER | I2S_FLAG_SET_SLAVE)) {
#if defined SPI_I2S_FULLDUPLEX_SUPPORT
#if defined I2S_FULLDUPLEXMODE_DISABLE
    state->i2s_hal_handle.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
#endif
#endif

    if (o_flags & I2S_FLAG_SET_SLAVE) {
      state->o_flags |= SPI_LOCAL_IS_ERRATA_REQUIRED;
      if (o_flags & I2S_FLAG_IS_TRANSMITTER) {
        state->i2s_hal_handle.Init.Mode = I2S_MODE_SLAVE_TX;
        check_full_duplex(o_flags, state);
      } else if (o_flags & I2S_FLAG_IS_RECEIVER) {
        state->i2s_hal_handle.Init.Mode = I2S_MODE_SLAVE_RX;
        check_full_duplex(o_flags, state);
      }
    } else {

      if (o_flags & I2S_FLAG_IS_TRANSMITTER) {
        state->i2s_hal_handle.Init.Mode = I2S_MODE_MASTER_TX;
        check_full_duplex(o_flags, state);
      } else if (o_flags & I2S_FLAG_IS_RECEIVER) {
        state->i2s_hal_handle.Init.Mode = I2S_MODE_MASTER_RX;
        check_full_duplex(o_flags, state);
      }
    }

    state->i2s_hal_handle.Init.Standard = I2S_STANDARD_PHILIPS;
    if (o_flags & I2S_FLAG_IS_FORMAT_MSB) {
      state->i2s_hal_handle.Init.Standard = I2S_STANDARD_MSB;
    } else if (o_flags & I2S_FLAG_IS_FORMAT_LSB) {
      state->i2s_hal_handle.Init.Standard = I2S_STANDARD_LSB;
    } else if (o_flags & I2S_FLAG_IS_FORMAT_PCM_SHORT) {
      state->i2s_hal_handle.Init.Standard = I2S_STANDARD_PCM_SHORT;
    } else if (o_flags & I2S_FLAG_IS_FORMAT_PCM_LONG) {
      state->i2s_hal_handle.Init.Standard = I2S_STANDARD_PCM_LONG;
    } else {
      state->o_flags |= SPI_LOCAL_IS_ERRATA_I2S;
    }

    state->i2s_hal_handle.Init.DataFormat = I2S_DATAFORMAT_16B;
    state->size_mult = 2;
    if (o_flags & I2S_FLAG_IS_WIDTH_24) {
      state->i2s_hal_handle.Init.DataFormat = I2S_DATAFORMAT_24B;
      state->size_mult = 4;
    } else if (o_flags & I2S_FLAG_IS_WIDTH_32) {
      state->i2s_hal_handle.Init.DataFormat = I2S_DATAFORMAT_32B;
      state->size_mult = 4;
    } else if (o_flags & I2S_FLAG_IS_WIDTH_16_EXTENDED) {
      state->i2s_hal_handle.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
    }

    state->i2s_hal_handle.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
    if (o_flags & I2S_FLAG_IS_MCK_ENABLED) {
      state->i2s_hal_handle.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
    }

    state->i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_8K;
    switch (attr->freq) {
    case 11000:
      state->i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_11K;
      break;
    case 16000:
      state->i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_16K;
      break;
    case 22050:
      state->i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_22K;
      break;
    case 32000:
      state->i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_32K;
      break;
    case 44100:
      state->i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_44K;
      break;
    case 48000:
      state->i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_48K;
      break;
    case 96000:
      state->i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_96K;
      break;
    case 192000:
      state->i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_192K;
      break;
    default:
      return SYSFS_SET_RETURN(EINVAL);
    }

    state->i2s_hal_handle.Init.CPOL = I2S_CPOL_LOW;
#if defined I2S_CLOCK_PLL
    state->i2s_hal_handle.Init.ClockSource = I2S_CLOCK_PLL;
#endif

#if defined I2S_FIRSTBIT_MSB
    state->i2s_hal_handle.Init.FirstBit = I2S_FIRSTBIT_MSB;
#endif

#if defined I2S_WS_INVERSION_DISABLE
    state->i2s_hal_handle.Init.FirstBit = I2S_WS_INVERSION_DISABLE;
#endif

#if defined I2S_DATA_24BIT_ALIGNMENT_RIGHT
    state->i2s_hal_handle.Init.Data24BitAlignment
      = I2S_DATA_24BIT_ALIGNMENT_RIGHT;
#endif

#if defined I2S_MASTER_KEEP_IO_STATE_DISABLE
    state->i2s_hal_handle.Init.MasterKeepIOState
      = I2S_MASTER_KEEP_IO_STATE_DISABLE;
#endif

#if !defined STM32H7
    // errata:
    // http://www.st.com/content/ccc/resource/technical/document/errata_sheet/0a/98/58/84/86/b6/47/a2/DM00037591.pdf/files/DM00037591.pdf/jcr:content/translations/en.DM00037591.pdf
    if (state->o_flags & SPI_LOCAL_IS_ERRATA_REQUIRED) {
      state->ws_pin = attr->pin_assignment.ws;
      if (state->ws_pin.port == 0xff) {
        // try the other config
        if (handle->config) {
          const i2s_attr_t *config_attr = handle->config;
          state->ws_pin = config_attr->pin_assignment.ws;
        }
      }
    }
#endif

    if (
      mcu_set_pin_assignment(
        &(attr->pin_assignment),
        MCU_CONFIG_PIN_ASSIGNMENT(i2s_config_t, handle),
        MCU_PIN_ASSIGNMENT_COUNT(i2s_pin_assignment_t),
        CORE_PERIPH_SPI,
        config->port,
        0,
        0,
        0)
      < 0) {
      return SYSFS_SET_RETURN(EINVAL);
    }

    // cancel any ongoing transfers (if available)
    devfs_execute_cancel_handler(
      &state->transfer_handler,
      0,
      SYSFS_SET_RETURN(ECANCELED),
      0);

    int hal_result;
    if ((hal_result = HAL_I2S_Init(&state->i2s_hal_handle)) != HAL_OK) {
      sos_debug_log_error(SOS_DEBUG_DEVICE, "Init I2S failed %d", hal_result);
      return SYSFS_SET_RETURN(EIO);
    }
  }

  return 0;
}

void i2s_spi_local_wait_for_errata_level(spi_state_t *state) {

#if defined STM32H7
  return;
#endif

  if (state->o_flags & SPI_LOCAL_IS_ERRATA_REQUIRED) {
    u32 pio_level;
    u32 pio_mask;
    u32 target_level;
    pio_mask = 1 << state->ws_pin.pin;

    GPIO_TypeDef *gpio = hal_get_pio_regs(state->ws_pin.port);

    sos_debug_log_info(
      SOS_DEBUG_DEVICE,
      "execute I2S slave errata on %d.%d",
      state->ws_pin.port,
      state->ws_pin.pin);

    // errata:
    // http://www.st.com/content/ccc/resource/technical/document/errata_sheet/0a/98/58/84/86/b6/47/a2/DM00037591.pdf/files/DM00037591.pdf/jcr:content/translations/en.DM00037591.pdf

    // MSB Mode errata_required & (1<<0) == 0 -- wait until high then wait until
    // low I2S Mode errata_required & (1<<0) == 1 -- wait until low then wait
    // until high

    target_level
      = (state->o_flags & SPI_LOCAL_IS_ERRATA_I2S) == 0; // MSB : 1, I2S: 0
    do {
      pio_level = (HAL_GPIO_ReadPin(gpio, pio_mask) == GPIO_PIN_SET);
    } while (pio_level != target_level);

    target_level = !target_level; // MSB : 0, I2S: 1
    do {
      pio_level = (HAL_GPIO_ReadPin(gpio, pio_mask) == GPIO_PIN_SET);
    } while (pio_level != target_level);
  }
}

void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
  HAL_I2S_TxHalfCpltCallback(hi2s);
  HAL_I2S_RxHalfCpltCallback(hi2s);
}

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s) {
  HAL_I2S_TxCpltCallback(hi2s);
  HAL_I2S_RxCpltCallback(hi2s);
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
  // no action when half complete -- could fire an event
  spi_state_t *state = (spi_state_t *)hi2s;
  int result;
  devfs_async_t *async;

  async = state->transfer_handler.write;
  result = devfs_execute_write_handler(
    &state->transfer_handler,
    0,
    0,
    MCU_EVENT_FLAG_WRITE_COMPLETE | MCU_EVENT_FLAG_LOW);

#if 0 // This needs to be cleaned higher up when the buffer is written
	if( state->o_flags & SPI_LOCAL_IS_DMA && async ){
		sos_config.cache.clean_data_block(
					state->transfer_handler.read->buf,
					state->transfer_handler.read->nbyte/2
					);
	}
#endif

  if (result) {
    state->transfer_handler.write = async;
  } else {
    // stop -- half transfer only happens on DMA
    if (state->o_flags & SPI_LOCAL_IS_DMA) {
      HAL_I2S_DMAPause(hi2s);
    }
  }
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
  spi_state_t *state = (spi_state_t *)hi2s;
  int result;
  devfs_async_t *async;

  async = state->transfer_handler.write;
  result = devfs_execute_write_handler(
    &state->transfer_handler,
    0,
    0, // zero means leave nbyte value alone
    MCU_EVENT_FLAG_HIGH | MCU_EVENT_FLAG_WRITE_COMPLETE);

#if 0 // This needs to be cleaned higher up when the buffer is written
	if( state->o_flags & SPI_LOCAL_IS_DMA && async ){
		sos_config.cache.clean_data_block(
					((u8*)state->transfer_handler.read->buf) + state->transfer_handler.read->nbyte/2,
					state->transfer_handler.read->nbyte/2
					);
	}
#endif

  if (result) {
    state->transfer_handler.write = async;
  } else {
    // stop -- half transfer only happens on DMA
    if (state->o_flags & SPI_LOCAL_IS_DMA) {
      HAL_I2S_DMAPause(hi2s);
    }
  }
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
  spi_state_t *state = (spi_state_t *)hi2s;
  int result;
  devfs_async_t *async;

  async = state->transfer_handler.read;
  if (state->o_flags & SPI_LOCAL_IS_DMA && async) {

    sos_config.cache.invalidate_data_block(
      state->transfer_handler.read->buf,
      state->transfer_handler.read->nbyte / 2);
  }
  async = state->transfer_handler.read;
  result = devfs_execute_read_handler(
    &state->transfer_handler,
    0,
    0,
    MCU_EVENT_FLAG_DATA_READY | MCU_EVENT_FLAG_LOW);

  if (result) {
    state->transfer_handler.read = async;
  } else {
    // stop -- half transfer only happens on DMA
    HAL_I2S_DMAPause(hi2s);
  }
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
  spi_state_t *state = (spi_state_t *)hi2s;
  int result;
  devfs_async_t *async;

  async = state->transfer_handler.read;
  if (state->o_flags & SPI_LOCAL_IS_DMA && async) {
    sos_config.cache.invalidate_data_block(
      ((u8 *)state->transfer_handler.read->buf)
        + state->transfer_handler.read->nbyte / 2,
      state->transfer_handler.read->nbyte / 2);
  }
  result = devfs_execute_read_handler(
    &state->transfer_handler,
    0,
    0,
    MCU_EVENT_FLAG_DATA_READY | MCU_EVENT_FLAG_HIGH);

  if (result) {
    state->transfer_handler.read = async;
  } else if (state->o_flags & SPI_LOCAL_IS_DMA) {
    HAL_I2S_DMAPause(hi2s);
  }
}

void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s) {
  // called on overflow and underrun
  spi_state_t *spi = (spi_state_t *)hi2s;
  volatile u32 status = hi2s->Instance->SR;
#if defined STM32H7
  status = hi2s->Instance->UDRDR;
#else
  status = hi2s->Instance->DR;
#endif
  sos_debug_log_error(
    SOS_DEBUG_DEVICE,
    "I2S Error %d on %p",
    hi2s->ErrorCode,
    hi2s->Instance);
  devfs_execute_cancel_handler(
    &spi->transfer_handler,
    (void *)&status,
    SYSFS_SET_RETURN(EIO),
    MCU_EVENT_FLAG_ERROR);
}

#endif
