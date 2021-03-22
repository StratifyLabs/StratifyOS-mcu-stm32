// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md

#include "cortexm/cortexm.h"
#include "mcu/core.h"
#include "sos/debug.h"
#include "mcu/i2c.h"
#include "mcu/pio.h"
#include "stm32_local.h"
#include <errno.h>
#include <fcntl.h>

#if MCU_I2C_PORTS > 0

static i2c_state_t *m_i2c_state_list[MCU_I2C_PORTS] MCU_SYS_MEM;
static I2C_TypeDef *const i2c_regs_table[MCU_I2C_PORTS] = MCU_I2C_REGS;
static u8 const i2c_irqs[MCU_I2C_PORTS] = MCU_I2C_IRQS;
static u8 const i2c_er_irqs[MCU_I2C_PORTS] = MCU_I2C_ER_IRQS;

static void i2c_clear_busy_flag_erratum(int port, i2c_state_t *i2c);

#if 0
typedef struct {
	u8 port;
	u8 is_pullup;
} post_configure_pin_t;
static void post_configure_pin(const mcu_pin_t * pin, void* arg);
#endif

DEVFS_MCU_DRIVER_IOCTL_FUNCTION_MIN(i2c, I2C_VERSION, I2C_IOC_IDENT_CHAR)

int mcu_i2c_open(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(i2c);
  if (state->ref_count == 0) {

    DEVFS_DRIVER_OPEN_STATE_LOCAL_V4(i2c);
    switch (config->port) {
    case 0:
      __HAL_RCC_I2C1_CLK_ENABLE();
      break;
#if defined I2C2

    case 1:
      __HAL_RCC_I2C2_CLK_ENABLE();
      break;
#endif
#if defined I2C3
    case 2:
      __HAL_RCC_I2C3_CLK_ENABLE();
      break;
#endif
    }
    state->transfer_handler.read = 0;
    state->transfer_handler.write = 0;
    state->hal_handle.Instance = i2c_regs_table[config->port];
    cortexm_enable_irq(i2c_irqs[config->port]);
    cortexm_enable_irq(i2c_er_irqs[config->port]);
  }
  state->ref_count++;
  return 0;
}

int mcu_i2c_close(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(i2c);

  if (state->ref_count > 0) {
    if (state->ref_count == 1) {
      devfs_execute_cancel_handler(
        &state->transfer_handler,
        0,
        SYSFS_SET_RETURN(EINTR),
        0);
      state->hal_handle.Instance = NULL;
      cortexm_disable_irq(i2c_irqs[config->port]);
      cortexm_disable_irq(i2c_er_irqs[config->port]);
      switch (config->port) {
      case 0:
        __HAL_RCC_I2C1_CLK_DISABLE();
        break;
#if defined I2C2
      case 1:
        __HAL_RCC_I2C2_CLK_DISABLE();
        break;
#endif
#if defined I2C3
      case 2:
        __HAL_RCC_I2C3_CLK_DISABLE();
        break;
#endif
      }
      DEVFS_DRIVER_CLOSE_STATE_LOCAL_V4(i2c);
    }
    state->ref_count--;
  }
  return 0;
}

int mcu_i2c_getinfo(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(i2c);
  i2c_info_t *info = ctl;

  info->err = state->err;
  info->o_flags = I2C_FLAG_SET_MASTER | I2C_FLAG_SET_SLAVE
                  | I2C_FLAG_PREPARE_PTR_DATA | I2C_FLAG_PREPARE_DATA
                  | I2C_FLAG_IS_PULLUP | I2C_FLAG_IS_SLAVE_ADDR0
                  | I2C_FLAG_IS_SLAVE_ADDR1 | I2C_FLAG_IS_SLAVE_ADDR2
                  | I2C_FLAG_IS_SLAVE_ADDR3 | I2C_FLAG_RESET;

  info->freq = 400000;

  info->o_events = MCU_EVENT_FLAG_WRITE_COMPLETE | MCU_EVENT_FLAG_DATA_READY
                   | MCU_EVENT_FLAG_CANCELED | MCU_EVENT_FLAG_ERROR;
  return 0;
}

int mcu_i2c_setattr(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(i2c);
  u32 freq;
  const i2c_attr_t *attr = DEVFS_ASSIGN_ATTRIBUTES(i2c, ctl);
  if (attr == 0) {
    return SYSFS_SET_RETURN(EINVAL);
  }
  freq = attr->freq;
  u32 o_flags = attr->o_flags;

  if (freq == 0) {
    freq = 100000;
  }

  if (o_flags & (I2C_FLAG_SET_MASTER | I2C_FLAG_SET_SLAVE)) {
#if 0
		post_configure_pin_t post_configure;
#endif
    const i2c_pin_assignment_t *pin_assignment;
    state->hal_handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    state->hal_handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    state->hal_handle.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
#if defined I2C_DUTYCYCLE_2
    state->hal_handle.Init.DutyCycle = I2C_DUTYCYCLE_2;
#endif
    state->hal_handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    state->hal_handle.Init.OwnAddress1 = 0;
    state->hal_handle.Init.OwnAddress2 = 0;
    if (o_flags & I2C_FLAG_SET_MASTER) {
#if defined STM32F7 || defined STM32L4 || defined STM32H7
      state->hal_handle.Init.Timing = freq;
#else
      state->hal_handle.Init.ClockSpeed = freq;
#endif
    } else if (o_flags & I2C_FLAG_SET_SLAVE) {
#if defined STM32F7 || defined STM32L4 || defined STM32H7
      state->hal_handle.Init.Timing = freq;
#else
      state->hal_handle.Init.ClockSpeed = freq;
#endif
      state->hal_handle.Init.OwnAddress1 = (attr->slave_addr[0].addr8[0]) << 1;
      if (o_flags & I2C_FLAG_IS_SLAVE_ADDR1) {
        state->hal_handle.Init.DualAddressMode = I2C_DUALADDRESS_ENABLE;
        state->hal_handle.Init.OwnAddress2 = (attr->slave_addr[1].addr8[0])
                                             << 1;
      }
      if (o_flags & I2C_FLAG_IS_SLAVE_ACK_GENERAL_CALL) {
        state->hal_handle.Init.GeneralCallMode = I2C_GENERALCALL_ENABLE;
      }
      // set slave memory location
      state->slave_memory_offset = 0;
      state->slave_memory = attr->data;
      state->slave_memory_size = attr->size;
    }
    if (o_flags & I2C_FLAG_STRETCH_CLOCK) {
      state->hal_handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    }

    if (freq < 100000) {
#if defined I2C_DUTYCYCLE_16_9
      state->hal_handle.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
#endif
    }

    state->o_flags = o_flags;
#if 0
		post_configure.port = port;
		if( o_flags & I2C_FLAG_IS_PULLUP ){
			post_configure.is_pullup = 1;
		} else {
			post_configure.is_pullup = 0;
		}
#endif

    // force a start and stop condition to clear the busy bit
    pin_assignment = mcu_select_pin_assignment(
      &attr->pin_assignment,
      MCU_CONFIG_PIN_ASSIGNMENT(i2c_config_t, handle),
      MCU_PIN_ASSIGNMENT_COUNT(i2c_pin_assignment_t));

    memcpy(
      &state->pin_assignment,
      pin_assignment,
      sizeof(i2c_pin_assignment_t));
    if (
      (state->pin_assignment.scl.port != 0xff)
      && (state->pin_assignment.sda.port != 0xff)) {
      int count = 0;
      do {
        if (__HAL_I2C_GET_FLAG((&state->hal_handle), I2C_FLAG_BUSY)) {
          sos_debug_log_info(SOS_DEBUG_DEVICE, "clear busy flag");
        } else {
          sos_debug_log_info(SOS_DEBUG_DEVICE, "I2C not busy");
        }
        i2c_clear_busy_flag_erratum(config->port, state);
        sos_debug_log_info(SOS_DEBUG_DEVICE, "done");
      } while ((__HAL_I2C_GET_FLAG((&state->hal_handle), I2C_FLAG_BUSY))
               && count++ < 5);

      if (__HAL_I2C_GET_FLAG((&state->hal_handle), I2C_FLAG_BUSY)) {
        return SYSFS_SET_RETURN(EIO);
      }

    } else {
      return SYSFS_SET_RETURN(EINVAL);
    }

    if (HAL_I2C_Init(&(state->hal_handle)) != HAL_OK) {
      return SYSFS_SET_RETURN(EINVAL);
    }

    if (o_flags & I2C_FLAG_SET_SLAVE) {
      if (HAL_I2C_EnableListen_IT(&state->hal_handle) != HAL_OK) {
        return SYSFS_SET_RETURN(EINVAL);
      }
    }
  }

  if (
    o_flags
    & (I2C_FLAG_PREPARE_PTR_DATA | I2C_FLAG_PREPARE_PTR | I2C_FLAG_PREPARE_DATA)) {
    state->o_flags = o_flags;
    state->slave_addr[0] = (attr->slave_addr[0].addr8[0]);
    state->slave_addr[1] = (attr->slave_addr[1].addr8[0]);
  }

  if (o_flags & I2C_FLAG_RESET) {
    // force a reset of the I2C
    i2c_clear_busy_flag_erratum(config->port, state);
    if (__HAL_I2C_GET_FLAG((&state->hal_handle), I2C_FLAG_BUSY)) {
      return SYSFS_SET_RETURN(EBUSY);
    }
  }

  return 0;
}

#if 0
void post_configure_pin(const mcu_pin_t * pin, void* arg){
	post_configure_pin_t * post_configure = arg;

	if( post_configure->is_pullup ){
		hal_set_alternate_pin_function(*pin,
												 CORE_PERIPH_I2C,
												 post_configure->port,
												 GPIO_MODE_AF_OD,
												 GPIO_SPEED_FREQ_LOW,
												 GPIO_PULLUP);
	}


}
#endif

int mcu_i2c_geterr(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(i2c);
  return state->err;
}

int mcu_i2c_setaction(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(i2c);
  mcu_action_t *action = (mcu_action_t *)ctl;

  cortexm_set_irq_priority(
    i2c_irqs[config->port],
    action->prio,
    action->o_events);

  if (action->handler.callback == 0) {
    // i2c_state_list[config->port].slave.handler.callback = 0;
    // i2c_state_list[config->port].slave.handler.context = 0;
    return 0;
  }

  if (cortexm_validate_callback(action->handler.callback) < 0) {
    return -1;
  }

  // i2c_state_list[config->port].slave.handler.callback =
  // action->handler.callback;
  // i2c_state_list[config->port].slave.handler.context =
  // action->handler.context;

  return 0;
}

int mcu_i2c_write(const devfs_handle_t *handle, devfs_async_t *async) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(i2c);
  int ret;
  int addr_size;

  DEVFS_DRIVER_IS_BUSY(state->transfer_handler.write, async);

  if (state->o_flags & I2C_FLAG_IS_PTR_16) {
    addr_size = I2C_MEMADD_SIZE_16BIT;
  } else {
    addr_size = I2C_MEMADD_SIZE_8BIT;
  }

  if (state->o_flags & I2C_FLAG_PREPARE_PTR_DATA) {
    ret = HAL_I2C_Mem_Write_IT(
      &state->hal_handle,
      (u16)(state->slave_addr[0] << 1),
      (u16)async->loc,
      (u16)addr_size,
      (u8 *)async->buf,
      (u16)async->nbyte);
  } else if (state->o_flags & I2C_FLAG_PREPARE_DATA) {
    ret = HAL_I2C_Master_Transmit_IT(
      &state->hal_handle,
      state->slave_addr[0] << 1,
      async->buf,
      async->nbyte);
  } else {
    ret = -1;
  }

  if (ret == HAL_OK) {
    return 0;
  } else {
    if (ret == HAL_TIMEOUT) {
      state->err = I2C_ERROR_TIMEOUT;
    }
    sos_debug_log_error(SOS_DEBUG_DEVICE, "I2C Write Error: %d", ret);
  }

  state->transfer_handler.write = 0;
  return SYSFS_SET_RETURN(EIO);
}

int mcu_i2c_read(const devfs_handle_t *handle, devfs_async_t *async) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(i2c);
  DEVFS_DRIVER_IS_BUSY(state->transfer_handler.read, async);

  int ret = HAL_OK;
  int addr_size;

  if (state->o_flags & I2C_FLAG_IS_PTR_16) {
    addr_size = I2C_MEMADD_SIZE_16BIT;
  } else {
    addr_size = I2C_MEMADD_SIZE_8BIT;
  }

  if (state->o_flags & I2C_FLAG_PREPARE_PTR_DATA) {
    ret = HAL_I2C_Master_Sequential_Transmit_IT(
      &state->hal_handle,
      (u16)state->slave_addr[0] << 1,
      (u8 *)&async->loc,
      (u16)addr_size,
      I2C_FIRST_FRAME);
  } else if (state->o_flags & I2C_FLAG_PREPARE_DATA) {
    ret = HAL_I2C_Master_Receive_IT(
      &state->hal_handle,
      state->slave_addr[0] << 1,
      async->buf,
      async->nbyte);
  }

  if (ret == HAL_OK) {
    return 0;
  } else {
    if (ret == HAL_TIMEOUT) {
      state->err = I2C_ERROR_TIMEOUT;
    }
    sos_debug_log_error(SOS_DEBUG_DEVICE, "I2C Read Error: %d", ret);
  }

  state->transfer_handler.read = 0;
  return SYSFS_SET_RETURN(EIO);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  i2c_state_t *state = (i2c_state_t *)hi2c;

  // sos_debug_log_info(SOS_DEBUG_DEVICE, "Tx complete");
  // is this a read operation to be continued?
  if (state->transfer_handler.read != 0) {
    if (
      HAL_I2C_Master_Sequential_Receive_IT(
        &state->hal_handle,
        state->slave_addr[0] << 1,
        state->transfer_handler.read->buf,
        state->transfer_handler.read->nbyte,
        I2C_LAST_FRAME)
      != HAL_OK) {
      devfs_execute_cancel_handler(
        &state->transfer_handler,
        0,
        0,
        MCU_EVENT_FLAG_ERROR);
    }

  } else {
    // TX complete
    devfs_execute_write_handler(
      &state->transfer_handler,
      0,
      0,
      MCU_EVENT_FLAG_WRITE_COMPLETE);
  }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  i2c_state_t *i2c = (i2c_state_t *)hi2c;

  // sos_debug_log_info(SOS_DEBUG_DEVICE, "Rx complete");

  devfs_execute_read_handler(
    &i2c->transfer_handler,
    0,
    0,
    MCU_EVENT_FLAG_DATA_READY);
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  i2c_state_t *state = (i2c_state_t *)hi2c;
  devfs_execute_write_handler(
    &state->transfer_handler,
    0,
    hi2c->XferSize,
    MCU_EVENT_FLAG_WRITE_COMPLETE);
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  i2c_state_t *state = (i2c_state_t *)hi2c;
  devfs_execute_read_handler(
    &state->transfer_handler,
    0,
    hi2c->XferSize,
    MCU_EVENT_FLAG_WRITE_COMPLETE);
}

void HAL_I2C_AddrCallback(
  I2C_HandleTypeDef *hi2c,
  uint8_t TransferDirection,
  uint16_t AddrMatchCode) {
  // slave has been addressed
  i2c_state_t *state = (i2c_state_t *)hi2c;
  HAL_StatusTypeDef hal_status;
  hal_status = HAL_I2C_Slave_Sequential_Transmit_IT(
    hi2c,
    state->slave_memory,
    state->slave_memory_size,
    I2C_LAST_FRAME);
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
  // listen event
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  i2c_state_t *state = (i2c_state_t *)hi2c;

  // sos_debug_log_info(SOS_DEBUG_DEVICE, "MEM Tx complete");

  // TX complete
  devfs_execute_write_handler(
    &state->transfer_handler,
    0,
    0,
    MCU_EVENT_FLAG_WRITE_COMPLETE);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  i2c_state_t *state = (i2c_state_t *)hi2c;
  // sos_debug_log_info(SOS_DEBUG_DEVICE, "MEM Rx complete");

  devfs_execute_read_handler(
    &state->transfer_handler,
    0,
    0,
    MCU_EVENT_FLAG_DATA_READY);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
  i2c_state_t *state = (i2c_state_t *)hi2c;
  if (hi2c->ErrorCode & HAL_I2C_ERROR_ARLO) {
    state->err = I2C_ERROR_ARBITRATION_LOST;
  } else if (hi2c->ErrorCode & HAL_I2C_ERROR_BERR) {
    state->err = I2C_ERROR_BUS_BUSY;
  } else if (hi2c->ErrorCode & HAL_I2C_ERROR_AF) {
    state->err = I2C_ERROR_ACK;
  } else if (hi2c->ErrorCode & HAL_I2C_ERROR_TIMEOUT) {
    state->err = I2C_ERROR_TIMEOUT;
  } else if (hi2c->ErrorCode & HAL_I2C_ERROR_OVR) {
    state->err = I2C_ERROR_OVERFLOW;
  }

  if (state->err != I2C_ERROR_ACK) {
    sos_debug_log_info(
      SOS_DEBUG_DEVICE,
      "Error %d (%d)",
      hi2c->Mode,
      hi2c->ErrorCode);
  }

  devfs_execute_cancel_handler(
    &state->transfer_handler,
    0,
    SYSFS_SET_RETURN(EIO),
    MCU_EVENT_FLAG_ERROR);
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c) {
  i2c_state_t *state = (i2c_state_t *)hi2c;

  // sos_debug_log_info(SOS_DEBUG_DEVICE, "Abort %d", hi2c->Mode);
  devfs_execute_cancel_handler(
    &state->transfer_handler,
    0,
    SYSFS_SET_RETURN(EINTR),
    0);
}

static void mcu_i2c_ev_isr(int port) {
  i2c_state_t *state = m_i2c_state_list[port];
  HAL_I2C_EV_IRQHandler(&state->hal_handle);
}

static void mcu_i2c_er_isr(int port) {
  i2c_state_t *state = m_i2c_state_list[port];
  HAL_I2C_ER_IRQHandler(&state->hal_handle);
}

void mcu_core_i2c1_ev_isr() { mcu_i2c_ev_isr(0); }
void mcu_core_i2c1_er_isr() { mcu_i2c_er_isr(0); }
#if MCU_I2C_PORTS > 1
void mcu_core_i2c2_ev_isr() { mcu_i2c_ev_isr(1); }
void mcu_core_i2c2_er_isr() { mcu_i2c_er_isr(1); }
#endif
#if MCU_I2C_PORTS > 2
void mcu_core_i2c3_ev_isr() { mcu_i2c_ev_isr(2); }
void mcu_core_i2c3_er_isr() { mcu_i2c_er_isr(2); }
#endif

#define I2C_CLEAR_BUSY_DEBUG_MESSAGES 0
#define I2C_WAIT_FOR_LINE_CHANGE 0

void i2c_clear_busy_flag_erratum(int port, i2c_state_t *i2c) {

  pio_attr_t scl_pio_attr;
  pio_attr_t sda_pio_attr;
  const u32 delay_us = 1;

#if I2C_WAIT_FOR_LINE_CHANGE
  u32 value;
#endif

  switch (port) {
  case 0:
    __HAL_RCC_I2C1_FORCE_RESET();
    cortexm_delay_us(1000);
    __HAL_RCC_I2C1_RELEASE_RESET();
    break;
#if defined I2C2
  case 1:
    __HAL_RCC_I2C2_FORCE_RESET();
    cortexm_delay_us(1000);
    __HAL_RCC_I2C2_RELEASE_RESET();
    break;
#endif
#if defined I2C3
  case 2:
    __HAL_RCC_I2C3_FORCE_RESET();
    cortexm_delay_us(1000);
    __HAL_RCC_I2C3_RELEASE_RESET();
    break;
#endif
  }

  const int sda_port = i2c->pin_assignment.sda.port;
  const int scl_port = i2c->pin_assignment.scl.port;
  // 1. Clear PE bit.
  i2c->hal_handle.Instance->CR1 &= ~(0x0001);

  //  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain,
  //  High level (Write 1 to GPIOx_ODR).
  scl_pio_attr.o_flags = PIO_FLAG_SET_OUTPUT | PIO_FLAG_IS_OPENDRAIN;
  scl_pio_attr.o_pinmask = (1 << i2c->pin_assignment.scl.pin);
  hal_pio_setattr(scl_port, &scl_pio_attr);

  sda_pio_attr.o_flags = PIO_FLAG_SET_OUTPUT | PIO_FLAG_IS_OPENDRAIN;
  sda_pio_attr.o_pinmask = (1 << i2c->pin_assignment.sda.pin);
  hal_pio_setattr(sda_port, &sda_pio_attr);

  hal_pio_setmask(scl_port, (void *)scl_pio_attr.o_pinmask);
  hal_pio_setmask(sda_port, (void *)sda_pio_attr.o_pinmask);

  // 3. Check SCL and SDA High level in GPIOx_IDR.

  cortexm_delay_us(delay_us);
#if I2C_CLEAR_BUSY_DEBUG_MESSAGES
  sos_debug_log_info(SOS_DEBUG_DEVICE, "clear busy:%d", __LINE__);
#endif
#if I2C_WAIT_FOR_LINE_CHANGE
  do {
    hal_pio_get(scl_port, &value);
  } while ((value & scl_pio_attr.o_pinmask) == 0);

  cortexm_delay_us(delay_us);
#endif

#if I2C_CLEAR_BUSY_DEBUG_MESSAGES
  sos_debug_log_info(SOS_DEBUG_DEVICE, "clear busy:%d", __LINE__);
#endif
#if I2C_WAIT_FOR_LINE_CHANGE
  do {
    hal_pio_get(sda_port, &value);
  } while ((value & sda_pio_attr.o_pinmask) == 0);
#endif
  // 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level
  // (Write 0 to GPIOx_ODR).
  hal_pio_clrmask(sda_port, (void *)sda_pio_attr.o_pinmask);
  //  5. Check SDA Low level in GPIOx_IDR.
  cortexm_delay_us(delay_us);
#if I2C_CLEAR_BUSY_DEBUG_MESSAGES
  sos_debug_log_info(SOS_DEBUG_DEVICE, "clear busy:%d", __LINE__);
#endif
#if I2C_WAIT_FOR_LINE_CHANGE
  do {
    hal_pio_get(sda_port, &value);
  } while ((value & sda_pio_attr.o_pinmask) != 0);
#endif
  // 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level
  // (Write 0 to GPIOx_ODR).
  hal_pio_clrmask(scl_port, (void *)scl_pio_attr.o_pinmask);
  //  7. Check SCL Low level in GPIOx_IDR.
  cortexm_delay_us(delay_us);
#if I2C_CLEAR_BUSY_DEBUG_MESSAGES
  sos_debug_log_info(SOS_DEBUG_DEVICE, "clear busy:%d", __LINE__);
#endif
#if I2C_WAIT_FOR_LINE_CHANGE
  do {
    hal_pio_get(scl_port, &value);
  } while ((value & scl_pio_attr.o_pinmask) != 0);
#endif
  // 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level
  // (Write 1 to GPIOx_ODR).
  hal_pio_setmask(scl_port, (void *)scl_pio_attr.o_pinmask);
  // 9. Check SCL High level in GPIOx_IDR.
  cortexm_delay_us(delay_us);
#if I2C_CLEAR_BUSY_DEBUG_MESSAGES
  sos_debug_log_info(SOS_DEBUG_DEVICE, "clear busy:%d", __LINE__);
#endif
#if I2C_WAIT_FOR_LINE_CHANGE
  do {
    hal_pio_get(scl_port, &value);
  } while ((value & scl_pio_attr.o_pinmask) == 0);
#endif
  // 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level
  // (Write 1 to GPIOx_ODR).
  hal_pio_setmask(sda_port, (void *)sda_pio_attr.o_pinmask);
  // 11. Check SDA High level in GPIOx_IDR.
  cortexm_delay_us(delay_us);
#if I2C_CLEAR_BUSY_DEBUG_MESSAGES
  sos_debug_log_info(SOS_DEBUG_DEVICE, "clear busy:%d", __LINE__);
#endif
#if I2C_WAIT_FOR_LINE_CHANGE
  do {
    hal_pio_get(sda_port, &value);
  } while ((value & sda_pio_attr.o_pinmask) == 0);
#endif

  // 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.

  hal_set_alternate_pin_function(
    i2c->pin_assignment.scl,
    CORE_PERIPH_I2C,
    port,
    GPIO_MODE_AF_OD,
    GPIO_SPEED_FREQ_HIGH,
    GPIO_PULLUP);

  hal_set_alternate_pin_function(
    i2c->pin_assignment.sda,
    CORE_PERIPH_I2C,
    port,
    GPIO_MODE_AF_OD,
    GPIO_SPEED_FREQ_HIGH,
    GPIO_PULLUP);

  // 13. Set SWRST bit in I2Cx_CR1 register.
  i2c->hal_handle.Instance->CR1 |= 0x8000;

  cortexm_delay_us(10);

  // 14. Clear SWRST bit in I2Cx_CR1 register.
  i2c->hal_handle.Instance->CR1 &= ~0x8000;

  cortexm_delay_us(10);

  // 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register
  i2c->hal_handle.Instance->CR1 |= 0x0001;
}

#endif
