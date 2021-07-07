// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md

#include <cortexm/cortexm.h>
#include <fcntl.h>
#include <mcu/core.h>
#include <mcu/pio.h>
#include <mcu/usb.h>
#include <sos/config.h>
#include <sos/debug.h>
#include <sos/events.h>
#include <usbd/types.h>

#include "stm32_local.h"

#if MCU_USB_PORTS > 0

static void usb_connect(const devfs_handle_t *handle, u32 con);
static void usb_configure(const devfs_handle_t *handle, u32 cfg);
static void usb_set_address(const devfs_handle_t *handle, u32 addr);
static void usb_reset_endpoint(const devfs_handle_t *handle, u32 endpoint_num);
static void usb_flush_endpoint(const devfs_handle_t *handle, u32 endpoint_num);
static void usb_enable_endpoint(const devfs_handle_t *handle, u32 endpoint_num);
static void
usb_disable_endpoint(const devfs_handle_t *handle, u32 endpoint_num);
static void usb_stall_endpoint(const devfs_handle_t *handle, u32 endpoint_num);
static void
usb_unstall_endpoint(const devfs_handle_t *handle, u32 endpoint_num);
static void usb_configure_endpoint(
  const devfs_handle_t *handle,
  u32 endpoint_num,
  u32 max_packet_size,
  u8 type);
static void usb_reset(const devfs_handle_t *handle);

static usb_state_t *m_usb_state_list[MCU_USB_PORTS] MCU_SYS_MEM;

static USB_OTG_GlobalTypeDef *const usb_regs_table[MCU_USB_PORTS]
  = MCU_USB_REGS;
static u8 const usb_irqs[MCU_USB_PORTS] = MCU_USB_IRQS;
static void clear_callbacks(usb_state_t *state);
static void clear_rx_buffer_offsets(usb_state_t *state);

void clear_callbacks(usb_state_t *state) {

  for(int i=0; i < MCU_USB_ENDPOINT_COUNT; i++){
    state->transfer_handler[i] = (devfs_transfer_handler_t){};
  }

  clear_rx_buffer_offsets(state);
  state->control_handler = (mcu_event_handler_t){};
  state->special_event_handler = (mcu_event_handler_t){};
  state->rx_buffer_used = 0;
}

void clear_rx_buffer_offsets(usb_state_t *state) {
  for(int i=0; i < MCU_USB_ENDPOINT_COUNT; i++){
    state->rx_buffer_offset[i] = 0;
  }
}

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(
  usb,
  USB_VERSION,
  USB_IOC_IDENT_CHAR,
  I_MCU_TOTAL + I_USB_TOTAL,
  mcu_usb_isconnected)

int mcu_usb_open(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(usb);
  if (state->ref_count == 0) {
    DEVFS_DRIVER_OPEN_STATE_LOCAL_V4(usb);
    // Set callbacks to NULL
    state->connected = 0;
    clear_callbacks(state);
    state->hal_handle.Instance = usb_regs_table[config->port];

    if (config->port == 0) {
#if MCU_USB_API > 0
#if defined STM32H735xx
      __HAL_RCC_USB_OTG_HS_CLK_ENABLE();
#else
      __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
#endif
#else
      __HAL_RCC_USB_CLK_ENABLE();
#endif
    } else {
#if MCU_USB_PORTS > 1
#if defined __HAL_RCC_OTGPHYC_CLK_ENABLE
      __HAL_RCC_OTGPHYC_CLK_ENABLE();
#endif
      __HAL_RCC_USB_OTG_HS_CLK_ENABLE();
      __HAL_RCC_USB_OTG_HS_ULPI_CLK_ENABLE();
#endif
    }
    cortexm_enable_irq(usb_irqs[config->port]); // Enable USB IRQ
  }
  state->ref_count++;
  return 0;
}

int mcu_usb_close(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(usb);
  if (state->ref_count > 0) {
    if (state->ref_count == 1) {
      HAL_PCD_Stop(&state->hal_handle);
      cortexm_disable_irq(usb_irqs[config->port]); // Disable the USB interrupt
      state->hal_handle.Instance = 0;
      if (config->port == 0) {
#if MCU_USB_API > 0
#if defined STM32H735xx
        __HAL_RCC_USB_OTG_HS_CLK_DISABLE();
#else
        __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
#endif
#else
        __HAL_RCC_USB_CLK_DISABLE();
#endif
      } else {
#if MCU_USB_PORTS > 1
#if defined __HAL_RCC_OTGPHYC_CLK_DISABLE
        __HAL_RCC_OTGPHYC_CLK_DISABLE();
#endif
        __HAL_RCC_USB_OTG_HS_CLK_DISABLE();
        __HAL_RCC_USB_OTG_HS_ULPI_CLK_DISABLE();
#endif
      }
      DEVFS_DRIVER_CLOSE_STATE_LOCAL_V4(usb);
    }
    state->ref_count--;
  }
  return 0;
}

int mcu_usb_getinfo(const devfs_handle_t *handle, void *ctl) {
  MCU_UNUSED_ARGUMENT(handle);
  usb_info_t *info = ctl;

  info->o_flags = 0;
  info->o_events = 0;
  return 0;
}

int mcu_usb_setattr(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(usb);
  u32 port = config->port;

  const usb_attr_t *attr = DEVFS_ASSIGN_ATTRIBUTES(usb, ctl);

  if (attr == 0) {
    return SYSFS_SET_RETURN(ENOSYS);
  }
  u32 o_flags = attr->o_flags;

  if (o_flags & USB_FLAG_SET_DEVICE) {
    int result;

    state->read_ready = 0;
    state->write_pending = 0;

    result = mcu_set_pin_assignment(
      &(attr->pin_assignment),
      MCU_CONFIG_PIN_ASSIGNMENT(usb_config_t, handle),
      MCU_PIN_ASSIGNMENT_COUNT(usb_pin_assignment_t),
      CORE_PERIPH_USB,
      port,
      0,
      0,
      0);

    if (result < 0) {
      return result;
    }

#if defined STM32L4
    if (__HAL_RCC_PWR_IS_CLK_DISABLED()) {
      __HAL_RCC_PWR_CLK_ENABLE();
      HAL_PWREx_EnableVddUSB();
      __HAL_RCC_PWR_CLK_DISABLE();
    } else {
      HAL_PWREx_EnableVddUSB();
    }
#endif


    if (port == 0) {
      state->hal_handle.Init.dev_endpoints = MCU_USB_ENDPOINT_COUNT;
      state->hal_handle.Init.speed = PCD_SPEED_FULL;
      state->hal_handle.Init.phy_itface = PCD_PHY_EMBEDDED;
    } else {
#if MCU_USB_PORTS > 1
      state->hal_handle.Init.dev_endpoints = MCU_USB_ENDPOINT_COUNT;
      if (o_flags & USB_FLAG_IS_HIGH_SPEED) {
        state->hal_handle.Init.speed = USB_OTG_SPEED_HIGH;
      } else {
        state->hal_handle.Init.speed = USB_OTG_SPEED_HIGH_IN_FULL;
      }

      // Need a flag to check for HW interface
      state->hal_handle.Init.phy_itface = USB_OTG_HS_EMBEDDED_PHY;
#endif
    }
    state->hal_handle.Init.dma_enable = DISABLE;
    state->hal_handle.Init.ep0_mps = DEP0CTL_MPS_64;
    if (sos_config.usb.control_endpoint_max_size <= 8) {
      state->hal_handle.Init.ep0_mps = DEP0CTL_MPS_8;
    } else if (sos_config.usb.control_endpoint_max_size <= 16) {
      state->hal_handle.Init.ep0_mps = DEP0CTL_MPS_16;
    } else if (sos_config.usb.control_endpoint_max_size <= 32) {
      state->hal_handle.Init.ep0_mps = DEP0CTL_MPS_32;
    }

    state->hal_handle.Init.Sof_enable = DISABLE;
    state->hal_handle.Init.low_power_enable = DISABLE;

#if !defined STM32F2
    state->hal_handle.Init.lpm_enable = DISABLE;
    state->hal_handle.Init.battery_charging_enable = DISABLE;
#endif

#if MCU_USB_API > 0
    state->hal_handle.Init.vbus_sensing_enable = DISABLE;
    state->hal_handle.Init.use_dedicated_ep1 = DISABLE;
    state->hal_handle.Init.use_external_vbus = DISABLE;

    if (o_flags & USB_FLAG_IS_VBUS_SENSING_ENABLED) {
      state->hal_handle.Init.vbus_sensing_enable = ENABLE;
    }
#endif

    if (o_flags & USB_FLAG_IS_SOF_ENABLED) {
      state->hal_handle.Init.Sof_enable = ENABLE;
    }

#if !defined STM32F2
    if (o_flags & USB_FLAG_IS_LOW_POWER_MODE_ENABLED) {
      state->hal_handle.Init.lpm_enable = ENABLE;
    }

    if (o_flags & USB_FLAG_IS_BATTERY_CHARGING_ENABLED) {
      state->hal_handle.Init.battery_charging_enable = ENABLE;
    }
#endif

    int pcd_init_result;
    if ((pcd_init_result = HAL_PCD_Init(&state->hal_handle)) != HAL_OK) {
#if 0
      sos_debug_printf("Result is %d\n", pcd_init_result);
      sos_debug_printf("State is %d\n", state->hal_handle.State);
#endif
      return SYSFS_SET_RETURN(EIO);
    }

#if MCU_USB_API > 0
    int i;
    HAL_PCDEx_SetRxFiFo(
      &state->hal_handle,
      stm32_config.usb
        .rx_fifo_word_size); // size is in 32-bit words for all fifo - 512

    for (i = 0; i < USB_TX_FIFO_WORD_SIZE_COUNT; i++) {
      if (stm32_config.usb.tx_fifo_word_size[i] > 0) {
        HAL_PCDEx_SetTxFiFo(
          &state->hal_handle,
          i,
          stm32_config.usb.tx_fifo_word_size[i]);
      }
    }

    usb_connect(handle, 1);

#else

#if 1
    HAL_PCDEx_PMAConfig(
      &state->hal_handle,
      0x00,
      PCD_SNG_BUF,
      0x18); // why do we start 24 bytes in?
    HAL_PCDEx_PMAConfig(
      &state->hal_handle,
      0x80,
      PCD_SNG_BUF,
      0x18 + 64); // 64 bytes for 00

    HAL_PCDEx_PMAConfig(
      &state->hal_handle,
      0x81,
      PCD_SNG_BUF,
      0x18 + 64 + 64); // interrupt in
    HAL_PCDEx_PMAConfig(
      &state->hal_handle,
      0x82,
      PCD_SNG_BUF,
      0x18 + 64 + 64 + 64); // bulk input -- sending data to computer
    HAL_PCDEx_PMAConfig(
      &state->hal_handle,
      0x02,
      PCD_SNG_BUF,
      0x18 + 64 + 64 + 64 + 64); // bulk output -- receiving data from computer
#else
    HAL_PCDEx_PMAConfig(&state->hal_handle, 0x00, PCD_SNG_BUF, 0x18);
    HAL_PCDEx_PMAConfig(&state->hal_handle, 0x80, PCD_SNG_BUF, 0x58);
    HAL_PCDEx_PMAConfig(&state->hal_handle, 0x81, PCD_SNG_BUF, 0xC0);
    HAL_PCDEx_PMAConfig(&state->hal_handle, 0x01, PCD_SNG_BUF, 0x110);
    HAL_PCDEx_PMAConfig(&state->hal_handle, 0x82, PCD_SNG_BUF, 0x100);
#endif

#endif
  }

  if (o_flags & USB_FLAG_RESET) {
    usb_reset(handle);
  }
  if (o_flags & USB_FLAG_ATTACH) {
    HAL_PCD_DevConnect(&state->hal_handle);
    // usb_connect(port, 1);
  }
  if (o_flags & USB_FLAG_DETACH) {
    HAL_PCD_DevDisconnect(&state->hal_handle);
    // usb_connect(port, 0);
  }
  if (o_flags & USB_FLAG_CONFIGURE) {
    usb_configure(handle, 1);
  }
  if (o_flags & USB_FLAG_UNCONFIGURE) {
    usb_configure(handle, 0);
  }

  if (o_flags & USB_FLAG_SET_ADDRESS) {
    usb_set_address(handle, attr->address);
  }

  if (o_flags & USB_FLAG_RESET_ENDPOINT) {
    usb_reset_endpoint(handle, attr->address);
  }

  if (o_flags & USB_FLAG_ENABLE_ENDPOINT) {
    usb_enable_endpoint(handle, attr->address);
  }

  if (o_flags & USB_FLAG_FLUSH_ENDPOINT) {
    usb_flush_endpoint(handle, attr->address);
  }

  if (o_flags & USB_FLAG_DISABLE_ENDPOINT) {
    usb_disable_endpoint(handle, attr->address);
  }

  if (o_flags & USB_FLAG_STALL_ENDPOINT) {
    usb_stall_endpoint(handle, attr->address);
  }

  if (o_flags & USB_FLAG_UNSTALL_ENDPOINT) {
    usb_unstall_endpoint(handle, attr->address);
  }

  if (o_flags & USB_FLAG_CONFIGURE_ENDPOINT) {
    usb_configure_endpoint(
      handle,
      attr->address,
      attr->max_packet_size,
      attr->type);
  }

  return 0;
}

void usb_connect(const devfs_handle_t *handle, u32 con) {
  DEVFS_DRIVER_DECLARE_STATE(usb);
  if (con) {
    // what is this delay waiting for
    // seems to be needed on STM32H7 between setattr and start
    cortexm_delay_ms(1);
    HAL_PCD_Start(&state->hal_handle);

#if defined STM32H7
    HAL_PWREx_EnableUSBVoltageDetector();
#endif
  } else {
    HAL_PCD_Stop(&state->hal_handle);
  }
}

int mcu_usb_setaction(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(usb);
  mcu_action_t *action = (mcu_action_t *)ctl;
  int log_ep;
  int ret = -1;

  // cortexm_set_irq_prio(USB_IRQn, action->prio);
  log_ep = action->channel & ~0x80;

  cortexm_set_irq_priority(
    usb_irqs[config->port],
    action->prio,
    action->o_events);

  if (
    action->o_events
    & (MCU_EVENT_FLAG_POWER | MCU_EVENT_FLAG_SUSPEND | MCU_EVENT_FLAG_STALL | MCU_EVENT_FLAG_SOF | MCU_EVENT_FLAG_WAKEUP)) {
    state->special_event_handler = action->handler;
    return 0;
  }

  if (action->channel & 0x80) {
    if (
      (action->handler.callback == 0)
      && (action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE)) {
      state->write_pending &= ~(1 << log_ep);
      if (log_ep > 0) {
        devfs_execute_cancel_handler(
          &(state->transfer_handler[log_ep]),
          0,
          SYSFS_SET_RETURN(ECANCELED),
          MCU_EVENT_FLAG_CANCELED);
      } else {
        devfs_execute_event_handler(
          &(state->control_handler),
          MCU_EVENT_FLAG_CANCELED,
          0);
      }
    }
  } else {
    if (
      (action->handler.callback == 0)
      && (action->o_events & MCU_EVENT_FLAG_DATA_READY)) {
      state->read_ready |= (1 << log_ep);
      if (log_ep > 0) {
        devfs_execute_cancel_handler(
          &(state->transfer_handler[log_ep]),
          0,
          SYSFS_SET_RETURN(ECANCELED),
          MCU_EVENT_FLAG_CANCELED);
      } else {
        devfs_execute_event_handler(
          &(state->control_handler),
          MCU_EVENT_FLAG_CANCELED,
          0);
      }
    }
  }

  if ((log_ep < MCU_USB_ENDPOINT_COUNT)) {
    if (action->o_events & MCU_EVENT_FLAG_DATA_READY) {
      // cortexm_enable_interrupts();
      if (cortexm_validate_callback(action->handler.callback) < 0) {
        return SYSFS_SET_RETURN(EPERM);
      }

      if (log_ep == 0) {
        if (action->o_events & MCU_EVENT_FLAG_SETUP) {
          state->control_handler = action->handler;
        } else {
          return SYSFS_SET_RETURN(EINVAL);
        }
      } else {
        return SYSFS_SET_RETURN(EINVAL);
      }
      ret = 0;
    }

    if (action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE) {
      if (cortexm_validate_callback(action->handler.callback) < 0) {
        return SYSFS_SET_RETURN(EPERM);
      }

      if (log_ep == 0) {
        if (action->o_events & MCU_EVENT_FLAG_SETUP) {
          state->control_handler = action->handler;
        } else {
          return SYSFS_SET_RETURN(EINVAL);
        }
      } else {
        return SYSFS_SET_RETURN(EINVAL);
      }
      ret = 0;
    }
  }

  if (ret < 0) {
    ret = SYSFS_SET_RETURN(EINVAL);
  }
  return ret;
}

int mcu_usb_read(const devfs_handle_t *handle, devfs_async_t *async) {
  DEVFS_DRIVER_DECLARE_STATE(usb);

  int loc = async->loc;
  if (loc > (MCU_USB_ENDPOINT_COUNT - 1)) {
    return SYSFS_SET_RETURN(EINVAL);
  }

  DEVFS_DRIVER_IS_BUSY(state->transfer_handler[loc].read, async);

  int result;

  // Synchronous read (only if data is ready) otherwise 0 is returned
  if (state->read_ready & (1 << loc)) {
    result = mcu_usb_root_read_endpoint(handle, loc, async->buf);
    if (result == 0) {
      result = SYSFS_SET_RETURN(EAGAIN);
    }
  } else {
    if (!(async->flags & O_NONBLOCK)) {
      // If this is a blocking call, set the callback and context
      if (cortexm_validate_callback(async->handler.callback) < 0) {
        result = SYSFS_SET_RETURN(EPERM);
      } else {
        result = 0;
      }
    } else {
      result = SYSFS_SET_RETURN(EAGAIN);
    }
  }

  if (result != 0) {
    state->transfer_handler[loc].read = 0;
  }

  return result;
}

int mcu_usb_write(const devfs_handle_t *handle, devfs_async_t *async) {
  // Asynchronous write
  DEVFS_DRIVER_DECLARE_STATE(usb);

  int loc = async->loc;
  int ep;
  ep = (loc & 0x7F);
  if (ep > (MCU_USB_ENDPOINT_COUNT - 1)) {
    return SYSFS_SET_RETURN(EINVAL);
  }

  DEVFS_DRIVER_IS_BUSY(state->transfer_handler[ep].write, async);

  int bytes_written;

  if (cortexm_validate_callback(async->handler.callback) < 0) {
    return SYSFS_SET_RETURN(EPERM);
  }

  state->write_pending |= (1 << ep);
  bytes_written
    = mcu_usb_root_write_endpoint(handle, loc, async->buf, async->nbyte);

  if (bytes_written < 0) {
    usb_disable_endpoint(handle, loc);
    usb_reset_endpoint(handle, loc);
    usb_enable_endpoint(handle, loc);
  }

  if (bytes_written != 0) {
    state->transfer_handler[loc].write = 0;
    state->write_pending &= ~(1 << ep);
  }

  return bytes_written;
}

void usb_reset(const devfs_handle_t *handle) {
  MCU_UNUSED_ARGUMENT(handle);
}

void usb_wakeup(int port) {
  MCU_UNUSED_ARGUMENT(port);
}

void usb_set_address(const devfs_handle_t *handle, u32 addr) {
  DEVFS_DRIVER_DECLARE_STATE(usb);
  HAL_PCD_SetAddress(&state->hal_handle, addr);
}

void usb_configure(const devfs_handle_t *handle, u32 cfg) {
  // m_usb_state_list[config->port].connected = 1;
  MCU_UNUSED_ARGUMENT(handle);
  MCU_UNUSED_ARGUMENT(cfg);
}

void usb_configure_endpoint(
  const devfs_handle_t *handle,
  u32 endpoint_num,
  u32 max_packet_size,
  u8 type) {
  DEVFS_DRIVER_DECLARE_STATE(usb);

  HAL_PCD_EP_Open(
    &state->hal_handle,
    endpoint_num,
    max_packet_size,
    type & EP_TYPE_MSK);
  // m_usb_state_list[config->port].connected = 1;

  if ((endpoint_num & 0x80) == 0) {
    void *dest_buffer;

    if (state->rx_buffer_offset[endpoint_num] == 0) {
      state->rx_buffer_offset[endpoint_num] = state->rx_buffer_used;

      state->rx_buffer_used += (max_packet_size * 2);
      if (state->rx_buffer_used > stm32_config.usb.rx_buffer_size) {
        // this is a fatal error -- using sos_debug_ will cause bootloader link
        // problems
        sos_handle_event(SOS_EVENT_ROOT_FATAL, "usbbuf");
      }
    }

    dest_buffer = stm32_config.usb.rx_buffer
                  + state->rx_buffer_offset[endpoint_num] + max_packet_size;

    HAL_PCD_EP_Receive(
      &state->hal_handle,
      endpoint_num,
      dest_buffer,
      max_packet_size);
  }
}

void usb_enable_endpoint(const devfs_handle_t *handle, u32 endpoint_num) {
  MCU_UNUSED_ARGUMENT(handle);
  MCU_UNUSED_ARGUMENT(endpoint_num);
}

void usb_disable_endpoint(const devfs_handle_t *handle, u32 endpoint_num) {
  DEVFS_DRIVER_DECLARE_STATE(usb);
  HAL_PCD_EP_Close(&state->hal_handle, endpoint_num);
}

void usb_reset_endpoint(const devfs_handle_t *handle, u32 endpoint_num) {
  MCU_UNUSED_ARGUMENT(handle);
  MCU_UNUSED_ARGUMENT(endpoint_num);
}

void usb_flush_endpoint(const devfs_handle_t *handle, u32 endpoint_num) {
  DEVFS_DRIVER_DECLARE_STATE(usb);
  PCD_HandleTypeDef *hpcd = &state->hal_handle;
  u8 logical_endpoint = endpoint_num & ~0x80;

  if (((endpoint_num & 0x80)
       && (hpcd->IN_ep[logical_endpoint].type == EP_TYPE_ISOC))) {
    HAL_PCD_EP_Close(hpcd, endpoint_num);
    HAL_PCD_EP_Open(
      hpcd,
      endpoint_num,
      hpcd->IN_ep[logical_endpoint].maxpacket,
      EP_TYPE_ISOC);
  } else if ((((endpoint_num & 0x80) == 0)
              && (hpcd->OUT_ep[logical_endpoint].type == EP_TYPE_ISOC))) {
    HAL_PCD_EP_Close(hpcd, endpoint_num);
    HAL_PCD_EP_Open(
      hpcd,
      endpoint_num,
      hpcd->OUT_ep[logical_endpoint].maxpacket,
      EP_TYPE_ISOC);
  }
  HAL_PCD_EP_Flush(hpcd, endpoint_num);
}

void usb_stall_endpoint(const devfs_handle_t *handle, u32 endpoint_num) {
  DEVFS_DRIVER_DECLARE_STATE(usb);
  HAL_PCD_EP_SetStall(&state->hal_handle, endpoint_num);
}

void usb_unstall_endpoint(const devfs_handle_t *handle, u32 endpoint_num) {
  DEVFS_DRIVER_DECLARE_STATE(usb);
  HAL_PCD_EP_ClrStall(&state->hal_handle, endpoint_num);
}

int mcu_usb_isconnected(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_STATE(usb);
  MCU_UNUSED_ARGUMENT(ctl);
  return state->connected;
}

void usb_clr_ep_buf(const devfs_handle_t *handle, u32 endpoint_num) {
  MCU_UNUSED_ARGUMENT(handle);
  MCU_UNUSED_ARGUMENT(endpoint_num);
}

int mcu_usb_root_read_endpoint(
  const devfs_handle_t *handle,
  u32 endpoint_num,
  void *dest) {
  DEVFS_DRIVER_DECLARE_STATE(usb);
  void *src_buffer;
  u8 epnum;
  epnum = endpoint_num & 0x7f;

  if (state->read_ready & (1 << epnum)) {
    state->read_ready &= ~(1 << epnum);
    src_buffer = stm32_config.usb.rx_buffer + state->rx_buffer_offset[epnum];
    // data is copied from fifo to buffer during the interrupt
    memcpy(dest, src_buffer, state->rx_count[epnum]);
    return state->rx_count[epnum];
  }

  return -1;
}

int mcu_usb_root_write_endpoint(
  const devfs_handle_t *handle,
  u32 endpoint_num,
  const void *src,
  u32 size) {
  int ret;

  DEVFS_DRIVER_DECLARE_STATE(usb);
#if MCU_USB_API > 0
  int logical_endpoint = endpoint_num & 0x7f;
  int type = state->hal_handle.IN_ep[logical_endpoint].type;
  if (type == EP_TYPE_ISOC) {
    // check to see if the packet will fit in the FIFO
    // if the packet won't fit, return EBUSY
#if !defined STM32H7
    USB_OTG_GlobalTypeDef *USBx = state->hal_handle.Instance;
    int available
      = (USBx_INEP(logical_endpoint)->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV);
    if ((available * 4) < size) {
      return SYSFS_SET_RETURN(EBUSY);
    }
#endif
  }
#endif

  ret
    = HAL_PCD_EP_Transmit(&state->hal_handle, endpoint_num, (void *)src, size);
  if (ret == HAL_OK) {
#if 0
    if (type == EP_TYPE_ISOC) {
      return size;
    }
#endif
    return 0;
  }

  return SYSFS_SET_RETURN(EIO);
}

void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd) {
  usb_state_t *state = (usb_state_t *)hpcd;
  // a setup packet has been received
  usb_event_t event;
  event.epnum = 0;
  void *dest_buffer;
  usbd_setup_packet_t *setup = (usbd_setup_packet_t *)&hpcd->Setup;

  // Setup data is in hpcd->Setup buffer at this point

  // copy setup data to ep0 data buffer
  state->read_ready |= (1 << 0);
  dest_buffer = stm32_config.usb.rx_buffer + state->rx_buffer_offset[0];
  state->rx_count[0] = sizeof(usbd_setup_packet_t);
  memcpy(dest_buffer, hpcd->Setup, state->rx_count[0]);

  devfs_execute_event_handler(
    &state->control_handler,
    MCU_EVENT_FLAG_SETUP,
    &event);

  // prepare EP zero for receiving out data
  if (
    (setup->bmRequestType.bitmap_t.dir
     == USBD_REQUEST_TYPE_DIRECTION_HOST_TO_DEVICE)
    && (setup->wLength > 0)) {

    // prepare the endpoint to receive some data
    HAL_PCD_EP_Receive(
      hpcd,
      0,
      stm32_config.usb.rx_buffer + hpcd->OUT_ep[0].maxpacket,
      setup->wLength);
  }
}

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
  usb_state_t *state = (usb_state_t *)hpcd;
  // data has already been received and is stored in buffer specified by
  // HAL_PCD_EP_Receive
  usb_event_t event;
  event.epnum = epnum;
  u16 count;
  void *src_buffer;
  void *dest_buffer;

  // set read ready flag
  count = HAL_PCD_EP_GetRxCount(&state->hal_handle, epnum);

  dest_buffer = stm32_config.usb.rx_buffer + state->rx_buffer_offset[epnum];
  src_buffer = dest_buffer + hpcd->OUT_ep[epnum].maxpacket;
  memcpy(dest_buffer, src_buffer, count); // free up the source buffer

  // prepare to receive the next packet in the local buffer for non-control
  // endpoints
  if (epnum != 0) {
    HAL_PCD_EP_Receive(hpcd, epnum, src_buffer, hpcd->OUT_ep[epnum].maxpacket);
  }

  state->rx_count[epnum] = count;
  if (count > 0) {
    if (epnum == 0) {
      state->read_ready |= (1 << epnum);
      devfs_execute_event_handler(
        &state->control_handler,
        MCU_EVENT_FLAG_DATA_READY,
        &event);
    } else if (state->transfer_handler[epnum].read) {
      devfs_async_t *async = state->transfer_handler[epnum].read;
      if (count > async->nbyte) {
        count = async->nbyte;
      }
      // copy directly to the async buffer that is waiting for data
      memcpy(state->transfer_handler[epnum].read->buf, dest_buffer, count);
      state->read_ready &= ~(1 << epnum);
      devfs_execute_read_handler(
        state->transfer_handler + epnum,
        &event,
        count,
        MCU_EVENT_FLAG_DATA_READY);
    } else {
      // data is ready to read synchronously
      state->read_ready |= (1 << epnum);
    }
  }
}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
  usb_state_t *state = (usb_state_t *)hpcd;
  u8 logical_ep = epnum & 0x7f;
  usb_event_t event;
  event.epnum = epnum;

  state->write_pending &= ~(1 << logical_ep);

  // devfs_execute_write_handler(usb->transfer_handlers + logical_ep, &event, 0,
  // MCU_EVENT_FLAG_WRITE_COMPLETE);

  if (logical_ep == 0) {
    devfs_execute_event_handler(
      &state->control_handler,
      MCU_EVENT_FLAG_WRITE_COMPLETE,
      &event);

    // only proceed it DataIn tx'd more than zero bytes
    if (hpcd->IN_ep[0].xfer_count == 0) {
      return;
    }

    // ep 0 data in complete
    // prepare EP0 for next setup packet
    HAL_PCD_EP_Receive(hpcd, 0, 0, 0);
  } else {
    devfs_execute_write_handler(
      state->transfer_handler + logical_ep,
      &event,
      hpcd->IN_ep[logical_ep].xfer_count,
      0);
  }
}

void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd) {
  MCU_UNUSED_ARGUMENT(hpcd);
}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd) {
  const u32 mps = sos_config.usb.control_endpoint_max_size;
  usb_state_t *usb = (usb_state_t *)hpcd;
  usb->connected = 1;
  usb->rx_buffer_used = mps;
  clear_rx_buffer_offsets(usb);

  HAL_PCD_SetAddress(hpcd, 0);
  HAL_PCD_EP_Open(hpcd, 0x00, mps, EP_TYPE_CTRL);
  HAL_PCD_EP_Open(hpcd, 0x80, mps, EP_TYPE_CTRL);
}

void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd) {
  usb_state_t *usb = (usb_state_t *)hpcd;
  usb->connected = 0;

#if MCU_USB_API > 0
  __HAL_PCD_GATE_PHYCLOCK(hpcd);
#endif

#if 0
  // this causes problems when USB cable is removed -- don't do it
  HAL_PCD_EP_Close(hpcd, 0x00);
  HAL_PCD_EP_Close(hpcd, 0x80);
#endif
}

void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd) {
  usb_state_t *usb = (usb_state_t *)hpcd;
  usb->connected = 1;
}

void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {

  // epnum value passed is not valid -- need to find it
  int i;
  for (i = 1; i < MCU_USB_ENDPOINT_COUNT; i++) {
    if (hpcd->IN_ep[i].type == EP_TYPE_ISOC) {

      epnum = 0x80 | i;
      u8 logical_ep = epnum & 0x7F;

      // Close will disable the endpoint and flush the TX FIFO
      if (HAL_PCD_EP_Close(hpcd, epnum) != HAL_OK) {
      }

      if (
        HAL_PCD_EP_Open(
          hpcd,
          epnum,
          hpcd->IN_ep[logical_ep].maxpacket,
          EP_TYPE_ISOC)
        != HAL_OK) {
      }

      usb_state_t *state = (usb_state_t *)hpcd;
      devfs_execute_write_handler(
        state->transfer_handler + logical_ep,
        0,
        SYSFS_SET_RETURN(EIO),
        MCU_EVENT_FLAG_ERROR | MCU_EVENT_FLAG_CANCELED);
    }
  }
}

void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {\
  MCU_UNUSED_ARGUMENT(hpcd);
  MCU_UNUSED_ARGUMENT(epnum);
}

void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd) {
  MCU_UNUSED_ARGUMENT(hpcd);
  // this is never called -- Reset callback is called when connection is
  // established
}

void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd) {
  MCU_UNUSED_ARGUMENT(hpcd);
  // this is never called -- Suspend callback is called when connection is lost
}




void mcu_core_otg_fs_isr() {
  HAL_PCD_IRQHandler(&m_usb_state_list[0]->hal_handle);
}

#if MCU_USB_PORTS > 1
void mcu_core_otg_hs_isr() {
  HAL_PCD_IRQHandler(&m_usb_state_list[1].hal_handle);
}
#else
void mcu_core_otg_hs_isr(){
  HAL_PCD_IRQHandler(&m_usb_state_list[0]->hal_handle);
}
#endif

#endif
