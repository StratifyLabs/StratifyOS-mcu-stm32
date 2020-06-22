/* Copyright 2011-2018 Tyler Gilbert;
 * This file is part of Stratify OS.
 *
 * Stratify OS is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Stratify OS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Stratify OS.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */

#include <fcntl.h>
#include <mcu/usb.h>
#include <mcu/pio.h>
#include <cortexm/cortexm.h>
#include <usbd/types.h>
#include <mcu/core.h>
#include <mcu/debug.h>
#include <mcu/boot_debug.h>
#include <usbd/types.h>

#include "stm32_local.h"

#if MCU_USB_PORTS > 0

static void usb_connect(u32 port, u32 con);
static void usb_configure(const devfs_handle_t * handle, u32 cfg);
static void usb_set_address(const devfs_handle_t * handle, u32 addr);
static void usb_reset_endpoint(const devfs_handle_t * handle, u32 endpoint_num);
static void usb_flush_endpoint(const devfs_handle_t * handle, u32 endpoint_num);
static void usb_enable_endpoint(const devfs_handle_t * handle, u32 endpoint_num);
static void usb_disable_endpoint(const devfs_handle_t * handle, u32 endpoint_num);
static void usb_stall_endpoint(const devfs_handle_t * handle, u32 endpoint_num);
static void usb_unstall_endpoint(const devfs_handle_t * handle, u32 endpoint_num);
static void usb_configure_endpoint(const devfs_handle_t * handle, u32 endpoint_num, u32 max_packet_size, u8 type);
static void usb_reset(const devfs_handle_t * handle);

typedef struct MCU_PACK {
	PCD_HandleTypeDef hal_handle;
	devfs_transfer_handler_t transfer_handler[DEV_USB_LOGICAL_ENDPOINT_COUNT];
	mcu_event_handler_t control_handler;
	u16 rx_count[DEV_USB_LOGICAL_ENDPOINT_COUNT];
	u16 rx_buffer_offset[DEV_USB_LOGICAL_ENDPOINT_COUNT];
	u16 rx_buffer_used;
	volatile u32 write_pending;
	volatile u32 read_ready;
	mcu_event_handler_t special_event_handler;
	u8 ref_count;
	u8 connected;
} usb_local_t;

static usb_local_t m_usb_local[MCU_USB_PORTS] MCU_SYS_MEM;

static USB_OTG_GlobalTypeDef * const usb_regs_table[MCU_USB_PORTS] = MCU_USB_REGS;
static u8 const usb_irqs[MCU_USB_PORTS] = MCU_USB_IRQS;
static void clear_callbacks();

void clear_callbacks(int port){
	memset(m_usb_local[port].transfer_handler, 0, (DEV_USB_LOGICAL_ENDPOINT_COUNT) * sizeof(devfs_transfer_handler_t));
	memset(&m_usb_local[port].control_handler, 0, sizeof(mcu_event_handler_t));
	memset(&m_usb_local[port].special_event_handler, 0, sizeof(mcu_event_handler_t));
	memset(m_usb_local[port].rx_buffer_offset, 0, DEV_USB_LOGICAL_ENDPOINT_COUNT * sizeof(u16));
	m_usb_local[port].rx_buffer_used = 0;
}

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(usb, USB_VERSION, USB_IOC_IDENT_CHAR, I_MCU_TOTAL + I_USB_TOTAL, mcu_usb_isconnected)


int mcu_usb_open(const devfs_handle_t * handle){
	u32 port = handle->port;
	if ( m_usb_local[port].ref_count == 0 ){
		//Set callbacks to NULL
		m_usb_local[port].connected = 0;
		clear_callbacks(handle->port);
		m_usb_local[port].hal_handle.Instance = usb_regs_table[port];

		if( port == 0 ){
#if MCU_USB_API > 0
			__HAL_RCC_USB_OTG_FS_CLK_ENABLE();
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
		cortexm_enable_irq(usb_irqs[port]);  //Enable USB IRQ
	}
	m_usb_local[port].ref_count++;
	return 0;
}

int mcu_usb_close(const devfs_handle_t * handle){
	u32 port = handle->port;
	if ( m_usb_local[port].ref_count > 0 ){
		if ( m_usb_local[port].ref_count == 1 ){
			HAL_PCD_Stop(&m_usb_local[port].hal_handle);
			cortexm_disable_irq(usb_irqs[port]);  //Disable the USB interrupt
			m_usb_local[port].hal_handle.Instance = 0;
			if( port == 0 ){
#if MCU_USB_API > 0
				__HAL_RCC_USB_OTG_FS_CLK_DISABLE();
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
		}
		m_usb_local[port].ref_count--;
	}
	return 0;
}

int mcu_usb_getinfo(const devfs_handle_t * handle, void * ctl){
	usb_info_t * info = ctl;

	info->o_flags = 0;
	info->o_events = 0;
	return 0;
}

int mcu_usb_setattr(const devfs_handle_t * handle, void * ctl){
	u32 port = handle->port;

	const usb_attr_t * attr = mcu_select_attr(handle, ctl);
	if( attr == 0 ){ return SYSFS_SET_RETURN(ENOSYS); }
	u32 o_flags = attr->o_flags;


	if( o_flags & USB_FLAG_SET_DEVICE ){
		//Start the USB clock
		int result;
		mcu_core_setusbclock(attr->freq);

		m_usb_local[port].read_ready = 0;
		m_usb_local[port].write_pending = 0;


		result = mcu_set_pin_assignment(
					&(attr->pin_assignment),
					MCU_CONFIG_PIN_ASSIGNMENT(usb_config_t, handle),
					MCU_PIN_ASSIGNMENT_COUNT(usb_pin_assignment_t),
					CORE_PERIPH_USB, port, 0, 0, 0
					);

		if( result < 0 ){ return result; }

#if defined STM32L4
		if(__HAL_RCC_PWR_IS_CLK_DISABLED()){
			__HAL_RCC_PWR_CLK_ENABLE();
			HAL_PWREx_EnableVddUSB();
			__HAL_RCC_PWR_CLK_DISABLE();
		} else {
			HAL_PWREx_EnableVddUSB();
		}
#endif

		if( port == 0 ){
			m_usb_local[port].hal_handle.Init.dev_endpoints = 9;
			m_usb_local[port].hal_handle.Init.speed = PCD_SPEED_FULL;
			m_usb_local[port].hal_handle.Init.phy_itface = PCD_PHY_EMBEDDED;
		} else {
#if MCU_USB_PORTS > 1
			m_usb_local[port].hal_handle.Init.dev_endpoints = DEV_USB_LOGICAL_ENDPOINT_COUNT;
			if( o_flags & USB_FLAG_IS_HIGH_SPEED ){
				m_usb_local[port].hal_handle.Init.speed = USB_OTG_SPEED_HIGH;
			} else {
				m_usb_local[port].hal_handle.Init.speed = USB_OTG_SPEED_HIGH_IN_FULL;
			}

			//Need a flag to check for HW interface
			m_usb_local[port].hal_handle.Init.phy_itface = USB_OTG_HS_EMBEDDED_PHY;
#endif
		}
		m_usb_local[port].hal_handle.Init.dma_enable = DISABLE;
		m_usb_local[port].hal_handle.Init.ep0_mps = DEP0CTL_MPS_64;
		if ( attr->max_packet_size <= 8 ){
			m_usb_local[port].hal_handle.Init.ep0_mps = DEP0CTL_MPS_8;
		} else if ( attr->max_packet_size <= 16 ){
			m_usb_local[port].hal_handle.Init.ep0_mps = DEP0CTL_MPS_16;
		} else if ( attr->max_packet_size <= 32 ){
			m_usb_local[port].hal_handle.Init.ep0_mps = DEP0CTL_MPS_32;
		}

		m_usb_local[port].hal_handle.Init.Sof_enable = DISABLE;
		m_usb_local[port].hal_handle.Init.low_power_enable = DISABLE;

#if !defined STM32F2
		m_usb_local[port].hal_handle.Init.lpm_enable = DISABLE;
		m_usb_local[port].hal_handle.Init.battery_charging_enable = DISABLE;
#endif

#if MCU_USB_API > 0
		m_usb_local[port].hal_handle.Init.vbus_sensing_enable = DISABLE;
		m_usb_local[port].hal_handle.Init.use_dedicated_ep1 = DISABLE;
		m_usb_local[port].hal_handle.Init.use_external_vbus = DISABLE;

		if( o_flags & USB_FLAG_IS_VBUS_SENSING_ENABLED ){
			m_usb_local[port].hal_handle.Init.vbus_sensing_enable = ENABLE;
		}
#endif

		if( o_flags & USB_FLAG_IS_SOF_ENABLED ){
			m_usb_local[port].hal_handle.Init.Sof_enable = ENABLE;
		}

#if !defined STM32F2
		if( o_flags & USB_FLAG_IS_LOW_POWER_MODE_ENABLED ){
			m_usb_local[port].hal_handle.Init.lpm_enable = ENABLE;
		}

		if( o_flags & USB_FLAG_IS_BATTERY_CHARGING_ENABLED ){
			m_usb_local[port].hal_handle.Init.battery_charging_enable = ENABLE;
		}
#endif

		if( HAL_PCD_Init(&m_usb_local[port].hal_handle) != HAL_OK ){
			return SYSFS_SET_RETURN(EIO);
		}

#if MCU_USB_API > 0
		int i;
		HAL_PCDEx_SetRxFiFo(
					&m_usb_local[port].hal_handle,
					attr->rx_fifo_word_size
					);  //size is in 32-bit words for all fifo - 512

		for(i=0; i < USB_TX_FIFO_WORD_SIZE_COUNT; i++){
			if( attr->tx_fifo_word_size[i] > 0 ){
				HAL_PCDEx_SetTxFiFo(
							&m_usb_local[port].hal_handle,
							i,
							attr->tx_fifo_word_size[i]
							);
			}
		}

		usb_connect(port, 1);
#else

#if 1
		HAL_PCDEx_PMAConfig(&m_usb_local[port].hal_handle, 0x00 , PCD_SNG_BUF, 0x18);  //why do we start 24 bytes in?
		HAL_PCDEx_PMAConfig(&m_usb_local[port].hal_handle, 0x80 , PCD_SNG_BUF, 0x18+64); //64 bytes for 00

		HAL_PCDEx_PMAConfig(&m_usb_local[port].hal_handle, 0x81 , PCD_SNG_BUF, 0x18+64+64); //interrupt in
		HAL_PCDEx_PMAConfig(&m_usb_local[port].hal_handle, 0x82 , PCD_SNG_BUF, 0x18+64+64+64); //bulk input -- sending data to computer
		HAL_PCDEx_PMAConfig(&m_usb_local[port].hal_handle, 0x02 , PCD_SNG_BUF, 0x18+64+64+64+64); //bulk output -- receiving data from computer
#else
		HAL_PCDEx_PMAConfig(&m_usb_local[port].hal_handle , 0x00 , PCD_SNG_BUF, 0x18);
		HAL_PCDEx_PMAConfig(&m_usb_local[port].hal_handle , 0x80 , PCD_SNG_BUF, 0x58);
		HAL_PCDEx_PMAConfig(&m_usb_local[port].hal_handle , 0x81 , PCD_SNG_BUF, 0xC0);
		HAL_PCDEx_PMAConfig(&m_usb_local[port].hal_handle , 0x01 , PCD_SNG_BUF, 0x110);
		HAL_PCDEx_PMAConfig(&m_usb_local[port].hal_handle , 0x82 , PCD_SNG_BUF, 0x100);
#endif

#endif
	}


	if( o_flags & USB_FLAG_RESET ){ usb_reset(handle); }
	if( o_flags & USB_FLAG_ATTACH ){
		HAL_PCD_DevConnect(&m_usb_local[port].hal_handle);
		//usb_connect(port, 1);
	}
	if( o_flags & USB_FLAG_DETACH ){
		HAL_PCD_DevDisconnect(&m_usb_local[port].hal_handle);
		//usb_connect(port, 0);
	}
	if( o_flags & USB_FLAG_CONFIGURE ){ usb_configure(handle, 1); }
	if( o_flags & USB_FLAG_UNCONFIGURE ){ usb_configure(handle, 0); }

	if( o_flags & USB_FLAG_SET_ADDRESS ){
		usb_set_address(handle, attr->address);
	}

	if( o_flags & USB_FLAG_RESET_ENDPOINT ){
		usb_reset_endpoint(handle, attr->address);
	}

	if( o_flags & USB_FLAG_ENABLE_ENDPOINT ){
		usb_enable_endpoint(handle, attr->address);
	}

	if( o_flags & USB_FLAG_FLUSH_ENDPOINT ){
		usb_flush_endpoint(handle, attr->address);
	}

	if( o_flags & USB_FLAG_DISABLE_ENDPOINT ){
		usb_disable_endpoint(handle, attr->address);
	}

	if( o_flags & USB_FLAG_STALL_ENDPOINT ){
		usb_stall_endpoint(handle, attr->address);
	}

	if( o_flags & USB_FLAG_UNSTALL_ENDPOINT ){
		usb_unstall_endpoint(handle, attr->address);
	}

	if( o_flags & USB_FLAG_CONFIGURE_ENDPOINT ){
		usb_configure_endpoint(handle, attr->address, attr->max_packet_size, attr->type);
	}

	return 0;
}

void usb_connect(u32 port, u32 con){
	if( con ){
		//what is this delay waiting for
		//seems to be needed on STM32H7 between setattr and start
		cortexm_delay_ms(1);
		HAL_PCD_Start(&m_usb_local[port].hal_handle);

#if defined STM32H7
		HAL_PWREx_EnableUSBVoltageDetector();
#endif
	} else {
		HAL_PCD_Stop(&m_usb_local[port].hal_handle);
	}
}

int mcu_usb_setaction(const devfs_handle_t * handle, void * ctl){
	u32 port = handle->port;
	mcu_action_t * action = (mcu_action_t*)ctl;
	int log_ep;
	int ret = -1;

	//cortexm_set_irq_prio(USB_IRQn, action->prio);
	log_ep = action->channel & ~0x80;

	cortexm_set_irq_priority(usb_irqs[port], action->prio, action->o_events);

	if( action->o_events &
			(MCU_EVENT_FLAG_POWER|MCU_EVENT_FLAG_SUSPEND|MCU_EVENT_FLAG_STALL|MCU_EVENT_FLAG_SOF|MCU_EVENT_FLAG_WAKEUP)
			){
		m_usb_local[port].special_event_handler = action->handler;
		return 0;
	}

	if( action->channel & 0x80 ){
		if( (action->handler.callback == 0) && (action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE) ){
			m_usb_local[port].write_pending &= ~(1<<log_ep);
			if( log_ep > 0 ){
				devfs_execute_cancel_handler(&(m_usb_local[port].transfer_handler[log_ep]), 0, SYSFS_SET_RETURN(ECANCELED), MCU_EVENT_FLAG_CANCELED);
			} else {
				mcu_execute_event_handler(&(m_usb_local[port].control_handler), MCU_EVENT_FLAG_CANCELED, 0);
			}
		}
	} else {
		if( (action->handler.callback == 0) && (action->o_events & MCU_EVENT_FLAG_DATA_READY) ){
			m_usb_local[port].read_ready |= (1<<log_ep);
			if( log_ep > 0 ){
				devfs_execute_cancel_handler(&(m_usb_local[port].transfer_handler[log_ep]), 0, SYSFS_SET_RETURN(ECANCELED), MCU_EVENT_FLAG_CANCELED);
			} else {
				mcu_execute_event_handler(&(m_usb_local[port].control_handler), MCU_EVENT_FLAG_CANCELED, 0);
			}
		}
	}

	if ( (log_ep < DEV_USB_LOGICAL_ENDPOINT_COUNT)  ){
		if( action->o_events & MCU_EVENT_FLAG_DATA_READY ){
			//cortexm_enable_interrupts();
			if( cortexm_validate_callback(action->handler.callback) < 0 ){
				return SYSFS_SET_RETURN(EPERM);
			}

			if( log_ep == 0 ){
				if( action->o_events & MCU_EVENT_FLAG_SETUP ){
					m_usb_local[port].control_handler = action->handler;
				} else {
					return SYSFS_SET_RETURN(EINVAL);
				}
			} else {
				return SYSFS_SET_RETURN(EINVAL);
			}
			ret = 0;
		}

		if( action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE ){
			if( cortexm_validate_callback(action->handler.callback) < 0 ){
				return SYSFS_SET_RETURN(EPERM);
			}

			if( log_ep == 0 ){
				if( action->o_events & MCU_EVENT_FLAG_SETUP ){
					m_usb_local[port].control_handler = action->handler;
				} else {
					return SYSFS_SET_RETURN(EINVAL);
				}
			} else {
				return SYSFS_SET_RETURN(EINVAL);
			}
			ret = 0;
		}
	}

	if( ret < 0 ){
		ret = SYSFS_SET_RETURN(EINVAL);
	}
	return ret;
}

int mcu_usb_read(const devfs_handle_t * handle, devfs_async_t * async){
	DEVFS_DRIVER_DECLARE_LOCAL(usb, MCU_USB_PORTS);

	int loc = async->loc;
	if ( loc > (DEV_USB_LOGICAL_ENDPOINT_COUNT-1) ){
		return SYSFS_SET_RETURN(EINVAL);
	}

	DEVFS_DRIVER_IS_BUSY(local->transfer_handler[loc].read, async);

	int result;

	//Synchronous read (only if data is ready) otherwise 0 is returned
	if ( local->read_ready & (1<<loc) ){
		result = mcu_usb_root_read_endpoint(handle, loc, async->buf);
		if( result == 0 ){
			result = SYSFS_SET_RETURN(EAGAIN);
		}
	} else {
		if ( !(async->flags & O_NONBLOCK) ){
			//If this is a blocking call, set the callback and context
			if( cortexm_validate_callback(async->handler.callback) < 0 ){
				result = SYSFS_SET_RETURN(EPERM);
			} else {
				result = 0;
			}
		} else {
			result = SYSFS_SET_RETURN(EAGAIN);
		}
	}

	if( result != 0 ){
		local->transfer_handler[loc].read = 0;
	}

	return result;
}

int mcu_usb_write(const devfs_handle_t * handle, devfs_async_t * async){
	//Asynchronous write
	DEVFS_DRIVER_DECLARE_LOCAL(usb, MCU_USB_PORTS);

	int loc = async->loc;
	int ep;
	ep = (loc & 0x7F);
	if ( ep > (DEV_USB_LOGICAL_ENDPOINT_COUNT-1) ){
		return SYSFS_SET_RETURN(EINVAL);
	}

	DEVFS_DRIVER_IS_BUSY(local->transfer_handler[ep].write, async);

	int bytes_written;

	if( cortexm_validate_callback(async->handler.callback) < 0 ){
		return SYSFS_SET_RETURN(EPERM);
	}

	local->write_pending |= (1<<ep);
	bytes_written = mcu_usb_root_write_endpoint(handle, loc, async->buf, async->nbyte);

	if ( bytes_written < 0 ){
		usb_disable_endpoint(handle, loc );
		usb_reset_endpoint(handle, loc );
		usb_enable_endpoint(handle, loc );
	}

	if( bytes_written != 0 ){
		local->transfer_handler[loc].write = 0;
		m_usb_local[port].write_pending &= ~(1<<ep);
	}

	return bytes_written;
}


void usb_reset(const devfs_handle_t * handle){

}

void usb_wakeup(int port){

}

void usb_set_address(const devfs_handle_t * handle, u32 addr){
	HAL_PCD_SetAddress(&m_usb_local[handle->port].hal_handle, addr);
}

void usb_configure(const devfs_handle_t * handle, u32 cfg){
	//m_usb_local[handle->port].connected = 1;
}

void usb_configure_endpoint(const devfs_handle_t * handle, u32 endpoint_num, u32 max_packet_size, u8 type){
	u32 port = handle->port;
	const stm32_config_t * stm32_config = mcu_board_config.arch_config;

	HAL_PCD_EP_Open(&m_usb_local[handle->port].hal_handle, endpoint_num, max_packet_size, type & EP_TYPE_MSK);
	//m_usb_local[handle->port].connected = 1;

	if( (endpoint_num & 0x80) == 0 ){
		void * dest_buffer;

		m_usb_local[port].rx_buffer_offset[endpoint_num] = m_usb_local[port].rx_buffer_used;
		m_usb_local[port].rx_buffer_used += (max_packet_size*2);
		if( m_usb_local[port].rx_buffer_used > stm32_config->usb_rx_buffer_size ){
			//this is a fatal error -- using mcu_debug_ will cause bootloader link problems
			mcu_board_execute_event_handler(MCU_BOARD_CONFIG_EVENT_ROOT_FATAL, "usbbuf");
		}

		dest_buffer = stm32_config->usb_rx_buffer +
				m_usb_local[port].rx_buffer_offset[endpoint_num] +
				max_packet_size;

		HAL_PCD_EP_Receive(&m_usb_local[port].hal_handle, endpoint_num, dest_buffer, max_packet_size);
	}
}

void usb_enable_endpoint(const devfs_handle_t * handle, u32 endpoint_num){
}

void usb_disable_endpoint(const devfs_handle_t * handle, u32 endpoint_num){
	HAL_PCD_EP_Close(&m_usb_local[handle->port].hal_handle, endpoint_num);
}

void usb_reset_endpoint(const devfs_handle_t * handle, u32 endpoint_num){
}

void usb_flush_endpoint(const devfs_handle_t * handle, u32 endpoint_num){
	PCD_HandleTypeDef * hpcd = &m_usb_local[handle->port].hal_handle;
	u8 logical_endpoint = endpoint_num & ~0x80;

	if( ((endpoint_num & 0x80) && (hpcd->IN_ep[logical_endpoint].type == EP_TYPE_ISOC)) ){
		HAL_PCD_EP_Close(hpcd, endpoint_num);
		HAL_PCD_EP_Open(hpcd, endpoint_num, hpcd->IN_ep[logical_endpoint].maxpacket, EP_TYPE_ISOC);
	} else if( (((endpoint_num & 0x80) == 0) && (hpcd->OUT_ep[logical_endpoint].type == EP_TYPE_ISOC)) ){
		HAL_PCD_EP_Close(hpcd, endpoint_num);
		HAL_PCD_EP_Open(hpcd, endpoint_num, hpcd->OUT_ep[logical_endpoint].maxpacket, EP_TYPE_ISOC);
	}
	HAL_PCD_EP_Flush(hpcd, endpoint_num);

}

void usb_stall_endpoint(const devfs_handle_t * handle, u32 endpoint_num){
	HAL_PCD_EP_SetStall(&m_usb_local[handle->port].hal_handle, endpoint_num);
}

void usb_unstall_endpoint(const devfs_handle_t * handle, u32 endpoint_num){
	HAL_PCD_EP_ClrStall(&m_usb_local[handle->port].hal_handle, endpoint_num);
}

int mcu_usb_isconnected(const devfs_handle_t * handle, void * ctl){
	return m_usb_local[handle->port].connected;
}

void usb_clr_ep_buf(const devfs_handle_t * handle, u32 endpoint_num){

}

int mcu_usb_root_read_endpoint(const devfs_handle_t * handle, u32 endpoint_num, void * dest){
	u32 port = handle->port;
	const stm32_config_t * stm32_config = mcu_board_config.arch_config;
	void * src_buffer;
	u8 epnum;
	epnum = endpoint_num & 0x7f;

	if( m_usb_local[port].read_ready & (1<<epnum) ){
		m_usb_local[port].read_ready &= ~(1<<epnum);
		src_buffer = stm32_config->usb_rx_buffer + m_usb_local[port].rx_buffer_offset[epnum];
		//data is copied from fifo to buffer during the interrupt
		memcpy(dest, src_buffer, m_usb_local[port].rx_count[epnum]);
		return m_usb_local[port].rx_count[epnum];
	}

	return -1;
}

int mcu_usb_root_write_endpoint(const devfs_handle_t * handle, u32 endpoint_num, const void * src, u32 size){
	int ret;

#if MCU_USB_API > 0
	int logical_endpoint = endpoint_num & 0x7f;
	int type = m_usb_local[handle->port].hal_handle.IN_ep[logical_endpoint].type;
	if( type == EP_TYPE_ISOC ){
		//check to see if the packet will fit in the FIFO
		//if the packet won't fit, return EBUSY
#if !defined STM32H7
		USB_OTG_GlobalTypeDef * USBx = m_usb_local[handle->port].hal_handle.Instance;
		int available = (USBx_INEP(logical_endpoint)->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV);
		if( (available * 4) < size ){
			return SYSFS_SET_RETURN(EBUSY);
		}
#endif
	}
#endif

	ret = HAL_PCD_EP_Transmit(&m_usb_local[handle->port].hal_handle, endpoint_num, (void*)src, size);
	if( ret == HAL_OK ){
#if 0
		if( type == EP_TYPE_ISOC ){
			return size;
		}
#endif
		return 0;
	}

	return SYSFS_SET_RETURN(EIO);
}


void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd){
	usb_local_t * local = (usb_local_t *)hpcd;
	//a setup packet has been received
	const stm32_config_t * stm32_config = mcu_board_config.arch_config;
	usb_event_t event;
	event.epnum = 0;
	void * dest_buffer;
	usbd_setup_packet_t * setup = (usbd_setup_packet_t *)&hpcd->Setup;

	//Setup data is in hpcd->Setup buffer at this point

	//copy setup data to ep0 data buffer
	local->read_ready |= (1<<0);
	dest_buffer = stm32_config->usb_rx_buffer + local->rx_buffer_offset[0];
	local->rx_count[0] = sizeof(usbd_setup_packet_t);
	memcpy(dest_buffer, hpcd->Setup, local->rx_count[0]);

	mcu_execute_event_handler(&local->control_handler, MCU_EVENT_FLAG_SETUP, &event);

	//prepare EP zero for receiving out data
	if( (setup->bmRequestType.bitmap_t.dir == USBD_REQUEST_TYPE_DIRECTION_HOST_TO_DEVICE) && (setup->wLength > 0) ){
		HAL_PCD_EP_Receive(hpcd, 0, stm32_config->usb_rx_buffer, setup->wLength);
	}


}

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum){
	usb_local_t * local = (usb_local_t *)hpcd;
	//data has already been received and is stored in buffer specified by HAL_PCD_EP_Receive
	const stm32_config_t * stm32_config = mcu_board_config.arch_config;
	usb_event_t event;
	event.epnum = epnum;
	u16 count;
	void * src_buffer;
	void * dest_buffer;

	//set read ready flag
	count = HAL_PCD_EP_GetRxCount(&local->hal_handle, epnum);
	dest_buffer = stm32_config->usb_rx_buffer + local->rx_buffer_offset[epnum];
	src_buffer = dest_buffer + hpcd->OUT_ep[epnum].maxpacket;
	memcpy(dest_buffer, src_buffer, count); //free up the source buffer

	//prepare to receive the next packet in the local buffer for non-control endpoints
	if( epnum != 0 ){
		HAL_PCD_EP_Receive(hpcd, epnum, src_buffer, hpcd->OUT_ep[epnum].maxpacket);
	}

	local->rx_count[epnum] = count;
	if( count > 0 ){
		if( epnum == 0 ){
			//memcpy(dest_buffer, src_buffer, count);
			local->read_ready |= (1<<epnum);
			mcu_execute_event_handler(&local->control_handler, MCU_EVENT_FLAG_DATA_READY, &event);
		} else if( local->transfer_handler[epnum].read ){
			devfs_async_t * async = local->transfer_handler[epnum].read;
			if( count > async->nbyte ){ count = async->nbyte; }
			//copy directly to the async buffer that is waiting for data
			memcpy(local->transfer_handler[epnum].read->buf, dest_buffer, count);
			local->read_ready &= ~(1<<epnum);
			devfs_execute_read_handler(
						local->transfer_handler + epnum,
						&event,
						count,
						MCU_EVENT_FLAG_DATA_READY
						);
		} else {
			//copy to buffer the root read function uses
			//memcpy(dest_buffer, src_buffer, count);
			local->read_ready |= (1<<epnum);
		}
	}
}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum){
	usb_local_t * local = (usb_local_t *)hpcd;
	u8 logical_ep = epnum & 0x7f;
	usb_event_t event;
	event.epnum = epnum;

	local->write_pending &= ~(1<<logical_ep);

	//devfs_execute_write_handler(usb->transfer_handlers + logical_ep, &event, 0, MCU_EVENT_FLAG_WRITE_COMPLETE);

	if( logical_ep == 0 ){
		mcu_execute_event_handler(&local->control_handler, MCU_EVENT_FLAG_WRITE_COMPLETE, &event);

		//only proceed it DataIn tx'd more than zero bytes
		if( hpcd->IN_ep[0].xfer_count == 0 ){
			return;
		}

		//ep 0 data in complete
		//prepare EP0 for next setup packet
		HAL_PCD_EP_Receive(hpcd, 0, 0, 0);
	} else {
		devfs_execute_write_handler(local->transfer_handler + logical_ep, &event, 0, 0);
	}
}

void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd){

}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd){
	int i;
	usb_local_t * usb = (usb_local_t *)hpcd;
	u32 mps = mcu_board_config.usb_max_packet_zero;
	usb->connected = 1;
	usb->rx_buffer_used = mps;
	for(i=0; i < DEV_USB_LOGICAL_ENDPOINT_COUNT; i++){
		usb->rx_buffer_offset[i] = 0;
	}

	HAL_PCD_SetAddress(hpcd, 0);

	HAL_PCD_EP_Open(hpcd, 0x00, mps, EP_TYPE_CTRL);
	HAL_PCD_EP_Open(hpcd, 0x80, mps, EP_TYPE_CTRL);

}

void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd){
	usb_local_t * usb = (usb_local_t *)hpcd;
	usb->connected = 0;

#if MCU_USB_API > 0
	__HAL_PCD_GATE_PHYCLOCK(hpcd);
#endif

#if 0
	//this causes problems when USB cable is removed -- don't do it
	HAL_PCD_EP_Close(hpcd, 0x00);
	HAL_PCD_EP_Close(hpcd, 0x80);
#endif

}

void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd){
	usb_local_t * usb = (usb_local_t *)hpcd;
	usb->connected = 1;
}

void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum){

	//epnum value passed is not valid -- need to find it
	int i;
	for(i=1; i < DEV_USB_LOGICAL_ENDPOINT_COUNT; i++){
		if( hpcd->IN_ep[i].type == EP_TYPE_ISOC ){

			epnum = 0x80 | i;
			u8 logical_ep = epnum & 0x7F;

			//Close will disable the endpoint and flush the TX FIFO
			if( HAL_PCD_EP_Close(hpcd, epnum) != HAL_OK ){

			}

			if( HAL_PCD_EP_Open(hpcd, epnum, hpcd->IN_ep[logical_ep].maxpacket, EP_TYPE_ISOC) != HAL_OK ){

			}

			usb_local_t * local = (usb_local_t *)hpcd;
			devfs_execute_write_handler(local->transfer_handler + logical_ep, 0, SYSFS_SET_RETURN(EIO), MCU_EVENT_FLAG_ERROR | MCU_EVENT_FLAG_CANCELED);
		}

	}
}

void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum){

}

void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd){
	MCU_UNUSED_ARGUMENT(hpcd);
	//this is never called -- Reset callback is called when connection is established
}

void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd){
	MCU_UNUSED_ARGUMENT(hpcd);
	//this is never called -- Suspend callback is called when connection is lost
}

void mcu_core_otg_fs_isr(){
	HAL_PCD_IRQHandler(&m_usb_local[0].hal_handle);
}

#if MCU_USB_PORTS > 1
void mcu_core_otg_hs_isr(){
	HAL_PCD_IRQHandler(&m_usb_local[1].hal_handle);
}
#endif

#endif








