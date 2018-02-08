/* Copyright 2011-2016 Tyler Gilbert; 
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

#include <errno.h>
#include <fcntl.h>
#include <mcu/usb.h>
#include <mcu/pio.h>
#include <cortexm/cortexm.h>
#include <usbd/types.h>
#include <mcu/core.h>
#include <mcu/debug.h>
#include <mcu/boot_debug.h>

#include "stm32_local.h"


#if MCU_USB_PORTS > 0

static void usb_connect(u32 con);
static inline void usb_configure(const devfs_handle_t * handle, u32 cfg) MCU_ALWAYS_INLINE;
static inline void usb_set_address(const devfs_handle_t * handle, u32 addr) MCU_ALWAYS_INLINE;
static inline void usb_reset_endpoint(const devfs_handle_t * handle, u32 endpoint_num) MCU_ALWAYS_INLINE;
static inline void usb_enable_endpoint(const devfs_handle_t * handle, u32 endpoint_num) MCU_ALWAYS_INLINE;
static inline void usb_disable_endpoint(const devfs_handle_t * handle, u32 endpoint_num) MCU_ALWAYS_INLINE;
static inline void usb_stall_endpoint(const devfs_handle_t * handle, u32 endpoint_num) MCU_ALWAYS_INLINE;
static inline void usb_unstall_endpoint(const devfs_handle_t * handle, u32 endpoint_num) MCU_ALWAYS_INLINE;
static inline void usb_configure_endpoint(const devfs_handle_t * handle, u32 endpoint_num, u32 max_packet_size, u8 type) MCU_ALWAYS_INLINE;
static inline void usb_reset(const devfs_handle_t * handle);


typedef struct MCU_PACK {
	PCD_HandleTypeDef hal_handle;
	mcu_event_handler_t write[DEV_USB_LOGICAL_ENDPOINT_COUNT];
	mcu_event_handler_t read[DEV_USB_LOGICAL_ENDPOINT_COUNT];
	u16 rx_count[DEV_USB_LOGICAL_ENDPOINT_COUNT];
	u16 rx_buffer_offset[DEV_USB_LOGICAL_ENDPOINT_COUNT];
	u16 rx_buffer_used;
	volatile u32 write_pending;
	volatile u32 read_ready;
	mcu_event_handler_t special_event_handler;
	u8 ref_count;
	u8 connected;
} usb_local_t;

static usb_local_t usb_local MCU_SYS_MEM;

static USB_OTG_GlobalTypeDef * const usb_regs_table[MCU_USB_PORTS] = MCU_USB_REGS;
static u8 const usb_irqs[MCU_USB_PORTS] = MCU_USB_IRQS;
static void clear_callbacks();

void clear_callbacks(){
	memset(usb_local.write, 0, DEV_USB_LOGICAL_ENDPOINT_COUNT * sizeof(mcu_event_handler_t));
	memset(usb_local.read, 0, DEV_USB_LOGICAL_ENDPOINT_COUNT * sizeof(mcu_event_handler_t));
	memset(&usb_local.special_event_handler, 0, sizeof(mcu_event_handler_t));
	memset(usb_local.rx_buffer_offset, 0, DEV_USB_LOGICAL_ENDPOINT_COUNT * sizeof(u16));
	usb_local.rx_buffer_used = 0;

}

int mcu_usb_open(const devfs_handle_t * handle){
	if ( usb_local.ref_count == 0 ){
		//Set callbacks to NULL
		usb_local.connected = 0;
		clear_callbacks();
		usb_local.special_event_handler.callback = 0;
		usb_local.hal_handle.Instance = usb_regs_table[0];
		__HAL_RCC_USB_OTG_FS_CLK_ENABLE();
		cortexm_enable_irq((void*)(u32)(usb_irqs[0]));  //Enable USB IRQ
	}
	usb_local.ref_count++;
    return 0;
}

int mcu_usb_close(const devfs_handle_t * handle){
	if ( usb_local.ref_count > 0 ){
		if ( usb_local.ref_count == 1 ){
			HAL_PCD_Stop(&usb_local.hal_handle);
			cortexm_disable_irq((void*)(u32)(usb_irqs[0]));  //Disable the USB interrupt
			usb_local.hal_handle.Instance = 0;
			__HAL_RCC_USB_OTG_FS_CLK_DISABLE();
		}
		usb_local.ref_count--;
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
	int port = handle->port;
    int i;

	const usb_attr_t * attr = mcu_select_attr(handle, ctl);
	if( attr == 0 ){
		return -1;
	}
	u32 o_flags = attr->o_flags;

	if( o_flags & USB_FLAG_SET_DEVICE ){
		//Start the USB clock
		mcu_core_setusbclock(attr->freq);

		usb_local.read_ready = 0;
		usb_local.write_pending = 0;

		if( mcu_set_pin_assignment(
				&(attr->pin_assignment),
				MCU_CONFIG_PIN_ASSIGNMENT(usb_config_t, handle),
				MCU_PIN_ASSIGNMENT_COUNT(usb_pin_assignment_t),
                CORE_PERIPH_USB, port, 0, 0, 0) < 0 ){
			return -1;
		}

		usb_local.hal_handle.Init.dev_endpoints = DEV_USB_LOGICAL_ENDPOINT_COUNT;
		usb_local.hal_handle.Init.speed = PCD_SPEED_FULL;
		usb_local.hal_handle.Init.dma_enable = DISABLE;
		usb_local.hal_handle.Init.ep0_mps = attr->max_packet_size;
		usb_local.hal_handle.Init.phy_itface = PCD_PHY_EMBEDDED;

		usb_local.hal_handle.Init.Sof_enable = DISABLE;
		usb_local.hal_handle.Init.low_power_enable = DISABLE;
		usb_local.hal_handle.Init.lpm_enable = DISABLE;
		usb_local.hal_handle.Init.vbus_sensing_enable = DISABLE;
		usb_local.hal_handle.Init.use_dedicated_ep1 = DISABLE;

		if( attr->o_flags & USB_FLAG_IS_SOF_ENABLED ){
			usb_local.hal_handle.Init.Sof_enable = ENABLE;
		}

		if( attr->o_flags & USB_FLAG_IS_LOW_POWER_MODE_ENABLED ){
			usb_local.hal_handle.Init.lpm_enable = ENABLE;
		}

		if( attr->o_flags & USB_FLAG_IS_VBUS_SENSING_ENABLED ){
			usb_local.hal_handle.Init.vbus_sensing_enable = ENABLE;
		}


        HAL_PCD_Init(&usb_local.hal_handle);

        //this needs an MCU definition to define the number of endpoints and the amount of RAM available 1.25K for at least some STM32F4xx
		//these need to be pulled from MCU FIFO
#if 0
		HAL_PCDEx_SetRxFiFo(&usb_local.hal_handle, 128);  //size is in 32-bit words for all fifo - 512
		HAL_PCDEx_SetTxFiFo(&usb_local.hal_handle, 0, 32); //128 / 640
		HAL_PCDEx_SetTxFiFo(&usb_local.hal_handle, 1, 32); //128 / 768
		HAL_PCDEx_SetTxFiFo(&usb_local.hal_handle, 2, 32); //128 / 896
		HAL_PCDEx_SetTxFiFo(&usb_local.hal_handle, 3, 64); //256 / 1152
#endif
        HAL_PCDEx_SetRxFiFo(&usb_local.hal_handle, attr->rx_fifo_word_size);  //size is in 32-bit words for all fifo - 512
        for(i=0; i < USB_TX_FIFO_WORD_SIZE_COUNT; i++){
            if( attr->tx_fifo_word_size[i] > 0 ){
                HAL_PCDEx_SetTxFiFo(&usb_local.hal_handle, i, attr->tx_fifo_word_size[i]);
            }
        }

		HAL_PCD_Start(&usb_local.hal_handle);
	}

	if( o_flags & USB_FLAG_RESET ){ usb_reset(handle); }
	if( o_flags & USB_FLAG_ATTACH ){ usb_connect(1); }
	if( o_flags & USB_FLAG_DETACH ){ usb_connect(0); }
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

void usb_connect(u32 con){
	if( con ){
		HAL_PCD_DevConnect(&usb_local.hal_handle);
	} else {
		HAL_PCD_DevDisconnect(&usb_local.hal_handle);
	}
}

int mcu_usb_setaction(const devfs_handle_t * handle, void * ctl){
	mcu_action_t * action = (mcu_action_t*)ctl;
	int log_ep;
	int ret = -1;

	//cortexm_set_irq_prio(USB_IRQn, action->prio);
	log_ep = action->channel & ~0x80;

	if( action->o_events &
			(MCU_EVENT_FLAG_POWER|MCU_EVENT_FLAG_SUSPEND|MCU_EVENT_FLAG_STALL|MCU_EVENT_FLAG_SOF|MCU_EVENT_FLAG_WAKEUP)
	){
		usb_local.special_event_handler = action->handler;
		return 0;
	}


	if( action->channel & 0x80 ){
		if( (action->handler.callback == 0) && (action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE) ){
			usb_local.write_pending &= ~(1<<log_ep);
			mcu_execute_event_handler(&(usb_local.write[log_ep]), MCU_EVENT_FLAG_CANCELED, 0);
		}
	} else {
		if( (action->handler.callback == 0) && (action->o_events & MCU_EVENT_FLAG_DATA_READY) ){
			usb_local.read_ready |= (1<<log_ep);
			mcu_execute_event_handler(&(usb_local.read[log_ep]), MCU_EVENT_FLAG_CANCELED, 0);
		}
	}


	if ( (log_ep < DEV_USB_LOGICAL_ENDPOINT_COUNT)  ){
		if( action->o_events & MCU_EVENT_FLAG_DATA_READY ){
			//cortexm_enable_interrupts(NULL);
			if( cortexm_validate_callback(action->handler.callback) < 0 ){
				return -1;
			}

			usb_local.read[log_ep].callback = action->handler.callback;
			usb_local.read[log_ep].context = action->handler.context;
			ret = 0;
		}

		if( action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE ){
			if( cortexm_validate_callback(action->handler.callback) < 0 ){
				return -1;
			}

			usb_local.write[log_ep].callback = action->handler.callback;
			usb_local.write[log_ep].context = action->handler.context;
			ret = 0;
		}
	}

	if( ret < 0 ){
		errno = EINVAL;
	}
	return ret;
}

int mcu_usb_read(const devfs_handle_t * handle, devfs_async_t * rop){
	int ret;
	int loc = rop->loc;

	if ( loc > (DEV_USB_LOGICAL_ENDPOINT_COUNT-1) ){
		errno = EINVAL;
		return -1;
	}

	if( usb_local.read[loc].callback ){
		errno = EBUSY;
		return -1;
	}

	//Synchronous read (only if data is ready) otherwise 0 is returned
	if ( usb_local.read_ready & (1<<loc) ){
		ret = mcu_usb_root_read_endpoint(0, loc, rop->buf);
	} else {
		rop->nbyte = 0;
		if ( !(rop->flags & O_NONBLOCK) ){
			//If this is a blocking call, set the callback and context
			if( cortexm_validate_callback(rop->handler.callback) < 0 ){
				return -1;
			}

			usb_local.read[loc].callback = rop->handler.callback;
			usb_local.read[loc].context = rop->handler.context;
			ret = 0;
		} else {
			errno = EAGAIN;
			ret = -1;
		}
	}

	return ret;
}

int mcu_usb_write(const devfs_handle_t * handle, devfs_async_t * wop){
	//Asynchronous write
	int ep;
	int loc = wop->loc;


	ep = (loc & 0x7F);

	if ( ep > (DEV_USB_LOGICAL_ENDPOINT_COUNT-1) ){
		errno = EINVAL;
		return -1;
	}

	if ( usb_local.write[ep].callback ){
		errno = EBUSY;
		return -1;
	}

	usb_local.write_pending |= (1<<ep);

	if( cortexm_validate_callback(wop->handler.callback) < 0 ){
		return -1;
	}

	usb_local.write[ep].callback = wop->handler.callback;
	usb_local.write[ep].context = wop->handler.context;

	wop->nbyte = mcu_usb_root_write_endpoint(handle, loc, wop->buf, wop->nbyte);

	if ( wop->nbyte < 0 ){
		usb_disable_endpoint(handle, loc );
		usb_reset_endpoint(handle, loc );
		usb_enable_endpoint(handle, loc );
		usb_local.write_pending &= ~(1<<ep);
		return -2;
	}

	return 0;
}


void usb_reset(const devfs_handle_t * handle){

}

void usb_wakeup(int port){

}

void usb_set_address(const devfs_handle_t * handle, u32 addr){
	HAL_PCD_SetAddress(&usb_local.hal_handle, addr);

}

void usb_configure(const devfs_handle_t * handle, u32 cfg){
	usb_local.connected = 1;
}

void usb_configure_endpoint(const devfs_handle_t * handle, u32 endpoint_num, u32 max_packet_size, u8 type){
	HAL_PCD_EP_Open(&usb_local.hal_handle, endpoint_num, max_packet_size, type & EP_TYPE_MSK);

	if( (endpoint_num & 0x80) == 0 ){
		void * dest_buffer;

		usb_local.rx_buffer_offset[endpoint_num] = usb_local.rx_buffer_used;
		usb_local.rx_buffer_used += (max_packet_size*2);
		if( usb_local.rx_buffer_used > mcu_board_config.usb_rx_buffer_size ){
			//this is a fatal error
			mcu_board_execute_event_handler(MCU_BOARD_CONFIG_EVENT_ROOT_FATAL, "usbbuf");
		}

		dest_buffer = mcu_board_config.usb_rx_buffer +
				usb_local.rx_buffer_offset[endpoint_num] +
				max_packet_size;

		HAL_PCD_EP_Receive(&usb_local.hal_handle, endpoint_num, dest_buffer, max_packet_size);

	}
}

void usb_enable_endpoint(const devfs_handle_t * handle, u32 endpoint_num){
}

void usb_disable_endpoint(const devfs_handle_t * handle, u32 endpoint_num){
	HAL_PCD_EP_Close(&usb_local.hal_handle, endpoint_num);
}

void usb_reset_endpoint(const devfs_handle_t * handle, u32 endpoint_num){
}

void usb_stall_endpoint(const devfs_handle_t * handle, u32 endpoint_num){
	HAL_PCD_EP_SetStall(&usb_local.hal_handle, endpoint_num);
}

void usb_unstall_endpoint(const devfs_handle_t * handle, u32 endpoint_num){
	HAL_PCD_EP_ClrStall(&usb_local.hal_handle, endpoint_num);
}

int mcu_usb_isconnected(const devfs_handle_t * handle, void * ctl){
	return usb_local.connected;
}

void usb_clr_ep_buf(const devfs_handle_t * handle, u32 endpoint_num){

}

int mcu_usb_root_read_endpoint(const devfs_handle_t * handle, u32 endpoint_num, void * dest){
	void * src_buffer;
	u8 epnum;
	epnum = endpoint_num & 0x7f;

	if( 	usb_local.read_ready & (1<<epnum) ){
		usb_local.read_ready &= ~(1<<epnum);
		src_buffer = mcu_board_config.usb_rx_buffer + usb_local.rx_buffer_offset[epnum];
		//data is copied from fifo to buffer during the interrupt
		memcpy(dest, src_buffer, usb_local.rx_count[epnum]);
		return usb_local.rx_count[epnum];
	}

	return -1;
}



int mcu_usb_root_write_endpoint(const devfs_handle_t * handle, u32 endpoint_num, const void * src, u32 size){
	int ret;
	ret = HAL_PCD_EP_Transmit(&usb_local.hal_handle, endpoint_num, (void*)src, size);
	if( ret == HAL_OK ){
		return size;
	}
	return -1;
}


void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd){
	//a setup packet has been received
	usb_event_t event;
	event.epnum = 0;
	void * dest_buffer;

	//Setup data is in hpcd->Setup buffer at this point

	//copy setup data to ep0 data buffer
	usb_local.read_ready |= (1<<0);
	dest_buffer = mcu_board_config.usb_rx_buffer + usb_local.rx_buffer_offset[0];
	usb_local.rx_count[0] = sizeof(usbd_setup_packet_t);
	memcpy(dest_buffer, hpcd->Setup, usb_local.rx_count[0]);
	//mcu_debug_root_printf("Setup\n");

	mcu_execute_event_handler(usb_local.read + 0, MCU_EVENT_FLAG_SETUP, &event);

	//prepare EP zero for receiving out data
	HAL_PCD_EP_Receive(hpcd, 0, mcu_board_config.usb_rx_buffer, hpcd->OUT_ep[0].maxpacket);
}

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum){
	//data has already been received and is stored in buffer specified by HAL_PCD_EP_Receive
	usb_event_t event;
	event.epnum = epnum;
	u16 count;
	void * src_buffer;
	void * dest_buffer;

	//set read ready flag
	usb_local.read_ready |= (1<<epnum);
	count = HAL_PCD_EP_GetRxCount(&usb_local.hal_handle, epnum);

	dest_buffer = mcu_board_config.usb_rx_buffer + usb_local.rx_buffer_offset[epnum];
	src_buffer = dest_buffer + hpcd->OUT_ep[epnum].maxpacket;

	memcpy(dest_buffer, src_buffer, count);
	usb_local.rx_count[epnum] = count;

	//mcu_debug_root_printf("Data out %d 0x%lX %d\n", epnum, (u32)usb_local.read[epnum].callback, count);
	mcu_execute_event_handler(usb_local.read + epnum, MCU_EVENT_FLAG_DATA_READY, &event);

	//prepare to receive the next packet in the local buffer
	HAL_PCD_EP_Receive(hpcd, epnum, src_buffer, hpcd->OUT_ep[epnum].maxpacket);

}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum){
	u8 logical_ep = epnum & 0x7f;
	usb_event_t event;
	event.epnum = epnum;


	//mcu_debug_root_printf("Data in %d 0x%lX\n", logical_ep, (u32)usb_local.read[logical_ep].callback);

	mcu_execute_event_handler(usb_local.write + logical_ep, MCU_EVENT_FLAG_WRITE_COMPLETE, &event);

	if( (epnum & 0x7f) == 0 ){
		//ep 0 data in complete
		//prepare EP0 for next setup packet
		HAL_PCD_EP_Receive(hpcd, 0, 0, 0);
	}

}

void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd){

}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd){
	int i;
	u32 mps = mcu_board_config.usb_max_packet_zero;

	usb_local.rx_buffer_used = mps;
	for(i=0; i < DEV_USB_LOGICAL_ENDPOINT_COUNT; i++){
		usb_local.rx_buffer_offset[i] = 0;
	}

	HAL_PCD_EP_Open(hpcd, 0x00, mps, 0);
	HAL_PCD_EP_Open(hpcd, 0x80, mps, 0);
}

void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd){
	__HAL_PCD_GATE_PHYCLOCK(hpcd);
}

void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd){

}

void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum){

}

void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum){

}

void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd){
	usb_local.connected = 1;
	//execute special event handler
}

void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd){
	usb_local.connected = 0;
}

void mcu_core_otg_fs_isr(){
	HAL_PCD_IRQHandler(&usb_local.hal_handle);
}

#endif








