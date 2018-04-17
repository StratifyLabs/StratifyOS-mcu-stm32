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

#include "stm32_local.h"


#if MCU_USB_PORTS > 0

static void usb_connect(u32 port, u32 con);
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

static usb_local_t usb_local[MCU_USB_PORTS] MCU_SYS_MEM;

static USB_OTG_GlobalTypeDef * const usb_regs_table[MCU_USB_PORTS] = MCU_USB_REGS;
static u8 const usb_irqs[MCU_USB_PORTS] = MCU_USB_IRQS;
static void clear_callbacks();

void clear_callbacks(int port){
    memset(usb_local[port].write, 0, DEV_USB_LOGICAL_ENDPOINT_COUNT * sizeof(mcu_event_handler_t));
    memset(usb_local[port].read, 0, DEV_USB_LOGICAL_ENDPOINT_COUNT * sizeof(mcu_event_handler_t));
    memset(&usb_local[port].special_event_handler, 0, sizeof(mcu_event_handler_t));
    memset(usb_local[port].rx_buffer_offset, 0, DEV_USB_LOGICAL_ENDPOINT_COUNT * sizeof(u16));
    usb_local[port].rx_buffer_used = 0;
}

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(usb, USB_VERSION, I_MCU_TOTAL + I_USB_TOTAL, mcu_usb_isconnected)


int mcu_usb_open(const devfs_handle_t * handle){
    u32 port = handle->port;
    if ( usb_local[port].ref_count == 0 ){
        //Set callbacks to NULL
        usb_local[port].connected = 0;
        clear_callbacks(handle->port);
        usb_local[port].hal_handle.Instance = usb_regs_table[port];

        if( port == 0 ){
            __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
        } else {
#if MCU_USB_PORTS > 1
            __HAL_RCC_OTGPHYC_CLK_ENABLE();
            __HAL_RCC_USB_OTG_HS_CLK_ENABLE();
            __HAL_RCC_USB_OTG_HS_ULPI_CLK_ENABLE();
#endif
        }
        cortexm_enable_irq((void*)(u32)(usb_irqs[port]));  //Enable USB IRQ
    }
    usb_local[port].ref_count++;
    return 0;
}

int mcu_usb_close(const devfs_handle_t * handle){
    u32 port = handle->port;
    if ( usb_local[port].ref_count > 0 ){
        if ( usb_local[port].ref_count == 1 ){
            HAL_PCD_Stop(&usb_local[port].hal_handle);
            cortexm_disable_irq((void*)(u32)(usb_irqs[port]));  //Disable the USB interrupt
            usb_local[port].hal_handle.Instance = 0;
            if( port == 0 ){
                __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
            } else {
#if MCU_USB_PORTS > 1
                __HAL_RCC_OTGPHYC_CLK_DISABLE();
                __HAL_RCC_USB_OTG_HS_CLK_DISABLE();
                __HAL_RCC_USB_OTG_HS_ULPI_CLK_DISABLE();
#endif
            }
        }
        usb_local[port].ref_count--;
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
    int i;

    const usb_attr_t * attr = mcu_select_attr(handle, ctl);
    if( attr == 0 ){
        return -1;
    }
    u32 o_flags = attr->o_flags;

    if( o_flags & USB_FLAG_SET_DEVICE ){
        //Start the USB clock
        mcu_core_setusbclock(attr->freq);

        usb_local[port].read_ready = 0;
        usb_local[port].write_pending = 0;

        if( mcu_set_pin_assignment(
                    &(attr->pin_assignment),
                    MCU_CONFIG_PIN_ASSIGNMENT(usb_config_t, handle),
                    MCU_PIN_ASSIGNMENT_COUNT(usb_pin_assignment_t),
                    CORE_PERIPH_USB, port, 0, 0, 0) < 0 ){
            return -1;
        }

        if( port == 0 ){
            usb_local[port].hal_handle.Init.dev_endpoints = 6;
            usb_local[port].hal_handle.Init.speed = PCD_SPEED_FULL;
        } else {
#if MCU_USB_PORTS > 1
            usb_local[port].hal_handle.Init.dev_endpoints = DEV_USB_LOGICAL_ENDPOINT_COUNT;
            if( o_flags & USB_FLAG_IS_HIGH_SPEED ){
                usb_local[port].hal_handle.Init.speed = USB_OTG_SPEED_HIGH;
            } else {
                usb_local[port].hal_handle.Init.speed = USB_OTG_SPEED_HIGH_IN_FULL;
            }

            //Need a flag to check for HW interface
            usb_local[port].hal_handle.Init.phy_itface = USB_OTG_HS_EMBEDDED_PHY;
#endif
        }
        usb_local[port].hal_handle.Init.dma_enable = DISABLE;
        usb_local[port].hal_handle.Init.ep0_mps = DEP0CTL_MPS_64;
        if ( attr->max_packet_size <= 8 ){
            usb_local[port].hal_handle.Init.ep0_mps = DEP0CTL_MPS_8;
        } else if ( attr->max_packet_size <= 16 ){
            usb_local[port].hal_handle.Init.ep0_mps = DEP0CTL_MPS_16;
        } else if ( attr->max_packet_size <= 32 ){
            usb_local[port].hal_handle.Init.ep0_mps = DEP0CTL_MPS_32;
        }

        usb_local[port].hal_handle.Init.Sof_enable = DISABLE;
        usb_local[port].hal_handle.Init.low_power_enable = DISABLE;
        usb_local[port].hal_handle.Init.lpm_enable = DISABLE;
        usb_local[port].hal_handle.Init.vbus_sensing_enable = DISABLE;
        usb_local[port].hal_handle.Init.use_dedicated_ep1 = DISABLE;
        usb_local[port].hal_handle.Init.battery_charging_enable = DISABLE;

        if( o_flags & USB_FLAG_IS_SOF_ENABLED ){
            usb_local[port].hal_handle.Init.Sof_enable = ENABLE;
        }

        if( o_flags & USB_FLAG_IS_LOW_POWER_MODE_ENABLED ){
            usb_local[port].hal_handle.Init.lpm_enable = ENABLE;
        }

        if( o_flags & USB_FLAG_IS_VBUS_SENSING_ENABLED ){
            usb_local[port].hal_handle.Init.vbus_sensing_enable = ENABLE;
        }

        if( HAL_PCD_Init(&usb_local[port].hal_handle) != HAL_OK ){
            return -1;
        }

        HAL_PCDEx_SetRxFiFo(&usb_local[port].hal_handle, attr->rx_fifo_word_size);  //size is in 32-bit words for all fifo - 512
        for(i=0; i < USB_TX_FIFO_WORD_SIZE_COUNT; i++){
            if( attr->tx_fifo_word_size[i] > 0 ){
                HAL_PCDEx_SetTxFiFo(&usb_local[port].hal_handle, i, attr->tx_fifo_word_size[i]);
            }
        }

        if( HAL_PCD_Start(&usb_local[port].hal_handle) != HAL_OK ){
            return -1;
        }
    }

    if( o_flags & USB_FLAG_RESET ){ usb_reset(handle); }
    if( o_flags & USB_FLAG_ATTACH ){ usb_connect(port, 1); }
    if( o_flags & USB_FLAG_DETACH ){ usb_connect(port, 0); }
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

void usb_connect(u32 port, u32 con){
    if( con ){
        HAL_PCD_DevConnect(&usb_local[port].hal_handle);
    } else {
        HAL_PCD_DevDisconnect(&usb_local[port].hal_handle);
    }
}

int mcu_usb_setaction(const devfs_handle_t * handle, void * ctl){
    u32 port = handle->port;
    mcu_action_t * action = (mcu_action_t*)ctl;
    int log_ep;
    int ret = -1;

    //cortexm_set_irq_prio(USB_IRQn, action->prio);
    log_ep = action->channel & ~0x80;

    if( action->o_events &
            (MCU_EVENT_FLAG_POWER|MCU_EVENT_FLAG_SUSPEND|MCU_EVENT_FLAG_STALL|MCU_EVENT_FLAG_SOF|MCU_EVENT_FLAG_WAKEUP)
            ){
        usb_local[port].special_event_handler = action->handler;
        return 0;
    }


    if( action->channel & 0x80 ){
        if( (action->handler.callback == 0) && (action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE) ){
            usb_local[port].write_pending &= ~(1<<log_ep);
            mcu_execute_event_handler(&(usb_local[port].write[log_ep]), MCU_EVENT_FLAG_CANCELED, 0);
        }
    } else {
        if( (action->handler.callback == 0) && (action->o_events & MCU_EVENT_FLAG_DATA_READY) ){
            usb_local[port].read_ready |= (1<<log_ep);
            mcu_execute_event_handler(&(usb_local[port].read[log_ep]), MCU_EVENT_FLAG_CANCELED, 0);
        }
    }


    if ( (log_ep < DEV_USB_LOGICAL_ENDPOINT_COUNT)  ){
        if( action->o_events & MCU_EVENT_FLAG_DATA_READY ){
            //cortexm_enable_interrupts(NULL);
            if( cortexm_validate_callback(action->handler.callback) < 0 ){
                return -1;
            }

            usb_local[port].read[log_ep].callback = action->handler.callback;
            usb_local[port].read[log_ep].context = action->handler.context;
            ret = 0;
        }

        if( action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE ){
            if( cortexm_validate_callback(action->handler.callback) < 0 ){
                return -1;
            }

            usb_local[port].write[log_ep].callback = action->handler.callback;
            usb_local[port].write[log_ep].context = action->handler.context;
            ret = 0;
        }
    }

    if( ret < 0 ){
        ret = SYSFS_SET_RETURN(EINVAL);
    }
    return ret;
}

int mcu_usb_read(const devfs_handle_t * handle, devfs_async_t * rop){
    u32 port = handle->port;
    int ret;
    int loc = rop->loc;

    if ( loc > (DEV_USB_LOGICAL_ENDPOINT_COUNT-1) ){
        return SYSFS_SET_RETURN(EINVAL);
    }

    if( usb_local[port].read[loc].callback ){
        return SYSFS_SET_RETURN(EBUSY);
    }

    //Synchronous read (only if data is ready) otherwise 0 is returned
    if ( usb_local[port].read_ready & (1<<loc) ){
        ret = mcu_usb_root_read_endpoint(handle, loc, rop->buf);
        if( ret == 0 ){
            return SYSFS_SET_RETURN(EAGAIN);
        }
    } else {
        rop->nbyte = 0;
        if ( !(rop->flags & O_NONBLOCK) ){
            //If this is a blocking call, set the callback and context
            if( cortexm_validate_callback(rop->handler.callback) < 0 ){
                return -1;
            }

            usb_local[port].read[loc].callback = rop->handler.callback;
            usb_local[port].read[loc].context = rop->handler.context;
            ret = 0;
        } else {
            return SYSFS_SET_RETURN(EAGAIN);
        }
    }

    return ret;
}

int mcu_usb_write(const devfs_handle_t * handle, devfs_async_t * wop){
    //Asynchronous write
    u32 port = handle->port;
    int ep;
    int loc = wop->loc;


    ep = (loc & 0x7F);

    if ( ep > (DEV_USB_LOGICAL_ENDPOINT_COUNT-1) ){
        return SYSFS_SET_RETURN(EINVAL);
    }

    if ( usb_local[port].write[ep].callback ){
        return SYSFS_SET_RETURN(EBUSY);
    }

    usb_local[port].write_pending |= (1<<ep);

    if( cortexm_validate_callback(wop->handler.callback) < 0 ){
        return -1;
    }

    usb_local[port].write[ep].callback = wop->handler.callback;
    usb_local[port].write[ep].context = wop->handler.context;

    wop->nbyte = mcu_usb_root_write_endpoint(handle, loc, wop->buf, wop->nbyte);

    if ( wop->nbyte < 0 ){
        usb_disable_endpoint(handle, loc );
        usb_reset_endpoint(handle, loc );
        usb_enable_endpoint(handle, loc );
        usb_local[port].write_pending &= ~(1<<ep);
        return -2;
    }

    return 0;
}


void usb_reset(const devfs_handle_t * handle){

}

void usb_wakeup(int port){

}

void usb_set_address(const devfs_handle_t * handle, u32 addr){
    HAL_PCD_SetAddress(&usb_local[handle->port].hal_handle, addr);
}

void usb_configure(const devfs_handle_t * handle, u32 cfg){
    usb_local[handle->port].connected = 1;
}

void usb_configure_endpoint(const devfs_handle_t * handle, u32 endpoint_num, u32 max_packet_size, u8 type){
    u32 port = handle->port;
    const stm32_config_t * stm32_config = mcu_board_config.arch_config;
    HAL_PCD_EP_Open(&usb_local[handle->port].hal_handle, endpoint_num, max_packet_size, type & EP_TYPE_MSK);

    if( (endpoint_num & 0x80) == 0 ){
        void * dest_buffer;

        usb_local[port].rx_buffer_offset[endpoint_num] = usb_local[port].rx_buffer_used;
        usb_local[port].rx_buffer_used += (max_packet_size*2);
        if( usb_local[port].rx_buffer_used > stm32_config->usb_rx_buffer_size ){
            //this is a fatal error
            mcu_board_execute_event_handler(MCU_BOARD_CONFIG_EVENT_ROOT_FATAL, "usbbuf");
        }

        dest_buffer = stm32_config->usb_rx_buffer +
                usb_local[port].rx_buffer_offset[endpoint_num] +
                max_packet_size;

        HAL_PCD_EP_Receive(&usb_local[port].hal_handle, endpoint_num, dest_buffer, max_packet_size);

    }
}

void usb_enable_endpoint(const devfs_handle_t * handle, u32 endpoint_num){
}

void usb_disable_endpoint(const devfs_handle_t * handle, u32 endpoint_num){
    HAL_PCD_EP_Close(&usb_local[handle->port].hal_handle, endpoint_num);
}

void usb_reset_endpoint(const devfs_handle_t * handle, u32 endpoint_num){
}

void usb_stall_endpoint(const devfs_handle_t * handle, u32 endpoint_num){
    HAL_PCD_EP_SetStall(&usb_local[handle->port].hal_handle, endpoint_num);
}

void usb_unstall_endpoint(const devfs_handle_t * handle, u32 endpoint_num){
    HAL_PCD_EP_ClrStall(&usb_local[handle->port].hal_handle, endpoint_num);
}

int mcu_usb_isconnected(const devfs_handle_t * handle, void * ctl){
    return usb_local[handle->port].connected;
}

void usb_clr_ep_buf(const devfs_handle_t * handle, u32 endpoint_num){

}

int mcu_usb_root_read_endpoint(const devfs_handle_t * handle, u32 endpoint_num, void * dest){
    u32 port = handle->port;
    const stm32_config_t * stm32_config = mcu_board_config.arch_config;
    void * src_buffer;
    u8 epnum;
    epnum = endpoint_num & 0x7f;

    if( usb_local[port].read_ready & (1<<epnum) ){
        usb_local[port].read_ready &= ~(1<<epnum);
        src_buffer = stm32_config->usb_rx_buffer + usb_local[port].rx_buffer_offset[epnum];
        //data is copied from fifo to buffer during the interrupt
        memcpy(dest, src_buffer, usb_local[port].rx_count[epnum]);
        return usb_local[port].rx_count[epnum];
    }

    return -1;
}



int mcu_usb_root_write_endpoint(const devfs_handle_t * handle, u32 endpoint_num, const void * src, u32 size){
    int ret;
    ret = HAL_PCD_EP_Transmit(&usb_local[handle->port].hal_handle, endpoint_num, (void*)src, size);
    if( ret == HAL_OK ){
        return size;
    }
    return -1;
}


void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd){
    usb_local_t * usb = (usb_local_t *)hpcd;
    //a setup packet has been received
    const stm32_config_t * stm32_config = mcu_board_config.arch_config;
    usb_event_t event;
    event.epnum = 0;
    void * dest_buffer;

    //Setup data is in hpcd->Setup buffer at this point

    //copy setup data to ep0 data buffer
    usb->read_ready |= (1<<0);
    dest_buffer = stm32_config->usb_rx_buffer + usb->rx_buffer_offset[0];
    usb->rx_count[0] = sizeof(usbd_setup_packet_t);
    memcpy(dest_buffer, hpcd->Setup, usb->rx_count[0]);

    mcu_execute_event_handler(usb->read + 0, MCU_EVENT_FLAG_SETUP, &event);

    //prepare EP zero for receiving out data
    HAL_PCD_EP_Receive(hpcd, 0, stm32_config->usb_rx_buffer, hpcd->OUT_ep[0].maxpacket);
}

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum){
    usb_local_t * usb = (usb_local_t *)hpcd;
    //data has already been received and is stored in buffer specified by HAL_PCD_EP_Receive
    const stm32_config_t * stm32_config = mcu_board_config.arch_config;
    usb_event_t event;
    event.epnum = epnum;
    u16 count;
    void * src_buffer;
    void * dest_buffer;

    //set read ready flag
    usb->read_ready |= (1<<epnum);
    count = HAL_PCD_EP_GetRxCount(&usb->hal_handle, epnum);

    dest_buffer = stm32_config->usb_rx_buffer + usb->rx_buffer_offset[epnum];
    src_buffer = dest_buffer + hpcd->OUT_ep[epnum].maxpacket;

    memcpy(dest_buffer, src_buffer, count);
    usb->rx_count[epnum] = count;

    //mcu_debug_root_printf("Data out %d 0x%lX %d\n", epnum, (u32)usb->read[epnum].callback, count);
    mcu_execute_event_handler(usb->read + epnum, MCU_EVENT_FLAG_DATA_READY, &event);

    //prepare to receive the next packet in the local buffer
    HAL_PCD_EP_Receive(hpcd, epnum, src_buffer, hpcd->OUT_ep[epnum].maxpacket);

}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum){
    usb_local_t * usb = (usb_local_t *)hpcd;
    u8 logical_ep = epnum & 0x7f;
    usb_event_t event;
    event.epnum = epnum;

    //mcu_debug_root_printf("Data in %d 0x%lX\n", logical_ep, (u32)usb->read[logical_ep].callback);

    mcu_execute_event_handler(usb->write + logical_ep, MCU_EVENT_FLAG_WRITE_COMPLETE, &event);

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
    usb_local_t * usb = (usb_local_t *)hpcd;
    u32 mps = mcu_board_config.usb_max_packet_zero;


    usb->rx_buffer_used = mps;
    for(i=0; i < DEV_USB_LOGICAL_ENDPOINT_COUNT; i++){
        usb->rx_buffer_offset[i] = 0;
    }

    HAL_PCD_EP_Open(hpcd, 0x00, mps, EP_TYPE_CTRL);
    HAL_PCD_EP_Open(hpcd, 0x80, mps, EP_TYPE_CTRL);
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
    usb_local_t * usb = (usb_local_t *)hpcd;
    usb->connected = 1;
    //execute special event handler
}

void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd){
    usb_local_t * usb = (usb_local_t *)hpcd;
    usb->connected = 0;
}

void mcu_core_otg_fs_isr(){
    HAL_PCD_IRQHandler(&usb_local[0].hal_handle);
}

#if MCU_USB_PORTS > 0
void mcu_core_otg_hs_isr(){
    HAL_PCD_IRQHandler(&usb_local[1].hal_handle);
}
#endif

#endif








