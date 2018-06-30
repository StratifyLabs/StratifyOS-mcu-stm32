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

#include <fcntl.h>
#include <mcu/eth.h>
#include "stm32_local.h"


#if MCU_ETH_PORTS > 0


typedef struct {
    ETH_HandleTypeDef hal_handle;
    devfs_transfer_handler_t transfer_handler;
    u8 ref_count;
} eth_local_t;

static eth_local_t eth_local[MCU_ETH_PORTS] MCU_SYS_MEM;
ETH_TypeDef * const eth_regs_table[MCU_ETH_PORTS] = MCU_ETH_REGS;
u8 const eth_irqs[MCU_ETH_PORTS] = MCU_ETH_IRQS;


DEVFS_MCU_DRIVER_IOCTL_FUNCTION_MIN(eth, ETH_VERSION, ETH_IOC_IDENT_CHAR)

int mcu_eth_open(const devfs_handle_t * handle){
    int port = handle->port;
    if ( eth_local[port].ref_count == 0 ){

        eth_local[port].hal_handle.Instance = eth_regs_table[port];

        switch(port){
        case 0:
            __HAL_RCC_ETH_CLK_ENABLE();
            break;
        }
        cortexm_enable_irq(eth_irqs[port]);
    }
    eth_local[port].ref_count++;

    return 0;
}

int mcu_eth_close(const devfs_handle_t * handle){
    int port = handle->port;
    if ( eth_local[port].ref_count > 0 ){
        if ( eth_local[port].ref_count == 1 ){
            cortexm_disable_irq(eth_irqs[port]);
            switch(port){
            case 0:
                __HAL_RCC_ETH_CLK_DISABLE();
                break;
            }
            eth_local[port].hal_handle.Instance = 0;
        }
        eth_local[port].ref_count--;
    }
    return 0;
}


int mcu_eth_getinfo(const devfs_handle_t * handle, void * ctl){
    eth_info_t * info = ctl;
    info->o_flags = ETH_FLAG_SET_INTERFACE |
            ETH_FLAG_IS_FULLDUPLEX |
            ETH_FLAG_IS_HALFDUPLEX |
            ETH_FLAG_IS_AUTONEGOTIATION_ENABLED |
            ETH_FLAG_IS_SPEED_100M |
            ETH_FLAG_IS_MII |
            ETH_FLAG_IS_RMII;
    info->o_events = MCU_EVENT_FLAG_DATA_READY | MCU_EVENT_FLAG_WRITE_COMPLETE;

    return 0;
}

int mcu_eth_setattr(const devfs_handle_t * handle, void * ctl){
    u32 o_flags;
    int port = handle->port;
    const eth_attr_t * attr;

    eth_local_t * eth = eth_local + port;
    attr = mcu_select_attr(handle, ctl);
    if( attr == 0 ){ return SYSFS_SET_RETURN(EINVAL); }

    o_flags = attr->o_flags;

    if( o_flags & ETH_FLAG_SET_INTERFACE ){

        //ETH_AUTONEGOTIATION_ENABLE
        //ETH_AUTONEGOTIATION_DISABLE
        eth->hal_handle.Init.AutoNegotiation = ETH_AUTONEGOTIATION_DISABLE;
        if( o_flags & ETH_FLAG_IS_AUTONEGOTIATION_ENABLED){
            eth->hal_handle.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
        }

        //ETH_SPEED_10M
        //ETH_SPEED_100M
        eth->hal_handle.Init.Speed = ETH_SPEED_10M;
        if( o_flags & (ETH_FLAG_IS_SPEED_100M | ETH_FLAG_IS_SPEED_1G) ){
            eth->hal_handle.Init.Speed = ETH_SPEED_100M;
        }

        //ETH_MODE_FULLDUPLEX
        //ETH_MODE_HALFDUPLEX
        eth->hal_handle.Init.DuplexMode = ETH_MODE_HALFDUPLEX;

        eth->hal_handle.Init.PhyAddress = attr->phy_address;
        eth->hal_handle.Init.MACAddr = (void*)attr->mac_addr;

        //ETH_RXPOLLING_MODE
        //ETH_RXINTERRUPT_MODE
        eth->hal_handle.Init.RxMode = ETH_RXINTERRUPT_MODE;

        //ETH_CHECKSUM_BY_HARDWARE
        //ETH_CHECKSUM_BY_SOFTWARE
        eth->hal_handle.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;

        //ETH_MEDIA_INTERFACE_MII
        //ETH_MEDIA_INTERFACE_RMII
        if( o_flags & ETH_FLAG_IS_RMII ){
            eth->hal_handle.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;
        } else if( o_flags & ETH_FLAG_IS_MII ){
            eth->hal_handle.Init.MediaInterface = ETH_MEDIA_INTERFACE_MII;
        } else {
            return SYSFS_SET_RETURN(EINVAL);
        }

        //pin assignments
        if( mcu_set_pin_assignment(
                    &(attr->pin_assignment),
                    MCU_CONFIG_PIN_ASSIGNMENT(eth_config_t, handle),
                    MCU_PIN_ASSIGNMENT_COUNT(eth_pin_assignment_t),
                    CORE_PERIPH_ENET, port, 0, 0, 0) < 0 ){
            return SYSFS_SET_RETURN(EINVAL);
        }

        if( HAL_ETH_Init(&eth->hal_handle) != HAL_OK ){
            return SYSFS_SET_RETURN(EIO);
        }
    }

    if( o_flags & ETH_FLAG_GET_STATE ){
        return HAL_ETH_GetState(&eth->hal_handle);
    }

    return 0;
}


int mcu_eth_setaction(const devfs_handle_t * handle, void * ctl){
    int port = handle->port;
    mcu_action_t * action = ctl;
    if( action->handler.callback != 0 ){
        return SYSFS_SET_RETURN(ENOTSUP);
    }
    cortexm_set_irq_priority(eth_irqs[port], action->prio, action->o_events);
    return 0;
}

int mcu_eth_read(const devfs_handle_t * handle, devfs_async_t * async){
    int port = handle->port;
    eth_local_t * eth = eth_local + port;

    DEVFS_DRIVER_IS_BUSY(eth->transfer_handler.read, async);

    if( HAL_ETH_GetReceivedFrame_IT(&eth->hal_handle) == HAL_OK ){
        return 0;
    }


    eth->transfer_handler.read = 0;
    return SYSFS_SET_RETURN(EIO);
}

int mcu_eth_write(const devfs_handle_t * handle, devfs_async_t * async){
    int port = handle->port;
    eth_local_t * eth = eth_local + port;

    DEVFS_DRIVER_IS_BUSY(eth->transfer_handler.write, async);

#if 0

    if( HAL_ETH_TransmitFrame(&eth->hal_handle, async->nbyte) == HAL_OK ){
        return 0;
    }
#endif

    eth->transfer_handler.write = 0;
    return SYSFS_SET_RETURN(EIO);
}


void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth){
    eth_local_t * eth =  (eth_local_t *)heth;
    mcu_execute_read_handler(&eth->transfer_handler, 0, 0);

}

void HAL_ETH_TxCpltCallback(ETH_HandleTypeDef *heth){
    eth_local_t * eth =  (eth_local_t *)heth;
    mcu_execute_write_handler(&eth->transfer_handler, 0, 0);
}


void mcu_core_eth_isr(){
    HAL_ETH_IRQHandler(&eth_local[0].hal_handle);
}

#endif
