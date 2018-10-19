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

#if STM32_ETH_DMA_MAX_PACKET_SIZE != ETH_MAX_PACKET_SIZE
#error("STM32_ETH_DMA_MAX_PACKET_SIZE is not equal to ETH_MAX_PACKET_SIZE")
#endif

#if (STM32_ETH_DMA_DESCRIPTOR_COUNT != ETH_RXBUFNB) || (STM32_ETH_DMA_DESCRIPTOR_COUNT != ETH_TXBUFNB)
#error("STM32_ETH_DMA_MAX_PACKET_SIZE is not equal to ETH_RXBUFNB or ETH_TXBUFNB")
#endif

typedef struct {
	ETH_HandleTypeDef hal_handle;
	devfs_transfer_handler_t transfer_handler;
	ETH_DMADescTypeDef tx_dma_desc[ETH_TXBUFNB];
	ETH_DMADescTypeDef rx_dma_desc[ETH_RXBUFNB];
	u8 ref_count;
} eth_local_t;

static eth_local_t eth_local[MCU_ETH_PORTS] MCU_SYS_MEM;
ETH_TypeDef * const eth_regs_table[MCU_ETH_PORTS] = MCU_ETH_REGS;
u8 const eth_irqs[MCU_ETH_PORTS] = MCU_ETH_IRQS;

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(eth, ETH_VERSION, ETH_IOC_IDENT_CHAR, I_MCU_TOTAL + I_ETH_TOTAL, mcu_eth_setregister, mcu_eth_getregister)

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
	if( attr == 0 ){ return SYSFS_SET_RETURN(ENOSYS); }

	o_flags = attr->o_flags;

	if( o_flags & ETH_FLAG_SET_INTERFACE ){

		const stm32_eth_dma_config_t * config = handle->config;
		if( config == 0 ){ return SYSFS_SET_RETURN(ENOSYS); }

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

		mcu_debug_log_info(MCU_DEBUG_DEVICE, "PHY address is %d", attr->phy_address);
		eth->hal_handle.Init.PhyAddress = attr->phy_address;
		eth->hal_handle.Init.MACAddr = (u8*)attr->mac_address;

		//ETH_RXPOLLING_MODE
		//ETH_RXINTERRUPT_MODE
		eth->hal_handle.Init.RxMode = ETH_RXPOLLING_MODE;

		//ETH_CHECKSUM_BY_HARDWARE
		//ETH_CHECKSUM_BY_SOFTWARE
		eth->hal_handle.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;

		//ETH_MEDIA_INTERFACE_MII
		//ETH_MEDIA_INTERFACE_RMII
		if( o_flags & ETH_FLAG_IS_RMII ){
			mcu_debug_log_info(MCU_DEBUG_DEVICE, "Use Ethernet RMII");
			eth->hal_handle.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;
		} else if( o_flags & ETH_FLAG_IS_MII ){
			eth->hal_handle.Init.MediaInterface = ETH_MEDIA_INTERFACE_MII;
		} else {
			return SYSFS_SET_RETURN(EINVAL);
		}

		mcu_debug_log_info(MCU_DEBUG_DEVICE, "Set eth pin assignments");

		//pin assignments
		if( mcu_set_pin_assignment(
				 &(attr->pin_assignment),
				 MCU_CONFIG_PIN_ASSIGNMENT(eth_config_t, handle),
				 MCU_PIN_ASSIGNMENT_COUNT(eth_pin_assignment_t),
				 CORE_PERIPH_ENET, port, 0, 0, 0) < 0 ){
			return SYSFS_SET_RETURN(EINVAL);
		}

		mcu_debug_log_info(MCU_DEBUG_DEVICE, "HAL_ETH_Init()");
		if( HAL_ETH_Init(&eth->hal_handle) != HAL_OK ){
			mcu_debug_log_error(MCU_DEBUG_DEVICE, "HAL_ETH_Init() failed");
			return SYSFS_SET_RETURN(EIO);
		}

		HAL_ETH_DMATxDescListInit(&eth->hal_handle, eth->tx_dma_desc, config->tx_buffer, ETH_TXBUFNB);
		HAL_ETH_DMARxDescListInit(&eth->hal_handle, eth->rx_dma_desc, config->rx_buffer, ETH_RXBUFNB);
		mcu_debug_log_info(MCU_DEBUG_DEVICE, "Start Ethernet");
		HAL_ETH_Start(&eth->hal_handle);
	}

	if( o_flags & ETH_FLAG_GET_STATE ){ return HAL_ETH_GetState(&eth->hal_handle); }
	if( o_flags & ETH_FLAG_STOP ){
		HAL_ETH_Stop(&eth->hal_handle);

		//if a read or write is active -- abort the read/write and execute the callback

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

int mcu_eth_getregister(const devfs_handle_t * handle, void * ctl){
	mcu_channel_t * channel = ctl;
	int result;

	result = HAL_ETH_ReadPHYRegister(&eth_local[handle->port].hal_handle, channel->loc, &channel->value);
	if( result != HAL_OK ){
		return SYSFS_SET_RETURN(EIO);
	}

	return SYSFS_RETURN_SUCCESS;
}


int mcu_eth_setregister(const devfs_handle_t * handle, void * ctl){
	mcu_channel_t * channel = ctl;
	int result;

	result = HAL_ETH_WritePHYRegister(&eth_local[handle->port].hal_handle, channel->loc, channel->value);
	if( result != HAL_OK ){
		return SYSFS_SET_RETURN(EIO);
	}

	return SYSFS_RETURN_SUCCESS;

}


int mcu_eth_read(const devfs_handle_t * handle, devfs_async_t * async){
	int port = handle->port;
	eth_local_t * eth = eth_local + port;
	__IO ETH_DMADescTypeDef * dma_rx_descriptor;
	u8 * buffer;

	DEVFS_DRIVER_IS_BUSY(eth->transfer_handler.read, async);

	//check to see if there is data ready to read in any of the buffers

	if( HAL_ETH_GetReceivedFrame(&eth->hal_handle) != HAL_OK ){
		//failed to check for the frame
		eth->transfer_handler.read = 0;
		return SYSFS_SET_RETURN(EAGAIN);

	} else {


		if( async->nbyte > eth->hal_handle.RxFrameInfos.length ){
			//buffer has enough bytes to read everything
			async->nbyte = eth->hal_handle.RxFrameInfos.length;
		} else {
			async->nbyte = async->nbyte - (async->nbyte % ETH_RX_BUF_SIZE); //make integer multiple of buffer size
			if( async->nbyte == 0 ){
				//target buffer is too small
				eth->transfer_handler.read = 0;
				return SYSFS_SET_RETURN(EINVAL);
			}
		}

		//set up descriptor pointer and first buffer
		dma_rx_descriptor = eth->hal_handle.RxFrameInfos.FSRxDesc;
		buffer = (u8*)eth->hal_handle.RxFrameInfos.buffer;

		int bytes_read = 0;
		int page_size;
		do {

			page_size = async->nbyte - bytes_read;
			if( page_size > ETH_RX_BUF_SIZE ){ page_size = ETH_RX_BUF_SIZE; }

			/* Copy data to Tx buffer*/
			memcpy(async->buf + bytes_read, buffer, page_size );
			bytes_read += page_size;

			/* Point to next descriptor */
			dma_rx_descriptor->Status |= ETH_DMARXDESC_OWN; //free the buffer
			if( eth->hal_handle.RxFrameInfos.SegCount ){
				eth->hal_handle.RxFrameInfos.SegCount--; //decrement the segment counter
			}

			dma_rx_descriptor = (ETH_DMADescTypeDef *)(dma_rx_descriptor->Buffer2NextDescAddr);

			buffer = (u8*)dma_rx_descriptor->Buffer1Addr;

		} while( bytes_read < async->nbyte );

		/* When Rx Buffer unavailable flag is set: clear it and resume reception */
		if ((eth->hal_handle.Instance->DMASR & ETH_DMASR_RBUS) != (uint32_t)RESET)
		{
			/* Clear RBUS ETHERNET DMA flag */
			eth->hal_handle.Instance->DMASR = ETH_DMASR_RBUS;
			/* Resume DMA reception */
			eth->hal_handle.Instance->DMARPDR = 0;
		}

		eth->transfer_handler.read = 0;
		return async->nbyte;
	}
}

int mcu_eth_write(const devfs_handle_t * handle, devfs_async_t * async){
	int port = handle->port;
	eth_local_t * eth = eth_local + port;

	DEVFS_DRIVER_IS_BUSY(eth->transfer_handler.write, async);

#if defined STM32H7


#else
	__IO ETH_DMADescTypeDef * DmaTxDesc = eth->hal_handle.TxDesc;

	/* Is this buffer available? If not, goto error */
	if((DmaTxDesc->Status & ETH_DMATXDESC_OWN) == (uint32_t)RESET){
		int bytes_written = 0;
		int page_size;
		u8 * buffer = (u8*)(DmaTxDesc->Buffer1Addr);

		/* Check if the length of data to copy is bigger than Tx buffer size*/
		do {

			page_size = async->nbyte - bytes_written;
			if( page_size > ETH_TX_BUF_SIZE ){ page_size = ETH_TX_BUF_SIZE; }

			/* Copy data to Tx buffer*/
			memcpy(buffer, async->buf + bytes_written, page_size );
			bytes_written += page_size;

			/* Point to next descriptor */
			if( page_size == ETH_TX_BUF_SIZE ){
				DmaTxDesc = (ETH_DMADescTypeDef *)(DmaTxDesc->Buffer2NextDescAddr);

				/* Check if the buffer is available */
				if((DmaTxDesc->Status & ETH_DMATXDESC_OWN) != (uint32_t)RESET){
					//error condition -- out of buffers
					break;
				}
				buffer = (u8*)(DmaTxDesc->Buffer1Addr);
			}

		} while( bytes_written < async->nbyte );

		if( bytes_written > 0 ){

			async->nbyte = bytes_written;
			if( HAL_ETH_TransmitFrame(&eth->hal_handle, async->nbyte) == HAL_OK ){
				eth->transfer_handler.write = 0;
				return async->nbyte;
			}
		}
	}

	if ((eth->hal_handle.Instance->DMASR & ETH_DMASR_TUS) != (uint32_t)RESET)
	{
		/* Clear TUS ETHERNET DMA flag */
		eth->hal_handle.Instance->DMASR = ETH_DMASR_TUS;

		/* Resume DMA transmission*/
		eth->hal_handle.Instance->DMATPDR = 0;
	}

#endif

	eth->transfer_handler.write = 0;
	return SYSFS_SET_RETURN(EIO);
}


void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth){
	eth_local_t * eth =  (eth_local_t *)heth;
	devfs_execute_read_handler(&eth->transfer_handler, 0, 0, MCU_EVENT_FLAG_DATA_READY);

}

void HAL_ETH_TxCpltCallback(ETH_HandleTypeDef *heth){
	eth_local_t * eth =  (eth_local_t *)heth;

	//have all bytes been sent?

	devfs_execute_write_handler(&eth->transfer_handler, 0, 0, MCU_EVENT_FLAG_WRITE_COMPLETE);
}


void mcu_core_eth_isr(){
	HAL_ETH_IRQHandler(&eth_local[0].hal_handle);
}

#endif
