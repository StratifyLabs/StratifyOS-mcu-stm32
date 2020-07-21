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

#define ETH_DMA_TRANSMIT_TIMEOUT               ( 20U )


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
#if MCU_ETH_API == 1
	ETH_TxPacketConfig tx_packet_config;
#endif
	u8 ref_count;
} eth_local_t;

static eth_local_t m_eth_local[MCU_ETH_PORTS] MCU_SYS_MEM;
ETH_TypeDef * const eth_regs_table[MCU_ETH_PORTS] = MCU_ETH_REGS;
u8 const eth_irqs[MCU_ETH_PORTS] = MCU_ETH_IRQS;

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(eth, ETH_VERSION, ETH_IOC_IDENT_CHAR, I_MCU_TOTAL + I_ETH_TOTAL, mcu_eth_setregister, mcu_eth_getregister)

int mcu_eth_open(const devfs_handle_t * handle){
	int port = handle->port;
	if ( m_eth_local[port].ref_count == 0 ){

		m_eth_local[port].hal_handle.Instance = eth_regs_table[port];

		switch(port){
			case 0:
#if MCU_ETH_API == 1
				__HAL_RCC_ETH1MAC_CLK_ENABLE();
				__HAL_RCC_ETH1TX_CLK_ENABLE();
				__HAL_RCC_ETH1RX_CLK_ENABLE();
#else
				__HAL_RCC_ETH_CLK_ENABLE();
#endif
				break;
		}
		cortexm_enable_irq(eth_irqs[port]);
	}
	m_eth_local[port].ref_count++;

	return 0;
}

int mcu_eth_close(const devfs_handle_t * handle){
	int port = handle->port;
	if ( m_eth_local[port].ref_count > 0 ){
		if ( m_eth_local[port].ref_count == 1 ){
			cortexm_disable_irq(eth_irqs[port]);
			switch(port){
				case 0:
#if MCU_ETH_API == 1
				__HAL_RCC_ETH1MAC_CLK_DISABLE();
				__HAL_RCC_ETH1TX_CLK_DISABLE();
				__HAL_RCC_ETH1RX_CLK_DISABLE();
#else
				__HAL_RCC_ETH_CLK_DISABLE();
#endif
					break;
			}
			m_eth_local[port].hal_handle.Instance = 0;
		}
		m_eth_local[port].ref_count--;
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

	DEVFS_DRIVER_DECLARE_LOCAL(eth, MCU_ETH_PORTS);
	int result MCU_UNUSED;
	const eth_attr_t * attr;
	attr = mcu_select_attr(handle, ctl);
	if( attr == 0 ){ return SYSFS_SET_RETURN(ENOSYS); }

	o_flags = attr->o_flags;

	if( o_flags & ETH_FLAG_SET_INTERFACE ){

		const stm32_eth_dma_config_t * config = handle->config;
		if( config == 0 ){ return SYSFS_SET_RETURN(ENOSYS); }

#if MCU_ETH_API == 1

		local->hal_handle.Init.MediaInterface = HAL_ETH_RMII_MODE;
		local->hal_handle.Init.MACAddr[0] =   0x00;
		local->hal_handle.Init.MACAddr[1] =   0x80;
		local->hal_handle.Init.MACAddr[2] =   0xE1;
		local->hal_handle.Init.MACAddr[3] =   0x00;
		local->hal_handle.Init.MACAddr[4] =   0x00;
		local->hal_handle.Init.MACAddr[5] =   0x00;
		local->hal_handle.Init.TxDesc = local->tx_dma_desc;
		local->hal_handle.Init.RxDesc = local->rx_dma_desc;
		local->hal_handle.Init.RxBuffLen = 1524;

		memset(&local->tx_packet_config, 0 , sizeof(ETH_TxPacketConfig));
		local->tx_packet_config.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
		local->tx_packet_config.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
		local->tx_packet_config.CRCPadCtrl = ETH_CRC_PAD_INSERT;

#else
		//ETH_AUTONEGOTIATION_ENABLE
		//ETH_AUTONEGOTIATION_DISABLE
		local->hal_handle.Init.AutoNegotiation = ETH_AUTONEGOTIATION_DISABLE;
		if( o_flags & ETH_FLAG_IS_AUTONEGOTIATION_ENABLED){
			local->hal_handle.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
		}

		//ETH_SPEED_10M
		//ETH_SPEED_100M
		local->hal_handle.Init.Speed = ETH_SPEED_10M;
		if( o_flags & (ETH_FLAG_IS_SPEED_100M | ETH_FLAG_IS_SPEED_1G) ){
			local->hal_handle.Init.Speed = ETH_SPEED_100M;
		}

		//ETH_MODE_FULLDUPLEX
		//ETH_MODE_HALFDUPLEX
		local->hal_handle.Init.DuplexMode = ETH_MODE_HALFDUPLEX;

		mcu_debug_log_info(MCU_DEBUG_DEVICE, "PHY address is %d", attr->phy_address);
		local->hal_handle.Init.PhyAddress = attr->phy_address;
		local->hal_handle.Init.MACAddr = (u8*)attr->mac_address;

		//ETH_RXPOLLING_MODE
		//ETH_RXINTERRUPT_MODE
		local->hal_handle.Init.RxMode = ETH_RXPOLLING_MODE;

		//ETH_CHECKSUM_BY_HARDWARE
		//ETH_CHECKSUM_BY_SOFTWARE
		local->hal_handle.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;

		//ETH_MEDIA_INTERFACE_MII
		//ETH_MEDIA_INTERFACE_RMII
		if( o_flags & ETH_FLAG_IS_RMII ){
			mcu_debug_log_info(MCU_DEBUG_DEVICE, "Use Ethernet RMII");
			local->hal_handle.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;
		} else if( o_flags & ETH_FLAG_IS_MII ){
			local->hal_handle.Init.MediaInterface = ETH_MEDIA_INTERFACE_MII;
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
		if( (result = HAL_ETH_Init(&local->hal_handle)) != HAL_OK ){
			mcu_debug_log_error(MCU_DEBUG_DEVICE, "HAL_ETH_Init() failed (%d)", result);
			return SYSFS_SET_RETURN(EIO);
		}

		HAL_ETH_DMATxDescListInit(&local->hal_handle, local->tx_dma_desc, config->tx_buffer, ETH_TXBUFNB);
		HAL_ETH_DMARxDescListInit(&local->hal_handle, local->rx_dma_desc, config->rx_buffer, ETH_RXBUFNB);
#endif
		mcu_debug_log_info(MCU_DEBUG_DEVICE, "Start Ethernet");
	}

	if( o_flags & ETH_FLAG_GET_STATE ){
		return HAL_ETH_GetState(&local->hal_handle);
	}

	if( o_flags & ETH_FLAG_START ){
		HAL_ETH_Start(&local->hal_handle);
	}

	if( o_flags & ETH_FLAG_STOP ){
		HAL_ETH_Stop(&local->hal_handle);

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

	result = HAL_ETH_ReadPHYRegister(
				&m_eth_local[handle->port].hal_handle,
		#if MCU_ETH_API == 1
			0,
		#endif
			channel->loc,
			&channel->value
			);
	if( result != HAL_OK ){
		return SYSFS_SET_RETURN(EIO);
	}

	return SYSFS_RETURN_SUCCESS;
}


int mcu_eth_setregister(const devfs_handle_t * handle, void * ctl){
	mcu_channel_t * channel = ctl;
	int result;

	result = HAL_ETH_WritePHYRegister(
				&m_eth_local[handle->port].hal_handle,
		#if MCU_ETH_API == 1
			0,
		#endif
			channel->loc,
			channel->value
			);
	if( result != HAL_OK ){
		return SYSFS_SET_RETURN(EIO);
	}

	return SYSFS_RETURN_SUCCESS;

}


int mcu_eth_read(const devfs_handle_t * handle, devfs_async_t * async){
	DEVFS_DRIVER_DECLARE_LOCAL(eth, MCU_ETH_PORTS);
	DEVFS_DRIVER_IS_BUSY(local->transfer_handler.read, async);

	//check to see if there is data ready to read in any of the buffers
#if MCU_ETH_API == 1

	if (HAL_ETH_IsRxDataAvailable(&local->hal_handle)){
		ETH_BufferTypeDef RxBuff;
		u32 framelength = 0;
		HAL_ETH_GetRxDataBuffer(&local->hal_handle, &RxBuff);
		HAL_ETH_GetRxDataLength(&local->hal_handle, &framelength);

		/* Build Rx descriptor to be ready for next data reception */
		HAL_ETH_BuildRxDescriptors(&local->hal_handle);

		mcu_core_invalidate_data_cache_block(
					RxBuff.buffer, framelength
					);

		//custom_pbuf  = (struct pbuf_custom*)LWIP_MEMPOOL_ALLOC(RX_POOL);
		//custom_pbuf->custom_free_function = pbuf_free_custom;
		//p = pbuf_alloced_custom(PBUF_RAW, framelength, PBUF_REF, custom_pbuf, RxBuff.buffer, ETH_RX_BUFFER_SIZE);

		//copy the data over to async->buf (if it fits)

		local->transfer_handler.read = 0;
		return async->nbyte;
	}

	local->transfer_handler.read = 0;
	return SYSFS_SET_RETURN(EAGAIN);

#else
	__IO ETH_DMADescTypeDef * dma_rx_descriptor;
	u8 * buffer;

	if( HAL_ETH_GetReceivedFrame(&local->hal_handle) != HAL_OK ){
		//failed to check for the frame
		local->transfer_handler.read = 0;
		return SYSFS_SET_RETURN(EAGAIN);

	} else {


		if( async->nbyte > local->hal_handle.RxFrameInfos.length ){
			//buffer has enough bytes to read everything
			async->nbyte = local->hal_handle.RxFrameInfos.length;
		} else {
			async->nbyte = async->nbyte - (async->nbyte % ETH_RX_BUF_SIZE); //make integer multiple of buffer size
			if( async->nbyte == 0 ){
				//target buffer is too small
				local->transfer_handler.read = 0;
				return SYSFS_SET_RETURN(EINVAL);
			}
		}

		//set up descriptor pointer and first buffer
		dma_rx_descriptor = local->hal_handle.RxFrameInfos.FSRxDesc;
		buffer = (u8*)local->hal_handle.RxFrameInfos.buffer;

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
			if( local->hal_handle.RxFrameInfos.SegCount ){
				local->hal_handle.RxFrameInfos.SegCount--; //decrement the segment counter
			}

			dma_rx_descriptor = (ETH_DMADescTypeDef *)(dma_rx_descriptor->Buffer2NextDescAddr);

			buffer = (u8*)dma_rx_descriptor->Buffer1Addr;

		} while( bytes_read < async->nbyte );

		/* When Rx Buffer unavailable flag is set: clear it and resume reception */
		if ((local->hal_handle.Instance->DMASR & ETH_DMASR_RBUS) != (uint32_t)RESET)
		{
			/* Clear RBUS ETHERNET DMA flag */
			local->hal_handle.Instance->DMASR = ETH_DMASR_RBUS;
			/* Resume DMA reception */
			local->hal_handle.Instance->DMARPDR = 0;
		}

		local->transfer_handler.read = 0;
		return async->nbyte;
	}
#endif
}

int mcu_eth_write(const devfs_handle_t * handle, devfs_async_t * async){
	DEVFS_DRIVER_DECLARE_LOCAL(eth, MCU_ETH_PORTS);

	DEVFS_DRIVER_IS_BUSY(local->transfer_handler.write, async);

#if MCU_ETH_API == 1

	local->tx_packet_config.Length = async->nbyte;
	local->tx_packet_config.TxBuffer = async->buf;

	HAL_ETH_Transmit(&local->hal_handle, &local->tx_packet_config, ETH_DMA_TRANSMIT_TIMEOUT);

	local->transfer_handler.write = NULL;
	return async->nbyte;

#else
	__IO ETH_DMADescTypeDef * DmaTxDesc = local->hal_handle.TxDesc;

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
			if( HAL_ETH_TransmitFrame(&local->hal_handle, async->nbyte) == HAL_OK ){
				local->transfer_handler.write = 0;
				return async->nbyte;
			}
		}
	}

	if ((local->hal_handle.Instance->DMASR & ETH_DMASR_TUS) != (uint32_t)RESET)
	{
		/* Clear TUS ETHERNET DMA flag */
		local->hal_handle.Instance->DMASR = ETH_DMASR_TUS;

		/* Resume DMA transmission*/
		local->hal_handle.Instance->DMATPDR = 0;
	}

#endif

	local->transfer_handler.write = 0;
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
	HAL_ETH_IRQHandler(&m_eth_local[0].hal_handle);
}

#endif
