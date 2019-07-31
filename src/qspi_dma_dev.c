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
#include "stm32_local.h"
#include "cortexm/cortexm.h"
#include "mcu/qspi.h"
#include "mcu/pio.h"
#include "mcu/debug.h"
#include "mcu/core.h"
#include <sched.h>

#if MCU_QSPI_PORTS > 0

#include "qspi_local.h"

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(qspi_dma, QSPI_VERSION, QSPI_IOC_IDENT_CHAR, I_QSPI_TOTAL + I_MCU_TOTAL, mcu_qspi_dma_execcommand)

int mcu_qspi_dma_open(const devfs_handle_t * handle){
	return qspi_local_open(handle);
}

int mcu_qspi_dma_close(const devfs_handle_t * handle){
	DEVFS_DRIVER_DECLARE_LOCAL(qspi, MCU_QSPI_PORTS);

	if( local->ref_count == 1 ){
		//disable the DMA
		const stm32_qspi_dma_config_t * config;

		if( local->transfer_handler.read || local->transfer_handler.write ){
			//HAL_QSPI_DMAStop(&local->hal_handle);
			devfs_execute_cancel_handler(&local->transfer_handler, 0, SYSFS_SET_RETURN(EIO), 0);
		}

		config = handle->config;
		if( config ){
			stm32_dma_clear_handle(config->dma_config.rx.dma_number, config->dma_config.rx.stream_number);
			stm32_dma_clear_handle(config->dma_config.tx.dma_number, config->dma_config.tx.stream_number);
		}
	}

	return qspi_local_close(handle);
}


int mcu_qspi_dma_getinfo(const devfs_handle_t * handle, void * ctl){
	qspi_info_t * info = ctl;
	info->o_flags = QSPI_FLAG_SET_MASTER;
	return 0;
}

int mcu_qspi_dma_setattr(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(qspi, MCU_QSPI_PORTS);
	const stm32_spi_dma_config_t * config;

	//BSP *MUST* provide DMA configuration information
	config = handle->config;
	if( config == 0 ){ return SYSFS_SET_RETURN(ENOSYS); }


	//setup the DMA for receiving

#if 1
	stm32_dma_channel_t * channel = stm32_dma_setattr(&config->dma_config.rx);
	if( channel == 0 ){ return SYSFS_SET_RETURN(EIO); }

#if MCU_QSPI_API == 1
	//__HAL_LINKDMA((&local->hal_handle), hmdma, channel->handle);
#else
	__HAL_LINKDMA((&local->hal_handle), hdma, channel->handle);
#endif
#else

	channel = stm32_dma_setattr(&config->dma_config.tx);
	if( channel == 0 ){ return SYSFS_SET_RETURN(EIO); }

	__HAL_LINKDMA((&local->hal_handle), hdma, channel->handle);
#endif

	return qspi_local_setattr(handle, ctl);
}

int mcu_qspi_dma_setaction(const devfs_handle_t * handle, void * ctl){
	return qspi_local_setaction(handle, ctl);
}

int mcu_qspi_dma_execcommand(const devfs_handle_t * handle, void * ctl){
	return qspi_local_execcommand(handle, ctl);
}

int mcu_qspi_dma_read(const devfs_handle_t * handle, devfs_async_t * async){
	DEVFS_DRIVER_DECLARE_LOCAL(qspi, MCU_QSPI_PORTS);

	//can't read and write at the same time
	if( local->transfer_handler.write != 0 ){
		return SYSFS_SET_RETURN(EBUSY);
	}
	//borrow async to qspi->transfer_handler.read
	DEVFS_DRIVER_IS_BUSY(local->transfer_handler.read, async);

	if (HAL_QSPI_Receive_DMA(&local->hal_handle, async->buf) != HAL_OK){
		local->transfer_handler.read = 0;
		return SYSFS_SET_RETURN(EIO);
	}
	return 0;
}

int mcu_qspi_dma_write(const devfs_handle_t * handle, devfs_async_t * async){
	DEVFS_DRIVER_DECLARE_LOCAL(qspi, MCU_QSPI_PORTS);

	//can't read and write at the same time
	if( local->transfer_handler.read != 0 ){
		return SYSFS_SET_RETURN(EBUSY);
	}
	//borrow async to qspi->transfer_handler.read
	DEVFS_DRIVER_IS_BUSY(local->transfer_handler.write, async);

#if defined STM32F7 || defined STM32H7
	//ensure data in cache is written to memory before writing
	mcu_core_clean_data_cache_block(async->buf, async->nbyte+31);
#endif

	if (HAL_QSPI_Transmit_DMA(&local->hal_handle, async->buf) != HAL_OK){
		local->transfer_handler.write = 0;
		return SYSFS_SET_RETURN(EIO);
	}

	return 0;
}

#endif
