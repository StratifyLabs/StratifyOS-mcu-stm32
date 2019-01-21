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

#include "i2s_spi_local.h"

#include <mcu/i2s.h>
#include <mcu/spi.h>

#if MCU_I2S_SPI_PORTS > 0

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(i2s_spi, I2S_VERSION, I2S_IOC_IDENT_CHAR, I_MCU_TOTAL + I_I2S_TOTAL, mcu_i2s_spi_mute, mcu_i2s_spi_unmute)

int mcu_i2s_spi_open(const devfs_handle_t * handle){
	//make sure port supports I2S -- not all ports support I2S
	spi_local_t * local = spi_local + handle->port;
	local->o_flags = 0;
	return i2s_spi_local_open(handle);
}

int mcu_i2s_spi_close(const devfs_handle_t * handle){
	//same as SPI
	return i2s_spi_local_close(handle);
}

int mcu_i2s_spi_getinfo(const devfs_handle_t * handle, void * ctl){
	i2s_info_t * info = ctl;

	//set I2S Capability flags
	info->o_flags = 0;

	return 0;
}

int mcu_i2s_spi_mute(const devfs_handle_t * handle, void * ctl){
	return 0;
}

int mcu_i2s_spi_unmute(const devfs_handle_t * handle, void * ctl){
	return 0;
}

int mcu_i2s_spi_setattr(const devfs_handle_t * handle, void * ctl){
	return i2s_spi_local_setattr(handle, ctl);
}


int mcu_i2s_spi_setaction(const devfs_handle_t * handle, void * ctl){
	return mcu_spi_setaction(handle, ctl);
}

int mcu_i2s_spi_write(const devfs_handle_t * handle, devfs_async_t * async){
	int ret;
	spi_local_t * local = spi_local + handle->port;
	DEVFS_DRIVER_IS_BUSY(local->transfer_handler.write, async);


	if( local->o_flags & SPI_LOCAL_IS_FULL_DUPLEX ){

#if defined SPI_I2S_FULLDUPLEX_SUPPORT
		if( local->transfer_handler.read->nbyte < async->nbyte ){
			return SYSFS_SET_RETURN(EINVAL);
		}

		i2s_spi_local_wait_for_errata_level(local);
		ret = HAL_I2SEx_TransmitReceive_IT(
					&local->i2s_hal_handle,
					async->buf,
					local->transfer_handler.read->buf,
					async->nbyte/local->size_mult);
#else
		return SYSFS_SET_RETURN(ENOTSUP);
#endif

	} else {
		i2s_spi_local_wait_for_errata_level(local);
		ret = HAL_I2S_Transmit_IT(&local->i2s_hal_handle, async->buf, async->nbyte/local->size_mult);
	}


	if( ret != HAL_OK ){
		local->transfer_handler.write = 0;
		return SYSFS_SET_RETURN_WITH_VALUE(EIO, ret);
	}

	return 0;
}

int mcu_i2s_spi_read(const devfs_handle_t * handle, devfs_async_t * async){
	int ret;
	spi_local_t * local = spi_local + handle->port;

	DEVFS_DRIVER_IS_BUSY(local->transfer_handler.read, async);

#if defined SPI_I2S_FULLDUPLEX_SUPPORT
	if( local->o_flags & SPI_LOCAL_IS_FULL_DUPLEX ){
		return SYSFS_RETURN_SUCCESS;
	}
#endif
	//check for overrun
	i2s_spi_local_wait_for_errata_level(local);
	ret = HAL_I2S_Receive_IT(&local->i2s_hal_handle, async->buf, async->nbyte/local->size_mult);

	if( ret != HAL_OK ){
		local->transfer_handler.read = 0;
		mcu_debug_log_error(MCU_DEBUG_DEVICE, "I2S SPI Read failed");
		return SYSFS_SET_RETURN_WITH_VALUE(EIO, ret);
	}

	return 0;
}


#endif

