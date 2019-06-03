/* Copyright 2011-2019 Tyler Gilbert;
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
#include "crypt_local.h"

#if MCU_CRYPT_PORTS > 0

DEVFS_MCU_DRIVER_IOCTL_FUNCTION_MIN(crypt, CRYPT_VERSION, CRYPT_IOC_CHAR)

int mcu_crypt_open(const devfs_handle_t * handle){
	return crypt_local_open(handle);
}

int mcu_crypt_close(const devfs_handle_t * handle){
	return crypt_local_close(handle);
}


int mcu_crypt_getinfo(const devfs_handle_t * handle, void * ctl){
	crypt_info_t * info = ctl;
	info->o_flags = CRYPT_FLAG_SET_CIPHER |
			CRYPT_FLAG_IS_AES_128 |
			CRYPT_FLAG_IS_AES_192 |
			CRYPT_FLAG_IS_AES_256;

	return SYSFS_RETURN_SUCCESS;

}

int mcu_crypt_setattr(const devfs_handle_t * handle, void * ctl){
	return crypt_local_setattr(handle, ctl);
}


int mcu_crypt_setaction(const devfs_handle_t * handle, void * ctl){
	return crypt_local_setaction(handle, ctl);
}


int mcu_crypt_read(const devfs_handle_t * handle, devfs_async_t * async){
	DEVFS_DRIVER_DECLARE_LOCAL(crypt, MCU_CRYPT_PORTS);
	DEVFS_DRIVER_IS_BUSY(local->transfer_handler.read, async);


	//read just sets up the buffer - the action starts when the device is written

	return SYSFS_RETURN_SUCCESS;
}


int mcu_crypt_write(const devfs_handle_t * handle, devfs_async_t * async){

	DEVFS_DRIVER_DECLARE_LOCAL(crypt, MCU_CRYPT_PORTS);
	DEVFS_DRIVER_IS_BUSY(local->transfer_handler.write, async);

	if( local->transfer_handler.read == 0 ){
		//read must be setup first -- that is where the destination data will go
		local->transfer_handler.write = 0;
		return SYSFS_SET_RETURN(EINVAL);
	}

	if( local->transfer_handler.read->nbyte < local->transfer_handler.write->nbyte ){
		//read and write need to have the same number of bytes
		local->transfer_handler.write = 0;
		return SYSFS_SET_RETURN(EINVAL);
	}

	int result;
	if( local->o_flags & CRYPT_FLAG_IS_ENCRYPT ){
		//nbyte is /4 based on DataWidthUnit
		result = HAL_CRYP_Encrypt_IT(&local->hal_handle,
											  async->buf, async->nbyte/4,
											  local->transfer_handler.read->buf);
	} else if( local->o_flags & CRYPT_FLAG_IS_DECRYPT ){
		//nbyte is /4 based on DataWidthUnit
		result = HAL_CRYP_Decrypt_IT(&local->hal_handle,
											  async->buf, async->nbyte/4,
											  local->transfer_handler.read->buf);
	} else {
		//must set either encrypt or decrypt
		local->transfer_handler.write = 0;
		return SYSFS_SET_RETURN(EINVAL);
	}

	if( result != HAL_OK ){
		local->transfer_handler.write = 0;
		return SYSFS_SET_RETURN(EIO);
	}


	//read just sets up the buffer - the action starts when the device is written

	return SYSFS_RETURN_SUCCESS;
}



#endif
