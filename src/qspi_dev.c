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

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(qspi, QSPI_VERSION, QSPI_IOC_IDENT_CHAR, I_QSPI_TOTAL + I_MCU_TOTAL, mcu_qspi_execcommand)

int mcu_qspi_open(const devfs_handle_t * handle){
	return qspi_local_open(handle);
}

int mcu_qspi_close(const devfs_handle_t * handle){
	return qspi_local_close(handle);
}

int mcu_qspi_getinfo(const devfs_handle_t * handle, void * ctl){
	qspi_info_t * info = ctl;
	info->o_flags = QSPI_FLAG_SET_MASTER | QSPI_FLAG_EXECUTE_COMMAND;
	return 0;
}

int mcu_qspi_setattr(const devfs_handle_t * handle, void * ctl){
	return qspi_local_setattr(handle, ctl);
}

int mcu_qspi_setaction(const devfs_handle_t * handle, void * ctl){
	return qspi_local_setaction(handle, ctl);
}

int mcu_qspi_execcommand(const devfs_handle_t * handle, void * ctl){
	return qspi_local_execcommand(handle, ctl);
}

int mcu_qspi_read(const devfs_handle_t * handle, devfs_async_t * async){
	DEVFS_DRIVER_DECLARE_LOCAL(qspi, MCU_QSPI_PORTS);

	//can't read and write at the same time
	if( local->transfer_handler.write != 0 ){
		return SYSFS_SET_RETURN(EBUSY);
	}
	//borrow async to qspi->transfer_handler.read
	DEVFS_DRIVER_IS_BUSY(local->transfer_handler.read, async);

	if (HAL_QSPI_Receive_IT(&local->hal_handle, async->buf) != HAL_OK){
		local->transfer_handler.read = 0;
		return SYSFS_SET_RETURN(EIO);
	}
	return 0;
}

int mcu_qspi_write(const devfs_handle_t * handle, devfs_async_t * async){
	DEVFS_DRIVER_DECLARE_LOCAL(qspi, MCU_QSPI_PORTS);
	//can't read and write at the same time
	if( local->transfer_handler.read != 0 ){
		MCU_DEBUG_LINE_TRACE();
		return SYSFS_SET_RETURN(EBUSY);
	}

	DEVFS_DRIVER_IS_BUSY(local->transfer_handler.write, async);

	int result;
	if( (result = HAL_QSPI_Transmit_IT(&local->hal_handle, async->buf)) != HAL_OK){
		local->transfer_handler.write = 0;
		return SYSFS_SET_RETURN(EIO);
	}

	return 0;
}

#endif
