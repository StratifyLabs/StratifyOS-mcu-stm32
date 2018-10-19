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

#if MCU_QSPI_PORTS > 0


typedef struct {
	QSPI_HandleTypeDef hal_handle;
	devfs_transfer_handler_t transfer_handler;
	u8 ref_count;
} qspi_local_t;

static qspi_local_t qspi_local[MCU_QSPI_PORTS] MCU_SYS_MEM;
QUADSPI_TypeDef * const qspi_regs_table[MCU_QSPI_PORTS] = MCU_QSPI_REGS;
u8 const qspi_irqs[MCU_QSPI_PORTS] = MCU_QSPI_IRQS;


DEVFS_MCU_DRIVER_IOCTL_FUNCTION_MIN(qspi, QSPI_VERSION, QSPI_IOC_IDENT_CHAR)

int mcu_qspi_open(const devfs_handle_t * handle){
	int port = handle->port;
	if ( qspi_local[port].ref_count == 0 ){

		qspi_local[port].hal_handle.Instance = qspi_regs_table[port];

		switch(port){
			case 0:
				__HAL_RCC_QSPI_CLK_ENABLE();
				break;
		}
		//reset HAL UART
		cortexm_enable_irq(qspi_irqs[port]);

	}
	qspi_local[port].ref_count++;

	return 0;
}

int mcu_qspi_close(const devfs_handle_t * handle){
	int port = handle->port;
	if ( qspi_local[port].ref_count > 0 ){
		if ( qspi_local[port].ref_count == 1 ){
			cortexm_disable_irq(qspi_irqs[port]);
			switch(port){
				case 0:
					__HAL_RCC_QSPI_CLK_DISABLE();
					break;
			}
			qspi_local[port].hal_handle.Instance = 0;
		}
		qspi_local[port].ref_count--;
	}
	return 0;
}


int mcu_qspi_getinfo(const devfs_handle_t * handle, void * ctl){
	qspi_info_t * info = ctl;
	info->o_flags = 0;

	return 0;
}

int mcu_qspi_setattr(const devfs_handle_t * handle, void * ctl){
	u32 o_flags;
	int port = handle->port;
	const qspi_attr_t * attr;

	qspi_local_t * qspi = qspi_local + port;
	attr = mcu_select_attr(handle, ctl);
	if( attr == 0 ){
		return SYSFS_SET_RETURN(EINVAL);
	}

	o_flags = attr->o_flags;

	if( o_flags & QSPI_FLAG_SET_MASTER ){

		qspi->hal_handle.Init.ClockPrescaler = 0; //need to calculate
		qspi->hal_handle.Init.FifoThreshold = 0; //not sure how this will be used
		qspi->hal_handle.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
		qspi->hal_handle.Init.FlashSize = 0;
		qspi->hal_handle.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_8_CYCLE;
		qspi->hal_handle.Init.ClockMode = QSPI_CLOCK_MODE_0;
		//Clock mode QSPI_CLOCK_MODE_3 is double data rate
		qspi->hal_handle.Init.FlashID = QSPI_FLASH_ID_1;
		qspi->hal_handle.Init.DualFlash = QSPI_DUALFLASH_DISABLE;


		//pin assignments
		if( mcu_set_pin_assignment(
				 &(attr->pin_assignment),
				 MCU_CONFIG_PIN_ASSIGNMENT(uart_config_t, handle),
				 MCU_PIN_ASSIGNMENT_COUNT(uart_pin_assignment_t),
				 CORE_PERIPH_QSPI, port, 0, 0, 0) < 0 ){
			return SYSFS_SET_RETURN(EINVAL);
		}

		if( HAL_QSPI_Init(&qspi->hal_handle) != HAL_OK ){
			return SYSFS_SET_RETURN(EIO);
		}
	}

	return 0;
}


int mcu_qspi_setaction(const devfs_handle_t * handle, void * ctl){
	int port = handle->port;
	mcu_action_t * action = ctl;
	if( action->handler.callback != 0 ){
		return SYSFS_SET_RETURN(ENOTSUP);
	}
	cortexm_set_irq_priority(qspi_irqs[port], action->prio, action->o_events);
	return 0;
}

int mcu_qspi_read(const devfs_handle_t * handle, devfs_async_t * async){
	int port = handle->port;
	qspi_local_t * qspi = qspi_local + port;
	QSPI_CommandTypeDef command;

	//can't read and write at the same time
	if( qspi->transfer_handler.write != 0 ){
		return SYSFS_SET_RETURN(EBUSY);
	}

	DEVFS_DRIVER_IS_BUSY(qspi->transfer_handler.read, async);

	if( cortexm_validate_callback(async->handler.callback) < 0 ){
		qspi->transfer_handler.read = 0;
		return SYSFS_SET_RETURN(EPERM);
	}

	//start by sending a command
	command.Address = async->loc;
	//fill out the rest of command

	if( HAL_QSPI_Command_IT(&qspi->hal_handle, &command) == HAL_OK ){
		return 0;
	}


	qspi->transfer_handler.read = 0;
	return SYSFS_SET_RETURN(EIO);
}

int mcu_qspi_write(const devfs_handle_t * handle, devfs_async_t * async){
	int port = handle->port;
	qspi_local_t * qspi = qspi_local + port;
	QSPI_CommandTypeDef command;

	//can't read and write at the same time
	if( qspi->transfer_handler.read != 0 ){
		return SYSFS_SET_RETURN(EBUSY);
	}

	DEVFS_DRIVER_IS_BUSY(qspi->transfer_handler.write, async);

	if( cortexm_validate_callback(async->handler.callback) < 0 ){
		qspi->transfer_handler.write = 0;
		return SYSFS_SET_RETURN(EPERM);
	}

	//start by sending a command
	command.Address = async->loc;
	//fill out the rest of command

	if( HAL_QSPI_Command_IT(&qspi->hal_handle, &command) == HAL_OK ){
		return 0;
	}

	return SYSFS_SET_RETURN(EIO);
}

void HAL_QSPI_ErrorCallback(QSPI_HandleTypeDef *hqspi){

}

void HAL_QSPI_AbortCpltCallback(QSPI_HandleTypeDef *hqspi){

}

void HAL_QSPI_FifoThresholdCallback(QSPI_HandleTypeDef *hqspi){

}

void HAL_QSPI_CmdCpltCallback(QSPI_HandleTypeDef *hqspi){
	int ret;
	qspi_local_t * qspi =  (qspi_local_t *)hqspi;
	if( qspi->transfer_handler.read ){
		//command is the start of a read operation -- complete the read
		ret = HAL_QSPI_Receive_IT(hqspi, qspi->transfer_handler.read->buf);
	} else if( qspi->transfer_handler.write ){
		ret = HAL_QSPI_Transmit_IT(hqspi, qspi->transfer_handler.write->buf);
	}

	if( ret != HAL_OK ){
		//there was an error -- execute the callback
	}

}

void HAL_QSPI_RxCpltCallback(QSPI_HandleTypeDef *hqspi){
	qspi_local_t * qspi =  (qspi_local_t *)hqspi;
	devfs_execute_read_handler(&qspi->transfer_handler, 0, hqspi->RxXferCount, MCU_EVENT_FLAG_DATA_READY);

}

void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *hqspi){
	qspi_local_t * qspi =  (qspi_local_t *)hqspi;
	devfs_execute_write_handler(&qspi->transfer_handler, 0, hqspi->TxXferCount, MCU_EVENT_FLAG_WRITE_COMPLETE);
}

void HAL_QSPI_RxHalfCpltCallback(QSPI_HandleTypeDef *hqspi){
	//this is for DMA only
}

void HAL_QSPI_TxHalfCpltCallback(QSPI_HandleTypeDef *hqspi){
	//this is for DMA only
}

void mcu_core_quadspi_isr(){
	HAL_QSPI_IRQHandler(&qspi_local[0].hal_handle);
}

#endif
