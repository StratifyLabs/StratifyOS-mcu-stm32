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
typedef struct {
    QSPI_HandleTypeDef hal_handle;
    devfs_transfer_handler_t transfer_handler;
    u32 state;
    u32 read_instruction;   /*sets in QSPI_FLAG_SET_MASTER*/
    u32 mem_mapped_read_instruction;
    u32 write_instruction;
    u32 dummy_cycle;
    u32 ref_count;
} qspi_local_t;

/**
  * @brief  switch external flash to memmapped mode
  * @param  hqspi: QSPI handle
  * @retval None
  */
static int external_flash_switch_to_mem_maped(QSPI_HandleTypeDef *hqspi,qspi_local_t * qspi);
/**
  * @brief  return config necceserry state(qspi->state)
  * @param  u32 state(qspi state)
  * @param  time_out
  * @retval None
  */
QSPI_CommandTypeDef get_command_config(u32 state);
static u8 cmd_cplt_count, rx_cplt_count, tx_cplt_count;

static qspi_local_t qspi_local[MCU_QSPI_PORTS] MCU_SYS_MEM;
static QUADSPI_TypeDef * const qspi_regs_table[MCU_QSPI_PORTS] = MCU_QSPI_REGS;
static u8 const qspi_irqs[MCU_QSPI_PORTS] = MCU_QSPI_IRQS;

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(qspi, QSPI_VERSION, QSPI_IOC_IDENT_CHAR, I_MCU_TOTAL + I_QSPI_TOTAL, \
                                mcu_qspi_command)
//DEVFS_MCU_DRIVER_IOCTL_FUNCTION_MIN(qspi, QSPI_VERSION, QSPI_IOC_IDENT_CHAR)

int mcu_qspi_open(const devfs_handle_t * handle){
	int port = handle->port;

    if ( qspi_local[port].ref_count == 0 ){
        memset(&qspi_local[port].hal_handle,0,sizeof(QSPI_HandleTypeDef));
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
    info->o_flags = QSPI_FLAG_SET_MASTER | QSPI_FLAG_IS_READ_MEM_MAPPED_MODE;
	return 0;
}

int mcu_qspi_setattr(const devfs_handle_t * handle, void * ctl){
	u32 o_flags;
    const qspi_attr_t * attr;
    int port = (int)handle->port;
    qspi_local_t * qspi = qspi_local + handle->port * sizeof (qspi_local_t);
	attr = mcu_select_attr(handle, ctl);
	if( attr == 0 ){
		return SYSFS_SET_RETURN(EINVAL);
	}
	o_flags = attr->o_flags;
    qspi->state = o_flags;
	if( o_flags & QSPI_FLAG_SET_MASTER ){
        uint32_t flash_size;
        qspi->read_instruction = attr->read_instruction;
        qspi->write_instruction = attr->write_instruction;
        qspi->dummy_cycle = attr->dummy_cycle;
        qspi->mem_mapped_read_instruction = attr->mem_mapped_read_instruction;
        if (attr->width!=0 && attr->width < 32){
            flash_size = attr->width - 1;
        }else{
            flash_size = 0;
        }
        __HAL_RCC_QSPI_FORCE_RESET();
        __HAL_RCC_QSPI_RELEASE_RESET();
		if( mcu_set_pin_assignment(
				 &(attr->pin_assignment),
                 MCU_CONFIG_PIN_ASSIGNMENT(qspi_config_t, handle),
                 MCU_PIN_ASSIGNMENT_COUNT(qspi_pin_assignment_t),
                 CORE_PERIPH_QSPI, port, 0, 0, 0) < 0 ){
			return SYSFS_SET_RETURN(EINVAL);
		}
        qspi->hal_handle.Init.ClockPrescaler = attr->freq; //need to calculate
        qspi->hal_handle.Init.FifoThreshold = 16; //not sure how this will be used
        qspi->hal_handle.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
        qspi->hal_handle.Init.FlashSize = flash_size;/*attribute size 2^size-1*/
        qspi->hal_handle.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
        if(o_flags & QSPI_FLAG_IS_CLK_HIGH_WHILE_CS ){
            qspi->hal_handle.Init.ClockMode = QSPI_CLOCK_MODE_3;
        }else{
            /*default*/
            qspi->hal_handle.Init.ClockMode = QSPI_CLOCK_MODE_0;
        }
        //Clock mode QSPI_CLOCK_MODE_3 is double data rate
        if(o_flags & QSPI_FLAG_IS_FLASH_ID_2){
            qspi->hal_handle.Init.FlashID = QSPI_FLASH_ID_2;
            qspi->hal_handle.Init.DualFlash = QSPI_DUALFLASH_ENABLE;
        }else{
            /*by default*/
            qspi->hal_handle.Init.FlashID = QSPI_FLASH_ID_1;
            qspi->hal_handle.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
        }
        if( HAL_QSPI_Init(&qspi->hal_handle) != HAL_OK ){
			return SYSFS_SET_RETURN(EIO);
        }
	}
    if((o_flags & QSPI_FLAG_IS_READ_MEM_MAPPED_MODE) && !(qspi->hal_handle.State & HAL_QSPI_STATE_BUSY_MEM_MAPPED)){
        external_flash_switch_to_mem_maped(&qspi->hal_handle,qspi );
    }
    if(o_flags & QSPI_FLAG_READ_REGISTER){
        QSPI_CommandTypeDef command;
        command = get_command_config(o_flags);
        command.Instruction       = attr->command;
        command.AddressMode       = QSPI_ADDRESS_NONE;
        if(o_flags & QSPI_FLAG_IS_REGISTER_WIDTH_8){
            command.NbData        = 1;
        }else if(o_flags & QSPI_FLAG_IS_REGISTER_WIDTH_16){
            command.NbData        = 2;
        }else if(o_flags & QSPI_FLAG_IS_REGISTER_WIDTH_24){
            command.NbData        = 3;
        }else if(o_flags & QSPI_FLAG_IS_REGISTER_WIDTH_32){
            command.NbData        = 4;
        }else{
            command.NbData        = 1;
        }
        if( HAL_QSPI_Command(&qspi->hal_handle, &command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK ){
            qspi->transfer_handler.write = 0;
            return SYSFS_SET_RETURN(EIO);
        }
        if (HAL_QSPI_Receive(&qspi->hal_handle, attr->data,HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
            qspi->transfer_handler.read = 0;
            return SYSFS_SET_RETURN(EIO);
        }
    }else if(o_flags & QSPI_FLAG_WRITE_REGISTER){
        qspi_local_t * qspi = qspi_local + handle->port * sizeof(qspi_local_t);
        QSPI_CommandTypeDef command;
        command = get_command_config(o_flags);
        command.Instruction       = attr->command;
        command.AddressMode       = QSPI_ADDRESS_NONE;
        if(o_flags & QSPI_FLAG_IS_REGISTER_WIDTH_8){
            command.NbData        = 1;
        }else if(o_flags & QSPI_FLAG_IS_REGISTER_WIDTH_16){
            command.NbData        = 2;
        }else if(o_flags & QSPI_FLAG_IS_REGISTER_WIDTH_24){
            command.NbData        = 3;
        }else if(o_flags & QSPI_FLAG_IS_REGISTER_WIDTH_32){
            command.NbData        = 4;
        }else{
            command.DataMode =QSPI_DATA_NONE;
            command.NbData        = 0;
        }
        if( HAL_QSPI_Command(&qspi->hal_handle, &command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK ){
            return SYSFS_SET_RETURN(EIO);
        }
        if (command.NbData){
            if(HAL_QSPI_Transmit(&qspi->hal_handle, attr->data, HAL_QPSI_TIMEOUT_DEFAULT_VALUE)!=HAL_OK){
                return SYSFS_SET_RETURN(EIO);
            }
        }
        return 0;
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

int mcu_qspi_command(const devfs_handle_t * handle, void * ctl){
    qspi_local_t * qspi = qspi_local + handle->port * sizeof(qspi_local_t);
    u32 data_command;
    QSPI_CommandTypeDef command;
    data_command = (u32)ctl;
    command = get_command_config(qspi->state);
    command.Instruction       = data_command;
    command.AddressMode       = QSPI_ADDRESS_NONE;
    command.DataMode          = QSPI_DATA_NONE;
    command.NbData = 0;
    if( HAL_QSPI_Command(&qspi->hal_handle, &command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK ){
        return SYSFS_SET_RETURN(EIO);
    }
    return 0;
}

int mcu_qspi_read(const devfs_handle_t * handle, devfs_async_t * async){
    qspi_local_t * qspi = qspi_local + handle->port * sizeof(qspi_local_t);
    QSPI_CommandTypeDef command;
    //can't read and write at the same time
	if( qspi->transfer_handler.write != 0 ){
		return SYSFS_SET_RETURN(EBUSY);
	}
    //borrow async to qspi->transfer_handler.read
    DEVFS_DRIVER_IS_BUSY(qspi->transfer_handler.read, async);
	if( cortexm_validate_callback(async->handler.callback) < 0 ){
		qspi->transfer_handler.read = 0;
		return SYSFS_SET_RETURN(EPERM);
	}
    command = get_command_config(qspi->state);
    command.Instruction       = qspi->read_instruction;
    command.Address           = (u32)async->loc;
    command.DummyCycles       = qspi->dummy_cycle;
    command.NbData            = (u32)async->nbyte;
    if( HAL_QSPI_Command(&qspi->hal_handle, &command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK ){
        return SYSFS_SET_RETURN(EIO);
    }
    if (HAL_QSPI_Receive_IT(&qspi->hal_handle, qspi->transfer_handler.read->buf) != HAL_OK){
        return SYSFS_SET_RETURN(EIO);
    }
    return 0;
}

int mcu_qspi_write(const devfs_handle_t * handle, devfs_async_t * async){
    qspi_local_t * qspi = qspi_local + handle->port * sizeof(qspi_local_t);
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
    command = get_command_config(qspi->state);
    command.Instruction = qspi->write_instruction;
    command.Address     = (u32)async->loc;
    command.NbData      = (u32)async->nbyte;
    if( HAL_QSPI_Command(&qspi->hal_handle, &command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK ){
        qspi->transfer_handler.write = 0;
        return SYSFS_SET_RETURN(EIO);
    }
    if(HAL_QSPI_Transmit_IT(&qspi->hal_handle, qspi->transfer_handler.write->buf)!=HAL_OK){
        qspi->transfer_handler.write = 0;
        return SYSFS_SET_RETURN(EIO);
    }
    return 0;
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
    cmd_cplt_count++;
    if( qspi->transfer_handler.read ){
		//command is the start of a read operation -- complete the read
        //ret = HAL_QSPI_Receive_IT(hqspi, qspi->transfer_handler.read->buf);
	} else if( qspi->transfer_handler.write ){
        //ret = HAL_QSPI_Transmit_IT(hqspi, qspi->transfer_handler.write->buf);
    }

	if( ret != HAL_OK ){
		//there was an error -- execute the callback
	}

}

void HAL_QSPI_RxCpltCallback(QSPI_HandleTypeDef *hqspi){
	qspi_local_t * qspi =  (qspi_local_t *)hqspi;
    rx_cplt_count++;
	devfs_execute_read_handler(&qspi->transfer_handler, 0, hqspi->RxXferCount, MCU_EVENT_FLAG_DATA_READY);

}

void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *hqspi){
	qspi_local_t * qspi =  (qspi_local_t *)hqspi;
    tx_cplt_count++;
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

/*@brief switch to mem maped mode
 * return non zero value if error
 * */
int external_flash_switch_to_mem_maped(QSPI_HandleTypeDef *hqspi,qspi_local_t * qspi){
    int result = 0;
    QSPI_MemoryMappedTypeDef mem_map_config;
    QSPI_CommandTypeDef qspi_command;
    qspi_command = get_command_config(qspi->state);
    qspi_command.Instruction = qspi->mem_mapped_read_instruction;
    qspi_command.DummyCycles = qspi->dummy_cycle;
    mem_map_config.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;
    if (HAL_QSPI_MemoryMapped(hqspi, &qspi_command, &mem_map_config) != HAL_OK) {
        result = -1;
        mcu_debug_printf("Failed %s:%d\n",__FILE__,__LINE__);
    }else{
        result = 0;
    }
    return result;
}

/**
 * @brief
 * sets
 * command.InstructionMode   do sets
 * command.AddressMode       do sets
 * command.AddressSize       do sets
 * command.AlternateByteMode do sets
 * command.DataMode          do sets
 * command.DummyCycles       do sets
 * command.DdrMode           do sets
 * command.DdrHoldHalfCycle  do sets
 * command.SIOOMode          do sets
 *
 * dont sets
 * command.Instruction  dont sets
 * command.Address      dont sets
 * command.NbData       dont sets
 * */
QSPI_CommandTypeDef get_command_config(u32 state){
    QSPI_CommandTypeDef command;
    if(state & QSPI_FLAG_IS_INSTRUCTION_1_LINE){
        command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    }else{
        command.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
    }
    if (state & QSPI_FLAG_IS_ADDRESS_1_LINE){
        command.AddressMode       = QSPI_ADDRESS_1_LINE;
    }else{
        command.AddressMode       = QSPI_ADDRESS_4_LINES;
    }
    /*if use QSPI_ADDRESS_NONE AddressSize not use*/
    if(state & QSPI_FLAG_IS_ADDRESS_8_BITS){
        command.AddressSize  = QSPI_ADDRESS_8_BITS;
    }else if(state & QSPI_FLAG_IS_ADDRESS_16_BITS){
        command.AddressSize  = QSPI_ADDRESS_16_BITS;
    }else if(state & QSPI_FLAG_IS_ADDRESS_24_BITS){
        command.AddressSize  = QSPI_ADDRESS_24_BITS;
    }else{
        command.AddressSize  = QSPI_ADDRESS_32_BITS;
    }
    command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    if(state & QSPI_FLAG_IS_DATA_1_LINE){
        command.DataMode = QSPI_DATA_1_LINE;
    }else{
        command.DataMode = QSPI_DATA_4_LINES;
    }
    command.DummyCycles       = 0;
    command.DdrMode           = QSPI_DDR_MODE_DISABLE;
    command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
    return command;
}
#endif
