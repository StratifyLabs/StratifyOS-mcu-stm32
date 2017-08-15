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

#include <errno.h>
#include <fcntl.h>
#include "cortexm/cortexm.h"
#include "mcu/i2c.h"
#include "mcu/debug.h"
#include "mcu/core.h"
#include "mcu/pio.h"

#include "hal.h"


#if MCU_I2C_PORTS > 0

typedef struct MCU_PACK {
	I2C_HandleTypeDef hal_handle;
	mcu_event_handler_t master;
	mcu_event_handler_t slave;
	void * slave_memory;
	u32 slave_memory_offset;
	u32 slave_memory_size;
	u32 o_flags;
	u16 slave_addr[2];
	u16 err;
	u16 ref_count;
} i2c_local_t;

static i2c_local_t i2c_local[MCU_I2C_PORTS] MCU_SYS_MEM;
static I2C_TypeDef * const i2c_regs_table[MCU_I2C_PORTS] = MCU_I2C_REGS;
static u8 const i2c_irqs[MCU_I2C_PORTS] = MCU_I2C_IRQS;
static u8 const i2c_er_irqs[MCU_I2C_PORTS] = MCU_I2C_ER_IRQS;

static void enable_opendrain_pin(const mcu_pin_t * pin, void * internal_pullup) MCU_PRIV_CODE;
void enable_opendrain_pin(const mcu_pin_t * pin, void * internal_pullup){
	pio_attr_t pattr;
	devfs_handle_t pio_handle;
	pio_handle.port = pin->port;
	pio_handle.config = 0;
	pattr.o_pinmask = (1<<pin->pin);
	pattr.o_flags = PIO_FLAG_SET_OUTPUT | PIO_FLAG_IS_OPENDRAIN | (u32)internal_pullup;
	mcu_pio_setattr(&pio_handle, &pattr);
}

void mcu_i2c_dev_power_on(const devfs_handle_t * handle){
	int port = handle->port;
	i2c_local_t * i2c = i2c_local + port;
	if ( i2c->ref_count == 0 ){
		i2c->hal_handle.Instance = i2c_regs_table[port];
		switch(port){
		case 0:
			//mcu_lpc_core_enable_pwr(PCI2C0);
			break;
#if MCU_I2C_PORTS > 1
		case 1:
			//mcu_lpc_core_enable_pwr(PCI2C1);
			break;
#endif
#if MCU_I2C_PORTS > 2
		case 2:
			//mcu_lpc_core_enable_pwr(PCI2C2);
			break;
#endif
		}
		cortexm_enable_irq((void*)(u32)(i2c_irqs[port]));
	}
	i2c_local[port].ref_count++;
}

void mcu_i2c_dev_power_off(const devfs_handle_t * handle){
	int port = handle->port;
	I2C_TypeDef * i2c_regs;
	if ( i2c_local[port].ref_count > 0 ){
		if ( i2c_local[port].ref_count == 1 ){
			i2c_regs = i2c_regs_table[port];
			//i2c_regs->CONCLR = (AA);
			//i2c_regs->ADR0 = 0;
			//i2c_regs->ADR1 = 0;
			//i2c_regs->ADR2 = 0;
			//i2c_regs->ADR3 = 0;
			cortexm_disable_irq((void*)(u32)(i2c_irqs[port]));
			switch(port){
			case 0:
				//mcu_lpc_core_disable_pwr(PCI2C0);
				break;
#if MCU_I2C_PORTS > 1
			case 1:
				//mcu_lpc_core_disable_pwr(PCI2C1);
				break;
#endif
#if MCU_I2C_PORTS > 2
			case 2:
				//mcu_lpc_core_disable_pwr(PCI2C2);
				break;
#endif
			}
		}
		i2c_local[port].ref_count--;
	}
}

int mcu_i2c_dev_is_powered(const devfs_handle_t * handle){
	int port = handle->port;
	return ( i2c_local[port].ref_count != 0 );
}

int mcu_i2c_getinfo(const devfs_handle_t * handle, void * ctl){
	int port = handle->port;
	i2c_info_t * info = ctl;

	info->err = i2c_local[port].err;
	info->o_flags = I2C_FLAG_SET_MASTER |
			I2C_FLAG_SET_SLAVE |
			I2C_FLAG_PREPARE_PTR_DATA |
			I2C_FLAG_PREPARE_DATA |
			I2C_FLAG_IS_PULLUP |
			I2C_FLAG_IS_SLAVE_ADDR0 |
			I2C_FLAG_IS_SLAVE_ADDR1 |
			I2C_FLAG_IS_SLAVE_ADDR2 |
			I2C_FLAG_IS_SLAVE_ADDR3;

	info->freq = 400000;
	info->o_events = MCU_EVENT_FLAG_WRITE_COMPLETE | MCU_EVENT_FLAG_DATA_READY;
	return 0;
}

int mcu_i2c_setattr(const devfs_handle_t * handle, void * ctl){
	int port = handle->port;
	int count;
	int internal_pullup;
	i2c_local_t * i2c = i2c_local + handle->port;
	u32 freq;


	const i2c_attr_t * attr = mcu_select_attr(handle, ctl);
	if( attr == 0 ){
		return -1;
	}

	u32 o_flags = attr->o_flags;

	if( freq == 0 ){
		freq = 400000;
	}

	if( freq > 400000 ){
		freq = 400000;
	}

	if( o_flags & I2C_FLAG_SET_MASTER ){
		i2c->hal_handle.Init.AddressingMode = HAL_I2C_MODE_MASTER;
		i2c->hal_handle.Init.ClockSpeed = freq;
	} else if( o_flags & I2C_FLAG_SET_SLAVE ){

		i2c->hal_handle.Init.AddressingMode = HAL_I2C_MODE_SLAVE;
		i2c->hal_handle.Init.OwnAddress1 = (attr->slave_addr[0].addr8[0])<<1;
		i2c->hal_handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
		if( o_flags & I2C_FLAG_IS_SLAVE_ADDR1 ){
			i2c->hal_handle.Init.DualAddressMode = I2C_DUALADDRESS_ENABLE;
			i2c->hal_handle.Init.OwnAddress2 = (attr->slave_addr[1].addr8[0])<<1;
		}

		i2c->hal_handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
		if( o_flags & I2C_FLAG_IS_SLAVE_ACK_GENERAL_CALL ){
			i2c->hal_handle.Init.GeneralCallMode = I2C_GENERALCALL_ENABLE;
		}

		//set slave memory location
		i2c->slave_memory_offset = 0;
		i2c->slave_memory = attr->data;
		i2c->slave_memory_size = attr->size;

	}

	if( o_flags & (I2C_FLAG_SET_MASTER | I2C_FLAG_SET_SLAVE) ){
		i2c->hal_handle.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
		if( o_flags & I2C_FLAG_STRETCH_CLOCK ){
			i2c->hal_handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
		}

		i2c->hal_handle.Init.DutyCycle = I2C_DUTYCYCLE_2;
		if( freq < 100000 ){
			i2c->hal_handle.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
		}


		if( HAL_I2C_Init(&i2c->hal_handle) < 0 ){
			errno = EINVAL;
			return -1;
		}

		if( o_flags & I2C_FLAG_SET_SLAVE ){
			HAL_I2C_EnableListen_IT(&i2c->hal_handle);
		}

		i2c->o_flags = o_flags;
	}

	if( o_flags & (I2C_FLAG_PREPARE_PTR_DATA|I2C_FLAG_PREPARE_PTR|I2C_FLAG_PREPARE_DATA) ){
		i2c->o_flags = o_flags;
		i2c->slave_addr[0] = (attr->slave_addr[0].addr8[0]);
		i2c->slave_addr[1] = (attr->slave_addr[1].addr8[0]);
	}

	return 0;
}



int mcu_i2c_reset(const devfs_handle_t * handle, void * ctl){
	return 0;
}

int mcu_i2c_geterr(const devfs_handle_t * handle, void * ctl){
	int port = handle->port;
	return i2c_local[port].err;
}

int mcu_i2c_setaction(const devfs_handle_t * handle, void * ctl){
	mcu_action_t * action = (mcu_action_t*)ctl;
	int port = handle->port;

	cortexm_set_irq_prio(i2c_irqs[port], action->prio);

	if( action->handler.callback == 0 ){
		//i2c_local[port].slave.handler.callback = 0;
		//i2c_local[port].slave.handler.context = 0;
		return 0;
	}

	if( cortexm_validate_callback(action->handler.callback) < 0 ){
		return -1;
	}

	//i2c_local[port].slave.handler.callback = action->handler.callback;
	//i2c_local[port].slave.handler.context = action->handler.context;

	return 0;
}

int mcu_i2c_dev_write(const devfs_handle_t * handle, devfs_async_t * async){
	int ret;
	i2c_local_t * i2c = i2c_local + handle->port;

	if( i2c->master.callback ){
		errno = EBUSY;
		return -1;
	}


	if( i2c->o_flags & I2C_FLAG_PREPARE_PTR_DATA ){
		ret = HAL_I2C_Master_Sequential_Transmit_IT(&i2c->hal_handle, i2c->slave_addr[0]<<1, &async->loc, 1, 0);
	} else if( i2c->o_flags & I2C_FLAG_PREPARE_DATA ){
		ret = HAL_I2C_Master_Transmit_IT(&i2c->hal_handle, i2c->slave_addr[0]<<1, async->buf, async->nbyte);
	}

	if( ret == HAL_OK ){
		return 0;
	}

	return -1;
}

int mcu_i2c_dev_read(const devfs_handle_t * handle, devfs_async_t * async){
	int ret;
	i2c_local_t * i2c = i2c_local + handle->port;

	if( i2c->master.callback ){
		errno = EBUSY;
		return -1;
	}

	if( i2c->o_flags & I2C_FLAG_PREPARE_PTR_DATA ){
		ret = HAL_I2C_Master_Sequential_Transmit_IT(&i2c->hal_handle, i2c->slave_addr[0]<<1, &async->loc, 1, 0);
	} else if( i2c->o_flags & I2C_FLAG_PREPARE_DATA ){
		ret = HAL_I2C_Master_Receive_IT(&i2c->hal_handle, i2c->slave_addr[0]<<1, async->buf, async->nbyte);
	}

	if( ret == HAL_OK ){
		return 0;
	}

	return -1;
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c){
	i2c_local_t * i2c = (i2c_local_t*)hi2c;

}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c){
	i2c_local_t * i2c = (i2c_local_t*)hi2c;

}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c){
	i2c_local_t * i2c = (i2c_local_t*)hi2c;

}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c){
	i2c_local_t * i2c = (i2c_local_t*)hi2c;

}


static int is_no_stop(const i2c_local_t * local){
	return ((local->o_flags & I2C_FLAG_IS_NO_STOP) != 0);
}

static void mcu_i2c_isr(int port) {
	i2c_local_t * i2c = i2c_local + port;
	HAL_I2C_EV_IRQHandler(&i2c->hal_handle);
}

static void mcu_i2cer_isr(int port) {
	i2c_local_t * i2c = i2c_local + port;
	HAL_I2C_ER_IRQHandler(&i2c->hal_handle);
}

void mcu_core_i2c0_isr(){ mcu_i2c_isr(0); }
#if MCU_I2C_PORTS > 1
void mcu_core_i2c1_isr(){ mcu_i2c_isr(1); }
#endif
#if MCU_I2C_PORTS > 2
void mcu_core_i2c2_isr(){ mcu_i2c_isr(2); }
#endif

void mcu_core_i2c0er_isr(){ mcu_i2c_isr(0); }
#if MCU_I2C_PORTS > 1
void mcu_core_i2c1er_isr(){ mcu_i2c_isr(1); }
#endif
#if MCU_I2C_PORTS > 2
void mcu_core_i2c2er_isr(){ mcu_i2c_isr(2); }
#endif

#endif


