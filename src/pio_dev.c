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
#include <mcu/pio.h>
#include <mcu/core.h>

#include <stm32_local.h>


#if MCU_PIO_PORTS > 0

typedef struct {
	mcu_event_handler_t handler;
	u8 ref_count;
} pio_local_t;

static pio_local_t m_mcu_pio_local[MCU_PIO_PORTS] MCU_SYS_MEM;
static GPIO_TypeDef * const m_pio_regs_table[MCU_PIO_PORTS] = MCU_PIO_REGS;
static u8 const m_pio_irqs[MCU_PIO_PORTS] = MCU_PIO_IRQS;

//this function is used by other modules to access pio regs
GPIO_TypeDef * const hal_get_pio_regs(u8 port){
	if( port < MCU_PIO_PORTS ){
		return m_pio_regs_table[port];
	}
	return 0;
}

void _mcu_pio_dev_power_on(const devfs_handle_t * handle){
	int port = handle->port;
	if ( m_mcu_pio_local[port].ref_count == 0 ){
		m_mcu_pio_local[port].handler.callback = NULL;
		switch(port){
		case 0: __HAL_RCC_GPIOA_CLK_ENABLE(); break;
		case 1: __HAL_RCC_GPIOB_CLK_ENABLE(); break;
		case 2: __HAL_RCC_GPIOC_CLK_ENABLE(); break;
		case 3: __HAL_RCC_GPIOD_CLK_ENABLE(); break;
		case 4: __HAL_RCC_GPIOE_CLK_ENABLE(); break;
		case 5: __HAL_RCC_GPIOF_CLK_ENABLE(); break;
		case 6: __HAL_RCC_GPIOG_CLK_ENABLE(); break;
		case 7: __HAL_RCC_GPIOH_CLK_ENABLE(); break;
		}
	}
	m_mcu_pio_local[port].ref_count++;
}

void _mcu_pio_dev_power_off(const devfs_handle_t * handle){
	int port = handle->port;
	if ( m_mcu_pio_local[port].ref_count > 0 ){
		if ( m_mcu_pio_local[port].ref_count == 1 ){


			m_mcu_pio_local[port].handler.callback = NULL;
		}
		m_mcu_pio_local[port].ref_count--;
	}
}

int _mcu_pio_dev_powered_on(const devfs_handle_t * handle){
	int port = handle->port;
	if ( m_mcu_pio_local[port].ref_count > 0 ){
		return 1;
	}
	return 0;
}




int _mcu_pio_dev_write(const devfs_handle_t * handle, devfs_async_t * wop){
	mcu_action_t * action;

	if( wop->nbyte != sizeof(mcu_action_t) ){
		errno = EINVAL;
		return -1;
	}

	action = (mcu_action_t*)wop->buf;
	action->handler = wop->handler;
	return mcu_pio_setaction(handle, action);
}

int mcu_pio_setaction(const devfs_handle_t * handle, void * ctl){

	return 0;
}


int mcu_pio_getattr(const devfs_handle_t * handle, void * ctl){
	//read the direction pin status
	errno = ENOTSUP;
	return -1;
}

int mcu_pio_setattr(const devfs_handle_t * handle, void * ctl){
	int port = handle->port;
	pio_attr_t * attr;
	GPIO_InitTypeDef gpio_init;
	GPIO_TypeDef * regs = m_pio_regs_table[port];
	attr = ctl;

	memset(&gpio_init, 0, sizeof(GPIO_InitTypeDef));

	//decode the direction
	gpio_init.Mode = GPIO_MODE_INPUT;
	if( attr->o_flags & (PIO_FLAG_SET_OUTPUT) ){
		if( attr->o_flags & (PIO_FLAG_IS_OPENDRAIN) ){
			gpio_init.Mode = GPIO_MODE_OUTPUT_OD;
		} else {
			//set output pins as output
			gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
		}
	}

	//code the speed
	gpio_init.Speed = GPIO_SPEED_FREQ_MEDIUM;
	if( attr->o_flags & (PIO_FLAG_IS_SPEED_LOW) ){
		gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
	} else if( attr->o_flags & (PIO_FLAG_IS_SPEED_HIGH) ){
		gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
	} else if( attr->o_flags & (PIO_FLAG_IS_SPEED_BLAZING) ){
		gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	}

	//decode the pull mechanism
	gpio_init.Pull = GPIO_NOPULL;
	if( attr->o_flags & (PIO_FLAG_IS_PULLUP) ){
		gpio_init.Pull = GPIO_PULLUP;
	} else if( attr->o_flags & (PIO_FLAG_IS_PULLDOWN) ){
		gpio_init.Pull = GPIO_PULLDOWN;
	}


	gpio_init.Pin = attr->o_pinmask;
	HAL_GPIO_Init(regs, &gpio_init);

	return 0;
}

int mcu_pio_setmask(const devfs_handle_t * handle, void * ctl){
	int port = handle->port;
	GPIO_TypeDef * regs = m_pio_regs_table[port];
	u32 o_pinmask = (u32)ctl;
	HAL_GPIO_WritePin(regs, o_pinmask, GPIO_PIN_SET);
	return 0;
}

int mcu_pio_clrmask(const devfs_handle_t * handle, void * ctl){
	int port = handle->port;
	GPIO_TypeDef * regs = m_pio_regs_table[port];
	u32 o_pinmask = (u32)ctl;
	HAL_GPIO_WritePin(regs, o_pinmask, GPIO_PIN_RESET);
	return 0;
}

int mcu_pio_get(const devfs_handle_t * handle, void * ctl){
	int port = handle->port;
	GPIO_TypeDef * regs = m_pio_regs_table[port];
	return regs->IDR;
}

int mcu_pio_set(const devfs_handle_t * handle, void * ctl){
	int port = handle->port;
	GPIO_TypeDef * regs = m_pio_regs_table[port];
	regs->ODR = (u32)ctl;
	return 0;
}

void exec_cancelled0(){
	//mcu_execute_event_handler(&(_mcu_pio0_local.handler), MCU_EVENT_SET_CODE(MCU_EVENT_OP_CANCELLED));
}

void exec_cancelled2(){
	//mcu_execute_event_handler(&(_mcu_pio2_local.handler), MCU_EVENT_SET_CODE(MCU_EVENT_OP_CANCELLED));
}

//On __lpc17xx The pio interrupts use the eint3 interrupt service routine -- this function should be called from there
void _mcu_core_pio0_isr(){

}


#endif

