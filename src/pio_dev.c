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

#include <mcu/pio.h>
#include <mcu/core.h>

#include "stm32_local.h"


#if MCU_PIO_PORTS > 0

typedef struct {
	u8 ref_count;
} pio_local_t;

typedef struct {
	mcu_event_handler_t handler;
	u32 o_events;
	u8 pull_mode;
	GPIO_TypeDef * pio_regs;
} pio_local_event_handler_t;

static pio_local_t m_mcu_pio_local[MCU_PIO_PORTS] MCU_SYS_MEM;
static GPIO_TypeDef * const m_pio_regs_table[MCU_PIO_PORTS] = MCU_PIO_REGS;
//static u8 const m_pio_irqs[MCU_PIO_PORTS] = MCU_PIO_IRQS;

static pio_local_event_handler_t pio_local_event_handler[16] MCU_SYS_MEM; //there are 23 events but on 16 interrupts/events
static const u8 pio_local_event_irq_numbers[] = {
	EXTI0_IRQn, EXTI1_IRQn, EXTI2_IRQn, EXTI3_IRQn, EXTI4_IRQn,
	EXTI9_5_IRQn, EXTI9_5_IRQn, EXTI9_5_IRQn, EXTI9_5_IRQn, EXTI9_5_IRQn,
	EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn
};

//this function is used by other modules to access pio regs
GPIO_TypeDef * const hal_get_pio_regs(u8 port){
	if( port < MCU_PIO_PORTS ){
		return m_pio_regs_table[port];
	}
	return 0;
}

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(pio, PIO_VERSION, PIO_IOC_IDENT_CHAR, I_MCU_TOTAL + I_PIO_TOTAL, mcu_pio_setmask, mcu_pio_clrmask, mcu_pio_get, mcu_pio_set)


int mcu_pio_open(const devfs_handle_t * handle){
	int port = handle->port;
	if ( m_mcu_pio_local[port].ref_count == 0 ){
		switch(port){
			case 0: __HAL_RCC_GPIOA_CLK_ENABLE(); break;
			case 1: __HAL_RCC_GPIOB_CLK_ENABLE(); break;
			case 2: __HAL_RCC_GPIOC_CLK_ENABLE(); break;
#if defined GPIOD
			case 3: __HAL_RCC_GPIOD_CLK_ENABLE(); break;
#endif
#if defined GPIOE
			case 4: __HAL_RCC_GPIOE_CLK_ENABLE(); break;
#endif
#if defined GPIOF
			case 5: __HAL_RCC_GPIOF_CLK_ENABLE(); break;
#endif
#if defined GPIOG
			case 6: __HAL_RCC_GPIOG_CLK_ENABLE(); break;
#endif
			case 7: __HAL_RCC_GPIOH_CLK_ENABLE(); break;
#if defined GPIOI
            case 8: __HAL_RCC_GPIOI_CLK_ENABLE(); break;
#endif

		}
	}
	m_mcu_pio_local[port].ref_count++;
	return 0;
}

int mcu_pio_close(const devfs_handle_t * handle){
	int port = handle->port;
	if ( m_mcu_pio_local[port].ref_count > 0 ){
		if ( m_mcu_pio_local[port].ref_count == 1 ){
			//
		}
		m_mcu_pio_local[port].ref_count--;
	}
	return 0;
}

int mcu_pio_dev_is_powered(const devfs_handle_t * handle){
	int port = handle->port;
	if ( m_mcu_pio_local[port].ref_count > 0 ){
		return 1;
	}
	return 0;
}

int mcu_pio_write(const devfs_handle_t * handle, devfs_async_t * wop){
	mcu_action_t * action;

	if( wop->nbyte != sizeof(mcu_action_t) ){
		return SYSFS_SET_RETURN(EINVAL);
	}

	action = (mcu_action_t*)wop->buf;
	action->handler = wop->handler;
	return mcu_pio_setaction(handle, action);
}

int mcu_pio_read(const devfs_handle_t * cfg, devfs_async_t * async){
	return SYSFS_SET_RETURN(ENOTSUP);
}

int mcu_pio_setaction(const devfs_handle_t * handle, void * ctl){
	const mcu_action_t * action = ctl;

	if( action->channel < 16 ){
		GPIO_TypeDef * regs = m_pio_regs_table[handle->port];
		GPIO_InitTypeDef gpio_init;
		pio_local_event_handler_t * local_handler = pio_local_event_handler + action->channel;

		if( action->handler.callback ){

			if( cortexm_validate_callback(action->handler.callback) < 0 ){ return SYSFS_SET_RETURN(EPERM); }

			if( action->o_events & MCU_EVENT_FLAG_RISING ){
				if( action->o_events & MCU_EVENT_FLAG_FALLING ){
					gpio_init.Mode = GPIO_MODE_IT_RISING_FALLING;
				} else {
					gpio_init.Mode = GPIO_MODE_IT_RISING;
				}
			} else if( action->o_events & MCU_EVENT_FLAG_FALLING ){
				gpio_init.Mode = GPIO_MODE_IT_FALLING;
			}

			//set pull to current status
			gpio_init.Pull = local_handler->pull_mode;
			gpio_init.Pin = 1<<action->channel;

			local_handler->o_events = action->o_events;
			local_handler->handler = action->handler;
			local_handler->pio_regs = regs;

			cortexm_enable_irq( pio_local_event_irq_numbers[action->channel] );
			HAL_GPIO_Init(regs, &gpio_init);
		} else {
			//cancel the event
			cortexm_disable_irq( pio_local_event_irq_numbers[action->channel] );
			devfs_execute_event_handler(&local_handler->handler, MCU_EVENT_FLAG_CANCELED, 0);
			local_handler->handler.callback = 0;

		}
		cortexm_set_irq_priority(pio_local_event_irq_numbers[action->channel], action->prio, action->o_events);
	} else {
		return SYSFS_SET_RETURN(EINVAL);
	}

	return 0;
}


int mcu_pio_getinfo(const devfs_handle_t * handle, void * ctl){
	//read the direction pin status
	return SYSFS_SET_RETURN(ENOTSUP);
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
	for(u32 i=0; i < 16; i++){
		if( attr->o_pinmask & (1<<i) ){
			pio_local_event_handler[i].pull_mode = gpio_init.Pull;
		}
	}

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
	u32 * value = ctl;
	if( value ){
		*value = m_pio_regs_table[handle->port]->IDR;
		return SYSFS_RETURN_SUCCESS;
	}
	return SYSFS_SET_RETURN(EINVAL);
}

int mcu_pio_set(const devfs_handle_t * handle, void * ctl){
	int port = handle->port;
	GPIO_TypeDef * regs = m_pio_regs_table[port];
	regs->ODR = (u32)ctl;
	return 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	u8 pin_number = 31 - __CLZ(GPIO_Pin);
	pio_local_event_handler_t * local_handler = pio_local_event_handler + pin_number;
	pio_event_data_t event_data;
	event_data.status = local_handler->pio_regs->IDR;
	event_data.falling = ((local_handler->o_events & MCU_EVENT_FLAG_FALLING) != 0) << pin_number;
	event_data.rising = ((local_handler->o_events & MCU_EVENT_FLAG_RISING) != 0) << pin_number;

	if( devfs_execute_event_handler(&local_handler->handler, local_handler->o_events, &event_data) == 0 ){
		//cancel the action -- what IO is it on? Or just disable the interrupt
		HAL_GPIO_DeInit(local_handler->pio_regs, GPIO_Pin);

		GPIO_InitTypeDef gpio_init;
		gpio_init.Pull = local_handler->pull_mode;
		gpio_init.Mode = GPIO_MODE_INPUT;
		gpio_init.Pin = GPIO_Pin;
		HAL_GPIO_Init(local_handler->pio_regs, &gpio_init);
	}
}


//argument shouldn't be zero
void mcu_core_exti0_isr(){
	HAL_GPIO_EXTI_IRQHandler(1<<0);
}

void mcu_core_exti1_isr(){
	HAL_GPIO_EXTI_IRQHandler(1<<1);
}

void mcu_core_exti2_isr(){
	HAL_GPIO_EXTI_IRQHandler(1<<2);
}


void mcu_core_exti3_isr(){
	HAL_GPIO_EXTI_IRQHandler(1<<3);
}


void mcu_core_exti4_isr(){
	HAL_GPIO_EXTI_IRQHandler(1<<4);
}

//pins 5 to 9
void mcu_core_exti9_5_isr(){
	HAL_GPIO_EXTI_IRQHandler(1<<5);
	HAL_GPIO_EXTI_IRQHandler(1<<6);
	HAL_GPIO_EXTI_IRQHandler(1<<7);
	HAL_GPIO_EXTI_IRQHandler(1<<8);
	HAL_GPIO_EXTI_IRQHandler(1<<9);
}

//pins 10 to 15
void mcu_core_exti15_10_isr(){
	HAL_GPIO_EXTI_IRQHandler(1<<10);
	HAL_GPIO_EXTI_IRQHandler(1<<11);
	HAL_GPIO_EXTI_IRQHandler(1<<12);
	HAL_GPIO_EXTI_IRQHandler(1<<13);
	HAL_GPIO_EXTI_IRQHandler(1<<14);
	HAL_GPIO_EXTI_IRQHandler(1<<15);
}



#endif
