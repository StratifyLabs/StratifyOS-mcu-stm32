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
#include <stdbool.h>
#include <mcu/tmr.h>
#include <mcu/cortexm.h>
#include <mcu/core.h>

#include "hal.h"
#include "stm32f4xx/stm32f4xx_ll_tim.h"

#define NUM_TMRS MCU_TMR_PORTS
#define NUM_OCS 4
#define NUM_ICS 2


#define MR0_FLAG (1<<0)
#define MR1_FLAG (1<<1)
#define MR2_FLAG (1<<2)
#define MR3_FLAG (1<<3)
#define CR0_FLAG (1<<4)
#define CR1_FLAG (1<<5)

#if MCU_TMR_PORTS > 0

static TIM_TypeDef * const tmr_regs_table[NUM_TMRS] = MCU_TMR_REGS;
static u8 const tmr_irqs[MCU_TMR_PORTS] = MCU_TMR_IRQS;



struct tmr_cfg {
	uint8_t pin_assign;
	uint8_t enabled_oc_chans;
	uint8_t enabled_ic_chans;
};

typedef struct MCU_PACK {
	mcu_event_handler_t handler[NUM_OCS+NUM_ICS];
	struct tmr_cfg cfg;
	uint8_t ref_count;
} tmr_local_t;

static tmr_local_t m_mcu_tmr_local[NUM_TMRS];

static void _mcu_tmr_clear_actions(int port);

void _mcu_tmr_clear_actions(int port){
	memset(m_mcu_tmr_local[port].handler, 0, (NUM_OCS+NUM_ICS)*sizeof(mcu_event_handler_t));
}

void _mcu_tmr_dev_power_on(int port){
	if ( m_mcu_tmr_local[port].ref_count == 0 ){
		_mcu_tmr_clear_actions(port);
		/*
		switch(port){
		case 0:
			_mcu_lpc_core_enable_pwr(PCTIM0);
			break;
		case 1:
			_mcu_lpc_core_enable_pwr(PCTIM1);
			break;
		case 2:
			_mcu_lpc_core_enable_pwr(PCTIM2);
			break;
		case 3:
			_mcu_lpc_core_enable_pwr(PCTIM3);
			break;
		}
		*/
		if( tmr_irqs[port] >= 0 ){
			_mcu_cortexm_priv_enable_irq((void*)(u32)(tmr_irqs[port]));
		}
	}
	m_mcu_tmr_local[port].ref_count++;
}


void _mcu_tmr_dev_power_off(int port){
	if ( m_mcu_tmr_local[port].ref_count > 0 ){
		if ( m_mcu_tmr_local[port].ref_count == 1 ){
			_mcu_tmr_clear_actions(port);
			_mcu_cortexm_priv_disable_irq((void*)(u32)(tmr_irqs[port]));
			/*
			switch(port){
			case 0:
				_mcu_lpc_core_disable_pwr(PCTIM0);
				break;
			case 1:
				_mcu_lpc_core_disable_pwr(PCTIM1);
				break;
			case 2:
				_mcu_lpc_core_disable_pwr(PCTIM2);
				break;
			case 3:
				_mcu_lpc_core_disable_pwr(PCTIM3);
				break;
			}
			*/
		}
		m_mcu_tmr_local[port].ref_count--;
	}
}

int _mcu_tmr_dev_powered_on(int port){
	return ( m_mcu_tmr_local[port].ref_count != 0);
}

int mcu_tmr_getattr(int port, void * ctl){

	return 0;
}

int mcu_tmr_setattr(int port, void * ctl){

	return 0;
}

int mcu_tmr_on(int port, void * ctl){
	TIM_TypeDef * regs;
	TIM_HandleTypeDef handle;
	regs = tmr_regs_table[port];
	handle.Instance = regs;
	HAL_TIM_Base_Start(&handle);
	return 0;
}

int mcu_tmr_off(int port, void * ctl){
	TIM_TypeDef * regs;
	TIM_HandleTypeDef handle;
	regs = tmr_regs_table[port];
	handle.Instance = regs;
	HAL_TIM_Base_Stop(&handle);
	return 0;
}

int mcu_tmr_setoc(int port, void * ctl){

	/*
	TIM_TypeDef * regs;
	regs = tmr_regs_table[port];
	//Write the output compare value
	tmr_reqattr_t * req = (tmr_reqattr_t*)ctl;
	if ( req->channel > 3 ){
		errno = EINVAL;
		return -1;
	}

#if MCU_TMR_API == 1
	regs->MR[req->channel] = req->value;
#else
	((uint32_t*)&(regs->MR0))[ req->channel ] = req->value;
#endif
*/
	return 0;
}

int mcu_tmr_getoc(int port, void * ctl){

	/*
	TIM_TypeDef * regs;
	regs = tmr_regs_table[port];
	//Read the output compare channel
	tmr_reqattr_t * req = (tmr_reqattr_t*)ctl;
	if ( req->channel > 3 ){
		errno = EINVAL;
		return -1;
	}
#if MCU_TMR_API == 1
	req->value = regs->MR[req->channel];
#else
	req->value = ((uint32_t*)&(regs->MR0))[ req->channel ];
#endif
*/
	return 0;

}

int mcu_tmr_setic(int port, void * ctl){
	/*
	TIM_TypeDef * regs;
	regs = tmr_regs_table[port];
	unsigned int chan;
	tmr_reqattr_t * req = (tmr_reqattr_t*)ctl;
	chan = req->channel - TMR_ACTION_CHANNEL_IC0;
	if ( chan > 1 ){
		errno = EINVAL;
		return -1;
	}
#if MCU_TMR_API == 1
	regs->CR[chan] = req->value;
#else
	((uint32_t*)&(regs->CR0))[ req->channel ] = req->value;
#endif
*/
	return 0;
}

int mcu_tmr_getic(int port, void * ctl){
	/*
	TIM_TypeDef * regs;
	unsigned int chan;
	regs = tmr_regs_table[port];
	tmr_reqattr_t * req = (tmr_reqattr_t*)ctl;
	chan = req->channel - TMR_ACTION_CHANNEL_IC0;
	if ( chan > 1 ){
		errno = EINVAL;
		return -1;
	}
#if MCU_TMR_API == 1
	req->value = regs->CR[chan];
#else
	req->value = ((uint32_t*)&(regs->CR0))[ chan ];
#endif
*/
	return 0;
}

int _mcu_tmr_dev_write(const devfs_handle_t * cfg, devfs_async_t * wop){
	int port;
	mcu_action_t * action;
	int chan;
	action = (mcu_action_t*)wop->buf;

	if( wop->nbyte != sizeof(mcu_action_t) ){
		errno = EINVAL;
		return -1;
	}

	port = cfg->port;
	chan = action->channel;
	if ( m_mcu_tmr_local[port].handler[chan].callback != 0 ){
		//The interrupt is on -- port is busy
		errno = EAGAIN;
		return -1;
	}

	action = (mcu_action_t*)wop->buf;
	action->handler = wop->handler;

	_mcu_cortexm_set_irq_prio(tmr_irqs[port], action->prio);

	return mcu_tmr_setaction(port, action);
}


int mcu_tmr_setaction(int port, void * ctl){

	return 0;
}

int _mcu_tmr_dev_read(const devfs_handle_t * cfg, devfs_async_t * rop){
	int port = DEVFS_GET_PORT(cfg);
	int chan = rop->loc;

	if( _mcu_cortexm_priv_validate_callback(rop->handler.callback) < 0 ){
		return -1;
	}

	m_mcu_tmr_local[port].handler[chan] = rop->handler;
	return 0;
}

int mcu_tmr_set(int port, void * ctl){
	TIM_TypeDef * regs;
	regs = tmr_regs_table[port];
	//regs->TC = (uint32_t)ctl;
	return 0;
}

int mcu_tmr_get(int port, void * ctl){
	TIM_TypeDef * regs;
	regs = tmr_regs_table[port];
	//return regs->TC;
	return 0;
}

static void tmr_isr(int port); //This is speed optimized
//void tmr_isr(int port); //This is size optimized

//Four timers with 4 OC's and 2 IC's each
void tmr_isr(int port){

	/*
	int flags;
	TIM_TypeDef * regs;
	regs = tmr_regs_table[port];
	flags = (regs->IR & 0x3F);
	regs->IR = flags; //clear the flags
	//execute the callbacks
#if MCU_TMR_API == 1
	if( flags & MR0_FLAG ){
		mcu_execute_event_handler(&(m_mcu_tmr_local[port].handler[0]), (mcu_event_t)regs->MR[0]);
	}
	if( flags & MR1_FLAG ){
		mcu_execute_event_handler(&(m_mcu_tmr_local[port].handler[1]), (mcu_event_t)regs->MR[1]);
	}
	if( flags & MR2_FLAG ){
		mcu_execute_event_handler(&(m_mcu_tmr_local[port].handler[2]), (mcu_event_t)regs->MR[2]);
	}
	if( flags & MR3_FLAG ){
		mcu_execute_event_handler(&(m_mcu_tmr_local[port].handler[3]), (mcu_event_t)regs->MR[3]);
	}
	if( flags & CR0_FLAG ){
		mcu_execute_event_handler(&(m_mcu_tmr_local[port].handler[4]), (mcu_event_t)regs->CR[0]);
	}
	if( flags & CR1_FLAG ){
		mcu_execute_event_handler(&(m_mcu_tmr_local[port].handler[5]), (mcu_event_t)regs->CR[1]);
	}
#else
	if( flags & MR0_FLAG ){
		mcu_execute_event_handler(&(m_mcu_tmr_local[port].handler[0]), (mcu_event_t)regs->MR0);
	}
	if( flags & MR1_FLAG ){
		mcu_execute_event_handler(&(m_mcu_tmr_local[port].handler[1]), (mcu_event_t)regs->MR1);
	}
	if( flags & MR2_FLAG ){
		mcu_execute_event_handler(&(m_mcu_tmr_local[port].handler[2]), (mcu_event_t)regs->MR2);
	}
	if( flags & MR3_FLAG ){
		mcu_execute_event_handler(&(m_mcu_tmr_local[port].handler[3]), (mcu_event_t)regs->MR3);
	}
	if( flags & CR0_FLAG ){
		mcu_execute_event_handler(&(m_mcu_tmr_local[port].handler[4]), (mcu_event_t)regs->CR0);
	}
	if( flags & CR1_FLAG ){
		mcu_execute_event_handler(&(m_mcu_tmr_local[port].handler[5]), (mcu_event_t)regs->CR1);
	}
#endif
*/
}

void _mcu_core_tmr0_isr(){ tmr_isr(0); }
void _mcu_core_tmr1_isr(){ tmr_isr(1); }
void _mcu_core_tmr2_isr(){ tmr_isr(2); }
void _mcu_core_tmr3_isr(){ tmr_isr(3); }

#endif

