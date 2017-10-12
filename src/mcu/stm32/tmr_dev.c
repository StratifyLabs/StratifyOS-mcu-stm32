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
#include "cortexm/cortexm.h"
#include "mcu/tmr.h"
#include "mcu/core.h"

#include "hal.h"

#define NUM_TMRS MCU_TMR_PORTS
#define NUM_OCS 4
#define NUM_ICS 4


#define MR0_FLAG (1<<0)
#define MR1_FLAG (1<<1)
#define MR2_FLAG (1<<2)
#define MR3_FLAG (1<<3)
#define CR0_FLAG (1<<4)
#define CR1_FLAG (1<<5)

#if MCU_TMR_PORTS > 0

static TIM_TypeDef * const tmr_regs_table[NUM_TMRS] = MCU_TMR_REGS;
static u8 const tmr_irqs[MCU_TMR_PORTS] = MCU_TMR_IRQS;
static u32 const tmr_channels[4] = {
		TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4
};

typedef struct MCU_PACK {
	TIM_HandleTypeDef hal_handle; //must be first
	mcu_event_handler_t handler[NUM_OCS+NUM_ICS];
	mcu_event_handler_t period_handler;
	u8 ref_count;
} tmr_local_t;

static tmr_local_t m_tmr_local[NUM_TMRS];

static void clear_actions(int port);

static int execute_handler(mcu_event_handler_t * handler, u32 o_events, u32 channel, u32 value);

void clear_actions(int port){
	memset(m_tmr_local[port].handler, 0, (NUM_OCS+NUM_ICS)*sizeof(mcu_event_handler_t));
}

void mcu_tmr_dev_power_on(const devfs_handle_t * handle){
	int port = handle->port;
	if ( m_tmr_local[port].ref_count == 0 ){
		clear_actions(port);
		m_tmr_local[port].hal_handle.Instance = tmr_regs_table[port];
#if 0
		switch(port){
		case 0:
			mcu_lpc_core_enable_pwr(PCTIM0);
			break;
		case 1:
			mcu_lpc_core_enable_pwr(PCTIM1);
			break;
		case 2:
			mcu_lpc_core_enable_pwr(PCTIM2);
			break;
		case 3:
			mcu_lpc_core_enable_pwr(PCTIM3);
			break;
		}
#endif
		cortexm_enable_irq((void*)(u32)(tmr_irqs[port]));
	}
	m_tmr_local[port].ref_count++;
}


void mcu_tmr_dev_power_off(const devfs_handle_t * handle){
	int port = handle->port;
	if ( m_tmr_local[port].ref_count > 0 ){
		if ( m_tmr_local[port].ref_count == 1 ){
			clear_actions(port);
			m_tmr_local[port].hal_handle.Instance = 0;
			cortexm_disable_irq((void*)(u32)(tmr_irqs[port]));
#if 0
			switch(port){
			case 0:
				mcu_lpc_core_disable_pwr(PCTIM0);
				break;
			case 1:
				mcu_lpc_core_disable_pwr(PCTIM1);
				break;
			case 2:
				mcu_lpc_core_disable_pwr(PCTIM2);
				break;
			case 3:
				mcu_lpc_core_disable_pwr(PCTIM3);
				break;
			}
#endif
		}
		m_tmr_local[port].ref_count--;
	}
}

int mcu_tmr_dev_is_powered(const devfs_handle_t * handle){
	int port = handle->port;
	return ( m_tmr_local[port].ref_count != 0);
}

int mcu_tmr_getinfo(const devfs_handle_t * handle, void * ctl){
	tmr_info_t * info = ctl;

	// set supported flags and events
	info->o_flags = TMR_FLAG_IS_AUTO_RELOAD;
	info->o_events = 0;
	//info->freq = mcu_board_config.core_periph_freq / (tmr_regs_table[port]->PR+1);


	return 0;
}

int mcu_tmr_setattr(const devfs_handle_t * handle, void * ctl){
	int port = handle->port;

	const tmr_attr_t * attr = mcu_select_attr(handle, ctl);
	if( attr == 0 ){
		return -1;
	}

	u32 o_flags = attr->o_flags;
	u32 freq = attr->freq;
	int chan = attr->channel.loc;
	//regs = tmr_regs_table[port];


	if( o_flags & TMR_FLAG_SET_TIMER ){

		if( (o_flags & (TMR_FLAG_IS_SOURCE_CPU|TMR_FLAG_IS_SOURCE_IC0|TMR_FLAG_IS_SOURCE_IC1)) ){

			if( o_flags & TMR_FLAG_IS_SOURCE_CPU ){
				if( attr->freq == 0 ){
					freq = 1000000;
				}
			} else {
				if( o_flags & TMR_FLAG_IS_SOURCE_EDGEFALLING ){

				} else if( o_flags & TMR_FLAG_IS_SOURCE_EDGEBOTH ){

				}
			}

			if( mcu_set_pin_assignment(
					&(attr->pin_assignment),
					MCU_CONFIG_PIN_ASSIGNMENT(tmr_config_t, handle),
					MCU_PIN_ASSIGNMENT_COUNT(tmr_pin_assignment_t),
					CORE_PERIPH_TMR, port, 0, 0) < 0 ){
				return -1;
			}


			if( o_flags & TMR_FLAG_IS_SOURCE_IC1 ){

			}

			//Set the prescalar so that the freq is correct
			if ( freq < mcu_board_config.core_periph_freq ){
				m_tmr_local[port].hal_handle.Init.Prescaler = ((mcu_board_config.core_periph_freq + freq/2) / freq);
			} else {
				m_tmr_local[port].hal_handle.Init.Prescaler = 0;
			}

			if( o_flags & TMR_FLAG_IS_SOURCE_COUNTDOWN ){
				m_tmr_local[port].hal_handle.Init.CounterMode = TIM_COUNTERMODE_DOWN;
			} else {
				m_tmr_local[port].hal_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
			}
			m_tmr_local[port].hal_handle.Init.Period = attr->period;

			HAL_TIM_Base_Init(&m_tmr_local[port].hal_handle);
		}

	}


	if( o_flags & TMR_FLAG_SET_CHANNEL ){
		int chan = attr->channel.loc & ~MCU_CHANNEL_FLAG_IS_INPUT;

		if( chan >= NUM_OCS ){
			errno = EINVAL;
			return -1;
		}

		u32 tim_channel = tmr_channels[chan];

		if(attr->channel.loc &  MCU_CHANNEL_FLAG_IS_INPUT){

		} else {
			TIM_OC_InitTypeDef init_oc;

			if( o_flags & TMR_FLAG_IS_CHANNEL_TOGGLE_OUTPUT_ON_MATCH ){
				init_oc.OCMode = TIM_OCMODE_TOGGLE;
			} else if ( o_flags & TMR_FLAG_IS_CHANNEL_SET_OUTPUT_ON_MATCH ){
				init_oc.OCMode = TIM_OCMODE_ACTIVE;
			} else if( o_flags & TMR_FLAG_IS_CHANNEL_SET_OUTPUT_ON_MATCH ){
				init_oc.OCMode = TIM_OCMODE_INACTIVE;
			} else {
				init_oc.OCMode = TIM_OCMODE_TIMING;
			}

			init_oc.Pulse = 0;
			init_oc.OCPolarity = TIM_OCPOLARITY_HIGH;
			init_oc.OCFastMode = TIM_OCFAST_DISABLE;
			HAL_TIM_OC_ConfigChannel(&m_tmr_local[port], &init_oc, tim_channel);
		}


		//Check for reset action
		if ( o_flags & TMR_FLAG_IS_CHANNEL_RESET_ON_MATCH){ //reset on match

		}

		//Check to see if the timer should stop on a match
		if ( o_flags & TMR_FLAG_IS_CHANNEL_STOP_ON_MATCH){

		}

		if( chan < NUM_OCS ){
			if( o_flags & TMR_FLAG_IS_CHANNEL_SET_OUTPUT_ON_MATCH ){
				//set OC output on event
			}

			if( o_flags & TMR_FLAG_IS_CHANNEL_CLEAR_OUTPUT_ON_MATCH ){
				//clr OC output on event
			}

			if( o_flags & TMR_FLAG_IS_CHANNEL_TOGGLE_OUTPUT_ON_MATCH ){
				//toggle OC output on event
			}
		}
	}

	return 0;
}

int mcu_tmr_enable(const devfs_handle_t * handle, void * ctl){
	int port = handle->port;
	if( m_tmr_local[port].hal_handle.Init.Period != 0 ){
		HAL_TIM_Base_Start_IT(&m_tmr_local[port].hal_handle);
	} else {
		HAL_TIM_Base_Start(&m_tmr_local[port].hal_handle);
	}
	return 0;
}

int mcu_tmr_disable(const devfs_handle_t * handle, void * ctl){
	int port = handle->port;
	HAL_TIM_Base_Stop_IT(&m_tmr_local[port].hal_handle);
	return 0;
}

int mcu_tmr_setchannel(const devfs_handle_t * handle, void * ctl){
	TIM_TypeDef * regs;
	int port = handle->port;
	regs = tmr_regs_table[port];
	//Write the output compare value
	mcu_channel_t * req = (mcu_channel_t*)ctl;
	u8 chan;

	if( req->loc & MCU_CHANNEL_FLAG_IS_INPUT ){
		chan = req->loc & ~MCU_CHANNEL_FLAG_IS_INPUT;
		if ( chan >= NUM_ICS ){
			errno = EINVAL;
			return -1;
		}
	} else {
		chan = req->loc;
		if ( chan >= NUM_OCS ){
			errno = EINVAL;
			return -1;
		}
	}

	((u32*)&(regs->CCR1))[ chan ] = req->value;

	return 0;
}

int mcu_tmr_getchannel(const devfs_handle_t * handle, void * ctl){
	TIM_TypeDef * regs;
	int port = handle->port;
	regs = tmr_regs_table[port];
	//Write the output compare value
	mcu_channel_t * req = (mcu_channel_t*)ctl;
	u8 chan;

	if( req->loc & MCU_CHANNEL_FLAG_IS_INPUT ){
		chan = req->loc & ~MCU_CHANNEL_FLAG_IS_INPUT;
		if ( chan >= NUM_ICS ){
			errno = EINVAL;
			return -1;
		}
	} else {
		chan = req->loc;
		if ( chan >= NUM_OCS ){
			errno = EINVAL;
			return -1;
		}
	}

	req->value = ((u32*)&(regs->CCR1))[ chan ];
	return 0;
}

int mcu_tmr_dev_write(const devfs_handle_t * handle, devfs_async_t * wop){
	errno = ENOTSUP;
	return -1;
}


int mcu_tmr_setaction(const devfs_handle_t * handle, void * ctl){
	mcu_action_t * action = (mcu_action_t*)ctl;
	int port = handle->port;
	//regs = tmr_regs_table[port];
	u32 chan;
	u32 o_events;
	u32 tim_channel;

	o_events = action->o_events;
	chan = action->channel & ~MCU_CHANNEL_FLAG_IS_INPUT;

	if( chan >= NUM_OCS ){
		errno = EINVAL;
		return -1;
	}

	tim_channel = tmr_channels[chan];

	if ( o_events == MCU_EVENT_FLAG_NONE ){ //Check to see if all actions are disabled for the channel
		m_tmr_local[port].handler[chan].callback = 0;
		if( action->channel & MCU_CHANNEL_FLAG_IS_INPUT ){
			HAL_TIM_IC_Stop_IT(&m_tmr_local[port].hal_handle, tim_channel);
		} else {
			HAL_TIM_OC_Stop_IT(&m_tmr_local[port].hal_handle, tim_channel);
		}

		//execute a cancelled callback

	} else if( o_events & MCU_EVENT_FLAG_MATCH ){

		if( action->channel & MCU_CHANNEL_FLAG_IS_INPUT ){

			if( action->handler.callback != 0 ){
				HAL_TIM_IC_Start_IT(&m_tmr_local[port].hal_handle, tim_channel);
			}  else {
				HAL_TIM_IC_Stop_IT(&m_tmr_local[port].hal_handle, tim_channel);
			}

		} else {

			if( action->handler.callback != 0 ){
				HAL_TIM_OC_Start_IT(&m_tmr_local[port].hal_handle, tim_channel);
			}  else {
				HAL_TIM_OC_Stop_IT(&m_tmr_local[port].hal_handle, tim_channel);
			}

		}

		m_tmr_local[port].handler[chan] = action->handler;
	}

	return 0;
}

int mcu_tmr_dev_read(const devfs_handle_t * handle, devfs_async_t * rop){
	errno = ENOTSUP;
	return -1;
}

int mcu_tmr_set(const devfs_handle_t * handle, void * ctl){
	TIM_TypeDef * regs;
	int port = handle->port;
	regs = tmr_regs_table[port];
	regs->CNT = (uint32_t)ctl;
	return 0;
}

int mcu_tmr_get(const devfs_handle_t * handle, void * ctl){
	TIM_TypeDef * regs;
	int port = handle->port;
	regs = tmr_regs_table[port];
	return regs->CNT;
}

static void tmr_isr(int port); //This is speed optimized
//void tmr_isr(int port); //This is size optimized

int execute_handler(mcu_event_handler_t * handler, u32 o_events, u32 channel, u32 value){
	tmr_event_t event;
	event.channel.loc = channel;
	event.channel.value = value;
	return mcu_execute_event_handler(handler, o_events, &event);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	//this happens on an overflow
	tmr_local_t * local = (tmr_local_t*)htim;
	execute_handler(&local->period_handler, MCU_EVENT_FLAG_OVERFLOW, -1, 0);
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim){
	//this happens on an output compare match
	tmr_local_t * local = (tmr_local_t*)htim;
	execute_handler(&local->period_handler, MCU_EVENT_FLAG_MATCH, htim->Channel, htim->Instance->CNT);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	//this happens on an output compare match
	tmr_local_t * local = (tmr_local_t*)htim;
	execute_handler(&local->period_handler, MCU_EVENT_FLAG_MATCH, htim->Channel | MCU_CHANNEL_FLAG_IS_INPUT, htim->Instance->CNT);

}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){

	//this belongs in the PWM driver (NOT here)
}

void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim){

}

void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim){

}

//Four timers with 4 OC's and 2 IC's each
void tmr_isr(int port){
	HAL_TIM_IRQHandler(&m_tmr_local[port].hal_handle);
}

void mcu_core_tmr0_isr(){
	tmr_isr(0);
}

void mcu_core_tmr1_isr(){
	tmr_isr(1);
}

void mcu_core_tmr2_isr(){
	tmr_isr(2);
}

void mcu_core_tmr3_isr(){
	tmr_isr(3);
}

#endif

