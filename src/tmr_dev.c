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

#include "tmr_local.h"


#define NUM_OCS MCU_TMR_CHANNELS
#define NUM_ICS MCU_TMR_CHANNELS



#if MCU_TMR_PORTS > 0

TIM_TypeDef * const tmr_regs_table[MCU_TMR_PORTS] = MCU_TMR_REGS;
u8 const tmr_irqs[MCU_TMR_PORTS] = MCU_TMR_IRQS;
u32 const tmr_channels[MCU_TMR_CHANNELS] = MCU_TMR_CHANNEL_NAMES;

static u8 decode_hal_channel(u8 channel){
	switch(channel){
		case HAL_TIM_ACTIVE_CHANNEL_1: return 0;
		case HAL_TIM_ACTIVE_CHANNEL_2: return 1;
		case HAL_TIM_ACTIVE_CHANNEL_3: return 2;
		case HAL_TIM_ACTIVE_CHANNEL_4: return 3;
#if MCU_TMR_API > 0
		case HAL_TIM_ACTIVE_CHANNEL_5: return 4;
		case HAL_TIM_ACTIVE_CHANNEL_6: return 5;
#endif
	}
	return 0;
}


tmr_local_t m_tmr_local[MCU_TMR_PORTS] MCU_SYS_MEM;

static void clear_actions(int port);

static int execute_handler(mcu_event_handler_t * handler, u32 o_events, u32 channel, u32 value);

void clear_actions(int port){
	memset(m_tmr_local[port].handler, 0, sizeof(m_tmr_local[port].handler));
}

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(tmr, TMR_VERSION, TMR_IOC_IDENT_CHAR, I_MCU_TOTAL + I_TMR_TOTAL, mcu_tmr_setchannel, mcu_tmr_getchannel, mcu_tmr_set, mcu_tmr_get, mcu_tmr_enable, mcu_tmr_disable)

int mcu_tmr_open(const devfs_handle_t * handle){
	int port = handle->port;
	if ( m_tmr_local[port].ref_count == 0 ){
		clear_actions(port);
		m_tmr_local[port].hal_handle.Instance = tmr_regs_table[port];
		switch(port){
			default:
				return SYSFS_SET_RETURN(EINVAL);
				TIM1_CASE_CLOCK_ENABLE()
						TIM2_CASE_CLOCK_ENABLE()
						TIM3_CASE_CLOCK_ENABLE()
						TIM4_CASE_CLOCK_ENABLE()
						TIM5_CASE_CLOCK_ENABLE()
						TIM6_CASE_CLOCK_ENABLE()
						TIM7_CASE_CLOCK_ENABLE()
						TIM8_CASE_CLOCK_ENABLE()
						TIM9_CASE_CLOCK_ENABLE()
						TIM10_CASE_CLOCK_ENABLE()
						TIM11_CASE_CLOCK_ENABLE()
						TIM12_CASE_CLOCK_ENABLE()
						TIM13_CASE_CLOCK_ENABLE()
						TIM14_CASE_CLOCK_ENABLE()
						TIM15_CASE_CLOCK_ENABLE()
						TIM16_CASE_CLOCK_ENABLE()
						TIM17_CASE_CLOCK_ENABLE()

		}
		if( tmr_irqs[port] != (u8)-1 ){
			cortexm_enable_irq(tmr_irqs[port]);
		}
	}
	m_tmr_local[port].ref_count++;
	return 0;
}


int mcu_tmr_close(const devfs_handle_t * handle){
	int port = handle->port;
	if ( m_tmr_local[port].ref_count > 0 ){
		if ( m_tmr_local[port].ref_count == 1 ){
			clear_actions(port);
			m_tmr_local[port].hal_handle.Instance = 0;
			if( tmr_irqs[port] != (u8)-1 ){
				cortexm_disable_irq(tmr_irqs[port]);
			}
			switch(port){
				default:
					return SYSFS_SET_RETURN(EINVAL);
					TIM1_CASE_CLOCK_DISABLE()
							TIM2_CASE_CLOCK_DISABLE()
							TIM3_CASE_CLOCK_DISABLE()
							TIM4_CASE_CLOCK_DISABLE()
							TIM5_CASE_CLOCK_DISABLE()
							TIM6_CASE_CLOCK_DISABLE()
							TIM7_CASE_CLOCK_DISABLE()
							TIM8_CASE_CLOCK_DISABLE()
							TIM9_CASE_CLOCK_DISABLE()
							TIM10_CASE_CLOCK_DISABLE()
							TIM11_CASE_CLOCK_DISABLE()
							TIM12_CASE_CLOCK_DISABLE()
							TIM13_CASE_CLOCK_DISABLE()
							TIM14_CASE_CLOCK_DISABLE()
							TIM15_CASE_CLOCK_DISABLE()
							TIM16_CASE_CLOCK_DISABLE()
							TIM17_CASE_CLOCK_DISABLE()
			}
			m_tmr_local[port].ref_count--;
		}
	}
	return 0;
}


int mcu_tmr_getinfo(const devfs_handle_t * handle, void * ctl){
	tmr_info_t * info = ctl;

	// set supported flags and events
	info->o_flags = TMR_FLAG_IS_AUTO_RELOAD |
			TMR_FLAG_SET_TIMER |
			TMR_FLAG_IS_SOURCE_COUNTDOWN |
			TMR_FLAG_SET_CHANNEL |
			TMR_FLAG_IS_CHANNEL_TOGGLE_OUTPUT_ON_MATCH |
			TMR_FLAG_IS_CHANNEL_CLEAR_OUTPUT_ON_MATCH |
			TMR_FLAG_IS_CHANNEL_SET_OUTPUT_ON_MATCH |
			TMR_FLAG_IS_CHANNEL_PWM_MODE;
	info->o_events = 0;


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


			if( o_flags & TMR_FLAG_IS_SOURCE_IC1 ){

			}

			//Set the prescalar so that the freq is correct
			//get the peripheral clock frequency

			u32 pclk;
			if( (((u32)m_tmr_local[port].hal_handle.Instance) & ~0xFFFF) == APB1PERIPH_BASE ){
				pclk = HAL_RCC_GetPCLK1Freq();
			} else {
				pclk = HAL_RCC_GetPCLK2Freq();
			}

			if( pclk <= mcu_board_config.core_cpu_freq/2 ){
				//timer clocks are double pclks
				pclk = pclk * 2;
			}

			mcu_debug_log_info(MCU_DEBUG_DEVICE, "Use pclk is %ld", pclk);

			if ( freq < pclk*2 ){
				m_tmr_local[port].hal_handle.Init.Prescaler = ((pclk + freq/2) / freq) - 1;
				mcu_debug_log_info(MCU_DEBUG_DEVICE, "Prescaler:%ld", m_tmr_local[port].hal_handle.Init.Prescaler);
			} else {
				m_tmr_local[port].hal_handle.Init.Prescaler = 0;
			}

			if( o_flags & TMR_FLAG_IS_SOURCE_COUNTDOWN ){
				m_tmr_local[port].hal_handle.Init.CounterMode = TIM_COUNTERMODE_DOWN;
			} else {
				m_tmr_local[port].hal_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
			}

			if( o_flags & TMR_FLAG_IS_AUTO_RELOAD ){
				m_tmr_local[port].hal_handle.Init.Period = attr->period;
			} else {
				m_tmr_local[port].hal_handle.Init.Period = (u32)-1; //set to the max
			}
			m_tmr_local[port].hal_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

#if defined TIM_AUTORELOAD_PRELOAD_DISABLE
			m_tmr_local[port].hal_handle.Init.RepetitionCounter = 0x00;
			m_tmr_local[port].hal_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
#endif

			if( HAL_TIM_Base_Init(&m_tmr_local[port].hal_handle) != HAL_OK ){
				return SYSFS_SET_RETURN(EIO);
			}

			if( o_flags & TMR_FLAG_IS_SOURCE_CPU ){
				//configure to use the internal clock
				TIM_ClockConfigTypeDef clock_source_config;
				clock_source_config.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
				if( HAL_TIM_ConfigClockSource(&m_tmr_local[port].hal_handle, &clock_source_config) != HAL_OK ){
					return SYSFS_SET_RETURN(EIO);
				}
			}
		}
	}


	if( o_flags & TMR_FLAG_SET_TRIGGER ){

		if( o_flags & TMR_FLAG_IS_SLAVE ){

		} else {
			TIM_MasterConfigTypeDef master_config;

			master_config.MasterOutputTrigger = TIM_TRGO_RESET;
			if( o_flags & TMR_FLAG_IS_TRIGGER_RELOAD ){
				master_config.MasterOutputTrigger = TIM_TRGO_UPDATE;
			} else if( o_flags & TMR_FLAG_IS_TRIGGER_OC0 ){
				master_config.MasterOutputTrigger = TIM_TRGO_OC1REF;
			} else if( o_flags & TMR_FLAG_IS_TRIGGER_OC1 ){
				master_config.MasterOutputTrigger = TIM_TRGO_OC2REF;
			} else if( o_flags & TMR_FLAG_IS_TRIGGER_OC2 ){
				master_config.MasterOutputTrigger = TIM_TRGO_OC3REF;
			} else if( o_flags & TMR_FLAG_IS_TRIGGER_OC3 ){
				master_config.MasterOutputTrigger = TIM_TRGO_OC4REF;
			}

			if( o_flags & TMR_FLAG_IS_MASTER ){
				master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
			} else {
				master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
			}
			if (HAL_TIMEx_MasterConfigSynchronization(&m_tmr_local[port].hal_handle, &master_config) != HAL_OK){
				return SYSFS_SET_RETURN(EIO);
			}
		}
	}


	if( o_flags & TMR_FLAG_SET_CHANNEL ){
		int chan = attr->channel.loc & ~MCU_CHANNEL_FLAG_IS_INPUT;
		int channel_configure_type;
		int ret;
		TIM_OC_InitTypeDef init_oc;
		TIM_IC_InitTypeDef init_ic;

		if( chan >= MCU_TMR_CHANNELS ){
			return SYSFS_SET_RETURN(EINVAL);
		}

		channel_configure_type = CHANNEL_TYPE_NONE;

		u32 tim_channel = tmr_channels[chan];

		if(attr->channel.loc &  MCU_CHANNEL_FLAG_IS_INPUT){
			TIM_IC_InitTypeDef init_ic;
			channel_configure_type = CHANNEL_TYPE_INPUT_CAPTURE;
			memset(&init_ic, 0, sizeof(init_ic));

		} else {

			if( o_flags & TMR_FLAG_IS_CHANNEL_PWM_MODE ) {
				channel_configure_type = CHANNEL_TYPE_PWM;
				init_oc.OCMode = TIM_OCMODE_PWM1; //output is ON until the value is hit
				if( o_flags & TMR_FLAG_IS_CHANNEL_SET_OUTPUT_ON_MATCH ){
					init_oc.OCMode = TIM_OCMODE_PWM2; //output if OFF until the value hits
				}
			} else if( o_flags & TMR_FLAG_IS_CHANNEL_TOGGLE_OUTPUT_ON_MATCH ){
				channel_configure_type = CHANNEL_TYPE_OUTPUT_COMPARE;
				init_oc.OCMode = TIM_OCMODE_TOGGLE;
			} else if ( o_flags & TMR_FLAG_IS_CHANNEL_SET_OUTPUT_ON_MATCH ){
				channel_configure_type = CHANNEL_TYPE_OUTPUT_COMPARE;
				init_oc.OCMode = TIM_OCMODE_ACTIVE;
			} else if( o_flags & TMR_FLAG_IS_CHANNEL_CLEAR_OUTPUT_ON_MATCH ){
				channel_configure_type = CHANNEL_TYPE_OUTPUT_COMPARE;
				init_oc.OCMode = TIM_OCMODE_INACTIVE;
			} else {
				init_oc.OCMode = TIM_OCMODE_TIMING;
			}

			ret = 0;

			init_oc.Pulse = 0;
			init_oc.OCPolarity = TIM_OCPOLARITY_HIGH;
			init_oc.OCFastMode = TIM_OCFAST_DISABLE;
		}

		switch(channel_configure_type){
			case CHANNEL_TYPE_OUTPUT_COMPARE:
				ret = HAL_TIM_OC_Init(&m_tmr_local[port].hal_handle);
				if( ret != HAL_OK ){ return SYSFS_SET_RETURN(EIO); }
				ret = HAL_TIM_OC_ConfigChannel(&m_tmr_local[port].hal_handle, &init_oc, tim_channel);
				if( ret != HAL_OK ){ return SYSFS_SET_RETURN(EIO); }
				ret = HAL_TIM_OC_Start(&m_tmr_local[port].hal_handle, tim_channel);
				break;

			case CHANNEL_TYPE_PWM:
				ret = HAL_TIM_PWM_Init(&m_tmr_local[port].hal_handle);
				if( ret != HAL_OK ){ return SYSFS_SET_RETURN(EIO); }
				ret = HAL_TIM_PWM_ConfigChannel(&m_tmr_local[port].hal_handle, &init_oc, tim_channel);
				if( ret != HAL_OK ){ return SYSFS_SET_RETURN(EIO); }
				ret = HAL_TIM_PWM_Start(&m_tmr_local[port].hal_handle, tim_channel);
				break;

			case CHANNEL_TYPE_INPUT_CAPTURE:
				ret = HAL_OK;
				ret = HAL_TIM_IC_Init(&m_tmr_local[port].hal_handle);
				if( ret != HAL_OK ){ return SYSFS_SET_RETURN(EIO); }
				ret = HAL_TIM_IC_ConfigChannel(&m_tmr_local[port].hal_handle, &init_ic, tim_channel);
				if( ret != HAL_OK ){ return SYSFS_SET_RETURN(EIO); }
				ret = HAL_TIM_IC_Start(&m_tmr_local[port].hal_handle, tim_channel);
				break;

			case CHANNEL_TYPE_NONE:
				HAL_TIM_OC_Stop(&m_tmr_local[port].hal_handle, tim_channel);
				HAL_TIM_PWM_Stop(&m_tmr_local[port].hal_handle, tim_channel);
				HAL_TIM_IC_Stop(&m_tmr_local[port].hal_handle, tim_channel);
				ret = HAL_OK;
				break;
		}

		if( ret != HAL_OK ){
			return SYSFS_SET_RETURN(EIO);
		}

		//this sets the value of the channel
		if( mcu_tmr_setchannel(handle, (void*)&attr->channel) < 0 ){
			return SYSFS_SET_RETURN(EIO);
		}
	}

	if( o_flags & (TMR_FLAG_SET_TIMER|TMR_FLAG_SET_CHANNEL) ){
		if( mcu_set_pin_assignment(
				 &(attr->pin_assignment),
				 MCU_CONFIG_PIN_ASSIGNMENT(tmr_config_t, handle),
				 MCU_PIN_ASSIGNMENT_COUNT(tmr_pin_assignment_t),
				 CORE_PERIPH_TMR, port, 0, 0, 0) < 0 ){
			return SYSFS_SET_RETURN(EINVAL);
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
	return SYSFS_RETURN_SUCCESS;
}

int mcu_tmr_disable(const devfs_handle_t * handle, void * ctl){
	int port = handle->port;
	//HAL_TIM_Base_Stop_IT(&m_tmr_local[port].hal_handle);
	m_tmr_local[port].hal_handle.Instance->CR1 &= ~(TIM_CR1_CEN);
	return SYSFS_RETURN_SUCCESS;
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
			return SYSFS_SET_RETURN(EINVAL);
		}
	} else {
		chan = req->loc;
		if ( chan >= NUM_OCS ){
			return SYSFS_SET_RETURN(EINVAL);
		}
	}

	((u32*)&(regs->CCR1))[ chan ] = req->value;

	return SYSFS_RETURN_SUCCESS;
}

int mcu_tmr_getchannel(const devfs_handle_t * handle, void * ctl){
	TIM_TypeDef * regs;
	int port = handle->port;
	regs = tmr_regs_table[port];
	mcu_channel_t * req = (mcu_channel_t*)ctl;
	u8 chan;

	if( req->loc & MCU_CHANNEL_FLAG_IS_INPUT ){
		chan = req->loc & ~MCU_CHANNEL_FLAG_IS_INPUT;
		if ( chan >= NUM_ICS ){
			return SYSFS_SET_RETURN(EINVAL);
		}
	} else {
		chan = req->loc;
		if ( chan >= MCU_TMR_CHANNELS ){
			return SYSFS_SET_RETURN(EINVAL);
		}
	}

	req->value = ((u32*)&(regs->CCR1))[ chan ];
	return SYSFS_RETURN_SUCCESS;
}

int mcu_tmr_write(const devfs_handle_t * handle, devfs_async_t * wop){
	return SYSFS_SET_RETURN(ENOTSUP);
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

	if( chan >= MCU_TMR_CHANNELS ){
		return SYSFS_SET_RETURN(EINVAL);
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
	} else if( o_events & MCU_EVENT_FLAG_OVERFLOW ){

		if( action->handler.callback != 0 ){
			HAL_TIM_Base_Start_IT(&m_tmr_local[port].hal_handle);
		}  else {
			HAL_TIM_Base_Stop_IT(&m_tmr_local[port].hal_handle);
		}

		m_tmr_local[port].period_handler = action->handler;
	}

	return SYSFS_RETURN_SUCCESS;
}

int mcu_tmr_read(const devfs_handle_t * handle, devfs_async_t * rop){
	return SYSFS_SET_RETURN(ENOTSUP);
}

int mcu_tmr_set(const devfs_handle_t * handle, void * ctl){
	TIM_TypeDef * regs;
	int port = handle->port;
	regs = tmr_regs_table[port];
	regs->CNT = (u32)ctl;
	return SYSFS_RETURN_SUCCESS;
}

int mcu_tmr_get(const devfs_handle_t * handle, void * ctl){
	u32 * value = ctl;
	if( value ){
		*value = tmr_regs_table[handle->port]->CNT;
		return SYSFS_RETURN_SUCCESS;
	}
	return SYSFS_SET_RETURN(EINVAL);
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
	execute_handler(&local->period_handler, MCU_EVENT_FLAG_OVERFLOW, -1, htim->Instance->ARR);
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim){
	//this happens on an output compare match
	tmr_local_t * local = (tmr_local_t*)htim;
	u8 channel = decode_hal_channel(htim->Channel);
	execute_handler(&local->handler[channel], MCU_EVENT_FLAG_MATCH, channel, htim->Instance->CNT);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	//this happens on an output compare match
	tmr_local_t * local = (tmr_local_t*)htim;
	u8 channel = decode_hal_channel(htim->Channel);
	execute_handler(&local->handler[channel], MCU_EVENT_FLAG_MATCH, channel | MCU_CHANNEL_FLAG_IS_INPUT, htim->Instance->CNT);
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

#if defined TIM1
void mcu_core_tim1_cc_isr(){ tmr_isr(0); }
#endif

#if defined TIM2
void mcu_core_tim2_isr(){ tmr_isr(1); }
#endif

#if defined TIM3
void mcu_core_tim3_isr(){ tmr_isr(2); }
#endif

#if defined TIM4
void mcu_core_tim4_isr(){ tmr_isr(3); }
#endif

#if defined TIM5
void mcu_core_tim5_isr(){ tmr_isr(4); }
#endif

void mcu_core_dac_isr() MCU_WEAK;
void mcu_core_dac_isr(){}
#if defined TIM6
void mcu_core_tim6_dac_isr(){
	//TIM6 is shared with the DAC
	mcu_core_dac_isr();
	if( m_tmr_local[5].hal_handle.Instance != 0 ){
		tmr_isr(5);
	}
}
#endif

#if defined TIM7
void mcu_core_tim7_isr(){ tmr_isr(6); }
#endif

#if defined TIM8
void mcu_core_tim8_cc_isr(){ tmr_isr(7); }
#endif

#if defined TIM9
void mcu_core_tim1_brk_tim9_isr(){ tmr_isr(8); }
#endif

#if defined TIM10
void mcu_core_tim1_up_tim10_isr(){ tmr_isr(9); }
#endif

#if defined TIM11
void mcu_core_tim1_trg_com_tim11_isr(){ tmr_isr(10); }
#endif

#if defined TIM12
void mcu_core_tim8_brk_tim12_isr(){ tmr_isr(11); }
#endif

#if defined TIM13
void mcu_core_tim8_up_tim13_isr(){ tmr_isr(12); }
#endif

#if defined TIM14
void mcu_core_tim8_trg_com_tim14_isr(){ tmr_isr(13); }
#endif

#if defined TIM15
void mcu_core_tim15_isr(){ tmr_isr(14); }
#endif

#if defined TIM16
void mcu_core_tim16_isr(){ tmr_isr(15); }
#endif

#if defined TIM17
void mcu_core_tim17_isr(){ tmr_isr(16); }
#endif

#endif

