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

#include <mcu/debug.h>
#include <stdbool.h>
#include "stm32_local.h"
#include "cortexm/cortexm.h"
#include "mcu/tmr.h"
#include "mcu/core.h"


#define NUM_TMRS MCU_TMR_PORTS
#define NUM_OCS 4
#define NUM_ICS 4

enum {
    CHANNEL_TYPE_NONE,
    CHANNEL_TYPE_OUTPUT_COMPARE,
    CHANNEL_TYPE_INPUT_CAPTURE,
    CHANNEL_TYPE_PWM
};

#if MCU_TMR_PORTS > 0

static TIM_TypeDef * const tmr_regs_table[NUM_TMRS] = MCU_TMR_REGS;
static u8 const tmr_irqs[MCU_TMR_PORTS] = MCU_TMR_IRQS;
static u32 const tmr_channels[4] = {
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4
};

static u8 decode_hal_channel(u8 channel){
    switch(channel){
    case HAL_TIM_ACTIVE_CHANNEL_1: return 0;
    case HAL_TIM_ACTIVE_CHANNEL_2: return 1;
    case HAL_TIM_ACTIVE_CHANNEL_3: return 2;
    case HAL_TIM_ACTIVE_CHANNEL_4: return 3;
    }
    return 0;
}

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

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(tmr, TMR_VERSION, I_MCU_TOTAL + I_TMR_TOTAL, mcu_tmr_setchannel, mcu_tmr_getchannel, mcu_tmr_set, mcu_tmr_get, mcu_tmr_enable, mcu_tmr_disable)

int mcu_tmr_open(const devfs_handle_t * handle){
    int port = handle->port;
    if ( m_tmr_local[port].ref_count == 0 ){
        clear_actions(port);
        m_tmr_local[port].hal_handle.Instance = tmr_regs_table[port];
        switch(port){
        case 0:
            __HAL_RCC_TIM1_CLK_ENABLE();
            break;
        case 1:
            __HAL_RCC_TIM2_CLK_ENABLE();
            break;
        case 2:
            __HAL_RCC_TIM3_CLK_ENABLE();
            break;
        case 3:
            __HAL_RCC_TIM4_CLK_ENABLE();
            break;
        case 4:
            __HAL_RCC_TIM5_CLK_ENABLE();
            break;
#if defined __HAL_RCC_TIM6_CLK_ENABLE
        case 5:
            __HAL_RCC_TIM6_CLK_ENABLE();
            break;
#endif
#if defined __HAL_RCC_TIM7_CLK_ENABLE
        case 6:
            __HAL_RCC_TIM7_CLK_ENABLE();
            break;
#endif
#if defined __HAL_RCC_TIM8_CLK_ENABLE
        case 7:
            __HAL_RCC_TIM8_CLK_ENABLE();
            break;
#endif
        }
        if( tmr_irqs[port] != (u8)-1 ){
            cortexm_enable_irq((void*)(u32)(tmr_irqs[port]));
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
                cortexm_disable_irq((void*)(u32)(tmr_irqs[port]));
            }
            switch(port){
            case 0:
                __HAL_RCC_TIM1_CLK_DISABLE();
                break;
            case 1:
                __HAL_RCC_TIM2_CLK_DISABLE();
                break;
            case 2:
                __HAL_RCC_TIM3_CLK_DISABLE();
                break;
            case 3:
                __HAL_RCC_TIM4_CLK_DISABLE();
                break;
            case 4:
                __HAL_RCC_TIM5_CLK_DISABLE();
                break;
#if defined __HAL_RCC_TIM6_CLK_DISABLE
            case 5:
                __HAL_RCC_TIM6_CLK_DISABLE();
                break;
#endif
#if defined __HAL_RCC_TIM7_CLK_DISABLE
            case 6:
                __HAL_RCC_TIM7_CLK_DISABLE();
                break;
#endif
#if defined __HAL_RCC_TIM8_CLK_DISABLE
            case 7:
                __HAL_RCC_TIM8_CLK_DISABLE();
                break;
#endif
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
            if( (port == 1) || (port == 8) || (port == 9) || (port == 10) || (port == 11) ){
                pclk = HAL_RCC_GetPCLK1Freq(); //timer clocks are double pclks
                if( pclk <= mcu_board_config.core_cpu_freq/2 ){
                    pclk = pclk * 2;
                }
            } else {
                pclk = HAL_RCC_GetPCLK2Freq();
                if( pclk <= mcu_board_config.core_cpu_freq/2 ){
                    pclk = pclk * 2;
                }
            }

            if ( freq < pclk ){
                m_tmr_local[port].hal_handle.Init.Prescaler = ((pclk + freq/2) / freq);
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

            if( HAL_TIM_Base_Init(&m_tmr_local[port].hal_handle) != HAL_OK ){
                return -1;
            }

            if( o_flags & TMR_FLAG_IS_SOURCE_CPU ){
                //configure to use the internal clock
                TIM_ClockConfigTypeDef clock_source_config;
                clock_source_config.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
                if( HAL_TIM_ConfigClockSource(&m_tmr_local[port].hal_handle, &clock_source_config) != HAL_OK ){
                    return -1;
                }
            }
        }
    }

    if( o_flags & TMR_FLAG_SET_CHANNEL ){
        int chan = attr->channel.loc & ~MCU_CHANNEL_FLAG_IS_INPUT;
        int channel_configure_type;
        int ret;
        TIM_OC_InitTypeDef init_oc;
        TIM_IC_InitTypeDef init_ic;

        if( chan >= NUM_OCS ){
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


            TIM_MasterConfigTypeDef master_config;
            master_config.MasterOutputTrigger = TIM_TRGO_RESET;
            master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
            if (HAL_TIMEx_MasterConfigSynchronization(&m_tmr_local[port].hal_handle, &master_config) != HAL_OK){
                return -1;
            }

            ret = 0;

            init_oc.Pulse = 0;
            init_oc.OCPolarity = TIM_OCPOLARITY_HIGH;
            init_oc.OCFastMode = TIM_OCFAST_DISABLE;
        }

        switch(channel_configure_type){
        case CHANNEL_TYPE_OUTPUT_COMPARE:
            ret = HAL_TIM_OC_Init(&m_tmr_local[port].hal_handle);
            if( ret != HAL_OK ){ return -1; }
            ret = HAL_TIM_OC_ConfigChannel(&m_tmr_local[port].hal_handle, &init_oc, tim_channel);
            if( ret != HAL_OK ){ return -1; }
            ret = HAL_TIM_OC_Start(&m_tmr_local[port].hal_handle, tim_channel);
            break;

        case CHANNEL_TYPE_PWM:
            ret = HAL_TIM_PWM_Init(&m_tmr_local[port].hal_handle);
            if( ret != HAL_OK ){ return -1; }
            ret = HAL_TIM_PWM_ConfigChannel(&m_tmr_local[port].hal_handle, &init_oc, tim_channel);
            if( ret != HAL_OK ){ return -1; }
            ret = HAL_TIM_PWM_Start(&m_tmr_local[port].hal_handle, tim_channel);
            break;

        case CHANNEL_TYPE_INPUT_CAPTURE:
            ret = HAL_OK;
            ret = HAL_TIM_IC_Init(&m_tmr_local[port].hal_handle);
            if( ret != HAL_OK ){ return -1; }
            ret = HAL_TIM_IC_ConfigChannel(&m_tmr_local[port].hal_handle, &init_ic, tim_channel);
            if( ret != HAL_OK ){ return -1; }
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
            return -1;
        }

        //this sets the value of the channel
        if( mcu_tmr_setchannel(handle, (void*)&attr->channel) < 0 ){
            return -1;
        }
    }

    if( o_flags & (TMR_FLAG_SET_TIMER|TMR_FLAG_SET_CHANNEL) ){
        if( mcu_set_pin_assignment(
                    &(attr->pin_assignment),
                    MCU_CONFIG_PIN_ASSIGNMENT(tmr_config_t, handle),
                    MCU_PIN_ASSIGNMENT_COUNT(tmr_pin_assignment_t),
                    CORE_PERIPH_TMR, port, 0, 0, 0) < 0 ){
            return -1;
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
            return SYSFS_SET_RETURN(EINVAL);
        }
    } else {
        chan = req->loc;
        if ( chan >= NUM_OCS ){
            return SYSFS_SET_RETURN(EINVAL);
        }
    }

    ((u32*)&(regs->CCR1))[ chan ] = req->value;

    return 0;
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
        if ( chan >= NUM_OCS ){
            return SYSFS_SET_RETURN(EINVAL);
        }
    }

    req->value = ((u32*)&(regs->CCR1))[ chan ];
    return 0;
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

    if( chan >= NUM_OCS ){
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

    return 0;
}

int mcu_tmr_read(const devfs_handle_t * handle, devfs_async_t * rop){
    return SYSFS_SET_RETURN(ENOTSUP);
}

int mcu_tmr_set(const devfs_handle_t * handle, void * ctl){
    TIM_TypeDef * regs;
    int port = handle->port;
    regs = tmr_regs_table[port];
    regs->CNT = (u32)ctl;
    return 0;
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

void mcu_core_tim1_cc_isr(){
    tmr_isr(0);
}

void mcu_core_tim2_isr(){
    tmr_isr(1);
}

void mcu_core_tim3_isr(){
    tmr_isr(2);
}

void mcu_core_tim4_isr(){
    tmr_isr(3);
}

void mcu_core_tim5_isr(){
    tmr_isr(4);
}

#endif

