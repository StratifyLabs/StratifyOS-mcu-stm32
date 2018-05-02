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
#include "mcu/adc.h"
#include "mcu/pio.h"
#include "mcu/debug.h"
#include "mcu/core.h"

#if MCU_ADC_PORTS > 100


typedef struct {
    ADC_HandleTypeDef hal_handle;
    devfs_transfer_handler_t transfer_handler;
    u8 ref_count;
} adc_local_t;

static adc_local_t adc_local[MCU_ADC_PORTS] MCU_SYS_MEM;
USART_TypeDef * const adc_regs_table[MCU_ADC_PORTS] = MCU_ADC_REGS;
u8 const adc_irqs[MCU_ADC_PORTS] = MCU_ADC_IRQS;


DEVFS_MCU_DRIVER_IOCTL_FUNCTION_MIN(adc, ADC_VERSION)

int mcu_adc_open(const devfs_handle_t * handle){
    int port = handle->port;
    if ( adc_local[port].ref_count == 0 ){

        adc_local[port].hal_handle.Instance = adc_regs_table[port];

        switch(port){
        case 0:
            __HAL_RCC_ADC1_CLK_ENABLE();
            break;
#if defined ADC2
        case 1:
            __HAL_RCC_ADC2_CLK_ENABLE();
            break;
#endif
#if defined ADC3
        case 2:
            __HAL_RCC_ADC3_CLK_ENABLE();
            break;
#endif
#if defined ADC4
        case 3:
            __HAL_RCC_ADC4_CLK_ENABLE();
            break;
#endif
        }
        //reset HAL UART
        cortexm_enable_irq(adc_irqs[port]);

    }
    adc_local[port].ref_count++;

    return 0;
}

int mcu_adc_close(const devfs_handle_t * handle){
    int port = handle->port;

    if ( adc_local[port].ref_count > 0 ){
        if ( adc_local[port].ref_count == 1 ){
            cortexm_disable_irq(adc_irqs[port]);
            switch(port){
            case 0:
                __HAL_RCC_ADC1_CLK_DISABLE();
                break;
#if defined ADC2
            case 1:
                __HAL_RCC_ADC2_CLK_DISABLE();
                break;
#endif
#if defined ADC3
            case 2:
                __HAL_RCC_ADC3_CLK_DISABLE();
                break;
#endif
#if defined ADC4
            case 3:
                __HAL_RCC_ADC4_CLK_DISABLE();
                break;
#endif
            }
            adc_local[port].hal_handle.Instance = 0;
        }
        adc_local[port].ref_count--;
    }
    return 0;
}


int mcu_adc_getinfo(const devfs_handle_t * handle, void * ctl){

    adc_info_t * info = ctl;

    info->o_flags = ADC_FLAG_IS_LEFT_JUSTIFIED;
    info->maximum = 0xffff; //max value
    info->freq = 1000000; //max frequency

    return 0;
}

int mcu_adc_setattr(const devfs_handle_t * handle, void * ctl){
    u32 o_flags;
    int port = handle->port;
    u32 freq;
    adc_attr_t * attr;

    adc_local_t * adc = adc_local + port;
    attr = mcu_select_attr(handle, ctl);
    if( attr == 0 ){
        return SYSFS_SET_RETURN(EINVAL);
    }

    o_flags = attr->o_flags;

    if( o_flags & UART_FLAG_SET_LINE_CODING ){
        freq = attr->freq;
        if( freq == 0 ){
            freq = 115200;
        }

        adc->hal_handle.Init.ClockPrescaler = 0; //fast as possible
        //ADC_RESOLUTION_12B
        //ADC_RESOLUTION_10B
        //ADC_RESOLUTION_8B
        //ADC_RESOLUTION_6B
        adc->hal_handle.Resolution = 0;

        //ADC_DATAALIGN_RIGHT
        //ADC_DATAALIGN_LEFT
        adc->hal_handle.DataAlign = 0;


        adc->hal_handle.ScanConvMode = 0;

        //ADC_EOC_SEQ_CONV
        //ADC_EOC_SINGLE_CONV
        //ADC_EOC_SINGLE_SEQ_CONV
        adc->hal_handle.EOCSelection = 0;

        //ENABLE or DISABLE
        adc->hal_handle.ContinuousConvMode = 0;
        adc->hal_handle.NbrOfConversion = 0;
        //ENABLE or DISABLE
        adc->hal_handle.DiscontinuousConvMode = 0;
        adc->hal_handle.NbrOfDiscConversion = 0;

        //ADC_SOFTWARE_START
        //ADC_EXTERNALTRIGCONV_T1_CC1
        //ADC_EXTERNALTRIGCONV_T1_CC2
        //ADC_EXTERNALTRIGCONV_T1_CC3
        //ADC_EXTERNALTRIGCONV_T2_CC2
        //ADC_EXTERNALTRIGCONV_T2_CC3
        //ADC_EXTERNALTRIGCONV_T2_CC4
        //ADC_EXTERNALTRIGCONV_T2_TRGO
        //ADC_EXTERNALTRIGCONV_T3_CC1
        //ADC_EXTERNALTRIGCONV_T3_TRGO
        //ADC_EXTERNALTRIGCONV_T4_CC4
        //ADC_EXTERNALTRIGCONV_T5_CC1
        //ADC_EXTERNALTRIGCONV_T5_CC2
        //ADC_EXTERNALTRIGCONV_T5_CC3
        //ADC_EXTERNALTRIGCONV_T8_CC1
        //ADC_EXTERNALTRIGCONV_T8_TRGO
        //ADC_EXTERNALTRIGCONV_Ext_IT11
        adc->hal_handle.ExternalTrigConv = 0;

        //ADC_EXTERNALTRIGCONVEDGE_NONE
        //ADC_EXTERNALTRIGCONVEDGE_RISING
        //ADC_EXTERNALTRIGCONVEDGE_FALLING
        //ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING
        adc->hal_handle.ExternalTrigConvEdge = 0;

        //ENABLE or DISABLE (if ENABLE DMA must be in circular buffer mode)
        //this driver does not support DMA
        adc->hal_handle.DMAContinuousRequests = DISABLE;

        //pin assignments
        if( mcu_set_pin_assignment(
                    &(attr->pin_assignment),
                    MCU_CONFIG_PIN_ASSIGNMENT(uart_config_t, handle),
                    MCU_PIN_ASSIGNMENT_COUNT(uart_pin_assignment_t),
                    CORE_PERIPH_UART, port, 0, 0, 0) < 0 ){
            return SYSFS_SET_RETURN(EINVAL);
        }

        if( HAL_ADC_Init(&adc->hal_handle) != HAL_OK ){
            return SYSFS_SET_RETURN(EIO);
        }
    }

    return 0;
}


int mcu_adc_setaction(const devfs_handle_t * handle, void * ctl){
    mcu_action_t * action = (mcu_action_t*)ctl;
    int port = handle->port;
    adc_local_t * uart = adc_local + port;

    if( action->handler.callback == 0 ){
        //if there is an ongoing operation -- cancel it

        if( action->o_events & MCU_EVENT_FLAG_DATA_READY ){
            //execute the read callback if not null
            if( uart->o_flags & UART_LOCAL_FLAG_IS_INCOMING_ENABLED ){
                uart->o_flags &= ~UART_LOCAL_FLAG_IS_INCOMING_ENABLED;
#if defined STM32F4
                HAL_UART_AbortReceive_IT(&uart->hal_handle);
#endif
            }

            exec_readcallback(uart, MCU_EVENT_FLAG_CANCELED);
            uart->read.callback = 0;
        }

        if( action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE ){
#if defined STM32F4
            HAL_UART_AbortTransmit_IT(&uart->hal_handle);
#endif
            exec_writecallback(uart, MCU_EVENT_FLAG_CANCELED);
            uart->write.callback = 0;
        }

    } else {

        if( cortexm_validate_callback(action->handler.callback) < 0 ){
            return SYSFS_SET_RETURN(EPERM);
        }

        if( action->o_events & MCU_EVENT_FLAG_DATA_READY ){
            uart->read = action->handler;

            //enable the receiver so that the action is called when a byte arrives
            uart->o_flags |= UART_LOCAL_FLAG_IS_INCOMING_ENABLED;
            uart->o_flags &= ~UART_LOCAL_FLAG_IS_INCOMING_AVAILABLE;
            if( HAL_UART_Receive_IT(&uart->hal_handle, &uart->incoming, 1) != HAL_OK ){
                return SYSFS_SET_RETURN(EIO);
            }
        }

        if ( action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE ){
            uart->write = action->handler;
        }
    }

    cortexm_set_irq_priority(adc_irqs[port], action->prio);
    return 0;
}

int mcu_adc_read(const devfs_handle_t * handle, devfs_async_t * async){
    int port = handle->port;
    adc_local_t * adc = adc_local + port;

    DEVFS_DRIVER_IS_BUSY(adc->transfer_handler.read, async);



    if ( async->flags & O_NONBLOCK ){
        return SYSFS_SET_RETURN(EAGAIN);
    } else {
        //no bytes
        if( cortexm_validate_callback(async->handler.callback) < 0 ){
            return SYSFS_SET_RETURN(EPERM);
        }

        uart->read = async->handler;
        if( HAL_UART_Receive_IT(&uart->hal_handle, async->buf, async->nbyte) == HAL_OK ){
            return 0;
        }
    }

    //this needs to read 1 byte at a time

    uart->read.callback = 0;
    return SYSFS_SET_RETURN(EIO);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    adc_local_t * uart = (adc_local_t*)huart;
    uart_event_t uart_event;

    if( uart->o_flags & UART_LOCAL_FLAG_IS_INCOMING_ENABLED ){
        uart->o_flags |= UART_LOCAL_FLAG_IS_INCOMING_AVAILABLE;
        uart->incoming_received = uart->incoming;
        uart_event.value = uart->incoming_received;
        HAL_UART_Receive_IT(&uart->hal_handle, &uart->incoming, 1);
    }

    mcu_execute_event_handler(&uart->read, MCU_EVENT_FLAG_DATA_READY, &uart_event);
}

int mcu_adc_write(const devfs_handle_t * handle, devfs_async_t * async){
    int ret;
    int port = handle->port;
    adc_local_t * uart = adc_local + port;


    if( uart->write.callback ){
        return SYSFS_SET_RETURN(EBUSY);
    }

    uart->write = async->handler;
    ret = HAL_UART_Transmit_IT(&uart->hal_handle, async->buf, async->nbyte);
    if( ret == HAL_OK ){
        return 0;
    }

    return SYSFS_SET_RETURN(EIO);
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    adc_local_t * uart = (adc_local_t*)huart;
    uart_event_t uart_event;
    uart_event.value = 0;
    mcu_execute_event_handler(&uart->write, MCU_EVENT_FLAG_WRITE_COMPLETE, &uart_event);
}


void mcu_adc_isr(int port){
    adc_local_t * uart = adc_local + port;
    HAL_UART_IRQHandler(&uart->hal_handle);
}

void mcu_core_usart1_isr(){
    mcu_adc_isr(0);
}

#if MCU_UART_PORTS > 1
void mcu_core_usart2_isr(){
    mcu_adc_isr(1);
}
#endif

#if MCU_UART_PORTS > 2
void mcu_core_usart3_isr(){
    mcu_adc_isr(2);
}
#endif

#if MCU_UART_PORTS > 3
void mcu_core_uart4_isr(){
    mcu_adc_isr(3);
}
#endif

#if MCU_UART_PORTS > 4
void mcu_core_uart5_isr(){
    mcu_adc_isr(4);
}
#endif

#if MCU_UART_PORTS > 5
void mcu_core_usart6_isr(){
    mcu_adc_isr(5);
}
#endif

#if MCU_UART_PORTS > 6
void mcu_core_uart7_isr(){
    mcu_adc_isr(6);
}
#endif


#if MCU_UART_PORTS > 7
void mcu_core_uart8_isr(){
    mcu_adc_isr(7);
}
#endif

#endif
