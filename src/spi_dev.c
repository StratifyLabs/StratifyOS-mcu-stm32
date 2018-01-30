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

#include "stm32_local.h"
#include <mcu/spi.h>

#if MCU_SPI_PORTS > 0

typedef struct {
    SPI_HandleTypeDef hal_handle; //must be first
    mcu_event_handler_t handler;
    int * nbyte_ptr;
    u8 is_full_duplex;
    u8 ref_count;
} spi_local_t;

static spi_local_t spi_local[MCU_SPI_PORTS] MCU_SYS_MEM;

static SPI_TypeDef * const spi_regs[MCU_SPI_PORTS] = MCU_SPI_REGS;
static u8 const spi_irqs[MCU_SPI_PORTS] = MCU_SPI_IRQS;

static int execute_handler(mcu_event_handler_t * handler, u32 o_events, u32 value);

void mcu_spi_dev_power_on(const devfs_handle_t * handle){
    int port = handle->port;
    if ( spi_local[port].ref_count == 0 ){

        //turn on RCC clock
        switch(port){
        case 0:
            __HAL_RCC_SPI1_CLK_ENABLE();
            break;
#if MCU_SPI_PORTS > 1
        case 1:
            __HAL_RCC_SPI2_CLK_ENABLE();
            break;
#endif
#if MCU_SPI_PORTS > 2
        case 2:
            __HAL_RCC_SPI3_CLK_ENABLE();
            break;
#endif
#if MCU_SPI_PORTS > 3
        case 3:
            __HAL_RCC_SPI4_CLK_ENABLE();
            break;
#endif
#if MCU_SPI_PORTS > 4
        case 4:
            __HAL_RCC_SPI5_CLK_ENABLE();
            break;
#endif
#if MCU_SPI_PORTS > 5
        case 5:
            __HAL_RCC_SPI6_CLK_ENABLE();
            break;
#endif
        }
        spi_local[port].hal_handle.Instance = spi_regs[port];
        cortexm_enable_irq((void*)(u32)(spi_irqs[port]));
        spi_local[port].handler.callback = NULL;
    }
    spi_local[port].ref_count++;

}

void mcu_spi_dev_power_off(const devfs_handle_t * handle){
    int port = handle->port;
    if ( spi_local[port].ref_count > 0 ){
        if ( spi_local[port].ref_count == 1 ){
            cortexm_disable_irq((void*)(u32)(spi_irqs[port]));

            //turn off RCC clock
            switch(port){
            case 0:
                __HAL_RCC_SPI1_CLK_DISABLE();
                break;
#if MCU_SPI_PORTS > 1
            case 1:
                __HAL_RCC_SPI2_CLK_DISABLE();
                break;
#endif
#if MCU_SPI_PORTS > 2
            case 2:
                __HAL_RCC_SPI3_CLK_DISABLE();
                break;
#endif
#if MCU_SPI_PORTS > 3
            case 3:
                __HAL_RCC_SPI4_CLK_DISABLE();
                break;
#endif
#if MCU_SPI_PORTS > 4
            case 4:
                __HAL_RCC_SPI5_CLK_DISABLE();
                break;
#endif
#if MCU_SPI_PORTS > 5
            case 5:
                __HAL_RCC_SPI6_CLK_DISABLE();
                break;
#endif
            }

        }
        spi_local[port].ref_count--;
    }
}

int mcu_spi_dev_is_powered(const devfs_handle_t * handle){
    int port = handle->port;
    return ( spi_local[port].ref_count != 0 );
}


int mcu_spi_getinfo(const devfs_handle_t * handle, void * ctl){
    spi_info_t * info = ctl;

    //set flags
    info->o_flags = SPI_FLAG_SET_MASTER |
            SPI_FLAG_IS_MODE0 |
            SPI_FLAG_IS_MODE1 |
            SPI_FLAG_IS_MODE2 |
            SPI_FLAG_IS_MODE3 |
            SPI_FLAG_IS_FORMAT_SPI |
            SPI_FLAG_IS_FORMAT_TI |
            SPI_FLAG_IS_FULL_DUPLEX |
            SPI_FLAG_IS_HALF_DUPLEX;

    return 0;
}

int mcu_spi_setattr(const devfs_handle_t * handle, void * ctl){
    int port = handle->port;
    u32 pclk;
    u32 prescalar;

    const spi_attr_t * attr = mcu_select_attr(handle, ctl);
    if( attr == 0 ){
        return -1;
    }

    u32 o_flags = attr->o_flags;

    if( o_flags & SPI_FLAG_SET_MASTER ){
        spi_local[port].hal_handle.Init.Mode = SPI_MODE_MASTER;

        if( attr->freq == 0 ){
            spi_local[port].hal_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
        } else {

//#if defined __stm32f401xc
            if( port == 0 || port == 3 ){
                pclk = HAL_RCC_GetPCLK1Freq(); //42MHz max
            } else {
                pclk = HAL_RCC_GetPCLK2Freq(); //84MHz max
            }
//#endif

            prescalar = pclk / attr->freq;

            if( prescalar < 2 ){
                spi_local[port].hal_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
            } else if( prescalar < 4 ){
                spi_local[port].hal_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
            } else if( prescalar < 8 ){
                spi_local[port].hal_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
            } else if( prescalar < 16 ){
                spi_local[port].hal_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
            } else if( prescalar < 32 ){
                spi_local[port].hal_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
            }else if( prescalar < 64 ){
                spi_local[port].hal_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
            }else if( prescalar < 128 ){
                spi_local[port].hal_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
            }else {
                spi_local[port].hal_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
            }
        }

    } else if( o_flags & SPI_FLAG_SET_SLAVE ){
        spi_local[port].hal_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
        spi_local[port].hal_handle.Init.Mode = SPI_MODE_SLAVE;
    }


    if( attr->width > 8 ){
        errno = EINVAL;
        return -1 - offsetof(spi_attr_t, width);
    }

    spi_local[port].hal_handle.Init.CLKPolarity = SPI_POLARITY_LOW;
    spi_local[port].hal_handle.Init.CLKPhase = SPI_PHASE_1EDGE;
    if( o_flags & SPI_FLAG_IS_MODE1 ){
        spi_local[port].hal_handle.Init.CLKPolarity = SPI_POLARITY_LOW;
        spi_local[port].hal_handle.Init.CLKPhase = SPI_PHASE_2EDGE;
    } else if( o_flags & SPI_FLAG_IS_MODE2 ){
        spi_local[port].hal_handle.Init.CLKPolarity = SPI_POLARITY_HIGH;
        spi_local[port].hal_handle.Init.CLKPhase = SPI_PHASE_1EDGE;
    } else if( o_flags & SPI_FLAG_IS_MODE3 ){
        spi_local[port].hal_handle.Init.CLKPolarity = SPI_POLARITY_HIGH;
        spi_local[port].hal_handle.Init.CLKPhase = SPI_PHASE_2EDGE;
    }

    if( attr->width == 8 ){
        spi_local[port].hal_handle.Init.DataSize = SPI_DATASIZE_8BIT;
    } else if( attr->width == 16 ){
        spi_local[port].hal_handle.Init.DataSize = SPI_DATASIZE_16BIT;
    }else {
        errno = EINVAL;
        return -1 - offsetof(spi_attr_t, width);
    }

    spi_local[port].hal_handle.Init.Direction = SPI_DIRECTION_2LINES;

    spi_local[port].hal_handle.Init.NSS = SPI_NSS_SOFT;
    spi_local[port].hal_handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    if( o_flags & SPI_FLAG_IS_FORMAT_TI ){
        spi_local[port].hal_handle.Init.TIMode = SPI_TIMODE_ENABLE;
    } else {
        spi_local[port].hal_handle.Init.TIMode = SPI_TIMODE_DISABLE;
    }
    spi_local[port].hal_handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spi_local[port].hal_handle.Init.CRCPolynomial = 10;

    if( o_flags & SPI_FLAG_IS_FULL_DUPLEX ){
        spi_local[port].is_full_duplex = 1;
    } else {
        spi_local[port].is_full_duplex = 0;
    }


    if( mcu_set_pin_assignment(
                &(attr->pin_assignment),
                MCU_CONFIG_PIN_ASSIGNMENT(spi_config_t, handle),
                MCU_PIN_ASSIGNMENT_COUNT(spi_pin_assignment_t),
                CORE_PERIPH_SPI, port, 0, 0) < 0 ){
        return -1;
    }

    if( HAL_SPI_Init(&spi_local[port].hal_handle) != HAL_OK ){
        return -1;
    }

    return 0;
}

int mcu_spi_swap(const devfs_handle_t * handle, void * ctl){
    int port = handle->port;
    u8 tx_data;
    u8 rx_data;
    int ret;
    tx_data = (u32)ctl;
    ret = HAL_SPI_TransmitReceive(&spi_local[port].hal_handle, &tx_data, &rx_data, 1, HAL_MAX_DELAY);
    if( ret != HAL_OK ){
        rx_data = 0xff;
    }
    return rx_data;
}

int mcu_spi_setduplex(const devfs_handle_t * handle, void * ctl){
    return -1;
}

int execute_handler(mcu_event_handler_t * handler, u32 o_events, u32 value){
    spi_event_data_t event;
    event.value = value;
    return mcu_execute_event_handler(handler, o_events, &event);
}


int mcu_spi_setaction(const devfs_handle_t * handle, void * ctl){
    int port = handle->port;

    mcu_action_t * action = (mcu_action_t*)ctl;

    if ( (action->handler.callback == 0) && (action->o_events & (MCU_EVENT_FLAG_DATA_READY|MCU_EVENT_FLAG_WRITE_COMPLETE)) ){
        execute_handler(&spi_local[port].handler, MCU_EVENT_FLAG_CANCELED, 0);
    }

    spi_local[port].handler = action->handler;
    cortexm_set_irq_prio(spi_irqs[port], action->prio);
    return 0;
}

int mcu_spi_dev_write(const devfs_handle_t * handle, devfs_async_t * async){
    int ret;
    int port = handle->port;

    //check to see if SPI bus is busy -- check to see if the interrupt is enabled?

    if( async->nbyte == 0 ){
        return 0;
    }

    spi_local[port].nbyte_ptr = &(async->nbyte);
    spi_local[port].handler = async->handler;

    if( spi_local[port].is_full_duplex ){
        ret = HAL_SPI_TransmitReceive_IT(&spi_local[port].hal_handle, async->buf, async->buf, async->nbyte);
    } else {
        ret = HAL_SPI_Transmit_IT(&spi_local[port].hal_handle, async->buf, async->nbyte);
    }

    if( ret != HAL_OK ){
        return -1;
    }

    return 0;
}

int mcu_spi_dev_read(const devfs_handle_t * handle, devfs_async_t * async){
    int ret;
    int port = handle->port;

    if( async->nbyte == 0 ){
        return 0;
    }

    spi_local[port].nbyte_ptr = &(async->nbyte);
    spi_local[port].handler = async->handler;

    if( spi_local[port].is_full_duplex ){
        ret = HAL_SPI_TransmitReceive_IT(&spi_local[port].hal_handle, async->buf, async->buf, async->nbyte);
    } else {
        ret = HAL_SPI_Receive_IT(&spi_local[port].hal_handle, async->buf, async->nbyte);
    }

    if( ret != HAL_OK ){
        return -1;
    }

    return 0;
}


void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
    //execute the handler
    spi_local_t * spi = (spi_local_t*)hspi;
    *spi->nbyte_ptr = hspi->TxXferCount;
    execute_handler(&spi->handler, MCU_EVENT_FLAG_WRITE_COMPLETE, hspi->TxXferCount);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
    //execute the handler
    spi_local_t * spi = (spi_local_t*)hspi;
    *spi->nbyte_ptr = hspi->RxXferCount;
    execute_handler(&spi->handler, MCU_EVENT_FLAG_DATA_READY, hspi->RxXferCount);

}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
    //execute the handler
    spi_local_t * spi = (spi_local_t*)hspi;
    *spi->nbyte_ptr = hspi->TxXferCount;
    execute_handler(&spi->handler, MCU_EVENT_FLAG_DATA_READY|MCU_EVENT_FLAG_WRITE_COMPLETE, hspi->TxXferCount);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi){
    //execute handler (cancelled)
    spi_local_t * spi = (spi_local_t*)hspi;
    execute_handler(&spi->handler, MCU_EVENT_FLAG_CANCELED | MCU_EVENT_FLAG_ERROR, 0);
}

void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi){
    //execute handler (cancelled)
    spi_local_t * spi = (spi_local_t*)hspi;
    execute_handler(&spi->handler, MCU_EVENT_FLAG_CANCELED, 0);
}

void mcu_core_spi1_isr(){
    HAL_SPI_IRQHandler(&spi_local[0].hal_handle);
}

void mcu_core_spi2_isr(){
    HAL_SPI_IRQHandler(&spi_local[1].hal_handle);
}

void mcu_core_spi3_isr(){
    HAL_SPI_IRQHandler(&spi_local[2].hal_handle);
}

void mcu_core_spi4_isr(){
    HAL_SPI_IRQHandler(&spi_local[3].hal_handle);
}

#if MCU_SPI_PORTS > 4
void mcu_core_spi5_isr(){
    HAL_SPI_IRQHandler(&spi_local[4].hal_handle);
}
#endif

#if MCU_SPI_PORTS > 5
void mcu_core_spi6_isr(){
    HAL_SPI_IRQHandler(&spi_local[5].hal_handle);
}
#endif

#endif

