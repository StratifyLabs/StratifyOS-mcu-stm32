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
	char * volatile rx_buf;
	char * volatile tx_buf;
	volatile int size;
	void * duplex_mem;
	int ret;
	u8 pin_assign;
	u8 width;
	u8 ref_count;
	mcu_event_handler_t handler;
} spi_local_t;

static spi_local_t spi_local[MCU_SPI_PORTS] MCU_SYS_MEM;

static SPI_TypeDef * const spi_regs[MCU_SPI_PORTS] = MCU_SPI_REGS;
static u8 const spi_irqs[MCU_SPI_PORTS] = MCU_SPI_IRQS;

static void exec_callback(int port, u32 o_events);

static int spi_port_transfer(const devfs_handle_t * handle, int is_read, devfs_async_t * dop);
static int byte_swap(int port, int byte);

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
		}
		cortexm_enable_irq((void*)(u32)(spi_irqs[port]));
		spi_local[port].duplex_mem = NULL;
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
	info->o_flags = 0;

	return 0;
}

int mcu_spi_setattr(const devfs_handle_t * handle, void * ctl){
	int port = handle->port;
	u32 tmp;
	u32 mode;
	u32 prescaler;

	const spi_attr_t * attr = mcu_select_attr(handle, ctl);
	if( attr == 0 ){
		return -1;
	}

	u32 o_flags = attr->o_flags;


	if( o_flags & SPI_FLAG_SET_MASTER ){
		spi_local[port].hal_handle.Init.Mode = SPI_MODE_MASTER;
		if( attr->freq == 0 ){
			errno = EINVAL;
			return -1 - offsetof(spi_attr_t, freq);
		}
		prescaler = 100; //need to calc based on frequency

	} else if( o_flags & SPI_FLAG_SET_SLAVE ){
		prescaler = 100;
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

	spi_local[port].hal_handle.Init.Direction = 0;
	spi_local[port].hal_handle.Init.DataSize = 0;

	spi_local[port].hal_handle.Init.NSS = SPI_NSS_SOFT;
	spi_local[port].hal_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	spi_local[port].hal_handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
	spi_local[port].hal_handle.Init.TIMode = SPI_TIMODE_DISABLE;
	spi_local[port].hal_handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	spi_local[port].hal_handle.Init.CRCPolynomial = 0;






	if( mcu_set_pin_assignment(
			&(attr->pin_assignment),
			MCU_CONFIG_PIN_ASSIGNMENT(spi_config_t, handle),
			MCU_PIN_ASSIGNMENT_COUNT(spi_pin_assignment_t),
			CORE_PERIPH_SPI, port, 0, 0) < 0 ){
		return -1;
	}


	HAL_SPI_Init(&spi_local[port].hal_handle);


	return 0;
}

int mcu_spi_swap(const devfs_handle_t * handle, void * ctl){
	int port = handle->port;
	return byte_swap(port, (int)ctl);
}

int mcu_spi_setduplex(const devfs_handle_t * handle, void * ctl){
	int port = handle->port;
	spi_local[port].duplex_mem = (void * volatile)ctl;
	return 0;
}

void exec_callback(int port, u32 o_events){


	mcu_execute_event_handler(&(spi_local[port].handler), o_events, 0);
}

int mcu_spi_setaction(const devfs_handle_t * handle, void * ctl){
	int port = handle->port;

	mcu_action_t * action = (mcu_action_t*)ctl;

	if ( (action->handler.callback == 0) && (action->o_events & (MCU_EVENT_FLAG_DATA_READY|MCU_EVENT_FLAG_WRITE_COMPLETE)) ){
		exec_callback(port, MCU_EVENT_FLAG_CANCELED);
	}

	spi_local[port].handler = action->handler;
	cortexm_set_irq_prio(spi_irqs[port], action->prio);
	return 0;
}


int byte_swap(int port, int byte){
	u8 tx_data;
	u8 rx_data;
	int ret;
	ret = HAL_SPI_TransmitReceive(&spi_local[port].hal_handle, &tx_data, &rx_data, 1, HAL_MAX_DELAY);
	if( ret != HAL_OK ){
		rx_data = 0xff;
	}
	return rx_data;
}

int mcu_spi_dev_write(const devfs_handle_t * handle, devfs_async_t * async){
	int ret;
	int port = handle->port;

	//if full duplex, call spi_port_transfer()

	ret = HAL_SPI_Transmit_IT(&spi_local[port].hal_handle, async->buf, async->nbyte);

	if( ret == HAL_OK ){
		return 0;
	}

	return -1;
}

int mcu_spi_dev_read(const devfs_handle_t * handle, devfs_async_t * async){
	int ret;
	int port = handle->port;

	//if full duplex, call spi_port_transfer()

	ret = HAL_SPI_Receive_IT(&spi_local[port].hal_handle, async->buf, async->nbyte);

	if( ret == HAL_OK ){
		return 0;
	}

	return -1;
}



int spi_port_transfer(const devfs_handle_t * handle, int is_read, devfs_async_t * async){
	int ret;
	int port = handle->port;

	HAL_SPI_TransmitReceive_IT(&spi_local[port].hal_handle, async->buf, async->buf, async->nbyte);


	return 0;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
	//execute the handler
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
	//execute the handler
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
	//execute the handler
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi){
	//execute handler (cancelled)
}

void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi){
	//execute handler (cancelled)
}

void mcu_core_spi0_isr(){
	HAL_SPI_IRQHandler(&spi_local[0].hal_handle);
}

void mcu_core_spi1_isr(){
	HAL_SPI_IRQHandler(&spi_local[1].hal_handle);
}

void mcu_core_spi2_isr(){
	HAL_SPI_IRQHandler(&spi_local[2].hal_handle);
}

void mcu_core_spi3_isr(){
	HAL_SPI_IRQHandler(&spi_local[3].hal_handle);
}

#endif

