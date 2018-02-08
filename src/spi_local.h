/* Copyright 2011-2017 Tyler Gilbert;
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

#ifndef SPI_LOCAL_H_
#define SPI_LOCAL_H_

#include "stm32_local.h"

typedef struct {
    union { //must be first
        SPI_HandleTypeDef hal_handle;
        I2S_HandleTypeDef i2s_hal_handle;
    };
    mcu_event_handler_t handler;
    int * nbyte_ptr;
    u8 is_full_duplex;
    u8 is_i2s;
    u8 ref_count;
} spi_local_t;

extern spi_local_t spi_local[MCU_SPI_PORTS] MCU_SYS_MEM;
extern SPI_TypeDef * const spi_regs[MCU_SPI_PORTS];
extern u8 const spi_irqs[MCU_SPI_PORTS];

void mcu_spi_dev_power_on(const devfs_handle_t * handle);
void mcu_spi_dev_power_off(const devfs_handle_t * handle);
int mcu_spi_dev_is_powered(const devfs_handle_t * handle);

#endif /* SPI_LOCAL_H_ */
