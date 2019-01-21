/* Copyright 2011-2018 Tyler Gilbert;
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

#ifndef I2S_SPI_LOCAL_H_
#define I2S_SPI_LOCAL_H_


#include "spi_local.h"

int i2s_spi_local_open(const devfs_handle_t * handle);
int i2s_spi_local_close(const devfs_handle_t * handle);
int i2s_spi_local_setattr(const devfs_handle_t * handle, void * ctl);
int i2s_spi_local_mute(const devfs_handle_t * handle, void * ctl);
int i2s_spi_local_unmute(const devfs_handle_t * handle, void * ctl);

void i2s_spi_local_wait_for_errata_level(spi_local_t * local);


#endif /* SPI_LOCAL_H_ */
