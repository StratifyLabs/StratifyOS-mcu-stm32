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

#ifndef STM32_FLASH_H_
#define STM32_FLASH_H_

#include "stm32_local.h"

extern u32 _text;
extern u32 _etext;
extern u32 _data;
extern u32 _edata;

extern u32 _flash_start;
extern u32 _flash_pages;
extern u32 _flash_size;
extern u32 _sram_size;
extern u32 _sram_start;

#if !defined DEVICE_RAM_PAGE_SIZE
#define DEVICE_RAM_PAGE_SIZE 1024
#endif

#define SRAM_SIZE ((int)&_sram_size)
#define SRAM_START ((int)&_sram_start)
#define SRAM_END (SRAM_START + SRAM_SIZE)
#define SRAM_PAGES (SRAM_SIZE / DEVICE_RAM_PAGE_SIZE)

#define MCU_FLASH_START ((u32)&_flash_start)
#define MCU_FLASH_CODE_START ((u32)&_text)
#define MCU_FLASH_CODE_END ((u32)&_etext + (u32)&_edata - (u32)&_data)
#define MCU_FLASH_SIZE ((u32)&_flash_size)
#define MCU_FLASH_PAGE_COUNT ((u32)&_flash_pages)

int stm32_flash_get_sector_size(u32 sector);
int stm32_flash_get_sector_addr(u32 sector);
int stm32_flash_is_flash(u32 addr, u32 size);
int stm32_flash_is_code(u32 addr, u32 size);
int stm32_flash_get_sector(u32 addr);

int stm32_flash_blank_check(int loc, int nbyte);
int stm32_flash_erase_sector(u32 sector);
int stm32_flash_write(u32 addr, const void * buf, int nbyte);


#endif /* STM32_FLASH_H_ */
