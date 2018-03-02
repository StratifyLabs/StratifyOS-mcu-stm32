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

#include "stm32_flash.h"

int stm32_flash_write(u32 addr, const void * buf, int nbyte){
	u32 i;
	int err;
	const u32 * pbuf = buf;

	cortexm_disable_interrupts(NULL);
    HAL_FLASH_Unlock();
	for(i=0; i < nbyte; i+=4){
		err = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i, *pbuf++);
		if( err != HAL_OK ){
			break;
		}
	}
    HAL_FLASH_Lock();
	cortexm_enable_interrupts(NULL);

	if( err != HAL_OK ){
		err = -1;
	} else {
		err = 0;
	}
	return err;
}

int stm32_flash_erase_sector(u32 sector){
	FLASH_EraseInitTypeDef hal_erase_attr;
	int err;
	u32 erase_error;

	cortexm_disable_interrupts(NULL);
    HAL_FLASH_Unlock();

	hal_erase_attr.TypeErase = FLASH_TYPEERASE_SECTORS;
	hal_erase_attr.VoltageRange = FLASH_VOLTAGE_RANGE_3;  //! \todo This needs to be added to mcu_board_config
	hal_erase_attr.Sector = sector;
	hal_erase_attr.NbSectors = 1;
	err = HAL_FLASHEx_Erase(&hal_erase_attr, &erase_error);
    HAL_FLASH_Lock();
	cortexm_enable_interrupts(NULL);

	if ( err != HAL_OK ){
		return -1;
	}

	return 0;
}

int stm32_flash_blank_check(int loc, int nbyte){
	int i;
	const s8 * locp;
	//Do a blank check
	locp = (const int8_t*)loc;
	for(i = 0; i < nbyte; i++){
		if ( locp[i] != -1 ){
			return -1;
		}
	}
	return 0;
}

int stm32_flash_get_sector_size(u32 sector){
	if ( sector < 4 ){
		return 16*1024;
	} else if( sector == 4 ){
		return 64*1024;
    } else if( sector < 12 ){
		return 128*1024;
    } else {
        return stm32_flash_get_sector_size(sector - 12);
    }
}

int stm32_flash_get_sector_addr(u32 sector){
	u32 offset = 0;
	if ( sector <= 4 ){
        offset = sector*16;
	} else if( sector == 5 ){
        offset = 4*16 + 64;
    } else if( sector < 12 ){
        offset = 4*16 + 64 + ((sector - 5) * 128);
    } else {
        //12 to 23 are mapped the same
        return (stm32_flash_get_sector_addr(sector - 12) + 1024*1024);
    }
    return offset*1024 + FLASH_START;
}

int stm32_flash_is_flash(u32 addr, u32 size){
	if ( ((addr + size) <= (FLASH_SIZE + FLASH_START)) && (addr >= FLASH_START) ){
		return 1;
	}
	return 0;
}

int stm32_flash_is_code(u32 addr, u32 size){
	if( addr + size <= FLASH_CODE_START ){
		return 0;
	}

	if( addr >= FLASH_CODE_END ){
		return 0;
	}

	return 1;
}

//Get the flash page that contains the address
int stm32_flash_get_sector(u32 addr){
	u32 offset;
	u32 sector;

    //up to 12 sectors with 1MB
	//4 16KB pages
	//1 64KB page
    //7 128KB pages

    //up to 12 more sectors with 2MB part (make a recursive call)

    offset = (addr - FLASH_START) / 1024;
    if ( offset <= 4*16 ){
        sector = offset / (16);
    } else if( offset < 4*16 + 64 ){
		sector = 4;
	} else {
        if( offset < 1024 ){ //1MB
            sector = (offset - (4*16 + 64))/(128) + 5;
        } else {
            sector = stm32_flash_get_sector(addr - 1024*1024) + 12;
        }
	}

	return sector;

}





