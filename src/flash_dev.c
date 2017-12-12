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

#include <string.h>
#include <errno.h>
#include <mcu/flash.h>
#include "stm32_flash.h"


static int get_last_bootloader_page(){
	return stm32_flash_get_sector(FLASH_CODE_END);
}

void flash_dev_power_on(const devfs_handle_t * handle){}
void flash_dev_power_off(const devfs_handle_t * handle){}
int flash_dev_is_powered(const devfs_handle_t * handle){
	return 1;
}

int mcu_flash_getinfo(const devfs_handle_t * handle, void * ctl){
	return 0;
}
int mcu_flash_setattr(const devfs_handle_t * handle, void * ctl){
	return 0;
}

int mcu_flash_setaction(const devfs_handle_t * handle, void * ctl){
	return 0;
}

int mcu_flash_getpageinfo(const devfs_handle_t * handle, void * ctl){
	u32 size;
	u32 addr;
	flash_pageinfo_t * ctlp = (flash_pageinfo_t *)ctl;
	int ret = 0;

	size = stm32_flash_get_sector_size(ctlp->page);
	addr = stm32_flash_get_sector_addr(ctlp->page);

	if( stm32_flash_is_flash(addr, size) ){
		ctlp->addr = addr;
		ctlp->size = size;
	} else {
		errno = EINVAL;
		ret = -1;
	}
	return ret;
}


int mcu_flash_eraseaddr(const devfs_handle_t * handle, void * ctl){
	int page;
	page = stm32_flash_get_sector((u32)ctl);
	return mcu_flash_erasepage(handle, (void*)page);
}


int mcu_flash_erasepage(const devfs_handle_t * handle, void * ctl){
	int err;
	int addr;
	int page_size;
	u32 page;
	page = (u32)ctl;
	int last_page = get_last_bootloader_page();

	if ( page < last_page ){
		//Never erase the bootloader
		errno = EROFS;
		return -1;
	}


	addr = stm32_flash_get_sector_addr(page);
	page_size = stm32_flash_get_sector_size(page);

	if( stm32_flash_is_flash(addr, page_size) == 0 ){
		errno = EINVAL;
		return -1;
	}

	//check to see if the page is already blank
	if ( stm32_flash_blank_check(addr, page_size) == 0 ){
		return 0;
	}

	err = stm32_flash_erase_sector(page);
	if( err < 0 ){
		errno = EIO;
	}
	return err;
}

int mcu_flash_getpage(const devfs_handle_t * handle, void * ctl){
	return stm32_flash_get_sector((u32)ctl);
}

int mcu_flash_getsize(const devfs_handle_t * handle, void * ctl){
	return FLASH_SIZE;
}

int mcu_flash_writepage(const devfs_handle_t * handle, void * ctl){
	int err;
	int nbyte;
	flash_writepage_t * wattr = ctl;

	nbyte = wattr->nbyte;
	if( nbyte > 256 ){
		nbyte = 256;
	}

	//is dest in flash?
	if( stm32_flash_is_flash(wattr->addr, nbyte) == 0 ){
		//not if flash
		errno = EINVAL;
		return -1;
	}

	//will dest overwrite bootloader?
	if( ((wattr->addr + nbyte >= FLASH_CODE_START) && (wattr->addr <= FLASH_CODE_END)) ){
		errno = EROFS;
		return -1;
	}


	if ( stm32_flash_blank_check(wattr->addr,  nbyte) ){
		errno = EROFS;
		return -1;
	}

	err = stm32_flash_write(wattr->addr, wattr->buf, nbyte);

	if( err == 0 ){
		err = nbyte;
	}
	return nbyte;

}

int mcu_flash_dev_read(const devfs_handle_t * cfg, devfs_async_t * async){
	int ret = 0;
	if( stm32_flash_is_flash(async->loc, async->nbyte) ){
		memcpy(async->buf, (const void*)async->loc, async->nbyte);
		ret = async->nbyte;
	} else {
		ret = -1;
	}

	return ret;
}
