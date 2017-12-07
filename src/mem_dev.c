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

#include <errno.h>
#include <cortexm/cortexm.h>
#include <mcu/mem.h>
#include <mcu/debug.h>
#include <mcu/core.h>
#include <mcu/wdt.h>
#include <mcu/bootloader.h>

#include <stm32_local.h>


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

#define FLASH_START ((u32)&_flash_start)
#define FLASH_OS_START ((int)&_text)
#define FLASH_OS_END ((int)&_etext + (int)&_edata - (int)&_data)
#define FLASH_SIZE ((u32)&_flash_size)

static int blank_check(int loc, int nbyte);
static int is_flash(int addr, int size);
static int get_flash_page(int addr);
static int get_flash_sector_size(int page);
static int get_flash_sector_addr(int page);

static int get_last_boot_page();

static int is_ram(int addr, int size);
static int get_ram_page(int addr);
static int get_ram_page_size(int page);
static int get_ram_page_addr(int page);

void mcu_mem_dev_power_on(const devfs_handle_t * handle){}
void mcu_mem_dev_power_off(const devfs_handle_t * handle){}
int mcu_mem_dev_is_powered(const devfs_handle_t * handle){
	return 1;
}

//this is used by appfs
int mcu_mem_getsyspage(){
	return (SRAM_PAGES);
}


int mcu_mem_getinfo(const devfs_handle_t * handle, void * ctl){
	mem_info_t * info = ctl;
	info->flash_pages = (u32)&_flash_pages;
	info->flash_size = FLASH_SIZE;
	info->ram_pages = (u32)&_flash_pages;
	info->ram_size = SRAM_SIZE;
	return 0;
}
int mcu_mem_setattr(const devfs_handle_t * handle, void * ctl){
	return 0;
}

int mcu_mem_setaction(const devfs_handle_t * handle, void * ctl){
	errno = ENOTSUP;
	return -1;
}

int mcu_mem_getpageinfo(const devfs_handle_t * handle, void * ctl){
	u32 size = 0;
	int32_t addr = 0;
	mem_pageinfo_t * ctlp = ctl;


	if( ctlp->o_flags & MEM_FLAG_IS_RAM ){

		size = get_ram_page_size(ctlp->num);
		addr = get_ram_page_addr(ctlp->num);
		if ( addr < 0 ){
			return -1;
		}

	} else if( ctlp->o_flags & MEM_FLAG_IS_FLASH ){

		size = get_flash_sector_size(ctlp->num);
		addr = get_flash_sector_addr(ctlp->num);
		if ( (addr + size) > (FLASH_SIZE + FLASH_START) ){
			return -1; //this page does not exist on this part
		}

	} else if( ctlp->o_flags & MEM_FLAG_IS_QUERY ){

		//Query needs to see if addr is RAM or FLASH
		if ( is_ram(ctlp->addr, DEVICE_RAM_PAGE_SIZE) ){
			ctlp->num = get_ram_page(ctlp->addr);
			ctlp->size = get_ram_page_size(ctlp->num);
			ctlp->o_flags = MEM_FLAG_IS_RAM;
			return 0;
		} else if ( is_flash(ctlp->addr, 0) ){
			ctlp->num = get_flash_page(ctlp->addr);
			ctlp->size = get_flash_sector_size(ctlp->num);
			ctlp->o_flags = MEM_FLAG_IS_FLASH;
			return 0;
		} else {
			return -1;
		}

	} else {
		return -1;
	}



	ctlp->size = size;
	ctlp->addr = addr;
	return 0;
}



int mcu_mem_erasepage(const devfs_handle_t * handle, void * ctl){
	int err;
	int addr;
	int page;
	FLASH_EraseInitTypeDef hal_erase_attr;
	u32 erase_error;
	page = (u32)ctl;
	addr = get_flash_sector_addr(page);  //this gets the beginning of the page
	int last_boot_page = get_last_boot_page();
	int page_size;

	page_size = get_flash_sector_size(page);

	//protect the OS and the bootloader from being erased
	if ( (page <= last_boot_page) ||
			((addr + page_size >= FLASH_OS_START) && (addr <= FLASH_OS_END)) ){
		mcu_debug_root_printf("Can't erase page %d\n", page);
		errno = EROFS;
		return -1;
	}


	cortexm_disable_interrupts(NULL);
	//err = mcu_lpc_flash_erase_page((u32)ctl);
    HAL_FLASH_Unlock();

	hal_erase_attr.TypeErase = FLASH_TYPEERASE_SECTORS;
	hal_erase_attr.VoltageRange = FLASH_VOLTAGE_RANGE_3;  //! \todo This needs to be added to mcu_board_config
	hal_erase_attr.Sector = page; //Specify sector number
	hal_erase_attr.NbSectors = 1; //This is also important!
	err = HAL_FLASHEx_Erase(&hal_erase_attr, &erase_error);
    HAL_FLASH_Lock();
	cortexm_enable_interrupts(NULL);

	if ( err != HAL_OK ){
		errno = EIO;
		return -1;
	}
	return 0;
}


int mcu_mem_writepage(const devfs_handle_t * handle, void * ctl){
	int err;
	int nbyte;
	mem_writepage_t * wattr = ctl;
	int last_boot_page;
	int i;
	int write_page;

	nbyte = wattr->nbyte;
	if( nbyte > 256 ){
		nbyte = 256;
	}

	if ( is_ram(wattr->addr, nbyte) ){
		memcpy((void*)wattr->addr, wattr->buf, nbyte);
		return nbyte;
	}

	last_boot_page = get_last_boot_page();
	write_page = get_flash_page(wattr->addr);

	if ( write_page <= last_boot_page ){
		mcu_debug_root_printf("Can't write to 0x%lX\n", wattr->addr);
		errno = EROFS;
		return -1;
	}

	if( ((wattr->addr + nbyte >= FLASH_OS_START) && (wattr->addr <= FLASH_OS_END)) ){
		errno = EROFS;
		return -1;
	}

	if ( wattr->addr >= (FLASH_SIZE + FLASH_START) ){
		errno = EINVAL;
		return -11;
	}

	if ( wattr->addr + nbyte > (FLASH_SIZE + FLASH_START) ){
		nbyte = FLASH_SIZE - wattr->addr; //update the bytes read to not go past the end of the disk
	}


	if ( blank_check(wattr->addr,  nbyte) ){
		mcu_debug_root_printf("not blank 0x%lX\n", wattr->addr);
		errno = EROFS;
		return -1;
	}

	cortexm_disable_interrupts(NULL);
    HAL_FLASH_Unlock();

	for(i=0; i < nbyte; i+=4){
		u32 data;
		data = *(u32*)(wattr->buf + i);
		err = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, wattr->addr + i, data);
		if( err != HAL_OK ){
			break;
		}
	}

    HAL_FLASH_Lock();
	cortexm_enable_interrupts(NULL);


	if( err != HAL_OK ){
		errno = EIO;
		return -1;
	}

	return nbyte;
}

int mcu_mem_dev_write(const devfs_handle_t * cfg, devfs_async_t * wop){

	if ( is_ram(wop->loc, wop->nbyte) ){
		memcpy((void*)wop->loc, wop->buf, wop->nbyte);
		return wop->nbyte;
	}

	errno = EINVAL;
	return -1;
}

int mcu_mem_dev_read(const devfs_handle_t * cfg, devfs_async_t * rop){
	if ( (is_flash(rop->loc, rop->nbyte) ) ||
			( is_ram(rop->loc, rop->nbyte) ) 	){
		memcpy(rop->buf, (const void*)rop->loc, rop->nbyte);
		return rop->nbyte;
	}
	errno = EINVAL;
	return -1;
}

//Get the flash page that contains the address
int get_flash_page(int addr){
	if ( addr < 0x10000 ){
		return addr / 4096;
	}
	addr -= 0x10000;
	return 16 + addr / 32768;
}

int get_flash_sector_size(int page){
	if ( page < 4 ){
		return 16*1024;
	} else if( page == 4 ){
		return 64*1024;
	} else {
		return 128*1024;
	}
}

int get_flash_sector_addr(int page){
	u32 offset = 0;
	if ( page <= 4 ){
		offset = page*16*1024;
	} else if( page == 5 ){
		offset = 4*16*1024 + 64*1024;
	} else {
		offset = 4*16*1024 + 64*1024 + ((page - 5) * 128*1024);
	}
	return offset + FLASH_START;
}

int is_flash(int addr, int size){
	if ( (addr + size) <= (FLASH_SIZE + FLASH_START) ){
		return 1;
	}
	return 0;
}

//RAM paging
int get_ram_page(int addr){
	int ret;
	if ( (addr >= SRAM_START) && (addr < SRAM_END) ){
		ret = ((addr - SRAM_START) / DEVICE_RAM_PAGE_SIZE);
	} else {
		ret = -1;
	}
	return ret;
}

int get_ram_page_size(int page){
	return DEVICE_RAM_PAGE_SIZE;
}

int get_ram_page_addr(int page){
	if ( page < SRAM_PAGES ){
		return SRAM_START + page*DEVICE_RAM_PAGE_SIZE;
	}

	return -1;
}

int is_ram(int addr, int size){
	if ( (addr >= SRAM_START) && ((addr + size) <= SRAM_END) ){
		return 1;
	}

	return 0;
}

int blank_check(int loc, int nbyte){
	int i;
	const s8 * locp;
	//Do a blank check
	locp = (const int8_t*)loc;
	for(i = 0; i < nbyte; i++){
		if ( locp[i] != -1 ){
			mcu_debug_root_printf("Not blank at offset %d\n", i);
			return -1;
		}
	}
	return 0;
}

int get_last_boot_page(){
	bootloader_api_t api;
	mcu_core_get_bootloader_api(&api);

	if( api.code_size > 0 ){ //zero means there is no bootloader installed
		return get_flash_page(api.code_size + FLASH_START);
	}

	return -1;

}

