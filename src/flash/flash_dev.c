// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md

#include <mcu/flash.h>
#include <mcu/mcu.h>
#include <sos/debug.h>
#include <sos/dev/bootloader.h>
#include <string.h>

#include "stm32_flash.h"

static int get_last_boot_page() {
  if (MCU_FLASH_CODE_START == MCU_FLASH_START) {
    // this is the bootloader
    return stm32_flash_get_sector(MCU_FLASH_CODE_END);
  }

  const bootloader_api_t *api = cortexm_get_bootloader_api();

  if (api && api->code_size > 0) {
    return stm32_flash_get_sector(api->code_size);
  }

  // zero means there is no bootloader installed
  return -1;
}

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(
  flash,
  FLASH_VERSION,
  FLASH_IOC_IDENT_CHAR,
  I_MCU_TOTAL + I_FLASH_TOTAL,
  mcu_flash_eraseaddr,
  mcu_flash_erasepage,
  mcu_flash_getpage,
  mcu_flash_getsize,
  mcu_flash_getpageinfo,
  mcu_flash_writepage)

int mcu_flash_open(const devfs_handle_t *handle) {
  MCU_UNUSED_ARGUMENT(handle);
  return 0;
}
int mcu_flash_close(const devfs_handle_t *handle) {
  MCU_UNUSED_ARGUMENT(handle);
  return 0;
}

int mcu_flash_getinfo(const devfs_handle_t *handle, void *ctl) {
  MCU_UNUSED_ARGUMENT(handle);
  MCU_UNUSED_ARGUMENT(ctl);
  return 0;
}
int mcu_flash_setattr(const devfs_handle_t *handle, void *ctl) {
  MCU_UNUSED_ARGUMENT(handle);
  MCU_UNUSED_ARGUMENT(ctl);
  return 0;
}

int mcu_flash_setaction(const devfs_handle_t *handle, void *ctl) {
  MCU_UNUSED_ARGUMENT(handle);
  MCU_UNUSED_ARGUMENT(ctl);
  return 0;
}

int mcu_flash_getpageinfo(const devfs_handle_t *handle, void *ctl) {
  MCU_UNUSED_ARGUMENT(handle);
  u32 size;
  u32 addr;
  flash_pageinfo_t *ctlp = (flash_pageinfo_t *)ctl;
  int ret = 0;

  size = stm32_flash_get_sector_size(ctlp->page);
  addr = stm32_flash_get_sector_addr(ctlp->page);

  if (stm32_flash_is_flash(addr, size)) {
    ctlp->addr = addr;
    ctlp->size = size;
  } else {
    ret = SYSFS_SET_RETURN(EINVAL);
  }
  return ret;
}

int mcu_flash_eraseaddr(const devfs_handle_t *handle, void *ctl) {
  int page;
  page = stm32_flash_get_sector((u32)ctl);
  return mcu_flash_erasepage(handle, (void *)page);
}

int mcu_flash_erasepage(const devfs_handle_t *handle, void *ctl) {
  int err;
  int addr;
  int page_size;
  int page;

  MCU_UNUSED_ARGUMENT(handle);


  page = (int)ctl;
  int last_boot_page = get_last_boot_page();

  if (page <= last_boot_page) {
    // Never erase the bootloader
    return SYSFS_SET_RETURN(EROFS);
  }

  addr = stm32_flash_get_sector_addr(page);
  page_size = stm32_flash_get_sector_size(page);

  if (stm32_flash_is_flash(addr, page_size) == 0) {
    return SYSFS_SET_RETURN(EROFS);
  }

  if (stm32_flash_is_code(addr, page_size)) {
    return SYSFS_SET_RETURN(EROFS);
  }

  err = stm32_flash_erase_sector(page);
  if (err < 0) {
    err = SYSFS_SET_RETURN(EIO);
  }

  return err;
}

int mcu_flash_getpage(const devfs_handle_t *handle, void *ctl) {
  MCU_UNUSED_ARGUMENT(handle);
  return stm32_flash_get_sector((u32)ctl);
}

int mcu_flash_getsize(const devfs_handle_t *handle, void *ctl) {
  MCU_UNUSED_ARGUMENT(handle);
  MCU_UNUSED_ARGUMENT(ctl);
  return MCU_FLASH_SIZE;
}

int mcu_flash_writepage(const devfs_handle_t *handle, void *ctl) {
  MCU_UNUSED_ARGUMENT(handle);
  int err;
  int nbyte;
  flash_writepage_t *wattr = ctl;

  nbyte = wattr->nbyte;
  if (nbyte > FLASH_MAX_WRITE_SIZE) {
    nbyte = FLASH_MAX_WRITE_SIZE;
  }

  // is dest in flash?
  if (stm32_flash_is_flash(wattr->addr, nbyte) == 0) {
    // not if flash
    return SYSFS_SET_RETURN(EINVAL);
  }

  // will dest overwrite bootloader?
  if (((wattr->addr + nbyte >= MCU_FLASH_CODE_START)
       && (wattr->addr <= MCU_FLASH_CODE_END))) {
    return SYSFS_SET_RETURN(EROFS);
  }

  if (stm32_flash_blank_check(wattr->addr, nbyte)) {
    return SYSFS_SET_RETURN(EROFS);
  }

  err = stm32_flash_write(wattr->addr, wattr->buf, nbyte);

  if (err == 0) {
    err = nbyte;
  }
  return err;
}

int mcu_flash_write(const devfs_handle_t *handle, devfs_async_t *async) {
  MCU_UNUSED_ARGUMENT(handle);
  MCU_UNUSED_ARGUMENT(async);
  return SYSFS_SET_RETURN(ENOTSUP);
}

int mcu_flash_read(const devfs_handle_t *handle, devfs_async_t *async) {
  MCU_UNUSED_ARGUMENT(handle);
  int ret = 0;
  if (stm32_flash_is_flash(async->loc, async->nbyte)) {
    memcpy(async->buf, (const void *)async->loc, async->nbyte);
    ret = async->nbyte;
  } else {
    ret = -1;
  }

  return ret;
}
