// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md

#include "stm32_flash.h"

#if defined STM32_FLASH_LAYOUT_32_32_32_32_128_256_256_256
static const u16 stm32_flash_layout[]
  = {32, 32, 32, 32, 128, 256, 256, 256, 256, 256, 256, 256};
u16 stm32_flash_local_get_sector_size(u16 sector) {
  return stm32_flash_layout[sector];
}
#elif defined STM32_FLASH_LAYOUT_16_16_16_16_64_128_128_128
const u16 stm32_flash_layout[]
  = {16, 16, 16, 16, 64, 128, 128, 128, 128, 128, 128, 128,
     16, 16, 16, 16, 64, 128, 128, 128, 128, 128, 128, 128};
u16 stm32_flash_local_get_sector_size(u16 sector) {
  return stm32_flash_layout[sector];
}
#elif defined STM32_FLASH_LAYOUT_16_16_16_16_64_128x11
const u16 stm32_flash_layout[]
  = {16, 16, 16, 16, 64, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128};
u16 stm32_flash_local_get_sector_size(u16 sector) {
  return stm32_flash_layout[sector];
}
#elif defined STM32_FLASH_LAYOUT_2K_PAGES

u16 stm32_flash_local_get_sector_size(u16 sector) { return 2; }
#elif defined STM32_FLASH_LAYOUT_128K_PAGES
u16 stm32_flash_local_get_sector_size(u16 sector) {
  MCU_UNUSED_ARGUMENT(sector);
  return 128;
}
#endif

#if 0
#define STM32_FLASH_SECTIONS_16_16_16_16_64_128_128_128                        \
  {.o_flags = MEM_FLAG_IS_FLASH,                                               \
   .page_count = 4,                                                            \
   .address = 0x08000000,                                                      \
   .page_size = 16 * 1024UL},                                                  \
    {.o_flags = MEM_FLAG_IS_FLASH,                                             \
     .page_count = 1,                                                          \
     .address = 0x08000000 + 16 * 1024UL,                                      \
     .page_size = 64 * 1024UL},                                                \
  {                                                                            \
    .o_flags = MEM_FLAG_IS_FLASH, .page_count = 3,                             \
    .address = 0x08000000 + 16 * 1024UL + 64 * 1024, .page_size = 128 * 1024UL \
  }
#endif

int stm32_flash_write(u32 addr, const void *buf, int nbyte) {
  u32 i;
  int err;

  cortexm_disable_interrupts();
  if (HAL_FLASH_Unlock() != HAL_OK) {
    return -1;
  }

  if ((addr & 0x00200000) != 0) {
    addr += 0x07e00000; // map to 0x08000000 for writing
  }

#if defined STM32L4
  const u64 *pbuf = buf;
  u8 empty[8];
  memset(empty, 0xff, 8);

  for (i = 0; i < nbyte; i += 8) {
    if (memcmp(pbuf, empty, 8)) { // don't touch unless it is non 0xFF
      err = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + i, *pbuf);
    } else {
      err = HAL_OK;
    }
    pbuf++;
    if (err != HAL_OK) {
      break;
    }
  }
#elif defined STM32H7

  const u8 *pbuf = buf;
  u8 empty[32]; // flash word is 256-bit or 32 bytes
  memset(empty, 0xff, 8);

  for (i = 0; i < nbyte; i += 32) {
    if (memcmp(pbuf + i, empty, 32)) { // don't touch unless it is non 0xFF
      err = HAL_FLASH_Program(
        FLASH_TYPEPROGRAM_FLASHWORD,
        addr + i,
        (u32)(pbuf + i));
    } else {
      err = HAL_OK;
    }
    if (err != HAL_OK) {
      break;
    }
  }

#else
  const u32 *pbuf = buf;
  for (i = 0; i < nbyte; i += 4) {
    err = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i, *pbuf++);
    if (err != HAL_OK) {
      break;
    }
  }
#endif
  HAL_FLASH_Lock();
  cortexm_enable_interrupts();

  if (err != HAL_OK) {
    err = -1;
  } else {
    err = 0;
  }
  return err;
}

int stm32_flash_erase_sector(u32 sector) {
  FLASH_EraseInitTypeDef hal_erase_attr;
  int err;
  u32 erase_error;

#if defined FLASH_TYPEERASE_PAGES
  hal_erase_attr.TypeErase = FLASH_TYPEERASE_PAGES;
  hal_erase_attr.Banks = FLASH_BANK_1;
  hal_erase_attr.Page = sector;
  hal_erase_attr.NbPages = 1;
#else
  hal_erase_attr.TypeErase = FLASH_TYPEERASE_SECTORS;

#if defined STM32H7
  if (sector / 8 == 0) {
    hal_erase_attr.Banks = FLASH_BANK_1; // only used for mass erase?
  }
#if defined DUAL_BANK
  else {
    hal_erase_attr.Banks = FLASH_BANK_2; // only used for mass erase?
  }
#endif
  hal_erase_attr.Sector = sector % 8;
#else
#if defined FLASH_BANK_1
  hal_erase_attr.Banks = FLASH_BANK_1; // only used for mass erase?
#endif
  hal_erase_attr.Sector = sector;
#endif

  hal_erase_attr.NbSectors = 1;


  hal_erase_attr.VoltageRange = FLASH_VOLTAGE_RANGE_1;
  if (stm32_config.flash_program_millivolts > 5000) {
    hal_erase_attr.VoltageRange = FLASH_VOLTAGE_RANGE_4;
  } else if (stm32_config.flash_program_millivolts > 2700) {
    hal_erase_attr.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  } else if (stm32_config.flash_program_millivolts > 2100) {
    hal_erase_attr.VoltageRange = FLASH_VOLTAGE_RANGE_2;
  }

#endif

  cortexm_disable_interrupts();
  HAL_FLASH_Unlock();
  err = HAL_FLASHEx_Erase(&hal_erase_attr, &erase_error);
  HAL_FLASH_Lock();
  cortexm_enable_interrupts();

  if (err != HAL_OK) {
    return -1;
  }

  return 0;
}

int stm32_flash_blank_check(int loc, int nbyte) {
  int i;
  const s8 *locp;
  // Do a blank check
  locp = (const s8 *)loc;
  for (i = 0; i < nbyte; i++) {
    if (locp[i] != -1) {
      return -1;
    }
  }
  return 0;
}

int stm32_flash_get_sector_size(u32 sector) {
  if (sector < MCU_FLASH_PAGE_COUNT) {
    return stm32_flash_local_get_sector_size(sector) * 1024;
  }

  return 0;
}

int stm32_flash_get_sector_addr(u32 sector) {
  u32 offset = 0;
  u16 sum = 0;
  int i;

  if (sector < MCU_FLASH_PAGE_COUNT) {
    for (i = 0; i < sector; i++) {
      sum += stm32_flash_local_get_sector_size(i);
    }
    offset = sum * 1024 + MCU_FLASH_START;
    return offset;
  }

  return -1;
}

int stm32_flash_is_flash(u32 addr, u32 size) {
  if (
    ((addr + size) <= (MCU_FLASH_SIZE + MCU_FLASH_START))
    && (addr >= MCU_FLASH_START)) {
    return 1;
  }
  return 0;
}

int stm32_flash_is_code(u32 addr, u32 size) {
  if (addr + size <= MCU_FLASH_CODE_START) {
    return 0;
  }

  if (addr >= MCU_FLASH_CODE_END) {
    return 0;
  }

  return 1;
}

// Get the flash page that contains the address
int stm32_flash_get_sector(u32 addr) {
  u32 offset;
  u16 search = 0;
  int i;

  offset = (addr - MCU_FLASH_START) / 1024;
  for (i = 0; i < MCU_FLASH_PAGE_COUNT; i++) {
    if (offset < search + stm32_flash_local_get_sector_size(i)) {
      return i;
    }
    search += stm32_flash_local_get_sector_size(i);
  }

  return -1;
}
