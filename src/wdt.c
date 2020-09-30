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

int mcu_wdt_init(int mode, int interval) {
  __HAL_RCC_WWDG_CLK_ENABLE();

  return 0;
}

int mcu_wdt_setinterval(int interval) { return 0; }

void mcu_wdt_root_reset(void *args) {
 
}

void mcu_wdt_reset() {

}

// ISR handler is in the cortex_m/fault.c source file
