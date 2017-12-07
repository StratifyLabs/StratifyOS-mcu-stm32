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

#ifndef CORE_STARTUP_H_
#define CORE_STARTUP_H_

#include <mcu/types.h>
#include <mcu/mcu.h>
#include <cortexm/cortexm.h>

void core_init();
int _main();

#define _DECLARE_ISR(name) void mcu_core_##name##_isr() MCU_ALIAS(mcu_core_default_isr)
#define _ISR(name) mcu_core_##name##_isr

#endif /* CORE_STARTUP_H_ */
