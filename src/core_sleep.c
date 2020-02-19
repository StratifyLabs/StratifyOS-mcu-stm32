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
#include <mcu/mcu.h>
#include <cortexm/cortexm.h>
#include <mcu/core.h>


extern void mcu_set_sleep_mode(int * level);

int mcu_core_user_sleep(core_sleep_t level){

	cortexm_svcall((cortexm_svcall_t)mcu_set_sleep_mode, &level);
	if ( level < 0 ){
		return level;
	}

	//Wait for an interrupt
	__WFI();
	return 0;
}

void mcu_core_prepare_deepsleep(int level){

}

void mcu_core_recover_deepsleep(int level){
	mcu_core_initclock(0);
}
