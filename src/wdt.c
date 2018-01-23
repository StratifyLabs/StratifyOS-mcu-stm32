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


#define WDEN (1<<0)
#define WDRESET (1<<1)
#define WDTOF (1<<2)
#define WDINT (1<<3)

#define WDLOCK (1<<31)

int mcu_wdt_init(int mode, int interval){



	return 0;
}

int mcu_wdt_setinterval(int interval){


	mcu_wdt_reset();

	return 0;
}

void mcu_wdt_root_reset(void * args){
	//LPC_WDT->FEED = 0xAA;
	//LPC_WDT->FEED = 0x55;
}

void mcu_wdt_reset(){
	//LPC_WDT->FEED = 0xAA;
	//LPC_WDT->FEED = 0x55;
}


//ISR handler is in the cortex_m/fault.c source file
