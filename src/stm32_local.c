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

#include "stm32_local.h"


static const u8 m_flash_latency[] = {
	#if defined FLASH_LATENCY_0
	FLASH_LATENCY_0,
	#endif
	#if defined FLASH_LATENCY_1
	FLASH_LATENCY_1,
	#endif
	#if defined FLASH_LATENCY_2
	FLASH_LATENCY_2,
	#endif
	#if defined FLASH_LATENCY_3
	FLASH_LATENCY_3,
	#endif
	#if defined FLASH_LATENCY_4
	FLASH_LATENCY_4,
	#endif
	#if defined FLASH_LATENCY_5
	FLASH_LATENCY_5,
	#endif
	#if defined FLASH_LATENCY_6
	FLASH_LATENCY_6,
	#endif
	#if defined FLASH_LATENCY_7
	FLASH_LATENCY_7,
	#endif
	#if defined FLASH_LATENCY_8
	FLASH_LATENCY_8,
	#endif
	#if defined FLASH_LATENCY_9
	FLASH_LATENCY_9,
	#endif
	#if defined FLASH_LATENCY_10
	FLASH_LATENCY_10,
	#endif
	#if defined FLASH_LATENCY_11
	FLASH_LATENCY_11,
	#endif
	#if defined FLASH_LATENCY_12
	FLASH_LATENCY_12,
	#endif
	#if defined FLASH_LATENCY_13
	FLASH_LATENCY_13,
	#endif
	#if defined FLASH_LATENCY_14
	FLASH_LATENCY_14,
	#endif
	#if defined FLASH_LATENCY_15
	FLASH_LATENCY_15,
	#endif
};

static const u16 m_voltage_scale[] = {
	#if defined PWR_REGULATOR_VOLTAGE_SCALE1
	PWR_REGULATOR_VOLTAGE_SCALE1,
	#endif
	#if defined PWR_REGULATOR_VOLTAGE_SCALE2
	PWR_REGULATOR_VOLTAGE_SCALE2,
	#endif
	#if defined PWR_REGULATOR_VOLTAGE_SCALE3
	PWR_REGULATOR_VOLTAGE_SCALE3,
	#endif
};

static const u16 m_ahb_sysclk_divider[] = {
	RCC_SYSCLK_DIV1,
	RCC_SYSCLK_DIV2,
	RCC_SYSCLK_DIV4,
	RCC_SYSCLK_DIV8,
	RCC_SYSCLK_DIV16,
	RCC_SYSCLK_DIV64,
	RCC_SYSCLK_DIV64,
	RCC_SYSCLK_DIV128,
	RCC_SYSCLK_DIV256,
	RCC_SYSCLK_DIV512
};

static const u16 m_ahb_hclk_divider[] = {
	RCC_HCLK_DIV1,
	RCC_HCLK_DIV2,
	RCC_HCLK_DIV4,
	RCC_HCLK_DIV8,
	RCC_HCLK_DIV16
};

u32 stm32_local_decode_flash_latency(u16 value){
	return m_flash_latency[value];
}

u32 stm32_local_decode_voltage_scale(u16 value){
	return m_voltage_scale[value];
}

u32 stm32_local_decode_sysclk_divider(u16 value){
	u32 i;
	for(i=0; i < 9; i++){
		if( (value & 1<<i) != 0 ){
			break;
		}
	}
	return m_ahb_sysclk_divider[i];
}

u32 stm32_local_decode_hclk_divider(u16 value){
	u32 i;
	if( value == 0 ){
		return m_ahb_hclk_divider[0];
	}

	for(i=0; i < 4; i++){
		if( (value & 1<<i) != 0 ){
			break;
		}
	}
	return m_ahb_hclk_divider[i];
}





