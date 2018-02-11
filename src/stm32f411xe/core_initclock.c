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


//requires mcu_core_osc_freq, mcu_board_config.core_cpu_freq, and mcu_board_config.core_periph_freq to be defined ext
int mcu_core_initclock(int div){
	//u8 clk_src = 0;
	//u32 fosc = mcu_board_config.core_osc_freq;
	//int pdiv = mcu_board_config.core_cpu_freq / mcu_board_config.core_periph_freq;
	//u32 fclk = mcu_board_config.core_cpu_freq / div;

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
    //RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

	__HAL_RCC_PWR_CLK_ENABLE();
    //SCALE3 is good up to 120MHz (this chip is 100MHz)
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);


	//0.95MHz < osc / PLLM < 2.1MHz
	//100MHz < osc / PLLM * PLLN < 400MHz
	// osc / PLLM * PLLN / PLLP < 180MHz

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
		return -1;
	}


    RCC_ClkInitStruct.ClockType =
            RCC_CLOCKTYPE_HCLK |
            RCC_CLOCKTYPE_SYSCLK |
            RCC_CLOCKTYPE_PCLK1 |
            RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    //APB1 is max 42MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    //APB2 is Max 84MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK){
		return -1;
	}

#if 0
	//PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
	//PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK){
		return -1;
	}
#endif

	SystemCoreClock = mcu_board_config.core_cpu_freq;

	return 0;
}

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority){
	return HAL_OK;
}

void HAL_Delay(__IO uint32_t Delay){
	u32 value = SysTick->VAL;
	while( SysTick->VAL && (value - SysTick->VAL < Delay) ){
		;
	}
}

/*! @} */
