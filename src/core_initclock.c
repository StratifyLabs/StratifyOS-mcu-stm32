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


void sos_led_root_enable(void * args);

//requires mcu_core_osc_freq, mcu_board_config.core_cpu_freq, and mcu_board_config.core_periph_freq to be defined ext
int mcu_core_initclock(int div){


#if defined STM32H7
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

	/**Supply configuration update enable
		 */
	MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);

	/**Configure the main internal regulator output voltage
		 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	while ((PWR->D3CR & (PWR_D3CR_VOSRDY)) != PWR_D3CR_VOSRDY)
	{

	}
	/**Initializes the CPU, AHB and APB busses clocks
		 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 24;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
		return -1;
	}

	/**Initializes the CPU, AHB and APB busses clocks
		 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
			|RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK){
		return -1;
	}

	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_USB;
	PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
	PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK){
		return -1;
	}



#elif !defined STM32L4
	const stm32_config_t * config = mcu_board_config.arch_config;

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	//RCC_PeriphCLKInitTypeDef PeriphClkInit;

	__HAL_RCC_PWR_CLK_ENABLE();
	//use HAL_PWREx_ControlVoltageScaling?
	__HAL_PWR_VOLTAGESCALING_CONFIG(stm32_local_decode_voltage_scale(config->clock_voltage_scale-1));


	//if HSE flag is on
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = config->clock_pllm;
	RCC_OscInitStruct.PLL.PLLN = config->clock_plln;
	RCC_OscInitStruct.PLL.PLLP = config->clock_pllp;
	RCC_OscInitStruct.PLL.PLLQ = config->clock_pllq;

#if STM32_LOCAL_HAS_RCC_PLLR
	RCC_OscInitStruct.PLL.PLLR = config->clock_pllr;
#endif

	int result;
	if ((result = HAL_RCC_OscConfig(&RCC_OscInitStruct)) != HAL_OK){
		if( result == HAL_TIMEOUT ){
			sos_led_root_enable(0);
		}

		return -1;
	}

#if defined STM32F7
	if( config->o_flags & STM32_CONFIG_FLAG_IS_OVERDRIVE_ON ){
		if (HAL_PWREx_EnableOverDrive() != HAL_OK){
			return -1;
		}
	}
#endif


	RCC_ClkInitStruct.ClockType =
			RCC_CLOCKTYPE_HCLK |
			RCC_CLOCKTYPE_SYSCLK |
			RCC_CLOCKTYPE_PCLK1 |
			RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = stm32_local_decode_sysclk_divider(config->clock_ahb_clock_divider);
	RCC_ClkInitStruct.APB1CLKDivider = stm32_local_decode_hclk_divider(config->clock_apb1_clock_divider);
	RCC_ClkInitStruct.APB2CLKDivider = stm32_local_decode_hclk_divider(config->clock_apb2_clock_divider);

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, stm32_local_decode_flash_latency(config->clock_flash_latency)) != HAL_OK){
		return -1;
	}


#if STM32_LOCAL_HAS_PERIPH_CLOCK_SELECTOR
	PeriphClkInit.PeriphClockSelection =
		#if defined RCC_PERIPHCLK_USART1
			RCC_PERIPHCLK_USART1 |
		#endif
		#if defined RCC_PERIPHCLK_USART3
			RCC_PERIPHCLK_USART3 |
		#endif
		#if defined RCC_PERIPHCLK_DFSDM1
			RCC_PERIPHCLK_DFSDM1 |
		#endif
		#if defined RCC_PERIPHCLK_USB
			RCC_PERIPHCLK_USB |
		#endif
		#if defined RCC_PERIPHCLK_USB
			RCC_PERIPHCLK_USB |
		#endif
			0;

#if defined RCC_USART1CLKSOURCE_PCLK2
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
#endif

#if defined RCC_USART3CLKSOURCE_PCLK1
	PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
#endif

#if defined RCC_I2C2CLKSOURCE_PCLK1
	PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
#endif

#if defined RCC_DFSDM1CLKSOURCE_PCLK
	PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
#endif

#if defined RCC_USBCLKSOURCE_PLLSAI1
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
#endif

	PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
	PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
	PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
	PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;

	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK){
		return -1;
	}

#endif


#if STM32_LOCAL_HAS_PERIPH_CLOCK_48
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;

	PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;

	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK){
		return -1;
	}

#endif


#else

	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_RCC_PWR_CLK_ENABLE();

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Configure LSE Drive Capability
		  */
	//HAL_PWR_EnableBkUpAccess();
	//__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

	/**Initializes the CPU, AHB and APB busses clocks
		  */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	//RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 40;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		return -1;
	}

	/**Initializes the CPU, AHB and APB busses clocks
		  */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		return -1;
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|
		#if defined USART3
			RCC_PERIPHCLK_USART3 |
		#endif
		#if defined I2C2
			RCC_PERIPHCLK_I2C2|
		#endif
		#if defined DFSDM1_Filter0
			RCC_PERIPHCLK_DFSDM1 |
		#endif
			RCC_PERIPHCLK_USB;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
#if defined USART3
	PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
#endif
#if defined I2C2
	PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
#endif
#if defined DFSDM1_Filter0
	PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
#endif
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
	PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
	PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
	PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
	PeriphClkInit.PLLSAI1.PLLSAI1N = 72;
	PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
	PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV6;
	PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK|RCC_PLLSAI1_48M2CLK;

	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		return -1;
	}

	/**Configure the main internal regulator output voltage
		  */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK){
		return -1;
	}

	/**Configure the Systick interrupt time
		  */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	/**Configure the Systick
		  */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/**Enable MSI Auto calibration
		  */
	HAL_RCCEx_EnableMSIPLLMode();

#endif

	cortexm_set_systick_reload(0x00ffffff);
	cortexm_start_systick();
	SystemCoreClock = mcu_board_config.core_cpu_freq;

#if defined STM32F7

#if (ART_ACCLERATOR_ENABLE != 0)
	__HAL_FLASH_ART_ENABLE();
#endif /* ART_ACCLERATOR_ENABLE */

	/* Configure Flash prefetch */
#if (PREFETCH_ENABLE != 0U)
	__HAL_FLASH_PREFETCH_BUFFER_ENABLE();
#endif /* PREFETCH_ENABLE */

#endif

	return 0;
}

//this overrides the weak function in the STM32 HAL library so that Stratify OS can take care of the SYS tick
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority){
	return HAL_OK;
}

uint32_t HAL_GetTick(void){
#if THIS_DOESNT_WORK
	static u32 tick = 0;
	//cortexm_delay_ms(1); //this needs to check if SYSTICK is running before delaying
	return tick++;
#else
	return 0;
#endif
}

void HAL_Delay(__IO uint32_t Delay){
	cortexm_delay_systick(Delay);
}

/*! @} */
