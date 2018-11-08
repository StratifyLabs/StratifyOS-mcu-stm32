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

#include <mcu/rtc.h>
#include "stm32_local.h"

#if MCU_RTC_PORTS > 0

typedef struct {
	RTC_HandleTypeDef hal_handle; //must be the first member of the struct
	mcu_event_handler_t alarm_handler;
	u8 ref_count;
} rtc_local_t;

rtc_local_t rtc_local[MCU_RTC_PORTS] MCU_SYS_MEM;
static RTC_TypeDef * const rtc_regs[MCU_RTC_PORTS] = MCU_RTC_REGS;
static const int rtc_irqs[MCU_RTC_PORTS] = MCU_RTC_IRQS;

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(rtc, RTC_VERSION, RTC_IOC_IDENT_CHAR, I_MCU_TOTAL + I_RTC_TOTAL, mcu_rtc_set, mcu_rtc_get)
static u8 year_is_leap(u16 year);
static u16 get_yday_self(u8 mon, u8 day, u16 year);
static u8 year_is_leap(u16 year){
	return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}
static u16 get_yday_self(u8 mon, u8 day, u16 year){
	static const u16 days[2][13] = {
		{0, 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334},
		{0, 0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335}
	};
	u8 leap = year_is_leap(year);
	return days[leap][mon] + day;
}

int mcu_rtc_open(const devfs_handle_t * handle){
	int port = handle->port;
	if( port < MCU_RTC_PORTS ){
		if ( rtc_local[port].ref_count == 0 ){
			//turn on RCC clock
			switch(port){
				case 0:
#if defined STM32H7
					__HAL_RCC_RTC_CLK_ENABLE();
#endif
#if defined STM32F4 || defined STM32F7
					__HAL_RCC_RTC_ENABLE();
#endif

					break;
			}
			rtc_local[port].alarm_handler.callback = NULL;
			rtc_local[port].hal_handle.Instance = rtc_regs[port];
			cortexm_enable_irq(rtc_irqs[port]);
		}
		rtc_local[port].ref_count++;
	}
	return 0;
}

int mcu_rtc_close(const devfs_handle_t * handle){

	//do the opposite of mcu_rtc_open() -- ref_count is zero -- turn off interrupt
	return 0;
}

int mcu_rtc_getinfo(const devfs_handle_t * handle, void * ctl){
	rtc_info_t * info = ctl;

	//set flags that are supported by this driver
	info->o_flags = RTC_FLAG_ENABLE | RTC_FLAG_DISABLE;

	return 0;
}

int mcu_rtc_set(const devfs_handle_t * handle, void * ctl){
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	rtc_time_t * time_p;
	int port = handle->port;
	time_p = (rtc_time_t*)ctl;
	sTime.Hours = time_p->time.tm_hour;
	sTime.Minutes = time_p->time.tm_min;
	sTime.Seconds = time_p->time.tm_sec;
	if(HAL_RTC_SetTime(&rtc_local[port].hal_handle, &sTime, RTC_FORMAT_BIN)!= HAL_OK){
		return SYSFS_SET_RETURN(EIO);
	}
	sDate.Date = time_p->time.tm_mday;
	sDate.Month = time_p->time.tm_mon;//maybe + 1
	sDate.Year = time_p->time.tm_year;
	sDate.WeekDay = time_p->time.tm_wday;
	if(HAL_RTC_SetDate(&rtc_local[port].hal_handle, &sDate, RTC_FORMAT_BIN)!= HAL_OK){
		return SYSFS_SET_RETURN(EIO);
	}
	return 0;
}

int mcu_rtc_get(const devfs_handle_t * handle, void * ctl){
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	rtc_time_t * time_p;
	int port = handle->port;
	time_p = (rtc_time_t*)ctl;
	if(HAL_RTC_GetTime(&rtc_local[port].hal_handle, &sTime, RTC_FORMAT_BIN)!= HAL_OK){
		return SYSFS_SET_RETURN(EIO);
	}
	if(HAL_RTC_GetDate(&rtc_local[port].hal_handle, &sDate, RTC_FORMAT_BIN)!= HAL_OK){
		return SYSFS_SET_RETURN(EIO);
	}
	time_p->time.tm_sec = sTime.Seconds;
	time_p->time.tm_min = sTime.Minutes;
	time_p->time.tm_hour = sTime.Hours;
	time_p->time.tm_wday = sDate.WeekDay;
	time_p->time.tm_mday = sDate.Date;
	time_p->time.tm_mon = sDate.Month;
	time_p->time.tm_year = sDate.Year;
	time_p->time.tm_yday = get_yday_self(time_p->time.tm_mon,\
													 time_p->time.tm_mday,time_p->time.tm_year);
	time_p->useconds = 0;
	return 0;
}

int mcu_rtc_setattr(const devfs_handle_t * handle, void * ctl){
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
	/*Initialize RTC Only */
	int port = handle->port;
	rtc_attr_t * attr = ctl;
	u32 o_flags = attr->o_flags;
	if( o_flags & RTC_FLAG_IS_SOURCE_INTERNAL_40000){
		RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI;
		RCC_OscInitStruct.LSIState = RCC_LSI_ON;
		PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
		mcu_debug_log_info(MCU_DEBUG_DEVICE, "RTC_FLAG_IS_SOURCE_INTERNAL_40000");
	}else{
		RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
		RCC_OscInitStruct.LSEState = RCC_LSE_ON;
		PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
		mcu_debug_log_info(MCU_DEBUG_DEVICE, "RTC_FLAG_IS_SOURCE_external");
	}

	rtc_local[port].hal_handle.Init.HourFormat = RTC_HOURFORMAT_24;


	if( o_flags & RTC_FLAG_ENABLE ){
		//set the init values based on the flags passed, we may need to add some flags to sos/dev/rtc.h
		/*Initialize RTC Only */
		HAL_PWR_EnableBkUpAccess();

		RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK){
			mcu_debug_log_error(MCU_DEBUG_DEVICE, "HAL_RCCEx_PeriphCLKConfig error");
			return SYSFS_SET_RETURN(EIO);
		}

		if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
			mcu_debug_log_error(MCU_DEBUG_DEVICE, "HAL_RCC_OscConfig 1 error");
			return SYSFS_SET_RETURN(EIO);
		}
		__HAL_RCC_RTC_ENABLE();
		rtc_local[port].hal_handle.Init.AsynchPrediv = 127;
		rtc_local[port].hal_handle.Init.SynchPrediv = 255;
		rtc_local[port].hal_handle.Init.OutPut = RTC_OUTPUT_DISABLE;
		rtc_local[port].hal_handle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
		rtc_local[port].hal_handle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
		if (HAL_RTC_Init(&rtc_local[port].hal_handle) != HAL_OK){
			mcu_debug_log_error(MCU_DEBUG_DEVICE, "HAL_RTC_Init");
			return SYSFS_SET_RETURN(EIO);
		}
	}
	if( o_flags & RTC_FLAG_ENABLE_ALARM ){
		RTC_AlarmTypeDef alarm;
		u32 format = 0;

		//set the alarm based on o_flags at attr
		alarm.AlarmTime.Hours = attr->time.time.tm_hour;
		alarm.AlarmTime.Minutes = attr->time.time.tm_min;

		//enable the alarm
		if( HAL_RTC_SetAlarm(&rtc_local[port].hal_handle, &alarm, format) != HAL_OK ){
			mcu_debug_log_error(MCU_DEBUG_DEVICE, "HAL_RTC_SetAlarm");
			return SYSFS_SET_RETURN(EIO);
		}
	} else if( o_flags & RTC_FLAG_DISABLE_ALARM ){

		//disable the alarm
	}

	return 0;

}


int mcu_rtc_setaction(const devfs_handle_t * handle, void * ctl){
	//mcu_action_t * action = ctl;
	//assign value to rtc_local[port].alarm_handler


	return 0;
}

int mcu_rtc_write(const devfs_handle_t * handle, devfs_async_t * async){
	return SYSFS_SET_RETURN(ENOTSUP);
}

int mcu_rtc_read(const devfs_handle_t * handle, devfs_async_t * async){
	return SYSFS_SET_RETURN(ENOTSUP);
}


void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc){
	rtc_local_t * rtc = (rtc_local_t *)hrtc;
	rtc_event_t rtc_data;
	u32 o_events;
	rtc_data.time.time.tm_hour = 0; //set the alarm value
	o_events = MCU_EVENT_FLAG_ALARM;
	mcu_execute_event_handler(&rtc->alarm_handler, o_events, &rtc_data);
}

void mcu_core_rtc_wkup_isr(){
	HAL_RTC_AlarmIRQHandler(&rtc_local[0].hal_handle);
}



#endif

