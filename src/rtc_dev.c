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
	mcu_event_handler_t count_handler;
	u8 ref_count;
} rtc_local_t;

rtc_local_t m_rtc_local[MCU_RTC_PORTS] MCU_SYS_MEM;
static RTC_TypeDef * const rtc_regs[MCU_RTC_PORTS] = MCU_RTC_REGS;
static const int rtc_irqs[MCU_RTC_PORTS] = MCU_RTC_IRQS;

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(rtc, RTC_VERSION, RTC_IOC_IDENT_CHAR, I_MCU_TOTAL + I_RTC_TOTAL, mcu_rtc_set, mcu_rtc_get)
static u8 year_is_leap(u16 year);
static u16 get_yday_self(u8 mon, u8 day, u16 year);
static u8 year_is_leap(u16 year){
	return ((year % 4 == 0) && (year % 100 != 0)) || ((year % 400 == 0));
}
static u16 get_yday_self(u8 mon, u8 day, u16 year){
	static const u16 days[2][13] = {
		{0, 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334},
		{0, 0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335}
	};
	u8 leap = year_is_leap(year);
	return days[leap][mon] + day;
}

static void convert_rtc_to_stm32(
		const rtc_time_t * source,
		RTC_TimeTypeDef * dest_time,
		RTC_DateTypeDef * dest_date
		);

static void convert_stm32_to_rtc(
		const RTC_TimeTypeDef * source_time,
		const RTC_DateTypeDef * source_date,
		rtc_time_t * dest
		);

static int set_alarm(rtc_local_t * local, const rtc_attr_t * attr);
static int set_count(rtc_local_t * local, const rtc_attr_t * attr);

int mcu_rtc_open(const devfs_handle_t * handle){
	DEVFS_DRIVER_DECLARE_LOCAL(rtc, MCU_RTC_PORTS);
	if( port < MCU_RTC_PORTS ){
		if ( local->ref_count == 0 ){
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
			local->alarm_handler.callback = NULL;
			local->hal_handle.Instance = rtc_regs[port];

			//uses EXTI line 17 -- need to enable on rising edge

			cortexm_enable_irq(rtc_irqs[port]);
		}
		local->ref_count++;
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
	info->o_flags =
			RTC_FLAG_ENABLE |
			RTC_FLAG_ENABLE_ALARM |
			RTC_FLAG_IS_ALARM_MONTHLY |
			RTC_FLAG_IS_ALARM_DAILY |
			RTC_FLAG_IS_ALARM_ONCE |
			RTC_FLAG_IS_ALARM_HOURLY |
			RTC_FLAG_IS_ALARM_MINUTE |
			RTC_FLAG_IS_ALARM_MINUTE |
			RTC_FLAG_DISABLE;

	return 0;
}

int mcu_rtc_set(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(rtc, MCU_RTC_PORTS);
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	const rtc_time_t * time_p = ctl;

	convert_rtc_to_stm32(time_p, &sTime, &sDate);

	if(HAL_RTC_SetTime(&local->hal_handle, &sTime, RTC_FORMAT_BIN)!= HAL_OK){
		return SYSFS_SET_RETURN(EIO);
	}

	if(HAL_RTC_SetDate(&local->hal_handle, &sDate, RTC_FORMAT_BIN)!= HAL_OK){
		return SYSFS_SET_RETURN(EIO);
	}
	return 0;
}

int mcu_rtc_get(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(rtc, MCU_RTC_PORTS);
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	rtc_time_t * time_p	= ctl;

	if(HAL_RTC_GetTime(&local->hal_handle, &sTime, RTC_FORMAT_BIN)!= HAL_OK){
		return SYSFS_SET_RETURN(EIO);
	}

	if(HAL_RTC_GetDate(&local->hal_handle, &sDate, RTC_FORMAT_BIN)!= HAL_OK){
		return SYSFS_SET_RETURN(EIO);
	}

	convert_stm32_to_rtc(&sTime, &sDate, time_p);
	return 0;
}

int mcu_rtc_setattr(const devfs_handle_t * handle, void * ctl){
	/*Initialize RTC Only */
	DEVFS_DRIVER_DECLARE_LOCAL(rtc, MCU_RTC_PORTS);
	int result;

	const rtc_attr_t * attr;
	attr = mcu_select_attr(handle, ctl);
	if( attr == 0 ){
		return SYSFS_SET_RETURN(EINVAL);
	}

	u32 o_flags = attr->o_flags;

	if( o_flags & RTC_FLAG_ENABLE ){
		//set the init values based on the flags passed, we may need to add some flags to sos/dev/rtc.h
		/*Initialize RTC Only */
		HAL_PWR_EnableBkUpAccess();

		__HAL_RCC_RTC_ENABLE();
		local->hal_handle.Init.HourFormat = RTC_HOURFORMAT_24;
		local->hal_handle.Init.AsynchPrediv = 127;
		local->hal_handle.Init.SynchPrediv = 255;
		local->hal_handle.Init.OutPut = RTC_OUTPUT_DISABLE;
		local->hal_handle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
		local->hal_handle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

		if( (result = HAL_RTC_Init(&local->hal_handle)) != HAL_OK ){
			return SYSFS_SET_RETURN(EIO);
		}
	}
	if( o_flags & RTC_FLAG_ENABLE_ALARM ){
		set_alarm(local, attr);
	} else if( o_flags & RTC_FLAG_DISABLE_ALARM ){
		//disable the alarm
		HAL_RTC_DeactivateAlarm(&local->hal_handle, RTC_ALARM_A);
	} else if( o_flags & RTC_FLAG_ENABLE_COUNT_EVENT ){
		set_count(local, attr);
	} else if( o_flags & RTC_FLAG_DISABLE_COUNT_EVENT ){
		HAL_RTC_DeactivateAlarm(&local->hal_handle, RTC_ALARM_B);
	}

	return 0;

}


int mcu_rtc_setaction(const devfs_handle_t * handle, void * ctl){
	mcu_action_t * action = ctl;
	//assign value to rtc_local[port].alarm_handler
	cortexm_set_irq_priority(RTC_Alarm_IRQn, action->prio, action->o_events);
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
	devfs_execute_event_handler(&rtc->alarm_handler, o_events, &rtc_data);
}

void HAL_RTC_AlarmBEventCallback(RTC_HandleTypeDef *hrtc){
	rtc_local_t * rtc = (rtc_local_t *)hrtc;
	rtc_event_t rtc_data;
	u32 o_events;
	rtc_data.time.time.tm_hour = 0; //set the alarm value
	o_events = MCU_EVENT_FLAG_COUNT;
	devfs_execute_event_handler(&rtc->count_handler, o_events, &rtc_data);
}

void mcu_core_rtc_alarm_isr(){
	HAL_RTC_AlarmIRQHandler(&m_rtc_local[0].hal_handle);
}

int set_alarm(rtc_local_t * local, const rtc_attr_t * attr){
	RTC_AlarmTypeDef alarm = {0};
	RTC_DateTypeDef date;

	//set the alarm based on o_flags at attr
	u32 o_flags = attr->o_flags;

	convert_rtc_to_stm32(&attr->time, &alarm.AlarmTime, &date);

	alarm.Alarm = RTC_ALARM_A;
	alarm.AlarmDateWeekDay = date.Date;
	alarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
#if defined RTC_ALARMSUBSECONDMASK_ALL
	alarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
#endif

	alarm.AlarmMask = RTC_ALARMMASK_NONE; //all alarms used
	if( o_flags & RTC_FLAG_IS_ALARM_DAILY ){
		//don't care about month date (just hours, minutes, seconds)
		alarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
	} else if( o_flags & RTC_FLAG_IS_ALARM_HOURLY ){
		alarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY | RTC_ALARMMASK_HOURS;
	} else if( o_flags & RTC_FLAG_IS_ALARM_MINUTE ){
		alarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY | RTC_ALARMMASK_HOURS |RTC_ALARMMASK_MINUTES;
	}

	//enable the alarm
	if( HAL_RTC_SetAlarm_IT(&local->hal_handle, &alarm, RTC_FORMAT_BIN) != HAL_OK ){
		mcu_debug_log_error(MCU_DEBUG_DEVICE, "HAL_RTC_SetAlarm");
		return SYSFS_SET_RETURN(EIO);
	}

	return SYSFS_RETURN_SUCCESS;
}

int set_count(rtc_local_t * local, const rtc_attr_t * attr){
	RTC_AlarmTypeDef alarm = {0};
	RTC_DateTypeDef date;

	//set the alarm based on o_flags at attr
	u32 o_flags = attr->o_flags;

	convert_rtc_to_stm32(&attr->time, &alarm.AlarmTime, &date);

	alarm.Alarm = RTC_ALARM_B;
	alarm.AlarmDateWeekDay = date.Date;
	alarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
#if defined RTC_ALARMSUBSECONDMASK_ALL
	alarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
#endif

	//default is to count every second
	alarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY | RTC_ALARMMASK_HOURS |RTC_ALARMMASK_MINUTES | RTC_ALARMMASK_SECONDS;
	if( o_flags & RTC_FLAG_IS_COUNT_DAY_OF_MONTH ){
		//don't care about month date (just hours, minutes, seconds)
		alarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
	} else if( o_flags & RTC_FLAG_IS_COUNT_HOUR ){
		alarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY | RTC_ALARMMASK_HOURS;
	} else if( o_flags & RTC_FLAG_IS_COUNT_MINUTE ){
		alarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY | RTC_ALARMMASK_HOURS |RTC_ALARMMASK_MINUTES;
	}

	//enable the alarm
	if( HAL_RTC_SetAlarm_IT(&local->hal_handle, &alarm, RTC_FORMAT_BIN) != HAL_OK ){
		return SYSFS_SET_RETURN(EIO);
	}
	return SYSFS_RETURN_SUCCESS;
}

void convert_rtc_to_stm32(
		const rtc_time_t * source,
		RTC_TimeTypeDef * dest_time,
		RTC_DateTypeDef * dest_date
		){
	dest_time->TimeFormat = RTC_HOURFORMAT12_AM; //AM or 24 hour format -- always 24 hour format
	dest_time->Hours = source->time.tm_hour;
	dest_time->Minutes = source->time.tm_min;
	dest_time->Seconds = source->time.tm_sec;
#if defined RTC_ALARMSUBSECONDMASK_NONE
	dest_time->SubSeconds = 0;
#endif
	dest_time->DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	dest_date->Date = source->time.tm_mday;
	dest_date->Month = source->time.tm_mon;
	dest_date->Year = source->time.tm_year;
	dest_date->WeekDay = source->time.tm_wday;
}

void convert_stm32_to_rtc(
		const RTC_TimeTypeDef * source_time,
		const RTC_DateTypeDef * source_date,
		rtc_time_t * dest
		){
	dest->time.tm_sec = source_time->Seconds;
	dest->time.tm_min = source_time->Minutes;
	dest->time.tm_hour = source_time->Hours;
	dest->time.tm_wday = source_date->WeekDay;
	dest->time.tm_mday = source_date->Date;
	dest->time.tm_mon = source_date->Month;
	dest->time.tm_year = source_date->Year;
	dest->time.tm_yday = get_yday_self(
				dest->time.tm_mon,
				dest->time.tm_mday,
				dest->time.tm_year);
	dest->useconds = 0;
}


#endif

