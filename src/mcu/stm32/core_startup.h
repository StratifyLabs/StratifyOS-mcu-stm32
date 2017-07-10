/*
 * core_startup.h
 *
 *  Created on: Jul 4, 2017
 *      Author: tgil
 */

#ifndef CORE_STARTUP_H_
#define CORE_STARTUP_H_

#include <mcu/types.h>
#include <mcu/mcu.h>
#include <mcu/cortexm.h>

void core_init();
int _main();

#define _DECLARE_ISR(name) void _mcu_core_##name##_isr() MCU_ALIAS(_mcu_core_default_isr)
#define _ISR(name) _mcu_core_##name##_isr

#endif /* CORE_STARTUP_H_ */
