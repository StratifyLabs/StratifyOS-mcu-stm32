// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md

#include "cortexm/fault.h"

int fault_dev_save(const fault_t *fault) {
#if 0
	LPC_RTC->GPREG0 = fault->num;
	LPC_RTC->GPREG1 = (uint32_t)fault->pc;
	LPC_RTC->GPREG2 = (uint32_t)fault->caller;
	LPC_RTC->GPREG3 = (uint32_t)fault->handler_pc;
	LPC_RTC->GPREG4 = (uint32_t)fault->handler_caller;
#endif
  return 0;
}

int fault_dev_load(fault_t *fault) {
#if 0
	fault->num = LPC_RTC->GPREG0;
	fault->pc = (void*)LPC_RTC->GPREG1;
	fault->caller = (void*)LPC_RTC->GPREG2;
	fault->handler_pc = (void*)LPC_RTC->GPREG3;
	fault->handler_caller = (void*)LPC_RTC->GPREG4;
	fault->addr = (void*)0xFFFFFFFF;
	LPC_RTC->GPREG0 = 0; //clear any existing faults since it has been read
	LPC_RTC->GPREG1 = 0; //clear any existing faults since it has been read
	LPC_RTC->GPREG2 = 0; //clear any existing faults since it has been read
	LPC_RTC->GPREG3 = 0; //clear any existing faults since it has been read
	LPC_RTC->GPREG4 = 0; //clear any existing faults since it has been read
#endif
  return 0;
}
