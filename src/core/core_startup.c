// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md

#include <sos/events.h>

#include "core_startup.h"
#include "stm32_local.h"

int pthread_mutex_init(pthread_mutex_t *mutex, const pthread_mutexattr_t *attr)
  MCU_WEAK;

int pthread_mutex_init(
  pthread_mutex_t *mutex,
  const pthread_mutexattr_t *attr) {
  return 0;
}
