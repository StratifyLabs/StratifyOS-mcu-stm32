// Copyright (c) 2011-2022 Tyler Gilbert and Stratify Labs, Inc. See LICENSE.md
// for details

#include <cortexm/cortexm.h>
#include <cortexm/fault.h>
#include <sos/events.h>
#include <sos/symbols.h>

#include "core/core_startup.h"
#include "stm32_local.h"

void mcu_core_default_isr();

// ISR's -- weakly bound to default handler
_DECLARE_ISR(wwdg);               // 0
_DECLARE_ISR(pvd);                // 1
_DECLARE_ISR(tamp_stamp);         // 2
_DECLARE_ISR(rtc_wkup);           // 3
_DECLARE_ISR(flash);              // 4
_DECLARE_ISR(rcc);                // 5
_DECLARE_ISR(exti0);              // 6
_DECLARE_ISR(exti1);              // 7
_DECLARE_ISR(exti2);              // 8
_DECLARE_ISR(exti3);              // 9
_DECLARE_ISR(exti4);              // 10
_DECLARE_ISR(dma1_stream0);       // 11
_DECLARE_ISR(dma1_stream1);       // 12
_DECLARE_ISR(dma1_stream2);       // 13
_DECLARE_ISR(dma1_stream3);       // 14
_DECLARE_ISR(dma1_stream4);       // 15
_DECLARE_ISR(dma1_stream5);       // 16
_DECLARE_ISR(dma1_stream6);       // 17
_DECLARE_ISR(adc);                // 18
_DECLARE_ISR(can1_tx);            // 19
_DECLARE_ISR(can1_rx0);           // 20
_DECLARE_ISR(can1_rx1);           // 21
_DECLARE_ISR(can1_sce);           // 22
_DECLARE_ISR(exti9_5);            // 23
_DECLARE_ISR(tim1_brk_tim9);      // 24
_DECLARE_ISR(tim1_up_tim10);      // 25
_DECLARE_ISR(tim1_trg_com_tim11); // 26
_DECLARE_ISR(tim1_cc);            // 27
_DECLARE_ISR(tim2);               // 28
_DECLARE_ISR(tim3);               // 29
_DECLARE_ISR(tim4);               // 30
_DECLARE_ISR(i2c1_ev);            // 31
_DECLARE_ISR(i2c1_er);            // 32
_DECLARE_ISR(i2c2_ev);            // 33
_DECLARE_ISR(i2c2_er);            // 34
_DECLARE_ISR(spi1);               // 35
_DECLARE_ISR(spi2);               // 36
_DECLARE_ISR(usart1);             // 37
_DECLARE_ISR(usart2);             // 38
_DECLARE_ISR(usart3);             // 39
_DECLARE_ISR(exti15_10);          // 40
_DECLARE_ISR(rtc_alarm);          // 41
_DECLARE_ISR(otg_fs_wkup);        // 42
_DECLARE_ISR(tim8_brk_tim12);     // 43
_DECLARE_ISR(tim8_up_tim13);      // 44
_DECLARE_ISR(tim8_trg_com_tim14); // 45
_DECLARE_ISR(tim8_cc);            // 46
_DECLARE_ISR(dma1_stream7);       // 47
_DECLARE_ISR(fsmc);               // 48
_DECLARE_ISR(sdio);               // 49
_DECLARE_ISR(tim5);               // 50
_DECLARE_ISR(spi3);               // 51
_DECLARE_ISR(uart4);              // 52
_DECLARE_ISR(uart5);              // 53
_DECLARE_ISR(tim6_dac);           // 54
_DECLARE_ISR(tim7);               // 55
_DECLARE_ISR(dma2_stream0);       // 56
_DECLARE_ISR(dma2_stream1);       // 57
_DECLARE_ISR(dma2_stream2);       // 58
_DECLARE_ISR(dma2_stream3);       // 59
_DECLARE_ISR(dma2_stream4);       // 60
_DECLARE_ISR(can2_tx);            // 63
_DECLARE_ISR(can2_rx0);           // 64
_DECLARE_ISR(can2_rx1);           // 65
_DECLARE_ISR(can2_sce);           // 66
_DECLARE_ISR(otg_fs);             // 67
_DECLARE_ISR(dma2_stream5);       // 68
_DECLARE_ISR(dma2_stream6);       // 69
_DECLARE_ISR(dma2_stream7);       // 70
_DECLARE_ISR(usart6);             // 71
_DECLARE_ISR(i2c3_ev);            // 72
_DECLARE_ISR(i2c3_er);            // 73
_DECLARE_ISR(otg_hs_ep1_out);     // 74
_DECLARE_ISR(otg_hs_ep1_in);      // 75
_DECLARE_ISR(otg_hs_wkup);        // 76
_DECLARE_ISR(otg_hs);             // 77
_DECLARE_ISR(rng);                // 80
_DECLARE_ISR(fpu);                // 81

void (*const mcu_core_vector_table[])() __attribute__((section(".startup"))) = {
  (void *)&_top_of_stack,        // The initial stack pointer
  cortexm_reset_handler,         // The reset handler
  cortexm_nmi_handler,           // The NMI handler
  cortexm_hardfault_handler,     // The hard fault handler
  cortexm_memfault_handler,      // The MPU fault handler
  cortexm_busfault_handler,      // The bus fault handler
  cortexm_usagefault_handler,    // The usage fault handler
  (void *)&_sos_hardware_id,     // hardware ID
  0,                             // Reserved
  (void *)&sos_config.boot.api,  // boot API pointer
  0,                             // Reserved
  cortexm_svcall_handler,        // SVCall handler
  cortexm_debug_monitor_handler, // Debug monitor handler
  0,                             // Reserved
  cortexm_pendsv_handler,        // The PendSV handler
  cortexm_systick_handler,       // The SysTick handler
  // Non Cortex M interrupts (device specific interrupts)

  _ISR(wwdg),               // 0
  _ISR(pvd),                // 1
  _ISR(tamp_stamp),         // 2
  _ISR(rtc_wkup),           // 3
  _ISR(flash),              // 4
  _ISR(rcc),                // 5
  _ISR(exti0),              // 6
  _ISR(exti1),              // 7
  _ISR(exti2),              // 8
  _ISR(exti3),              // 9
  _ISR(exti4),              // 10
  _ISR(dma1_stream0),       // 11
  _ISR(dma1_stream1),       // 12
  _ISR(dma1_stream2),       // 13
  _ISR(dma1_stream3),       // 14
  _ISR(dma1_stream4),       // 15
  _ISR(dma1_stream5),       // 16
  _ISR(dma1_stream6),       // 17
  _ISR(adc),                // 18
  _ISR(can1_tx),            // 19
  _ISR(can1_rx0),           // 20
  _ISR(can1_rx1),           // 21
  _ISR(can1_sce),           // 22
  _ISR(exti9_5),            // 23
  _ISR(tim1_brk_tim9),      // 24
  _ISR(tim1_up_tim10),      // 25
  _ISR(tim1_trg_com_tim11), // 26
  _ISR(tim1_cc),            // 27
  _ISR(tim2),               // 28
  _ISR(tim3),               // 29
  _ISR(tim4),               // 30
  _ISR(i2c1_ev),            // 31
  _ISR(i2c1_er),            // 32
  _ISR(i2c2_ev),            // 33
  _ISR(i2c2_er),            // 34
  _ISR(spi1),               // 35
  _ISR(spi2),               // 36
  _ISR(usart1),             // 37
  _ISR(usart2),             // 38
  _ISR(usart3),             // 39
  _ISR(exti15_10),          // 40
  _ISR(rtc_alarm),          // 41
  _ISR(otg_fs_wkup),        // 42
  _ISR(tim8_brk_tim12),     // 43
  _ISR(tim8_up_tim13),      // 44
  _ISR(tim8_trg_com_tim14), // 45
  _ISR(tim8_cc),            // 46
  _ISR(dma1_stream7),       // 47
  _ISR(fsmc),               // 48
  _ISR(sdio),               // 49
  _ISR(tim5),               // 50
  _ISR(spi3),               // 51
  _ISR(uart4),              // 52
  _ISR(uart5),              // 53
  _ISR(tim6_dac),           // 54
  _ISR(tim7),               // 55
  _ISR(dma2_stream0),       // 56
  _ISR(dma2_stream1),       // 57
  _ISR(dma2_stream2),       // 58
  _ISR(dma2_stream3),       // 59
  _ISR(dma2_stream4),       // 60
  mcu_core_default_isr,     // 61
  mcu_core_default_isr,     // 62
  _ISR(can2_tx),            // 63
  _ISR(can2_rx0),           // 64
  _ISR(can2_rx1),           // 65
  _ISR(can2_sce),           // 66
  _ISR(otg_fs),             // 67
  _ISR(dma2_stream5),       // 68
  _ISR(dma2_stream6),       // 69
  _ISR(dma2_stream7),       // 70
  _ISR(usart6),             // 71
  _ISR(i2c3_ev),            // 72
  _ISR(i2c3_er),            // 73
  _ISR(otg_hs_ep1_out),     // 74
  _ISR(otg_hs_ep1_in),      // 75
  _ISR(otg_hs_wkup),        // 76
  _ISR(otg_hs),             // 77
  mcu_core_default_isr,     // 78
  mcu_core_default_isr,     // 79
  _ISR(rng),                // 80
  _ISR(fpu),                // 81
};

void mcu_core_default_isr() { sos_handle_event(SOS_EVENT_ROOT_FATAL, "dflt"); }
