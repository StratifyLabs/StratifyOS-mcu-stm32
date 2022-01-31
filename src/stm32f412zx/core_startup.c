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

#include <cortexm/cortexm.h>
#include <cortexm/fault.h>
#include <mcu/mcu.h>
#include <sos/events.h>
#include <sos/symbols.h>

#include "../core/core_startup.h"

void mcu_core_default_isr();

// ISR's -- weakly bound to default handler
_DECLARE_ISR(wwdg); // 0
_DECLARE_ISR(pvd);
_DECLARE_ISR(tamp_stamp);
_DECLARE_ISR(rtc_wkup);
_DECLARE_ISR(flash);
_DECLARE_ISR(rcc); // 5
_DECLARE_ISR(exti0);
_DECLARE_ISR(exti1);
_DECLARE_ISR(exti2);
_DECLARE_ISR(exti3);
_DECLARE_ISR(exti4); // 10
_DECLARE_ISR(dma1_stream0);
_DECLARE_ISR(dma1_stream1);
_DECLARE_ISR(dma1_stream2);
_DECLARE_ISR(dma1_stream3);
_DECLARE_ISR(dma1_stream4); // 15
_DECLARE_ISR(dma1_stream5);
_DECLARE_ISR(dma1_stream6);
_DECLARE_ISR(adc); // 18
_DECLARE_ISR(can1_tx);
_DECLARE_ISR(can1_rx0);
_DECLARE_ISR(can1_rx1);
_DECLARE_ISR(can1_sce);
_DECLARE_ISR(exti9_5); // 23
_DECLARE_ISR(tim1_brk_tim9);
_DECLARE_ISR(tim1_up_tim10); // 25
_DECLARE_ISR(tim1_trg_com_tim11);
_DECLARE_ISR(tim1_cc);
_DECLARE_ISR(tim2);
_DECLARE_ISR(tim3);
_DECLARE_ISR(tim4); // 30
_DECLARE_ISR(i2c1_ev);
_DECLARE_ISR(i2c1_er);
_DECLARE_ISR(i2c2_ev);
_DECLARE_ISR(i2c2_er);
_DECLARE_ISR(spi1); // 35
_DECLARE_ISR(spi2);
_DECLARE_ISR(usart1);
_DECLARE_ISR(usart2);
_DECLARE_ISR(usart3);
_DECLARE_ISR(exti15_10); // 40
_DECLARE_ISR(rtc_alarm);
_DECLARE_ISR(otg_fs_wkup);
_DECLARE_ISR(tim8_brk_tim12);
_DECLARE_ISR(tim8_up_tim13);
_DECLARE_ISR(tim8_trg_com_tim14); // 45
_DECLARE_ISR(tim8_cc);
_DECLARE_ISR(dma1_stream7); // 47
//_DECLARE_ISR(fmc);
_DECLARE_ISR(sdio);
_DECLARE_ISR(tim5); // 50
_DECLARE_ISR(spi3); // 51
//_DECLARE_ISR(uart4);
//_DECLARE_ISR(uart5);
_DECLARE_ISR(tim6_dac);
_DECLARE_ISR(tim7);
_DECLARE_ISR(dma2_stream0); // 56
_DECLARE_ISR(dma2_stream1);
_DECLARE_ISR(dma2_stream2);
_DECLARE_ISR(dma2_stream3);
_DECLARE_ISR(dma2_stream4); // 60
_DECLARE_ISR(dfsdm1_flt0);
_DECLARE_ISR(dfsdm1_flt1);

_DECLARE_ISR(can2_tx); // 63
_DECLARE_ISR(can2_rx0);
_DECLARE_ISR(can2_rx1);
_DECLARE_ISR(can2_sce);
// 65
// 66
_DECLARE_ISR(otg_fs);       // 67
_DECLARE_ISR(dma2_stream5); // 68
_DECLARE_ISR(dma2_stream6);
_DECLARE_ISR(dma2_stream7); // 70
_DECLARE_ISR(usart6);
_DECLARE_ISR(i2c3_ev);
_DECLARE_ISR(i2c3_er); // 73
//_DECLARE_ISR(otg_hs_ep1_out);
//_DECLARE_ISR(otg_hs_ep1_in);
//_DECLARE_ISR(otg_hs_wkup);
//_DECLARE_ISR(otg_hs);
_DECLARE_ISR(rng);  // 80
_DECLARE_ISR(fpu);  // 81
_DECLARE_ISR(spi4); // 84
_DECLARE_ISR(spi5); // 85
//_DECLARE_ISR(sai1);
//_DECLARE_ISR(sai2);
_DECLARE_ISR(quadspi); // 92
//_DECLARE_ISR(cec);
//_DECLARE_ISR(spdif_rx);
_DECLARE_ISR(fmpi2c1_ev); // 95
_DECLARE_ISR(fmpi2c1_er); // 96

void (*const mcu_core_vector_table[])() __attribute__((section(".startup"))) = {
  // Core Level - CM3
  (void *)&_top_of_stack,        // The initial stack pointer
  cortexm_reset_handler,         // The reset handler
  cortexm_nmi_handler,           // The NMI handler
  cortexm_hardfault_handler,     // The hard fault handler
  cortexm_memfault_handler,      // The MPU fault handler
  cortexm_busfault_handler,      // The bus fault handler
  cortexm_usagefault_handler,    // The usage fault handler
  (void *)&_sos_hardware_id,     // Reserved
  0,                             // Reserved
  (void *)&sos_config.boot.api,  // Reserved -- this is the kernel signature
                                 // checksum value 0x24
  0,                             // Reserved
  cortexm_svcall_handler,        // SVCall handler
  cortexm_debug_monitor_handler, // Debug monitor handler
  0,                             // Reserved
  cortexm_pendsv_handler,        // The PendSV handler
  cortexm_systick_handler,       // The SysTick handler
  // Non Cortex M interrupts (device specific interrupts)

  _ISR(wwdg), // 0
  _ISR(pvd),
  _ISR(tamp_stamp),
  _ISR(rtc_wkup),
  _ISR(flash),
  _ISR(rcc),
  _ISR(exti0),
  _ISR(exti1),
  _ISR(exti2),
  _ISR(exti3),
  _ISR(exti4), // 10
  _ISR(dma1_stream0),
  _ISR(dma1_stream1),
  _ISR(dma1_stream2),
  _ISR(dma1_stream3),
  _ISR(dma1_stream4),
  _ISR(dma1_stream5),
  _ISR(dma1_stream6),
  _ISR(adc), // 18
  _ISR(can1_tx),
  _ISR(can1_rx0), // 20
  _ISR(can1_rx1),
  _ISR(can1_sce),
  _ISR(exti9_5), // 23
  _ISR(tim1_brk_tim9),
  _ISR(tim1_up_tim10),
  _ISR(tim1_trg_com_tim11),
  _ISR(tim1_cc),
  _ISR(tim2),
  _ISR(tim3),
  _ISR(tim4), // 30
  _ISR(i2c1_ev),
  _ISR(i2c1_er),
  _ISR(i2c2_ev),
  _ISR(i2c2_er),
  _ISR(spi1),
  _ISR(spi2),
  _ISR(usart1),
  _ISR(usart2),
  _ISR(usart3),
  _ISR(exti15_10), // 40
  _ISR(rtc_alarm),
  _ISR(otg_fs_wkup),
  _ISR(tim8_brk_tim12),
  _ISR(tim8_up_tim13),      // 44
  _ISR(tim8_trg_com_tim14), // 45
  _ISR(tim8_cc),            // 46
  _ISR(dma1_stream7),       // 47
  mcu_core_default_isr,
  _ISR(sdio),
  _ISR(tim5), // 50
  _ISR(spi3), // 51
  mcu_core_default_isr,
  mcu_core_default_isr,
  _ISR(tim6_dac), // 54
  _ISR(tim7),
  _ISR(dma2_stream0), // 56
  _ISR(dma2_stream1),
  _ISR(dma2_stream2),
  _ISR(dma2_stream3),
  _ISR(dma2_stream4), // 60
  _ISR(dfsdm1_flt0),
  _ISR(dfsdm1_flt1),
  _ISR(can2_tx),
  _ISR(can2_rx0),
  _ISR(can2_rx1),
  _ISR(can2_sce),
  _ISR(otg_fs), // 67
  _ISR(dma2_stream5),
  _ISR(dma2_stream6),
  _ISR(dma2_stream7), // 70
  _ISR(usart6),       // 71
  _ISR(i2c3_ev),      // 72
  _ISR(i2c3_er),      // 73
  mcu_core_default_isr,
  mcu_core_default_isr,
  mcu_core_default_isr,
  mcu_core_default_isr,
  mcu_core_default_isr,
  mcu_core_default_isr,
  mcu_core_default_isr, // 80
  _ISR(fpu),            // 81
  mcu_core_default_isr,
  mcu_core_default_isr,
  _ISR(spi4), // 84
  _ISR(spi5), // 85
  mcu_core_default_isr,
  mcu_core_default_isr,
  mcu_core_default_isr,
  mcu_core_default_isr,
  mcu_core_default_isr, // 90
  mcu_core_default_isr,
  _ISR(quadspi), // 92
  mcu_core_default_isr,
  mcu_core_default_isr,
  _ISR(fmpi2c1_ev), // 95
  _ISR(fmpi2c1_er)};

void mcu_core_default_isr() { sos_handle_event(SOS_EVENT_ROOT_FATAL, "dflt"); }
