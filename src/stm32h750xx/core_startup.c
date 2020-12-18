/* Copyright 2011-2021 Tyler Gilbert;
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
#include <mcu/bootloader.h>
#include <mcu/mcu.h>
#include <sos/events.h>

#include "core/core_startup.h"
#include "stm32_local.h"

const bootloader_api_t mcu_core_bootloader_api MCU_WEAK;
const bootloader_api_t mcu_core_bootloader_api = {
  .code_size = 0,
};

void mcu_core_default_isr();

void mcu_core_hardware_id() MCU_ALIAS(mcu_core_default_isr);
void mcu_core_nmi_isr() MCU_WEAK;

// ISR's -- weakly bound to default handler
_DECLARE_ISR(wwdg); // 0
_DECLARE_ISR(pvd);  // 1
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
_DECLARE_ISR(tim1_brk);
_DECLARE_ISR(tim1_up);      // 25
_DECLARE_ISR(tim1_trg_com); // 26
_DECLARE_ISR(tim1_cc);      // 27
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
_DECLARE_ISR(exti15_10);          // 40
_DECLARE_ISR(rtc_alarm);          // 41
_DECLARE_ISR(tim8_brk_tim12);     // 43
_DECLARE_ISR(tim8_up_tim13);      // 44
_DECLARE_ISR(tim8_trg_com_tim14); // 45
_DECLARE_ISR(tim8_cc);
_DECLARE_ISR(dma1_stream7); // 47
_DECLARE_ISR(fmc);
_DECLARE_ISR(sdmmc1);
_DECLARE_ISR(tim5); // 50
_DECLARE_ISR(spi3); // 51
_DECLARE_ISR(uart4);
_DECLARE_ISR(uart5);
_DECLARE_ISR(tim6_dac);
_DECLARE_ISR(tim7);
_DECLARE_ISR(dma2_stream0); // 56
_DECLARE_ISR(dma2_stream1);
_DECLARE_ISR(dma2_stream2);
_DECLARE_ISR(dma2_stream3);
_DECLARE_ISR(dma2_stream4); // 60
_DECLARE_ISR(eth);
_DECLARE_ISR(eth_wkup);
_DECLARE_ISR(can2_tx);      // 63
_DECLARE_ISR(dma2_stream5); // 68
_DECLARE_ISR(dma2_stream6);
_DECLARE_ISR(dma2_stream7); // 70
_DECLARE_ISR(usart6);
_DECLARE_ISR(i2c3_ev);
_DECLARE_ISR(i2c3_er); // 73
_DECLARE_ISR(otg_hs_ep1_out);
_DECLARE_ISR(otg_hs_ep1_in);
_DECLARE_ISR(otg_hs_wkup);
_DECLARE_ISR(otg_hs);
_DECLARE_ISR(dcmi);           // 78
_DECLARE_ISR(cryp);           // 79
_DECLARE_ISR(hash);           // 80
_DECLARE_ISR(fpu);            // 81
_DECLARE_ISR(uart7);          // 82
_DECLARE_ISR(uart8);          // 83
_DECLARE_ISR(spi4);           // 84
_DECLARE_ISR(spi5);           // 85
_DECLARE_ISR(spi6);           // 86
_DECLARE_ISR(sai1);           // 87
_DECLARE_ISR(ltdc);           // 88
_DECLARE_ISR(ltdc_er);        // 89
_DECLARE_ISR(dma2d);          // 90
_DECLARE_ISR(sai2);           // 91
_DECLARE_ISR(quadspi);        // 92
_DECLARE_ISR(lptim1);         // 93
_DECLARE_ISR(cec);            // 94
_DECLARE_ISR(i2c4_ev);        // 95
_DECLARE_ISR(i2c4_er);        // 96
_DECLARE_ISR(spdif);          // 97
_DECLARE_ISR(otg_fs_ep1_out); // 98
_DECLARE_ISR(otg_fs_ep1_in);  // 99
_DECLARE_ISR(otg_fs_wkup);    // 100
_DECLARE_ISR(otg_fs);         // 101
_DECLARE_ISR(dmamux1);        // 102
_DECLARE_ISR(hrtim1_master);  // 103
_DECLARE_ISR(hrtim1_tima);    // 104
_DECLARE_ISR(hrtim1_timb);    // 105
_DECLARE_ISR(hrtim1_timc);    // 106
_DECLARE_ISR(hrtim1_timd);    // 107
_DECLARE_ISR(hrtim1_time);    // 108
_DECLARE_ISR(hrtim1_flt);     // 109
_DECLARE_ISR(dfsdm1_flt0);    // 110
_DECLARE_ISR(dfsdm1_flt1);    // 111
_DECLARE_ISR(dfsdm1_flt2);    // 112
_DECLARE_ISR(dfsdm1_flt3);    // 113
_DECLARE_ISR(sai3);           // 114
_DECLARE_ISR(swpwm1);         // 115
_DECLARE_ISR(tim15);          // 116
_DECLARE_ISR(tim16);          // 117
_DECLARE_ISR(tim17);          // 118
_DECLARE_ISR(mdios_wkup);     // 119
_DECLARE_ISR(mdios);          // 120
_DECLARE_ISR(jpeg);           // 121
_DECLARE_ISR(mdma);           // 122
_DECLARE_ISR(sdmmc2);         // 124
_DECLARE_ISR(hsem1);          // 125
_DECLARE_ISR(adc3);           // 127
_DECLARE_ISR(dmamux2);        // 128
_DECLARE_ISR(bdma_channel0);  // 129
_DECLARE_ISR(bdma_channel1);  // 130
_DECLARE_ISR(bdma_channel2);  // 131
_DECLARE_ISR(bdma_channel3);  // 132
_DECLARE_ISR(bdma_channel4);  // 133
_DECLARE_ISR(bdma_channel5);  // 134
_DECLARE_ISR(bdma_channel6);  // 135
_DECLARE_ISR(bdma_channel7);  // 136
_DECLARE_ISR(comp);           // 137
_DECLARE_ISR(lptim2);         // 138
_DECLARE_ISR(lptim3);         // 139
_DECLARE_ISR(lptim4);         // 140
_DECLARE_ISR(lptim5);         // 141
_DECLARE_ISR(lpuart1);        // 142
_DECLARE_ISR(crs);            // 144
_DECLARE_ISR(sai4);           // 146
_DECLARE_ISR(wakup_pin);      // 149

void (*const mcu_core_vector_table[])() __attribute__((section(".startup"))) = {
  // Core Level - CM3
  (void *)&_top_of_stack,           // The initial stack pointer
  cortexm_reset_handler,            // The reset handler
  mcu_core_nmi_isr,                 // The NMI handler
  cortexm_hardfault_handler,        // The hard fault handler
  cortexm_memfault_handler,         // The MPU fault handler
  cortexm_busfault_handler,         // The bus fault handler
  cortexm_usagefault_handler,       // The usage fault handler
  mcu_core_hardware_id,             // Reserved
  0,                                // Reserved
  (void *)&mcu_core_bootloader_api, // Reserved -- this is the kernel signature
                                    // checksum value 0x24
  0,                                // Reserved
  cortexm_svcall_handler,           // SVCall handler
  cortexm_debug_monitor_handler,    // Debug monitor handler
  0,                                // Reserved
  cortexm_pendsv_handler,           // The PendSV handler
  cortexm_systick_handler,          // The SysTick handler
  // Non Cortex M interrupts (device specific interrupts)

  _ISR(wwdg), // 0
  _ISR(pvd),  // 1
  _ISR(tamp_stamp),
  _ISR(rtc_wkup),
  _ISR(flash),
  _ISR(rcc), // 5
  _ISR(exti0),
  _ISR(exti1),
  _ISR(exti2),
  _ISR(exti3),
  _ISR(exti4), // 10
  _ISR(dma1_stream0),
  _ISR(dma1_stream1),
  _ISR(dma1_stream2),
  _ISR(dma1_stream3),
  _ISR(dma1_stream4), // 15
  _ISR(dma1_stream5),
  _ISR(dma1_stream6),
  _ISR(adc), // 18
  _ISR(can1_tx),
  _ISR(can1_rx0),
  _ISR(can1_rx1),
  _ISR(can1_sce),
  _ISR(exti9_5), // 23
  _ISR(tim1_brk),
  _ISR(tim1_up),      // 25
  _ISR(tim1_trg_com), // 26
  _ISR(tim1_cc),      // 27
  _ISR(tim2),
  _ISR(tim3),
  _ISR(tim4), // 30
  _ISR(i2c1_ev),
  _ISR(i2c1_er),
  _ISR(i2c2_ev),
  _ISR(i2c2_er),
  _ISR(spi1), // 35
  _ISR(spi2),
  _ISR(usart1),
  _ISR(usart2),
  _ISR(usart3),
  _ISR(exti15_10),          // 40
  _ISR(rtc_alarm),          // 41
  mcu_core_default_isr,     // 42
  _ISR(tim8_brk_tim12),     // 43
  _ISR(tim8_up_tim13),      // 44
  _ISR(tim8_trg_com_tim14), // 45
  _ISR(tim8_cc),
  _ISR(dma1_stream7), // 47
  _ISR(fmc),
  _ISR(sdmmc1),
  _ISR(tim5), // 50
  _ISR(spi3), // 51
  _ISR(uart4),
  _ISR(uart5),
  _ISR(tim6_dac),
  _ISR(tim7),
  _ISR(dma2_stream0), // 56
  _ISR(dma2_stream1),
  _ISR(dma2_stream2),
  _ISR(dma2_stream3),
  _ISR(dma2_stream4), // 60
  _ISR(eth),
  _ISR(eth_wkup),
  _ISR(can2_tx), // 63
  mcu_core_default_isr,
  mcu_core_default_isr,
  mcu_core_default_isr,
  mcu_core_default_isr,
  _ISR(dma2_stream5), // 68
  _ISR(dma2_stream6),
  _ISR(dma2_stream7), // 70
  _ISR(usart6),
  _ISR(i2c3_ev),
  _ISR(i2c3_er), // 73
  _ISR(otg_hs_ep1_out),
  _ISR(otg_hs_ep1_in),
  _ISR(otg_hs_wkup),
  _ISR(otg_hs),
  _ISR(dcmi), // 78
  _ISR(cryp),
  _ISR(hash),           // 80
  _ISR(fpu),            // 81
  _ISR(uart7),          // 82
  _ISR(uart8),          // 83
  _ISR(spi4),           // 84
  _ISR(spi5),           // 85
  _ISR(spi6),           // 86
  _ISR(sai1),           // 87
  _ISR(ltdc),           // 88
  _ISR(ltdc_er),        // 89
  _ISR(dma2d),          // 90
  _ISR(sai2),           // 91
  _ISR(quadspi),        // 92
  _ISR(lptim1),         // 93
  _ISR(cec),            // 94
  _ISR(i2c4_ev),        // 95
  _ISR(i2c4_er),        // 96
  _ISR(spdif),          // 97
  _ISR(otg_fs_ep1_out), // 98
  _ISR(otg_fs_ep1_in),  // 99
  _ISR(otg_fs_wkup),    // 100
  _ISR(otg_fs),         // 101
  _ISR(dmamux1),        // 102
  _ISR(hrtim1_master),  // 103
  _ISR(hrtim1_tima),    // 104
  _ISR(hrtim1_timb),    // 105
  _ISR(hrtim1_timc),    // 106
  _ISR(hrtim1_timd),    // 107
  _ISR(hrtim1_time),    // 108
  _ISR(hrtim1_flt),     // 109
  _ISR(dfsdm1_flt0),    // 110
  _ISR(dfsdm1_flt1),    // 111
  _ISR(dfsdm1_flt2),    // 112
  _ISR(dfsdm1_flt3),    // 113
  _ISR(sai3),           // 114
  _ISR(swpwm1),         // 115
  _ISR(tim15),          // 116
  _ISR(tim16),          // 117
  _ISR(tim17),          // 118
  _ISR(mdios_wkup),     // 119
  _ISR(mdios),          // 120
  _ISR(jpeg),           // 121
  _ISR(mdma),           // 122
  mcu_core_default_isr,
  _ISR(sdmmc2), // 124
  _ISR(hsem1),  // 125
  mcu_core_default_isr,
  _ISR(adc3),          // 127
  _ISR(dmamux2),       // 128
  _ISR(bdma_channel0), // 129
  _ISR(bdma_channel1), // 130
  _ISR(bdma_channel2), // 131
  _ISR(bdma_channel3), // 132
  _ISR(bdma_channel4), // 133
  _ISR(bdma_channel5), // 134
  _ISR(bdma_channel6), // 135
  _ISR(bdma_channel7), // 136
  _ISR(comp),          // 137
  _ISR(lptim2),        // 138
  _ISR(lptim3),        // 139
  _ISR(lptim4),        // 140
  _ISR(lptim5),        // 141
  _ISR(lpuart1),       // 142
  mcu_core_default_isr,
  _ISR(crs), // 144
  mcu_core_default_isr,
  _ISR(sai4), // 146
  mcu_core_default_isr,
  mcu_core_default_isr,
  _ISR(wakup_pin), // 149
};

void mcu_core_reset_handler() {

}

void mcu_core_default_isr() { sos_handle_event(SOS_EVENT_ROOT_FATAL, "dflt"); }
