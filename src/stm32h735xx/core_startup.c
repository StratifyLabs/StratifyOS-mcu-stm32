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
#include <sos/events.h>
#include <sos/symbols.h>

#include "core/core_startup.h"
#include "stm32_local.h"

void mcu_core_default_isr();

// ISR's -- weakly bound to default handler
_DECLARE_ISR(wwdg); //0
_DECLARE_ISR(pvd_avd); //1
_DECLARE_ISR(tamp_stamp); //2
_DECLARE_ISR(rtc_wkup); //3
_DECLARE_ISR(flash); //4
_DECLARE_ISR(rcc); //5
_DECLARE_ISR(exti0); //6
_DECLARE_ISR(exti1); //7
_DECLARE_ISR(exti2); //8
_DECLARE_ISR(exti3); //9
_DECLARE_ISR(exti4); //10
_DECLARE_ISR(dma1_stream0); //11
_DECLARE_ISR(dma1_stream1); //12
_DECLARE_ISR(dma1_stream2); //13
_DECLARE_ISR(dma1_stream3); //14
_DECLARE_ISR(dma1_stream4); //15
_DECLARE_ISR(dma1_stream5); //16
_DECLARE_ISR(dma1_stream6); //17
_DECLARE_ISR(adc); //18
_DECLARE_ISR(fdcan1_it0); //19
_DECLARE_ISR(fdcan2_it0); //20
_DECLARE_ISR(fdcan1_it1); //21
_DECLARE_ISR(fdcan2_it1); //22
_DECLARE_ISR(exti9_5); //23
_DECLARE_ISR(tim1_brk); //24
_DECLARE_ISR(tim1_up); //25
_DECLARE_ISR(tim1_trg_com); //26
_DECLARE_ISR(tim1_cc); //27
_DECLARE_ISR(tim2); //28
_DECLARE_ISR(tim3); //29
_DECLARE_ISR(tim4); //30
_DECLARE_ISR(i2c1_ev); //31
_DECLARE_ISR(i2c1_er); //32
_DECLARE_ISR(i2c2_ev); //33
_DECLARE_ISR(i2c2_er); //34
_DECLARE_ISR(spi1); //35
_DECLARE_ISR(spi2); //36
_DECLARE_ISR(usart1); //37
_DECLARE_ISR(usart2); //38
_DECLARE_ISR(usart3); //39
_DECLARE_ISR(exti15_10); //40
_DECLARE_ISR(rtc_alarm); //41
_DECLARE_ISR(tim8_brk_tim12); //43
_DECLARE_ISR(tim8_up_tim13); //44
_DECLARE_ISR(tim8_trg_com_tim14); //45
_DECLARE_ISR(tim8_cc); //46
_DECLARE_ISR(dma1_stream7); //47
_DECLARE_ISR(fmc); //48
_DECLARE_ISR(sdmmc1); //49
_DECLARE_ISR(tim5); //50
_DECLARE_ISR(spi3); //51
_DECLARE_ISR(uart4); //52
_DECLARE_ISR(uart5); //53
_DECLARE_ISR(tim6_dac); //54
_DECLARE_ISR(tim7); //55
_DECLARE_ISR(dma2_stream0); //56
_DECLARE_ISR(dma2_stream1); //57
_DECLARE_ISR(dma2_stream2); //58
_DECLARE_ISR(dma2_stream3); //59
_DECLARE_ISR(dma2_stream4); //60
_DECLARE_ISR(eth); //61
_DECLARE_ISR(eth_wkup); //62
_DECLARE_ISR(fdcan_cal); //63
_DECLARE_ISR(dma2_stream5); //68
_DECLARE_ISR(dma2_stream6); //69
_DECLARE_ISR(dma2_stream7); //70
_DECLARE_ISR(usart6); //71
_DECLARE_ISR(i2c3_ev); //72
_DECLARE_ISR(i2c3_er); //73
_DECLARE_ISR(otg_hs_ep1_out); //74
_DECLARE_ISR(otg_hs_ep1_in); //75
_DECLARE_ISR(otg_hs_wkup); //76
_DECLARE_ISR(otg_hs); //77
_DECLARE_ISR(dcmi_pssi); //78
_DECLARE_ISR(cryp); //79
_DECLARE_ISR(hash_rng); //80
_DECLARE_ISR(fpu); //81
_DECLARE_ISR(uart7); //82
_DECLARE_ISR(uart8); //83
_DECLARE_ISR(spi4); //84
_DECLARE_ISR(spi5); //85
_DECLARE_ISR(spi6); //86
_DECLARE_ISR(sai1); //87
_DECLARE_ISR(ltdc); //88
_DECLARE_ISR(ltdc_er); //89
_DECLARE_ISR(dma2d); //90
_DECLARE_ISR(octospi1); //92
_DECLARE_ISR(lptim1); //93
_DECLARE_ISR(cec); //94
_DECLARE_ISR(i2c4_ev); //95
_DECLARE_ISR(i2c4_er); //96
_DECLARE_ISR(spdif_rx); //97
_DECLARE_ISR(dmamux1_ovr); //102
_DECLARE_ISR(dfsdm1_flt0); //110
_DECLARE_ISR(dfsdm1_flt1); //111
_DECLARE_ISR(dfsdm1_flt2); //112
_DECLARE_ISR(dfsdm1_flt3); //113
_DECLARE_ISR(swpmi1); //115
_DECLARE_ISR(tim15); //116
_DECLARE_ISR(tim16); //117
_DECLARE_ISR(tim17); //118
_DECLARE_ISR(mdios_wkup); //119
_DECLARE_ISR(mdios); //120
_DECLARE_ISR(mdma); //122
_DECLARE_ISR(sdmmc2); //124
_DECLARE_ISR(hsem1); //125
_DECLARE_ISR(adc3); //127
_DECLARE_ISR(dmamux2_ovr); //128
_DECLARE_ISR(bdma_channel0); //129
_DECLARE_ISR(bdma_channel1); //130
_DECLARE_ISR(bdma_channel2); //131
_DECLARE_ISR(bdma_channel3); //132
_DECLARE_ISR(bdma_channel4); //133
_DECLARE_ISR(bdma_channel5); //134
_DECLARE_ISR(bdma_channel6); //135
_DECLARE_ISR(bdma_channel7); //136
_DECLARE_ISR(comp); //137
_DECLARE_ISR(lptim2); //138
_DECLARE_ISR(lptim3); //139
_DECLARE_ISR(lptim4); //140
_DECLARE_ISR(lptim5); //141
_DECLARE_ISR(lpuart1); //142
_DECLARE_ISR(crs); //144
_DECLARE_ISR(ecc); //145
_DECLARE_ISR(sai4); //146
_DECLARE_ISR(dts); //147
_DECLARE_ISR(wakeup_pin); //149
_DECLARE_ISR(octospi2); //150
_DECLARE_ISR(otfdec1); //151
_DECLARE_ISR(otfdec2); //152
_DECLARE_ISR(fmac); //153
_DECLARE_ISR(cordic); //154
_DECLARE_ISR(uart9); //155
_DECLARE_ISR(usart10); //156
_DECLARE_ISR(i2c5_ev); //157
_DECLARE_ISR(i2c5_er); //158
_DECLARE_ISR(fdcan3_it0); //159
_DECLARE_ISR(fdcan3_it1); //160
_DECLARE_ISR(tim23); //161
_DECLARE_ISR(tim24); //162

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

  _ISR(wwdg), //0
  _ISR(pvd_avd), //1
  _ISR(tamp_stamp), //2
  _ISR(rtc_wkup), //3
  _ISR(flash), //4
  _ISR(rcc), //5
  _ISR(exti0), //6
  _ISR(exti1), //7
  _ISR(exti2), //8
  _ISR(exti3), //9
  _ISR(exti4), //10
  _ISR(dma1_stream0), //11
  _ISR(dma1_stream1), //12
  _ISR(dma1_stream2), //13
  _ISR(dma1_stream3), //14
  _ISR(dma1_stream4), //15
  _ISR(dma1_stream5), //16
  _ISR(dma1_stream6), //17
  _ISR(adc), //18
  _ISR(fdcan1_it0), //19
  _ISR(fdcan2_it0), //20
  _ISR(fdcan1_it1), //21
  _ISR(fdcan2_it1), //22
  _ISR(exti9_5), //23
  _ISR(tim1_brk), //24
  _ISR(tim1_up), //25
  _ISR(tim1_trg_com), //26
  _ISR(tim1_cc), //27
  _ISR(tim2), //28
  _ISR(tim3), //29
  _ISR(tim4), //30
  _ISR(i2c1_ev), //31
  _ISR(i2c1_er), //32
  _ISR(i2c2_ev), //33
  _ISR(i2c2_er), //34
  _ISR(spi1), //35
  _ISR(spi2), //36
  _ISR(usart1), //37
  _ISR(usart2), //38
  _ISR(usart3), //39
  _ISR(exti15_10), //40
  _ISR(rtc_alarm), //41
  mcu_core_default_isr, // 42
  _ISR(tim8_brk_tim12), //43
  _ISR(tim8_up_tim13), //44
  _ISR(tim8_trg_com_tim14), //45
  _ISR(tim8_cc), //46
  _ISR(dma1_stream7), //47
  _ISR(fmc), //48
  _ISR(sdmmc1), //49
  _ISR(tim5), //50
  _ISR(spi3), //51
  _ISR(uart4), //52
  _ISR(uart5), //53
  _ISR(tim6_dac), //54
  _ISR(tim7), //55
  _ISR(dma2_stream0), //56
  _ISR(dma2_stream1), //57
  _ISR(dma2_stream2), //58
  _ISR(dma2_stream3), //59
  _ISR(dma2_stream4), //60
  _ISR(eth), //61
  _ISR(eth_wkup), //62
  _ISR(fdcan_cal), //63
  mcu_core_default_isr, // 64
  mcu_core_default_isr, // 65
  mcu_core_default_isr, // 66
  mcu_core_default_isr, // 67
  _ISR(dma2_stream5), //68
  _ISR(dma2_stream6), //69
  _ISR(dma2_stream7), //70
  _ISR(usart6), //71
  _ISR(i2c3_ev), //72
  _ISR(i2c3_er), //73
  _ISR(otg_hs_ep1_out), //74
  _ISR(otg_hs_ep1_in), //75
  _ISR(otg_hs_wkup), //76
  _ISR(otg_hs), //77
  _ISR(dcmi_pssi), //78
  _ISR(cryp), //79
  _ISR(hash_rng), //80
  _ISR(fpu), //81
  _ISR(uart7), //82
  _ISR(uart8), //83
  _ISR(spi4), //84
  _ISR(spi5), //85
  _ISR(spi6), //86
  _ISR(sai1), //87
  _ISR(ltdc), //88
  _ISR(ltdc_er), //89
  _ISR(dma2d), //90
  mcu_core_default_isr, // 91
  _ISR(octospi1), //92
  _ISR(lptim1), //93
  _ISR(cec), //94
  _ISR(i2c4_ev), //95
  _ISR(i2c4_er), //96
  _ISR(spdif_rx), //97
  mcu_core_default_isr, // 98
  mcu_core_default_isr, // 99
  mcu_core_default_isr, // 100
  mcu_core_default_isr, // 101
  _ISR(dmamux1_ovr), //102
  mcu_core_default_isr, // 103
  mcu_core_default_isr, // 104
  mcu_core_default_isr, // 105
  mcu_core_default_isr, // 106
  mcu_core_default_isr, // 107
  mcu_core_default_isr, // 108
  mcu_core_default_isr, // 109
  _ISR(dfsdm1_flt0), //110
  _ISR(dfsdm1_flt1), //111
  _ISR(dfsdm1_flt2), //112
  _ISR(dfsdm1_flt3), //113
  mcu_core_default_isr, // 114
  _ISR(swpmi1), //115
  _ISR(tim15), //116
  _ISR(tim16), //117
  _ISR(tim17), //118
  _ISR(mdios_wkup), //119
  _ISR(mdios), //120
  mcu_core_default_isr, // 121
  _ISR(mdma), //122
  mcu_core_default_isr, // 123
  _ISR(sdmmc2), //124
  _ISR(hsem1), //125
  mcu_core_default_isr, // 126
  _ISR(adc3), //127
  _ISR(dmamux2_ovr), //128
  _ISR(bdma_channel0), //129
  _ISR(bdma_channel1), //130
  _ISR(bdma_channel2), //131
  _ISR(bdma_channel3), //132
  _ISR(bdma_channel4), //133
  _ISR(bdma_channel5), //134
  _ISR(bdma_channel6), //135
  _ISR(bdma_channel7), //136
  _ISR(comp), //137
  _ISR(lptim2), //138
  _ISR(lptim3), //139
  _ISR(lptim4), //140
  _ISR(lptim5), //141
  _ISR(lpuart1), //142
  mcu_core_default_isr, // 143
  _ISR(crs), //144
  _ISR(ecc), //145
  _ISR(sai4), //146
  _ISR(dts), //147
  mcu_core_default_isr, // 148
  _ISR(wakeup_pin), //149
  _ISR(octospi2), //150
  _ISR(otfdec1), //151
  _ISR(otfdec2), //152
  _ISR(fmac), //153
  _ISR(cordic), //154
  _ISR(uart9), //155
  _ISR(usart10), //156
  _ISR(i2c5_ev), //157
  _ISR(i2c5_er), //158
  _ISR(fdcan3_it0), //159
  _ISR(fdcan3_it1), //160
  _ISR(tim23), //161
  _ISR(tim24), //162
};

void mcu_core_default_isr() { sos_handle_event(SOS_EVENT_ROOT_FATAL, "dflt"); }
