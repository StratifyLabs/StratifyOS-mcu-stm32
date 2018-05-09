/* Copyright 2011-2017 Tyler Gilbert;
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
 * along with Stratify OS.  If not, see <http://www.gnu.org/licenses/>. */


//#include "config.h"

#include <mcu/bootloader.h>
#include "stm32_flash.h"


static u32 mcu_core_get_reset_src();
static int enable_clock_out(int o_flags, int div);
static u32 mcu_core_reset_source = CORE_FLAG_IS_RESET_SYSTEM;



int mcu_core_setpinfunc(const devfs_handle_t * handle, void * arg){
    core_pinfunc_t * argp = arg;
    return mcu_core_set_pinsel_func(&(argp->io),
                                    argp->periph_func,
                                    argp->periph_port);
}

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(core, CORE_VERSION, I_MCU_TOTAL + I_CORE_TOTAL, mcu_core_setpinfunc, mcu_core_setclkout, mcu_core_setclkdivide, mcu_core_getmcuboardconfig)

int mcu_core_open(const devfs_handle_t * handle){ return 0; }
int mcu_core_close(const devfs_handle_t * handle){ return 0; }

int mcu_core_getinfo(const devfs_handle_t * handle, void * arg){
    core_info_t * info = arg;
    info->o_flags = 0;
    info->freq = mcu_board_config.core_cpu_freq;
    if( mcu_core_reset_source == CORE_FLAG_IS_RESET_SYSTEM ){
        mcu_core_reset_source = mcu_core_get_reset_src();
    }

    info->o_flags |= mcu_core_reset_source;
    return 0;
}

int mcu_core_setattr(const devfs_handle_t * handle, void * arg){
    int port = handle->port;
    core_attr_t * attr = arg;

    if( attr == 0 ){
        return SYSFS_SET_RETURN(EINVAL);
    }

    u32 o_flags = attr->o_flags;


    if( o_flags & CORE_FLAG_SET_CLKOUT ){
        enable_clock_out(o_flags, attr->freq);
    }

    if( o_flags & CORE_FLAG_EXEC_RESET ){
        mcu_core_reset(port, 0);
    }

    if( o_flags & CORE_FLAG_EXEC_INVOKE_BOOTLOADER ){
        mcu_core_invokebootloader(port, 0);
    }

    if( o_flags & CORE_FLAG_EXEC_SLEEP ){
        mcu_core_execsleep(port, (void*)CORE_SLEEP);
    } else if( o_flags & CORE_FLAG_EXEC_DEEPSLEEP ){
        mcu_core_execsleep(port, (void*)CORE_DEEPSLEEP);
    } else if( o_flags & CORE_FLAG_EXEC_DEEPSLEEP_STOP ){
        mcu_core_execsleep(port, (void*)CORE_DEEPSLEEP_STOP);
    } else if( o_flags & CORE_FLAG_EXEC_DEEPSLEEP_STANDBY ){
        mcu_core_execsleep(port, (void*)CORE_DEEPSLEEP_STANDBY);
    }


    return 0;
}

int mcu_core_setaction(const devfs_handle_t * handle, void * arg){
    return SYSFS_SET_RETURN(ENOTSUP);
}

int mcu_core_execsleep(int port, void * arg){
    int level;
    level = (int)arg;

    mcu_set_sleep_mode(&level);
    if ( level < 0 ){
        return level;
    }

    //Wait for an interrupts
    __WFI();
    return 0;
}

int mcu_core_reset(int port, void * arg){
    //delay first
    cortexm_delay_us(20*1000);
    cortexm_reset(NULL);
    //doesn't arrive here
    return 0;
}

int mcu_core_invokebootloader(int port, void * arg){
    cortexm_delay_us(500*1000);
    bootloader_api_t api;
    mcu_core_get_bootloader_api(&api);
    api.exec(0);
    return 0;
}


int mcu_core_setclkout(const devfs_handle_t * handle, void * arg){
    //core_clkout_t * clkout = arg;
    //return
    return 0;
}

int mcu_core_setclkdivide(const devfs_handle_t * handle, void * arg){

#ifdef __lpc17xx
    //the errata on the LPC17xx chips prevent this from working correctly
    return SYSFS_SET_RETURN(ENOTSUP);
#endif

#ifdef LPCXX7X_8X
    uint32_t div = (int)arg;
    uint32_t clksel;

    if( (div >= 1) && (div <= 31) ){
        clksel = LPC_SC->CCLKSEL & ~0x1F;
        clksel |= div;
        LPC_SC->CCLKSEL = clksel;
    } else {
        return SYSFS_SET_RETURN(EINVAL);
    }
#endif

    return 0;
}

int mcu_core_getmcuboardconfig(const devfs_handle_t * handle, void * arg){
    memcpy(arg, &mcu_board_config, sizeof(mcu_board_config));
    return 0;
}


void mcu_set_sleep_mode(int * level){
    SCB->SCR &= ~(1<<SCB_SCR_SLEEPDEEP_Pos);

#if defined LPC_SC
    LPC_SC->PCON = 0;
    switch(*level){
    case CORE_DEEPSLEEP_STOP:
        LPC_SC->PCON = 1; //turn off the flash as well
        //no break
    case CORE_DEEPSLEEP:
        SCB->SCR |= (1<<SCB_SCR_SLEEPDEEP_Pos);
        break;
    case CORE_SLEEP:
        break;
    case CORE_DEEPSLEEP_STANDBY:
        SCB->SCR |= (1<<SCB_SCR_SLEEPDEEP_Pos);
        LPC_SC->PCON = 3;
        break;
    default:
        *level = -1;
        return;
    }
    *level = 0;
#endif
}

u32 mcu_core_get_reset_src(){
    u32 src = CORE_FLAG_IS_RESET_SYSTEM;
    u32 src_reg = RCC->CSR;
    RCC->CSR |= RCC_CSR_RMVF; //clear flags

#if defined RCC_CSR_BORRSTF
    if ( src_reg & RCC_CSR_BORRSTF ){
        return CORE_FLAG_IS_RESET_BOR;
    }
#endif

    if ( src_reg & (RCC_CSR_IWDGRSTF|RCC_CSR_WWDGRSTF) ){
        return CORE_FLAG_IS_RESET_WDT;
    }

    if ( src_reg & RCC_CSR_PORRSTF ){
        return CORE_FLAG_IS_RESET_POR;
    }

    if( src_reg & RCC_CSR_SFTRSTF ){
        return CORE_FLAG_IS_RESET_SOFTWARE;
    }

    if ( src_reg & RCC_CSR_PINRSTF ){
        return CORE_FLAG_IS_RESET_EXTERNAL;
    }

    return src;
}

int enable_clock_out(int o_flags, int div){

    return 0;

}

void mcu_core_set_nvic_priority(int irq, int prio){
    NVIC_SetPriority((IRQn_Type)irq, prio);
}

void mcu_core_get_bootloader_api(void * args){
    void * ptr;
    u32 * value = (u32*)(FLASH_START + 36);

    if( *value != 0 ){
        memcpy(&ptr, (void*)(FLASH_START + 36), sizeof(void*)); //get pointer to boot api
        memcpy(args, ptr, sizeof(bootloader_api_t)); //copy boot api
    } else {
        memset(args, 0, sizeof(bootloader_api_t));
    }
}

int mcu_core_write(const devfs_handle_t * cfg, devfs_async_t * async){
    return SYSFS_SET_RETURN(ENOTSUP);
}

int mcu_core_read(const devfs_handle_t * cfg, devfs_async_t * async){
    return SYSFS_SET_RETURN(ENOTSUP);
}

//--------------------------------CACHE Operations--------------------------------

#if defined STM32F7
#define USE_CACHE 1
#endif

void mcu_core_enable_cache(){
#if USE_CACHE
    if( mcu_board_config.o_flags & MCU_BOARD_CONFIG_FLAG_ENABLE_CACHE ){
        SCB_EnableICache();
        SCB_EnableDCache();
    }
#endif
}

void mcu_core_disable_cache(){
#if USE_CACHE
    if( mcu_board_config.o_flags & MCU_BOARD_CONFIG_FLAG_ENABLE_CACHE ){
        SCB_DisableICache();
        SCB_DisableDCache();
    }
#endif
}

void mcu_core_invalidate_instruction_cache(){
#if USE_CACHE
    if( mcu_board_config.o_flags & MCU_BOARD_CONFIG_FLAG_ENABLE_CACHE ){
        SCB_InvalidateICache();
    }
#endif
}

void mcu_core_clean_data_cache(){
#if USE_CACHE
    if( mcu_board_config.o_flags & MCU_BOARD_CONFIG_FLAG_ENABLE_CACHE ){
        SCB_CleanDCache();
    }
#endif
}

void mcu_core_invalidate_data_cache(){
#if USE_CACHE
    if( mcu_board_config.o_flags & MCU_BOARD_CONFIG_FLAG_ENABLE_CACHE ){
        SCB_InvalidateDCache();
    }
#endif
}

void mcu_core_clean_data_cache_block(void * addr, u32 size){
#if USE_CACHE
    if( mcu_board_config.o_flags & MCU_BOARD_CONFIG_FLAG_ENABLE_CACHE ){
        SCB_CleanDCache_by_Addr(addr, size);
    }
#endif
}

void mcu_core_invalidate_data_cache_block(void * addr, u32 size){
#if USE_CACHE
    if( mcu_board_config.o_flags & MCU_BOARD_CONFIG_FLAG_ENABLE_CACHE ){
        SCB_InvalidateDCache_by_Addr(addr, size);
    }
#endif
}


