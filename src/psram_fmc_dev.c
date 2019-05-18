
#include <errno.h>
#include <fcntl.h>
#include <mcu/emc.h>
#include <mcu/debug.h>
#include <mcu/core.h>
#include <mcu/pio.h>
#include <cortexm/cortexm.h>

#include "stm32_local.h"

#if MCU_FMC_PORTS > 0

typedef struct{
  volatile u16 reg;
  volatile u16 ram;
}ahb_control_t;

typedef struct MCU_PACK {
    SRAM_HandleTypeDef hal_handle;
    FMC_NORSRAM_TimingTypeDef timing;
	mcu_event_handler_t event_handler;
    u32 ref_count;
    ahb_control_t * lcd_control;
} ram_local_t;

static ram_local_t m_ram_local[MCU_FMC_PORTS] MCU_SYS_MEM;
static u8 const psram_irqs[MCU_FMC_PORTS] = MCU_FMC_IRQS;
static int mcu_emc_ram_close(const devfs_handle_t * handle);
static int mcu_emc_ram_getinfo(const devfs_handle_t * handle, void * ctl);
static int mcu_emc_ram_open(const devfs_handle_t * handle);
static int mcu_emc_ram_setattr(const devfs_handle_t * handle, void * ctl);
DEVFS_MCU_DRIVER_IOCTL_FUNCTION_MIN(emc_psram, EMC_VERSION, EMC_IOC_IDENT_CHAR)
DEVFS_MCU_DRIVER_IOCTL_FUNCTION_MIN(emc_fmc_ahb, EMC_VERSION, EMC_IOC_IDENT_CHAR)


int mcu_emc_fmc_ahb_open(const devfs_handle_t * handle){
    return mcu_emc_ram_open(handle);
}
int mcu_emc_psram_open(const devfs_handle_t * handle){
    return mcu_emc_ram_open(handle);
}
/**
 * @brief mcu_emc_ram_open
 * @param handle
 * @return
 */
static int mcu_emc_ram_open(const devfs_handle_t * handle){
    DEVFS_DRIVER_DECLARE_LOCAL(ram, MCU_FMC_PORTS);
    if ( local->ref_count == 0 ){
        switch(port){
            case 0:
            case 1:
                __HAL_RCC_FMC_CLK_ENABLE();
                break;
        }
        local->hal_handle.Instance = FMC_NORSRAM_DEVICE;
        local->hal_handle.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
    }
    local->ref_count++;
    return 0;
}

int mcu_emc_fmc_ahb_close(const devfs_handle_t * handle){
    return mcu_emc_ram_close(handle);
}

int mcu_emc_psram_close(const devfs_handle_t * handle){
    return mcu_emc_ram_close(handle);
}
/**
 * @brief mcu_emc_ram_close use from both psrma and lcd
 * @param handle
 * @return
 */
int mcu_emc_ram_close(const devfs_handle_t * handle){
    DEVFS_DRIVER_DECLARE_LOCAL(ram, MCU_FMC_PORTS);
    if ( local->ref_count > 0 ){
        if ( local->ref_count == 1 ){
            local->hal_handle.Instance = 0;
            switch(port){
                case 0:
                case 1:
                    __HAL_RCC_FMC_CLK_DISABLE();
                    break;
            }
        }
        local->ref_count--;
    }
    return 0;
}
/**
 * @brief mcu_emc_psram_getinfo for psram use
 * @param handle
 * @param ctl
 * @return
 */
int mcu_emc_psram_getinfo(const devfs_handle_t * handle, void * ctl){
    return mcu_emc_ram_getinfo(handle, ctl);
}
/**
 * @brief mcu_emc_lcd_getinfo
 * @param handle
 * @param ctl
 * @return
 */
int mcu_emc_fmc_ahb_getinfo(const devfs_handle_t * handle, void * ctl){
    return mcu_emc_ram_getinfo(handle, ctl);
}
/**
 * @brief mcu_emc_ram_getinfo use from psram and lcd
 * @param handle
 * @param ctl
 * @return
 */
static int mcu_emc_ram_getinfo(const devfs_handle_t * handle, void * ctl){
    DEVFS_DRIVER_DECLARE_LOCAL(ram, MCU_FMC_PORTS);
    emc_info_t * info = ctl;
    if( handle->config == 0 ){
        return SYSFS_SET_RETURN(ENOSYS);
    }
    const emc_attr_t * config = handle->config;
    info->o_flags = EMC_FLAG_DISABLE | EMC_FLAG_ENABLE | EMC_FLAG_IS_SDRAM |\
            EMC_FLAG_IS_PSRAM | EMC_FLAG_IS_SRAM | EMC_FLAG_IS_NOR |\
            EMC_FLAG_IS_NAND | EMC_FLAG_IS_8B_ACCESS | EMC_FLAG_IS_16B_ACCESS |\
            EMC_FLAG_IS_32B_ACCESS | EMC_FLAG_IS_AHB | EMC_FLAG_IS_PSRAM_BANK1 |\
            EMC_FLAG_IS_PSRAM_BANK2 | EMC_FLAG_IS_PSRAM_BANK3 | EMC_FLAG_IS_PSRAM_BANK4 ;
    info->freq = config->freq;
    info->o_events =  MCU_EVENT_FLAG_ERROR;
    info->base_address = config->base_address;
    info->size = config->size;
    return 0;
}

int mcu_emc_fmc_ahb_setattr(const devfs_handle_t * handle, void * ctl){
    return mcu_emc_ram_setattr(handle, ctl);
}

int mcu_emc_psram_setattr(const devfs_handle_t * handle, void * ctl){
    return mcu_emc_ram_setattr(handle, ctl);
}
/**
 * @brief mcu_emc_ram_setattr use pcram and lcd
 * lcd using same pin description without 3 pins TE RESET and BL(back light)
 * @param handle
 * @param ctl
 * @return
 */
static int mcu_emc_ram_setattr(const devfs_handle_t * handle, void * ctl){
    DEVFS_DRIVER_DECLARE_LOCAL(ram, MCU_FMC_PORTS);
	const emc_attr_t * attr = mcu_select_attr(handle, ctl);
	if( attr == 0 ){
        mcu_debug_printf("mcu_emc_psram_setattr attr nll \n");
		return SYSFS_SET_RETURN(ENOSYS);
	}
	u32 o_flags = attr->o_flags;
    if( o_flags & EMC_FLAG_ENABLE ){
        mcu_debug_printf("mcu_emc_psram_setattr enable \n");
        if( mcu_set_pin_assignment(
                 &(attr->pin_assignment),
                 MCU_CONFIG_PIN_ASSIGNMENT(emc_config_t, handle),
                 MCU_PIN_ASSIGNMENT_COUNT(emc_pin_assignment_t),
                 CORE_PERIPH_EMC, handle->port, 0, 0, 0) < 0 ){
            mcu_debug_printf("pin assignment failed %s:%d\n",__FILE__,__LINE__);
            return SYSFS_SET_RETURN(EINVAL);
        }
        /* SRAM device configuration */
        local->hal_handle.Instance = FMC_NORSRAM_DEVICE;
        local->hal_handle.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
        /* Timing configuration derived from system clock (up to 200Mhz)
           for 100Mhz as SRAM clock frequency */
        local->timing.AddressSetupTime      = 9;
        local->timing.AddressHoldTime       = 2;
        local->timing.DataSetupTime         = 6;
        local->timing.BusTurnAroundDuration = 1;
        local->timing.CLKDivision           = 2;
        local->timing.DataLatency           = 2;
        local->timing.AccessMode            = FMC_ACCESS_MODE_A;
#if defined STM32F723xx
        if(o_flags & EMC_FLAG_IS_PSRAM_BANK1){
            local->lcd_control = (ahb_control_t*)FMC_PSRAM_BANK1_BASE;
            local->hal_handle.Init.NSBank             = FMC_NORSRAM_BANK1;
        }else if(o_flags & EMC_FLAG_IS_PSRAM_BANK2){
            local->lcd_control = (ahb_control_t*)FMC_PSRAM_BANK2_BASE;
            local->hal_handle.Init.NSBank             = FMC_NORSRAM_BANK2;
        }else if(o_flags & EMC_FLAG_IS_PSRAM_BANK3){
            local->lcd_control = (ahb_control_t*)FMC_PSRAM_BANK3_BASE;
            local->hal_handle.Init.NSBank             = FMC_NORSRAM_BANK3;
        }else if(o_flags & EMC_FLAG_IS_PSRAM_BANK4){
            local->lcd_control = (ahb_control_t*)FMC_PSRAM_BANK4_BASE;
            local->hal_handle.Init.NSBank             = FMC_NORSRAM_BANK4;
        }else{
            local->lcd_control = (ahb_control_t*)FMC_PSRAM_BANK1_BASE;
            local->hal_handle.Init.NSBank             = FMC_NORSRAM_BANK1;
        }
#else
        local->lcd_control = (ahb_control_t*)((uint32_t)(0x60000000));
        local->hal_handle.Init.NSBank             = FMC_NORSRAM_BANK1;
#endif
        mcu_debug_printf("lcd pointer %p \n",local->lcd_control);
        local->hal_handle.Init.DataAddressMux     = FMC_DATA_ADDRESS_MUX_DISABLE;
        local->hal_handle.Init.MemoryType         = FMC_MEMORY_TYPE_SRAM;
        local->hal_handle.Init.MemoryDataWidth    = attr->data_bus_width;
        local->hal_handle.Init.BurstAccessMode    = FMC_BURST_ACCESS_MODE_DISABLE;
        local->hal_handle.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
        local->hal_handle.Init.WaitSignalActive   = FMC_WAIT_TIMING_BEFORE_WS;
        local->hal_handle.Init.WriteOperation     = FMC_WRITE_OPERATION_ENABLE;
        local->hal_handle.Init.WaitSignal         = FMC_WAIT_SIGNAL_DISABLE;
        local->hal_handle.Init.ExtendedMode       = FMC_EXTENDED_MODE_DISABLE;
        local->hal_handle.Init.AsynchronousWait   = FMC_ASYNCHRONOUS_WAIT_DISABLE;
        local->hal_handle.Init.WriteBurst         = FMC_WRITE_BURST_DISABLE;
        local->hal_handle.Init.ContinuousClock    = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
        /* Initialize the SRAM controller */
        if(HAL_SRAM_Init(&local->hal_handle, &local->timing, &local->timing) != HAL_OK){
            mcu_debug_printf("sram init failed %s:%d\n",__FILE__,__LINE__);
        }
        //power up
	}
    if( o_flags & EMC_FLAG_IS_AHB){
        if(o_flags & EMC_FLAG_AHB_WRITE_DATA){
            local->lcd_control->ram = attr->data_or_reg;
            __DSB();
        }else if(o_flags & EMC_FLAG_AHB_WRITE_REG){
            local->lcd_control->reg = attr->data_or_reg;
            __DSB();
        }
    }
	if( o_flags & EMC_FLAG_DISABLE ){
		//power down
	}
	return SYSFS_RETURN_SUCCESS;
}
int mcu_emc_fmc_ahb_setaction(const devfs_handle_t * handle, void * ctl){
    DEVFS_DRIVER_DECLARE_LOCAL(ram, MCU_FMC_PORTS);
    //assign event handler to catch errors
    return SYSFS_SET_RETURN(ENOTSUP);
}
int mcu_emc_psram_setaction(const devfs_handle_t * handle, void * ctl){
    DEVFS_DRIVER_DECLARE_LOCAL(ram, MCU_FMC_PORTS);
	//assign event handler to catch errors
	return SYSFS_SET_RETURN(ENOTSUP);
}
/**
 * @brief mcu_emc_lcd_read read regs throuth FMC AHB interface
 * @param handle
 * @param async
 * @return
 */
int mcu_emc_fmc_ahb_read(const devfs_handle_t * handle, devfs_async_t * async){
    DEVFS_DRIVER_DECLARE_LOCAL(ram, MCU_FMC_PORTS);
    mcu_debug_printf("mcu_emc_fmc_ahb_read\n");
    if(local->lcd_control!=NULL && async->buf!=NULL){
        u16 * lcd_reg = async->buf;
        * lcd_reg = local->lcd_control->ram;
        return 2;
    }else{
        return 0;
    }
        //memory is mapped
    return async->nbyte;
}

int mcu_emc_psram_read(const devfs_handle_t * handle, devfs_async_t * async){
    ram_local_t * psram = m_ram_local + handle->port * sizeof(ram_local_t);
    const emc_attr_t * config = handle->config;
    if( config == 0 ){ return SYSFS_SET_RETURN(ENOSYS); }
    if( async->loc >= (int)config->size ){
        mcu_debug_printf("async->loc >= config->size %u %u \n",async->loc ,config->size );
        return SYSFS_RETURN_EOF;
    }
    if( async->loc + async->nbyte > config->size ){
        async->nbyte = config->size - async->loc;
    }
    if( async->loc & 0x03 || async->nbyte & 0x03 ){
        HAL_SRAM_Read_8b(&(psram->hal_handle), (u32*)(config->base_address + async->loc), async->buf, async->nbyte);
    } else {
        HAL_SRAM_Read_32b(&psram->hal_handle, (u32*)(config->base_address + async->loc), async->buf, async->nbyte/4);
    }

    //memory is mapped
    return async->nbyte;
}
/**
 * @brief mcu_emc_lcd_write not used instead use SET_ATTRIBUTE
 * @param handle
 * @param async
 * @return
 */
int mcu_emc_fmc_ahb_write(const devfs_handle_t * handle, devfs_async_t * async){
    DEVFS_DRIVER_DECLARE_LOCAL(ram, MCU_FMC_PORTS);
    u16 * data;
    data = async->buf;
    local->lcd_control->ram = *data;
    __DSB();
    return 2;
}

int mcu_emc_psram_write(const devfs_handle_t * handle, devfs_async_t * async){
    DEVFS_DRIVER_DECLARE_LOCAL(ram, MCU_FMC_PORTS);
    const emc_attr_t * config = handle->config;
	if( config == 0 ){ return SYSFS_SET_RETURN(ENOSYS); }
    if( async->loc >= (int)config->size ){
        mcu_debug_printf("async->loc >= config->size %u %u \n",async->loc ,config->size );
		return SYSFS_RETURN_EOF;
	}
    if( (async->loc + async->nbyte) > (int)config->size ){
        mcu_debug_printf("async->loc >= config->size %u %u \n",async->loc ,config->size );
		async->nbyte = config->size - async->loc;
	}
    if( async->loc & 0x03 || async->nbyte & 0x03 ){
        HAL_SRAM_Write_8b(&local->hal_handle, (u32*)(config->base_address + async->loc), async->buf, async->nbyte);
	} else {
        HAL_SRAM_Write_32b(&local->hal_handle, (u32*)(config->base_address + async->loc), async->buf, async->nbyte/4);
	}
	//memory is mapped
	return async->nbyte;
}


void HAL_SRAM_RefreshErrorCallback(SRAM_HandleTypeDef *hpsram){
    ram_local_t * local = (ram_local_t *)hpsram;
	MCU_UNUSED_ARGUMENT(local);
	mcu_debug_printf("refresh error\n");
}

void mcu_core_fmc_isr(){
}


#endif
