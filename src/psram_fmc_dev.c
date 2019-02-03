
#include <errno.h>
#include <fcntl.h>
#include <mcu/emc.h>
#include <mcu/debug.h>
#include <mcu/core.h>
#include <mcu/pio.h>
#include <cortexm/cortexm.h>

#include "stm32_local.h"

#if MCU_FMC_PORTS > 0

typedef struct MCU_PACK {
    SRAM_HandleTypeDef hal_handle;
    FMC_NORSRAM_TimingTypeDef timing;
	mcu_event_handler_t event_handler;
    u32 ref_count;
} psram_local_t;

static psram_local_t m_psram_local[MCU_FMC_PORTS] MCU_SYS_MEM;
static u8 const psram_irqs[MCU_FMC_PORTS] = MCU_FMC_IRQS;

DEVFS_MCU_DRIVER_IOCTL_FUNCTION_MIN(emc_psram, EMC_VERSION, EMC_IOC_IDENT_CHAR)

int mcu_emc_psram_open(const devfs_handle_t * handle){
    DEVFS_DRIVER_DECLARE_LOCAL(psram, MCU_FMC_PORTS);
	if ( local->ref_count == 0 ){
		switch(port){
			case 0:
				__HAL_RCC_FMC_CLK_ENABLE();
				break;
		}
        local->hal_handle.Instance = FMC_NORSRAM_DEVICE;
        local->hal_handle.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
	}
	local->ref_count++;
	return 0;
}

int mcu_emc_psram_close(const devfs_handle_t * handle){
	DEVFS_DRIVER_DECLARE_LOCAL(psram, MCU_FMC_PORTS);
	if ( local->ref_count > 0 ){
		if ( local->ref_count == 1 ){
			local->hal_handle.Instance = 0;
			switch(port){
				case 0:
					__HAL_RCC_FMC_CLK_DISABLE();
					break;
			}
		}
		local->ref_count--;
	}
	return 0;
}

int mcu_emc_psram_getinfo(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(psram, MCU_FMC_PORTS);
	emc_info_t * info = ctl;
	if( handle->config == 0 ){
		return SYSFS_SET_RETURN(ENOSYS);
	}
    const emc_attr_t * config = handle->config;
	info->o_flags = 0;
	info->freq = config->freq;
	info->o_events =  MCU_EVENT_FLAG_ERROR;
	info->base_address = config->base_address;
	info->size = config->size;
	return 0;
}

int mcu_emc_psram_setattr(const devfs_handle_t * handle, void * ctl){
    psram_local_t * psram = m_psram_local + handle->port * sizeof(psram_local_t);
	const emc_attr_t * attr = mcu_select_attr(handle, ctl);
    mcu_debug_printf("mcu_emc_psram_setattr start\n");
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
        psram->hal_handle.Instance = FMC_NORSRAM_DEVICE;
        psram->hal_handle.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
        /* Timing configuration derived from system clock (up to 200Mhz)
           for 100Mhz as SRAM clock frequency */
        psram->timing.AddressSetupTime      = 9;
        psram->timing.AddressHoldTime       = 2;
        psram->timing.DataSetupTime         = 6;
        psram->timing.BusTurnAroundDuration = 1;
        psram->timing.CLKDivision           = 2;
        psram->timing.DataLatency           = 2;
        psram->timing.AccessMode            = FMC_ACCESS_MODE_A;
        psram->hal_handle.Init.NSBank             = FMC_NORSRAM_BANK1;
        psram->hal_handle.Init.DataAddressMux     = FMC_DATA_ADDRESS_MUX_DISABLE;
        psram->hal_handle.Init.MemoryType         = FMC_MEMORY_TYPE_SRAM;
        psram->hal_handle.Init.MemoryDataWidth    = attr->data_bus_width;
        psram->hal_handle.Init.BurstAccessMode    = FMC_BURST_ACCESS_MODE_DISABLE;
        psram->hal_handle.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
        psram->hal_handle.Init.WaitSignalActive   = FMC_WAIT_TIMING_BEFORE_WS;
        psram->hal_handle.Init.WriteOperation     = FMC_WRITE_OPERATION_ENABLE;
        psram->hal_handle.Init.WaitSignal         = FMC_WAIT_SIGNAL_DISABLE;
        psram->hal_handle.Init.ExtendedMode       = FMC_EXTENDED_MODE_DISABLE;
        psram->hal_handle.Init.AsynchronousWait   = FMC_ASYNCHRONOUS_WAIT_DISABLE;
        psram->hal_handle.Init.WriteBurst         = FMC_WRITE_BURST_DISABLE;
        psram->hal_handle.Init.ContinuousClock    = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
        /* Initialize the SRAM controller */
        if(HAL_SRAM_Init(&psram->hal_handle, &psram->timing, &psram->timing) != HAL_OK){
            mcu_debug_printf("sram init failed %s:%d\n",__FILE__,__LINE__);
        }
        //power up
	}
	if( o_flags & EMC_FLAG_DISABLE ){
		//power down
	}
	return SYSFS_RETURN_SUCCESS;
}

int mcu_emc_psram_setaction(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(psram, MCU_FMC_PORTS);

	//assign event handler to catch errors

	return SYSFS_SET_RETURN(ENOTSUP);
}
int mcu_emc_psram_read(const devfs_handle_t * handle, devfs_async_t * async){
    psram_local_t * psram = m_psram_local + handle->port * sizeof(psram_local_t);
    const emc_attr_t * config = handle->config;
    if( config == 0 ){ return SYSFS_SET_RETURN(ENOSYS); }
    if( async->loc >= config->size ){
        mcu_debug_printf("async->loc >= config->size\n");
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

int mcu_emc_psram_write(const devfs_handle_t * handle, devfs_async_t * async){
	DEVFS_DRIVER_DECLARE_LOCAL(psram, MCU_FMC_PORTS);
    const emc_attr_t * config = handle->config;
	if( config == 0 ){ return SYSFS_SET_RETURN(ENOSYS); }
	if( async->loc >= config->size ){
        mcu_debug_printf("async->loc >= config->size \n");
		return SYSFS_RETURN_EOF;
	}
	if( async->loc + async->nbyte > config->size ){
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
	psram_local_t * local = (psram_local_t *)hpsram;
	MCU_UNUSED_ARGUMENT(local);
	mcu_debug_printf("refresh error\n");
}

void mcu_core_fmc_isr(){
}


#endif
