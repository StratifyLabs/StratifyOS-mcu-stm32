
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
	SDRAM_HandleTypeDef hal_handle;
	mcu_event_handler_t event_handler;
	u8 ref_count;
} sdram_local_t;

static sdram_local_t m_sdram_local[MCU_FMC_PORTS] MCU_SYS_MEM;
static u8 const sdram_irqs[MCU_FMC_PORTS] = MCU_FMC_IRQS;


DEVFS_MCU_DRIVER_IOCTL_FUNCTION_MIN(emc_sdram, EMC_VERSION, EMC_IOC_IDENT_CHAR)

int mcu_emc_sdram_open(const devfs_handle_t * handle){
	DEVFS_DRIVER_DECLARE_LOCAL(sdram, MCU_FMC_PORTS);
	if ( local->ref_count == 0 ){
		switch(port){
			case 0:
				__HAL_RCC_FMC_CLK_ENABLE();
				break;
		}

		local->hal_handle.Instance = FMC_SDRAM_DEVICE;
		cortexm_enable_irq(sdram_irqs[port]);
	}
	local->ref_count++;
	return 0;
}

int mcu_emc_sdram_close(const devfs_handle_t * handle){
	DEVFS_DRIVER_DECLARE_LOCAL(sdram, MCU_FMC_PORTS);
	if ( local->ref_count > 0 ){
		if ( local->ref_count == 1 ){
			local->hal_handle.Instance = 0;
			cortexm_disable_irq(sdram_irqs[port]);
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

int mcu_emc_sdram_getinfo(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(sdram, MCU_FMC_PORTS);
	emc_info_t * info = ctl;
	if( handle->config == 0 ){
		return SYSFS_SET_RETURN(ENOSYS);
	}
	const emc_config_t * config = handle->config;
	info->o_flags = 0;
	info->freq = config->freq;
	info->o_events =  MCU_EVENT_FLAG_ERROR;
	info->base_address = config->base_address;
	info->size = config->size;
	return 0;
}

int mcu_emc_sdram_setattr(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(sdram, MCU_FMC_PORTS);

	const emc_attr_t * attr = mcu_select_attr(handle, ctl);
	if( attr == 0 ){
		return SYSFS_SET_RETURN(ENOSYS);
	}

	u32 o_flags = attr->o_flags;

	if( o_flags & EMC_FLAG_ENABLE ){
		//power up
	}

	if( o_flags & EMC_FLAG_DISABLE ){
		//power down
	}


	return SYSFS_RETURN_SUCCESS;
}

int mcu_emc_sdram_setaction(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(sdram, MCU_FMC_PORTS);

	//assign event handler to catch errors

	return SYSFS_SET_RETURN(ENOTSUP);
}

int mcu_emc_sdram_write(const devfs_handle_t * handle, devfs_async_t * async){
	DEVFS_DRIVER_DECLARE_LOCAL(sdram, MCU_FMC_PORTS);

	const emc_config_t * config = handle->config;
	if( config == 0 ){ return SYSFS_SET_RETURN(ENOSYS); }

	if( async->loc >= config->size ){
		return SYSFS_RETURN_EOF;
	}

	if( async->loc + async->nbyte > config->size ){
		async->nbyte = config->size - async->loc;
	}

	async->nbyte &= ~0x03; //word aligned

	HAL_SDRAM_Write_32b(&local->hal_handle, (u32*)(config->base_address + async->loc), async->buf, async->nbyte/4);

	//memory is mapped
	return async->nbyte;
}

int mcu_emc_sdram_read(const devfs_handle_t * handle, devfs_async_t * async){
	DEVFS_DRIVER_DECLARE_LOCAL(sdram, MCU_FMC_PORTS);

	const emc_config_t * config = handle->config;
	if( config == 0 ){ return SYSFS_SET_RETURN(ENOSYS); }

	if( async->loc >= config->size ){
		return SYSFS_RETURN_EOF;
	}

	if( async->loc + async->nbyte > config->size ){
		async->nbyte = config->size - async->loc;
	}

	async->nbyte &= ~0x03; //word aligned

	HAL_SDRAM_Read_32b(&local->hal_handle, (u32*)(config->base_address + async->loc), async->buf, async->nbyte/4);

	//memory is mapped
	return async->nbyte;
}

void HAL_SDRAM_RefreshErrorCallback(SDRAM_HandleTypeDef *hsdram){
	sdram_local_t * local = (sdram_local_t *)hsdram;
	MCU_UNUSED_ARGUMENT(local);
	mcu_debug_printf("refresh error\n");
}

void mcu_core_fmc_isr(){
	HAL_SDRAM_IRQHandler(&m_sdram_local[0].hal_handle);
}


#endif
