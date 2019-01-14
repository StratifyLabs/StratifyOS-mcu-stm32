


#include <errno.h>
#include <fcntl.h>
#include "stm32_local.h"
#include "cortexm/cortexm.h"
#include "mcu/i2c.h"
#include "mcu/debug.h"
#include "mcu/core.h"
#include "mcu/pio.h"



#if MCU_LTDC_PORTS > 10

typedef struct MCU_PACK {
	LTDC_HandleTypeDef hal_handle;
	devfs_transfer_handler_t transfer_handler;
} ltdc_local_t;

static ltdc_local_t ltdc_local[MCU_LTDC_PORTS] MCU_SYS_MEM;
static I2C_TypeDef * const ltdc_regs_table[MCU_LTDC_PORTS] = MCU_LTDC_REGS;
static u8 const ltdc_irqs[MCU_LTDC_PORTS] = MCU_LTDC_IRQS;
static u8 const ltdc_er_irqs[MCU_LTDC_PORTS] = MCU_LTDC_ER_IRQS;


DEVFS_MCU_DRIVER_IOCTL_FUNCTION_MIN(ltdc, I2C_VERSION, I2C_IOC_IDENT_CHAR)

int mcu_ltdc_open(const devfs_handle_t * handle){
	DEVFS_DRIVER_DECLARE_LOCAL(ltdc, MCU_LTDC_PORTS);
	if ( local->ref_count == 0 ){
		switch(port){
			case 0:
				__HAL_RCC_LTDC_CLK_ENABLE();
				break;
		}

		local->hal_handle.Instance = ltdc_regs_table[port];
		cortexm_enable_irq(ltdc_irqs[port]);
		cortexm_enable_irq(ltdc_er_irqs[port]);
	}
	local->ref_count++;
	return 0;
}

int mcu_ltdc_close(const devfs_handle_t * handle){
	DEVFS_DRIVER_DECLARE_LOCAL(ltdc, MCU_LTDC_PORTS);
	if ( local->ref_count > 0 ){
		if ( local->ref_count == 1 ){
			devfs_execute_cancel_handler(&local->transfer_handler, 0, SYSFS_SET_RETURN(EINTR), 0);
			local->hal_handle.Instance = 0;
			cortexm_disable_irq(ltdc_irqs[port]);
			cortexm_disable_irq(ltdc_er_irqs[port]);
			switch(port){
				case 0:
					__HAL_RCC_LTDC_CLK_DISABLE();
					break;
			}
		}
		i2c_local[port].ref_count--;
	}
	return 0;
}

int mcu_ltdc_getinfo(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(ltdc, MCU_LTDC_PORTS);
#if 0
	i2c_info_t * info = ctl;

	info->err = i2c_local[port].err;
	info->o_flags = I2C_FLAG_SET_MASTER |
			I2C_FLAG_SET_SLAVE |
			I2C_FLAG_PREPARE_PTR_DATA |
			I2C_FLAG_PREPARE_DATA |
			I2C_FLAG_IS_PULLUP |
			I2C_FLAG_IS_SLAVE_ADDR0 |
			I2C_FLAG_IS_SLAVE_ADDR1 |
			I2C_FLAG_IS_SLAVE_ADDR2 |
			I2C_FLAG_IS_SLAVE_ADDR3 |
			I2C_FLAG_RESET;

	info->freq = 400000;

	info->o_events = MCU_EVENT_FLAG_WRITE_COMPLETE |
			MCU_EVENT_FLAG_DATA_READY |
			MCU_EVENT_FLAG_CANCELED |
			MCU_EVENT_FLAG_ERROR;
#endif
	return 0;
}

int mcu_ltdc_setattr(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(ltdc, MCU_LTDC_PORTS);


	return -1;
}

int mcu_ltdc_setaction(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(ltdc, MCU_LTDC_PORTS);


	return -1;
}

#endif
