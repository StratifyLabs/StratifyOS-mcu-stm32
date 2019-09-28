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

#include <mcu/crypt.h>
#include "crypt_local.h"

#if MCU_CRYPT_PORTS > 0

crypt_local_t m_crypt_local[MCU_CRYPT_PORTS] MCU_SYS_MEM;
CRYP_TypeDef * const crypt_regs[MCU_CRYPT_PORTS] = MCU_CRYPT_REGS;
u8 const crypt_irqs[MCU_CRYPT_PORTS] = MCU_CRYPT_IRQS;

int crypt_local_open(const devfs_handle_t * handle){
	DEVFS_DRIVER_DECLARE_LOCAL(crypt, MCU_CRYPT_PORTS);
	if( port < MCU_SPI_PORTS ){
		if ( local->ref_count == 0 ){
			//turn on RCC clock
			switch(port){
				case 0:
					__HAL_RCC_CRYP_CLK_ENABLE();
					break;
			}
			local->transfer_handler.read = NULL;
			local->transfer_handler.write = NULL;
			local->hal_handle.Instance = crypt_regs[port];
			mcu_debug_printf("enable irq %d\n", crypt_irqs[port]);
			cortexm_enable_irq(crypt_irqs[port]);
		}
		local->ref_count++;
		return 0;
	}

	return SYSFS_SET_RETURN(EINVAL);
}

int crypt_local_close(const devfs_handle_t * handle){
	DEVFS_DRIVER_DECLARE_LOCAL(crypt, MCU_CRYPT_PORTS);
	if ( local->ref_count > 0 ){
		if ( local->ref_count == 1 ){
			HAL_CRYP_DeInit(&local->hal_handle);
			cortexm_disable_irq(crypt_irqs[port]);
			devfs_execute_cancel_handler(&local->transfer_handler, 0, SYSFS_SET_RETURN(EDEADLK), MCU_EVENT_FLAG_CANCELED);
			//turn off RCC clock
			switch(port){
				case 0:
					__HAL_RCC_CRYP_CLK_DISABLE();
					break;
			}
		}
		local->ref_count--;
	}
	return 0;
}

int crypt_local_setattr(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(crypt, MCU_CRYPT_PORTS);
	const crypt_attr_t * attr = mcu_select_attr(handle, ctl);
	if( attr == 0 ){
		return SYSFS_SET_RETURN(EINVAL);
	}

	u32 o_flags = attr->o_flags;
	local->o_flags = 0;

	if( o_flags & CRYPT_FLAG_SET_CIPHER ){

		local->o_flags = o_flags;

		local->hal_handle.Init.DataType = CRYP_DATATYPE_32B;
		if( o_flags & CRYPT_FLAG_IS_DATA_1 ){
			local->hal_handle.Init.DataType = CRYP_DATATYPE_1B;
		} else if( o_flags & CRYPT_FLAG_IS_DATA_8 ){
			local->hal_handle.Init.DataType = CRYP_DATATYPE_8B;
		} else if( o_flags & CRYPT_FLAG_IS_DATA_16 ){
			local->hal_handle.Init.DataType = CRYP_DATATYPE_16B;
		}

		local->hal_handle.Init.KeySize = CRYP_KEYSIZE_128B;
		if( o_flags & CRYPT_FLAG_IS_AES_192 ){
			local->hal_handle.Init.KeySize = CRYP_KEYSIZE_192B;
		} else if( o_flags & CRYPT_FLAG_IS_AES_256 ){
			local->hal_handle.Init.KeySize = CRYP_KEYSIZE_256B;
		}

		//Key and initialization vector
		memcpy(local->key, attr->key, MAX_KEY_SIZE);
		memcpy(local->iv, attr->iv, MAX_IV_SIZE);
		local->hal_handle.Init.pKey = (u32*)local->key;
		local->hal_handle.Init.pInitVect = (u32*)local->iv;

		//Algorithm
		local->hal_handle.Init.Algorithm = CRYP_AES_CTR;
		if( o_flags & CRYPT_FLAG_IS_AES_CBC ){
			local->hal_handle.Init.Algorithm = CRYP_AES_CBC;
		}
		if( o_flags & CRYPT_FLAG_IS_AES_ECB ){
			local->hal_handle.Init.Algorithm = CRYP_AES_ECB;
		}

		//GCM and CCM for CRYPT units that support it (not all chips do)
#if defined CRYP_CR_ALGOMODE_AES_GCM
		if( o_flags & CRYPT_FLAG_IS_AES_GCM ){
			local->hal_handle.Init.Algorithm = CRYP_AES_GCM;
		}

		local->hal_handle.Init.B0 = 0;	// \todo This needs to be updated
		if( o_flags & CRYPT_FLAG_IS_AES_CCM ){
			local->hal_handle.Init.Algorithm = CRYP_AES_CCM;
		}
#else
		//only used with CCM
		local->hal_handle.Init.B0 = 0;
#endif

		//header buffer
		//header size
		local->hal_handle.Init.Header = (u32*)local->header;
		local->hal_handle.Init.HeaderSize = 0;

		//DataWidthUnit
		local->hal_handle.Init.DataWidthUnit = CRYP_DATAWIDTHUNIT_WORD;


		//init if not yet initialized
		if( HAL_CRYP_Init(&local->hal_handle) != HAL_OK ){
			return SYSFS_SET_RETURN(EIO);
		}
	}

	if( o_flags & CRYPT_FLAG_SET_MODE ){

		//assume encryption if decrypt is not specified
		local->o_flags |= CRYPT_FLAG_IS_ENCRYPT;
		local->o_flags &= ~CRYPT_FLAG_IS_DECRYPT;

		if( o_flags & CRYPT_FLAG_IS_DECRYPT ){
			local->o_flags |= CRYPT_FLAG_IS_DECRYPT;
			local->o_flags &= ~CRYPT_FLAG_IS_ENCRYPT;
		}
	}

	return SYSFS_RETURN_SUCCESS;
}

int crypt_local_setaction(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(crypt, MCU_CRYPT_PORTS);
	mcu_action_t * action = ctl;

	if( action->handler.callback == 0 ){
		devfs_execute_cancel_handler(&local->transfer_handler, 0, 0, 0);
	}

	//update the priority
	cortexm_set_irq_priority(crypt_irqs[port], action->prio, action->o_events);

	return SYSFS_RETURN_SUCCESS;
}


void HAL_CRYP_InCpltCallback(CRYP_HandleTypeDef *hcryp){
	crypt_local_t * local = (crypt_local_t *)hcryp;

	//execute the callbacks
	//devfs_execute_read_handler(&local->transfer_handler, 0, 0, MCU_EVENT_FLAG_DATA_READY);
	devfs_execute_write_handler(&local->transfer_handler, 0, 0, MCU_EVENT_FLAG_WRITE_COMPLETE);

}

void HAL_CRYP_OutCpltCallback(CRYP_HandleTypeDef *hcryp){
	crypt_local_t * local = (crypt_local_t *)hcryp;
	//execute the callbacks
	devfs_execute_read_handler(&local->transfer_handler, 0, 0, MCU_EVENT_FLAG_DATA_READY);
	//devfs_execute_write_handler(&local->transfer_handler, 0, 0, MCU_EVENT_FLAG_WRITE_COMPLETE);
	//update the IV once writing is complete
	*(uint32_t*)(hcryp->Init.pInitVect) = hcryp->Instance->IV0LR;
	*(uint32_t*)(hcryp->Init.pInitVect+1) = hcryp->Instance->IV0RR;
	*(uint32_t*)(hcryp->Init.pInitVect+2) = hcryp->Instance->IV1LR;
	*(uint32_t*)(hcryp->Init.pInitVect+3) = hcryp->Instance->IV1RR;
}

void HAL_CRYP_ErrorCallback(CRYP_HandleTypeDef *hcryp){
	crypt_local_t * local = (crypt_local_t *)hcryp;
	devfs_execute_cancel_handler(&local->transfer_handler, 0, 0, MCU_EVENT_FLAG_ERROR);
}

void mcu_core_cryp_isr(){
	crypt_local_t * local = m_crypt_local + 0;
	HAL_CRYP_IRQHandler(&local->hal_handle);
}


#endif

