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

#include <fcntl.h>

#include <cortexm/cortexm.h>
#include <mcu/crc.h>
#include <mcu/debug.h>

#include "stm32_local.h"


#if MCU_CRC_PORTS > 0

typedef struct {
	CRC_HandleTypeDef hal_handle;
	u32 value;
	u8 ref_count;
} crc_local_t;

static crc_local_t m_crc_local[MCU_CRC_PORTS] MCU_SYS_MEM;
CRC_TypeDef * const crc_regs_table[MCU_CRC_PORTS] = MCU_CRC_REGS;
s8 const crc_irqs[MCU_CRC_PORTS] = MCU_CRC_IRQS;

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(crc, CRC_VERSION, CRC_IOC_IDENT_CHAR, I_MCU_TOTAL + I_CRC_TOTAL, mcu_crc_get)


u32 mcu_calc_crc32(u32 seed, u32 polynomial, const u8 * buffer, u32 nbyte){
	crc_local_t * local = m_crc_local;

#if MCU_CRC_API == 1
	local->hal_handle.Instance = CRC;
	local->hal_handle.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
	local->hal_handle.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
	local->hal_handle.Init.InitValue = seed;
	local->hal_handle.Init.GeneratingPolynomial = polynomial;
	local->hal_handle.Init.CRCLength = CRC_POLYLENGTH_32B;
	local->hal_handle.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
	local->hal_handle.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
	local->hal_handle.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
#endif
	HAL_CRC_Init(&local->hal_handle);

	return HAL_CRC_Accumulate(&local->hal_handle, (u32*)buffer, nbyte);
}

u16 mcu_calc_crc16(u16 seed, u16 polynomial, const u8 * buffer, u32 nbyte){
	crc_local_t * local = m_crc_local;

#if MCU_CRC_API == 1
	local->hal_handle.Instance = CRC;
	local->hal_handle.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
	local->hal_handle.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
	local->hal_handle.Init.InitValue = seed;
	local->hal_handle.Init.GeneratingPolynomial = polynomial;
	local->hal_handle.Init.CRCLength = CRC_POLYLENGTH_16B;
	local->hal_handle.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
	local->hal_handle.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
	local->hal_handle.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
#endif
	HAL_CRC_Init(&local->hal_handle);

	return HAL_CRC_Accumulate(&local->hal_handle, (u32*)buffer, nbyte);
}

u8 mcu_calc_crc7(u8 seed, u8 polynomial, const u8 * chr, u32 len){
	int i,a;
	unsigned char crc,data;

	//this can be done in hardware on STM32
	crc=seed;
	for(a=0; a<len; a++){
		data=chr[a];
		for (i=0;i<8;i++) {
			crc <<= 1;
			if ((data & 0x80)^(crc & 0x80)){
				crc ^=0x09;
			}
			data <<= 1;
		}
	}
	crc=(crc<<1)|1;
	return(crc);
}

int mcu_crc_open(const devfs_handle_t * handle){
	DEVFS_DRIVER_DECLARE_LOCAL(crc, MCU_CRC_PORTS);
	if ( local->ref_count == 0 ){

		local->hal_handle.Instance = crc_regs_table[port];

		switch(port){
			case 0:
				__HAL_RCC_CRC_CLK_ENABLE();
				break;
		}

		if( crc_irqs[port] > 0 ){
			cortexm_enable_irq(crc_irqs[port]);
		}

	}
	m_crc_local[port].ref_count++;

	return 0;
}

int mcu_crc_close(const devfs_handle_t * handle){
	DEVFS_DRIVER_DECLARE_LOCAL(crc, MCU_CRC_PORTS);
	if ( local->ref_count > 0 ){
		if ( local->ref_count == 1 ){
			if( crc_irqs[port] > 0 ){
				cortexm_disable_irq(crc_irqs[port]);
			}
			switch(port){
				case 0:
					__HAL_RCC_CRC_CLK_DISABLE();
					break;
			}
			local->hal_handle.Instance = 0;
		}
		local->ref_count--;
	}
	return 0;
}


int mcu_crc_getinfo(const devfs_handle_t * handle, void * ctl){
	crc_info_t * info = ctl;
	memset(info, 0, sizeof(crc_info_t));
	info->o_flags = CRC_FLAG_ENABLE | CRC_FLAG_DISABLE | CRC_FLAG_IS_32BIT;
	info->polynomial = 0x4C11DB7;
	return 0;
}

int mcu_crc_setattr(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(crc, MCU_CRC_PORTS);
	const crc_attr_t * attr = mcu_select_attr(handle, ctl);
	if( attr == 0 ){ return SYSFS_SET_RETURN(ENOSYS); }

	u32 o_flags = attr->o_flags;

	if( o_flags & CRC_FLAG_ENABLE ){
#if MCU_CRC_API == 1
		local->hal_handle.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
		local->hal_handle.Init.GeneratingPolynomial = attr->seed;
		if( o_flags & CRC_FLAG_IS_DEFAULT_POLYNOMIAL ){
			local->hal_handle.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
		}

		local->hal_handle.Init.DefaultPolynomialUse = DEFAULT_INIT_VALUE_DISABLE;
		local->hal_handle.Init.InitValue = attr->initial_value;
		if( o_flags & CRC_FLAG_IS_DEFAULT_INTIAL_VALUE ){
			local->hal_handle.Init.DefaultPolynomialUse = DEFAULT_INIT_VALUE_ENABLE;
		}

		local->hal_handle.Init.CRCLength = CRC_POLYLENGTH_32B;
		if( o_flags & CRC_FLAG_IS_7BIT ){
			local->hal_handle.Init.CRCLength = CRC_POLYLENGTH_7B;
		} else  if( o_flags & CRC_FLAG_IS_8BIT ){
			local->hal_handle.Init.CRCLength = CRC_POLYLENGTH_8B;
		} else if( o_flags & CRC_FLAG_IS_16BIT ){
			local->hal_handle.Init.CRCLength = CRC_POLYLENGTH_16B;
		}

		local->hal_handle.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
		if( o_flags & CRC_FLAG_IS_INVERT_INPUT_8BIT ){
			local->hal_handle.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_BYTE;
		} else  if( o_flags & CRC_FLAG_IS_INVERT_INPUT_16BIT ){
			local->hal_handle.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_HALFWORD;
		} else if( o_flags & CRC_FLAG_IS_INVERT_INPUT_32BIT ){
			local->hal_handle.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_WORD;
		}

		local->hal_handle.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
		if( o_flags & CRC_FLAG_IS_INVERT_OUTPUT ){
			local->hal_handle.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE;
		}
#endif

		local->value = 0UL;
		if( HAL_CRC_Init(&local->hal_handle) != HAL_OK ){
			return SYSFS_SET_RETURN(EIO);
		}
	}

	if( o_flags & CRC_FLAG_DISABLE ){
		local->value = 0UL;
		if( HAL_CRC_DeInit(&local->hal_handle) != HAL_OK ){
			return SYSFS_SET_RETURN(EIO);
		}
	}

	return SYSFS_RETURN_SUCCESS;
}


int mcu_crc_setaction(const devfs_handle_t * handle, void * ctl){
	return 0;
}

int mcu_crc_read(const devfs_handle_t * handle, devfs_async_t * async){
	int port = handle->port;
	int nbyte = async->nbyte;
	if( nbyte > 4 ){ nbyte = 4; }
	memcpy(async->buf, &m_crc_local[port].value, nbyte);
	return nbyte;
}

int mcu_crc_write(const devfs_handle_t * handle, devfs_async_t * async){
	int port = handle->port;
	int nbyte = async->nbyte / 4;
	if( async->loc == 0 ){
		m_crc_local[port].value = HAL_CRC_Calculate(&m_crc_local[port].hal_handle, async->buf, nbyte*4);
	} else {
		m_crc_local[port].value = HAL_CRC_Accumulate(&m_crc_local[port].hal_handle, async->buf, nbyte*4);
	}
	//when data is written - return the CRC value -- this is a problem if the value is negative
	return nbyte;
}

int mcu_crc_get(const devfs_handle_t * handle, void * arg){
	int port = handle->port;
	u32 * value = arg;
	*value = m_crc_local[port].value;
	return 0;
}



#endif
