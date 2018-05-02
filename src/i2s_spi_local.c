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

#include "i2s_spi_local.h"
#include <mcu/i2s.h>
#include <mcu/spi.h>

#if MCU_SPI_PORTS > 0


int i2s_spi_local_mute(const devfs_handle_t * handle, void * ctl){
    return 0;
}

int i2s_spi_local_unmute(const devfs_handle_t * handle, void * ctl){
    return 0;
}

int i2s_spi_local_setattr(spi_local_t * spi, const devfs_handle_t * handle, void * ctl){
    int port = handle->port;
    const i2s_attr_t * attr = mcu_select_attr(handle, ctl);
    if( attr == 0 ){
        return SYSFS_SET_RETURN(EINVAL);
    }

    int is_errata_required = 0;
    u32 o_flags = attr->o_flags;

    //set I2S Flags

    spi->is_i2s = 1;


    if( o_flags & (I2S_FLAG_SET_MASTER|I2S_FLAG_SET_SLAVE) ){
#if defined I2S_FULLDUPLEXMODE_ENABLE
        spi->i2s_hal_handle.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
#endif

        if( o_flags & I2S_FLAG_SET_SLAVE ){
            is_errata_required |= (1<<1);
            if( o_flags & I2S_FLAG_IS_TRANSMITTER ){
                spi->i2s_hal_handle.Init.Mode = I2S_MODE_SLAVE_TX;
                if( o_flags & I2S_FLAG_IS_RECEIVER ){
#if defined I2S_FULLDUPLEXMODE_ENABLE
                    spi->i2s_hal_handle.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
                    spi->is_full_duplex = 1;
#endif
                }
            } else if ( o_flags & I2S_FLAG_IS_RECEIVER ){
                spi->i2s_hal_handle.Init.Mode = I2S_MODE_SLAVE_RX;
            }
        } else {
            if( o_flags & I2S_FLAG_IS_TRANSMITTER ){
                spi->i2s_hal_handle.Init.Mode = I2S_MODE_MASTER_TX;
                if( o_flags & I2S_FLAG_IS_RECEIVER ){
#if defined I2S_FULLDUPLEXMODE_ENABLE
                    spi->i2s_hal_handle.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
                    spi->is_full_duplex = 1;
#endif
                }
            } else if ( o_flags & I2S_FLAG_IS_RECEIVER ){
                spi->i2s_hal_handle.Init.Mode = I2S_MODE_MASTER_RX;
            }
        }

        spi->i2s_hal_handle.Init.Standard = I2S_STANDARD_PHILIPS;
        if( o_flags & I2S_FLAG_IS_FORMAT_MSB ){
            spi->i2s_hal_handle.Init.Standard = I2S_STANDARD_MSB;
        } else if( o_flags & I2S_FLAG_IS_FORMAT_LSB ){
            spi->i2s_hal_handle.Init.Standard = I2S_STANDARD_LSB;
        } else if( o_flags & I2S_FLAG_IS_FORMAT_PCM_SHORT ){
            spi->i2s_hal_handle.Init.Standard = I2S_STANDARD_PCM_SHORT;
        } else if( o_flags & I2S_FLAG_IS_FORMAT_PCM_LONG ){
            spi->i2s_hal_handle.Init.Standard = I2S_STANDARD_PCM_LONG;
        } else {
            is_errata_required |= (1<<0);
        }

        spi->i2s_hal_handle.Init.DataFormat = I2S_DATAFORMAT_16B;
        if( o_flags & I2S_FLAG_IS_WIDTH_24 ){
            spi->i2s_hal_handle.Init.DataFormat = I2S_DATAFORMAT_24B;
        } else if( o_flags & I2S_FLAG_IS_WIDTH_32 ){
            spi->i2s_hal_handle.Init.DataFormat = I2S_DATAFORMAT_32B;
        } else if ( o_flags & I2S_FLAG_IS_WIDTH_16_EXTENDED ){
            spi->i2s_hal_handle.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
        }

        spi->i2s_hal_handle.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
        if( o_flags & I2S_FLAG_IS_MCK_ENABLED ){
            spi->i2s_hal_handle.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
        }

        spi->i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_8K;
        switch(attr->freq){
        case 11000: spi->i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_11K; break;
        case 16000: spi->i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_16K; break;
        case 22050: spi->i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_22K; break;
        case 32000: spi->i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_32K; break;
        case 44100: spi->i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_44K; break;
        case 48000: spi->i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_48K; break;
        case 96000: spi->i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_96K; break;
        case 192000: spi->i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_192K; break;
        default:
            return SYSFS_SET_RETURN(EINVAL);
        }

        spi->i2s_hal_handle.Init.CPOL = I2S_CPOL_LOW;
        spi->i2s_hal_handle.Init.ClockSource = I2S_CLOCK_PLL;

        //this might be better implemented in the "core" driver for controlling the clocks
#if defined RCC_PERIPHCLK_I2S
        RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
        PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
        PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK){
            return SYSFS_SET_RETURN(EIO);
        }
#endif

        //errata: http://www.st.com/content/ccc/resource/technical/document/errata_sheet/0a/98/58/84/86/b6/47/a2/DM00037591.pdf/files/DM00037591.pdf/jcr:content/translations/en.DM00037591.pdf
        if( is_errata_required & (1<<1) ){
            u32 pio_value;
            u32 pio_mask;
            u32 pio_level;
            u32 target_level;
            devfs_handle_t handle;
            handle.port = attr->pin_assignment.ws.port;
            pio_mask = 1<<attr->pin_assignment.ws.pin;

            //MSB Mode errata_required & (1<<0) == 0 -- wait until high then wait until low
            //I2S Mode errata_required & (1<<0) == 1 -- wait until low then wait until high

            target_level = (is_errata_required & (1<<0)) == 0; //MSB : 1, I2S: 0
            do {
                mcu_pio_get(&handle, &pio_value);
                pio_level = (pio_value & pio_mask) != 0; //1 for set, 0 for not
            } while( pio_level != target_level );

            target_level = (is_errata_required & (1<<0)) != 0; //MSB : 1, I2S: 1
            do {
                mcu_pio_get(&handle, &pio_value);
                pio_level = (pio_value & pio_mask) != 0; //1 for set, 0 for not
            } while( pio_level != target_level );

        }

        if( mcu_set_pin_assignment(
                    &(attr->pin_assignment),
                    MCU_CONFIG_PIN_ASSIGNMENT(i2s_config_t, handle),
                    MCU_PIN_ASSIGNMENT_COUNT(i2s_pin_assignment_t),
                    CORE_PERIPH_SPI, port, 0, 0, 0) < 0 ){
            return SYSFS_SET_RETURN(EINVAL);
        }

        if( HAL_I2S_Init(&spi->i2s_hal_handle) != HAL_OK ){
            return SYSFS_SET_RETURN(EIO);
        }
    }

    return 0;
}


#endif

