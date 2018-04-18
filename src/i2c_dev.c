/* Copyright 2011-2018 Tyler Gilbert;
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

#include <errno.h>
#include <fcntl.h>
#include "stm32_local.h"
#include "cortexm/cortexm.h"
#include "mcu/i2c.h"
#include "mcu/debug.h"
#include "mcu/core.h"
#include "mcu/pio.h"



#if MCU_I2C_PORTS > 0

typedef struct MCU_PACK {
    I2C_HandleTypeDef hal_handle;
    mcu_event_handler_t master;
    mcu_event_handler_t slave;
    void * slave_memory;
    u32 slave_memory_offset;
    u32 slave_memory_size;
    u32 o_flags;
    u16 slave_addr[2];
    u16 err;
    u16 ref_count;
    i2c_pin_assignment_t pin_assignment; //this is only needed because of the errata
} i2c_local_t;

static i2c_local_t i2c_local[MCU_I2C_PORTS] MCU_SYS_MEM;
static I2C_TypeDef * const i2c_regs_table[MCU_I2C_PORTS] = MCU_I2C_REGS;
static u8 const i2c_irqs[MCU_I2C_PORTS] = MCU_I2C_IRQS;
static u8 const i2c_er_irqs[MCU_I2C_PORTS] = MCU_I2C_ER_IRQS;

static void exec_master_callback(i2c_local_t * i2c, u32 o_events, u32 value);
static void exec_slave_callback(i2c_local_t * i2c, u32 o_events, u32 value);

static void i2c_clear_busy_flag_erratum(int port, i2c_local_t * i2c);

#if 0
typedef struct {
    u8 port;
    u8 is_pullup;
} post_configure_pin_t;
static void post_configure_pin(const mcu_pin_t * pin, void* arg);
#endif

DEVFS_MCU_DRIVER_IOCTL_FUNCTION_MIN(i2c, I2C_VERSION)

int mcu_i2c_open(const devfs_handle_t * handle){
    int port = handle->port;
    i2c_local_t * i2c = i2c_local + port;
    if ( i2c->ref_count == 0 ){
        switch(port){
        case 0:
            __HAL_RCC_I2C1_CLK_ENABLE();
            break;
#if MCU_I2C_PORTS > 1
        case 1:
            __HAL_RCC_I2C2_CLK_ENABLE();
            break;
#endif
#if MCU_I2C_PORTS > 2
        case 2:
            __HAL_RCC_I2C3_CLK_ENABLE();
            break;
#endif
        }
        i2c->master.callback = 0;
        i2c->hal_handle.Instance = i2c_regs_table[port];
        cortexm_enable_irq((void*)(u32)(i2c_irqs[port]));
        cortexm_enable_irq((void*)(u32)(i2c_er_irqs[port]));
    }
    i2c_local[port].ref_count++;
    return 0;
}

int mcu_i2c_close(const devfs_handle_t * handle){
    int port = handle->port;
    i2c_local_t * i2c = i2c_local + port;
    if ( i2c->ref_count > 0 ){
        if ( i2c->ref_count == 1 ){
            i2c->master.callback = 0;
            i2c->hal_handle.Instance = 0;
            cortexm_disable_irq((void*)(u32)(i2c_irqs[port]));
            cortexm_disable_irq((void*)(u32)(i2c_er_irqs[port]));
            switch(port){
            case 0:
                __HAL_RCC_I2C1_CLK_DISABLE();
                break;
#if MCU_I2C_PORTS > 1
            case 1:
                __HAL_RCC_I2C2_CLK_DISABLE();
                break;
#endif
#if MCU_I2C_PORTS > 2
            case 2:
                __HAL_RCC_I2C3_CLK_DISABLE();
                break;
#endif
            }
        }
        i2c_local[port].ref_count--;
    }
    return 0;
}

int mcu_i2c_getinfo(const devfs_handle_t * handle, void * ctl){
    int port = handle->port;
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
    return 0;
}

int mcu_i2c_setattr(const devfs_handle_t * handle, void * ctl){
    int port = handle->port;
    i2c_local_t * i2c = i2c_local + handle->port;
    u32 freq;


    const i2c_attr_t * attr = mcu_select_attr(handle, ctl);
    if( attr == 0 ){
        return SYSFS_SET_RETURN(EINVAL);
    }

    u32 o_flags = attr->o_flags;

    if( freq == 0 ){
        freq = 100000;
    }

    if( freq > 400000 ){
        freq = 400000;
    }

    if( o_flags & (I2C_FLAG_SET_MASTER | I2C_FLAG_SET_SLAVE) ){
        //Init init structure with defaults
        i2c->hal_handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
        i2c->hal_handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
        i2c->hal_handle.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
#if defined I2C_DUTYCYCLE_2
        i2c->hal_handle.Init.DutyCycle = I2C_DUTYCYCLE_2;
#endif
        i2c->hal_handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
        i2c->hal_handle.Init.OwnAddress1 = 0;
        i2c->hal_handle.Init.OwnAddress2 = 0;
    }

    if( o_flags & I2C_FLAG_SET_MASTER ){
#if defined STM32F7
        i2c->hal_handle.Init.Timing = freq;
#else
        i2c->hal_handle.Init.ClockSpeed = freq;
#endif

    } else if( o_flags & I2C_FLAG_SET_SLAVE ){

        i2c->hal_handle.Init.OwnAddress1 = (attr->slave_addr[0].addr8[0])<<1;
        if( o_flags & I2C_FLAG_IS_SLAVE_ADDR1 ){
            i2c->hal_handle.Init.DualAddressMode = I2C_DUALADDRESS_ENABLE;
            i2c->hal_handle.Init.OwnAddress2 = (attr->slave_addr[1].addr8[0])<<1;
        }

        if( o_flags & I2C_FLAG_IS_SLAVE_ACK_GENERAL_CALL ){
            i2c->hal_handle.Init.GeneralCallMode = I2C_GENERALCALL_ENABLE;
        }

        //set slave memory location
        i2c->slave_memory_offset = 0;
        i2c->slave_memory = attr->data;
        i2c->slave_memory_size = attr->size;
    }

    if( o_flags & (I2C_FLAG_SET_MASTER | I2C_FLAG_SET_SLAVE) ){
#if 0
        post_configure_pin_t post_configure;
#endif
        const void * pin_assignment;

        if( o_flags & I2C_FLAG_STRETCH_CLOCK ){
            i2c->hal_handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
        }

        if( freq < 100000 ){
#if defined I2C_DUTYCYCLE_16_9
            i2c->hal_handle.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
#endif
        }

        i2c->o_flags = o_flags;
#if 0
        post_configure.port = port;
        if( o_flags & I2C_FLAG_IS_PULLUP ){
            post_configure.is_pullup = 1;
        } else {
            post_configure.is_pullup = 0;
        }
#endif


        //force a start and stop condition to clear the busy bit
        pin_assignment = mcu_select_pin_assignment(&attr->pin_assignment,
                                                   MCU_CONFIG_PIN_ASSIGNMENT(i2c_config_t, handle),
                                                   MCU_PIN_ASSIGNMENT_COUNT(i2c_pin_assignment_t));

        memcpy(&i2c->pin_assignment, pin_assignment, sizeof(i2c_pin_assignment_t));

        if( (i2c->pin_assignment.scl.port != 0xff) && (i2c->pin_assignment.sda.port != 0xff) ){
            i2c_clear_busy_flag_erratum(port, i2c);
        } else {
            return SYSFS_SET_RETURN(EINVAL);
        }


        if( HAL_I2C_Init(&i2c->hal_handle) < 0 ){
            return SYSFS_SET_RETURN(EINVAL);
        }


        if( o_flags & I2C_FLAG_SET_SLAVE ){
            HAL_I2C_EnableListen_IT(&i2c->hal_handle);
        }

    }

    if( o_flags & (I2C_FLAG_PREPARE_PTR_DATA|I2C_FLAG_PREPARE_PTR|I2C_FLAG_PREPARE_DATA) ){
        i2c->o_flags = o_flags;
        i2c->slave_addr[0] = (attr->slave_addr[0].addr8[0]);
        i2c->slave_addr[1] = (attr->slave_addr[1].addr8[0]);
    }

    if( o_flags & I2C_FLAG_RESET ){
        //force a reset of the I2C
        i2c_clear_busy_flag_erratum(port, i2c);
    }

    return 0;
}

#if 0
void post_configure_pin(const mcu_pin_t * pin, void* arg){
    post_configure_pin_t * post_configure = arg;

    if( post_configure->is_pullup ){
        hal_set_alternate_pin_function(*pin,
                                       CORE_PERIPH_I2C,
                                       post_configure->port,
                                       GPIO_MODE_AF_OD,
                                       GPIO_SPEED_FREQ_LOW,
                                       GPIO_PULLUP);
    }


}
#endif


int mcu_i2c_geterr(const devfs_handle_t * handle, void * ctl){
    int port = handle->port;
    return i2c_local[port].err;
}

int mcu_i2c_setaction(const devfs_handle_t * handle, void * ctl){
    mcu_action_t * action = (mcu_action_t*)ctl;
    int port = handle->port;

    cortexm_set_irq_prio(i2c_irqs[port], action->prio);

    if( action->handler.callback == 0 ){
        //i2c_local[port].slave.handler.callback = 0;
        //i2c_local[port].slave.handler.context = 0;
        return 0;
    }

    if( cortexm_validate_callback(action->handler.callback) < 0 ){
        return -1;
    }

    //i2c_local[port].slave.handler.callback = action->handler.callback;
    //i2c_local[port].slave.handler.context = action->handler.context;

    return 0;
}

int mcu_i2c_write(const devfs_handle_t * handle, devfs_async_t * async){
    int ret;
    i2c_local_t * i2c = i2c_local + handle->port;
    int addr_size;

    if( i2c->master.callback ){
        return SYSFS_SET_RETURN(EBUSY);
    }

    i2c->master = async->handler;

    if( i2c->o_flags & I2C_FLAG_IS_PTR_16 ){
        addr_size = I2C_MEMADD_SIZE_16BIT;
    } else {
        addr_size = I2C_MEMADD_SIZE_8BIT;
    }

    if( i2c->o_flags & I2C_FLAG_PREPARE_PTR_DATA ){
        ret = HAL_I2C_Mem_Write_IT(&i2c->hal_handle, i2c->slave_addr[0]<<1, async->loc, addr_size, (u8*)async->buf, async->nbyte);
    } else if( i2c->o_flags & I2C_FLAG_PREPARE_DATA ){
        ret = HAL_I2C_Master_Transmit_IT(&i2c->hal_handle, i2c->slave_addr[0]<<1, async->buf, async->nbyte);
    } else {
        ret = -1;
    }

    if( ret == HAL_OK ){
        return 0;
    }

    return -1;
}

int mcu_i2c_read(const devfs_handle_t * handle, devfs_async_t * async){
    int ret = HAL_OK;
    i2c_local_t * i2c = i2c_local + handle->port;
    int addr_size;

    if( i2c->master.callback ){
        return SYSFS_SET_RETURN(EBUSY);
    }


    if( i2c->o_flags & I2C_FLAG_IS_PTR_16 ){
        addr_size = I2C_MEMADD_SIZE_16BIT;
    } else {
        addr_size = I2C_MEMADD_SIZE_8BIT;
    }

    if( i2c->o_flags & I2C_FLAG_PREPARE_PTR_DATA ){
        ret = HAL_I2C_Mem_Read_IT(&i2c->hal_handle, i2c->slave_addr[0]<<1, async->loc, addr_size, (u8*)async->buf, async->nbyte);
    } else if( i2c->o_flags & I2C_FLAG_PREPARE_DATA ){
        ret = HAL_I2C_Master_Receive_IT(&i2c->hal_handle, i2c->slave_addr[0]<<1, async->buf, async->nbyte);
    }

    if( ret == HAL_OK ){
        i2c->master = async->handler;
        return 0;
    } else {
        if( ret == HAL_TIMEOUT ){
            i2c->err = I2C_ERROR_TIMEOUT;
        }
    }

    return -1;
}

void exec_master_callback(i2c_local_t * i2c, u32 o_events, u32 value){
    i2c_event_t i2c_event;
    i2c_event.value = value;
    mcu_execute_event_handler(&i2c->master, o_events, &i2c_event);
}

void exec_slave_callback(i2c_local_t * i2c, u32 o_events, u32 value){
    i2c_event_t i2c_event;
    i2c_event.value = value;
    mcu_execute_event_handler(&i2c->master, o_events, &i2c_event);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c){
    i2c_local_t * i2c = (i2c_local_t*)hi2c;

    //TX complete
    exec_master_callback(i2c, MCU_EVENT_FLAG_WRITE_COMPLETE, 0);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c){
    i2c_local_t * i2c = (i2c_local_t*)hi2c;
    exec_master_callback(i2c, MCU_EVENT_FLAG_DATA_READY, 0);
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c){
    i2c_local_t * i2c = (i2c_local_t*)hi2c;
    exec_slave_callback(i2c, 0, 0);
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c){
    i2c_local_t * i2c = (i2c_local_t*)hi2c;
    exec_slave_callback(i2c, 0, 0);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode){
    //slave has been addressed
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c){
    //listen event
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c){
    i2c_local_t * i2c = (i2c_local_t*)hi2c;

    //TX complete
    exec_master_callback(i2c, MCU_EVENT_FLAG_WRITE_COMPLETE, 0);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
    i2c_local_t * i2c = (i2c_local_t*)hi2c;
    exec_master_callback(i2c, MCU_EVENT_FLAG_DATA_READY, 0);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c){
    i2c_local_t * i2c = (i2c_local_t*)hi2c;
    if( hi2c->ErrorCode & HAL_I2C_ERROR_ARLO ){
        i2c->err = I2C_ERROR_ARBITRATION_LOST;
    } else if ( hi2c->ErrorCode & HAL_I2C_ERROR_BERR ){
        i2c->err = I2C_ERROR_BUS_BUSY;
    } else if ( hi2c->ErrorCode & HAL_I2C_ERROR_AF ){
        i2c->err = I2C_ERROR_ACK;
    } else if ( hi2c->ErrorCode & HAL_I2C_ERROR_TIMEOUT ){
        i2c->err = I2C_ERROR_TIMEOUT;
    } else if ( hi2c->ErrorCode & HAL_I2C_ERROR_OVR ){
        i2c->err = I2C_ERROR_OVERFLOW;
    }

    if( hi2c->Mode == HAL_I2C_MODE_MASTER ){
        exec_master_callback(i2c, MCU_EVENT_FLAG_CANCELED | MCU_EVENT_FLAG_ERROR, 0);
    } else {
        //?
        exec_slave_callback(i2c, MCU_EVENT_FLAG_CANCELED | MCU_EVENT_FLAG_ERROR, 0);
    }
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c){
    i2c_local_t * i2c = (i2c_local_t*)hi2c;
    if( hi2c->Mode == HAL_I2C_MODE_MASTER ){
        exec_master_callback(i2c, MCU_EVENT_FLAG_CANCELED, 0);
    } else {
        //?
        exec_slave_callback(i2c, MCU_EVENT_FLAG_CANCELED, 0);
    }
}

static void mcu_i2c_ev_isr(int port) {
    i2c_local_t * i2c = i2c_local + port;
    HAL_I2C_EV_IRQHandler(&i2c->hal_handle);
}

static void mcu_i2c_er_isr(int port) {
    i2c_local_t * i2c = i2c_local + port;
    HAL_I2C_ER_IRQHandler(&i2c->hal_handle);
}

void mcu_core_i2c1_ev_isr(){ mcu_i2c_ev_isr(0); }
void mcu_core_i2c1_er_isr(){ mcu_i2c_er_isr(0); }
#if MCU_I2C_PORTS > 1
void mcu_core_i2c2_ev_isr(){ mcu_i2c_ev_isr(1); }
void mcu_core_i2c2_er_isr(){ mcu_i2c_er_isr(1); }
#endif
#if MCU_I2C_PORTS > 2
void mcu_core_i2c3_ev_isr(){ mcu_i2c_ev_isr(2); }
void mcu_core_i2c3_er_isr(){ mcu_i2c_er_isr(2); }
#endif

void i2c_clear_busy_flag_erratum(int port, i2c_local_t * i2c){

    devfs_handle_t sda_pio_handle;
    devfs_handle_t scl_pio_handle;
    pio_attr_t scl_pio_attr;
    pio_attr_t sda_pio_attr;
    u32 value;

    // 1. Clear PE bit.
    i2c->hal_handle.Instance->CR1 &= ~(0x0001);

    //  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    memset(&scl_pio_handle, 0, sizeof(devfs_handle_t));
    scl_pio_handle.port = i2c->pin_assignment.scl.port;
    scl_pio_attr.o_flags = PIO_FLAG_SET_OUTPUT | PIO_FLAG_IS_OPENDRAIN;
    scl_pio_attr.o_pinmask = (1 << i2c->pin_assignment.scl.pin);
    mcu_pio_setattr(&scl_pio_handle, &scl_pio_attr);

    memset(&sda_pio_handle, 0, sizeof(devfs_handle_t));
    sda_pio_handle.port = i2c->pin_assignment.sda.port;
    sda_pio_attr.o_flags = PIO_FLAG_SET_OUTPUT | PIO_FLAG_IS_OPENDRAIN;
    sda_pio_attr.o_pinmask = (1 << i2c->pin_assignment.sda.pin);
    mcu_pio_setattr(&sda_pio_handle, &sda_pio_attr);


    mcu_pio_setmask(&scl_pio_handle, (void*)scl_pio_attr.o_pinmask);
    mcu_pio_setmask(&sda_pio_handle, (void*)sda_pio_attr.o_pinmask);

    // 3. Check SCL and SDA High level in GPIOx_IDR.
    do {
        mcu_pio_get(&scl_pio_handle, &value);
    } while( (value & scl_pio_attr.o_pinmask) == 0);

    do {
        mcu_pio_get(&sda_pio_handle, &value);
    } while( (value & sda_pio_attr.o_pinmask) == 0);

    // 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    mcu_pio_clrmask(&sda_pio_handle, (void*)sda_pio_attr.o_pinmask);
    //  5. Check SDA Low level in GPIOx_IDR.
    do {
        mcu_pio_get(&sda_pio_handle, &value);
    } while( (value & sda_pio_attr.o_pinmask) != 0);
    // 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    mcu_pio_clrmask(&scl_pio_handle, (void*)scl_pio_attr.o_pinmask);
    //  7. Check SCL Low level in GPIOx_IDR.
    do {
        mcu_pio_get(&scl_pio_handle, &value);
    } while( (value & scl_pio_attr.o_pinmask) != 0);
    // 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    mcu_pio_setmask(&scl_pio_handle, (void*)scl_pio_attr.o_pinmask);
    // 9. Check SCL High level in GPIOx_IDR.
    do {
        mcu_pio_get(&scl_pio_handle, &value);
    } while( (value & scl_pio_attr.o_pinmask) == 0);
    // 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
    mcu_pio_setmask(&sda_pio_handle, (void*)sda_pio_attr.o_pinmask);
    // 11. Check SDA High level in GPIOx_IDR.
    do {
        mcu_pio_get(&sda_pio_handle, &value);
    } while( (value & sda_pio_attr.o_pinmask) == 0);


    // 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.

    hal_set_alternate_pin_function(i2c->pin_assignment.scl,
                                   CORE_PERIPH_I2C,
                                   port,
                                   GPIO_MODE_AF_OD,
                                   GPIO_SPEED_FREQ_LOW,
                                   GPIO_PULLUP);

    hal_set_alternate_pin_function(i2c->pin_assignment.sda,
                                   CORE_PERIPH_I2C,
                                   port,
                                   GPIO_MODE_AF_OD,
                                   GPIO_SPEED_FREQ_LOW,
                                   GPIO_PULLUP);


    // 13. Set SWRST bit in I2Cx_CR1 register.
    i2c->hal_handle.Instance->CR1 |= 0x8000;

    asm("nop");

    // 14. Clear SWRST bit in I2Cx_CR1 register.
    i2c->hal_handle.Instance->CR1 &= ~0x8000;

    asm("nop");

    // 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register
    i2c->hal_handle.Instance->CR1 |= 0x0001;
}

#endif

