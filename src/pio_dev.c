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

#include "lpc_local.h"


#if MCU_PIO_PORTS > 0


typedef struct {
    mcu_event_handler_t handler;
    u8 ref_count;
} pio_local_t;

static pio_local_t m_pio0_local MCU_SYS_MEM;
static pio_local_t m_pio2_local MCU_SYS_MEM;

static int set_event(int port, int event, int pin);

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(pio, PIO_VERSION, I_MCU_TOTAL + I_PIO_TOTAL, mcu_pio_setmask, mcu_pio_clrmask, mcu_pio_get, mcu_pio_set)

int mcu_pio_open(const devfs_handle_t * handle){
    int port = handle->port;
    if ( port == 0 ){
        if ( m_pio0_local.ref_count == 0 ){
            m_pio0_local.handler.callback = NULL;
        }
        m_pio0_local.ref_count++;
    } else if ( port == 2 ){
        if ( m_pio2_local.ref_count == 0 ){
            m_pio2_local.handler.callback = NULL;
        }
        m_pio2_local.ref_count++;
    }
    return 0;
}

int mcu_pio_close(const devfs_handle_t * handle){
    int port = handle->port;

    if ( port == 0 ){
        if ( m_pio0_local.ref_count > 0 ){
            if ( m_pio0_local.ref_count == 1 ){
                m_pio0_local.handler.callback = NULL;
            }
            m_pio0_local.ref_count--;
        }
    } else if ( port == 2 ){
        if ( m_pio2_local.ref_count > 0 ){
            if ( m_pio2_local.ref_count == 1 ){
                m_pio2_local.handler.callback = NULL;
            }
            m_pio2_local.ref_count--;
        }
    }
    return 0;
}

int set_event(int port, int event, int pin){

    if ( port == 0 ){

#ifdef __lpc17xx
        if ( (pin > 30) ||
             (pin == 12) ||
             (pin == 13) ||
             (pin == 14) ){
            return SYSFS_SET_RETURN(EINVAL);
        }
#endif

        LPC_GPIOINT->IO0IntEnR &= ~(1<<pin);
        LPC_GPIOINT->IO0IntEnF &= ~(1<<pin);

        if( event & MCU_EVENT_FLAG_RISING ){
            LPC_GPIOINT->IO0IntEnR |= (1<<pin);
        }

        if( event & MCU_EVENT_FLAG_FALLING ){
            LPC_GPIOINT->IO0IntEnF |= (1<<pin);
        }


    } else if ( port == 2 ){

#ifdef __lpc17xx
        if ( (pin > 13) ){
            return SYSFS_SET_RETURN(EINVAL);
        }
#endif

        LPC_GPIOINT->IO2IntEnR &= ~(1<<pin);
        LPC_GPIOINT->IO2IntEnF &= ~(1<<pin);

        if( event & MCU_EVENT_FLAG_RISING ){
            LPC_GPIOINT->IO2IntEnR |= (1<<pin);
        }

        if( event & MCU_EVENT_FLAG_FALLING ){
            LPC_GPIOINT->IO2IntEnF |= (1<<pin);
        }
    }
    return 0;
}

int mcu_pio_read(const devfs_handle_t * handle, devfs_async_t * async){
    return SYSFS_SET_RETURN(ENOTSUP);
}

int mcu_pio_write(const devfs_handle_t * handle, devfs_async_t * async){
    mcu_action_t * action;

    if( async->nbyte != sizeof(mcu_action_t) ){
        return SYSFS_SET_RETURN(EINVAL);
    }

    action = (mcu_action_t*)async->buf;
    action->handler.callback = async->handler.callback;
    action->handler.context = async->handler.context;
    return mcu_pio_setaction(handle, action);
}

int mcu_pio_setaction(const devfs_handle_t * handle, void * ctl){
    int err;
    int port = handle->port;
    mcu_action_t * action = (mcu_action_t*)ctl;

    if( action->handler.callback == 0 ){
        if( port == 0 ){
            mcu_execute_event_handler(&(m_pio0_local.handler), MCU_EVENT_FLAG_CANCELED, 0);
        } else if ( port == 2 ){
            mcu_execute_event_handler(&(m_pio2_local.handler), MCU_EVENT_FLAG_CANCELED, 0);
        }
    }

    err = set_event(port, action->o_events, action->channel);
    if ( err ){
        return err;
    }

    if ( port == 0 ){
        if( cortexm_validate_callback(action->handler.callback) < 0 ){
            return SYSFS_SET_RETURN(EPERM);
        }

        m_pio0_local.handler.callback = action->handler.callback;
        m_pio0_local.handler.context = action->handler.context;
        LPC_GPIOINT->IO0IntClr = -1; //clear pending interrupts
    } else if ( port == 2 ){
        if( cortexm_validate_callback(action->handler.callback) < 0 ){
            return SYSFS_SET_RETURN(EPERM);
        }

        m_pio2_local.handler.callback = action->handler.callback;
        m_pio2_local.handler.context = action->handler.context;
        LPC_GPIOINT->IO2IntClr = -1; //clear pending interrupts
    } else {
        return SYSFS_SET_RETURN(EINVAL);
    }


#ifdef LPCXX7X_8X
    //This is the interrupt for GPIO0 and GPIO2
    cortexm_enable_irq(GPIO_IRQn);
    cortexm_set_irq_priority(GPIO_IRQn, action->prio);
#else
    //This is the interrupt for GPIO0 and GPIO2 (shared with EINT3)
    cortexm_enable_irq(EINT3_IRQn);
    cortexm_set_irq_priority(EINT3_IRQn, action->prio);
#endif

    return 0;
}


int mcu_pio_getinfo(const devfs_handle_t * handle, void * ctl){
    //read the direction pin status
    return SYSFS_SET_RETURN(ENOTSUP);
}

int mcu_pio_setattr(const devfs_handle_t * handle, void * ctl){
    int i;
    int mode;
    int port = handle->port;

    const pio_attr_t * attr = mcu_select_attr(handle, ctl);
    if( attr == 0 ){
        return SYSFS_SET_RETURN(EINVAL);
    }

#ifdef LPCXX7X_8X
    __IO u32 * regs_iocon;
#else
    int shift;
    __IO u32 * regs_pin;
    __IO u32 * regs_od = &(LPC_PINCON->PINMODE_OD0);
#endif
    LPC_GPIO_Type * gpio_regs = (LPC_GPIO_Type *)LPC_GPIO0_BASE + port;

    if( attr->o_flags & PIO_FLAG_SET_INPUT ){
        gpio_regs->DIR &= ~(attr->o_pinmask);
    }

    if( attr->o_flags & (PIO_FLAG_SET_OUTPUT) ){
        //set output pins as output
        gpio_regs->DIR |= attr->o_pinmask;
    }

    if( attr->o_flags & PIO_FLAG_IS_DIRONLY ){
        return 0;
    }

    mode = 0;


#ifdef LPCXX7X_8X

    if( attr->o_flags & PIO_FLAG_IS_PULLUP ){
        mode = 2;
    } else if( attr->o_flags & PIO_FLAG_IS_REPEATER ){
        mode = 3;
    } else if( attr->o_flags & PIO_FLAG_IS_FLOAT ){
        mode = 0;
    } else if ( attr->o_flags & PIO_FLAG_IS_PULLDOWN ){
        mode = 1;
    }

    for(i = 0; i < 32; i++){
        if ( (1<<i) & (attr->o_pinmask) ){
            regs_iocon = (u32*)LPC_IOCON + port*32 + i;
            mcu_pin_t pin = mcu_pin(port, i);
            mcu_core_set_pinsel_func(&pin, CORE_PERIPH_PIO, port); //set the pin to use GPIO

            if( attr->o_flags & PIO_FLAG_IS_OPENDRAIN ){
                *regs_iocon |= (1<<10);
            } else {
                *regs_iocon &= ~(1<<10);
            }

            if( attr->o_flags & PIO_FLAG_IS_HYSTERESIS ){
                *regs_iocon |= (1<<5);
            } else {
                *regs_iocon &= ~(1<<5);
            }

            if( (attr->o_flags & PIO_FLAG_IS_ANALOG) == 0 ){
                *regs_iocon |= (1<<7);
            } else {
                *regs_iocon &= ~(1<<7);
            }

            if( (attr->o_flags & (PIO_FLAG_IS_SPEED_BLAZING|PIO_FLAG_IS_SPEED_HIGH) ) != 0 ){
                *regs_iocon |= (1<<9);
            } else {
                *regs_iocon &= ~(1<<9);
            }

            *regs_iocon &= ~(0x03 << 3);
            *regs_iocon |= (mode << 3);

        }
    }

#else

    if( attr->o_flags & PIO_FLAG_IS_PULLUP ){
        mode = 0;
    } else if( attr->o_flags & PIO_FLAG_IS_REPEATER ){
        mode = 1;
    } else if( attr->o_flags & PIO_FLAG_IS_FLOAT ){
        mode = 2;
    } else if ( attr->o_flags & PIO_FLAG_IS_PULLDOWN ){
        mode = 3;
    }

    for(i = 0; i < 8*sizeof(u32); i++){
        if ( (1<<i) & attr->o_pinmask ){

            mcu_pin_t pin = mcu_pin(port, i);
            mcu_core_set_pinsel_func(&pin, CORE_PERIPH_PIO, 0); //set the pin to use GPIO

            if( attr->o_flags & PIO_FLAG_IS_OPENDRAIN ){
                regs_od[port] |= (1<<i);
            } else {
                regs_od[port] &= ~(1<<i);
            }

            regs_pin = &(&(LPC_PINCON->PINMODE0))[port*2 + i/16];
            shift = (i < 16 ? i*2 : ((i-16)*2));
            MCU_CLR_MASK( *regs_pin, (3<<shift) );
            MCU_SET_MASK( *regs_pin, (mode<<(shift) ) );
        }
    }
#endif

    return 0;
}

int mcu_pio_setmask(const devfs_handle_t * handle, void * ctl){
    int port = handle->port;
    LPC_GPIO_Type * gpio_regs = (LPC_GPIO_Type *)LPC_GPIO0_BASE;
    gpio_regs[port].SET=(int)ctl;
    return 0;
}

int mcu_pio_clrmask(const devfs_handle_t * handle, void * ctl){
    int port = handle->port;
    LPC_GPIO_Type * gpio_regs = (LPC_GPIO_Type *)LPC_GPIO0_BASE;
    gpio_regs[port].CLR=(int)ctl;
    return 0;
}

int mcu_pio_get(const devfs_handle_t * handle, void * ctl){
    int port = handle->port;
    u32 * value = ctl;
    LPC_GPIO_Type * gpio_regs = (LPC_GPIO_Type *)LPC_GPIO0_BASE;
    if( value ){
        *value = gpio_regs[port].PIN;
        return 0;
    }
    return SYSFS_SET_RETURN(EINVAL);
}

int mcu_pio_set(const devfs_handle_t * handle, void * ctl){
    int port = handle->port;
    LPC_GPIO_Type * gpio_regs = (LPC_GPIO_Type *)LPC_GPIO0_BASE;
    gpio_regs[port].PIN=(int)ctl;
    return 0;
}

void exec_cancelled0(){
    mcu_execute_event_handler(&(m_pio0_local.handler), MCU_EVENT_FLAG_CANCELED, 0);
}

void exec_cancelled2(){
    mcu_execute_event_handler(&(m_pio2_local.handler), MCU_EVENT_FLAG_CANCELED, 0);
}

//On __lpc17xx The pio interrupts use the eint3 interrupt service routine -- this function should be called from there
void mcu_core_pio0_isr(){
    pio_event_data_t ev;
    u32 status = LPC_GPIOINT->IntStatus;

    if ( status & (1<<0) ){
        ev.status = 0;
        ev.rising = LPC_GPIOINT->IO0IntStatR;
        ev.falling = LPC_GPIOINT->IO0IntStatF;
        LPC_GPIOINT->IO0IntClr = ev.rising | ev.falling;
        mcu_execute_event_handler(&(m_pio0_local.handler), MCU_EVENT_FLAG_RISING | MCU_EVENT_FLAG_FALLING, &ev);
    }

    if ( status & (1<<2) ){
        ev.status = 0;
        ev.rising = LPC_GPIOINT->IO2IntStatR;
        ev.falling = LPC_GPIOINT->IO2IntStatF;
        LPC_GPIOINT->IO2IntClr = ev.rising | ev.falling;
        mcu_execute_event_handler(&(m_pio2_local.handler), MCU_EVENT_FLAG_RISING | MCU_EVENT_FLAG_FALLING, &ev);
    }
}


#endif

