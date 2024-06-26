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

#include <mcu/qei.h>
#include "lpc_local.h"

#if MCU_QEI_PORTS > 0

typedef struct {
	mcu_event_handler_t handler;
	uint8_t ref_count;
} qei_local_t;

static qei_local_t qei_local[MCU_QEI_PORTS] MCU_SYS_MEM;
static LPC_QEI_Type * const qei_regs[MCU_QEI_PORTS] = MCU_QEI_REGS;
static u8 const qei_irqs[MCU_QEI_PORTS] = MCU_QEI_IRQS;

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(qei, QEI_VERSION, QEI_IOC_IDENT_CHAR, I_MCU_TOTAL + I_QEI_TOTAL, mcu_qei_get, mcu_qei_getvelocity, mcu_qei_getindex)

int mcu_qei_open(const devfs_handle_t * handle){
	int port = handle->port;
	if ( qei_local[port].ref_count == 0 ){
		mcu_lpc_core_enable_pwr(PCQEI);
		cortexm_enable_irq(qei_irqs[port]);
	}
	qei_local[port].ref_count++;
    return 0;
}

int mcu_qei_close(const devfs_handle_t * handle){
	int port = handle->port;
	if ( qei_local[port].ref_count > 0 ){
		if ( qei_local[port].ref_count == 1 ){
			cortexm_disable_irq(qei_irqs[port]);
			mcu_lpc_core_disable_pwr(PCQEI);
		}
		qei_local[port].ref_count--;
	}
    return 0;
}

int mcu_qei_setattr(const devfs_handle_t * handle, void * ctl){
	int port = handle->port;

	const qei_attr_t * attr = mcu_select_attr(handle, ctl);
	if( attr == 0 ){
        return SYSFS_SET_RETURN(EINVAL);
	}

	LPC_QEI_Type * regs = qei_regs[port];
	u32 o_flags = attr->o_flags;

	if( o_flags & QEI_FLAG_RESET ){
		if ( o_flags & QEI_FLAG_IS_RESET_POS ){
			regs->CON |= (1<<0);
		}

		if ( o_flags & QEI_FLAG_IS_RESET_VELOCITY ){
			regs->CON |= (1<<2);
		}

		if ( o_flags & QEI_FLAG_IS_RESET_INDEX ){
			regs->CON |= (1<<3);
		}

		if ( o_flags & QEI_FLAG_IS_RESET_POS_ONINDEX ){
			regs->CON |= (1<<1);
		}
	}

	if( o_flags & QEI_FLAG_SET ){


		if ( attr->velocity_freq == 0 ){
            return SYSFS_SET_RETURN(EINVAL);
		}

                if ( attr->velocity_freq > lpc_config.clock_peripheral_freq ){
            return SYSFS_SET_RETURN(EINVAL);
		}

		if( mcu_set_pin_assignment(
				&(attr->pin_assignment),
				MCU_CONFIG_PIN_ASSIGNMENT(qei_config_t, handle),
				MCU_PIN_ASSIGNMENT_COUNT(qei_pin_assignment_t),
                CORE_PERIPH_QEI, port, 0, 0, 0) < 0 ){
            return SYSFS_SET_RETURN(EINVAL);
		}

		regs->MAXPOS = attr->max_position;
                regs->LOAD = lpc_config.clock_peripheral_freq / attr->velocity_freq;
#ifdef __lpc17xx
		regs->FILTER = attr->filter;
#endif

		regs->CONF = 0;
		if( o_flags & QEI_FLAG_IS_INVERT_DIR ){
			regs->CONF |= (1<<0);
		}

		if( o_flags & QEI_FLAG_IS_SIGNAL_MODE ){
			regs->CONF |= (1<<1);
		}

		if( o_flags & QEI_FLAG_IS_DOUBLE_EDGE ){
			regs->CONF |= (1<<2);
		}

		if( o_flags & QEI_FLAG_IS_INVERT_INDEX ){
			regs->CONF |= (1<<3);
		}
	}
	return 0;
}

int mcu_qei_getinfo(const devfs_handle_t * handle, void * ctl){
	return 0;
}

int mcu_qei_setaction(const devfs_handle_t * handle, void * ctl){
	int port = handle->port;
	LPC_QEI_Type * regs = qei_regs[port];

	mcu_action_t * action = (mcu_action_t*)ctl;
	regs->IEC = 0x1FFF;
	if( action->o_events & MCU_EVENT_FLAG_INDEX ){
		regs->IES = (1<<0);
	}

	if( action->o_events & MCU_EVENT_FLAG_DIRECTION_CHANGED ){
		regs->IES = (1<<4);
	}

	if( cortexm_validate_callback(action->handler.callback) < 0 ){
        return SYSFS_SET_RETURN(EPERM);
	}

	qei_local[port].handler.callback = action->handler.callback;
	qei_local[port].handler.context = action->handler.context;

    cortexm_set_irq_priority(qei_irqs[port], action->prio, action->o_events);


	return 0;
}

int mcu_qei_read(const devfs_handle_t * handle, devfs_async_t * rop){
	const int port = handle->port;
	if( cortexm_validate_callback(rop->handler.callback) < 0 ){
        return SYSFS_SET_RETURN(EPERM);
    }

	qei_local[port].handler.callback = rop->handler.callback;
	qei_local[port].handler.context = rop->handler.context;
	return 0;
}

int mcu_qei_get(const devfs_handle_t * handle, void * ctl){
	int port = handle->port;
    u32 * value = ctl;
	LPC_QEI_Type * regs = qei_regs[port];
    if( value ){
        *value = regs->POS;
        return 0;
    }
    return SYSFS_SET_RETURN(EINVAL);
}

int mcu_qei_getvelocity(const devfs_handle_t * handle, void * ctl){
	int port = handle->port;
	LPC_QEI_Type * regs = qei_regs[port];
    u32 * value = ctl;
    if( value ){
        *value = regs->CAP;
        return 0;
    }
    return SYSFS_SET_RETURN(EINVAL);
}

int mcu_qei_getindex(const devfs_handle_t * handle, void * ctl){
	int port = handle->port;
	LPC_QEI_Type * regs = qei_regs[port];
    u32 * value = ctl;
    if( value ){
        *value = regs->INXCNT;
        return 0;
    }
    return SYSFS_SET_RETURN(EINVAL);
}

int mcu_qei_write(const devfs_handle_t * handle, devfs_async_t * async){
    return SYSFS_SET_RETURN(ENOTSUP);
}

void mcu_core_qei0_isr(){
	const int port = 0;
	u32 o_events = 0;
	LPC_QEI_Type * regs = qei_regs[port];

	o_events = regs->INTSTAT;
	regs->CLR = 0x1FFF;
	mcu_execute_event_handler(&(qei_local[port].handler), o_events, 0);
}

#endif




