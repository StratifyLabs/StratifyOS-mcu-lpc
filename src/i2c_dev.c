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

#include <mcu/i2c.h>
#include "lpc_local.h"


#if MCU_I2C_PORTS > 0

enum {
	I2C_STATE_NONE,
	I2C_STATE_START /*! Internal Use only */,
	I2C_STATE_RD_OP /*! Internal Use only */,
	I2C_STATE_WR_OP /*! Internal Use only */,
	I2C_STATE_RD_16_OP /*! Internal Use only */,
	I2C_STATE_WR_16_OP /*! Internal Use only */,
	I2C_STATE_WR_PTR_ONLY /*! Internal use only */,
	I2C_STATE_WR_PTR_16_ONLY /*! Internal use only */,
	I2C_STATE_WR_ONLY /*! Internal use only */,
	I2C_STATE_NO_STOP,
	I2C_STATE_MASTER_COMPLETE_NO_STOP /*! Internal use only */,
	I2C_STATE_MASTER_COMPLETE /*! Internal use only */,
	I2C_STATE_SLAVE_READ /*! Internal use only */,
	I2C_STATE_SLAVE_READ_PTR /*! Internal use only */,
	I2C_STATE_SLAVE_READ_PTR_16 /*! Internal use only */,
	I2C_STATE_SLAVE_READ_PTR_COMPLETE /*! Internal use only */,
	I2C_STATE_SLAVE_READ_COMPLETE /*! Internal use only */,
	I2C_STATE_SLAVE_WRITE /*! Internal use only */,
	I2C_STATE_SLAVE_WRITE_COMPLETE /*! Internal use only */,
	I2C_STATE_GENERAL_READ /*! Internal use only */,
	I2C_STATE_GENERAL_WRITE /*! Internal use only */,
	I2C_ERROR_TOTAL
};

#define WRITE_OP 0
#define READ_OP 1

typedef union {
	u16 ptr16;
	u8 ptr8[2];
} i2c_ptr_t;

typedef struct MCU_PACK {
	mcu_event_handler_t handler;
	char * volatile data;
	int * ret;
	volatile u16 size;
	u16 resd;
} i2c_local_transfer_t;

typedef struct MCU_PACK {
	//i2c_slave_setup_t setup;
	u32 size;
	char * data;
	mcu_event_handler_t handler;
	i2c_ptr_t ptr;
} i2c_local_slave_t;


typedef struct MCU_PACK {
	u8 ref_count;
	volatile u8 state;
	volatile u8 err;
	u8 resd[1];
	volatile i2c_ptr_t ptr;
	i2c_slave_addr_t slave_addr[2];
	u32 o_flags;
	i2c_local_transfer_t master;
	i2c_local_slave_t slave;
} i2c_local_t;

static void enable_opendrain_pin(const mcu_pin_t * pin, void * internal_pullup) MCU_PRIV_CODE;
void enable_opendrain_pin(const mcu_pin_t * pin, void * internal_pullup){
	pio_attr_t pattr;
	devfs_handle_t pio_handle;
	pio_handle.port = pin->port;
	pio_handle.config = 0;
	pattr.o_pinmask = (1<<pin->pin);
	pattr.o_flags = PIO_FLAG_SET_OUTPUT | PIO_FLAG_IS_OPENDRAIN | (u32)internal_pullup;
	mcu_pio_setattr(&pio_handle, &pattr);
}

static LPC_I2C_Type * const i2c_regs_table[MCU_I2C_PORTS] = MCU_I2C_REGS;
static u8 const i2c_irqs[MCU_I2C_PORTS] = MCU_I2C_IRQS;
static void set_master_done(LPC_I2C_Type * regs, int port, int error);
static int set_slave_attr(const devfs_handle_t * handle, const i2c_attr_t * attr);

static void receive_byte(LPC_I2C_Type * regs, i2c_local_transfer_t * op, int no_stop);
static void transmit_byte(LPC_I2C_Type * regs, i2c_local_transfer_t * op, int no_stop);
static int is_no_stop(const i2c_local_t * local);

static int receive_slave_byte(LPC_I2C_Type * regs, i2c_local_slave_t * op);
static int transmit_slave_byte(LPC_I2C_Type * regs, i2c_local_slave_t * op);


static void set_ack(LPC_I2C_Type * regs, u16 size, int no_stop);
static void set_slave_ack(LPC_I2C_Type * regs, i2c_local_slave_t * slave);

static inline LPC_I2C_Type * i2c_get_regs(int port) MCU_ALWAYS_INLINE;
LPC_I2C_Type * i2c_get_regs(int port){
	return i2c_regs_table[port];
}


i2c_attr_t i2c_local_attr[MCU_I2C_PORTS] MCU_SYS_MEM;
i2c_local_t i2c_local[MCU_I2C_PORTS] MCU_SYS_MEM;

#define AA (1<<2) //assert acknowledge flag
#define SI (1<<3) //I2C Interrupt flag
#define STO (1<<4) //stop condition
#define STA (1<<5) //start condition
#define I2EN (1<<6) //interface enable


static int i2c_transfer(const devfs_handle_t * handle, int op, devfs_async_t * dop);

#define i2c_slave_ack(port) (LPC_I2C[port].CONSET = I2CONSET_AA)
#define i2c_slave_nack(port) (LPC_I2C[port].CONCLR = I2CONCLR_AAC)
#define i2c_slave_clr_int(port) (LPC_I2C[port].CONCLR = I2CONCLR_SIC)

DEVFS_MCU_DRIVER_IOCTL_FUNCTION_MIN(i2c, I2C_VERSION)

int mcu_i2c_open(const devfs_handle_t * handle){
	int port = handle->port;
	if ( i2c_local[port].ref_count == 0 ){
		switch(port){
		case 0:
			mcu_lpc_core_enable_pwr(PCI2C0);
			break;
#if MCU_I2C_PORTS > 1
		case 1:
			mcu_lpc_core_enable_pwr(PCI2C1);
			break;
#endif
#if MCU_I2C_PORTS > 2
		case 2:
			mcu_lpc_core_enable_pwr(PCI2C2);
			break;
#endif
		}
		memset(&(i2c_local[port].master), 0, sizeof(i2c_local_transfer_t));
		memset(&(i2c_local[port].slave), 0, sizeof(i2c_local_slave_t));
		cortexm_enable_irq(i2c_irqs[port]);
	}
	i2c_local[port].ref_count++;
    return 0;
}

int mcu_i2c_close(const devfs_handle_t * handle){
	int port = handle->port;
	LPC_I2C_Type * i2c_regs;
	if ( i2c_local[port].ref_count > 0 ){
		if ( i2c_local[port].ref_count == 1 ){
			i2c_regs = i2c_regs_table[port];
			i2c_regs->CONCLR = (AA);
			i2c_regs->ADR0 = 0;
			i2c_regs->ADR1 = 0;
			i2c_regs->ADR2 = 0;
			i2c_regs->ADR3 = 0;
			cortexm_disable_irq(i2c_irqs[port]);
			switch(port){
			case 0:
				mcu_lpc_core_disable_pwr(PCI2C0);
				break;
#if MCU_I2C_PORTS > 1
			case 1:
				mcu_lpc_core_disable_pwr(PCI2C1);
				break;
#endif
#if MCU_I2C_PORTS > 2
			case 2:
				mcu_lpc_core_disable_pwr(PCI2C2);
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
			I2C_FLAG_IS_SLAVE_ADDR3;

	info->freq = 400000;
	info->o_events = MCU_EVENT_FLAG_WRITE_COMPLETE | MCU_EVENT_FLAG_DATA_READY;
	return 0;
}

int mcu_i2c_setattr(const devfs_handle_t * handle, void * ctl){
	int port = handle->port;
	LPC_I2C_Type * i2c_regs;
	int count;
	int internal_pullup;
	u32 freq;

	const i2c_attr_t * attr = mcu_select_attr(handle, ctl);
	if( attr == 0 ){
        return SYSFS_SET_RETURN(EINVAL);
	}

	u32 o_flags = attr->o_flags;

	//Check for a valid port
	i2c_regs = i2c_regs_table[port];

	if ( attr->freq == 0 ){
		freq = 100000;
	} else {
		freq = attr->freq;
	}

	if( o_flags & (I2C_FLAG_SET_MASTER|I2C_FLAG_SET_SLAVE) ){

		if ( o_flags & I2C_FLAG_IS_PULLUP ){
			internal_pullup = PIO_FLAG_IS_PULLUP;
		} else {
			internal_pullup = 0;
		}


		if( mcu_set_pin_assignment(
				&(attr->pin_assignment),
				MCU_CONFIG_PIN_ASSIGNMENT(i2c_config_t, handle),
				MCU_PIN_ASSIGNMENT_COUNT(i2c_pin_assignment_t),
                CORE_PERIPH_I2C, port, enable_opendrain_pin, 0, (void*)internal_pullup) < 0 ){
        return SYSFS_SET_RETURN(EINVAL);
		}

		i2c_regs->CONCLR = 0xFF;

		i2c_local[port].state = I2C_STATE_NONE;
		i2c_local[port].master.size = 0; //no bytes
		i2c_local[port].master.data = 0; //no data pointer

		count = ((mcu_board_config.core_periph_freq) / (freq * 2));
		if ( count > 0xFFFF ){
			count = 0xFFFF;
		}
		i2c_regs->SCLH = count;
		i2c_regs->SCLL = count;

		//Enable the I2C unit
		i2c_regs->CONSET = (I2EN);
	}

	if( o_flags & I2C_FLAG_SET_SLAVE ){
		//setup the device in slave mode
		return set_slave_attr(handle, attr);

	}

	if( o_flags & (I2C_FLAG_PREPARE_PTR_DATA|I2C_FLAG_PREPARE_PTR|I2C_FLAG_PREPARE_DATA) ){
		i2c_local[port].slave_addr[0] = attr->slave_addr[0];
		i2c_local[port].slave_addr[1] = attr->slave_addr[1];
		i2c_local[port].o_flags = o_flags;
	}

	return 0;
}


int set_slave_attr(const devfs_handle_t * handle, const i2c_attr_t * attr){
	int port = handle->port;
	LPC_I2C_Type * i2c_regs;
	int gen_call = 0;
	u32 o_flags = attr->o_flags;

	// \todo To fix Issue #30, we need to check the dest mem and make sure it is writable by the caller

	//memcpy( &(i2c_local[port].slave.setup), ctl, sizeof(i2c_slave_setup_t));

	i2c_local[port].slave.data = attr->data;
	i2c_local[port].slave.size = attr->size;
	i2c_local[port].slave.ptr.ptr16 = 0;


	if( attr->size > 255 ){
		i2c_local[port].o_flags |= I2C_FLAG_IS_SLAVE_PTR_16;
	} else {
		i2c_local[port].o_flags |= I2C_FLAG_IS_SLAVE_PTR_8;
	}


	i2c_regs = i2c_regs_table[port];

	if( o_flags & I2C_FLAG_IS_SLAVE_ACK_GENERAL_CALL ){
		gen_call = 1;
	}

	if( o_flags & I2C_FLAG_IS_SLAVE_ADDR0 ){
		i2c_regs->ADR0 = (attr->slave_addr[0].addr8[0] << 1) | gen_call;
#if MCU_I2C_API == 0
		i2c_regs->MASK0 = 0x00;
#endif
	}

	if( o_flags & I2C_FLAG_IS_SLAVE_ADDR1 ){
		i2c_regs->ADR1 = (attr->slave_addr[0].addr8[1] << 1) | gen_call;
#if MCU_I2C_API == 0
		i2c_regs->MASK1 = 0x00;
#endif
	}

	if( o_flags & I2C_FLAG_IS_SLAVE_ADDR2 ){
		i2c_regs->ADR2 = (attr->slave_addr[1].addr8[0] << 1) | gen_call;
#if MCU_I2C_API == 0
		i2c_regs->MASK2 = 0x00;
#endif
	}

	if( o_flags & I2C_FLAG_IS_SLAVE_ADDR3 ){
		i2c_regs->ADR3 = (attr->slave_addr[1].addr8[1] << 1) | gen_call;
#if MCU_I2C_API == 0
		i2c_regs->MASK3 = 0x00;
#endif
	}

	i2c_regs->CONSET = (AA);
	return 0;
}

int mcu_i2c_reset(const devfs_handle_t * handle, void * ctl){
	LPC_I2C_Type * i2c_regs;
	int port = handle->port;
	i2c_regs = i2c_regs_table[port];

	i2c_regs->CONCLR = 0xFF;
	i2c_regs->CONSET = I2EN;

	return 0;
}

int mcu_i2c_geterr(const devfs_handle_t * handle, void * ctl){
	int port = handle->port;
	return i2c_local[port].err;
}

int mcu_i2c_setaction(const devfs_handle_t * handle, void * ctl){
	mcu_action_t * action = (mcu_action_t*)ctl;
	int port = handle->port;

	cortexm_set_irq_priority(i2c_irqs[port], action->prio);

	if( action->handler.callback == 0 ){
		i2c_local[port].slave.handler.callback = 0;
		i2c_local[port].slave.handler.context = 0;
		return 0;
	}

	if( cortexm_validate_callback(action->handler.callback) < 0 ){
        return SYSFS_SET_RETURN(EPERM);
	}

	i2c_local[port].slave.handler.callback = action->handler.callback;
	i2c_local[port].slave.handler.context = action->handler.context;

	return 0;
}

int mcu_i2c_write(const devfs_handle_t * handle, devfs_async_t * wop){
	return i2c_transfer(handle, WRITE_OP, wop);
}

int mcu_i2c_read(const devfs_handle_t * handle, devfs_async_t * rop){
	return i2c_transfer(handle, READ_OP, rop);
}

void set_master_done(LPC_I2C_Type * regs, int port, int error){
	if( error != 0 ){
		*(i2c_local[port].master.ret) = -1;
	} else {
		*(i2c_local[port].master.ret) -= i2c_local[port].master.size;
	}
	i2c_local[port].master.size = 0;
	i2c_local[port].err = error;
	i2c_local[port].master.data = 0;

	if( !is_no_stop(i2c_local + port) || error ){
		i2c_local[port].state = I2C_STATE_MASTER_COMPLETE;
		regs->CONSET = STO;
	} else {
		i2c_local[port].state = I2C_STATE_MASTER_COMPLETE_NO_STOP;
	}

}


void set_ack(LPC_I2C_Type * regs, u16 size, int no_stop){
	if( size > 1 ){
		regs->CONSET = AA;
	} else {
		if( no_stop == 0 ){
			regs->CONCLR = AA;
		}
	}
}

static int is_no_stop(const i2c_local_t * local){
	return ((local->o_flags & I2C_FLAG_IS_NO_STOP) != 0);
}

void receive_byte(LPC_I2C_Type * regs, i2c_local_transfer_t * op, int no_stop){
	if( op->size && op->data ){
		*(op->data) = regs->DAT;
		op->data++;
		op->size--;
	} else {
		regs->DAT;
	}

	set_ack(regs, op->size, no_stop);

}

void transmit_byte(LPC_I2C_Type * regs, i2c_local_transfer_t * op, int no_stop){
	if( op->size && op->data ){
		regs->DAT = *(op->data);
		op->data++;
		op->size--;
	} else {
		regs->DAT = 0xFF;
	}

	if( no_stop == 0 ){
		set_ack(regs, op->size, no_stop);
	}
}

int receive_slave_byte(LPC_I2C_Type * regs, i2c_local_slave_t * op){
	if( op->ptr.ptr16 < op->size ){
		op->data[ op->ptr.ptr16 ] = regs->DAT;
		op->ptr.ptr16++;
	} else {
		regs->DAT;
		return 0;
	}

	return (op->ptr.ptr16+1) < op->size;
}

int transmit_slave_byte(LPC_I2C_Type * regs, i2c_local_slave_t * op){
	if( op->ptr.ptr16 < op->size ){
		regs->DAT = op->data[ op->ptr.ptr16 ];
		op->ptr.ptr16++;
		if( op->ptr.ptr16 == op->size ){
			op->ptr.ptr16 = 0;
		}
	} else {
		regs->DAT = 0xFF;
	}
	return 0;
}

void set_slave_ack(LPC_I2C_Type * regs, i2c_local_slave_t * slave){
	if( slave->size > (slave->ptr.ptr16 + 1) ){
		regs->CONSET = AA;
	} else {
		regs->CONCLR = AA;
	}
}

static void mcu_i2c_isr(int port) {
	u8 stat_value;
	u32 o_events;
	LPC_I2C_Type * i2c_regs;
	// this handler deals with master read and master write only
	i2c_regs = i2c_get_regs(port);

	stat_value = i2c_regs->STAT;

	switch ( stat_value ){
	case 0x08: //Start Condition has been sent
	case 0x10: //Repeated Start condition
		i2c_local[port].err = 0;
		if ( i2c_local[port].state == I2C_STATE_START ){
			i2c_regs->DAT = (i2c_local[port].slave_addr[0].addr8[0] << 1) | 0x01; //Set the Read bit -- repeated start after write ptr
		} else {
			i2c_regs->DAT = (i2c_local[port].slave_addr[0].addr8[0] << 1); //send the address -- in write mode
		}

		i2c_regs->CONSET = AA;
		i2c_regs->CONCLR = STA;
		break;
	case 0x18: //SLA+W transmitted -- Ack Received
		if( i2c_local[port].state == I2C_STATE_WR_ONLY ){
			//this is write only mode -- don't send the ptr
			transmit_byte(i2c_regs, &(i2c_local[port].master), is_no_stop(i2c_local + port));
		} else {
			i2c_regs->DAT = i2c_local[port].ptr.ptr8[1]; //Send the offset pointer (MSB of 16 or 8 bit)
			i2c_regs->CONSET = AA;
		}
		break;
	case 0x20: //SLA+W has been transmitted; NOT ACK has been received
		set_master_done(i2c_regs, port, I2C_ERROR_ACK);
		break;
	case 0x28: //Data byte transmitted -- Ack Received
		if( i2c_local[port].state == I2C_STATE_WR_PTR_16_ONLY ){
			i2c_regs->DAT = i2c_local[port].ptr.ptr8[0]; //Send the offset pointer (LSB)
			i2c_regs->CONSET = AA;
			i2c_local[port].state = I2C_STATE_WR_PTR_ONLY;
			break;
		} else if( i2c_local[port].state == I2C_STATE_WR_PTR_ONLY ){
			i2c_local[port].state = I2C_STATE_MASTER_COMPLETE; //writing the pointer only doesn't support I2C_FLAG_IS_NO_STOP
			i2c_regs->CONSET = STO|AA;
			break;
		} else if ( i2c_local[port].state == I2C_STATE_RD_16_OP ){
			i2c_regs->DAT = i2c_local[port].ptr.ptr8[0]; //Send the offset pointer (LSB)
			i2c_regs->CONSET = AA;
			i2c_local[port].state = I2C_STATE_RD_OP;
			break;
		} else if ( i2c_local[port].state == I2C_STATE_WR_16_OP ){
			i2c_regs->DAT = i2c_local[port].ptr.ptr8[0]; //Send the offset pointer (LSB)
			i2c_regs->CONSET = AA;
			i2c_local[port].state = I2C_STATE_WR_OP;
			break;
		}

		if ( i2c_local[port].master.size ){

			if ( i2c_local[port].state == I2C_STATE_RD_OP ){
				i2c_regs->CONSET = STA; //Restart (then send read command)
				i2c_local[port].state = I2C_STATE_START;
				break;
			}

			//Transmit data
			transmit_byte(i2c_regs, &(i2c_local[port].master), is_no_stop(i2c_local + port));

		} else {
			set_master_done(i2c_regs, port, 0);
		}
		break;
	case 0x48: //SLA+R has been transmitted; NOT ACK has been received.
		set_master_done(i2c_regs, port, I2C_ERROR_ACK);
		break;
	case 0x30: //Data byte in I2DAT has been transmitted; NOT ACK has been received.
		set_master_done(i2c_regs, port, 0);
		break;
	case 0x38:
		set_master_done(i2c_regs, port, I2C_ERROR_ARBITRATION_LOST);
		break;
	case 0x40: //SLA+R transmitted -- Ack received
		set_ack(i2c_regs, i2c_local[port].master.size, is_no_stop(i2c_local + port));
		break;
	case 0x50: //Data Byte received -- Ack returned
		//Receive Data
		receive_byte(i2c_regs, &(i2c_local[port].master), is_no_stop(i2c_local + port));
		if( i2c_local[port].master.size == 0 ){
			set_master_done(i2c_regs, port, 0);
		}
		break;
	case 0x58: //Data byte received -- Not Ack returned
		receive_byte(i2c_regs, &(i2c_local[port].master), 0);
		set_master_done(i2c_regs, port, 0);
		break;

	case 0x00: //Bus error in Master or selected slave mode -- illegal start or stop
		if( (i2c_local[port].state >= I2C_STATE_START) && (i2c_local[port].state <= I2C_STATE_MASTER_COMPLETE)){
			set_master_done(i2c_regs, port, I2C_ERROR_BUS_BUSY);
		} else if( (i2c_local[port].state >= I2C_STATE_SLAVE_READ) && (i2c_local[port].state <= I2C_STATE_SLAVE_WRITE_COMPLETE)){
			i2c_regs->CONSET = STO;
			mcu_execute_event_handler(&(i2c_local[port].slave.handler), MCU_EVENT_FLAG_ERROR, 0);
		}

		break;

		//SLAVE OPERATION STARTS HERE --------------------------------------------------------------------------------
	case 0x68: //Arbitration lost in SLA+R/W as master; Own SLA+W has been received, ACK returned
	case 0x60: //Slave mode SLA+W has been received -- Ack returned
	case 0x78: //Arbitration lost and general ack received
	case 0x70: //General call has been received -- ack returned
		if( i2c_local[port].o_flags & I2C_FLAG_IS_SLAVE_PTR_16 ){
			i2c_local[port].state = I2C_STATE_SLAVE_READ_PTR_16;
		} else {
			i2c_local[port].state = I2C_STATE_SLAVE_READ_PTR;
		}
		i2c_local[port].slave.ptr.ptr16 = 0;
		i2c_regs->CONSET = AA;
		break;

	case 0x90: //data has been received and ack returned on general call
	case 0x80: //data has been received and ack has been returned
		if( i2c_local[port].state == I2C_STATE_SLAVE_READ_PTR_16 ){
			i2c_local[port].slave.ptr.ptr8[1] = i2c_regs->DAT; //MSB is first
			i2c_local[port].state = I2C_STATE_SLAVE_READ_PTR;
		} else if( i2c_local[port].state == I2C_STATE_SLAVE_READ_PTR ){
			i2c_local[port].slave.ptr.ptr8[0] = i2c_regs->DAT; //LSB is second
			i2c_local[port].state = I2C_STATE_SLAVE_READ_PTR_COMPLETE;
		} else {
			i2c_local[port].state = I2C_STATE_SLAVE_READ;
			receive_slave_byte(i2c_regs, &(i2c_local[port].slave));
		}
		set_slave_ack(i2c_regs, &(i2c_local[port].slave));

		break;

	case 0x88: //data has been received and NOT ack has been returned
	case 0x98: //data has been received and NOT ack has been returned on general call address
		receive_slave_byte(i2c_regs, &(i2c_local[port].slave));

		//keep slave address active
		i2c_regs->CONSET = AA;
		i2c_local[port].state = I2C_STATE_SLAVE_READ_COMPLETE;
		//the device is no longer addressed and won't interrupt when the stop condition occurs
		mcu_execute_event_handler(&(i2c_local[port].slave.handler), MCU_EVENT_FLAG_DATA_READY, 0);
		break;

	case 0xA8: //Own SLA+R has been received and ack returned
	case 0xB0: //Arbitration lost in SLA+R/W as master; Own SLA+R has been received, ACK has been returned.
	case 0xB8: //Data byte in I2DAT has been transmitted; ACK has been received.
		//load byte and send ack
		i2c_local[port].state = I2C_STATE_SLAVE_WRITE; //Slave is being read (host will write the data)
		transmit_slave_byte(i2c_regs, &(i2c_local[port].slave));
		i2c_regs->CONSET = AA;

		break;

	case 0xC0: //Data byte in I2DAT has been transmitted; NOT ACK has been received.
	case 0xC8: //Last data byte in I2DAT has been transmitted (AA = 0); ACK has been received.
		//set ack to stay in slave mode
		i2c_local[port].state = I2C_STATE_SLAVE_WRITE_COMPLETE;
		i2c_regs->CONSET = AA;

		//the device is no longer addressed and won't interrupt when the stop condition occurs
		mcu_execute_event_handler(&(i2c_local[port].slave.handler), MCU_EVENT_FLAG_WRITE_COMPLETE, 0);
		break;

	case 0xA0: //stop or restart has been received while addressed

		//execute read and write callbacks
		o_events = 0;
		if( (i2c_local[port].state == I2C_STATE_SLAVE_WRITE_COMPLETE) ||
				(i2c_local[port].state == I2C_STATE_SLAVE_WRITE) ){
			o_events = MCU_EVENT_FLAG_WRITE_COMPLETE;
		} else if( (i2c_local[port].state == I2C_STATE_SLAVE_READ_COMPLETE) ||
				(i2c_local[port].state == I2C_STATE_SLAVE_READ) ){
			o_events = MCU_EVENT_FLAG_DATA_READY;
		} else {
			o_events = MCU_EVENT_FLAG_ADDRESSED;
		}

		i2c_regs->CONSET = AA;

		mcu_execute_event_handler(&(i2c_local[port].slave.handler), o_events, 0);

		break;
	}

	i2c_regs->CONCLR = SI; //clear the interrupt flag

	if ( i2c_local[port].state == I2C_STATE_MASTER_COMPLETE ){
		i2c_local[port].state = I2C_STATE_NONE;
		mcu_execute_event_handler(&(i2c_local[port].master.handler), MCU_EVENT_FLAG_WRITE_COMPLETE | MCU_EVENT_FLAG_DATA_READY, 0);
	}

	if ( i2c_local[port].state == I2C_STATE_MASTER_COMPLETE_NO_STOP ){
		i2c_local[port].state = I2C_STATE_NO_STOP;
		mcu_execute_event_handler(&(i2c_local[port].master.handler), MCU_EVENT_FLAG_WRITE_COMPLETE | MCU_EVENT_FLAG_DATA_READY, 0);
	}
}


int i2c_transfer(const devfs_handle_t * handle, int op, devfs_async_t * dop){
	LPC_I2C_Type * i2c_regs;
	int size = dop->nbyte;
	int port = handle->port;
	i2c_regs = i2c_get_regs(port);
	int no_stop = 0;

	//writes will always write the loc value -- so only abort on read only
	if( i2c_local[port].o_flags & I2C_FLAG_PREPARE_DATA){
		if ( dop->nbyte == 0 ){
			return 0;
		}
	}

	if ( (i2c_local[port].state != I2C_STATE_NONE) && (i2c_local[port].state != I2C_STATE_NO_STOP) ){
        return SYSFS_SET_RETURN(EBUSY);
	}

	if(i2c_local[port].state == I2C_STATE_NO_STOP){
		no_stop = 1;
	}

	if( cortexm_validate_callback(dop->handler.callback) < 0 ){
        return SYSFS_SET_RETURN(EPERM);
	}

	i2c_local[port].master.handler.callback = dop->handler.callback;
	i2c_local[port].master.handler.context = dop->handler.context;
	i2c_local[port].master.data = (void * volatile)dop->buf;
	i2c_local[port].master.ret = &(dop->nbyte);
	i2c_local[port].master.size = size;


	if( i2c_local[port].o_flags & I2C_FLAG_PREPARE_PTR_DATA ){

		if( i2c_local[port].o_flags & I2C_FLAG_IS_PTR_16){
			i2c_local[port].ptr.ptr16 = dop->loc;  //16-bit ptr
			if ( op == WRITE_OP ){
				i2c_local[port].state = I2C_STATE_WR_16_OP;
			} else {
				i2c_local[port].state = I2C_STATE_RD_16_OP;
			}
		} else {
			i2c_local[port].ptr.ptr8[1] = dop->loc; //8-bit ptr
			if ( op == WRITE_OP ){
				i2c_local[port].state = I2C_STATE_WR_OP;
			} else {
				i2c_local[port].state = I2C_STATE_RD_OP;
			}
		}

	} else if( i2c_local[port].o_flags & I2C_FLAG_PREPARE_PTR ){
		i2c_local[port].master.size = 0;

		if( i2c_local[port].o_flags & I2C_FLAG_IS_PTR_16){
			i2c_local[port].ptr.ptr16 = dop->loc;  //16-bit ptr
		} else {
			i2c_local[port].ptr.ptr8[1] = dop->loc; //8-bit ptr
			i2c_local[port].state = I2C_STATE_WR_PTR_ONLY;
		}
	} else if( i2c_local[port].o_flags & (I2C_FLAG_PREPARE_DATA) ){
		if ( op == WRITE_OP ){
			i2c_local[port].state = I2C_STATE_WR_ONLY;
		} else {
			i2c_local[port].state = I2C_STATE_START;
		}
	} else {
        return SYSFS_SET_RETURN(EINVAL);
	}

	//If no start is set, then read write data directly
	if( no_stop ){
		if( op == WRITE_OP ){
			transmit_byte(i2c_regs, &(i2c_local[port].master), is_no_stop(i2c_local + port));
		} else {
			i2c_regs->CONSET = AA;
		}
	} else {
		i2c_regs->CONSET = STA; //exec start condition
	}
	return 0;
}

void mcu_core_i2c0_isr(){ mcu_i2c_isr(0); }
#if MCU_I2C_PORTS > 1
void mcu_core_i2c1_isr(){ mcu_i2c_isr(1); }
#endif
#if MCU_I2C_PORTS > 2
void mcu_core_i2c2_isr(){ mcu_i2c_isr(2); }
#endif

#endif


