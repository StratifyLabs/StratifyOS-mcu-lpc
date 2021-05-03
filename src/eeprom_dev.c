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

#include <mcu/eeprom.h>
#include "lpc_local.h"


#if MCU_EEPROM_PORTS > 0

#define EEPROM_PAGES (MCU_EEPROM_SIZE/MCU_EEPROM_PAGE_SIZE)

typedef struct MCU_PACK {
    u8 * buf;
    mcu_event_handler_t handler;
    u16 len;
    u8 isread;
    u8 offset;
    u8 page;
    u8 ref_count;
} eeprom_local_t;

static eeprom_local_t eeprom_local[MCU_EEPROM_PORTS];
LPC_EEPROM_Type * const eeprom_regs[MCU_EEPROM_PORTS] = MCU_EEPROM_REGS;
u8 const eeprom_irqs[MCU_EEPROM_PORTS] = MCU_EEPROM_IRQS;

static void write_eeprom_byte(int port);
static void write_eeprom_halfword(int port);

static void read_eeprom_byte(int port);
static void read_eeprom_halfword(int port);

static void exec_callback(int port, u32 o_flags, void * data);

static int calc_offset(int loc){
    return loc % MCU_EEPROM_PAGE_SIZE;
}

static int calc_page(int loc){
    return loc / MCU_EEPROM_PAGE_SIZE;
}

DEVFS_MCU_DRIVER_IOCTL_FUNCTION_MIN(eeprom, EEPROM_VERSION, EEPROM_IOC_IDENT_CHAR)

int mcu_eeprom_open(const devfs_handle_t * handle){
    int port = handle->port;
    uint8_t phase[3];
    int cpu_mhz;
    LPC_EEPROM_Type * regs = eeprom_regs[port];
    if( eeprom_local[port].ref_count == 0 ){
        regs->PWRDWN = 0;

        //enable the interrupt
        if( eeprom_irqs[port] != 0xFF ){
            cortexm_enable_irq(eeprom_irqs[port]);
        }

        //initialize the EEPROM clock
        regs->CLKDIV = (sos_config.sys.core_clock_frequency / 350000) - 1;

        //initialize the STATE register
        cpu_mhz = sos_config.sys.core_clock_frequency / 1000000;
        phase[0] = (((cpu_mhz*20) + 999) / 1000);
        phase[1] = (((cpu_mhz*65) + 999) / 1000);
        phase[2] = (((cpu_mhz*40) + 999) / 1000);
        regs->WSTATE = phase[0] | (phase[1]<<8) | (phase[2]<<16);

    }

    eeprom_local[port].ref_count++;
    return 0;
}

int mcu_eeprom_close(const devfs_handle_t * handle){
    int port = handle->port;
    LPC_EEPROM_Type * regs = eeprom_regs[port];

    if ( eeprom_local[port].ref_count > 0 ){
        if ( eeprom_local[port].ref_count == 1 ){
            //disable the interrupt
            cortexm_disable_irq(eeprom_irqs[port]);

            //power down
            regs->PWRDWN = 1;
        }
        eeprom_local[port].ref_count--;
    }
    return 0;
}

int mcu_eeprom_getinfo(const devfs_handle_t * handle, void * ctl){
    eeprom_info_t * info = ctl;
    info->size = MCU_EEPROM_SIZE;
    info->page_size = MCU_EEPROM_PAGE_SIZE;
    info->o_flags = 0;
    return 0;
}

int mcu_eeprom_setattr(const devfs_handle_t * handle, void * ctl){
    return 0;
}

int mcu_eeprom_setaction(const devfs_handle_t * handle, void * ctl){
    int port = handle->port;
    mcu_action_t * action = (mcu_action_t *)ctl;
    if( action->handler.callback == 0 ){
        if( action->o_events & (MCU_EVENT_FLAG_DATA_READY|MCU_EVENT_FLAG_WRITE_COMPLETE) ){
            exec_callback(port, MCU_EVENT_FLAG_CANCELED, 0);
        }
    }

    if( cortexm_validate_callback(action->handler.callback) < 0 ){
        return SYSFS_SET_RETURN(EPERM);
    }

    eeprom_local[port].handler.callback = action->handler.callback;
    eeprom_local[port].handler.context = action->handler.context;
    return 0;
}


int mcu_eeprom_write(const devfs_handle_t * handle, devfs_async_t * wop){
    int port = handle->port;
    if ( wop->nbyte == 0 ){
        return 0;
    }

    //Check to see if the port is busy
    if ( eeprom_local[port].handler.callback ){
        return SYSFS_SET_RETURN(EBUSY);
    }

    //check for a valid wop->loc value
    if( ((wop->loc + wop->nbyte) > MCU_EEPROM_SIZE) || (wop->loc < 0) ){
        return SYSFS_SET_RETURN(EINVAL);
    }

    //Initialize variables
    eeprom_local[port].buf = wop->buf;
    eeprom_local[port].len = wop->nbyte;
    eeprom_local[port].isread = 0;

    eeprom_local[port].page = calc_page(wop->loc);
    eeprom_local[port].offset = calc_offset(wop->loc);

    if( cortexm_validate_callback(wop->handler.callback) < 0 ){
        return SYSFS_SET_RETURN(EPERM);
    }

    eeprom_local[port].handler.callback = wop->handler.callback;
    eeprom_local[port].handler.context = wop->handler.context;

    write_eeprom_byte(port);

    while( (eeprom_local[port].offset < MCU_EEPROM_PAGE_SIZE) && (eeprom_local[port].len > 1) ) {
        write_eeprom_halfword(port);
    }

    write_eeprom_byte(port);

#if 0
    //fill the page
    regs->ADDR = eeprom_local[port].offset;
    regs->CMD = 3;
    do {
        regs->WDATA = *eeprom_local[port].buf;
        //wait until the previous data has written
        while( (regs->INTSTAT & (1<<26)) == 0 ){
            ;
        }
        regs->INTSTATCLR = (1<<26);
        eeprom_local[port].len--;
        eeprom_local[port].offset++;
        eeprom_local[port].buf++;
    } while( (eeprom_local[port].offset < MCU_EEPROM_PAGE_SIZE) && (eeprom_local[port].len > 0) );
#endif

    LPC_EEPROM_Type * regs = eeprom_regs[port];
    regs->ADDR = eeprom_local[port].page << 6;
    eeprom_local[port].page++;
    eeprom_local[port].offset = 0;
    regs->INT_SET_ENABLE = (1<<28);
    regs->CMD = 6; //erase/program page

    return 0;
}

int mcu_eeprom_read(const devfs_handle_t * handle, devfs_async_t * rop){
    int port = handle->port;

    //Check to see if the port is busy
    if ( eeprom_local[port].handler.callback ){
        return SYSFS_SET_RETURN(EBUSY);
    }


    if ( rop->nbyte == 0 ){
        return 0;
    }

    //check for a valid rop->loc value
    if( ((rop->loc + rop->nbyte) > MCU_EEPROM_SIZE) || (rop->loc < 0) ){
        return SYSFS_SET_RETURN(EINVAL);
    }

    //Initialize variables
    eeprom_local[port].buf = rop->buf;
    eeprom_local[port].len = rop->nbyte;
    eeprom_local[port].page = calc_page(rop->loc);
    eeprom_local[port].offset = calc_offset(rop->loc);

    read_eeprom_byte(port);

    if( eeprom_local[port].offset == MCU_EEPROM_PAGE_SIZE ){
        eeprom_local[port].offset = 0;
        eeprom_local[port].page++;
    }
    do {
        read_eeprom_halfword(port);

        if( eeprom_local[port].offset == MCU_EEPROM_PAGE_SIZE ){
            eeprom_local[port].offset = 0;
            eeprom_local[port].page++;
        }

    } while( eeprom_local[port].len > 1 );

    read_eeprom_byte(port);

#if 0
    LPC_EEPROM_Type * regs = eeprom_regs[port];
    do {
        regs->ADDR = eeprom_local[port].offset | (eeprom_local[port].page << 6);
        regs->CMD = 0;

        while( (regs->INTSTAT & (1<<26)) == 0 ){ ; }
        regs->INTSTATCLR = (1<<26);

        *eeprom_local[port].buf = regs->RDATA;
        //wait until the previous data has been read

        eeprom_local[port].len--;
        eeprom_local[port].offset++;
        eeprom_local[port].buf++;

        if( eeprom_local[port].offset == MCU_EEPROM_PAGE_SIZE ){
            eeprom_local[port].offset = 0;
            eeprom_local[port].page++;
        }

    } while( eeprom_local[port].len > 0 );
#endif

    eeprom_local[port].buf = 0;
    return rop->nbyte;
}

void exec_callback(int port, u32 o_flags, void * data){
    LPC_EEPROM_Type * regs = eeprom_regs[port];
    eeprom_local[port].buf = 0;
    mcu_execute_event_handler(&(eeprom_local[port].handler), o_flags, data);
    if( eeprom_local[port].handler.callback == 0 ){
        regs->INTENCLR = (1<<26)|(1<<28); //disable the interrupts
    }
}

void write_eeprom_byte(int port){
    LPC_EEPROM_Type * regs = eeprom_regs[port];

    if( (eeprom_local[port].len == 1) ||
            ((eeprom_local[port].len > 0) && ((eeprom_local[port].offset & 0x01) != 0)) ){
        regs->ADDR = eeprom_local[port].offset | eeprom_local[port].page << 6;
        regs->CMD = 3; //8-bit write

        regs->WDATA = *eeprom_local[port].buf;
        //wait until the previous data has written
        while( (regs->INTSTAT & (1<<26)) == 0 ){
            ;
        }
        regs->INTSTATCLR = (1<<26);
        eeprom_local[port].len--;
        eeprom_local[port].offset++;
        eeprom_local[port].buf++;
    }
}

void write_eeprom_halfword(int port){
    LPC_EEPROM_Type * regs = eeprom_regs[port];

    if( (eeprom_local[port].len > 1) && ((eeprom_local[port].offset & 0x01) == 0)  ){
        regs->ADDR = eeprom_local[port].offset | eeprom_local[port].page << 6;
        regs->CMD = 4; //16-bit write

        regs->WDATA = *((u16*)(eeprom_local[port].buf));
        //wait until the previous data has written
        while( (regs->INTSTAT & (1<<26)) == 0 ){
            ;
        }
        regs->INTSTATCLR = (1<<26);
        eeprom_local[port].len-=2;
        eeprom_local[port].offset+=2;
        eeprom_local[port].buf+=2;
    }
}

void read_eeprom_byte(int port){
    LPC_EEPROM_Type * regs = eeprom_regs[port];

    if( (eeprom_local[port].len == 1) ||
            ((eeprom_local[port].len > 0) && ((eeprom_local[port].offset & 0x01) != 0)) ){
        regs->ADDR = eeprom_local[port].offset | (eeprom_local[port].page << 6);
        regs->CMD = 0;

        while( (regs->INTSTAT & (1<<26)) == 0 ){ ; }
        regs->INTSTATCLR = (1<<26);

        *eeprom_local[port].buf = regs->RDATA;
        //wait until the previous data has been read

        eeprom_local[port].len--;
        eeprom_local[port].offset++;
        eeprom_local[port].buf++;
    }
}

void read_eeprom_halfword(int port){
    LPC_EEPROM_Type * regs = eeprom_regs[port];

    if( (eeprom_local[port].len > 1) && ((eeprom_local[port].offset & 0x01) == 0)  ){
        regs->ADDR = eeprom_local[port].offset | (eeprom_local[port].page << 6);
        regs->CMD = 1;

        while( (regs->INTSTAT & (1<<26)) == 0 ){ ; }
        regs->INTSTATCLR = (1<<26);

        *((u16*)(eeprom_local[port].buf)) = regs->RDATA;
        //wait until the previous data has been read

        eeprom_local[port].len-=2;
        eeprom_local[port].offset+=2;
        eeprom_local[port].buf+=2;
    }
}

void mcu_core_eeprom0_isr(){
    const int port = 0;
    LPC_EEPROM_Type * regs = eeprom_regs[port];
    u32 status = regs->INTSTAT;
    regs->INTSTATCLR = status;
    if( status & (1<<28) ){
        //this was a program/erase action
        if( eeprom_local[port].len > 0 ){

            while( (eeprom_local[port].offset < MCU_EEPROM_PAGE_SIZE) && (eeprom_local[port].len > 1) ) {
                write_eeprom_halfword(port);
            }

            write_eeprom_byte(port);

            regs->ADDR = eeprom_local[port].page << 6;
            regs->CMD = 6; //erase/program page
            eeprom_local[port].page++;
            eeprom_local[port].offset = 0;
            return;
        }
    }

    if( eeprom_local[port].len == 0 ){
        exec_callback(0, MCU_EVENT_FLAG_WRITE_COMPLETE|MCU_EVENT_FLAG_DATA_READY, 0);
    }
}

#endif

