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
 * along with Stratify OS.  If not, see <http://www.gnu.org/licenses/>. */



#include <mcu/bootloader.h>
#include "lpc_local.h"


static u32 mcu_core_get_reset_src();
static int enable_clock_out(int o_flags, int div);
static u32 mcu_core_reset_source = CORE_FLAG_IS_RESET_SOFTWARE;



int mcu_core_setpinfunc(const devfs_handle_t * handle, void * arg){
	core_pinfunc_t * argp = arg;
	return mcu_core_set_pinsel_func(&(argp->io),
			argp->periph_func,
			argp->periph_port);
}

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(core, CORE_VERSION, I_MCU_TOTAL + I_CORE_TOTAL, mcu_core_setpinfunc, mcu_core_setclkout, mcu_core_setclkdivide, mcu_core_getmcuboardconfig)

int mcu_core_open(const devfs_handle_t * handle){ return 0; }
int mcu_core_close(const devfs_handle_t * handle){ return 0; }
int mcu_core_dev_is_powered(const devfs_handle_t * handle){ return 1; }


int mcu_core_getinfo(const devfs_handle_t * handle, void * arg){
	core_info_t * info = arg;
	info->o_flags = 0;
	info->freq = mcu_board_config.core_cpu_freq;
	if( mcu_core_reset_source == CORE_FLAG_IS_RESET_SOFTWARE ){
		mcu_core_reset_source = mcu_core_get_reset_src();
	}

	info->o_flags |= mcu_core_reset_source;
	return mcu_lpc_flash_get_serialno(info->serial_number);
}

int mcu_core_setattr(const devfs_handle_t * handle, void * arg){
	int port = handle->port;
	core_attr_t * attr = arg;

	if( attr == 0 ){
		errno = EINVAL;
		return -1;
	}

	u32 o_flags = attr->o_flags;


	if( o_flags & CORE_FLAG_SET_CLKOUT ){
		enable_clock_out(o_flags, attr->freq);
	}

	if( o_flags & CORE_FLAG_EXEC_RESET ){
		mcu_core_reset(port, 0);
	}

	if( o_flags & CORE_FLAG_EXEC_INVOKE_BOOTLOADER ){
		mcu_core_invokebootloader(port, 0);
	}

	if( o_flags & CORE_FLAG_EXEC_SLEEP ){
		mcu_core_execsleep(port, (void*)CORE_SLEEP);
	} else if( o_flags & CORE_FLAG_EXEC_DEEPSLEEP ){
		mcu_core_execsleep(port, (void*)CORE_DEEPSLEEP);
	} else if( o_flags & CORE_FLAG_EXEC_DEEPSLEEP_STOP ){
		mcu_core_execsleep(port, (void*)CORE_DEEPSLEEP_STOP);
	} else if( o_flags & CORE_FLAG_EXEC_DEEPSLEEP_STANDBY ){
		mcu_core_execsleep(port, (void*)CORE_DEEPSLEEP_STANDBY);
	}


	return 0;
}

int mcu_core_setaction(const devfs_handle_t * handle, void * arg){
	errno = ENOTSUP;
	return -1;
}

int mcu_core_execsleep(int port, void * arg){
	int level;
	level = (int)arg;

	mcu_set_sleep_mode(&level);
	if ( level < 0 ){
		return level;
	}

	//Wait for an interrupts
	__WFI();
	return 0;
}

int mcu_core_reset(int port, void * arg){
	//delay first
	cortexm_delay_us(20*1000);
	cortexm_reset(NULL);
	//doesn't arrive here
	return 0;
}

int mcu_core_invokebootloader(int port, void * arg){
	cortexm_delay_us(500*1000);
	bootloader_api_t api;
	mcu_core_get_bootloader_api(&api);
    if( api.exec != 0 ){
        api.exec(0);
    } else {
        //best we can do is a regular reset
        cortexm_reset(NULL);
    }
	return 0;
}


int mcu_core_setclkout(const devfs_handle_t * handle, void * arg){
	//core_clkout_t * clkout = arg;
	//return
	return 0;
}

int mcu_core_setclkdivide(const devfs_handle_t * handle, void * arg){

#ifdef __lpc17xx
	//the errata on the LPC17xx chips prevent this from working correctly
	errno = ENOTSUP;
	return -1;
#endif

#ifdef LPCXX7X_8X
	u32 div = (int)arg;
	u32 clksel;

	if( div < 1 ){
		div = 1;
	}

	if( div > 31 ){
		div = 31;
	}

	clksel = LPC_SC->CCLKSEL & ~0x1F;
	clksel |= div;
	LPC_SC->CCLKSEL = clksel;
#endif

	return 0;
}

int mcu_core_getmcuboardconfig(const devfs_handle_t * handle, void * arg){
	memcpy(arg, &mcu_board_config, sizeof(mcu_board_config));
	return 0;
}


void mcu_set_sleep_mode(int * level){
	SCB->SCR &= ~(1<<SCB_SCR_SLEEPDEEP_Pos);

#if defined LPC_SC
	LPC_SC->PCON = 0;
	switch(*level){
	case CORE_DEEPSLEEP_STOP:
		LPC_SC->PCON = 1; //turn off the flash as well
		//no break
	case CORE_DEEPSLEEP:
		SCB->SCR |= (1<<SCB_SCR_SLEEPDEEP_Pos);
		break;
	case CORE_SLEEP:
		break;
	case CORE_DEEPSLEEP_STANDBY:
		SCB->SCR |= (1<<SCB_SCR_SLEEPDEEP_Pos);
		LPC_SC->PCON = 3;
		break;
	default:
		*level = -1;
		return;
	}
	*level = 0;
#endif
}

u32 mcu_core_get_reset_src(){
	u32 src = CORE_FLAG_IS_RESET_SOFTWARE;
	u32 src_reg;

#if defined LPC_SC
	src_reg = LPC_SC->RSID;
	LPC_SC->RSID = 0x0F;
#endif

	if ( src_reg & (1<<3) ){
		return CORE_FLAG_IS_RESET_BOR;
	}

	if ( src_reg & (1<<2) ){
		return CORE_FLAG_IS_RESET_WDT;
	}

	if ( src_reg & (1<<0) ){
		return CORE_FLAG_IS_RESET_POR;
	}

	if ( src_reg & (1<<1) ){
		return CORE_FLAG_IS_RESET_EXTERNAL;
	}

	return src;
}

int enable_clock_out(int o_flags, int div){
	div = (div & 0xF);
	if ( div != 0 ){
		div = div - 1;
	}

#if defined LPC_SC
	if( o_flags & CORE_FLAG_IS_CLKOUT_CPU ){
		LPC_SC->CLKOUTCFG = (1<<8)|(div<<4)|(0<<0);
	} else if( o_flags & CORE_FLAG_IS_CLKOUT_MAIN_OSC ){
		LPC_SC->CLKOUTCFG = (1<<8)|(div<<4)|(1<<0);
	} else if( o_flags & CORE_FLAG_IS_CLKOUT_INTERNAL_OSC ){
		LPC_SC->CLKOUTCFG = (1<<8)|(div<<4)|(2<<0);
	} else if( o_flags & CORE_FLAG_IS_CLKOUT_USB ){
		LPC_SC->CLKOUTCFG = (1<<8)|(div<<4)|(3<<0);
	} else if( o_flags & CORE_FLAG_IS_CLKOUT_RTC ){
		LPC_SC->CLKOUTCFG = (1<<8)|(div<<4)|(4<<0);
	} else {
		return -1;
	}
#endif

	mcu_pin_t pin = mcu_pin(1,27);
	mcu_core_set_pinsel_func(&pin, CORE_PERIPH_CORE, 0);
	return 0;

}

void mcu_core_set_nvic_priority(int irq, int prio){
	NVIC_SetPriority((IRQn_Type)irq, prio);
}

void mcu_core_get_bootloader_api(void * args){
	void * ptr;

    u32 * value = (u32*)36;

    if( *value != 0 ){
        memcpy(&ptr, (void*)(36), sizeof(void*)); //get pointer to boot api
        memcpy(args, ptr, sizeof(bootloader_api_t)); //copy boot api
    } else {
        memset(args, 0, sizeof(bootloader_api_t));
    }
}

int mcu_core_read(const devfs_handle_t * handle, devfs_async_t * rop){
    errno = ENOTSUP;
    return -1;
}

int mcu_core_write(const devfs_handle_t * handle, devfs_async_t * rop){
    errno = ENOTSUP;
    return -1;
}
