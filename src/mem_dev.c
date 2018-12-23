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

#include <mcu/mem.h>
#include <mcu/bootloader.h>
#include "lpc_local.h"

#define MCU_RAM_PAGE_SIZE 1024

extern u32 _text;
extern u32 _etext;
extern u32 _data;
extern u32 _edata;

extern u32 _flash_pages;
extern u32 _flash_size;
extern u32 _sram_size;
extern u32 _ahb_sram_size;

#define END_OF_SRAM ((int)&_sram_size + MCU_START_OF_SRAM)
#define END_OF_AHB_SRAM ((int)&_ahb_sram_size + MCU_START_OF_AHB_SRAM)
#define SRAM_PAGES (((int)&_sram_size) / MCU_RAM_PAGE_SIZE)
#define AHB_SRAM_PAGES ((int)&_ahb_sram_size / MCU_RAM_PAGE_SIZE)

#define START_OS ((int)&_text)
#define END_OS ((int)&_etext + (int)&_edata - (int)&_data)

#define FLASH_SIZE ((u32)&_flash_size)

static int blank_check(int loc, int nbyte);
static bool is_flash(int addr, int size);
static int get_flash_page(int addr);
static int get_flash_page_size(int page);
static int get_flash_page_addr(int page);

static int get_last_boot_page();

static bool is_ram(int addr, int size);
static int get_ram_page(int addr);
static int get_ram_page_size(int page);
static int get_ram_page_addr(int page);

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(mem, MEM_VERSION, MEM_IOC_IDENT_CHAR, I_MCU_TOTAL + I_MEM_TOTAL, mcu_mem_erasepage, mcu_mem_getpageinfo, mcu_mem_writepage)

int mcu_mem_open(const devfs_handle_t * handle){ return 0; }
int mcu_mem_close(const devfs_handle_t * handle){ return 0; }

int mcu_mem_getsyspage(){
    //this returns the page where the system memory is stored -- for LPC it is the page after SRAM (AHB SRAM)
    return (SRAM_PAGES);
}

int mcu_mem_getinfo(const devfs_handle_t * handle, void * ctl){
    mem_info_t * info = ctl;
    info->flash_pages = (u32)&_flash_pages;
    info->flash_size = FLASH_SIZE;
    info->ram_pages = (u32)&_flash_pages;
    info->ram_size = (u32)&_sram_size + (u32)&_ahb_sram_size;
    return 0;
}
int mcu_mem_setattr(const devfs_handle_t * handle, void * ctl){
    return 0;
}

int mcu_mem_setaction(const devfs_handle_t * handle, void * ctl){
    return SYSFS_SET_RETURN(ENOTSUP);
}

int mcu_mem_getpageinfo(const devfs_handle_t * handle, void * ctl){
    u32 size = 0;
    int32_t addr = 0;
    mem_pageinfo_t * ctlp = ctl;


    if( ctlp->o_flags & MEM_FLAG_IS_RAM ){

        size = get_ram_page_size(ctlp->num);
        addr = get_ram_page_addr(ctlp->num);
        if ( addr < 0 ){
            return SYSFS_SET_RETURN(EINVAL);
        }

    } else if( ctlp->o_flags & MEM_FLAG_IS_FLASH ){

        size = get_flash_page_size(ctlp->num);
        addr = get_flash_page_addr(ctlp->num);
        if ( (addr + size) > FLASH_SIZE){
            return SYSFS_SET_RETURN(EINVAL);
        }

    } else if( ctlp->o_flags & MEM_FLAG_IS_QUERY ){

        //Query needs to see if addr is RAM or FLASH
        if ( is_ram(ctlp->addr, MCU_RAM_PAGE_SIZE) ){
            ctlp->num = get_ram_page(ctlp->addr);
            ctlp->size = get_ram_page_size(ctlp->num);
            ctlp->o_flags = MEM_FLAG_IS_RAM;
            return 0;
        } else if ( is_flash(ctlp->addr, 0) ){
            ctlp->num = get_flash_page(ctlp->addr);
            ctlp->size = get_flash_page_size(ctlp->num);
            ctlp->o_flags = MEM_FLAG_IS_FLASH;
            return 0;
        } else {
            return SYSFS_SET_RETURN(EINVAL);
        }

    } else {
        return SYSFS_SET_RETURN(EINVAL);
    }



    ctlp->size = size;
    ctlp->addr = addr;
    return 0;
}



int mcu_mem_erasepage(const devfs_handle_t * handle, void * ctl){
    int err;
    int addr;
    int page;
    page = (u32)ctl;
    addr = get_flash_page_addr(page);  //this gets the beginning of the page
    int last_boot_page = get_last_boot_page();

    //protect the OS and the bootloader from being erased
    if ( (page <= last_boot_page) ||
         ((addr >= START_OS) && (addr <= END_OS)) ){
        return SYSFS_SET_RETURN(EROFS);
    }

	 cortexm_disable_interrupts();
    err = mcu_lpc_flash_erase_page((u32)ctl);
	 cortexm_enable_interrupts();
    if ( err < 0 ){
        return SYSFS_SET_RETURN(EIO);
    }
    return 0;
}

int mcu_mem_writepage(const devfs_handle_t * handle, void * ctl){
    int err;
    int nbyte;
    mem_writepage_t * wattr = ctl;
    int last_boot_page;
    int page;

    nbyte = wattr->nbyte;
    if( nbyte > 256 ){
        nbyte = 256;
    }

    if ( is_ram(wattr->addr, nbyte) ){
        memcpy((void*)wattr->addr, wattr->buf, nbyte);
        return nbyte;
    }

    if( nbyte < 256 ){
        nbyte = 256;
    }

    page = get_flash_page(wattr->addr);
    last_boot_page = get_last_boot_page();

    if ( page <= last_boot_page ){
        return SYSFS_SET_RETURN(EROFS);
    }

    if ( wattr->addr >= FLASH_SIZE ){
        return SYSFS_SET_RETURN(EINVAL);
    }

    if ( wattr->addr + nbyte > FLASH_SIZE ){
        return SYSFS_SET_RETURN(EINVAL);
    }


    if ( blank_check(wattr->addr,  nbyte) ){
        return SYSFS_SET_RETURN(EROFS);
    }

    err = mcu_lpc_flash_write_page(page, (void*)wattr->addr, wattr->buf, nbyte);
    if( err < 0 ){
        return SYSFS_SET_RETURN(EIO);
    }

    return nbyte;
}

int mcu_mem_write(const devfs_handle_t * cfg, devfs_async_t * wop){

    if ( is_ram(wop->loc, wop->nbyte) ){
        memcpy((void*)wop->loc, wop->buf, wop->nbyte);
        return wop->nbyte;
    }

    return SYSFS_SET_RETURN(EINVAL);
}

int mcu_mem_read(const devfs_handle_t * cfg, devfs_async_t * rop){
    if ( (is_flash(rop->loc, rop->nbyte) ) ||
         ( is_ram(rop->loc, rop->nbyte) ) 	){
        memcpy(rop->buf, (const void*)rop->loc, rop->nbyte);
        return rop->nbyte;
    }
    return SYSFS_SET_RETURN(EINVAL);
}

//Get the flash page that contains the address
int get_flash_page(int addr){
    if ( addr < 0x10000 ){
        return addr / 4096;
    }
    addr -= 0x10000;
    return 16 + addr / 32768;
}

int get_flash_page_size(int page){
    if ( page < 16 ){
        return 4*1024;
    } else {
        return 32*1024;
    }
}

int get_flash_page_addr(int page){
    if ( page < 16 ){
        return page * 4096;
    } else {
        return (16*4096) + ((page-16)*32*1024);
    }
}

bool is_flash(int addr, int size){
    if ( (addr + size) <= FLASH_SIZE ){
        return true;
    }
    return false;
}

//RAM paging
int get_ram_page(int addr){
    int page;
    int offset;
    int page_offset;
    if ( (addr >= MCU_START_OF_AHB_SRAM) && (addr < END_OF_AHB_SRAM) ){
        offset = MCU_START_OF_AHB_SRAM;
        page_offset = SRAM_PAGES;
    } else if ( (addr >= MCU_START_OF_SRAM) && (addr < END_OF_SRAM) ){
        offset = MCU_START_OF_SRAM;
        page_offset = 0;
    } else {
        return -1;
    }
    page = (addr - offset) / MCU_RAM_PAGE_SIZE + page_offset;
    return page;
}

int get_ram_page_size(int page){
    int size;
    size = MCU_RAM_PAGE_SIZE;
    return size;
}

int get_ram_page_addr(int page){
    if ( page < SRAM_PAGES ){
        return MCU_START_OF_SRAM + page*MCU_RAM_PAGE_SIZE;
    } else if ( page < (SRAM_PAGES + AHB_SRAM_PAGES)){
        return MCU_START_OF_AHB_SRAM + (page - SRAM_PAGES)*MCU_RAM_PAGE_SIZE;
    }

    return -1;
}

bool is_ram(int addr, int size){
    if ( (addr >= MCU_START_OF_SRAM) && ((addr + size) <= END_OF_SRAM) ){
        return true;
    }

    if ( (addr >= MCU_START_OF_AHB_SRAM) && ((addr + size) <= END_OF_AHB_SRAM) ){
        return true;
    }

    return false;
}

int blank_check(int loc, int nbyte){
    int i;
    const int8_t * locp;
    //Do a blank check
    locp = (const int8_t*)loc;
    for(i = 0; i < nbyte; i++){
        if ( locp[i] != -1 ){
            return -1;
        }
    }
    return 0;
}

int get_last_boot_page(){
    bootloader_api_t api;
    mcu_core_get_bootloader_api(&api);

    if( api.code_size > 0 ){ //zero means there is not bootloader installed
        return get_flash_page(api.code_size);
    }

    return -1;

}

