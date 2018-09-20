/* Copyright 2011-2017 Tyler Gilbert;
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

#include <sos/dev/appfs.h>
#include "lpc_local.h"

#if !defined SOS_GIT_HASH
#define SOS_GIT_HASH "??"
#endif

const mcu_config_t mcu_config = {
	.irq_total = MCU_LAST_IRQ,
	.irq_middle_prio = MCU_MIDDLE_IRQ_PRIORITY,
	.usb_logical_endpoint_count = DEV_USB_LOGICAL_ENDPOINT_COUNT,
	.delay_factor = MCU_DELAY_FACTOR,
	.git_hash = SOS_GIT_HASH
};


//for RAM usage there are 2 bits per page -- MCU_RAM_PAGES*2/32 words ->
u32 mcu_ram_usage_table[APPFS_RAM_USAGE_WORDS(MCU_RAM_PAGES)] MCU_SYS_MEM;

