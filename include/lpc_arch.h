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

#ifndef LPC_ARCH_H_
#define LPC_ARCH_H_

#include <sdk/types.h>

#ifdef __lpc17xx
#include "mcu_lpc17xx.h"
#endif

#ifdef __lpc177x_8x
#include "mcu_lpc177x_8x.h"
#endif

#ifdef __lpc407x_8x
#define CORE_M4 1
#include "mcu_lpc407x_8x.h"
#endif

#ifdef __lpc43xx
#define CORE_M4 1
#include "mcu_lpc43xx.h"
#endif

typedef struct {
  u32 max_packet_zero;
} lpc_usb_config_t;

typedef struct {
  u32 clock_oscillator_freq;
  u32 clock_peripheral_freq;
  lpc_usb_config_t usb;
} lpc_config_t;

void lpc_clock_initialize(
    int (*handle_match_channel0)(void *context, const mcu_event_t *data),
    int (*handle_match_channel1)(void *context, const mcu_event_t *data),
    int (*handle_overflow)(void *context, const mcu_event_t *data));
void lpc_clock_enable();
u32 lpc_clock_disable();
void lpc_clock_set_channel(const mcu_channel_t *channel);
void lpc_clock_get_channel(mcu_channel_t *channel);
u32 lpc_clock_microseconds();
u32 lpc_clock_nanoseconds();

extern const lpc_config_t lpc_config;

#endif /* LPC_ARCH_H_ */
