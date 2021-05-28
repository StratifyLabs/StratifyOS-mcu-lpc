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
#include <mcu/adc.h>

#if MCU_ADC_PORTS > 0

#define ADC_PDN 21
#define ADC_START 24
#define ADC_DONE 31

typedef struct {
  mcu_event_handler_t handler;
  u32 *bufp;
  int len;
  u8 ref_count;
} adc_local_t;

static adc_local_t adc_local[MCU_ADC_PORTS] MCU_SYS_MEM;
static void exec_callback(int port, u32 o_events) MCU_PRIV_CODE;

static LPC_ADC_Type *const adc_regs[MCU_ADC_PORTS] = MCU_ADC_REGS;
static u8 const adc_irqs[MCU_ADC_PORTS] = MCU_ADC_IRQS;

static void enable_pin(const mcu_pin_t *pin, void *arg) MCU_PRIV_CODE;
void enable_pin(const mcu_pin_t *pin, void *arg) {
  MCU_UNUSED_ARGUMENT(arg);
  pio_attr_t pattr;
  devfs_handle_t pio_handle;
  pio_handle.config = 0;
  pio_handle.port = pin->port;
  pattr.o_pinmask = (1 << pin->pin);
  pattr.o_flags = PIO_FLAG_SET_INPUT | PIO_FLAG_IS_PULLUP | PIO_FLAG_IS_ANALOG;
  mcu_pio_setattr(&pio_handle, &pattr);
}

DEVFS_MCU_DRIVER_IOCTL_FUNCTION_MIN(adc, ADC_VERSION, ADC_IOC_IDENT_CHAR)

int mcu_adc_open(const devfs_handle_t *handle) {
  int port = handle->port;
  LPC_ADC_Type *regs = adc_regs[port];
  if (adc_local[port].ref_count == 0) {
    mcu_lpc_core_enable_pwr(PCADC);
    cortexm_enable_irq(adc_irqs[port]);
    regs->INTEN = 0;
    memset(&adc_local, 0, sizeof(adc_local_t));
  }
  adc_local[port].ref_count++;
  return 0;
}

int mcu_adc_close(const devfs_handle_t *handle) {
  int port = handle->port;
  LPC_ADC_Type *regs = adc_regs[port];
  if (adc_local[port].ref_count > 0) {
    if (adc_local[port].ref_count == 1) {
      regs->CR = 0; // reset the control -- clear the PDN bit
      cortexm_disable_irq(adc_irqs[port]);
      mcu_lpc_core_disable_pwr(PCADC);
    }
    adc_local[port].ref_count--;
  }
  return 0;
}

int mcu_adc_write(const devfs_handle_t *handle, devfs_async_t *async) {
  MCU_UNUSED_ARGUMENT(handle);
  MCU_UNUSED_ARGUMENT(async);
  return SYSFS_SET_RETURN(ENOTSUP);
}

int mcu_adc_read(const devfs_handle_t *handle, devfs_async_t *async) {
  const int port = handle->port;
  LPC_ADC_Type *regs = adc_regs[port];

  if ((uint8_t)async->loc > 7) {
    return SYSFS_SET_RETURN(EINVAL);
  }

  if (adc_local[port].handler.callback) {
    // The interrupt is on -- port is busy
    return SYSFS_SET_RETURN(EBUSY);
  }

  if (cortexm_validate_callback(async->handler.callback) < 0) {
    return SYSFS_SET_RETURN(EPERM);
  }

  adc_local[port].handler.callback = async->handler.callback;
  adc_local[port].handler.context = async->handler.context;
  adc_local[port].bufp = async->buf;
  adc_local[port].len = async->nbyte & ~(0x03);
  async->nbyte = adc_local[port].len;
  async->result = async->nbyte;

  regs->INTEN = (1 << 8);
  regs->CR |=
      ((1 << ADC_START) | (1 << async->loc)); // start the first conversion

  return 0;
}

void exec_callback(int port, u32 o_events) {
  LPC_ADC_Type *regs = adc_regs[port];
  adc_local[port].bufp = NULL;
  // Disable the interrupt
  regs->INTEN = 0;
  regs->CR &=
      ((1 << ADC_PDN) | (0xFF00)); // leave the clock div bits in place and PDN
  mcu_execute_event_handler(&(adc_local[port].handler), o_events, 0);
}

//! \todo Should this use DMA instead of this interrupt?
void mcu_core_adc0_isr();
void mcu_core_adc0_isr() {
  const int port = 0;
  LPC_ADC_Type *regs = adc_regs[0];

  if (adc_local[port].len > 0) {
    const u32 result = regs->GDR;
    *adc_local[port].bufp++ = result & 0xFFFF;
    adc_local[port].len = adc_local[port].len - sizeof(u32);
  }

  if (adc_local[port].len > 0) {
    regs->CR |= (1 << ADC_START); // set the start bit
  } else {
    exec_callback(0, MCU_EVENT_FLAG_DATA_READY);
  }
}

int mcu_adc_getinfo(const devfs_handle_t *handle, void *ctl) {
  adc_info_t *info = ctl;
  const adc_config_t *config = handle->config;

  info->freq = ADC_MAX_FREQ;
  info->o_flags = (ADC_FLAG_SET_CONVERTER |
                   ADC_FLAG_IS_RIGHT_JUSTIFIED);
  info->resolution = 12;
  info->bytes_per_sample = 4;
  info->maximum = 0xfff0;

  if (config) {
    info->reference_mv = config->reference_mv;
  } else {
    info->reference_mv = 0;
  }

  return 0;
}

int mcu_adc_setattr(const devfs_handle_t *handle, void *ctl) {
  int port = handle->port;
  LPC_ADC_Type *regs = adc_regs[port];
  u16 clk_div;
  u32 freq;

  const adc_attr_t *attr = mcu_select_attr(handle, ctl);
  if (attr == NULL) {
    return SYSFS_SET_RETURN(EINVAL);
  }

  freq = attr->freq;

  if (freq == 0) {
    freq = ADC_MAX_FREQ;
  } else if (freq > ADC_MAX_FREQ) {
    freq = ADC_MAX_FREQ;
  }

  if (mcu_set_pin_assignment(&(attr->pin_assignment),
                             MCU_CONFIG_PIN_ASSIGNMENT(adc_config_t, handle),
                             MCU_PIN_ASSIGNMENT_COUNT(adc_pin_assignment_t),
                             CORE_PERIPH_ADC, port, enable_pin, 0, 0) < 0) {
    return SYSFS_SET_RETURN(EINVAL);
  }

  // Calculate the clock setting
#ifdef __lpc17xx
  clk_div = (lpc_config.clock_peripheral_freq + freq*65/2) / (freq * 65);
#endif
#ifdef LPCXX7X_8X
  clk_div = (lpc_config.clock_peripheral_freq + freq*31/2) / (freq * 31);
#endif

  if (clk_div > 0) {
    clk_div = clk_div - 1;
  }
  clk_div = clk_div > 255 ? 255 : clk_div;

  regs->CR = (1 << ADC_PDN) | (clk_div << 8); // Set the clock bits
  regs->INTEN = 0;
  return 0;
}

int mcu_adc_setaction(const devfs_handle_t *handle, void *ctl) {
  int port = handle->port;

  mcu_action_t *action = (mcu_action_t *)ctl;
  if (action->handler.callback == 0) {
    if (action->o_events & MCU_EVENT_FLAG_DATA_READY) {
      exec_callback(port, MCU_EVENT_FLAG_CANCELED);
    }
  }

  if (cortexm_validate_callback(action->handler.callback) < 0) {
    return SYSFS_SET_RETURN(EPERM);
  }

  adc_local[port].handler.callback = action->handler.callback;
  adc_local[port].handler.context = action->handler.context;

  cortexm_set_irq_priority(adc_irqs[port], action->prio, action->o_events);

  return 0;
}

#endif
