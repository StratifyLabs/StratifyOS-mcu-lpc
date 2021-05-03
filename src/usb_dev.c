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

#include <mcu/usb.h>
#include <sos/boot/boot_debug.h>

#include "core_local.h"
#include "lpc_local.h"
#include "usb_flags.h"

#if MCU_USB_PORTS > 0

static u32 usb_irq_mask;
static void usb_connect(u32 con);
static inline void usb_configure(const devfs_handle_t *handle,
                                 u32 cfg) MCU_ALWAYS_INLINE;
static inline void usb_set_address(const devfs_handle_t *handle,
                                   u32 addr) MCU_ALWAYS_INLINE;
static inline void usb_reset_endpoint(const devfs_handle_t *handle,
                                      u32 endpoint_num) MCU_ALWAYS_INLINE;
static inline void usb_enable_endpoint(const devfs_handle_t *handle,
                                       u32 endpoint_num) MCU_ALWAYS_INLINE;
static inline void usb_disable_endpoint(const devfs_handle_t *handle,
                                        u32 endpoint_num) MCU_ALWAYS_INLINE;
static inline void usb_stall_endpoint(const devfs_handle_t *handle,
                                      u32 endpoint_num) MCU_ALWAYS_INLINE;
static inline void usb_unstall_endpoint(const devfs_handle_t *handle,
                                        u32 endpoint_num) MCU_ALWAYS_INLINE;
static inline void
usb_configure_endpoint(const devfs_handle_t *handle, u32 endpoint_num,
                       u32 max_packet_size) MCU_ALWAYS_INLINE;
static inline void usb_reset(const devfs_handle_t *handle);
static inline void slow_ep_int() MCU_ALWAYS_INLINE;

static void set_usb_clock();

typedef struct {
  mcu_event_handler_t control_read;
  mcu_event_handler_t control_write;
  devfs_transfer_handler_t endpoint_handlers[DEV_USB_LOGICAL_ENDPOINT_COUNT];
  volatile u32 read_ready;
  mcu_event_handler_t special_event_handler;
  u8 ref_count;
  u8 connected;
  u8 address;
} usb_local_t;

static usb_local_t m_usb_local[MCU_USB_PORTS] MCU_SYS_MEM;

static void clear_callbacks(usb_local_t *local);
void clear_callbacks(usb_local_t *local) {
  memset(&local->control_read, 0, sizeof(local->control_read));
  memset(&local->control_write, 0, sizeof(local->control_write));
  memset(&local->endpoint_handlers, 0, sizeof(local->endpoint_handlers));
}

#define EP_MSK_CTRL 0x0001 // Control Endpoint Logical Address Mask
#define EP_MSK_BULK 0xC924 // Bulk Endpoint Logical Address Mask
#define EP_MSK_INT 0x4492 // Interrupt Endpoint Logical Address Mask
#define EP_MSK_ISO 0x1248 // Isochronous Endpoint Logical Address Mask

static u32 calc_ep_addr(u32 endpoint_num);

// The following should be static
static void usb_sie_wr_cmd(u32 cmd) MCU_NEVER_INLINE;
static u32 usb_sie_rd_dat() MCU_NEVER_INLINE;
static void usb_sie_wr_cmd_dat(u32 cmd, u32 val);
static void usb_sie_wr_cmd_ep(u32 ep_num, u32 cmd);
static u32 usb_sie_rd_cmd_dat(u32 cmd);

#ifdef LPCXX7X_8X
static void configure_pin(const mcu_pin_t *pin, void *arg) {
  if (pin->pin == 31) {
    LPC_USB->StCtrl |= 0x03;
  }
}
#else
#define configure_pin 0
#endif

void usb_sie_wr_cmd(u32 cmd) {
  LPC_USB->DevIntClr = CCEMTY_INT | CDFULL_INT;
  LPC_USB->CmdCode = cmd;
  while ((LPC_USB->DevIntSt & CCEMTY_INT) == 0)
    ;
}

u32 usb_sie_rd_dat() {
  while ((LPC_USB->DevIntSt & CDFULL_INT) == 0)
    ;
  return LPC_USB->CmdData;
}

void usb_sie_wr_cmd_dat(u32 cmd, u32 val) {
  usb_sie_wr_cmd(cmd);
  usb_sie_wr_cmd(val);
}

void usb_sie_wr_cmd_ep(u32 endpoint_num, u32 cmd) {
  usb_sie_wr_cmd(USB_SIE_CMD_SEL_EP(calc_ep_addr(endpoint_num)));
  usb_sie_wr_cmd(cmd);
}

u32 usb_sie_rd_cmd_dat(u32 cmd) {
  u32 dat;
  dat = (cmd & ~0xFFFF) | 0x0200;
  usb_sie_wr_cmd(cmd);
  usb_sie_wr_cmd(dat);
  return usb_sie_rd_dat();
}

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(usb, USB_VERSION, USB_IOC_IDENT_CHAR,
                                I_MCU_TOTAL + I_USB_TOTAL, mcu_usb_isconnected)

int mcu_usb_open(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_LOCAL(usb, MCU_USB_PORTS);
  if (local->ref_count == 0) {
    // Set callbacks to NULL
    local->connected = 0;
    clear_callbacks(local);
    local->special_event_handler.callback = 0;
    mcu_lpc_core_enable_pwr(PCUSB);
    LPC_USB->USBClkCtrl = 0x12; // turn on dev clk en and AHB clk en
    while (LPC_USB->USBClkCtrl != 0x12) {
    } // wait for clocks
  }
  local->ref_count++;
  return 0;
}

int mcu_usb_close(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_LOCAL(usb, MCU_USB_PORTS);
  if (local->ref_count > 0) {
    if (local->ref_count == 1) {
      cortexm_disable_irq((USB_IRQn)); // Enable the USB interrupt
      LPC_USB->USBClkCtrl = 0x0;       // turn off dev clk en and AHB clk en
      while (LPC_USB->USBClkCtrl != 0) {
      }
      mcu_lpc_core_disable_pwr(PCUSB);
    }
    local->ref_count--;
  }
  return 0;
}

int mcu_usb_getinfo(const devfs_handle_t *handle, void *ctl) {
  MCU_UNUSED_ARGUMENT(handle);
  usb_info_t *info = ctl;
  info->o_flags = USB_FLAG_SET_DEVICE | USB_FLAG_RESET | USB_FLAG_ATTACH |
                  USB_FLAG_DETACH | USB_FLAG_CONFIGURE | USB_FLAG_UNCONFIGURE |
                  USB_FLAG_SET_ADDRESS | USB_FLAG_RESET_ENDPOINT |
                  USB_FLAG_ENABLE_ENDPOINT | USB_FLAG_DISABLE_ENDPOINT |
                  USB_FLAG_STALL_ENDPOINT | USB_FLAG_UNSTALL_ENDPOINT |
                  USB_FLAG_CONFIGURE_ENDPOINT | 0;
  info->o_events = 0;
  return 0;
}

int mcu_usb_setattr(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_LOCAL(usb, MCU_USB_PORTS);

  const usb_attr_t *attr = mcu_select_attr(handle, ctl);
  if (attr == 0) {
    return SYSFS_SET_RETURN(EINVAL);
  }
  u32 o_flags = attr->o_flags;

  if (o_flags & USB_FLAG_SET_DEVICE) {
    // Start the USB clock
    set_usb_clock();

    local->read_ready = 0;

#ifdef LPCXX7X_8X
    LPC_USB->USBClkCtrl &= ~(1 << 3); // disable portsel clock
    while ((LPC_USB->USBClkSt & (1 << 3)) != 0) {
    }
#endif

    if (mcu_set_pin_assignment(&(attr->pin_assignment),
                               MCU_CONFIG_PIN_ASSIGNMENT(usb_config_t, handle),
                               MCU_PIN_ASSIGNMENT_COUNT(usb_pin_assignment_t),
                               CORE_PERIPH_USB, port, configure_pin, 0,
                               0) < 0) {
      return SYSFS_SET_RETURN(EINVAL);
    }

#ifdef LPCXX7X_8X
    LPC_USB->USBClkCtrl |= (1 << 3); // enable portsel clock
    while ((LPC_USB->USBClkSt & (1 << 3)) == 0) {
    }
#endif

    usb_irq_mask = DEV_STAT_INT | EP_FAST_INT | EP_SLOW_INT;

    cortexm_enable_irq(USB_IRQn); // Enable the USB interrupt
    usb_reset(handle);
    local->address = 0;
    usb_set_address(handle, 0);
  }

  if (o_flags & USB_FLAG_RESET) {
    usb_reset(handle);
  }
  if (o_flags & USB_FLAG_ATTACH) {
    usb_connect(1);
  }
  if (o_flags & USB_FLAG_DETACH) {
    usb_connect(0);
  }
  if (o_flags & USB_FLAG_CONFIGURE) {
    usb_configure(handle, 1);
  }
  if (o_flags & USB_FLAG_UNCONFIGURE) {
    usb_configure(handle, 0);
  }

  if (o_flags & USB_FLAG_SET_ADDRESS) {
    // store the address until the status stage is over
    local->address = attr->address;
  }

  if (o_flags & USB_FLAG_RESET_ENDPOINT) {
    usb_reset_endpoint(handle, attr->address);
  }

  if (o_flags & USB_FLAG_ENABLE_ENDPOINT) {
    usb_enable_endpoint(handle, attr->address);
  }

  if (o_flags & USB_FLAG_DISABLE_ENDPOINT) {
    usb_disable_endpoint(handle, attr->address);
  }

  if (o_flags & USB_FLAG_STALL_ENDPOINT) {
    usb_stall_endpoint(handle, attr->address);
  }

  if (o_flags & USB_FLAG_UNSTALL_ENDPOINT) {
    usb_unstall_endpoint(handle, attr->address);
  }

  if (o_flags & USB_FLAG_CONFIGURE_ENDPOINT) {
    usb_configure_endpoint(handle, attr->address, attr->max_packet_size);
  }

  return 0;
}

void usb_connect(u32 con) {
  usb_sie_wr_cmd_dat(USB_SIE_CMD_SET_DEV_STAT,
                     USB_SIE_DAT_WR_BYTE(con ? DEV_CON : 0));
}

int mcu_usb_setaction(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_LOCAL(usb, MCU_USB_PORTS);
  mcu_action_t *action = (mcu_action_t *)ctl;
  int log_ep;
  int result = -1;

  cortexm_set_irq_priority(USB_IRQn, action->prio, action->o_events);
  log_ep = action->channel & ~0x80;

  if (action->o_events &
      (MCU_EVENT_FLAG_POWER | MCU_EVENT_FLAG_SUSPEND | MCU_EVENT_FLAG_STALL |
       MCU_EVENT_FLAG_SOF | MCU_EVENT_FLAG_WAKEUP)) {
    local->special_event_handler = action->handler;
    return 0;
  }

  if (action->handler.callback == 0) {

    // cancel any pending actions and execute the callback
    if (action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE) {
      devfs_execute_write_handler(&local->endpoint_handlers[log_ep], 0, 0,
                                  MCU_EVENT_FLAG_CANCELED);
      if (log_ep == 0) {
        mcu_execute_event_handler(&local->control_write,
                                  MCU_EVENT_FLAG_CANCELED, 0);
      } else {
        devfs_execute_write_handler(&local->endpoint_handlers[log_ep], 0, 0,
                                    MCU_EVENT_FLAG_CANCELED);
      }
    }

    if (action->o_events & MCU_EVENT_FLAG_DATA_READY) {
      local->read_ready &= ~(1 << log_ep);
      if (log_ep == 0) {
        mcu_execute_event_handler(&local->control_read, MCU_EVENT_FLAG_CANCELED,
                                  0);
      } else {
        devfs_execute_read_handler(&local->endpoint_handlers[log_ep], 0, 0,
                                   MCU_EVENT_FLAG_CANCELED);
      }
    }

  } else if (action->channel == 0) {

    if (cortexm_validate_callback(action->handler.callback) < 0) {
      return SYSFS_SET_RETURN(EPERM);
    }

    // setup the control callback for channel zero
    if (action->o_events & MCU_EVENT_FLAG_DATA_READY) {
      local->control_read.callback = action->handler.callback;
      local->control_read.context = action->handler.context;
      result = 0;
    }

    if (action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE) {
      local->control_write.callback = action->handler.callback;
      local->control_write.context = action->handler.context;
      result = 0;
    }
  } else {
    return SYSFS_SET_RETURN(EINVAL);
  }

  if (result < 0) {
    result = SYSFS_SET_RETURN(EINVAL);
  }
  return result;
}

int mcu_usb_read(const devfs_handle_t *handle, devfs_async_t *async) {
  DEVFS_DRIVER_DECLARE_LOCAL(usb, MCU_USB_PORTS);
  int ret;
  int loc = async->loc;

  if (loc > (DEV_USB_LOGICAL_ENDPOINT_COUNT - 1)) {
    return SYSFS_SET_RETURN(EINVAL);
  }

  DEVFS_DRIVER_IS_BUSY(local->endpoint_handlers[loc].read, async);

  // Synchronous read (only if data is ready) otherwise 0 is returned
  if (local->read_ready & (1 << loc)) {
    local->read_ready &= ~(1 << loc); // clear the read ready bit
    ret = mcu_usb_root_read_endpoint(handle, loc, async->buf);
  } else {
    // data won't be ready until another call to this
    async->nbyte = 0;
    if (!(async->flags & O_NONBLOCK)) {
      // If this is a blocking call, set the callback and context
      if (cortexm_validate_callback(async->handler.callback) < 0) {
        return SYSFS_SET_RETURN(EPERM);
      }
      ret = 0;
    } else {
      ret = SYSFS_SET_RETURN(EAGAIN);
    }
  }

  if (ret != 0) {
    local->endpoint_handlers[loc].read = 0;
  }

  return ret;
}

int mcu_usb_write(const devfs_handle_t *handle, devfs_async_t *async) {
  // Asynchronous write
  DEVFS_DRIVER_DECLARE_LOCAL(usb, MCU_USB_PORTS);
  int ep;
  int loc = async->loc;

  ep = (loc & 0x7F);

  if (ep > (DEV_USB_LOGICAL_ENDPOINT_COUNT - 1)) {
    return SYSFS_SET_RETURN(EINVAL);
  }

  DEVFS_DRIVER_IS_BUSY(local->endpoint_handlers[ep].write, async);

  async->nbyte =
      mcu_usb_root_write_endpoint(handle, loc, async->buf, async->nbyte);

  if (async->nbyte < 0) {
    usb_disable_endpoint(handle, loc);
    usb_reset_endpoint(handle, loc);
    usb_enable_endpoint(handle, loc);
    local->endpoint_handlers[loc].write = 0;
    return SYSFS_SET_RETURN(EIO);
  }

  return 0;
}

u32 calc_ep_addr(u32 endpoint_num) {
  u32 val;
  val = (endpoint_num & 0x0F) << 1;
  if (endpoint_num & 0x80) {
    val += 1;
  }
  return (val);
}

void usb_reset(const devfs_handle_t *handle) {
  MCU_UNUSED_ARGUMENT(handle);
  // Set max packet size of phy ep 0
  LPC_USB->EpInd = 0;
  LPC_USB->MaxPSize = lpc_config.usb.max_packet_zero;

  // set max packet size of phy ep 1
  LPC_USB->EpInd = 1;
  LPC_USB->MaxPSize = lpc_config.usb.max_packet_zero;

  while ((LPC_USB->DevIntSt & EP_RLZED_INT) == 0) {
  }

  LPC_USB->EpIntClr = 0xFFFFFFFF;
  LPC_USB->EpIntEn = 0xFFFFFFFF ^ USB_DMA_EP;
  LPC_USB->DevIntClr = 0xFFFFFFFF;
  LPC_USB->DevIntEn = usb_irq_mask;
}

void usb_wakeup(int port) {
  MCU_UNUSED_ARGUMENT(port);
  usb_sie_wr_cmd_dat(USB_SIE_CMD_SET_DEV_STAT, USB_SIE_DAT_WR_BYTE(DEV_CON));
}

void usb_set_address(const devfs_handle_t *handle, u32 addr) {
  MCU_UNUSED_ARGUMENT(handle);
  usb_sie_wr_cmd_dat(USB_SIE_CMD_SET_ADDR,
                     USB_SIE_DAT_WR_BYTE(DEV_EN | addr)); // Don't wait for next
  usb_sie_wr_cmd_dat(USB_SIE_CMD_SET_ADDR,
                     USB_SIE_DAT_WR_BYTE(DEV_EN | addr)); //  Setup Status Phase
}

void usb_configure(const devfs_handle_t *handle, u32 cfg) {
  MCU_UNUSED_ARGUMENT(handle);
  usb_sie_wr_cmd_dat(USB_SIE_CMD_CFG_DEV,
                     USB_SIE_DAT_WR_BYTE(cfg ? CONF_DVICE : 0));
}

void usb_configure_endpoint(const devfs_handle_t *handle, u32 endpoint_num,
                            u32 max_packet_size) {
  u32 num;
  MCU_UNUSED_ARGUMENT(handle);
  num = calc_ep_addr(endpoint_num);
  LPC_USB->ReEp |= (1 << num);
  LPC_USB->EpInd = num;
  LPC_USB->MaxPSize = max_packet_size;
  while ((LPC_USB->DevIntSt & EP_RLZED_INT) == 0)
    ;
  LPC_USB->DevIntClr = EP_RLZED_INT;
}

void usb_enable_endpoint(const devfs_handle_t *handle, u32 endpoint_num) {
  MCU_UNUSED_ARGUMENT(handle);
  usb_sie_wr_cmd_dat(USB_SIE_CMD_SET_EP_STAT(calc_ep_addr(endpoint_num)),
                     USB_SIE_DAT_WR_BYTE(0));
}

void usb_disable_endpoint(const devfs_handle_t *handle, u32 endpoint_num) {
  MCU_UNUSED_ARGUMENT(handle);
  usb_sie_wr_cmd_dat(USB_SIE_CMD_SET_EP_STAT(calc_ep_addr(endpoint_num)),
                     USB_SIE_DAT_WR_BYTE(EP_STAT_DA));
}

void usb_reset_endpoint(const devfs_handle_t *handle, u32 endpoint_num) {
  MCU_UNUSED_ARGUMENT(handle);
  usb_sie_wr_cmd_dat(USB_SIE_CMD_SET_EP_STAT(calc_ep_addr(endpoint_num)),
                     USB_SIE_DAT_WR_BYTE(0));
}

void usb_stall_endpoint(const devfs_handle_t *handle, u32 endpoint_num) {
  MCU_UNUSED_ARGUMENT(handle);
  usb_sie_wr_cmd_dat(USB_SIE_CMD_SET_EP_STAT(calc_ep_addr(endpoint_num)),
                     USB_SIE_DAT_WR_BYTE(EP_STAT_ST));
}

void usb_unstall_endpoint(const devfs_handle_t *handle, u32 endpoint_num) {
  MCU_UNUSED_ARGUMENT(handle);
  usb_sie_wr_cmd_dat(USB_SIE_CMD_SET_EP_STAT(calc_ep_addr(endpoint_num)),
                     USB_SIE_DAT_WR_BYTE(0));
}

int mcu_usb_isconnected(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_LOCAL(usb, MCU_USB_PORTS);
  MCU_UNUSED_ARGUMENT(ctl);
  return local->connected;
}

void usb_clr_ep_buf(const devfs_handle_t *handle, u32 endpoint_num) {
  MCU_UNUSED_ARGUMENT(handle);
  usb_sie_wr_cmd_ep(endpoint_num, USB_SIE_CMD_CLR_BUF);
}

int mcu_usb_root_read_endpoint(const devfs_handle_t *handle, u32 endpoint_num,
                               void *dest) {
  MCU_UNUSED_ARGUMENT(handle);
  u32 n;
  u32 *ptr;
  u32 size;
  u32 mask;

  LPC_USB->Ctrl = ((endpoint_num & 0x0F) << 2) | CTRL_RD_EN;

  // There needs to be a delay between the time USBCtrl is set and reading
  // USBRxPLen due to the difference in clocks
  if (sos_config.sys.core_clock_frequency > 48000000) {
    u32 i;
    for (i = 0; i < sos_config.sys.core_clock_frequency / 4000000; i++) {
      asm volatile("nop");
    }
  }

  ptr = (u32 *)dest;
  n = 0;
  mask = ((EP_MSK_ISO >> endpoint_num) & 1);
  size = LPC_USB->RxPLen;

  while ((LPC_USB->Ctrl & CTRL_RD_EN) && (n < 64)) {
    *ptr++ = LPC_USB->RxData;
    n += 4;
  }

  if (mask == 0) { // Non-Isochronous Endpoint
    // Clear the buffer so more data can be received
    usb_sie_wr_cmd_ep(endpoint_num, USB_SIE_CMD_CLR_BUF);
  }

  return size & 0x3FF;
}

/*! \details
 */
int mcu_usb_root_write_endpoint(const devfs_handle_t *handle, u32 endpoint_num,
                                const void *src, u32 size) {
  MCU_UNUSED_ARGUMENT(handle);
  u32 n;
  u32 *ptr = (u32 *)src;

  LPC_USB->Ctrl = ((endpoint_num & 0x0F) << 2) | CTRL_WR_EN;
  LPC_USB->TxPLen = size;

  n = 0;
  while (LPC_USB->Ctrl & CTRL_WR_EN) {
    if (n < size) {
      LPC_USB->TxData = *ptr++;
    } else {
      LPC_USB->TxData = 0;
    }
    n += 4;
  }

  usb_sie_wr_cmd_ep(endpoint_num, USB_SIE_CMD_VALID_BUF);
  return size;
}

void mcu_core_usb0_isr() {
  u32 device_interrupt_status;
  u32 tmp;
  int i;

  device_interrupt_status = LPC_USB->DevIntSt; // Device interrupt status
  if (device_interrupt_status & ERR_INT) {     // Error interrupt
    tmp = usb_sie_rd_cmd_dat(USB_SIE_CMD_RD_ERR_STAT);
    mcu_execute_event_handler(&(m_usb_local[0].special_event_handler),
                              MCU_EVENT_FLAG_ERROR, 0);
    LPC_USB->DevIntClr = ERR_INT;
  }

  if (device_interrupt_status & FRAME_INT) { // start of frame
    mcu_execute_event_handler(&(m_usb_local[0].special_event_handler),
                              MCU_EVENT_FLAG_SOF, 0);
    LPC_USB->DevIntClr = FRAME_INT;
  }

  if (device_interrupt_status &
      DEV_STAT_INT) { // Status interrupt (Reset, suspend/resume or connect)
    cortexm_delay_us(100);
    tmp = usb_sie_rd_cmd_dat(USB_SIE_CMD_GET_DEV_STAT);
    if (tmp & DEV_RST) {
      // mcu_usb_reset(0, NULL);
      m_usb_local[0].connected = 1;
      m_usb_local[0].read_ready = 0;
      mcu_execute_event_handler(&(m_usb_local[0].special_event_handler),
                                MCU_EVENT_FLAG_RESET, 0);
    }

    if (tmp == 0x0D) {
      m_usb_local[0].connected = 0;
      m_usb_local[0].read_ready = 0;
      for (i = 1; i < DEV_USB_LOGICAL_ENDPOINT_COUNT; i++) {
        devfs_execute_cancel_handler(&m_usb_local[0].endpoint_handlers[i], 0,
                                     SYSFS_SET_RETURN(EIO),
                                     MCU_EVENT_FLAG_CANCELED);
        // mcu_execute_event_handler(&(m_usb_local.read[i]),
        // MCU_EVENT_FLAG_CANCELED, 0);
        // mcu_execute_event_handler(&(m_usb_local.write[i]),
        // MCU_EVENT_FLAG_CANCELED, 0);
      }
    }

    if (tmp & DEV_CON_CH) {
      mcu_execute_event_handler(&(m_usb_local[0].special_event_handler),
                                MCU_EVENT_FLAG_POWER, 0);
    }

    if (tmp & DEV_SUS_CH) {
      if (tmp & DEV_SUS) {
        m_usb_local[0].connected = 0;
        mcu_execute_event_handler(&(m_usb_local[0].special_event_handler),
                                  MCU_EVENT_FLAG_SUSPEND, 0);
      } else {
        m_usb_local[0].connected = 1;
        mcu_execute_event_handler(&(m_usb_local[0].special_event_handler),
                                  MCU_EVENT_FLAG_RESUME, 0);
      }
    }

    LPC_USB->DevIntClr = DEV_STAT_INT | CCEMTY_INT | CDFULL_INT | RxENDPKT_INT |
                         TxENDPKT_INT | EP_RLZED_INT;
    return;
  }

  if (device_interrupt_status & EP_SLOW_INT) { // Slow endpoint interrupt
    slow_ep_int();
  }

  LPC_USB->DevIntClr =
      CCEMTY_INT | CDFULL_INT | RxENDPKT_INT | TxENDPKT_INT | EP_RLZED_INT;
  return;
}

void slow_ep_int() {
  u32 episr;
  u32 phy_ep, log_ep;
  u32 tmp;
  episr = LPC_USB->EpIntSt;
  usb_event_t event;

  for (phy_ep = 0; phy_ep < USB_EP_NUM; phy_ep++) {

    if (episr & (1 << phy_ep)) {
      // dstr("i:"); dhex(phy_ep); dstr("\n");
      // Calculate the logical endpoint value (associated with the USB Spec)
      log_ep = phy_ep >> 1;
      event.epnum = log_ep;

      // Clear the interrupt
      LPC_USB->EpIntClr = (1 << phy_ep);

      // Read the SIE command data
      while ((LPC_USB->DevIntSt & CDFULL_INT) == 0)
        ;
      tmp = LPC_USB->CmdData;

      if ((phy_ep & 1) == 0) { // These are the OUT endpoints

        // Check for a setup packet
        // Check for endpoint 0
        if ((phy_ep == 0) && (tmp & EP_SEL_STP)) {
          mcu_execute_event_handler(&m_usb_local[0].control_read,
                                    MCU_EVENT_FLAG_SETUP, &event);
        } else {
          if (log_ep == 0) {
            mcu_execute_event_handler(&m_usb_local[0].control_read,
                                      MCU_EVENT_FLAG_DATA_READY, &event);
          } else {
            if (m_usb_local[0].endpoint_handlers[log_ep].read) {
              m_usb_local[0].endpoint_handlers[log_ep].read->nbyte =
                  mcu_usb_root_read_endpoint(
                      0, log_ep,
                      m_usb_local[0].endpoint_handlers[log_ep].read->buf);
              devfs_execute_read_handler(
                  &m_usb_local[0].endpoint_handlers[log_ep], &event, 0,
                  MCU_EVENT_FLAG_DATA_READY);
            } else {
              m_usb_local[0].read_ready |= (1 << log_ep);
            }
          }
          // mcu_execute_event_handler(&(m_usb_local.read[log_ep]),
          // MCU_EVENT_FLAG_DATA_READY, &event);
        }
      } else { // These are the IN endpoints

        event.epnum |= 0x80;

        if (log_ep == 0) {
          mcu_execute_event_handler(&m_usb_local[0].control_write,
                                    MCU_EVENT_FLAG_WRITE_COMPLETE, &event);
        } else {
          devfs_execute_write_handler(&m_usb_local[0].endpoint_handlers[log_ep],
                                      &event, 0, MCU_EVENT_FLAG_WRITE_COMPLETE);
        }

        if (m_usb_local[0].address) {
          usb_set_address(0, m_usb_local[0].address);
          m_usb_local[0].address = 0;
        }
      }
    }
  }
  LPC_USB->DevIntClr = EP_SLOW_INT;
}

void set_usb_clock() {
  const u32 fosc = lpc_config.clock_oscillator_freq;
#ifdef __lpc17xx
  u16 m_mult;
  u32 fcco;
  // see if PLL0 can be used instead of PLL1
  if (LPC_SC->PLL0CON & (1 << PLLC0)) {
    m_mult = LPC_SC->PLL0STAT + 1; // Assume N = 1
    fcco = fosc * (uint32_t)m_mult * 2;
    switch (fcco) {
    case 288000000UL:
      LPC_SC->USBCLKCFG = 5;
      return;
    case 384000000UL:
      LPC_SC->USBCLKCFG = 7;
      return;
    case 480000000UL:
      LPC_SC->USBCLKCFG = 9;
      return;
    }
  }

  // Configure the USB PLL system to produce 48MHz
  switch (fosc) {
  case 24000000UL:
    m_mult = 2;
    break;
  case 16000000UL:
    m_mult = 3;
    break;
  case 12000000UL:
    m_mult = 4;
    break;
  default:
    // USB is not supported
    return;
  }

  // If connected to PLL1, disconnect with a feed sequence
  if (MCU_TEST_BIT(LPC_SC->PLL1STAT, PLLC1_STAT)) {
    MCU_CLR_BIT(LPC_SC->PLL1CON, PLLC1);
    // PLL1 is connected
    LPC_SC->PLL1FEED = 0xAA;
    LPC_SC->PLL1FEED = 0x55;
  }

  // disable PLL1
  MCU_CLR_BIT(LPC_SC->PLL1CON, PLLE1);
  LPC_SC->PLL1FEED = 0xAA;
  LPC_SC->PLL1FEED = 0x55;

  // If PLL1 is enabled, it is automatically used for the USB clock source
  // Update PLL1CFG and execute feed
  LPC_SC->PLL1CFG = (m_mult - 1) | (1 << 5);
  LPC_SC->PLL1FEED = 0xAA;
  LPC_SC->PLL1FEED = 0x55;

  // Enable PLL1
  MCU_SET_BIT(LPC_SC->PLL1CON, PLLE1);
  LPC_SC->PLL1FEED = 0xAA;
  LPC_SC->PLL1FEED = 0x55;

  // Wait for the PLL to lock
  while (!MCU_TEST_BIT(LPC_SC->PLL1STAT, PLOCK1))
    ;

  // Connect the PLL
  MCU_SET_BIT(LPC_SC->PLL1CON, PLLC1);
  LPC_SC->PLL1FEED = 0xAA;
  LPC_SC->PLL1FEED = 0x55;
#endif

#ifdef LPCXX7X_8X
  uint8_t p;
  uint8_t m;

  // Configure the USB PLL system to produce 48MHz
  switch (fosc) {
  case 24000000UL:
    m = 1; // msel is m - 1 (24 * (1+1) = 48)
    p = 2;
    break;
  case 16000000UL:
    m = 2;
    p = 3;
    break;
  case 12000000UL:
    m = 3;
    p = 3;
    break;
  default:
    // USB is not supported
    return;
  }

  // If PLL1 is on, turn it off
  if (MCU_TEST_BIT(LPC_SC->PLL1STAT, 8)) {
    LPC_SC->PLL1CON = 0;
    // PLL0 is connected
    LPC_SC->PLL1FEED = 0xAA;
    LPC_SC->PLL1FEED = 0x55;
  }

  // If PLL1 is enabled, it is automatically used for the USB clock source
  // Update PLL1CFG and execute feed
  LPC_SC->PLL1CFG = m | (p << 5);

  // Enable PLL1
  LPC_SC->PLL1CON = 1;
  LPC_SC->PLL1FEED = 0xAA;
  LPC_SC->PLL1FEED = 0x55;

  // Wait for the PLL to lock
  while (!MCU_TEST_BIT(LPC_SC->PLL1STAT, 10))
    ;

  LPC_SC->USBCLKSEL = (2 << 8) | 1; // Use PLL1 with no clock divider
#endif
}

#endif
