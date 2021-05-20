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
#include <sos/boot/bootloader.h>
#include <cortexm/cortexm.h>
#include <cortexm/fault.h>
#include <sos/config.h>
#include <sos/symbols.h>

static void core_init();
extern int _main();



#define _DECLARE_ISR(name)                                                     \
  void mcu_core_##name##_isr() MCU_ALIAS(mcu_core_default_isr)
#define _ISR(name) mcu_core_##name##_isr

#if defined __lpc43xx
// ISR's -- weakly bound to default handler
_DECLARE_ISR(dac0);
_DECLARE_ISR(m0core);
_DECLARE_ISR(dma);
_DECLARE_ISR(sw0);
_DECLARE_ISR(sw1);
_DECLARE_ISR(enet0);
_DECLARE_ISR(sdio0);
_DECLARE_ISR(lcd0);
_DECLARE_ISR(usb0);
_DECLARE_ISR(usb1);
_DECLARE_ISR(sct0);
_DECLARE_ISR(rit0);
_DECLARE_ISR(tmr0);
_DECLARE_ISR(tmr1);
_DECLARE_ISR(tmr2);
_DECLARE_ISR(tmr3);
_DECLARE_ISR(mcpwm0);
_DECLARE_ISR(adc0);
_DECLARE_ISR(i2c0);
_DECLARE_ISR(i2c1);
_DECLARE_ISR(spi0);
_DECLARE_ISR(adc1);
_DECLARE_ISR(ssp0);
_DECLARE_ISR(ssp1);
_DECLARE_ISR(uart0);
_DECLARE_ISR(uart1);
_DECLARE_ISR(uart2);
_DECLARE_ISR(uart3);
_DECLARE_ISR(i2s0);
_DECLARE_ISR(i2s1);
_DECLARE_ISR(sw3);
_DECLARE_ISR(sgpio0);
_DECLARE_ISR(eint0);
_DECLARE_ISR(eint1);
_DECLARE_ISR(eint2);
_DECLARE_ISR(eint3);
_DECLARE_ISR(eint4);
_DECLARE_ISR(eint5);
_DECLARE_ISR(eint6);
_DECLARE_ISR(eint7);
_DECLARE_ISR(pio0);
_DECLARE_ISR(pio1);
_DECLARE_ISR(eventrouter0);
_DECLARE_ISR(can1);
_DECLARE_ISR(sw4);
_DECLARE_ISR(sw5);
_DECLARE_ISR(atmr0);
_DECLARE_ISR(rtc0);
_DECLARE_ISR(sw6);
_DECLARE_ISR(wdt);
_DECLARE_ISR(sw7);
_DECLARE_ISR(can0);
_DECLARE_ISR(qei);
#endif

#if defined LPCXX7X_8X
// ISR's -- weakly bound to default handler
_DECLARE_ISR(wdt); // 0
_DECLARE_ISR(tmr0);
_DECLARE_ISR(tmr1);
_DECLARE_ISR(tmr2);
_DECLARE_ISR(tmr3);
_DECLARE_ISR(uart0);
_DECLARE_ISR(uart1);
_DECLARE_ISR(uart2);
_DECLARE_ISR(uart3);
_DECLARE_ISR(pwm1);
_DECLARE_ISR(i2c0); // 10
_DECLARE_ISR(i2c1);
_DECLARE_ISR(i2c2);
_DECLARE_ISR(sw0);
_DECLARE_ISR(ssp0);
_DECLARE_ISR(ssp1);
_DECLARE_ISR(pll0);
_DECLARE_ISR(rtc0);
_DECLARE_ISR(eint0);
_DECLARE_ISR(eint1);
_DECLARE_ISR(eint2); // 20
_DECLARE_ISR(eint3);
_DECLARE_ISR(adc0);
_DECLARE_ISR(bod);
_DECLARE_ISR(usb0);
_DECLARE_ISR(can0);
_DECLARE_ISR(dma);
_DECLARE_ISR(i2s0);
_DECLARE_ISR(enet0);
_DECLARE_ISR(mci0);
_DECLARE_ISR(mcpwm0); // 30
_DECLARE_ISR(qei0);
_DECLARE_ISR(pll1);
_DECLARE_ISR(usb_activity);
_DECLARE_ISR(can_activity);
_DECLARE_ISR(uart4);
_DECLARE_ISR(ssp2);
_DECLARE_ISR(lcd0);
_DECLARE_ISR(pio0);
_DECLARE_ISR(pwm0);
_DECLARE_ISR(eeprom0); // 40
_DECLARE_ISR(cmp0);
_DECLARE_ISR(cmp1);

#endif

#if defined __lpc17xx
// ISR's -- weakly bound to default handler
_DECLARE_ISR(wdt); // 0
_DECLARE_ISR(tmr0);
_DECLARE_ISR(tmr1);
_DECLARE_ISR(tmr2);
_DECLARE_ISR(tmr3);
_DECLARE_ISR(uart0);
_DECLARE_ISR(uart1);
_DECLARE_ISR(uart2);
_DECLARE_ISR(uart3);
_DECLARE_ISR(pwm1);
_DECLARE_ISR(i2c0); // 10
_DECLARE_ISR(i2c1);
_DECLARE_ISR(i2c2);
_DECLARE_ISR(spi0);
_DECLARE_ISR(ssp0);
_DECLARE_ISR(ssp1);
_DECLARE_ISR(pll0);
_DECLARE_ISR(rtc0);
_DECLARE_ISR(eint0);
_DECLARE_ISR(eint1);
_DECLARE_ISR(eint2); // 20
_DECLARE_ISR(eint3);
_DECLARE_ISR(adc0);
_DECLARE_ISR(bod);
_DECLARE_ISR(usb0);
_DECLARE_ISR(can0);
_DECLARE_ISR(dma);
_DECLARE_ISR(i2s0);
_DECLARE_ISR(enet0);
_DECLARE_ISR(mci0);
_DECLARE_ISR(mcpwm0); // 30
_DECLARE_ISR(qei0);
_DECLARE_ISR(pll1);
_DECLARE_ISR(sw0);
_DECLARE_ISR(sw1);
_DECLARE_ISR(sw2);
_DECLARE_ISR(sw3);
_DECLARE_ISR(sw4);
_DECLARE_ISR(sw5);
_DECLARE_ISR(sw6);
_DECLARE_ISR(sw7); // 40
#endif

/*! \details This is the startup code which gets written to
 * address 0 (or wherever the text starts if there is another bootloader) in
 * flash memory
 */

void (*const mcu_core_vector_table[])() __attribute__((section(".startup"))) = {
    (void *)&_top_of_stack,        // The initial stack pointer
    cortexm_reset_handler,         // The reset handler
    cortexm_nmi_handler,           // The NMI handler
    cortexm_hardfault_handler,     // The hard fault handler
    cortexm_memfault_handler,      // The MPU fault handler
    cortexm_busfault_handler,      // The bus fault handler
    cortexm_usagefault_handler,    // The usage fault handler
    (void *)&_sos_hardware_id,     // hardware ID
    0,                             // Reserved
    (void *)&sos_config.boot.api,  // boot API pointer
    0,                             // Reserved
    cortexm_svcall_handler,        // SVCall handler
    cortexm_debug_monitor_handler, // Debug monitor handler
    0,                             // Reserved
    cortexm_pendsv_handler,        // The PendSV handler
    cortexm_systick_handler,       // The SysTick handler
// Non Cortex M interrupts (device specific interrupts)

#if defined __lpc43xx
    _ISR(dac0), // 0
    _ISR(m0core),
    _ISR(dma),
    _ISR(sw0),
    _ISR(sw1),
    _ISR(enet0),
    _ISR(sdio0),
    _ISR(lcd0),
    _ISR(usb0),
    _ISR(usb1),
    _ISR(sct0), // 10
    _ISR(rit0),
    _ISR(tmr0),
    _ISR(tmr1),
    _ISR(tmr2),
    _ISR(tmr3),
    _ISR(mcpwm0),
    _ISR(adc0),
    _ISR(i2c0),
    _ISR(i2c1),
    _ISR(spi0), // 20
    _ISR(adc1),
    _ISR(ssp0),
    _ISR(ssp1),
    _ISR(uart0),
    _ISR(uart1),
    _ISR(uart2),
    _ISR(uart3),
    _ISR(i2s0),
    _ISR(i2s1),
    _ISR(sw3), // 30
    _ISR(sgpio0),
    _ISR(eint0),
    _ISR(eint1),
    _ISR(eint2),
    _ISR(eint3),
    _ISR(eint4),
    _ISR(eint5),
    _ISR(eint6),
    _ISR(eint7),
    _ISR(pio0), // 40
    _ISR(pio1),
    _ISR(eventrouter0),
    _ISR(can1),
    _ISR(sw4),
    _ISR(sw5),
    _ISR(atmr0),
    _ISR(rtc0),
    _ISR(sw6),
    _ISR(wdt),
    _ISR(sw7), // 50
    _ISR(can0),
    _ISR(qei)
#endif

#if defined LPCXX7X_8X

        _ISR(wdt), // 0
    _ISR(tmr0),
    _ISR(tmr1),
    _ISR(tmr2),
    _ISR(tmr3),
    _ISR(uart0),
    _ISR(uart1),
    _ISR(uart2),
    _ISR(uart3),
    _ISR(pwm1),
    _ISR(i2c0), // 10
    _ISR(i2c1),
    _ISR(i2c2),
    _ISR(sw0),
    _ISR(ssp0),
    _ISR(ssp1),
    _ISR(pll0),
    _ISR(rtc0),
    _ISR(eint0),
    _ISR(eint1),
    _ISR(eint2), // 20
    _ISR(eint3),
    _ISR(adc0),
    _ISR(bod),
    _ISR(usb0),
    _ISR(can0),
    _ISR(dma),
    _ISR(i2s0),
    _ISR(enet0),
    _ISR(mci0),
    _ISR(mcpwm0), // 30
    _ISR(qei0),
    _ISR(pll1),
    _ISR(usb_activity),
    _ISR(can_activity),
    _ISR(uart4),
    _ISR(ssp2),
    _ISR(lcd0),
    _ISR(pio0),
    _ISR(pwm0),
    _ISR(eeprom0), // 40
    _ISR(cmp0),
    _ISR(cmp1)
#endif

#if defined __lpc17xx
        _ISR(wdt), // 0
    _ISR(tmr0),
    _ISR(tmr1),
    _ISR(tmr2),
    _ISR(tmr3),
    _ISR(uart0),
    _ISR(uart1),
    _ISR(uart2),
    _ISR(uart3),
    _ISR(pwm1),
    _ISR(i2c0), // 10
    _ISR(i2c1),
    _ISR(i2c2),
    _ISR(spi0),
    _ISR(ssp0),
    _ISR(ssp1),
    _ISR(pll0),
    _ISR(rtc0),
    _ISR(eint0),
    _ISR(eint1),
    _ISR(eint2), // 20
    _ISR(eint3),
    _ISR(adc0),
    _ISR(bod),
    _ISR(usb0),
    _ISR(can0),
    _ISR(dma),
    _ISR(i2s0),
    _ISR(enet0),
    _ISR(mci0),
    _ISR(mcpwm0), // 30
    _ISR(qei0),
    _ISR(pll1),
    _ISR(sw0),
    _ISR(sw1),
    _ISR(sw2),
    _ISR(sw3),
    _ISR(sw4),
    _ISR(sw5),
    _ISR(sw6),
    _ISR(sw7) // 40
#endif

};

void mcu_core_getserialno(mcu_sn_t *serial_number) {
  mcu_lpc_flash_get_serialno(serial_number->sn);
}

void core_init() {

  // This is the de facto MCU initialization -- turn off power to peripherals
  // that must be "open()"ed.
#if defined LPC_SC
  LPC_SC->PCONP = (1 << PCGPIO) | (1 << PCRTC);
#else

#endif
}

void mcu_core_default_isr() { sos_handle_event(SOS_EVENT_ROOT_FATAL, "dflt"); }

