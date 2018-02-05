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

#ifndef LPC17XX_DEVICES_H_
#define LPC17XX_DEVICES_H_

#define PWM_CHANNEL1 (0)
#define PWM_CHANNEL2 (1)
#define PWM_CHANNEL3 (2)
#define PWM_CHANNEL4 (3)
#define PWM_CHANNEL5 (4)
#define PWM_CHANNEL6 (5)

#ifdef __cplusplus
extern "C" {
#endif

#define ADC_MAX 0x0000FFF0
#define ADC_MIN 0x01
#define ADC_MAX_FREQ 200000
#define ADC_SAMPLE_T
#define DAC_MAX_FREQ 1000000
#define FLASH_MIN_WRITE_SIZE 256
#define FLASH_MAX_WRITE_SIZE 1024
#define MEM_MIN_FLASH_WRITE_SIZE 256
#define MEM_MAX_FLASH_WRITE_SIZE 1024

/*! \details This defines the timer channels.
 * It is assigned to the channel member of \ref mcu_action_t
 * and \ref mcu_channel_t.
 */
typedef enum {
	TMR_ACTION_CHANNEL_OC0 /*! Output compare channel 0 */,
	TMR_ACTION_CHANNEL_OC1 /*! Output compare channel 1 */,
	TMR_ACTION_CHANNEL_OC2 /*! Output compare channel 2 */,
	TMR_ACTION_CHANNEL_OC3 /*! Output compare channel 3 */,
	TMR_ACTION_CHANNEL_IC0 /*! Input capture channel 0 */,
	TMR_ACTION_CHANNEL_IC1 /*! Input capture channel 1 */,
} lpc17xx_tmr_action_channel_t;

typedef lpc17xx_tmr_action_channel_t tmr_action_channel_t;


#define MCU_CORE_BOOTLOADER_LOC 0x10002000
#define MCU_CORE_BOOTLOADER_VALUE 0x55AA55AA

int mcu_lpc_flash_write_page(int page, void * addr, const void * src, int size);
int mcu_lpc_flash_erase_page(int page);
int mcu_lpc_flash_get_serialno(uint32_t * dest);

#define IAP_ADDRESS 0x1FFF1FF1

#include "cmsis/LPC17xx.h"

#define MCU_LAST_IRQ SW7_IRQn
#define MCU_MIDDLE_IRQ_PRIORITY 16

#ifdef __cplusplus
}
#endif


#define MCU_DMA_CHANNELS 8

#define MCU_DMA_CHANNEL_REGS { LPC_GPDMACH0, LPC_GPDMACH1, LPC_GPDMACH2, LPC_GPDMACH3, LPC_GPDMACH4, LPC_GPDMACH5, LPC_GPDMACH6, LPC_GPDMACH7 }

#include "mcu_lpc_dma.h"
#include "mcu_lpc_core.h"

#define MCU_CORE_PORTS 1
#define MCU_EEPROM_PORTS 0
#define MCU_SPI_API 0
#define MCU_SPI_PORTS 1
#define MCU_SPI_REGS { LPC_SPI }
#define MCU_SPI_IRQS { SPI_IRQn }

#define MCU_SSP_PORTS 2


#define MCU_SSP_API 0
#define MCU_SSP_PORTS 2
#define MCU_SSP_REGS { LPC_SSP0, LPC_SSP1 }
#define MCU_SSP_IRQS { SSP0_IRQn, SSP1_IRQn }

#define MCU_GPIO_PORTS 5
#define MCU_PIO_PORTS 5

#define MCU_I2C_API 0
#define MCU_I2C_PORTS 3
#define MCU_I2C_REGS { LPC_I2C0, LPC_I2C1, LPC_I2C2 }
#define MCU_I2C_IRQS { I2C0_IRQn, I2C1_IRQn , I2C2_IRQn }

#define MCU_I2S_PORTS 1
#define MCU_I2S_REGS { LPC_I2S }
#define MCU_I2S_IRQS { I2S_IRQn }

#define MCU_UART_API 0
#define MCU_UART_PORTS 4
#define MCU_UART_REGS { (LPC_UART_Type *)LPC_UART0, (LPC_UART_Type *)LPC_UART1, LPC_UART2, LPC_UART3 }
#define MCU_UART_IRQS { UART0_IRQn, UART1_IRQn, UART2_IRQn, UART3_IRQn }

#define MCU_TMR_API 0
#define MCU_TMR_PORTS 4
#define MCU_TMR_REGS { LPC_TIM0, LPC_TIM1, LPC_TIM2, LPC_TIM3 }
#define MCU_TMR_IRQS { TIMER0_IRQn, TIMER1_IRQn, TIMER2_IRQn, TIMER3_IRQn }


#define MCU_EINT_PORTS 4


#define MCU_ENET_PORTS 1
#define MCU_FLASH_PORTS 1
#define MCU_MEM_PORTS 1

#define MCU_ADC_PORTS 1
#define MCU_ADC_REGS { LPC_ADC }
#define MCU_ADC_IRQS { ADC_IRQn }

#define MCU_DAC_PORTS 1
#define MCU_DAC_REGS { LPC_DAC }
#define MCU_DAC_IRQS { DAC_IRQn }


#define MCU_QEI_PORTS 1
#define MCU_QEI_REGS { LPC_QEI }
#define MCU_QEI_IRQS { QEI_IRQn }

#define MCU_RTC_API 0
#define MCU_RTC_PORTS 1
#define MCU_RTC_REGS { LPC_RTC }
#define MCU_RTC_IRQS { RTC_IRQn }

#define MCU_USB_PORTS 1

#define MCU_PWM_PORTS 2
#define MCU_PWM_REGS { 0, LPC_PWM1 }
#define MCU_PWM_IRQS { PWM1_IRQn, PWM1_IRQn }

#define MCU_ENET_API 0
#define MCU_ENET_PORTS 1
#define MCU_ENET_REGS { LPC_EMAC }
#define MCU_ENET_IRQS { ENET_IRQn }

#define MCU_WDT_API 0

#define MCU_START_OF_SRAM 0x10000000
#define MCU_START_OF_AHB_SRAM 0x2007C000

#define MCU_RAM_PAGES 60
#define MCU_DELAY_FACTOR 12


//Pin mapping differences between 17xx and 177x_8x
#define MCU_ADC_CHANNEL0_PORT 0
#define MCU_ADC_CHANNEL0_PIN 23
#define MCU_ADC_CHANNEL1_PORT 0
#define MCU_ADC_CHANNEL1_PIN 24
#define MCU_ADC_CHANNEL2_PORT 0
#define MCU_ADC_CHANNEL2_PIN 25
#define MCU_ADC_CHANNEL3_PORT 0
#define MCU_ADC_CHANNEL3_PIN 26
#define MCU_ADC_CHANNEL4_PORT 1
#define MCU_ADC_CHANNEL4_PIN 30
#define MCU_ADC_CHANNEL5_PORT 1
#define MCU_ADC_CHANNEL5_PIN 31
#define MCU_ADC_CHANNEL6_PORT 0
#define MCU_ADC_CHANNEL6_PIN 3
#define MCU_ADC_CHANNEL7_PORT 0
#define MCU_ADC_CHANNEL7_PIN 2

#define MCU_UART_PORT0_PINASSIGN0 0
#define MCU_UART_TXPIN0_PINASSIGN0 3
#define MCU_UART_RXPIN0_PINASSIGN0 2

#define MCU_I2C_PORT0_PINASSIGN0 0
#define MCU_I2C_SCLPIN0_PINASSIGN0 28
#define MCU_I2C_SDAPIN0_PINASSIGN0 27

#define MCU_SPI_PORT0_PINASSIGN0 0
#define MCU_SPI_MOSIPIN0_PINASSIGN0 18
#define MCU_SPI_MISOPIN0_PINASSIGN0 17
#define MCU_SPI_SCKPIN0_PINASSIGN0 15

#define MCU_SPI_PORT0_PINASSIGN1 1
#define MCU_SPI_MOSIPIN0_PINASSIGN1 24
#define MCU_SPI_MISOPIN0_PINASSIGN1 23
#define MCU_SPI_SCKPIN0_PINASSIGN1 20


#endif /* LPC17XX_DEVICES_H_ */

/*! @} */
