/* Copyright 2011-2018 Tyler Gilbert;
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

#include "core_local.h"
#include "lpc_local.h"

//int mcu_board_config.core_cpu_freq MCU_SYS_MEM;



#define PCLKSEL0_ALL_1 0x55515155
#define PCLKSEL1_ALL_1 0x54555455
#define PCLKSEL0_ALL_2 0xAAA2A2AA
#define PCLKSEL1_ALL_2 0xA8AAA8AA
#define PCLKSEL0_ALL_4 0x00000000
#define PCLKSEL1_ALL_4 0x00000000
#define PCLKSEL0_ALL_8 0xFFF3F3FF
#define PCLKSEL1_ALL_8 0xFCFFFCFF


static void set_flash_timing(u32 fclk){
	u32 flash_cfg_value;
	if( fclk < 20000000UL ){
		flash_cfg_value = 0;
	} else if ( fclk < 40000000UL ){
		flash_cfg_value = 0x1000;
	} else if ( fclk < 60000000UL ){
		flash_cfg_value = 0x2000;
	} else if ( fclk < 80000000UL ){
		flash_cfg_value = 0x3000;
	} else if ( fclk < 100000000UL ){
		flash_cfg_value = 0x4000;
	} else {
		flash_cfg_value = 0x5000;
	}

#if !defined __lpc43xx
	LPC_SC->FLASHCFG = flash_cfg_value | 0x03A;
#endif
}


#ifdef LPCXX7X_8X
static void mcu_core_initclock_dev(int fclk, int fosc, u8 clk_src, int pdiv){
	u16 clk_div;
	u8 msel;
	u8 psel;

	LPC_SC->CLKSRCSEL = 0; //Use IRC
	LPC_SC->CCLKSEL = 1; //Use sysclk with no divider
	LPC_SC->PCLKSEL = pdiv;

	set_flash_timing(fclk);

	switch(clk_src){
	case MAIN_OSC:
		if ( fosc > 18000000 ){  //If fosc is > 18M set to high speed oscillator mode
			MCU_SET_BIT(LPC_SC->SCS, OSCRANGE);
		}
		MCU_SET_BIT(LPC_SC->SCS, OSCEN); //enable the main oscillator
		while ( !MCU_TEST_BIT(LPC_SC->SCS, OSCSTAT )); //wait for oscillator to turn on
		LPC_SC->CLKSRCSEL = MAIN_OSC;
		break;
	}

	if( fclk <= fosc ){
		clk_div = (fosc + fclk/2) / (fclk);
		LPC_SC->CCLKSEL = (clk_div);  //use main clock -- not PLL
		fclk = fosc / clk_div;
		return;
	}

	//fclk = fosc * M -- calculate M
	msel = (fclk + (fosc>>1)) / fosc;
	if ( msel > 32 ) {
		msel = 32;
	}

	fclk = msel * fosc;

	msel--; //msel is m minus 1

	//fcco = fclk * 2 * p (fcc must be 156MHz to 320MHz)

	//P is 1, 2, 4, 8 and psel is 0, 1, 2, 3
	psel = 0;
	if( fclk < 80000000 ){ //fcco must be at least 156MHz -- 80MHz * 2 = 160MHz below 80 fcco will be at least 80
		psel = 1;
	}

	if( fclk < 40000000 ){
		psel = 2;
	}

	if( fclk < 20000000 ){
		psel = 3;
	}

	if( fclk <= 100000000 ){
		//disable PBOOST
		LPC_SC->PBOOST = 0;
	} else {
		LPC_SC->PBOOST = 0x03; //this is enabled by default
	}

	LPC_SC->PLL0CFG = (msel) | (psel<<5);

	do {
		//Enable PLL0
		MCU_SET_BIT(LPC_SC->PLL0CON, PLLE0);
		LPC_SC->PLL0FEED = 0xAA;
		LPC_SC->PLL0FEED = 0x55;

		//Wait for PLL to lock
		while( !MCU_TEST_BIT(LPC_SC->PLL0STAT, 10) );

	} while( MCU_TEST_BIT(LPC_SC->PLL0STAT, 8) == 0 ); //make sure operation was successful

	//Update clock divider
	LPC_SC->CCLKSEL = (1) | (1<<8);

}
#endif


#ifdef __lpc17xx
static int mcu_core_initclock_dev(int fclk, int fosc, u8 clk_src, int pdiv){
	u16 m_mult;
	u16 clk_div;
	u32 fcco;
	u32 sel0, sel1;

	switch(pdiv){
	case 1:
		sel0 = PCLKSEL0_ALL_1;
		sel1 = PCLKSEL1_ALL_1;
		break;
	case 2:
		sel0 = PCLKSEL0_ALL_2;
		sel1 = PCLKSEL1_ALL_2;
		break;
	case 4:
		sel0 = PCLKSEL0_ALL_4;
		sel1 = PCLKSEL1_ALL_4;
		break;
	case 8:
		sel0= PCLKSEL0_ALL_8;
		sel1 = PCLKSEL1_ALL_8;
		break;
	default:
		errno = EINVAL;
		return -1;
	}

	set_flash_timing(fclk);

	//If connected to PLL0, disconnect with a feed sequence
	if ( MCU_TEST_BIT(LPC_SC->PLL0STAT, PLLC0_STAT)){
		MCU_CLR_BIT(LPC_SC->PLL0CON, PLLC0);
		//PLL0 is connected
		LPC_SC->PLL0FEED = 0xAA;
		LPC_SC->PLL0FEED = 0x55;
	}

	//disable PLL0
	MCU_CLR_BIT(LPC_SC->PLL0CON, PLLE0);
	LPC_SC->PLL0FEED = 0xAA;
	LPC_SC->PLL0FEED = 0x55;

	//-- Errata won't let PCLK be changed after PLL is running
	LPC_SC->PCLKSEL0 = sel0;
	LPC_SC->PCLKSEL1 = sel1;

	switch(clk_src){
	case MAIN_OSC:
		MCU_SET_BIT(LPC_SC->SCS, OSCEN);
		while ( !MCU_TEST_BIT(LPC_SC->SCS, OSCSTAT ));
		if ( MCU_TEST_BIT(LPC_SC->SCS, OSCSTAT) ){
			//Enable the main oscillator
			MCU_SET_BIT(LPC_SC->SCS, OSCEN);
			if ( fosc > 18000000 ){  //If mcu_board_config.core_cpu_freq is > 18M set to high speed oscillator mode
				MCU_SET_BIT(LPC_SC->SCS, OSCRANGE);
			}
		}
		break;
	}


	if ( fclk > 96000000UL ){
		fcco = 480000000UL;
	} else {
		fcco = 288000000UL;
	}


	LPC_SC->CLKSRCSEL = clk_src;

	if ( fclk > fosc ){
		//FCCO must be between 275 MHZ and 550MHz
		//Calcute the divisors and multiplier for the PLL
		m_mult = (fcco + fosc)/(fosc*2);
		m_mult = ( m_mult > 512 ) ? 512 : m_mult;
		m_mult =  ( m_mult < 6 ) ? 6 : m_mult;

		clk_div = (fcco + fclk/2) / (fclk);
		clk_div = ( clk_div < 3 ) ? 3 : clk_div;
		clk_div = ( clk_div > 256 ) ? 256 : clk_div;
		//mcu_board_config.core_cpu_freq = fosc * 2 * m_mult / clk_div;
	} else if ( fclk <= fosc ){
		clk_div = (fosc + fclk/2) / (fclk);
		LPC_SC->CCLKCFG = (clk_div - 1);
		return 0;
	}


	//Update PLL0CFG and execute feed
	LPC_SC->PLL0CFG = (m_mult-1);  //Assume N = 1
	LPC_SC->PLL0FEED = 0xAA;
	LPC_SC->PLL0FEED = 0x55;

	//Enable PLL0
	MCU_SET_BIT(LPC_SC->PLL0CON, PLLE0);
	LPC_SC->PLL0FEED = 0xAA;
	LPC_SC->PLL0FEED = 0x55;

	//Update clock divider
	LPC_SC->CCLKCFG = (clk_div - 1);

	//Wait for the PLL to lock
	while( !MCU_TEST_BIT(LPC_SC->PLL0STAT, PLOCK0) );

	//Connect the PLL
	MCU_SET_BIT(LPC_SC->PLL0CON, PLLC0);
	LPC_SC->PLL0FEED = 0xAA;
	LPC_SC->PLL0FEED = 0x55;

	return 0;
}
#endif


//requires mcu_core_osc_freq, mcu_board_config.core_cpu_freq, and mcu_board_config.core_periph_freq to be defined ext
int mcu_core_initclock(int div){
	u8 clk_src = 0;
	u32 fosc = mcu_board_config.core_osc_freq;
	int pdiv = mcu_board_config.core_cpu_freq / mcu_board_config.core_periph_freq;
	u32 fclk = mcu_board_config.core_cpu_freq / div;

	//validate div
	switch(div){
	case 1:
	case 2:
#ifdef LPCXX7X_8X
	case 3:
#endif
	case 4:
#ifdef __lpc17xx
	case 8:
#endif
		break;
	default:
		errno = EINVAL;
		return -1;
	}

	//validate pdiv
	switch(pdiv){
	case 1:
	case 2:
#ifdef LPCXX7X_8X
	case 3:
#endif
	case 4:
#ifdef __lpc17xx
	case 8:
#endif
		break;
	default:
		errno = EINVAL;
		return -1;
	}

	if( fosc == 0 ){
		fosc = 4000000UL;
		clk_src = IRC_OSC;
	} else {
		clk_src = MAIN_OSC;
	}

	mcu_core_initclock_dev(fclk, fosc, clk_src, pdiv);


	return 0;
}

/*! @} */
