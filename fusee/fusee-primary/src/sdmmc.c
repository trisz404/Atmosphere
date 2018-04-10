#include <string.h>

#include "sdmmc.h"

 #define TIMER_BASE  0x60005000 

/* Initialize the SDMMC1 (SD card) controller */
static void udelay(unsigned usecs)
{
	volatile uint32_t * timerval = (volatile uint32_t *)(TIMER_BASE + 0x10);
	uint32_t start = *timerval;
	while ((*timerval - start) < usecs);
}


void sdmmc1_init(void)
{
	//0x02	0x24	E, 4	SDCard Power		Out	
	//0x38	0xC9	Z, 1	SDCard Card Detect	In	
    volatile uint32_t * sdmmc1_rst_set = (volatile uint32_t *)CLK_RST_CONTROLLER_RST_DEV_L_SET_0;
    volatile uint32_t * sdmmc1_rst_clr = (volatile uint32_t *)CLK_RST_CONTROLLER_RST_DEV_L_CLR_0;

	//put sdmmc1 in reset
    *sdmmc1_rst_set |= SWR_SDMMC1_RST;

    //set sdmmc1 clk source and divisor
    volatile uint32_t * sdmmc1_clk_src_reg = (volatile uint32_t *)CLK_RST_CONTROLLER_CLK_SOURCE_SDMMC1_0;
    *sdmmc1_clk_src_reg &= ~CLK_M_MASK;
    *sdmmc1_clk_src_reg |= CLK_M_PLLP_OUT0;
    *sdmmc1_clk_src_reg |= CLK_DIV_IDENT;

    //set sdmmc1 timeout clock and divider
    volatile uint32_t * sdmmc1_clk_legacy = (volatile uint32_t *)CLK_RST_CONTROLLER_CLK_SOURCE_SDMMC_LEGACY_TM_0;
    *sdmmc1_clk_legacy &= ~CLK_M_MASK;
    *sdmmc1_clk_legacy |= SDMMC_LEGACY_PLLP_OUT0;
    *sdmmc1_clk_legacy &= ~SDMMC_LEGACY_DIV_MSK;
    *sdmmc1_clk_legacy |= SDMMC_LEGACY_CLK_DIV;
    
    //set sdmmc1 host clock and divider
    volatile uint32_t * car_sdmmc_legacy = (volatile uint32_t *)CLK_RST_CONTROLLER_CLK_OUT_ENB_Y_0;
    volatile uint32_t * car_sdmmc_clk = (volatile uint32_t *)CLK_RST_CONTROLLER_CLK_ENB_L_SET_0;
    *car_sdmmc_legacy |= CLK_ENB_SDMMC_LEGACY_TM_ENB;
    *car_sdmmc_clk |= SET_CLK_ENB_SDMMC1;

    //delay 100 host clock cycles (5 us should be overkill)
    udelay(5);

    //pull sdmmc1 out of reset
    *sdmmc1_rst_clr |= SWR_SDMMC1_RST;

    //need pinmux for gpios
    //need gpio direction
    //need gpio values 
}

/* Initialize the SDMMC2 (GC asic) controller */
void sdmmc2_init(void)
{
    /* TODO */
}

/* Initialize the SDMMC3 (unused) controller */
void sdmmc3_init(void)
{
    /* TODO */
}

/* Initialize the SDMMC4 (eMMC) controller */
void sdmmc4_init(void)
{
    /* TODO */
}