#include <string.h>

#include "sdmmc.h"
#include "hwinit/t210.h"
#include "lib/printk.h"

#define TIMER_BASE  0x60005000 
#define GPIO_BASE 0x6000D000
#define GPIO_7_BASE (GPIO_BASE + 0x600)
#define GPIO_7(off) _REG(GPIO_7_BASE, off)

#define MISC_BASE 					0x70000000
#define PINMUX_BASE 				MISC_BASE + 0x3000
#define PINMUX_AUX_GPIO_PZ1_0 		(*(volatile uint32_t *)(PINMUX_BASE + 0x280))

#define APBDEV_PMC_BASE 			0x7000e400
#define APBDEV_PMC_PWR_DET_0 		(*(volatile uint32_t *)(APBDEV_PMC_BASE +  0x48))
#define APBDEV_PMC_PWR_DET_VAL_0	(*(volatile uint32_t *)(APBDEV_PMC_BASE +  0xe4))
#define APBDEV_PMC_IO_DPD_REQ_0		(*(volatile uint32_t *)(APBDEV_PMC_BASE + 0x1b8))
#define APBDEV_PMC_IO_DPD2_REQ_0	(*(volatile uint32_t *)(APBDEV_PMC_BASE + 0x1c0))
/* Initialize the SDMMC1 (SD card) controller */
static void udelay(unsigned usecs)
{
	volatile uint32_t * timerval = (volatile uint32_t *)(TIMER_BASE + 0x10);
	uint32_t start = *timerval;
	while ((*timerval - start) < usecs);
}


void sdmmc1_init(void)
{
	
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

    //gpio setup: 
    //need pinmux for gpios
    //need gpio direction
    //need gpio values 

    //0x02	0x24	E, 4	SDCard Power		Out	
	//0x38	0xC9	Z, 1	SDCard Card Detect	In	


    //E, 4 is GPIO_2, offset 0x10

    //getting i/o = 1 requires:
    
    
    //set pin to high (0x1 to output val reg)

    //uint32_t pin_4 = 0b10000;
    //uint32_t pin_4_mask = ~pin_4;

    
    //bring up GPIO power rails
	APBDEV_PMC_PWR_DET_0 |= 0xA42000u;
	udelay(5);
  	APBDEV_PMC_PWR_DET_VAL_0 &= 0xFF5BDFFF;
  	udelay(5);
  	APBDEV_PMC_IO_DPD_REQ_0 = 0x40000000;
  	udelay(5);
  	APBDEV_PMC_IO_DPD2_REQ_0 = 0x40000000;
  	udelay(5);
   
    //nonsense printouts
  	printk("PMC_PWR_DET 	= %08x\n", APBDEV_PMC_PWR_DET_0);
  	printk("PMC_PWR_DET_VAL = %08x\n", APBDEV_PMC_PWR_DET_VAL_0);
  	printk("PMC_IO_DPD_REQ 	= %08x\n", APBDEV_PMC_IO_DPD_REQ_0);
  	printk("PMC_IO_DPD2_REQ = %08x\n", APBDEV_PMC_IO_DPD2_REQ_0);
  	

  	uint32_t pinmux_pullup_mask = (0b11 << 2);
  	uint32_t pull_up = (0b10 << 2);

  	//set pull-up resistors on Z1
 	PINMUX_AUX_GPIO_PZ1_0 = (PINMUX_AUX_GPIO_PZ1_0 & ~pinmux_pullup_mask) | pull_up;



    uint32_t pin_1 = 0b10;
    uint32_t pin_1_mask = ~pin_1;

    //set as GPIO (0x0 to GPIO CNF 1)
    GPIO_7( 0x4) = (GPIO_7( 0x4) & pin_1_mask) | pin_1;

    //direction config (0x1 to GPIO OE)
    GPIO_7(0x14) = (GPIO_7(0x14) & pin_1_mask) | 0x0;

    //= (GPIO_2(0x20) & pin_4_mask) | pin_4;

    uint32_t reg_state;

    while(1){
    //read bit 
    	reg_state = GPIO_7(0x34);
		printk("%d", reg_state & 0xFF);
		udelay(100000);
	}

    printk("done in sdmmc\n");
    //done?

	//read for presence of sd card
	//if present, power sd card and return
	//if not present, print error and panic
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