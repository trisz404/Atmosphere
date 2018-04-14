#include <string.h>

#include "sdmmc.h"
#include "hwinit/t210.h"
#include "lib/printk.h"

#define SDMMC1_BASE 				0x700b0000
#define SDMMC1_SDMEMCOMPPADCTRL_0 	(*(volatile uint32_t *)(SDMMC1_BASE + 0x1e0))


#define TIMER_BASE  0x60005000 
#define GPIO_BASE 0x6000D000
#define GPIO_7_BASE (GPIO_BASE + 0x600)
#define GPIO_7(off) _REG(GPIO_7_BASE, off)

#define MISC_BASE 					0x70000000
#define PINMUX_BASE 				MISC_BASE + 0x3000
#define PINMUX_AUX_GPIO_PZ1_0 		(*(volatile uint32_t *)(PINMUX_BASE + 0x280))

#define PINMUX_AUX_DMIC3_CLK_0		(*(volatile uint32_t *)(PINMUX_BASE + 0xb4))
#define PINMUX_AUX_GPIO_PE4_0 		PINMUX_AUX_DMIC3_CLK_0

#define PINMUX_AUX_SDMMC1_CLK_0		(*(volatile uint32_t *)(APBDEV_PMC_BASE + 0x00))
#define PINMUX_AUX_SDMMC1_CMD_0		(*(volatile uint32_t *)(APBDEV_PMC_BASE + 0x04))
#define PINMUX_AUX_SDMMC1_DAT3_0	(*(volatile uint32_t *)(APBDEV_PMC_BASE + 0x08))
#define PINMUX_AUX_SDMMC1_DAT2_0	(*(volatile uint32_t *)(APBDEV_PMC_BASE + 0x10))
#define PINMUX_AUX_SDMMC1_DAT1_0	(*(volatile uint32_t *)(APBDEV_PMC_BASE + 0x14))
#define PINMUX_AUX_SDMMC1_DAT0_0	(*(volatile uint32_t *)(APBDEV_PMC_BASE + 0x18))

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
  	//sdmmc card detect is now possible
 	PINMUX_AUX_GPIO_PZ1_0 = (PINMUX_AUX_GPIO_PZ1_0 & ~pinmux_pullup_mask) | pull_up;
 	
 	//set pull-up resistors on E4
 	//sdmmc card detect 
 	PINMUX_AUX_GPIO_PE4_0 = (PINMUX_AUX_GPIO_PE4_0 & ~pinmux_pullup_mask) | pull_up;


    uint32_t gpio_pin_1 = 1 << 1;
    uint32_t gpio_pin_1_mask = ~gpio_pin_1;
    uint32_t gpio_pin_4 = 1 << 4;
    uint32_t gpio_pin_4_mask = ~ gpio_pin_4;

    //set Z1 in GPIO mode (0x0 to GPIO CNF 1)
    GPIO_7( 0x4) = (GPIO_7( 0x4) & gpio_pin_1_mask) | gpio_pin_1;
    //set Z1 as input (0x0 to GPIO OE)
    GPIO_7(0x14) = (GPIO_7(0x14) & gpio_pin_1_mask) | 0x0;
    //set E4 in GPIO mode 
    GPIO_2( 0x0) = (GPIO_2( 0x0) & gpio_pin_4_mask) | gpio_pin_4;
    //set E4 as output (0x1 to GPIO OE)
    GPIO_2(0x10) = (GPIO_2(0x10) & gpio_pin_4_mask) | gpio_pin_4;

    uint32_t reg_state;
	udelay(100000);

    reg_state = GPIO_7(0x34);

    printk("sdmmc1_init: waiting for SD card\n");

    while((reg_state >> 1) & 0b1){
    //read bit 
    	reg_state = GPIO_7(0x34);
		//printk("%d\n", (reg_state >> 1) & 0b1);
		udelay(100000);
	}

    printk("sdmmc1_init: SD card found, turning on\n");
    //done?

    //write 1 to SDMMCA_SDMEMCOMPPADCTRL_0_PAD_E_INPUT_OR_E_PWRD because the TRM says we should
    SDMMC1_SDMEMCOMPPADCTRL_0 |= (1 << 0x1F);
    //write to E4, turning on SD card
    GPIO_2(0x20) = (GPIO_2(0x20) & gpio_pin_4_mask) | gpio_pin_4;

    printk("sdmmc1_init: SD card on! Let's roll!\n");


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