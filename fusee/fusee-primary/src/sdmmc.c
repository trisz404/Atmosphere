#include <string.h>

#include "sdmmc.h"
#include "hwinit/t210.h"
#include "lib/printk.h"


#define REGISTER(address) (*(volatile uint32_t *)(address))
#define SDMMC1_BASE 				0x700b0000
#define SDMMC1_SDMEMCOMPPADCTRL_0 	(*(volatile uint32_t *)(SDMMC1_BASE + 0x1e0))
#define SDMMC1_CMD_REG 				(*(volatile uint32_t *)(SDMMC1_BASE + 0x0C))
#define SDIO_HOST_PWR_BLOCK_WAKE 	(*(volatile uint32_t *)(SDMMC1_BASE + 0x28))
#define SDIO_CLK_TIMEOUT_SOFTRST	(*(volatile uint32_t *)(SDMMC1_BASE + 0x2C))

#define SDMMC1_IO_SPARE_0				REGISTER(SDMMC1_BASE + 0x1f0)
#define SDMMC1_VENDOR_IO_TRIM_CNTRL_0	REGISTER(SDMMC1_BASE + 0x1ac)
#define SDMMC1_VENDOR_CLOCK_CNTRL_0 	REGISTER(SDMMC1_BASE + 0x100)
#define SDMMC1_AUTO_CAL_CONFIG_0 		REGISTER(SDMMC1_BASE + 0x1e4)
#define SDMMC1_AUTO_CAL_STATUS_0 		REGISTER(SDMMC1_BASE + 0x1ec)

#define TIMER_BASE  0x60005000 
#define GPIO_BASE 0x6000D000
#define GPIO_7_BASE (GPIO_BASE + 0x600)
#define GPIO_7(off) _REG(GPIO_7_BASE, off)

#define MISC_BASE 					0x70000000
#define PINMUX_BASE 				MISC_BASE + 0x3000
#define PINMUX_AUX_GPIO_PZ1_0 		(*(volatile uint32_t *)(PINMUX_BASE + 0x280))
#define APB_MISC_GP_VGPIO_GPIO_MUX_SEL_0 	REGISTER(MISC_BASE + 0xb74)
#define APB_MISC_GP_SDMMC1_PAD_CFGPADCTRL_0 REGISTER(MISC_BASE + 0xa98)

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

enum sdio_directions{
	SDIO_WRITE = 0,
	SDIO_READ = 1
};

/* Initialize the SDMMC1 (SD card) controller */
static void udelay(unsigned usecs)
{
	volatile uint32_t * timerval = (volatile uint32_t *)(TIMER_BASE + 0x10);
	uint32_t start = *timerval;
	while ((*timerval - start) < usecs);
}

void reg_dump(uint32_t base, uint32_t size){
	printk("   | 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f\n");
	printk("----------------------------------------------------\n");
	for(uint32_t i = 0; size > i; i += 0x10){
		printk("%02x | ", i);
		for(uint32_t j = 0; j < 4; j++){
			for(uint32_t k = 4; k > 0; k--){
				printk("%02x ", (REGISTER(base + i + (4 * j)) >> (8 * (k - 1))) & 0b11111111);
			}
		}
		printk("\n");
	}
}

void sdio_start_bus_clock(){
	uint32_t sdio_clk_enable_bit = 0b100;
	SDIO_CLK_TIMEOUT_SOFTRST = (SDIO_CLK_TIMEOUT_SOFTRST & ~sdio_clk_enable_bit) | sdio_clk_enable_bit; 
	printk("AFTER: \n");
	reg_dump(SDMMC1_BASE, 0x100);
}

void sdio_stop_clock(){;}

void send_sdio_command(uint32_t command, uint32_t direction){

	uint32_t direction_mask = 1 << 4;
	uint32_t cmd_num_mask = 0b111111 << (8+16); 

	SDMMC1_CMD_REG = (SDMMC1_CMD_REG & ~(direction_mask | cmd_num_mask)) | 
						((command << (8+16)) | (cmd_num_mask << 4));

	//write direction to transfer mode register

	//write command number to cmd register
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
   
	printk("\n-\n-\n\n");

	printk("BEFORE:\n");
	reg_dump(SDMMC1_BASE, 0x100);
	printk("\n\n");
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
    	reg_state = GPIO_7(0x34);
		udelay(100000);
	}

    printk("sdmmc1_init: SD card found, turning on\n");

    //write 1 to SDMMCA_SDMEMCOMPPADCTRL_0_PAD_E_INPUT_OR_E_PWRD because the TRM says we should
    SDMMC1_SDMEMCOMPPADCTRL_0 |= (1 << 0x1F);
    //write to E4, turning on SD card
    GPIO_2(0x20) = (GPIO_2(0x20) & gpio_pin_4_mask) | gpio_pin_4;

    printk("sdmmc1_init: SD card on! Let's roll!\n");


    //set chip detect to gpio
    APB_MISC_GP_VGPIO_GPIO_MUX_SEL_0 &= ~(1 << 2);

    //NOTE: CHECK IF SDIO rail is set up

    //need to try enable card detect via pinmux. see:
    //10. SDMMC1 controller gets SDCD and SDWP status from GPIO pins (SDMMC1_CD and SDMMC1_WP). Software has
//to program CD/WP source appropriately in below register to get SDMMC1 correct status.
//a. APB_MISC_GP_VGPIO_GPIO_MUX_SEL_0_ SDMMC1_WP_SOURCE = GPIO
//b. APB_MISC_GP_VGPIO_GPIO_MUX_SEL_0_ SDMMC1_CD_SOURCE = GPIO

    //setup pinmux for SDIO clk
    //passthrough tristate driver:
    PINMUX_AUX_SDMMC1_CLK_0 &= ~(1 << 4);

    //sdmmc weird magic config registers
    SDMMC1_IO_SPARE_0 |= (1 << 19); //bit 19 << 0b1
    SDMMC1_VENDOR_IO_TRIM_CNTRL_0 &= ~(1 << 2); //bit 2 << 0b0

    //clock trimmer tap outbound
    uint32_t trim_val_mask = (0b11111 << 24);
    uint32_t trim_val_new = (0x2 << 24);
    SDMMC1_VENDOR_CLOCK_CNTRL_0 = ((SDMMC1_VENDOR_CLOCK_CNTRL_0 & ~(trim_val_mask)) | trim_val_new); //TRIM_VAL = 0x2


    //clock trimmer tap inbound
    uint32_t tap_val_mask = (0b11111111 << 16);
    uint32_t tap_val_new = (0x4 << 16);
    SDMMC1_VENDOR_CLOCK_CNTRL_0 = ((SDMMC1_VENDOR_CLOCK_CNTRL_0 & ~(tap_val_mask)) | tap_val_new); //TAP_VAL = 0x4

    //set slew codes
    APB_MISC_GP_SDMMC1_PAD_CFGPADCTRL_0 &= ~(0b11 << 30);//CFG2TMC_SDMMC1_CLK_CFG_CAL_DRVUP_SLWF = 0x0
    APB_MISC_GP_SDMMC1_PAD_CFGPADCTRL_0 &= ~(0b11 << 28);//CFG2TMC_SDMMC1_CLK_CFG_CAL_DRVDN_SLWR = 0x0

    //set VSEL
    uint32_t vref_sel_mask = 0b1111;
    uint32_t vref_sel_new = 0x7;
    SDMMC1_SDMEMCOMPPADCTRL_0 = ((SDMMC1_SDMEMCOMPPADCTRL_0 & ~vref_sel_mask) | vref_sel_new);//SDMMC2TMC_CFG_SDMEMCOMP_VREF_SEL = 0x7

	printk("%08x\n", APB_MISC_GP_SDMMC1_PAD_CFGPADCTRL_0);
    //do autocal
    uint32_t pu_offset_mask = 0b1111111;
    uint32_t pd_offset_mask = 0b1111111 << 8;
    uint32_t pu_offset_new =  0b0000000;
    uint32_t pd_offset_new = 125 << 8;
    
    SDMMC1_AUTO_CAL_CONFIG_0 = ((SDMMC1_AUTO_CAL_CONFIG_0 & ~pu_offset_mask) | pu_offset_new);
    
    SDMMC1_AUTO_CAL_CONFIG_0 = ((SDMMC1_AUTO_CAL_CONFIG_0 & ~pd_offset_mask) | pd_offset_new); //auto_cal_pu_offset = 0
    

    uint32_t auto_cal_enable = 0b1 << 29;
    uint32_t auto_cal_start = 0b1 << 31;
    SDMMC1_AUTO_CAL_CONFIG_0 |= auto_cal_enable; 
    SDMMC1_AUTO_CAL_CONFIG_0 |= auto_cal_start; //auto_cal_start and auto_cal_enable to 1
    

    udelay(1);
    //while(~((SDMMC1_AUTO_CAL_STATUS_0 >> 31) & 0b1)){;}
    

    while((SDMMC1_AUTO_CAL_STATUS_0 >> 31) & 0b1){
    	printk("%08x\n", SDMMC1_AUTO_CAL_STATUS_0);
    }
    printk("%08x\n", SDMMC1_AUTO_CAL_STATUS_0);
    printk("%08x\n", APB_MISC_GP_SDMMC1_PAD_CFGPADCTRL_0);
    //power on sdio subsystem
    //set voltage to 3.3V
    uint32_t vdd1_select_mask = (0b111 << (8 + 1));
    uint32_t vdd1_3v3 = (0b111 << (8 + 1));
    SDIO_HOST_PWR_BLOCK_WAKE = (SDIO_HOST_PWR_BLOCK_WAKE & ~vdd1_select_mask) | vdd1_3v3;
    udelay(5);
    //write 1 to power on 
    SDIO_HOST_PWR_BLOCK_WAKE |= (1 << 8);


    SDIO_CLK_TIMEOUT_SOFTRST |= 0b1;

    sdio_start_bus_clock();
    //need to set up:
    //host control register
    //clock control register
    //power control register

    //send_sdmmc_command(0x01, SDIO_WRITE);
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