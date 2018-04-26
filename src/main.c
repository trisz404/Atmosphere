#include "utils.h"
#include "hwinit.h"
#include "fuse.h"
#include "se.h"
#include "sd_utils.h"
#include "lib/printk.h"
#include "display/video_fb.h"

int main(void) {
    u32 *lfb_base;
    
    /* Initialize DRAM. */
    /* TODO: What can be stripped out to make this minimal? */
    nx_hwinit();
    
    /* Initialize the display. */
    display_init();
    
    /* Register the display as a printk provider. */
    lfb_base = display_init_framebuffer();
    video_init(lfb_base);
    
    /* Turn on the backlight after initializing the lfb */
    /* to avoid flickering. */
    display_enable_backlight(true);
    
    /* Say hello. */
    printk("Hi there!\n");
    printk("You want some frash tsec key, right?\n");

    return 0;
}

