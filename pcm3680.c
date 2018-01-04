/* pcm3680.c     - can4linux source file 
 *
 * Advantech  PCM3680 PC104 board using iup to two SJA1000 
 * 
 * (c) 2006 oe@port.de
 */

#include "defs.h"

#include <linux/irq.h>
#include <asm/irq.h>


/* check memory region if there is a CAN controller
*  assume the controller was resetted before testing 
*
*  The check for an avaliable controller is difficult !
*  After an Hardware Reset (or power on) the Conroller 
*  is in the so-called 'BasicCAN' mode.
*     we can check for: 
*         adress  name      value
*	    0x00  mode       0x21
*           0x02  status     0xc0
*           0x03  interrupt  0xe0
* Once loaded thr driver switches into 'PeliCAN' mode and things are getting
* difficult, because we now have only a 'soft reset' with not so  unique
* values. The have to be masked before comparing.
*         adress  name       mask   value
*	    0x00  mode               
*           0x01  command    0xff    0x00
*           0x02  status     0x37    0x34
*           0x03  interrupt  0xfb    0x00
*
*/

int controller_available(upointer_t address, int offset)
{
unsigned long ptr = (unsigned long)ioremap(address, 32 * offset);

    printk("controller_available 0x%0x\n", (unsigned)address);


    printk("0x%0x, ", readb((void __iomem *)ptr + (2 * offset)) );
    printk("0x%0x\n", readb((void __iomem *)ptr + (3 * offset)) );

    if ( 0x21 == readb((void __iomem *)ptr))  {
	/* compare reset values of status and interrupt register */
	if(   0x0c == readb((void __iomem *)ptr + (2 * offset))
	   && 0xe0 == readb((void __iomem *)ptr + (3 * offset)) ) {
	    return 1;
	} else {
	    return 0;
	}
    } else {
	/* may be called after a 'soft reset' in 'PeliCAN' mode */
	/*   value     address                     mask    */
	if(   0x00 ==  readb((void __iomem *)ptr + (1 * offset))
	   && 0x34 == (readb((void __iomem *)ptr + (2 * offset))    & 0x37)
	   && 0x00 == (readb((void __iomem *)ptr + (3 * offset))    & 0xfb)
	  ) {
	return 1;
    } else {
	return 0;
    }

    }
}


void board_clear_interrupts(int minor)
{
}

#if CONFIG_TIME_MEASURE
    init_measure();
#endif

/*
 * PCM3680  Remarks
 *
 * Adresses used related to the Basde adress (set by dip switches) 
 * Base address (hex)      CAN controller
 * base:0000h - base:00FFh Basic- Port 1
 * base:0100h - base:01FFh HW reset Basic - Port 1
 * base:0200h - base:02FFh Basic- Port 2
 * base:0300h - base:03FFh HW reset Basic - Port 2
 * base:0400h - base:0FFFh Not used
 * 
 * Each CAN channel uses 0x200 bytes
 */


/*
 * Perform Vendor, that means sometimes CAN controller
 * or only board manufacturer specific initialization.
 *
 * Mainly it gets needed IO and IRQ ressources and initilaizes 
 * special hardware functions.
 *
 * This code should normally be in the CAN_VendorInit() function
 * in a TARGET specific file  target.c
 *
*/




 
int CAN_VendorInit (int minor)
{
    DBGin();
    can_range[minor] = 0x200;
    

    /* Some LINUX systems, e.g. the PIP10 I tested on,
     * locate already the memory using the information
     * provided in the "Option ROM"
     * The memory is marked as "Adapter-ROM" in /proc/iomem.
     * In this case the drive should not try to allocate the IO mem */

#if !defined(PC104_OPTION_ROM)
    /* Request the controllers address space */
    if(NULL == request_mem_region(Base[minor], can_range[minor], "CAN-IO")) {
	DBGprint(DBG_DATA,("Request_mem-Region CAN-IO failed at 0x%lx\n",
		Base[minor]));
	return -EBUSY;
    }
#endif

    controller_available(Base[minor], 1);

    can_base[minor] = ioremap(Base[minor], can_range[minor]);
    /* now the virtual address can be used for the register access macros */


    if( Base[minor] & 0x200 ) {
	    /* printk("Resetting Advantech Pcm-3680 [contr 1]\n"); */
	    /* perform HW reset 2. contr*/
	    writeb(0xff, can_base[minor] + 0x300);
    } else {
	    /* printk("Resetting Advantech Pcm-3680 [contr 0]\n"); */
	    /* perform HW reset 1. contr*/
	    writeb(0xff, can_base[minor] + 0x100);
    }
    mdelay(100);



    if( IRQ[minor] > 0 && IRQ[minor] < MAX_IRQNUMBER ){
        int err;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18) 
	err = request_irq( IRQ[minor], CAN_Interrupt, IRQF_SHARED, 
				"Can", &Can_minors[minor]);
#else
	err = request_irq( IRQ[minor], CAN_Interrupt, SA_SHIRQ, 
				"Can", &Can_minors[minor]);
#endif
        if( !err ){
	    DBGprint(DBG_BRANCH,("Requested IRQ: %d @ 0x%lx",
				    IRQ[minor], (unsigned long)CAN_Interrupt));
	    IRQ_requested[minor] = 1;
	} else {
	    release_mem_region(Base[minor], can_range[minor]);
	    DBGout(); return -EBUSY;
	}
    } else {
	/* Invalid IRQ number in /proc/.../IRQ */
	release_mem_region(Base[minor], can_range[minor]);
	DBGout(); return -EBUSY;
    }
    DBGout(); return 0;
}



int Can_FreeIrq(int minor, int irq )
{
    DBGin();
    IRQ_requested[minor] = 0;
    free_irq(irq, &Can_minors[minor]);
    DBGout();
    return 0;
}



/* Release IRQ and IO ressources */
int CAN_Release(int minor)
{
    DBGin();

    /* call this before freeing any memory or io area.
     * this can contain registers needed by Can_FreeIrq()
     */
    Can_FreeIrq(minor, IRQ[minor]);


    printk("iounmap %p \n", can_base[minor]);
    iounmap(can_base[minor]);

    /* release_mem_region(Base[minor], can_range[minor]); */
    /* Release the memory region */
    printk("release mem %lx \n", Base[minor]);
    release_mem_region(Base[minor], can_range[minor]);

    DBGout(); return 0;
}


#if CONFIG_TIME_
/* Functions to Switch the stet of output pins
   used for time measurement within the CAN ISR
 */
void init_measure(void)
{
    /* set port to output and initially to low */
}

void set_measure_pin(void) 
{
    /* set port to output and initially to high */
    /* at91_set_gpio_output(AT91_PIN_PC1, 1); */
}

void reset_measure_pin(void) 
{
    /* set port to output and initially to high */
    /* at91_set_gpio_output(AT91_PIN_PC1, 0); */
}
#endif
