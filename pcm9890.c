/* AT-CAN-MINI board specific functions
 * 
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
* Once loaded the driver switches into 'PeliCAN' mode and things are getting
* difficult, because we now have only a 'soft reset' with not so  unique
* values. The values have to be masked before comparing.
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

   /* From include/linux/irq.h
       IRQ line status.
	IRQ types
	IRQ_TYPE_NONE		Default, unspecified type
	IRQ_TYPE_EDGE_RISING	Edge rising type
	IRQ_TYPE_EDGE_FALLING	Edge falling type
	IRQ_TYPE_EDGE_BOTH (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING)
	IRQ_TYPE_LEVEL_HIGH	Level high type
	IRQ_TYPE_LEVEL_LOW	Level low type
	IRQ_TYPE_SENSE_MASK	Mask of the above
	IRQ_TYPE_PROBE		Probing in progress
       From include/linux/irq.h
       IRQF_DISABLED - keep irqs disabled when calling the action handler


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
    can_range[minor] = CAN_RANGE;

    /* Request the controllers address space
     *
     * It's Memory I/O
     */

#if 0   /* already requested by the kernel for some reasons and marked as
	 * reserved cat/proc/iomem
	 ....
	 000c8000-000dffff : reserved
	 ....
	 */
    if(NULL == request_mem_region(Base[minor], can_range[minor], "CAN-IO")) {
	DBGprint(DBG_DATA,("Request_mem_region CAN-IO failed at %x\n",
		Base[minor]));
	return -EBUSY;
    }
#endif
    can_base[minor] = ioremap(Base[minor], CAN_RANGE);

   /*

    int request_irq(unsigned int irq,			// interrupt number  
              void (*handler)(int, void *, struct pt_regs *), // pointer to ISR
		              irq, dev_id, registers on stack
              unsigned long irqflags, const char *devname,
              void *dev_id);

       dev_id - The device ID of this handler (see below).       
       This parameter is usually set to NULL,
       but should be non-null if you wish to do  IRQ  sharing.
       This  doesn't  matter when hooking the
       interrupt, but is required so  that,  when  free_irq()  is
       called,  the  correct driver is unhooked.  Since this is a
       void *, it can point to anything (such  as  a  device-spe-
       cific  structure,  or even empty space), but make sure you
       pass the same pointer to free_irq().

       If the kernel receives an interrupt he knows
       how often it was registered,
       and calls the ISR for each registered interrupt.
       In this case, can4linux, for example
       every time with a different pointer to Can_minors[]
       contaiing the device minor which has registered.
       The ISR should now check this CAN controller relaed to the minor
       number, if it was the cause of the interrupt.

    */

    /* test for valid IRQ number in /proc/sys/.../IRQ */
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
	    DBGout(); return -EBUSY;
	}
    } else {
	/* Invalid IRQ number in /proc/.../IRQ */
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




#if CONFIG_TIME_MEASURE
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
}

void reset_measure_pin(void) 
{
    /* set port to output and initially to high */
}
#endif
