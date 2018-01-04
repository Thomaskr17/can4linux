/* MMC Some board specific functions
 * Micro Maritime Controller Heyfra/Weidmüller ATMEL AT91SAM9260,
 * 3 externally connected SJA1000
 * suitable for maritime CANopen redundancy appications.
 * 
 */

#include "defs.h"

#include <asm/arch/gpio.h>
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
int controller_available(unsigned long address, int offset)
{
unsigned long ptr = (unsigned long)ioremap(address, 32 * offset);

    printk("controller_available 0x%0lx\n", address);


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

/* Called from __init,  once when driver is loaded */
void init_mmc_hw(void)
{

    /* PIN number can be found in linux/include/asm-arm/arch-at91/gpio.h
     * gpio_to_irq(AT91_PIN_PC12) translates the pin to an IRQ number
     * BUT if this definition is used in request_irq()
     * than the interrupt is _only_ a standard GPIO interrupt
     * not handled bei the AIC (advanced interrupt controller)
     * To use the features of the AIC, the correct IRQ
     * to be used is 
     *  IRQ[i]           =  AT91SAM9260_ID_IRQ0;
     * in core.c
    */ 

    /* PC12 == =x40+12 == 76,  PD12 == 0x60+12  == 108 */
    at91_set_A_periph(CAN_IRQ_PIN, 1);	/* External IRQ0, with pullup */

    at91_set_deglitch(AT91_PIN_PC12, 1);

    /* Documentation/arm/Interrupts */
    /* set_irq_type(CAN_IRQ_PIN, IRQ_TYPE_LEVEL_LOW); */
    set_irq_type(CAN_IRQ_PIN, IRQT_LOW);

#if CONFIG_TIME_MEASURE
    init_measure();
#endif
}

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

   /* We don't ned the interrupt to be shared for now */
   /* On AT91 
   	IRQF_DISABLED		- recognized
   	IRQF_NOBALANCING	- not
   	IRQF_TRIGGER_FALLING,	- not
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
 * special code for initializing the IRQ input Pin is done already
 * in init_mmc_hw() 
 * if the module is loaded.
*/

int CAN_VendorInit (int minor)
{
    DBGin();
    can_range[minor] = CAN_RANGE;

    /* Request the controllers address space */
    /* It's indexed Memory I/O, we need only some adresses */
    if(NULL == request_mem_region(Base[minor], can_range[minor], "CAN-IO")) {
	DBGprint(DBG_DATA,("Request_mem_region CAN-IO failed at %lx\n",
		Base[minor]));
	return -EBUSY;
    }

#if 0
    printk("success, got memory access!\n");
    printk("CAN[%d]: Base %lx\n", minor, Base[minor]);
    printk("CAN[%d]: base %p\n", minor, can_base[minor]);
    printk("CAN[%d]: range %x\n", minor, can_range[minor]);
#endif



    /* printk(" --> use ioremap\n"); */
    can_base[minor] = ioremap(Base[minor], can_range[minor]);

#if 0
    printk("CAN[%d]: base %p\n", minor, can_base[minor]);

    can_base[0] = ioremap(Base[0], 2);
    can_base[1] = ioremap(Base[1], 2);
    can_base[2] = ioremap(Base[2], 2);
	CAN_ShowStat (0);
	CAN_ShowStat (1);
	CAN_ShowStat (2);
#endif


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

    /* test for valid IRQ number in /proc/.../IRQ */
    if( IRQ[minor] > 0 && IRQ[minor] < MAX_IRQNUMBER ){
        int err;
	err = request_irq(IRQ[minor], CAN_Interrupt,
		IRQF_SHARED | IRQF_DISABLED | IRQ_TYPE_LEVEL_LOW,
		"Can", &Can_minors[minor]);

	if( !err ){
	    DBGprint(DBG_BRANCH,("Requested IRQ: %d @ %p",
				IRQ[minor], CAN_Interrupt));
	    set_irq_type(CAN_IRQ_PIN, IRQT_LOW);
	    /* arch/arm/mach-at91/irq.c */

	    IRQ_requested[minor] = 1;
	    DBGout(); return 0;
	}
	printk("Can[%d]: Can't request IRQ %d \n", minor, IRQ[minor]);
    } else {
	;
	DBGprint(DBG_DATA,("Can[%d]: invalid IRQ number %d\n",
						minor, IRQ[minor]));
    }

    /* if somethig fails, we are here */
    /* release I/O memory mapping -> release virtual memory */
    /* printk("iounmap %p \n", can_base[minor]); */
    iounmap(can_base[minor]);

    /* Release the memory region */
    /* printk("release mem %x \n", Base[minor]); */
    release_mem_region(Base[minor], can_range[minor]);

    DBGout(); return -EBUSY;
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
    at91_set_gpio_output(AT91_PIN_PC1, 0);
}

void set_measure_pin(void) 
{
    /* set port to output and initially to high */
    /* at91_set_gpio_output(AT91_PIN_PC1, 1); */
    at91_set_gpio_value(AT91_PIN_PC1, 1);
}

void reset_measure_pin(void) 
{
    /* set port to output and initially to high */
    /* at91_set_gpio_output(AT91_PIN_PC1, 0); */
    at91_set_gpio_value(AT91_PIN_PC1, 0);
}
#endif
