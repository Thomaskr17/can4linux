
/* MPL VCMA9 using SJA1000 specific stuff
 * 
 * (c) 2006 oe@port.de
 */

#include "defs.h"



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

int controller_available(unsigned long address, int offset)
{
unsigned long ptr = (unsigned long)ioremap(address, 32 * offset);

    DBGin("controller_available");
    /* printk("controller_available 0x%lx\n", address); */


    /* printk("0x%0x, ", readb(ptr + (0 * offset)) ); */
    /* printk("0x%0x, ", readb(ptr + (2 * offset)) ); */
    /* printk("0x%0x\n", readb(ptr + (3 * offset)) ); */

    if ( 0x21 == readb((void __iomem *)ptr))  {
	/* compare rest values of status and interrupt register */
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


int CAN_VendorInit (int minor)
{
    DBGin("CAN_VendorInit");
/* 1. Vendor specific part ------------------------------------------------ */
    can_range[minor] = 0x100;
    
/* End: 1. Vendor specific part ------------------------------------------- */
    /* Request the controllers address space */
    if(NULL == request_mem_region(Base[minor], can_range[minor], "CAN-IO")) {
	DBGprint(DBG_DATA,("Request CAN-IO failed at 0x%x\n", Base[minor]));
	return -EBUSY;
    }


    can_base[minor] = ioremap(Base[minor], can_range[minor]);
    /* now the virtual address can be used for the register address macros */





/* 2. Vendor specific part ------------------------------------------------ */

/* End: 2. Vendor specific part ------------------------------------------- */

    if( IRQ[minor] > 0 || IRQ[minor] > MAX_IRQNUMBER ){
        int err;
	err = request_irq( IRQ[minor], CAN_Interrupt, SA_SHIRQ, 
				"Can", &Can_minors[minor]);
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
