/* EMS CPC-PCI card handling   for the new 4 channnel PCI board
 * available since end of 2008
 * 
 * This still is old-style handling
 * walking through the list of PCI devices manually
 * This has to be replaced by pci_register_driver()
 * and the pci_probe function()
 */

/*
 * The PCI adapter is accessable through memory-access read/write,
 * not I/O read/write.
 * Thus, we need to map it to some virtual address area
 * in order to access the registers as normal memory.



plx9030_clear_interrupts



*/


#include "defs.h"
#include <linux/pci.h>


#ifdef CAN4LINUX_PCI
# ifndef CONFIG_PCI
#   error "trying to compile a PCI driver for a kernel without CONFIG_PCI"
# endif

#ifdef CAN4LINUX_PCI


int Can_RequestIrq(int minor, int irq,
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18) 
    irqreturn_t (*handler)(int, void *))
#else
    irqreturn_t (*handler)(int, void *, struct pt_regs *))
#endif
{
int err = 0;

    DBGin();
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

    */

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18) 
    err = request_irq(irq, handler, IRQF_SHARED, "Can", &Can_minors[minor]);
#else
    err = request_irq(irq, handler, SA_SHIRQ, "Can", &Can_minors[minor]);
#endif


    if( !err ){
      DBGprint(DBG_BRANCH,("Requested IRQ: %d @ 0x%lx",
      				irq, (unsigned long)handler));
      IRQ_requested[minor] = 1;
    }
    DBGout(); return err;


    	disable_irq(irq);
	/* enable IRQ in PLX 9030 */
	writel(PLX9030_ICR_ENABLE_IRQ0,
          (void __iomem *)Can_pitapci_control[minor] + PLX9030_ICR);
	enable_irq(irq);

}


int Can_FreeIrq(int minor, int irq )
{
    DBGin();
    IRQ_requested[minor] = 0;
    /* printk(" Free IRQ %d  minor %d\n", irq, minor);  */
    free_irq(irq, &Can_minors[minor]);
    DBGout();
    return 0;
}

/*
 * Perform Vendor-Init, that means sometimes CAN controller
 * or only board manufacturer specific initialization.
 *
 * Mainly it gets needed IO and IRQ ressources and initilaizes 
 * special hardware functions.
 *
 */

int CAN_VendorInit (int minor)
{
    DBGin();
    can_range[minor] = CAN_RANGE;
    
    /* PCI scan for CPC-PCI (or others ) has already remapped the address */
    /* printk(" assign address direct\n"); */
    can_base[minor] = (void __iomem *)Base[minor];

    /* The Interrupt Line is alrady requestes by th PC CARD Services
     * (in case of CPC-Card: cpc-card_cs.c) 
    */ 

    /* printk("MAX_IRQNUMBER %d/IRQ %d\n", MAX_IRQNUMBER, IRQ[minor]); */
    if( IRQ[minor] > 0 && IRQ[minor] < MAX_IRQNUMBER ){
        if( Can_RequestIrq( minor, IRQ[minor] , CAN_Interrupt) ) {
	     printk("Can[%d]: Can't request IRQ %d \n", minor, IRQ[minor]);
	     DBGout(); return -EBUSY;
        }
    } else {
	/* Invalid IRQ number in /proc/.../IRQ */
	DBGout(); return -EINVAL;
    }
    DBGout(); return 0;
}








/* reset both CAN controllers on the EMS-Wünsche CPC-PCI Board */
/* writing to the control range at BAR1 of the PCI board */
void reset_CPC_PCI(unsigned long address)
{
unsigned long ptr = (unsigned long)ioremap(address, 32);
    writeb(0x01, (void __iomem *)ptr);
}

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
/* unsigned long ptr = (unsigned long)ioremap(address, 32 * offset); */
void __iomem *ptr = ioremap(address, CAN_RANGE);


#if 0     /* debugging */
    printk("controller_available 0x%0lx, offset used %d\n",
    			address, offset);

    printk("0x%0x, ", readb(ptr) );
    printk("0x%0x, ", readb(ptr + (2 * offset)) );
    printk("0x%0x\n", readb(ptr + (3 * offset)) );

    /* return 1; */

#endif
    /* Try to reset the CAN Controller before reading it.
     * Not really correct, in case it's not a CAN card, anyway.
     */
    writeb(CAN_RESET_REQUEST, ptr);

    if ( 0x21 == readb(ptr))  {
	/* compare reset values of status and interrupt register */
	if(   0x0c == readb(ptr + (2 * offset))
	   && 0xe0 == readb(ptr + (3 * offset)) ) {
	    return 1;
	} else {
	    return 0;
	}
    } else {
	/* may be called after a 'soft reset' in 'PeliCAN' mode */
	/*   value     address                     mask    */
	if(   0x00 ==  readb(ptr + (1 * offset))
	   && 0x34 == (readb(ptr + (2 * offset))    & 0x37)
	   && 0x00 == (readb(ptr + (3 * offset))    & 0xfb)
	  ) {
	    return 1;
	} else {
	    return 0;
	}

    }
}
#endif




#define PCI_BASE_ADDRESS0(dev) (dev->resource[0].start)
#define PCI_BASE_ADDRESS1(dev) (dev->resource[1].start)
#define PCI_BASE_ADDRESS2(dev) (dev->resource[2].start)
#define PCI_BASE_ADDRESS3(dev) (dev->resource[3].start)

/* used for storing the global pci register address */
void __iomem *Can_pitapci_control[MAX_CHANNELS];

# if defined(CPC_PCI2)



#ifndef PCI_DEVICE_ID_PLX_9030
#define PCI_DEVICE_ID_PLX_9030 0x9030
#endif

#if 0
static struct pci_device_id  ems_pci_tbl[] = {
      { PCI_DEVICE(PCI_VENDOR_ID_PLX, PCI_DEVICE_ID_PLX_9030), },
      { 0, },                 /* End of list */
};
#endif

static int register_new_cpcpci(struct pci_dev *pdev, int i)
{
void __iomem *ptr;		/* ptr to PCI control registers*/
void __iomem *cptr;		/* ptr to start of CAN control registers*/
int j;				/* loop through possible CAN controllers */
int minor = -1;			/* to make DBGin() happy */

    DBGin();
    	                 /* dev, bar, size */
    ptr		= pci_iomap(pdev, 0, 128);      /* control base address */
    cptr	= pci_iomap(pdev, 2, 2048);     /* CAN pointer */

    err("register_new_cpcpci %d", i);
    printk("cptr= %p \n", cptr);

    /* look for a CAN controllers starting at 0x400 */
    for(j = 0; j < 4; j++) {   
	if(controller_available(PCI_BASE_ADDRESS2(pdev) 
		+ CPC_PCI_CHANNEL_BASE + (CPC_PCI_CHANNEL_WIDTH * j), 1)) {
	    err(" CAN controller %d. at pos %d\n", i + 1, j);
	    if(i > MAX_CHANNELS) {
		err("only %d devices supported", MAX_CHANNELS);
		break; /* the devices scan loop */
	    }
	    IOModel[i]	= 'm';
	    IRQ[i]	= pdev->irq;
	    Base[i]	= (upointer_t) cptr
	                  + CPC_PCI_CHANNEL_BASE
	                  + (CPC_PCI_CHANNEL_WIDTH * j);
	    /* all EMS PCI Boards use the sam clock for all CAN */
	    Clock = 8000000;
#ifdef __x86_64__
	  /*  err("Base %llx/%lld",
		(long long unsigned) Base[i], (long long unsigned)Base[i]); */
#else
	    err("Base %lx", Base[i]);
#endif
	    /* store pointer to control reg */
	    Can_pitapci_control[i] = ptr;
	    i++;
	}
    }
    disable_irq(IRQ[i]);
    /* enable IRQ in PLX 9030 */
    writel(PLX9030_ICR_ENABLE_IRQ0, ptr + PLX9030_ICR);
    enable_irq(IRQ[i]);
    DBGout();
    return i;	/* returns last CAN controller number found */
}

/* Scan the PCI bus for CAN board with PLX_9030
 * and test if a CAN controller is on it.
 * If yes fill our internal structures
 */
int pcimod_scan(void)
{
struct	pci_dev *pdev = NULL;
int	candev = 0;			/* number of found devices */
int	minor = -1;			/* to make DBGin() happy */

    DBGin();
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
    /* Testing the PCI presence is no longer neccessary
     * On a non  PCI kernel, insmod will just complain
     */
    if (pci_present ()) {
#endif

    for_each_pci_dev(pdev) {
	/* too many devices are using a PLX9030,
	   we have to check  sub-system ids as well */
	if(   pdev->vendor           == PCI_VENDOR_ID_PLX
	   && pdev->device           == PCI_DEVICE_ID_PLX_9030
	   && pdev->subsystem_vendor == 0x10b5
	   ) {
	    err("found new EMS pci board %d", candev);
	    printk("Subsystem Vendor 0x%0x\n", pdev->subsystem_vendor);
	    printk("Subsystem Device 0x%0x\n", pdev->subsystem_device);
	    /* reading delivers 0x10b5, 0x4000 */
	    if (pci_enable_device(pdev)) {
		    continue;
	    } else {
		candev += register_new_cpcpci(pdev, candev);
	    }
	}
	if(pdev->vendor == PCI_VENDOR_CAN_EMS
	&& pdev->device == PCI_DEVICE_CAN) {
	    err("found old EMS pci board %d\nUse TARGET=CPC_PCI compiled module", candev);
	    continue;
	    /* following code not used yet */
	    if (pci_enable_device(pdev)) {
		    continue;
	    } else {
		/* candev += register_old_cpcpci(pdev, candev); */
		;
	    }
	}
    }
    if(candev == 0) {
	err("No CAN device found");
	return -ENODEV;
    } else {
    	err("found %d CAN controllers", candev);
    }

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
    } else { 
        err("No PCI bios present");
        return -ENODEV;
    }
#endif
    DBGout();
    return 0;
}



void board_clear_interrupts(int minor)
{
    /* err("board_clear_interrupts %d", minor); */

    writel(PLX9030_ICR_CLEAR_IRQ0 | PLX9030_ICR_ENABLE_IRQ0,
          (void __iomem *)Can_pitapci_control[minor] + PLX9030_ICR);
}
# endif 	/* defined(CPC_PCI) */





#if CONFIG_TIME_MEASURE
/* Functions to Switch the stet of output pins
   used for time measurement within the CAN ISR
 */
void init_measure(void)
{
}

void set_measure_pin(void) 
{
}

void reset_measure_pin(void) 
{
}
#endif


#endif
