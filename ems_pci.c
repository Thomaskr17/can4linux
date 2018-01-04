/* EMS CPC-PCI card handling
 * 
 * This still is old-style handling
 * walking through the list of PCI devices manually
 * This has to be replaced by pci_register_driver()
 * and the pci_probe function()
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
    err = request_irq(irq, handler, IRQF_SHARED, "can4linux", &Can_minors[minor]);
#else
    err = request_irq(irq, handler, SA_SHIRQ, "Can", &Can_minors[minor]);
#endif


    if( !err ){
      DBGprint(DBG_BRANCH,("Requested IRQ: %d @ 0x%lx",
      				irq, (unsigned long)handler));
      IRQ_requested[minor] = 1;
    }
    DBGout(); return err;
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
static void reset_CPC_PCI(unsigned long address)
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
    printk("controller_available 0x%0lx\n", address);

    printk("0x%0x, ", readb(ptr) );
    printk("0x%0x, ", readb(ptr + (2 * offset)) );
    printk("0x%0x\n", readb(ptr + (3 * offset)) );

    /* return 1; */

#endif


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
upointer_t Can_pitapci_control[MAX_CHANNELS];

# if defined(CPC_PCI)



#ifndef PCI_DEVICE_ID_PLX_9030
#define PCI_DEVICE_ID_PLX_9030 0x9030
#endif

#if 0
static struct pci_device_id  ems_pci_tbl[] = {
      { PCI_DEVICE(PCI_VENDOR_ID_PLX, PCI_DEVICE_ID_PLX_9030), },
      { 0, },                 /* End of list */
};
#endif

#if 0      /* version to support both EMS-PCI boards - not used yet */
static int register_new_cpcpci(struct pci_dev *pdev, int i)
{
void __iomem *ptr;		/* ptr to PCI control registers*/
void __iomem *cptr;		/* ptr to start of CAN control registers*/
int j;				/* loop through possible CAN controllers */
int minor = -1;			/* to make DBGin() happy */

    DBGin();

    	                 /* dev, bar, size */
    ptr		= pci_iomap(pdev, 0, 128);
    cptr	= pci_iomap(pdev, 2, 2048);


    printk("cptr= %p \n", cptr);

    /* look for a CAN controllers starting at 0x400 */
    for(j = 0; j < 4; j++) {   
	if(controller_available(PCI_BASE_ADDRESS2(pdev) 
					+ 0x400 + (0x200 * j), 1)) {
	    err(" CAN controller %d. at pos %d\n", i + 1, j);
	    if(i > MAX_CHANNELS) {
		err("only %d devices supported\n", MAX_CHANNELS);
		break; /* the devices scan loop */
	    }
	    IOModel[i]	= 'm';
	    IRQ[i]	= pdev->irq;
	    Base[i]	= (upointer_t) cptr + 0x400 + (0x200 * j);
#ifdef __x86_64__
	    err("Base %llx/%lld",
	    	(long long unsigned)Base[i], (long long unsigned)Base[i]);
#else
	    err("Base %lx", Base[i]);
#endif
	    Can_pitapci_control[i] = (upointer_t)ptr;/* store pointer to control reg */
	    i++;
	}
    }
    /* enable IRQ in PLX 9030 */
    writel(PLX9030_ICR_ENABLE_IRQ0, ptr + PLX9030_ICR);
    DBGout();
    return i;	/* returns last CAN controller number found */
}



/* check if the pci device is a valid old style CPC-PCI and 
gett all the hardware information nneded and fill in the board information
the device itself is already registered  pci_enable_device()
*/
static int register_old_cpcpci(struct pci_dev *pdev, int i)
{
unsigned long ptr;		/* ptr to PITA control */
int minor = -1;			/* to make DBGin() happy */

    DBGin();

    err("int i ist %d", i);

    ptr = (unsigned long)ioremap(PCI_BASE_ADDRESS0(pdev), 256);
    /* enable memory access */
    /* printk("write to pita\n"); */
    writel(PITA2_MISC_CONFIG, (void __iomem *)ptr + PITA2_MISC);
    Can_pitapci_control[i] = ptr;

    /* printk("        pita ptr %lx\n", ptr); */
    /* printk("---------------\n"); */
    /* dump_CAN(PCI_BASE_ADDRESS1(pdev)+0x400, 4); */
    /* printk("---------------\n"); */
    /* dump_CAN(PCI_BASE_ADDRESS1(pdev)+0x600, 4); */

    /* PCI_BASE_ADDRESS1:
     * at address 0 are some EMS control registers
     * at address 0x400 the first controller area 
     * at address 0x600 the second controller area 
     * registers are read as 32bit
     *
     * at adress 0 we can verify the card
     * 0x55 0xaa 0x01 0xcb
*/
    {
	void __iomem *sigptr; /* ptr to EMS signature  */
	unsigned long signature = 0;
	sigptr = (void __iomem *)ioremap(PCI_BASE_ADDRESS1(pdev), 256);
	signature =
		  (readb(sigptr)      << 24)
		+ (readb(sigptr +  4) << 16)
		+ (readb(sigptr +  8) <<  8)
		+  readb(sigptr + 12);
	/* printk("        signature  %lx\n", signature); */
	if( 0x55aa01cb != signature) {
	    printk(" wrong signature -- no EMS CPC-PCI board\n");
	    return -ENODEV;
	}
    }
    /* we are now sure to have the right board,
       reset the CAN controller(s) */
    reset_CPC_PCI(PCI_BASE_ADDRESS1(pdev) + 0x400);
    reset_CPC_PCI(PCI_BASE_ADDRESS1(pdev) + 0x600);

    /* enable interrupts Int_0 */
    /* write to PITAs ICR register */
    writel(PITA2_ICR_INT0_EN,
    (void __iomem *)Can_pitapci_control[i] + PITA2_ICR);

    err("int i ist %d", i);
    /* look for a CAN controller at 0x400 */
    if(controller_available(PCI_BASE_ADDRESS1(pdev) + 0x400, 4)) {
	printk(" CAN: %d. at pos 1\n", i);
	if(i > MAX_CHANNELS) {
	    printk("CAN: only %d devices supported\n", MAX_CHANNELS);
	    return i; /* the devices scan loop */
	}
	Base[i]
	= (unsigned long)ioremap(PCI_BASE_ADDRESS1(pdev) + 0x400, 32*4);
	IOModel[i] = 'm';
	IRQ[i] = pdev->irq;
	i++;
    } else {
	/* printk(" CAN: NO at pos 1\n"); */
	;
    }

    err("int i ist %d", i);
    /* look for a second CAN controller at 0x400 */
    if(controller_available(PCI_BASE_ADDRESS1(pdev) + 0x600, 4)) {
	printk(" CAN: %d. at pos 2\n", i);
	if(i > MAX_CHANNELS) {
	    printk("CAN: only %d devices supported\n", MAX_CHANNELS);
	    return i; /* the devices scan loop */
	}
	/* share the board control register with prev ch */
	Can_pitapci_control[i] = 
	    Can_pitapci_control[i - 1];
	Base[i]
	= (unsigned long)ioremap(PCI_BASE_ADDRESS1(pdev) + 0x600, 32*4);
	IOModel[i] = 'm';
	IRQ[i] = pdev->irq;
	i++;
    } else {
	/* printk(" CAN: NO at pos 2\n"); */
	;
    }

    err("int i ist %d", i);
    DBGout();
    return i;
}

static int new_pcimod_scan(void)
{
struct	pci_dev *pdev = NULL;
int	candev = 0;			/* number of devices found */
int	minor = -1;			/* to make DBGin() happy */

    DBGin();
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
    /* Testing the PCI presence is no longer neccessary
     * On a non  PCI kernel, insmod will just complain
     */
    if (pci_present ()) {
#endif

    for_each_pci_dev(pdev) {
	if(pdev->vendor == PCI_VENDOR_ID_PLX
	&& pdev->device == PCI_DEVICE_ID_PLX_9030) {
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
	    err("found old EMS pci board %d", candev);
	    if (pci_enable_device(pdev)) {
		    continue;
	    } else {
		candev += register_old_cpcpci(pdev, candev);
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
#endif


/* Should be replaced by  new_pcimod_scan() soon
 * to be able to handle both kinds of CPC-PCI
 * hopefully for both boards with the same driver
 * and if possible for the Kvaser PCI as well
 */
int pcimod_scan(void)
{
struct	pci_dev *pdev = NULL;
int	candev = 0;				/* number of devices found */
unsigned long ptr;				/* ptr to PITA control */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
    /* Testing the PCI presence is no longer neccessary
     * On a non  PCI kernel, insmod will just complain
     */
    if (pci_present ()) {
#endif
	    while((pdev =
	    	pci_get_device (PCI_VENDOR_CAN_EMS, PCI_DEVICE_CAN, pdev))) {

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
	    printk("  found CPC-PCI: %s\n", pdev->name);
#else
	    printk("  found CPC-PCI: %s\n", pci_pretty_name(pdev));
	    printk("               : %s\n", pci_name(pdev));
#endif
	    if (pci_enable_device(pdev)) {
		continue;
	    }
	    /* printk("        using IRQ %d\n", pdev->irq); */

	    ptr = (unsigned long)ioremap(PCI_BASE_ADDRESS0(pdev), 256);
	    /* enable memory access */
	    /* printk("write to pita\n"); */
	    writel(PITA2_MISC_CONFIG, (void __iomem *)ptr + PITA2_MISC);
	    Can_pitapci_control[candev] = ptr;

	    /* printk("        pita ptr %lx\n", ptr); */
	    /* printk("---------------\n"); */
	    /* dump_CAN(PCI_BASE_ADDRESS1(pdev)+0x400, 4); */
	    /* printk("---------------\n"); */
	    /* dump_CAN(PCI_BASE_ADDRESS1(pdev)+0x600, 4); */

	    /* PCI_BASE_ADDRESS1:
	     * at address 0 are some EMS control registers
	     * at address 0x400 the first controller area 
	     * at address 0x600 the second controller area 
	     * registers are read as 32bit
	     *
	     * at adress 0 we can verify the card
	     * 0x55 0xaa 0x01 0xcb
     */
	    {
		void __iomem *sigptr; /* ptr to EMS signature  */
		unsigned long signature = 0;
	        sigptr = (void __iomem *)ioremap(PCI_BASE_ADDRESS1(pdev), 256);
	        signature =
	        	  (readb(sigptr)      << 24)
	        	+ (readb(sigptr +  4) << 16)
	        	+ (readb(sigptr +  8) <<  8)
	        	+  readb(sigptr + 12);
	    	/* printk("        signature  %lx\n", signature); */
	    	if( 0x55aa01cb != signature) {
	    	    printk(" wrong signature -- no EMS CPC-PCI board\n");
		    return -ENODEV;
	    	}
	    }
	    /* we are now sure to have the right board,
	       reset the CAN controller(s) */
	    reset_CPC_PCI(PCI_BASE_ADDRESS1(pdev) + 0x400);
	    reset_CPC_PCI(PCI_BASE_ADDRESS1(pdev) + 0x600);

	    /* enable interrupts Int_0 */
	    /* write to PITAs ICR register */
	    writel(PITA2_ICR_INT0_EN,
	    (void __iomem *)Can_pitapci_control[candev] + PITA2_ICR);

	    /* look for a CAN controller at 0x400 */
	    if(controller_available(PCI_BASE_ADDRESS1(pdev) + 0x400, 4)) {
		printk(" CAN: %d. at pos 1\n", candev + 1);
		if(candev > MAX_CHANNELS) {
		    printk("CAN: only %d devices supported\n", MAX_CHANNELS);
		    break; /* the devices scan loop */
		}


#ifdef __x86_64__
		Base[candev]
		= (upointer_t)ioremap(PCI_BASE_ADDRESS1(pdev) + 0x400, 32*4);
	    err("Base %llx/%lld", 
	    	(long long unsigned)Base[candev],
	    	(long long unsigned)Base[candev]);
	    err("Base %lx", Base[candev]);
#else
	    err("Base %lx", Base[candev]);
		Base[candev]
		= (unsigned long)ioremap(PCI_BASE_ADDRESS1(pdev) + 0x400, 32*4);
#endif

		IOModel[candev] = 'm';
		IRQ[candev] = pdev->irq;
		/* all EMS PCI Boards use the sam clock for all CAN */
		Clock = 8000000;
		candev++;
	    } else {
		printk(" CAN: NO CAN at pos 1\n");
		;
	    }
	    /* look for a second CAN controller at 0x400 */
	    if(controller_available(PCI_BASE_ADDRESS1(pdev) + 0x600, 4)) {
		printk(" CAN: %d. at pos 2\n", candev + 1);
		if(candev > MAX_CHANNELS) {
		    printk("CAN: only %d devices supported\n", MAX_CHANNELS);
		    break; /* the devices scan loop */
		}
		/* share the board control register with prev ch */
		Can_pitapci_control[candev] = 
		    Can_pitapci_control[candev - 1];
		Base[candev]
		= (unsigned long)ioremap(PCI_BASE_ADDRESS1(pdev) + 0x600, 32*4);
		IOModel[candev] = 'm';
		IRQ[candev] = pdev->irq;
		candev++;
	    } else {
		printk(" CAN: NO CAN at pos 2\n");
		;
	    }
	    /* all EMS CPC PCI Boards are using 16 MHz CAN Clock
	     * divided by two athe pre-scaler input */
	    Clock = 8000000;
	}
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
    } else {
        printk("CAN: No PCI bios present\n");
        return -ENODEV;
    }
#endif
    return 0;
}


void board_clear_interrupts(int minor)
{

/* old  Siemens PITA */
    /* Interrupt_0_Enable (bit 17) + Int_0_Reset (bit 1) */
    /*  
     Uttenthaler:
      nur 
        writel(0x00020002, Can_pitapci_control[minor] + 0x0);
      als letzte Anweisung in der ISR
     Schoett:
      bei Eintritt
        writel(0x00000000, Can_pitapci_control[minor] + 0x0);
      am ende
        writel(0x00020002, Can_pitapci_control[minor] + 0x0);
    */
    writel(0x00020002, (void __iomem *)Can_pitapci_control[minor] + 0x0);
    writel(0x00020000, (void __iomem *)Can_pitapci_control[minor] + 0x0);

/* new */

    writel(PLX9030_ICR_CLEAR_IRQ0 | PLX9030_ICR_ENABLE_IRQ0,
          (void __iomem *)Can_pitapci_control[minor] + PLX9030_ICR);

}
# endif 	/* defined(CPC_PCI) */

#endif
