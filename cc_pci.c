/* CC CANCPC card handling
 *
 * This still is old-style handling
 * walking through the list of PCI devices manually
 * This has to be replaced by pci_register_driver()
 * and the pci_probe function()
 *
 * The PCI board has two I/O register areas
 * area 2 - tha SJA100 CAN controller
 * area 1: some registers controlling the on board LEDs
 *  and a hardware reset.
 * write a 0 to second_address_space_base_address+0x54
 * you will turn off the LEDs.
 * If you write a 0x16 to the same address you will see a green LED
 * and if you write an 0x32 you will see a red LED.
 *
 * If you set bit 30 (decimal) in area1_base_address+0x50
 * will enable the reset to the CAN controller.
 * Reset the bit to take the SJA1000 out of reset.
 * Perform a read-modify-write such that you don't affect any other bits
 * in this register.
 *
 */

#include "defs.h"
#include <linux/pci.h>

#ifdef CAN4LINUX_PCI
# ifndef CONFIG_PCI
#   error "trying to compile a PCI driver for a kernel without CONFIG_PCI"
# endif
#endif


/* hold base address of the I/O are with led control registers */
static unsigned long ledaddr[MAX_CHANNELS];

/* special handling of pci stuff of the Contemporary Controls CANPCI */
/* ================================================================= */
static char *pcican_devnames[] = {
	"PCICAN/CANopen",
	"PCICAN/DeviceNet"
};

/* used for storing the global pci register address */
/* one element more than needed for marking the end */
struct	pci_dev *Can_pcidev[MAX_CHANNELS + 1] = { NULL };


int Can_RequestIrq(int minor, int irq,
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)
    irqreturn_t (*handler)(int, void *))
#else
    irqreturn_t (*handler)(int, void *, struct pt_regs *)
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
 * Mainly it gets needed IO and IRQ resources and initializes
 * special hardware functions.
 *
 */

int CAN_VendorInit (int minor)
{
int i;

    DBGin();
    can_range[minor] = CAN_RANGE;

    /* PCI scan for CC_CANPCI (or others ) has already scanned
     * for the io address.
     * Here we request the io areas for exclusive use with this driver
     */


    i = pci_request_region(Can_pcidev[minor], 1, "CAN-LED");
    if(i != 0) {
	DBGout(); return -EBUSY;
    }
    i = pci_request_region(Can_pcidev[minor], 2, "CAN-IO");
    if(i != 0) {
	DBGout(); return -EBUSY;
    }

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


/* check memory region if there is a CAN controller
*  assume the controller was reseted before testing
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
    printk("controller_available(%lx, %d\n", address, offset);

    if ( 0x21 == inb(address))  {
	/* compare reset values of status and interrupt register */
	if(   0x0c == inb(address + (2 * offset))
	   && 0xe0 == inb(address + (3 * offset)) ) {
	    return 1;
	} else {
	    return 0;
	}
    } else {
	/* may be called after a 'soft reset' in 'PeliCAN' mode */
	/*   value     address                     mask    */
	if(   0x00 ==  inb(address + (1 * offset))
	   && 0x34 == (inb(address + (2 * offset))    & 0x37)
	   && 0x00 == (inb(address + (3 * offset))    & 0xfb)
	  ) {
	    return 1;
	} else {
	    return 0;
	}
    }
}

int pcimod_scan(void)
{
struct	pci_dev *pdev = NULL;
int	candev = 0;				/* number of found devices */
int i;


    while((pdev = pci_get_device (PCI_VENDOR_CAN_CC, PCI_ANY_ID, pdev))) {
	/* we are only interseted in these types of boards */
	if( !
	    (pdev->device == PCI_DEVICE_CC_CANopen)
	    ||
	    (pdev->device == PCI_DEVICE_CC_CANDnet)
	    ) continue;

	printk("  found CC CANPCI: %s\n", pci_name(pdev));
	printk("                 : %s at 0x%lx\n",
	    pcican_devnames[(pdev->device) - PCI_DEVICE_CC_MASK],
	    (long unsigned int)pci_resource_start(pdev, 1));

	if (pci_enable_device(pdev)) {
	    printk(" pci_enable_device not succeded\n");
	    continue;
	}
	printk(" using IRQ %d\n", pdev->irq);
#if 0
	{
	long ioaddr;
	for( i = 0; i < DEVICE_COUNT_RESOURCE; i++ ) {
	printk("   %d: 0x%lx\n",
		i, (long unsigned int)pci_resource_start(pdev, i));
	}
	printk(" using I/O at: 0x%lx\n",
		(long unsigned int)pci_resource_start(pdev, 1));
	i = pci_request_region(pdev, 1, "CAN-IO");
	printk("pci_request_region(1) = %d\n", i);

	ioaddr = pci_resource_start(pdev, 1);
	printk("ioaddr %lx\n", ioaddr);
	/* dump memory */
	for( i = 0; i < 64; i++ ) {
		printk("%02x ", inb(ioaddr + i));
	}
	printk("\n");
	}
#endif

	/* get the io area to scan for a CAN controller avaailable */
	i = pci_request_region(pdev, 2, "CAN-IO");

#if 0
	{
	long ioaddr;
	ioaddr = pci_resource_start(pdev, 2);
	printk("ioaddr %lx\n", ioaddr);
	/* dump memory */
	for( i = 0; i < 64; i++ ) {
		printk("%02x ", inb(ioaddr + i));
	}
	printk("\n");
	}
#endif
	/* look for a CAN controller at address 0 of the selected area */
	if( 1 /* controller_available(pci_resource_start(pdev, 2), 1)*/ ) {
	    printk(" CAN: %d. at pos 1\n", candev + 1);
	    Can_pcidev[candev] = pdev; /* store pdev for release at close() */
	    if(candev > MAX_CHANNELS) {
		printk("CAN: only %d devices supported\n", MAX_CHANNELS);
		break; /* the devices scan loop */
	    }
	    Base[candev] = pci_resource_start(pdev, 2);
	    ledaddr[candev] = pci_resource_start(pdev, 1);
	    IOModel[candev] = 'p';
	    IRQ[candev] = pdev->irq;
	    printk("ioports at %x, IRQ %d\n", (unsigned int)Base[candev], IRQ[candev]);
/* FIXME: should Clock be an array with values different for each CAN channel ?
 */
	    /* the value at the bit rate pre-scaler */
	    Clock = 8000000;
	    candev++;
	} else {
	    printk(" CAN: NO CAN controller found at pos 1\n");
	    ;
	}
	/* in all cases, release for now the CAN io area */
	pci_release_region(pdev, 2);
    }
    if (candev == 0)
    	/* no can device found */
    	return -ENODEV;
    else
	return 0;
}


void board_clear_interrupts(int minor)
{
}

/*
The CC PCICAN Board has only one LED, it as a bicolor LED.
That said, only three states are possible
 LED off
 green on
 red on

*/

void CAN_control_led(int minor, Command_par_t * argp)
{
int reg;
int val;

	DBGin();
	/* 16 green, 32 red */
	if (argp->val1 == 1)
		val = 0x16;	/* green */
	else if (argp->val1 == 4)
		val = 0x32;	/* red */
	else {
		DBGout();
		return;
	}
	reg = inb(ledaddr[minor] + 0x54);
	if (argp->val2) {
		outb(val, ledaddr[minor] + 0x54);
	}
	else { /* Swich off */
		outb(0, ledaddr[minor] + 0x54);
	}
	DBGout();
}
