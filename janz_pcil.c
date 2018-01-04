/* Janz CPC-PCIL card handling
 *
 * 2013 hj.oertel@t-online.de
 *
 * This still is old-style handling
 * walking through the list of PCI devices manually
 * This has to be replaced by pci_register_driver()
 * and the pci_probe function()
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include "defs.h"
#include <linux/pci.h>



#ifdef CAN4LINUX_PCI
# ifndef CONFIG_PCI
#   error "trying to compile a PCI driver for a kernel without CONFIG_PCI"
# endif

int Can_RequestIrq(int minor, int irq,
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 18)
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

       If the kernel receives an interrupt he knows
       how often it was registered,
       and calls the ISR for each registered interrupt.
       In this case, can4linux, for example
       every time with a different pointer to Can_minors[]
       contaiing the device minor which has registered.
       The ISR should now check this CAN controller relaed to the minor
       number, if it was the cause of the interrupt.

	#define IRQ_TYPE_NONE           // Default, unspecified type
	#define IRQ_TYPE_EDGE_RISING    // Edge rising type
	#define IRQ_TYPE_EDGE_FALLING   // Edge falling type
	#define IRQ_TYPE_EDGE_BOTH (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING)
	#define IRQ_TYPE_LEVEL_HIGH     // Level high type
	#define IRQ_TYPE_LEVEL_LOW      // Level low type
	#define IRQ_TYPE_SENSE_MASK     // Mask of the above
	#define IRQ_TYPE_PROBE          // Probing in progress


*/

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 18)
	err = request_irq(irq, handler, IRQF_SHARED, "Can", &Can_minors[minor]);
#else
	err = request_irq(irq, handler, SA_SHIRQ, "Can", &Can_minors[minor]);
#endif


	if (!err ){
		DBGprint(DBG_BRANCH,("Requested IRQ: %d @ 0x%lx",
					irq, (unsigned long)handler));
		IRQ_requested[minor] = 1;
	}
	DBGout();
	return err;
}


int Can_FreeIrq(int minor, int irq )
{
	DBGin();
	IRQ_requested[minor] = 0;
	/* pr_info(" Free IRQ %d  minor %d\n", irq, minor);  */
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
	DBGin();
	can_range[minor] = CAN_RANGE;

	/* PCI scan for CPC-PCI (or others ) has already remapped the address */
	/* pr_info(" assign address direct\n"); */
	can_base[minor] = (void __iomem *)Base[minor];

	/* The Interrupt Line is already requested by th PC CARD Services
	* (in case of CPC-Card: cpc-card_cs.c)
	*/

	/* pr_info("MAX_IRQNUMBER %d/IRQ %d\n", MAX_IRQNUMBER, IRQ[minor]); */
	if (IRQ[minor] > 0 && IRQ[minor] < MAX_IRQNUMBER) {
		if (Can_RequestIrq( minor, IRQ[minor] , CAN_Interrupt)) {
			pr_err("Can[%d]: Can't request IRQ %d \n",
					minor, IRQ[minor]);
			DBGout();
			return -EBUSY;
		}
	} else {
		/* Invalid IRQ number in /proc/.../IRQ */
		DBGout();
		return -EINVAL;
	}
	DBGout();
	return 0;
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

int controller_available(void __iomem *ptr, int offset)
{

#if 1     /* debugging */
	pr_info("controller_available 0x%p\n", ptr);

	pr_info("%p:0x%0x, ", ptr, readb(ptr) );
	pr_info("%p:0x%0x, ", ptr + (2 * offset), readb(ptr + (2 * offset)));
	pr_info("%p:0x%0x\n", ptr + (3 * offset), readb(ptr + (3 * offset)));
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
	if(   (0x00 ==  readb(ptr + (1 * offset))	    )
	   && (0x34 == (readb(ptr + (2 * offset))    & 0x37))
	   && (0x00 == (readb(ptr + (3 * offset))    & 0xfb))
	  ) {
	    return 1;
	} else {
	    return 0;
	}

    }
}
#endif


/* used for storing the global pci register address */
upointer_t Can_pci_control[MAX_CHANNELS];
void __iomem *Can_ob_control[MAX_CHANNELS]; /* on board control registers */


#ifndef PCI_DEVICE_ID_PLX_9030
#define PCI_DEVICE_ID_PLX_9030 0x9030
#endif

#if 0
static struct pci_device_id  ems_pci_tbl[] = {
      { PCI_DEVICE(PCI_VENDOR_ID_PLX, PCI_DEVICE_ID_PLX_9030), },
      { 0, },                 /* End of list */
};
#endif
/* pdev - pci dev struct
 * i    - last number of CAN controller, where to start now
 * n    - max number of CAN controllers on Board
 */
int register_pcil(struct pci_dev *pdev, int i, int n)
{
void __iomem *ptr;		/* ptr to PCI control registers*/
void __iomem *cptr;		/* ptr to start of CAN control registers*/
void __iomem *rptr;		/* ptr to on board registers */
int j;				/* loop through possible CAN controllers */
int minor = -1;			/* to make DBGin() happy */

    DBGin();
    /* JANZ CAN_PCIL
       Adressspace 2   - CAN address space
       Adressspace 4   - On board registers
     */
    ptr		= pci_iomap(pdev, 0, 128);
    cptr	= pci_iomap(pdev, 2, 2048); /* CAN address space  */
    rptr	= pci_iomap(pdev, 4, 8);    /* On-Board registers */	


    /* pr_info("cptr= %p \n", cptr); */

    /* look for a CAN controllers starting at 0x400 */

    /* for some reasons, the controller_available() check does not work
       but fortunately, Janz is coding the number of CAN controllers
       in the Subsystem ID
      */
    for(j = 0; j < n; j++) {
	/* loop over the board's CAN controllers */
#if 0
	if(controller_available(cptr
		+ JANZ_PCIL_CHANNEL_BASE
		+ (JANZ_PCIL_CHANNEL_WIDTH * j), 1)) {
	    err(" CAN controller %d. at pos %d\n", i + 1, j);
#endif
	    if(i > MAX_CHANNELS) {
		pr_err("only %d devices supported\n", MAX_CHANNELS);
		break; /* the devices scan loop */
	    }
	    IOModel[i]	= 'm';
	    IRQ[i]	= pdev->irq;
	    Base[i]	= (upointer_t) cptr
	    		+ JANZ_PCIL_CHANNEL_BASE
	    		+ (JANZ_PCIL_CHANNEL_WIDTH * j);
#if defined(DEBUG)
#ifdef __x86_64__
	    pr_info(" Base[%d] address %llx\n", i, (long long unsigned)Base[i]);
#else
	    pr_info(" Base[%d] address %lx\n", i, Base[i]);
#endif
#endif
	    /* store pointer to control reg */
	    Can_pci_control[i] = (upointer_t)ptr;
	    /* store pointer to on board  reg */
	    Can_ob_control[i] = rptr;
	    i++;

	    /* enable INT of CAN controllers
	     * On-board registers
               address space (Base address register 4)
               provides some on-board registers,
               run-time configuration purposes
               that are outside of the scope
               of the 9030's local configuration registers.
	                         read       write
		0x1 (byte only!) INT_STAT    INT_DISABLE
		0x3 (byte only!) BOARD_NUM   INT_ENABLE
		0x5 (byte only!)      -      RESET_ASSERT
		0x7 (byte only!)      -      RESET_DEASSERT
	     */

	    /* read hex switch coding the board id */
	    proc_board_id[i] = readb(rptr +3);

	    /* assert reset */
	    writeb((1 << (i % 2)), (rptr + 5));
	    udelay(10);
	    /* de assert reset */
	    writeb((1 << (i % 2)), (rptr + 7));
	    /* enable INT of CAN controllers : offset 3 */
	    writeb((1 << (i % 2)), (rptr + 3));
#if 0
	}
#endif
    }
	if ( i > 0 ) {
		/* found CAN */
		/* enable IRQ in PLX 9030 */
		/* writel(PLX9030_ICR_ENABLE_IRQ0, ptr + PLX9030_ICR); */
	}
	DBGout();
	return i;	/* returns last CAN controller number found */
}




int pcimod_scan(void)
{
struct	pci_dev *pdev = NULL;
int	candev = 0;			/* number of found devices */
int	minor = -1;			/* to make DBGin() happy */
int	can_on_board;			/* CAN controllers on one board */

    DBGin();
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 5, 0)
    /* Testing the PCI presence is no longer neccessary
     * On a non  PCI kernel, insmod will just complain
     */
    if (pci_present()) {
#endif

    for_each_pci_dev(pdev) {
	if (pdev->vendor == PCI_VENDOR_ID_PLX
	&& pdev->device == PCI_DEVICE_ID_PLX_9030) {
	    /* pr_info("found PLX 9030 pci board %d\n", candev); */
	    /* pr_info("Subsystem Vendor 0x%0x\n", pdev->subsystem_vendor); */
	    pr_info("Subsystem Device 0x%0x\n", pdev->subsystem_device);
	    /*
		Subsystem Vendor ID 0x13C3
		Subsystem ID CAN-PCIL/1 0x19xx
		Subsystem ID CAN-PCIL/2 0x1Axx
	    reading delivers 0x10b5, 0x4000
	    */
	    if(pdev->subsystem_vendor == PCI_SUBVENDOR_ID) {
		pr_info("found JANZ AG CAN board, starting with CAN%d\n", candev);
		if((pdev->subsystem_device && 0xFF00) == 0x1900) {
		    pr_info(" CAN-PCIL with one CAN\n");
		    can_on_board = 1;
		} else if((pdev->subsystem_device & 0xFF00) == 0x1a00) {
		    pr_info(" CAN-PCIL with two CAN\n");
		    can_on_board = 2;
		}
		else
		  continue;    /* PCI board scan loop */

		if (pci_enable_device(pdev)) {
			continue;
		} else {
		    candev += register_pcil(pdev, candev, can_on_board);
		}
	    }
	}
    }
    if(candev == 0) {
	pr_info("No CAN device found\n");
	return -ENODEV;
    } else {
    	pr_info("found %d CAN controllers\n", candev);
    }

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 5, 0)
    } else {
        pr_info("No PCI bios present\n");
        return -ENODEV;
    }
#endif
    DBGout();
    return 0;
}




void board_clear_interrupts(int minor)
{
#if 0
	writel(PLX9030_ICR_CLEAR_IRQ0 | PLX9030_ICR_ENABLE_IRQ0,
		(void __iomem *)Can_pci_control[minor] + PLX9030_ICR);
	writeb(3, Can_ob_control[minor] + 1);
#endif
}

/*
 * The JANZ CAN-PCIL has two LEDs assigned to the first two CAN controllers
 * of each board. One green, one red.
 *
 * CAN_LED[01] register is write only
 */
void CAN_control_led(int minor, Command_par_t * argp)
{
int val;
static int led_state[MAX_CHANNELS];

void __iomem *led_ptr = (canregs_t *)can_base[minor];


	DBGin();

	led_ptr += CAN_LED_ADDRESS;

	/* 16 green, 32 red */
	if (argp->val1 == 1)
		val = CAN_LED_GREEN;
	else if (argp->val1 == 4)
		val = CAN_LED_RED;
	else {
		DBGout();
		return;
	}
    	if (argp->val2) {
		led_state[minor] |=  val;
	}
	else {
		led_state[minor] &=  ~val;
	}
	writeb(led_state[minor], led_ptr);
	DBGout();
}

/*
 * The JANZ CAN-PCIL can activate an on board termination resistor
 * by software for the first two CAN controllers on board.
 * Not for the CAN controllers on the extension module.
 * BUT only if the appropriate hardware jumper is not set on the board.
 *
 * CAN_TERM[01] register is write only
 */
void can_control_termination(int minor, Command_par_t * argp)
{
void __iomem *term_ptr = (canregs_t *)can_base[minor];

	DBGin();
	term_ptr += CAN_TERM_ADDRESS;

	if (argp->val1 == term_on) {
		/* pr_info("Switch termination on\n"); */
		writeb(CAN_TERM_ON, term_ptr);

	} else if (argp->val1 == term_off) {
		/* pr_info("Switch termination off\n"); */
		writeb(CAN_TERM_OFF, term_ptr);
	}
	DBGout();
}
