/* cpc_104.c     - can4linux source file 
 *
 * EMS Wünsche CPC-104 using SJA1000 specific stuff
 * This version supports both, CPC-104 one channel SJA100
 * and CPC-104M with up to four channel SJA1000
 * 
 * (c) 2006 oe@port.de
 * (c) 2006 haas@ems-wuensche.com
 */

#include "defs.h"

static void * card_base[MAX_CHANNELS] = {NULL};

#define MAX_CPC104_CHANNELS 4
#define MAX_CPC104_CARDS MAX_IRQNUMBER

struct cpc104 {
	int interrupt_allocations;
	int card_resetted;
	struct cpc104_channel {
		int minor;
		int opened;
	} channels[MAX_CPC104_CHANNELS];
} cpc104_cards[MAX_CPC104_CARDS];

static int initialize_structs = 1;

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

irqreturn_t CPC104_Wrapped_Interrupt ( int irq, void *dev_id, struct pt_regs *ptregs )
{
	int cardidx = irq;	
	int chanidx;
	int minor = *(int*)dev_id;
	irqreturn_t retval = IRQ_NONE;

	do {
		for(chanidx = 0; chanidx < MAX_CPC104_CHANNELS; chanidx++) {
			if(cpc104_cards[cardidx].channels[chanidx].opened) {
				if(CAN_Interrupt(irq, &Can_minors[cpc104_cards[cardidx].channels[chanidx].minor], ptregs) == IRQ_HANDLED) {
					retval = IRQ_HANDLED;
				}
			}
		}
		if(readb(card_base[minor] + 6) != 0x02) {
			printk(KERN_INFO "CPC-104mc: CAN Controller registers not mapped. Status %2.2X\n", readb(card_base[minor] + 6));
			break;
		}
	} while(readb(card_base[minor] + 8) != 0x00); // iterate until no more IRQs need to be processed
	
	return retval;
}

int CAN_VendorInit (int minor)
{
	unsigned long signature = 0;
	int cardidx = IRQ[minor]; // we use the interrupt to index the cards
	int chanidx = ((Base[minor] >> 8) & 0x7) - 1; // get the channel number from its base address
	int i = 0;
	int j = 0;
	
	DBGin("CAN_VendorInit");
	can_range[minor] = 0x100;

	if(initialize_structs) {
		initialize_structs = 0;

		for(i = 0; i < MAX_CPC104_CARDS; i++) {
			cpc104_cards[i].interrupt_allocations = 0;
			cpc104_cards[i].card_resetted = 0;
			for(j = 0; j < MAX_CPC104_CHANNELS; j++) {
				cpc104_cards[i].channels[j].minor = 0;
				cpc104_cards[i].channels[j].opened= 0;
			}
		}
	}
    
	if( IRQ[minor] == 0 || IRQ[minor] > MAX_IRQNUMBER || chanidx < 0 || chanidx > MAX_CPC104_CHANNELS) {
		printk(KERN_INFO "Wrong Interrupt or Channel number %d %d\n", IRQ[minor], chanidx);
		return -EINVAL;
	}

	/* Some LINUX systems, e.g. the PIP10 I tested on,
	 * locate already the memory using the information
	 * provided in the "Option ROM"
	 * The memory is marked as "Adapter-ROM" in /proc/iomem.
	 * In this case the drive should not try to allocate the IO mem
	 */

#if !defined(PC104_OPTION_ROM)
	/* Request the controllers address space */
	if(NULL == request_mem_region(Base[minor], can_range[minor], "CAN-IO")) {
		DBGprint(DBG_DATA,("Request_mem-Region CAN-IO failed at 0x%x\n", Base[minor]));
		return -EBUSY;
	}
#endif

	can_base[minor]  = ioremap(Base[minor], can_range[minor]);
	card_base[minor] = ioremap(Base[minor] & 0xFF000, can_range[minor]);
	
	/* now the virtual address can be used for the register access macros */

	/* Signature of the CPC-104 Board
	 * byte offset	value description
	 * 0		0x55 for Read / acts as control register for Write
	 * 1		0xaa
	 * 2		occupied memory range in 512 byte units (0x01)
	 * 3		0xcb
	 * 4		CAN controller id
		    1 - 82527
		    2 - 82c200
		    8 - SJA1000
	6		Status register
	*/
	
	signature = readl(card_base[minor]);

	if(signature == 0xCB01AA55 || signature == 0xCB03AA55) {
		cpc104_cards[cardidx].channels[chanidx].minor = minor; // remember minor
		if(!cpc104_cards[cardidx].card_resetted) {
			/* Hardware reset of CAN controller.
			 * The minimum reset time for the individual controllers
			 * is generated by logic on CPC-104.
			 */
			writeb(0, card_base[minor]);
			mdelay(10);
		
			/* Map CAN controller into memory address range. */
			writeb(3, card_base[minor]);

			if(signature == 0xCB03AA55) {
				printk(KERN_INFO "CPC-104mc found.\n");
				writeb(0xFF, card_base[minor]+7); /* Enable */
			}

			/* We have to set all CAN controllers to PeliCAN mode
			 * and setup the clock, because each controller depends
			 * on the clock from it's previous controller. */
			{
				void *ccs = ioremap((Base[minor] & 0xFF000)|0x100, 4*can_range[minor]);
				if(ccs) {
					int i = 0;
					unsigned char *cc = (unsigned char *)ccs;
					for(i = 0; i < MAX_CPC104_CHANNELS; i++) {
						writeb(CAN_MODE_PELICAN + CAN_MODE_CLK, cc+31);
						cc += 0x100;
					}
					iounmap(ccs);
				}
			}			
			cpc104_cards[cardidx].card_resetted = 1;
		}
	} else {
		printk(KERN_INFO "CPC-104(mc) wrong card Signature\n");
		/* release I/O memory mapping -> release virtual memory */
		iounmap(can_base[minor]);
		iounmap(card_base);
		
#if !defined(PC104_OPTION_ROM)
		/* Release the memory region */
		release_mem_region(Base[minor], can_range[minor]);
#endif

		return -EBUSY;
	}

	if(cpc104_cards[cardidx].interrupt_allocations == 0) {
		int err;
		err = request_irq(IRQ[minor], CPC104_Wrapped_Interrupt, SA_SHIRQ, "Can", &Can_minors[minor]);
		
		if( !err ) {
			DBGprint(DBG_BRANCH,("Requested IRQ: %d @ 0x%lx", IRQ[minor], (unsigned long)CAN_Interrupt));
			IRQ_requested[minor] = 1;
		} else {
			release_mem_region(Base[minor], can_range[minor]);
			DBGout(); return -EBUSY;
		}
	}
	
	cpc104_cards[cardidx].interrupt_allocations++;

	cpc104_cards[cardidx].channels[chanidx].opened = 1;
	DBGout();
	
	return 0;
}

/* Release IRQ and IO ressources */
int CAN_Release(int minor)
{
	int cardidx = IRQ[minor];
	int chanidx = ((Base[minor] >> 8) & 0x7) - 1; // get the channel number from its base address
	
	DBGin("CAN_Release()");

	cpc104_cards[cardidx].channels[chanidx].opened = 0;
	
	if(--cpc104_cards[cardidx].interrupt_allocations == 0) {
		printk(KERN_INFO "Freeing IRQ%d\n", IRQ[minor]);
		/* call this before freeing any memory or io area.
		 * this can contain registers needed by Can_FreeIrq()
		 */
		Can_FreeIrq(minor, IRQ[minor]);
	}
	
	printk(KERN_INFO "iounmap %p \n", can_base[minor]);
	iounmap(can_base[minor]);
	iounmap(card_base[minor]);

#if !defined(PC104_OPTION_ROM)
	/* release_mem_region(Base[minor], can_range[minor]); */
	/* Release the memory region */
	printk("release mem %x \n", Base[minor]);
	release_mem_region(Base[minor], can_range[minor]);
#endif

	DBGout();
	
	return 0;
}

