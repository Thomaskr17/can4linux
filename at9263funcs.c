/* at9263funcs.c
*
* can4linux -- LINUX CAN device driver source
* 
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * 
* Copyright (c) 2007 port GmbH Halle/Saale
* (c) 2001 Heinz-Juergen Oertel (oe@port.de)
*------------------------------------------------------------------
* $Header: /z2/cvsroot/products/0530/software/can4linux/src/at9263funcs.c,v 1.1 2008/11/23 12:05:28 oe Exp $
*
*--------------------------------------------------------------------------
*
*
* modification history
* --------------------
* $Log: at9263funcs.c,v $
* Revision 1.1  2008/11/23 12:05:28  oe
* - Update and released for 3.5.3
*
* Revision 1.1  2008/10/17 15:19:24  oe
* Initial revision
*
*
*/
#include "linux/delay.h"
#include <linux/sched.h>
#include <asm/arch/hardware.h>
#include <asm/arch/at91_pmc.h>
#include <asm/arch/at91_pio.h>

#include "defs.h"
#include "at9263.h"
#include <asm/arch/gpio.h>


static enum can_state state;
static enum can_state oldstate;

/* timing values */
static const BTR_TAB_AT91_T can_btr_tab_at91[] = {
#  ifdef CAN_BTR_10K
					{  10, CAN_BTR_10K   },
#  endif					
					{  20, CAN_BTR_20K   },
					{  50, CAN_BTR_50K   },
					{ 100, CAN_BTR_100K  },
					{ 125, CAN_BTR_125K  },
					{ 250, CAN_BTR_250K  },
					{ 500, CAN_BTR_500K  },
					{ 800, CAN_BTR_800K  },
					{1000, CAN_BTR_1000K },
					{0, 0}  /* last entry */
				      };

#ifdef CAN_INDEXED_PORT_IO
canregs_t* regbase = 0;
#endif


#ifdef DEBUG
int CAN_ShowStat (int board)
{
    if (dbgMask && (dbgMask & DBG_DATA)) {
#if 0
    printk(" MODE 0x%x,", CANin(board, canmode));
    printk(" STAT 0x%x,", CANin(board, canstat));
    printk(" IRQE 0x%x,", CANin(board, canirq_enable));
    printk(" INT 0x%x\n", CANin(board, canirq));
#endif
    printk("\n");
    }
    return 0;
}
#endif

/* can_GetStat - read back as many status information as possible 
*
* Because the CAN protocol itself describes different kind of information
* already, and the driver has some generic information
* (e.g about it's buffers)
* we can define a more or less hardware independent format.
*
*
*/

int can_GetStat(
	struct inode *inode,
	struct file *file,
	CanStatusPar_t *stat
	)
{
unsigned int minor = iminor(inode);	
msg_fifo_t *Fifo;
unsigned long flags;		/* processor irq flags 	*/
int rx_fifo = ((struct _instance_data *)(file->private_data))->rx_index;
u32 tmp;


    stat->type = CAN_TYPE_AT91SAM9263;

    stat->baud                = Baud[minor];
    stat->error_warning_limit = 96; /* fix in the AT91SAM9263 */
    tmp                       = CANinl(minor, ecr);
    stat->rx_errors           = (tmp & 0xff);
    stat->tx_errors           = (tmp & 0x00ff0000ul) >> 16;
    stat->status              = CANinl(minor, sr);
    stat->error_code          = stat->status; 

    /* Disable CAN (All !!) Interrupts */
    /* !!!!!!!!!!!!!!!!!!!!! */
    /* save_flags(flags); cli(); */
    local_irq_save(flags);

    Fifo = &Rx_Buf[minor][rx_fifo];
    stat->rx_buffer_size = MAX_BUFSIZE;	/**< size of rx buffer  */
    /* number of messages */
    stat->rx_buffer_used =
    	(Fifo->head < Fifo->tail)
    	? (MAX_BUFSIZE - Fifo->tail + Fifo->head) : (Fifo->head - Fifo->tail);
    Fifo = &Tx_Buf[minor];
    stat->tx_buffer_size = MAX_BUFSIZE;	/* size of tx buffer  */
    /* number of messages */
    stat->tx_buffer_used = 
    	(Fifo->head < Fifo->tail)
    	? (MAX_BUFSIZE - Fifo->tail + Fifo->head) : (Fifo->head - Fifo->tail);
    /* Enable CAN Interrupts */
    /* !!!!!!!!!!!!!!!!!!!!! */
    /* restore_flags(flags); */
    local_irq_restore(flags);
    return 0;
}

int CAN_ChipReset(int minor)
{
/* u8	status; */
u32	pcsr;			/* peripheral clock status */
int	i;			/* counter */

    DBGin();
    /* printk(" ------- RESET ----------------------- \n"); */

    /*
     * After power-up reset, the CAN controller is disabled.
     * The CAN controller clock must be activated
     * by the Power Management Controller (PMC)
     * and the CAN controller interrupt line must be enabled
     * by the interrupt controller (AIC).
     * For both a peripheral identifier is used in case of CAN
     * this is AT91SAM9263_ID_CAN (Table 10-1) 
     *

     The user can individually enable and disable the Master Clock
     on the peripherals by writing into the Peripheral Clock Enable
     (PMC_PCER). The bit number within the Peripheral Clock Control registers
     (PMC_PCER, PMC_PCDR, and PMC_PCSR) is the Peripheral Identifier
     defined at the product level.
     Generally, the bit number corresponds to the interrupt source number
     assigned to the peripheral.
     We assum that the system clock etc. are all set,
     Linux is running already
     */
#if 1
  /* clock settings in at91sam9263.c */
    pcsr = at91_sys_read(AT91_PMC_PCSR);
    pcsr |= 1 << AT91SAM9263_ID_CAN; 
    at91_sys_write(AT91_PMC_PCER, pcsr);
#endif

    /* 
     * In case CAN is not in Reset mode, reset it
     */
    CANoutl(minor, mode, CANBIT_MR_CANDIS);
    udelay(10);

    /*
     * The pins used for interfacing the CAN
     * may be multiplexed with the PIO lines (chapter 30).
     * The programmer must first program the PIO controller
     * to assign the desired CAN pins to their peripheral function.

	When a pin is multiplexed with one or two peripheral
	functions, the selection is controlled with the registers
	PIO_PER (PIO Enable Register) and PIO_PDR (PIO Disable
	Register).
	The register PIO_PSR (PIO Status Register) is the result
	of the set and clear registers and indicates whether the
	pin is controlled by the corresponding peripheral or by
	the PIO Controller.
	A value of 0 indicates that the pin is controlled by the
	corresponding on-chip peripheral selected in the PIO_ABSR
	(AB Select Status Register). A value of 1 indicates the
	pin is controlled by the PIO controller.

	PIO pin programming/assignment is done in
	./arch/arm/mach-at91rm9200/at91sam9263_devices.c
	function at91_add_device_can()
     */

#if 1
    at91_set_A_periph(AT91_PIN_PA13,   0);	/* CANTX */
    at91_set_A_periph(AT91_PIN_PA14,   0);	/* CANRX */
#endif

    /* disable all IRQ's of all channels */
    CANoutl(minor, idr, 0xFFFFFFFFul);

    #if 0
    DBGprint(DBG_DATA, ("[%d] CAN_mode 0x%x\n", minor, CANinl(minor, mode)));
    DBGprint(DBG_DATA, ("[%d] CAN_stat 0x%x\n", minor, CANinl(minor, sr)));
    DBGprint(DBG_DATA, ("[%d] CAN_br   0x%x\n", minor, CANinl(minor, br)));
    #endif

    CAN_SetTiming(minor, Baud[minor]);
    DBGprint(DBG_DATA, ("[%d] CAN_br   0x%x\n", minor, CANinl(minor, br)));

    /* CAN_register_dump(minor); */
    /* CAN_object_dump(minor, TRANSMIT_OBJ); */
    /* CAN_object_dump(minor, RECEIVE_STD_OBJ); */

    /* initialize all Mailboxes */
    for(i = 0; i <= CAN_LAST_OBJ; i++) {
	/* disable all */
	CAN_OBJ[i].mmr = CANBIT_MMR_MOT_DISABLE;
	/* receive only the Message equal the Identifier Register */
	CAN_OBJ[i].mam = 0x7FFul << 18;
    }

    /* enable CAN controller and time stamping */
    CANoutl(minor, mode, CANBIT_MR_CANEN | CANBIT_MR_TEOF);

#if 0
    /* busy waiting for CAN ready */
    for(i = 0; i < 10000; i++) {
	if(0 != (CANinl(minor, sr) & CANBIT_SR_WAKEUP)) break;
    }
    printk("==> wake up cnt %d %x\n", i, i);
    /* FIXME: use better return value ? */
    if (i == 10000) return -ENXIO; /* Configuration mode not left */
#endif

    /* Create some CAN mail boxes */
    /* ---- TRANSMIT_OBJ --------------- */
    /* Mailbox Mode register */
    /* set MOT Mailbox Object type to TX */
    CAN_OBJ[TRANSMIT_OBJ].mmr = CANBIT_MMR_MOT_TX;
    /* Mailbox ID register */
    /* CAN_WRITE_OID(TRANSMIT_OBJ, 0x123); */
    /* CAN_OBJ[TRANSMIT_OBJ].mcr = (8 << CANBIT_MCR_MDLC_POS) | CANBIT_MCR_MTCR; */
    CAN_OBJ[TRANSMIT_OBJ].mcr = 0x00400000;

    /* CAN_register_dump(minor); */
    /* CAN_object_dump(minor, TRANSMIT_OBJ); */

    /* ---- RECEIVE_STD_OBJ ------------- */
    /* set MOT Mailbox Object type to RX */
    /* receive all Messages
     * If a bit in the Message Acceptance Register is zero
       than it always psses the acceptance test, it is not compared 
       with the bit in the Message ID Register
     */
    CAN_OBJ[RECEIVE_STD_OBJ].mam = 0;
    CAN_OBJ[RECEIVE_STD_OBJ].mmr = CANBIT_MMR_MOT_RX;
    CAN_OBJ[RECEIVE_STD_OBJ].mcr &= ~CANBIT_MCR_MTCR;

    /* ---- RECEIVE_EXT_OBJ ------------- */
    /* ---- RECEIVE_RTR_OBJ ------------- */
    /* AT the moment it seems working using only one CAN mailbox
     * for all kind of message to be received. 
     * No need for these additional receive objects
     */  

    DBGout();
    return 0;
}


/*
 * Configures bit timing registers directly. Chip must be in configuration mode.
 * The CAN Baud Reta register is 32 bit width.
 * in the can4linux we use btr0 for
 *       bit       24 containing the SMP bit 
 *       bits 23 - 16 containing the BRP Baudrate Prescaler value
 * and btr1 for
 *       bits 15 -  0 containing the fields SJW,PROPAG, PHASE1 and PHASE2
 *   according to the AT91SAM9263 manual
 *
 */
int CAN_SetBTR (int minor, int btr0, int btr1)
{
unsigned long br;

    DBGin();
    DBGprint(DBG_DATA, ("[%d] btr0=%d, btr1=%d", minor, btr0, btr1));
    br = (btr1 & 0xFFFF) + ((btr0 & 0x01FF) << 16);
    /* write timings to CAN controller */
    CANoutl(minor, br, br);
    DBGprint(DBG_DATA,("CAN_BR 0x%8x", CANinl(minor, br)));
    DBGout();
    return 0;
}

/*
 * Configures bit timing of selected CAN channel.
 * Chip must be in configuration state.
 */
int CAN_SetTiming (int minor, int baud)
{
BTR_TAB_AT91_T * table = (BTR_TAB_AT91_T*)can_btr_tab_at91;

    DBGin();
    DBGprint(DBG_DATA, ("baud[%d]=%d", minor, baud));

    /* search for data from table */
    while(1) {
        if (table->rate == 0 || table->rate == baud) {
    	    break;
    	}
    	table++;
    }
    if (table->rate == 0) {
	/* try to use baud  as custom specific bit rate
	 * not implemented yet
	 */
	return -ENXIO;
    }

    /* disable CAN Controller, if it is not disabled */
    CANoutl(minor, mode, CANBIT_MR_CANDIS);

    /* write timings to CAN controller */
    CANoutl(minor, br, (unsigned long)table->btr);
    
    DBGprint(DBG_DATA,("CAN Baudrate register = 0x%x\n", CANinl(minor, br)));

    /*
     * Stay in configuration mode; a call to CAN_StartCHip() is necessary to
     * activate the CAN controller with the new bit rate
     */
    DBGout();
    return 0;
}


int CAN_StartChip (int minor)
{
#if 0
int i;
#endif

    DBGin();
    RxErr[minor] = TxErr[minor] = 0L;
    DBGprint(DBG_DATA, ("[%d] CAN_mode 0x%x\n", minor, CANinl(minor, mode)));

    /* clear _All_  tx and rx interrupts */
    /* no idea how this could be done on AT91SAM9263 */ 


    udelay(10);

    /* Interrupts on Rx, TX, any Status change and data overrun */
    /* CAN Interrupt Enable Register
     * anclinux does not care about specific protocol errors 
     * like bit error, crc error, bit stuffing error, etc 
     */
    CANoutl(minor, ier, 
	    (1 << (TRANSMIT_OBJ))
	+   (1 << (RECEIVE_STD_OBJ))
	/* +   (1 << (RECEIVE_EXT_OBJ) ) */
	+    CANBIT_IR_BOFF
	+    CANBIT_IR_ERRP
	/* +    CANBIT_IR_ERRA */
    );



    /* Now: leave configuration mode */
    /* enable CAN controller and time stamping */
    CANoutl(minor, mode, CANBIT_MR_CANEN | CANBIT_MR_TEOF);

#if 0
    /* Wake-up of CAN was checked already in CAN_ChipReset 
     * we dont need it here again 
     */
    /* busy waiting for CAN ready */
    for(i = 0; i < 10000; i++) {
	if(0 != (CANinl(minor, sr) & CANBIT_SR_WAKEUP)) break;
    }
    printk("==> wake up cnt %d %x\n", i, i);
    /* FIXME: use better return value ? */
    if (i == 10000) return -ENXIO; /* Configuration mode not left */

#endif

    state = oldstate = active;

    /* printk(" int mask = %x\n", CANinl(minor, imr) ); */
    DBGprint(DBG_DATA, ("[%d] Int Mask 0x%x\n", minor, CANinl(minor, imr)));

    DBGout();
    return 0;
}


int CAN_StopChip(int minor)
{
    DBGin();
    CANresetl(minor, mode, CANBIT_MR_CANEN);
    /* Disable Interrupts */
    CANoutl(minor, idr, 0xFFFFFFFFul);
    /* CAN_StartChip will enable again */
    DBGout();
    return 0;
}

/* set value of the output control register
 * The register is not available, nothing happens here 
 * besides printing some debug information
 *
 * The AT91SAM9263 has no such register
 */
int CAN_SetOMode (int minor, int arg)
{

    DBGin();
    DBGprint(DBG_DATA,("[%d] outc=0x%x", minor, arg));
    DBGout();
    return 0;
}

/*
Listen-Only Mode
In listen-only mode, the CAN module is able to receive messages
without giving an acknowledgment.
Since the module does not influence the CAN bus in this mode
the host device is capable of functioning like a monitor
or for automatic bit-rate detection.

 must be done after CMD_START (CAN_StopChip)
 and before CMD_START (CAN_StartChip)
*/
int CAN_SetListenOnlyMode (int minor, int arg)
{
    DBGin();
    if (arg) {
	CANsetl(minor, mode, CANBIT_MR_ABM);
    } else {
	CANresetl(minor, mode, CANBIT_MR_ABM);
    }

    DBGout();
    return 0;
}

/*
 * Not implemented yet,
 * the driver has no filter and receives all messages on the bus
 */
int CAN_SetMask (int minor, unsigned int code, unsigned int mask)
{

    DBGin();
    DBGprint(DBG_DATA,("CAN_SetMask ! Not yet implemented !"));
    DBGprint(DBG_DATA,("[%d] acc=0x%x mask=0x%x",  minor, code, mask));
    /* put values back in global variables for sysctl */
    /* AccCode[minor] = code; */
    /* AccMask[minor] = mask; */
    DBGout();
    return 0;
}


/* 
Single CAN frames or the very first Message are copied into the CAN controller
using this function. After that an transmission request is set in the
CAN controllers command register.
After a succesful transmission, an interrupt will be generated,
which will be handled in the CAN ISR CAN_Interrupt()
*/
int CAN_SendMessage (int minor, canmsg_t *tx)
{
int i = 0;
u32 stat;

    DBGin();
    /*
    Wait for the transmitter to be idle
    At the Moment I don't know how to achive that.
    It might not be necessarry anyway.
    The CAN Status Register CAN_SR signals in bit TBSY:
    	0 - CAN transmitter is not transmitting a frame
    	1 - CAN transmitter is transmitting a frame
    */
    while ( ! ((stat = CANinl(minor, sr))
  	& CANBIT_SR_TBSY)) {
	    #if LINUX_VERSION_CODE >= 131587
	    # if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
	    cond_resched();
	    # else
	    if( current->need_resched ) schedule();
	    # endif
	    #else
	    if( need_resched ) schedule();
	    #endif
    }

    DBGprint(DBG_DATA,(
    		"CAN[%d]: tx.flags=%d tx.id=0x%lx tx.length=%d stat=0x%x",
		minor, tx->flags, tx->id, tx->length, stat));

    tx->length %= 9;			/* limit CAN message length to 8 */

    /*
      To prevent concurrent access with the internal CAN core,
      the application must disable the mailbox before writing to
      CAN_MIDx registers.
     */
    CAN_OBJ[TRANSMIT_OBJ].mmr = 0;

    /* fill the frame info and identifier fields , ID-Low and ID-High */
    if(tx->flags & MSG_EXT) {
    	/* use ID in extended message format */
	DBGprint(DBG_DATA, ("---> send ext message \n"));
	CAN_WRITE_XOID(TRANSMIT_OBJ, tx->id);
    } else {
	DBGprint(DBG_DATA, ("---> send std message \n"));
	CAN_WRITE_OID(TRANSMIT_OBJ, tx->id);
    }

    /* - fill data ---------------------------------------------------- */
    /* FIXME:
       Don't take care of message length for now */

    CAN_OBJ[TRANSMIT_OBJ].mdl = 
		   tx->data[0]
		+ (tx->data[1] <<  8)
		+ (tx->data[2] << 16)
		+ (tx->data[3] << 24);
    CAN_OBJ[TRANSMIT_OBJ].mdh = 
		   tx->data[4]
		+ (tx->data[5] <<  8)
		+ (tx->data[6] << 16)
		+ (tx->data[7] << 24);
   /* - /end --------------------------------------------------------- */

    if( tx->flags & MSG_RTR) {
    } else {
    }

    /* Enable transmit mail box */
    CAN_OBJ[TRANSMIT_OBJ].mmr = CANBIT_MMR_MOT_TX;
    /* Writing data length code,
       set transmit request 
    */
    CAN_OBJ[TRANSMIT_OBJ].mcr =
    	(tx->length << CANBIT_MCR_MDLC_POS)
    	 + ((tx->flags & MSG_RTR) ? CANBIT_MCR_MRTR : 0ul)
    	 + CANBIT_MCR_MTCR;


    if(selfreception[minor]) {
	/* prepare for next TX Interrupt and selfreception */
	memcpy(
	    (void *)&last_Tx_object[minor],
	    (void *)tx,
	    sizeof(canmsg_t));
    }

    /* CAN_object_dump(minor, TRANSMIT_OBJ); */
    DBGout();return i;
}

/* CAN_GetMessage is used in Polling Mode with ioctl()
 * !!!! curently not working for the PELICAN mode 
 * and BASIC CAN mode code already removed !!!!
 *
 * Not implemented for the AT91SAM9263
 */
int CAN_GetMessage (int minor, canmsg_t *rx )
{
    DBGin();
    DBGprint(DBG_DATA,
    	("CAN_GetMessage ! Polling for frames not implemented !"));
    DBGout();
    return 0;
}


/*
 * Perform Vendor, that means sometimes CAN controller
 * or only board manufacturer specific initialization.
 *
 * Mainly it gets needed IO and IRQ ressources and initilaizes 
 * special hardware functions.
 *
 * This code should normally be in the CAN_VendorInit() function
 * in a TERGET specific file  target.c
 */

int CAN_VendorInit (int minor)
{
    DBGin();
    /* It's Memory I/O , CAN is located somewhere, see Base[minor] */
    can_range[minor] = 0x4000;	/* 16 Kbyte */


    if(NULL == request_mem_region(Base[minor], can_range[minor], "CAN-IO")) {
	DBGprint(DBG_DATA,("Request_mem_region CAN-IO failed at %lx\n",
		Base[minor]));
	return -EBUSY;
    }
    can_base[minor] = ioremap(Base[minor], can_range[minor]);
    /* now the virtual address can be used for the register address macros */
    if( IRQ[minor] > 0 ) {
        if( Can_RequestIrq( minor, IRQ[minor] , CAN_Interrupt) ) {
	     printk(KERN_ERR "Can[%d]: Can't request IRQ %d \n",
	     			minor, 		IRQ[minor]);
	     DBGout(); return -EBUSY;
        }
    }
    DBGout(); return 0;
}


int Can_RequestIrq(int minor, int irq,
    irqreturn_t (*handler)(int, void *))
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

    err = request_irq(irq, handler, IRQF_SHARED, "Can", &Can_minors[minor]);

    if( !err ){
/* printk("Requested IRQ[%d]: %d @ 0x%x", minor, irq, handler); */
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

#if CONFIG_TIME_MEASURE
/*
  You can use all unassigned PIO which are available on expansion
  slots J24 and J25.  Check the PIO_USAGE (Schemactics 1/1), unused
  PIO are without function suffixes, for example PB11_SPI0_NPCS3 is
  used and PB12 unused.  You can use, for example, PB21 available
  on J24 pin 21.
*/

/* switch LED on */
inline void set_led(void)
{
}

/* switch LED off */
inline void reset_led(void)
{
}
#endif

/*
 * The plain AT91SAM9263 interrupt
 *
 * After completing the driver, do some measures and write results
 * down here.
 */
    	

irqreturn_t CAN_Interrupt(int irq, void *dev_id)
{
int minor;
int rx_fifo;			/* loopindex for different rx queues */
#if 0 /* debug the error registers */
int ecr;			/* error counter register 		*/
#endif
struct timeval  timestamp;
unsigned fflags = 0;		/* CAN frame flags 			*/
unsigned long iflags;		/* processor interrupt flags 		*/
msg_fifo_t   *RxFifo; 
msg_fifo_t   *TxFifo;
u32 sr;				/* status register 			*/


static int errorints = 0;

#if CONFIG_TIME_MEASURE
    /* do we have some LEDS on the EVA board ? */
    set_led();
#endif


    minor = *(int *)dev_id;
    /* printk("CAN - ISR ; minor = %d\n", *(int *)dev_id); */

    RxFifo = &Rx_Buf[minor][0]; /* predefined to use the first */
    TxFifo = &Tx_Buf[minor];




    /* beginn interrupt handling */
#define CAN_ISR_MASK (3 + 0x000C0000)   /* MB 0+1, BOFF, ERRP */
    while( (sr  = CANinl(minor, sr)) &  CAN_ISR_MASK) {
	/* loop through all interrupts until done */
	canmsg_t *rp = &RxFifo->data[RxFifo->head];

#if 0 /* for testing only */
	ecr   = CANinl(minor, ecr);
	printk("==>\nstatus     = 0x%04x + 0x%04x\n",
	    (unsigned int)(sr & 0xffff0000ul) >> 16,
	    (unsigned int)(sr & 0xFFFF));
	printk(" rxe %3d; txe %3d\n", (int)(ecr & 0xff),
	    (int)(ecr & 0x00ff0000ul) >> 16);
#endif

	/* fill timestamp as first action. 
	 * Getting a precises time takes a lot of time
	 * (additional 7 µs of ISR time )
	 * if a time stamp is not needed, it can be switched of
	 * by ioctl() */
	if (use_timestamp[minor]) {
	    do_gettimeofday(&timestamp);
	} else {
	    timestamp.tv_sec  = 0;
	    timestamp.tv_usec = 0;
	}

	/* here must be the for-loop for multiple rx queues */
	for(rx_fifo = 0; rx_fifo < CAN_MAX_OPEN; rx_fifo++) {
	    RxFifo = &Rx_Buf[minor][rx_fifo];
	    rp = &RxFifo->data[RxFifo->head];

	    RxFifo->data[RxFifo->head].timestamp = timestamp;

	    /* preset flags */
	    rp->flags = (RxFifo->status & BUF_OVERRUN ? MSG_BOVR : 0);

	    if(state == passive) { 
		rp->flags |= MSG_PASSIVE;
	    }
	}
	rx_fifo = 0;

	/* The CAN Status register indicates for each Mailbox an 
	 * Mailbox Event
	 */

	/*========== receive interrupt */
	/*=============================*/
	if(sr & (1 << RECEIVE_STD_OBJ)) {
	    int ext;
	    int extf = 0;	/* extended flag */
	    int data;
	    u32 mid;
	    unsigned int oid = 0xffffffff;	/* pre set with 'error' */

	    /* printk("RX ISR, sr=0x%08x\n", sr); */

	    /* CAN_OBJ[RECEIVE_STD_OBJ].mmr = 0; */
	    mid = CAN_OBJ[RECEIVE_STD_OBJ].mid;
	    ext = (mid & 0x20000000) ? 1 : 0;
	    /* printk(" ID  = 0x%x\n", mid); */
	    if(ext) {
		/* printk(" xID  = 0x%x\n", CAN_READ_XOID(RECEIVE_STD_OBJ)); */
		/* printk(" xID  = 0x%x\n", mid & 0x1fffffff); */
		/* oid = CAN_READ_XOID(RECEIVE_STD_OBJ); */
		oid = mid & 0x1fffffff;
		extf = MSG_EXT;
	    } else {
		/* printk(" sID  = 0x%x\n", CAN_READ_OID(RECEIVE_STD_OBJ)); */
		/* printk(" sID  = 0x%x\n", (mid & 0x03fc0000) >> 18); */
		/* oid = CAN_READ_OID(RECEIVE_STD_OBJ); */
		oid = (mid & 0x01ffc0000) >> 18;
	    }


	    /* ---------- fill frame data -------------------------------- */
	    /* handle all subscribed rx fifos */
	    for(rx_fifo = 0; rx_fifo < CAN_MAX_OPEN; rx_fifo++) {
		/* for every rx fifo */
		if (CanWaitFlag[minor][rx_fifo] == 1) {
		/* this FIFO is in use */

		    RxFifo = &Rx_Buf[minor][rx_fifo]; /* use it */
		    rp = &RxFifo->data[RxFifo->head];


		    rp->id =  oid;
		    rp->flags |= extf;
		    /* printk(" X = %d\n", ext); */
		    ext = CAN_OBJ[RECEIVE_STD_OBJ].msr;
		    rp->length = (ext & 0x000f0000) >> 16; 

		    ext = (ext & CANBIT_MSR_MRTR) ? 1 : 0;
		    /* printk(" DLC = %d %s\n", rp->length, ext ? "- RTR" : "");  */

		    if(ext) {
			rp->flags |= MSG_RTR;
		    }
		    rp->length =
			(CAN_OBJ[RECEIVE_STD_OBJ].msr & 0x000f0000) >> 16; 

		    /* Get data bytes
		       FIXME:
		       Don't take care of message length for now */
		    data = CAN_OBJ[RECEIVE_STD_OBJ].mdl;
		    rp->data[0] = (u8)data; 
		    rp->data[1] = (u8)(data >>  8); 
		    rp->data[2] = (u8)(data >> 16); 
		    rp->data[3] = (u8)(data >> 24); 
		    data = CAN_OBJ[RECEIVE_STD_OBJ].mdh;
		    rp->data[4] = (u8)data; 
		    rp->data[5] = (u8)(data >>  8); 
		    rp->data[6] = (u8)(data >> 16); 
		    rp->data[7] = (u8)(data >> 24); 

		    RxFifo->status = BUF_OK;
		    RxFifo->head = ++(RxFifo->head) % MAX_BUFSIZE;

		    if(RxFifo->head == RxFifo->tail) {
			printk(KERN_WARNING "CAN[%d] RX: FIFO overrun\n",
						minor);
			RxFifo->status = BUF_OVERRUN;
		    }

		    /*---------- kick the select() call  -*/
		    /* This function will wake up all processes
		       that are waiting on this event queue,
		       that are in interruptible sleep
		    */
		    wake_up_interruptible(&CanWait[minor][rx_fifo]); 

		    /* check for CAN controller overrun */
		}
	    }

	    /* reset interrupt condition of this object */
	    CAN_OBJ[RECEIVE_STD_OBJ].mcr = CANBIT_MCR_MTCR;

	}

	/*========== transmit interrupt */
	/*=============================*/
	if(sr & (1 << TRANSMIT_OBJ)) {
	    /* CAN frame successfully sent */
	    canmsg_t *tp = &TxFifo->data[TxFifo->tail];

	    /* use time stamp sampled with last INT */
	    last_Tx_object[minor].timestamp = timestamp;

	    /* depending on the number of open processes
	     * the TX data has to be copied in different
	     * rx fifos
	     */
	    for(rx_fifo = 0; rx_fifo < CAN_MAX_OPEN; rx_fifo++) {
		/* for every rx fifo */
		if (CanWaitFlag[minor][rx_fifo] == 1) {
		    /* this FIFO is in use */
			
		    /*
		     * Don't copy the message in the receive queue
		     * of the process that sent the message unless
		     * this process requested selfreception.
		     */
		    if ((last_Tx_object[minor].cob == rx_fifo) && 
			(selfreception[minor][rx_fifo] == 0))
		    {
			continue;
		    }

		    /* prepare buffer to be used */
		    RxFifo = &Rx_Buf[minor][rx_fifo];
		    memcpy(  
			(void *)&RxFifo->data[RxFifo->head],
			(void *)&last_Tx_object[minor],
			sizeof(canmsg_t));
		    
		    /* Mark message as 'self sent/received' */
		    RxFifo->data[RxFifo->head].flags |= MSG_SELF;

		    /* increment write index */
		    RxFifo->status = BUF_OK;
		    RxFifo->head = ++(RxFifo->head) % MAX_BUFSIZE;

		    if(RxFifo->head == RxFifo->tail) {
			printk("CAN[%d][%d] RX: FIFO overrun\n", minor, rx_fifo);
			RxFifo->status = BUF_OVERRUN;
		    } 
		    /*---------- kick the select() call  -*/
		    /* This function will wake up all processes
		       that are waiting on this event queue,
		       that are in interruptible sleep
		    */
		    wake_up_interruptible(&CanWait[minor][rx_fifo]); 
		} /* this FIFO is in use */
	    } /* end for loop filling all rx-fifos */

	    /* Reset Interrupt Flag of transmit object
	     * mailbox mode register */
	    CAN_OBJ[TRANSMIT_OBJ].mmr = 0;

	    if( TxFifo->free[TxFifo->tail] == BUF_EMPTY ) {
		/* TX FIFO empty, nothing more to sent */
		/* printk("TXE\n"); */
		TxFifo->status = BUF_EMPTY;
		TxFifo->active = 0;
		/* This function will wake up all processes
		   that are waiting on this event queue,
		   that are in interruptible sleep
		*/
		wake_up_interruptible(&CanOutWait[minor]); 
		goto Tx_done;
	    }

	    /* enter critical section */
	    local_irq_save(iflags);

	    /* The TX message FIFO contains other CAN frames to be sent
	     * The next frame in the FIFO is copied into the last_Tx_object
	     * and directly into the hardware of the CAN controller
	     */
	    /* The TX message FIFO contains other CAN frames to be sent
	     * The next frame in the FIFO is copied into the last_Tx_object
	     * and directly into the hardware of the CAN controller
	     */
	    memcpy(
		    (void *)&last_Tx_object[minor],
		    (void *)&TxFifo->data[TxFifo->tail],
		    sizeof(canmsg_t));
	    /*
	      To prevent concurrent access with the internal CAN core,
	      the application must disable the mailbox before writing to
	      CAN_MIDx registers.
	     */
	    CAN_OBJ[TRANSMIT_OBJ].mmr = 0;

	    /* fill the frame info and identifier fields , ID-Low and ID-High */
	    if(tp->flags & MSG_EXT) {
		/* use ID in extended message format */
		CAN_WRITE_XOID(TRANSMIT_OBJ, tp->id);
	    } else {
		CAN_WRITE_OID(TRANSMIT_OBJ, tp->id);
	    }

	    /* - fill data -------------------------------------------------- */
	    /* FIXME:
	       Don't take care of message length for now */

	    CAN_OBJ[TRANSMIT_OBJ].mdl = 
			   tp->data[0]
			+ (tp->data[1] <<  8)
			+ (tp->data[2] << 16)
			+ (tp->data[3] << 24);
	    CAN_OBJ[TRANSMIT_OBJ].mdh = 
			   tp->data[4]
			+ (tp->data[5] <<  8)
			+ (tp->data[6] << 16)
			+ (tp->data[7] << 24);
	   /* - /end -------------------------------------------------------- */

	    if( tp->flags & MSG_RTR) {
	    } else {
	    }


	    /* Enable transmit mail box */
	    CAN_OBJ[TRANSMIT_OBJ].mmr = CANBIT_MMR_MOT_TX;
	    /* Writing data length code,
	       set transmit request 
	    */
	    CAN_OBJ[TRANSMIT_OBJ].mcr =
		(tp->length << CANBIT_MCR_MDLC_POS)
		 + ((tp->flags & MSG_RTR) ? CANBIT_MCR_MRTR : 0ul)
		 + CANBIT_MCR_MTCR;

	    /* now this entry is EMPTY */
	    TxFifo->free[TxFifo->tail] = BUF_EMPTY;
	    TxFifo->tail = ++(TxFifo->tail) % MAX_BUFSIZE;

	    local_irq_restore(iflags);

	    Tx_done:
		;
	}  /* Tx ISR */

	/*=========== status interrupt */
	/*=============================*/

	/* take care of:
	 * in Error Passive State, also ERRA bit is set
	 */

	if(sr & (
		    CANBIT_SR_BOFF
		  + CANBIT_SR_ERRA
		  + CANBIT_SR_ERRP )) {

	    
	    int status_changed = 0;
	    char *m = NULL;

    /* printk(">> state = %d, status_changed %d\n", state, status_changed  ); */

	    if(sr & CANBIT_SR_BOFF) {
	    	m = "BusOFF";
	    	sr &= ~CANBIT_SR_BOFF;	/* reset error passive bit */
		/* Bus-Off: disable the Controller
		   - the user has to call Start_CAN() */
		CANoutl(minor, mode, CANBIT_MR_CANDIS);
		/* disable the IRQ.
	         * If no Busoff, we enable the IRQ again.
	         */
		CANoutl(minor, idr, CANBIT_IR_BOFF);
		errorints++;
	    	if(state != busoff); {
		    /* printk(" CAN_Status %s Interrupt(%d)\n", m, errorints); */
		    state = busoff;
		    fflags = MSG_BUSOFF;
		    status_changed = 1;
		}
		goto signal_error;
	    }

	    if(sr & CANBIT_SR_ERRP) {
		m = "ErrorPassive";
	    	sr &= ~CANBIT_SR_ERRP;	/* reset error passive bit */
	    	if(state != passive) {
		    state = passive;
		    fflags = MSG_PASSIVE;
		    status_changed = 1;
	    	}
		errorints++;
		CANoutl(minor, idr, CANBIT_IR_ERRP);
		/* printk("Imask = 0x%08x stat = 0x%08x\n", CANinl(minor, imr), CANinl(minor, sr)); */

	    } else if(sr & CANBIT_SR_ERRA) {
	    	m = "ErrorActive";
	    	sr &= ~CANBIT_SR_ERRA;	/* reset error passive bit */
	    	if(state != active) {
		    state = active;
		    fflags = 0;
		    status_changed = 1;
		    CANoutl(minor, ier, CANBIT_IR_BOFF
	                      + CANBIT_IR_ERRP
	                   /* + CANBIT_IR_ERRA */ );

	    	}
		errorints = 0;
	    }


	    if (m != NULL && status_changed) {
		/* printk(" CAN Status %s Interrupt(%d)\n", m, errorints); */
		;
	    }

	    if(errorints > 40)  { 
		/* disable all error interrupts */
	    CANoutl(minor, idr, CANBIT_IR_BOFF
	                      + CANBIT_IR_ERRP
	                      + CANBIT_IR_ERRA);
	      }
	signal_error:
	    /* Insert CAN error in rx queues */
	    if(status_changed) {
	    /* generate a pseude message with id 0xffffffff */
	    /* and put it in every rx queu where someone is waiting */

	for(rx_fifo = 0; rx_fifo < CAN_MAX_OPEN; rx_fifo++) {
	    /* for every rx fifo */
	    if (CanWaitFlag[minor][rx_fifo] == 1) {
		/* this FIFO is in use */
		RxFifo = &Rx_Buf[minor][rx_fifo]; /* prepare buffer to be used */
		(RxFifo->data[RxFifo->head]).flags = fflags; 
		(RxFifo->data[RxFifo->head]).id = CANDRIVERERROR;
        	(RxFifo->data[RxFifo->head]).length = 0;
		RxFifo->status = BUF_OK;

		/* handle fifo wrap around */
		RxFifo->head = ++(RxFifo->head) % MAX_BUFSIZE;
		if(RxFifo->head == RxFifo->tail) {
		    printk("CAN[%d][%d] RX: FIFO overrun\n", minor, rx_fifo);
		    RxFifo->status = BUF_OVERRUN;
		} 
		/* tell someone that there is a new error message */
		wake_up_interruptible(&CanWait[minor][rx_fifo]); 
		status_changed = 0;
	    } /* this FIFO is in use */
	} /* end for loop filling all rx-fifos */

	    } /* Status changed */
	} /* end status interrupt */

	break;
	/* TODO: looping through all interruts */
    } /* while interrupt available */

#if CONFIG_TIME_MEASURE
    reset_led();
#endif
    return IRQ_RETVAL(IRQ_HANDLED);
}



/* dump all global CAN registers to printk */
void CAN_register_dump(int minor)
{
/* pointer to the global registers of the CAN memory map */
volatile at9263can_t *at_can = (at9263can_t *)(can_base[minor]);
unsigned long s;

    printk("ATMEL AT91SAM9263 CAN register layout\n");

#define  printregister(name) printk(" " #name  " \t%p %0x\n", &name , name)

    printregister(CAN_Mode);
    printregister(CAN_InterruptEnableRegister);
    printregister(CAN_InterruptDisableRegister);
    printregister(CAN_InterruptMaskRegister);
    printregister(CAN_StatusRegister);
    s = CAN_StatusRegister;
    printk("  %s ",   (s & CANBIT_SR_ERRA) ? "ERROR-Actve" : "");
    printk("%s ",     (s & CANBIT_SR_ERRP) ? "ERROR-Passive" : "");
    printk("%s ",     (s & CANBIT_SR_BOFF) ? "Bus-Off" : "");
    printk("%s\n",     (s & CANBIT_SR_WARN) ? "Warning-Limit" : "");

    printregister(CAN_BaudrateRegister);
    printregister(CAN_TimerRegister);
    printregister(CAN_TimeStampRegister);
    printregister(CAN_ErrorCounterRegister);
    printregister(CAN_TransferCommandRegister);
    printregister(CAN_AbortCommandRegister);
    printk("\n");
}

/* dump the content of the selected message object (MB) to printk
 *
 * only 2byte word or short int access is allowed,
 * each register starts on a 4 byte boundary
 * Mailbox 0 starts at 0xFFC02C00
 * Each Mailbox is built from 8 word registers
 *
 * Additional information is unfortunately located on other memory
 * locations like the acceptance masks
*/

void CAN_object_dump(int minor, int object)
{
unsigned long v;	/* content value of an register */
char *m;
/* pointer to an CAN object, "Mailbox" according to the ATMEL doc */
volatile unsigned long *cp =
		(unsigned long *)(can_base[minor] + 0x200 + (0x20 * object));

    printk(KERN_INFO "CAN object %d (at %p)\n", object, cp);

    for(v = 0; v < 8; v++) {
	printk("%8lx ", *(cp + (v * 4)));
	if (v == 3) printk("| ");
    }
    printk("\n");

    v = CAN_OBJ[object].mmr;
    v &= CANBIT_MMR_MOT_MSK;
    switch(v) {
    case CANBIT_MMR_MOT_DISABLE:
    	m = "mailbox disabled";
    	break;
    case CANBIT_MMR_MOT_RX:
    	m = "reception mailbox";
    	break;
    case CANBIT_MMR_MOT_RX_OVL:
    	m = "reception mailbox with overwrite";
    	break;
    case CANBIT_MMR_MOT_TX:
    	m = "transmit mailbox";
    	break;
    case CANBIT_MMR_MOT_RX_RTR:
    	m = "consumer mailbox";
    	break;
    case CANBIT_MMR_MOT_TX_RTR:
    	m = "producer mailbox";
    	break;
    default:
	m = "reserveed bit combination in MOT";
    }
    printk(KERN_INFO  "mailbox type: %s,", m);
    v = CAN_OBJ[object].msr;
    printk(" DLC=0x%lx", (v & CANBIT_MSR_MDLC_MSK) >> 16); 

    v = CAN_OBJ[object].msr;
    printk(" RTR=0x%lx", (v & CANBIT_MSR_MRTR) >> 20); 
    printk(" RDY=0x%lx\n", (v & CANBIT_MSR_MRDY) >> 23); 
}
