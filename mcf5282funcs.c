/* can_mcf5282funcs.c  - Motorola FlexCAN functions
 *
 * can4linux -- LINUX CAN device driver source
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 *
 * Copyright (c) 2003-2005 port GmbH Halle/Saale
 * (c) 2003 Heinz-Jürgen Oertel (oe@port.de)
 *------------------------------------------------------------------
 * $Header: /z2/cvsroot/products/0530/software/can4linux/src/mcf5282funcs.c,v 1.4 2007/06/22 08:43:53 oe Exp $
 *
 *--------------------------------------------------------------------------
 *
 *
 *  The driver is simulating a so-called Basic CAN concept,
 *  thats can4linux was  designed for.
 *  There is on the users API only one chanell to send CAN frames,
 *  the write() call, and only one to receive, the read(call).
 *  FlexCAN is a Full CAN controller,
 *  providing 16 Message Buffers (MB according the doc.)
 *  Each one can be used as transmit or receive object.
 *  The driver only uses the following MBs:
 *
 *  TRAMSMIT_OBJ  - used to transmit messages, possible are:
 *  	standard, extended, standard RTR, extended RTR
 *  RECEIVE_STD_OBJ - used to receice all messages in base frame format
 *  RECEIVE_EXT_OBJ - used to receice all messages in extended frame format
 *  RECEIVE_RTR_OBJ - what be nice to have, but this doesn't work
 *        the driver is not able to receive any RTE frames.
 *
 *
 * modification history
 * --------------------
 * $Log: mcf5282funcs.c,v $
 * Revision 1.4  2007/06/22 08:43:53  oe
 * - using kernel Fifos started (not fully functionally)
 *
 * Revision 1.3  2006/09/08 09:48:12  oe
 * - removed Bug in mcf5282 when transmitting from ISR
 *
 * Revision 1.2  2006/06/02 12:22:08  oe
 * - Bug in mcf5282, early resetting interrupts
 *
 * Revision 1.1  2006/04/21 16:26:52  oe
 * - new version with new file structure for all architectures
 *
 * Revision 1.2  2005/11/08 11:31:48  oe
 * massive changes for SSV IGW/900
 *
 * Revision 1.1  2005/07/29 05:42:22  oe
 * Initial revision
 *
 *
 *
 */
#include "defs.h"
#include <asm/delay.h>
/* Motorola ColdFire Board FlexCAN */
#include <asm/coldfire.h>
#include <asm/mcfsim.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,0)
# error use the Coldfire FLexCAN only with Kernel > 2.4
#endif

/* int	CAN_Open = 0; */

/* timing values */
static const BTR_TAB_TOUCAN_T can_btr_tab_toucan[] = {
 { 10,
  CAN_PRESDIV_10K,   CAN_PROPSEG_10K,   CAN_PSEG1_10K,   CAN_PSEG2_10K  },
 { 20,
  CAN_PRESDIV_20K,   CAN_PROPSEG_20K,   CAN_PSEG1_20K,   CAN_PSEG2_20K  },
 { 50,
  CAN_PRESDIV_50K,   CAN_PROPSEG_50K,   CAN_PSEG1_50K,   CAN_PSEG2_50K  },
 { 100, 
  CAN_PRESDIV_100K,  CAN_PROPSEG_100K,  CAN_PSEG1_100K,  CAN_PSEG2_100K },
 { 125,
  CAN_PRESDIV_125K,  CAN_PROPSEG_125K,  CAN_PSEG1_125K,  CAN_PSEG2_125K },
 { 250,
  CAN_PRESDIV_250K,  CAN_PROPSEG_250K,  CAN_PSEG1_250K,  CAN_PSEG2_250K },
 { 500,
  CAN_PRESDIV_500K,  CAN_PROPSEG_500K,  CAN_PSEG1_500K,  CAN_PSEG2_500K },
 { 800,
  CAN_PRESDIV_800K,  CAN_PROPSEG_800K,  CAN_PSEG1_800K,  CAN_PSEG2_800K },
 { 1000,
  CAN_PRESDIV_1000K, CAN_PROPSEG_1000K, CAN_PSEG1_1000K, CAN_PSEG2_1000K },

 {0,
  0, 0, 0, 0}  /* last entry */
};




/* Board reset
   means the following procedure:
  set Reset Request
  wait for Rest request is changing - used to see if board is available
  initialize board (with valuse from /proc/sys/Can)
    set output control register
    set timings
    set acceptance mask
*/


#ifdef DEBUG
int CAN_ShowStat (int board)
{
    if (dbgMask && (dbgMask & DBG_DATA)) {
    printk(" MCR 0x%x,", CANinw(board, canmcr));
    printk(" ESTAT 0x%x,", CANinw(board, estat));
    printk(" IFLAGS 0x%x,", CANinw(board, iflag));
    printk(" IMASK 0x%x,", CANinw(board, imask));
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
* The FlexCAN modul provides status and error-status information
* in one 16 bit register: Error and Status Flag - ESTAT.
* Therfore this content is used twice in the returned 
* CanStatusPar_t structure.
*/

int can_GetStat(
	struct inode *inode,
	CanStatusPar_t *stat
	)
{
unsigned int board = MINOR(inode->i_rdev);
msg_fifo_t *Fifo;
unsigned long flags;


    stat->type = CAN_TYPE_FlexCAN;

    stat->baud = Baud[board];
    /* printk(" STAT ST 0x%x\n", CANin(board, canstat)); */
    stat->status = CANinw(board, estat);
    /* not available in the FlexCAN, lets fill 127 here */
    stat->error_warning_limit = 127;
    stat->rx_errors = CANin(board, rxectr);
    stat->tx_errors = CANin(board, txectr);
    /* error code is not available, use estat again */
    stat->error_code= CANinw(board, estat);

    /* Collect information about the RX and TX buffer usage */
    /* Disable CAN Interrupts */
    /* !!!!!!!!!!!!!!!!!!!!! */
    save_flags(flags); cli();
    Fifo = &Rx_Buf[board];
    stat->rx_buffer_size = MAX_BUFSIZE;	/**< size of rx buffer  */
    /* number of messages */
    stat->rx_buffer_used =
    	(Fifo->head < Fifo->tail)
    	? (MAX_BUFSIZE - Fifo->tail + Fifo->head) : (Fifo->head - Fifo->tail);
    Fifo = &Tx_Buf[board];
    stat->tx_buffer_size = MAX_BUFSIZE;	/* size of tx buffer  */
    /* number of messages */
    stat->tx_buffer_used =
    	(Fifo->head < Fifo->tail)
    	? (MAX_BUFSIZE - Fifo->tail + Fifo->head) : (Fifo->head - Fifo->tail);
    /* Enable CAN Interrupts */
    /* !!!!!!!!!!!!!!!!!!!!! */
    restore_flags(flags);
    return 0;
}



/*
 * CAN_ChipReset - performs the first initialization or re-iniatalization of the chip
 *
 *  set INIT mode
 *  initialize the I/O pin modes as CAN TX/RX
 *  initialize the CAN bit timing
 *  initialize message buffers
 *  initialize interrut sources
 */
int CAN_ChipReset (int board)
{
int i;

    DBGin("CAN_ChipReset");
    /* printk("CAN_ChipReset\n"); */
	
    /* 
     * Initialize Port AS PAR to have Can TX/RX signals enabled 
     */
#define MCF5282_GPIO_PASPAR  	(*(volatile u16 *)(void *)(0x40100056))
    MCF5282_GPIO_PASPAR = 0x0FF0;
    /* printk("I/O %p %04x\n",
           (volatile u16 *)(void *)(0x40100056), MCF5282_GPIO_PASPAR); */
	

    /*
     * go to INIT mode
     * Any configuration change/initialization requires that the FlexCAN be
     * frozen by either asserting the HALT bit in the
     * module configuration register or by reset.
     * For Init_CAN() we choose reset.
     */
    CANresetw(board, canmcr, CAN_MCR_FRZ);
    /* CANoutw(board, canmcr, 0x0080); */
    CANsetw(board, canmcr, CAN_MCR_SOFT_RST);
    udelay(10);
    /* Test Reset Status */
    if(CANtestw(board, canmcr, CAN_MCR_SOFT_RST) != 0) {
	MOD_DEC_USE_COUNT;
	DBGout();return -1;
    }

    udelay(10);
    /* Initialize the transmit and receive pin modes in control register 0 */
    CANout(board, canctrl0, 
    	  CAN_CTRL0_DISABLE_BOFF_INT	/* disable Bus-Off Interrupt	*/
    	+ CAN_CTRL0_DISABLE_ERR_INT	/* disable Error Interrupt	*/
    	+ 0				/*  4 , logic RX level		*/
    	+ 0				/*  1 , logic TX level		*/
    	);
    /* CANresetw(board, canmcr, CAN_MCR_HALT); */


    CAN_SetTiming(board, Baud[board]);

    /*
     * Select the internal arbitration mode
     * LBUF in CANCTRL1
     * LBUF Lowest Buffer Transmitted First
     * The LBUF bit defines the transmit-first scheme.
     * 0 = Message buffer with lowest ID is transmitted first.
     * 1 = Lowest numbered buffer ist transmitted first.
     *
     * should have no impact here, the driver is using only one
     * TX object
     *
     * !! change the order of the next two statements  !!
     */
     
    CANset(board, canctrl1, CAN_CTRL1_LBUF);
    CANreset(board, canctrl1, CAN_CTRL1_LBUF);

    /*
     * Initialize message buffers.
     * The control/status word of all message buffers are written
     * as an inactive receive message buffer.
     */
    for(i = 0; i < 16; i++) {
	CAN_WRITE_CTRL(i, REC_CODE_NOT_ACTIVE, 0);
    }

    /* create a transmit object, that can be used for all kinds of messages */

    /* some receive objects
     * - RECEIVE_STD_OBJ to receive all standard frames
     * - RECEIVE_EXT_OBJ to receive all extended frames
     * - ??
     */
    CAN_WRITE_CTRL(RECEIVE_STD_OBJ, REC_CODE_NOT_ACTIVE, 0);
    CAN_WRITE_OID(RECEIVE_STD_OBJ, 0);	/* set IDE - extended bit to zero */
    CAN_SetMask  (board, AccCode[board], AccMask[board] );
    CAN_WRITE_CTRL(RECEIVE_STD_OBJ, REC_CODE_EMPTY, 8);

    /* The IDE-Bit (Bit 4) must be set to 1 for extended */
    CAN_WRITE_CTRL(RECEIVE_EXT_OBJ, REC_CODE_NOT_ACTIVE, 0);
    CAN_WRITE_XOID(RECEIVE_EXT_OBJ, 0);	/* set IDE - extended bit to 1 */
    CAN_SetMask  (board, AccCode[board], AccMask[board] );
    CAN_WRITE_CTRL(RECEIVE_EXT_OBJ, REC_CODE_EMPTY, 8);

#if 0
    /* RTR Receive Object !!! No such thing possible !!! */
    CAN_WRITE_CTRL(RECEIVE_RTR_OBJ, REC_CODE_NOT_ACTIVE, 0);
    CAN_WRITE_OID(RECEIVE_RTR_OBJ, 1);
    CAN_SetMask  (board, AccCode[board], AccMask[board] );
    CAN_WRITE_CTRL(RECEIVE_RTR_OBJ, REC_CODE_EMPTY, 8);
#endif


    /* CAN_register_dump(); */
    /* CAN_object_dump(RECEIVE_STD_OBJ); */
    /* CAN_object_dump(RECEIVE_EXT_OBJ); */
    /* CAN_object_dump(RECEIVE_RTR_OBJ); */



    /*
     * - Initialize IARB[3:0] to a non zero value in CANMCR
     *   ( This is done in CAN-ChipReset() )
     * - Set the required mask bits in the IMASK register (for all message
     *  buffer interrupts) in CANCTRL0 for bus off and error interrupts,
     *  and in CANMCR for WAKE interrupt.
     */

    /* dont't forget error int's ! */
    CANsetw(board, canmcr, 1);
    CANoutw(board, imask, 
               (1 << RECEIVE_STD_OBJ)
              +(1 << RECEIVE_EXT_OBJ)
              +(1 << TRANSMIT_OBJ));

    /* printk("imask at %p \n", &((canregs_t *)can_base[board])->imask ); */

    /* CAN_register_dump(); */
    DBGout();
    return 0;
}

/*
 * Configures bit timing registers directly. Chip must be in configuration mode.
 */
int CAN_SetBTR (int board, int btr0, int btr1)
{
    DBGin("CAN_SetBTR");
    DBGprint(DBG_DATA, ("[%d] btr0=%d, btr1=%d", board, btr0, btr1));


    /* ToDO for MCF FlexCAN */

    DBGout();
    return 0;
}

/* change the bit timings of the selected CAN channel */
int CAN_SetTiming (int board, int baud)
{
/* int custom=0; */
BTR_TAB_TOUCAN_T * table = (BTR_TAB_TOUCAN_T*)can_btr_tab_toucan;

    DBGin("CAN_SetTiming");
    DBGprint(DBG_DATA, ("baud[%d]=%d", board, baud));
    /* enable changing of bit timings */
    CANsetw(board, canmcr, CAN_MCR_HALT);
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

    /*
     * Set Timing Register values.
     * Initialize the bit timing parameters PROPSEG, PSEG1, PSEG2 and RJW
     * in control registers 1 and 2.
     *
     * The FlexCAN module uses three 8-bit registers to set-up
     * the bit timing parameters required by the CAN protocol.
     * Control registers 1 and 2 contain the PROPSEG, PSEG1, PSEG2
     * and RJW fields which allow the user to configure
     * the bit timing parameters.
     * The prescaler divide register (PRESDIV) allows the user to select
     * the ratio used to derive the S-Clock from the system clock.
     * The time quanta clock operates at the S-clock frequency.
     */
    CANout(board, presdiv, table->presdiv);
    CANout(board, canctrl2, ((table->pseg1 << 3)+ table->pseg2));
    CANout(board, canctrl1, (
    	  0		/* SAMP = 0 , 0/0x80		*/	
    	+ 0		/* LOOP = 0 , 0/0x40		*/	
    	+ 0		/* TSYNC= 0 , 0/0x20		*/
    	+ 0		/* LBUF = 0 , 0/0x10		*/
    	+ 0		/* RSVD = 0 , 0/0x08		*/
    	+ table->propseg)
    	);

    /*
     * Stay in configuration mode; a call to Start-CAN() is necessary to
     * activate the CAN controller with the new bit rate
     */
    DBGprint(DBG_DATA,("canctrl2=0x%x presdiv=0x%x",
    		CANin(board, canctrl2), CANin(board, presdiv)) );

    DBGout();
    return 0;
}


int CAN_StartChip (int board)
{
/* int i; */
    RxErr[board] = TxErr[board] = 0L;
    DBGin("CAN_StartChip");
    /* printk("CAN_StartChip\n"); */


    /* clear interrupts */
    CANinw(board, iflag);
    CANoutw(board, iflag, 0xffff);	/* 5282 overwrites with '1' */
    CANoutw(board, estat, 0); 		/* 5282 overwrites ESTAT with '0' */

    CANresetw(board, canmcr, CAN_MCR_HALT | CAN_MCR_FRZ);
    /* Interrupts on Rx, TX, any Status change and data overrun */
    /* Initialize the transmit and receive pin modes in control register 0 */
    CANset(board, canctrl0,
    	  CAN_CTRL0_ENABLE_BOFF_INT	/* enable Bus-Off Interrupt	*/
    	+ CAN_CTRL0_ENABLE_ERR_INT	/* enable Error Interrupt	*/
    	+ 0				/*  4 , logic RX level		*/
    	+ 0				/*  1 , logic TX level		*/
    	);

   CANoutw(board, imask, 
               (1 << RECEIVE_STD_OBJ)
              +(1 << RECEIVE_EXT_OBJ)
              +(1 << TRANSMIT_OBJ));

    /* to do */
    /* CAN_register_dump(); */

    DBGout();
    return 0;
}


/* Disable all CAN activities */
int CAN_StopChip (int board)
{
    DBGin("CAN_StopChip");
    /* printk("CAN_StopChip\n"); */
    CANset(board, canmcr, CAN_MCR_HALT);
    /* disable all error interrupts */
    CANreset(board, canctrl0,
    	  CAN_CTRL0_ENABLE_BOFF_INT	/* disable Bus-Off Interrupt	*/
    	+ CAN_CTRL0_ENABLE_ERR_INT	/* disable Error Interrupt	*/
    	);
    /* disable all MB interrupts */
    CANoutw(board, imask, 0);
    DBGout();
    return 0;
}

/* set value of the output control register
 * The register is not available, nothing happens here 
 * besides printing some debug information
 */
int CAN_SetOMode (int board, int arg)
{

    DBGin("CAN_SetOMode");
	DBGprint(DBG_DATA,("[%d] outc=0x%x", board, arg));

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

*/
int CAN_SetListenOnlyMode (int board, int arg)
{
    DBGin("CAN_SetListenOnlyMode");
    /* has to be filled for the FlexCAN */
    /* ================================ */
    DBGout();
    return 0;
}

/* FlexCAN only knows a 'mask' value, code is ignored */
int CAN_SetMask (int board, unsigned int code, unsigned int mask)
{

    DBGin("CAN_SetMask");
    DBGprint(DBG_DATA,("[%d] acc=0x%x mask=0x%x",  board, code, mask));
    CANoutw(board, rxgmskhi, mask >> 16);
    CANoutw(board, rxgmsklo, mask && 16);

    /* put values back in global variables for sysctl */
    AccCode[minor] = code;
    AccMask[minor] = mask;

    DBGout();
    return 0;
}


int CAN_SendMessage (int board, canmsg_t *tx)
{
int i = 0;
volatile u16 stat;

    DBGin("CAN_SendMessage");

    /*
    IDLE - Idle Status. The IDLE bit indicates, when there is activity
    on the CAN bus
    1 - The CAN bus is Idle

    TX/RX - Transmission/receive status.
    Indicates when the FlexCAN module is transmitting or receiving a message.
    TX/RX has no meaning, when IDLE = 1
	0 - FlexCAN is receiving when IDLE = 0
	1 - FlexCAN is transmitting when IDLE = 0
    */

    while (
    	    ((stat = CANinw(board, estat)) & (CAN_ESTAT_IDLE + CAN_ESTAT_TX_RX))
    	    == CAN_ESTAT_TX_RX
    	  ) {

	if (current->need_resched) schedule();
	/* if( need_resched ) schedule(); */

    }

    /* DBGprint(DBG_DATA,( */
    		/* "CAN[%d]: tx.flags=%d tx.id=0x%lx tx.length=%d stat=0x%x", */
		/* board, tx->flags, tx->id, tx->length, stat)); */

    tx->length %= 9;			/* limit CAN message length to 8 */


    /* Writing Control/Status word to hold TX Message Object inactive */
    CAN_WRITE_CTRL(TRANSMIT_OBJ, TRANS_CODE_NOT_READY, 1);

    /* fill the frame info and identifier fields , ID-Low and ID-High */
    if(tx->flags & MSG_EXT) {
    	/* use ID in extended message format */
	DBGprint(DBG_DATA, ("---> send ext message \n"));
	if( tx->flags & MSG_RTR) {
	    CAN_WRITE_XOID_RTR(TRANSMIT_OBJ, tx->id);
	} else {
	    CAN_WRITE_XOID(TRANSMIT_OBJ, tx->id);
	}
    } else {
	DBGprint(DBG_DATA, ("---> send std message \n"));
	if( tx->flags & MSG_RTR) {
	    CAN_WRITE_OID_RTR(TRANSMIT_OBJ, tx->id);
	} else {
	    CAN_WRITE_OID(TRANSMIT_OBJ, tx->id);
	}
    }

    /* - fill data ---------------------------------------------------- */

    for(i = 0; i < tx->length; i++) {
    	CAN_OBJ[TRANSMIT_OBJ].msg[i] = tx->data[i];
    }
    /* Writing Control/Status word (active code, length) */
    CAN_WRITE_CTRL(TRANSMIT_OBJ, TRANS_CODE_TRANSMIT_ONCE, tx->length);

    /* - /end --------------------------------------------------------- */
    DBGout();return i;
}


/* look if one of the receive message objects has something received */
int CAN_GetMessage (int board, canmsg_t *rx )
{
volatile unsigned int stat;
volatile unsigned int ctrl;
int i = 0;

    DBGin("CAN_GetMessage");
    stat = CANinw(board, estat);
    DBGprint(DBG_DATA,("0x%x: stat=0x%x iflag=0x%x imask=0x%x" ,
    			Base[board], stat,
    			CANinw(board, iflag),
    			CANinw(board, imask)));

    rx->flags  = 0;
    rx->length = 0;

    /* CAN_register_dump(); */
    /* CAN_object_dump(RECEIVE_STD_OBJ); */
    i = CANinw(board, iflag);
    if( i & (1 << RECEIVE_STD_OBJ)) {
	/* reading the control status word of the identified message
	 * buffer triggers a lock for that buffer.
	 * A new received message frame which maches the message buffer
	 * can not be written into this buffer while it is locked
	 */
        while (((ctrl = CAN_READ_CTRL(RECEIVE_STD_OBJ)) & (REC_CODE_BUSY << 4)) == (REC_CODE_BUSY << 4)) {
	    /* 20 cycles maximum wait */
	    /* printf1("CAN_int, rx REC_CODE_BUSY"); */
	}
	/* printk("message received 0x%x\n", ctrl); */
	rx->length = ctrl & 0x0f;
	rx->id =  CAN_READ_OID(RECEIVE_STD_OBJ);
	memcpy((void *)&rx->data[0],
	       (const void *)&(CAN_OBJ[RECEIVE_STD_OBJ].MSG0), 8);

	/* clear interrupts */
	/* Int is cleared when the CPU reads the intrerupt flag register iflag
	 * while the associated bit is set, and then writes it back as '1'
	 * (and no new event of the same type occurs
	 * between the read and write action.)
	 * ! This is opposit to the TouCAN module, where the iflag bit
	 * has to be written back with '0'
	 */
	CANsetw(board, iflag, (1 << RECEIVE_STD_OBJ));
	/* Reset message object */
	CAN_WRITE_CTRL(RECEIVE_STD_OBJ, REC_CODE_EMPTY, 8);
	/* reading the free running timer will unlock any message buffers */
	(void) CANinw(board, timer);
	i = 1; /* signal one received message */
    } else {
	i = 0;
    }

    DBGout();
    return i;
}


int CAN_VendorInit (int minor)
{
    DBGin("CAN_VendorInit");
/* 1. Vendor specific part ------------------------------------------------ */
    can_range[minor] = 0x180;

/* End: 1. Vendor specific part ------------------------------------------- */

    /* Request the controllers address space */

    /* looks like not needed in uClinux with internal ressources ? */
    /* It's Memory I/O */
    if(NULL == request_mem_region(Base[minor], can_range[minor], "CAN-IO")) {
	return -EBUSY;
    }

    /* not necessary in uClinux, but ... */
    can_base[minor] = ioremap(Base[minor], can_range[minor]);
    /* now the virtual address can be used for the register address macros */

/* 2. Vendor specific part ------------------------------------------------ */



/* End: 2. Vendor specific part ------------------------------------------- */

    if( IRQ[minor] > 0 ) {
        if( Can_RequestIrq( minor, IRQ[minor] , CAN_Interrupt) ) {
	     printk("Can[%d]: Can't request IRQ %d \n", minor, IRQ[minor]);
	     DBGout(); return -EBUSY;
        }
    }
    DBGout(); return 1;

}



/* enable the interrupts at the interrupt controller
 used:
  - receivebuffer 1

*/
void mcf_irqsetup(void)
{
volatile unsigned char *icrp;	/* interrupt control register pointer */
volatile unsigned long *imrp;	/* interrupt mask register pointer    */

    /* Initialize FlexCAN interrupt handler
      - Initialize the interrupt configurataion register (CANICR with
        a specific request level and vector base address
      - Initialize IARB[3:0] to a non zero value in CANMCR
        ( This is done in CAN-ChipReset() )
      - Set the required mask bits in the IMASK register (for all message
        buffer interrupts) in CANCTRL0 for bus off and error interrupts,
        and in CANMCR for WAKE interrupt.
        (happens in CAN-ChipReset() too)

	MCF5282 has two interrupt controllers, FlexCAN interrupts are
	scheduled to INTC1 (the second one, with lower priority)
	Int source nr  -- Source --                    vector
	 8             -- message buffer 0             136
	 9             -- message buffer 1             137
	   ...
	23             -- message buffer 15            151
	24             -- Error Int                    152
	25             -- Bus-Off Int                  153
	26             -- Wake-Up Int                  154


       Interrupt Vector number is vector_number = 128 + interrupt source number
       (for INTC1)                    
       vector_number = 137 (std receive object == MB 1) 


       The Interrupt Priority Within a Level is set at IP ICR[2:0]
       The Interrupt Level                   is set at IL ICR[5:3]


     */
	/* TRANSMIT_OBJ 0 */
    icrp = (volatile unsigned char *) (MCF_MBAR + MCFICM_INTC1 + 
	    MCFINTC_ICR0 + MCFINT_CAN_BUF00 );

    *icrp = (6 << 3) + 2; /* CANx with level 6, priority 3 */
    /* printk("icrp %p = 0x%08x\n", icrp, *icrp); */

	/* RECEIVE_STD_OBJ 1 */
    icrp = (volatile unsigned char *) (MCF_MBAR + MCFICM_INTC1 + 
	    MCFINTC_ICR0 + MCFINT_CAN_BUF01 );
    *icrp = (6 << 3) + 4; /* CANx with level 6, priority 3 */
    /* printk("icrp %p = 0x%08x\n", icrp, *icrp); */

	/* RECEIVE_EXT_OBJ 2 */
    icrp = (volatile unsigned char *) (MCF_MBAR + MCFICM_INTC1 + 
	    MCFINTC_ICR0 + MCFINT_CAN_BUF02 );
    *icrp = (6 << 3) + 3; /* CANx with level 6, priority 3 */
    /* printk("icrp %p = 0x%08x\n", icrp, *icrp); */
    
    icrp = (volatile unsigned char *) (MCF_MBAR + MCFICM_INTC1 + 
	    MCFINTC_ICR0 + MCFINT_CAN_WARN );
    *icrp = (6 << 3) + 1; /* CANx with level 6, priority 3 */
    /* printk("icrp %p = 0x%08x\n", icrp, *icrp); */

    icrp = (volatile unsigned char *) (MCF_MBAR + MCFICM_INTC1 + 
	    MCFINTC_ICR0 + MCFINT_CAN_BUSOFF );
    *icrp = (6 << 3) + 0; /* CANx with level 6, priority 3 */
    /* printk("icrp %p = 0x%08x\n", icrp, *icrp); */

    /* set Mask register too
    The IMRHn and IMRLn registers are each 32 bits in size and provide a
    bit map for each interrupt to allow the request to be disabled (1 =
    disable the request, 0 = enable the request).  The IMRn is set to all
    ones by reset, disabling all interrupt requests. The IMRn can be read
    and written. A write that sets bit 0 of the IMR forces the other 63
    bits to be set, disabling all interrupt sources, and providing a global
    mask-all capability.
    */
    imrp = (volatile unsigned long *) (MCF_MBAR + MCFICM_INTC1 +
		MCFINTC_IMRL);
    *imrp &= ~(   (1 << (MCFINT_CAN_BUF00 ))
    	        | (1 << (MCFINT_CAN_BUF01 ))
    	        | (1 << (MCFINT_CAN_BUF02 ))
    	        | (1 << (MCFINT_CAN_WARN ))
    	        | 1);
    /*            ^ unmask all */

    /* printk("imrp %p = 0x%08x\n", imrp, *imrp); */
}

void mcf_irqreset(void)
{
volatile unsigned long *imrp;	/* interrupt mask register pointer    */

    /* Mask register auch rücksetzen
    The IMRHn and IMRLn registers are each 32 bits in size and provide a
    bit map for each interrupt to allow the request to be disabled (1 =
    disable the request, 0 = enable the request).  The IMRn is set to all
    ones by reset, disabling all interrupt requests. The IMRn can be read
    and written. A write that sets bit 0 of the IMR forces the other 63
    bits to be set, disabling all interrupt sources, and providing a global
    mask-all capability.
    */
    imrp = (volatile unsigned long *) (MCF_MBAR + MCFICM_INTC1 +
		MCFINTC_IMRL);
    *imrp |= ((1 << (MCFINT_CAN_BUF00 ))
    	    | (1 << (MCFINT_CAN_BUF01 )) 
    	    | (1 << (MCFINT_CAN_BUF02 )) 
    	    | (1 << (MCFINT_CAN_WARN ))
            | 1);

}


int Can_RequestIrq(int minor, int irq,
    irqreturn_t (*handler)(int, void *, struct pt_regs *))
{
int err=0;

    DBGin("Can_RequestIrq");
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
       void *, it can point to anything (such  as  a  device-spe­
       cific  structure,  or even empty space), but make sure you
       pass the same pointer to free_irq().

    */

    {
	int i;
	/* 19 Int vectors are used on Interrupt Controller 1 */
	for( i = 136; i < 155; i++) {
	    err = request_irq( i, handler, SA_INTERRUPT, "Can", &Can_minors[minor]);
	    if(err) {
    		DBGout();return err;
	    }
	}
    }
    if( !err ){
	/* printk("Requested IRQ[%d]: %d @ 0x%x", minor, irq, handler); */

/* Now the kernel has assigned a service to the Interruptvector,
   time to enable the hardware to generate an ISR.

   here should be used a generic function e.g. can_irqsetup(minor)
   and do whatever needed for the app. hardware
   to reduce #ifdef clutter
   */
	mcf_irqsetup();

	/* irq2minormap[irq] = minor; */

	/* irq2pidmap[irq] = current->pid; */
	DBGprint(DBG_BRANCH,("Requested IRQ: %d @ 0x%lx",
				irq, (unsigned long)handler));
	IRQ_requested[minor] = 1;
    }
    DBGout();return err;
}

int Can_FreeIrq(int minor, int irq )
{
    DBGin("Can_FreeIrq");
    IRQ_requested[minor] = 0;

    /* reset interrupt masks */
    mcf_irqreset();
    {
    int i;
    	/* 19 Int vectors are used on Interrupt Controller 1 */
	for(i = 136; i < 155; i++) {
	    free_irq(i, &Can_minors[minor]);
	}
    }
    DBGout();
    return 0;
}

#define MCFBAR  0x40000000

#define PORTQA   (MCFBAR + 0x00190006)	/* 8 bit */
#define PORTQB   (MCFBAR + 0x00190007)	/* 8 bit */
#define DDRQA    (MCFBAR + 0x00190008)	/* 8 bit */
#define DDRQB    (MCFBAR + 0x00190009)	/* 8 bit */

/* switch LED on */
inline void set_led(void)
{
#if defined(CONFIG_TIME_MEASURE) && defined(CONFIG_DNP5280)
    
    *(volatile u8 *) DDRQA = 0x10;
    *(volatile u8 *) DDRQB = 0x03;

    *(volatile u8 *) PORTQA = 0x10;
    *(volatile u8 *) PORTQB = 0x06;
#endif
}

/* switch LED off */
inline void reset_led(void)
{
#if defined(CONFIG_TIME_MEASURE) && defined(CONFIG_DNP5280)
    *(volatile u8 *) PORTQA = 0x00;
    *(volatile u8 *) PORTQB = 0x00;
#endif
}



/*
 * The plain CAN interrupt
 *
 *				RX ISR           TX ISR
 *                              8/0 byte
 *                               _____            _   ___
 *                         _____|     |____   ___| |_|   |__
 *---------------------------------------------------------------------------
 * 1) Motorola ColdFire 5282
 *  63,4 MHz, 42,29 bogomips
 *                              __/__µs            __ µs
 *    Freescale ColdFire 548x
 *  100 MHz, xxxxx bogomips
 *                              __/__µs            __ µs
 *
 * 1) 1Byte = (42-27)/8 =      µs
 * 2) 1Byte = (24-12)/8 =      µs
 *
 *
 *
 * RX Int with to Receive channels:
 * 1)                _____   ___
 *             _____|     |_|   |__
 *                   30    5  20  µs
 *   first ISR normal length,
 *   time between the two ISR -- short
 *   sec. ISR shorter than first, why? it's the same message
 */

irqreturn_t CAN_Interrupt ( int irq, void *dev_id, struct pt_regs *ptregs )
{
volatile unsigned int		estat;
volatile unsigned int		ctrl;
volatile unsigned int 		irqsrc;
static int erroractive = 1;
int		dummy;
unsigned long	flags;
int		board;
msg_fifo_t	*RxFifo;
msg_fifo_t	*TxFifo;
int 		i;
static int	last_tx_id;	/* should be global, -1, set bei transmit  */
				/* and should be an array for each channel */

int full = 0;

    set_led();

    board = *(int *)dev_id;

    RxFifo = &Rx_Buf[board];
    TxFifo = &Tx_Buf[board];

    /* DBGprint(DBG_DATA, (" => got  IRQ[%d]: 0x%0x\n", board, irqsrc)); */


    /* One of the following can actually occur:
    IntNr Reason
     153   BusOff				
     152   ErrInt - in this case:
	    bits 4-5 - ESTAT.FCS - Fault Confinment State tells us
	     00 - all ok, error active mode
	     01 - error passive mode
	    bit 8 - RX warning level reached
	    bit 9 - Tx warning level reached
     154   WakeUp Interrupt

       At the moment we aren't interested in more information

       all bits in estat are read only except for
       BOFF_INT WAKE_INT und ERR_INT which are interrupt sources
       and can be written by the host to '0' to reset Interrupt
       conditions.
    */
    estat = CANinw(board, estat);
    if( estat
        & (CAN_ESTAT_BOFF_INT | CAN_ESTAT_ERR_INT | CAN_ESTAT_WAKE_INT))  {
	CANoutw(board, estat, 0);
	CANoutw(board, estat, 0);
	CANoutw(board, estat, 0xffff);
    }

    /* error interrupts are :
       if (irq == 152 || irq == 153 || irq == 154)
       we could test the irq or read out the estat register
     */
/* printk("1"); */
    if (estat &
       (CAN_ESTAT_BOFF_INT | CAN_ESTAT_ERR_INT | CAN_ESTAT_WAKE_INT))  {
	/* we have an error interrupt
	 * later on we can move this error handling at the end of the ISR
	 * to have better RX response times */
            /* 1111 */
	/* printk(" error-1 status 0x%04x \n", estat); */
	/* printk(" 0x%04x\n", estat); */
	/* reset all error conditions, we have saved them in estat
	 * BusOff disables the Interrupt generation itself by resetting
	 * the mask in canctrl0.
	 * ErrInt - to clear this bit, first read it (already done)
	 * then write as a zero.
	 *
	 * we do reset all possible interrupts and look what we got later
	 */
/* printk("2"); */
	CANresetw(board, estat,
	    (CAN_ESTAT_BOFF_INT + CAN_ESTAT_ERR_INT + CAN_ESTAT_WAKE_INT));
	CANoutw(board, estat, 0xffff);

	/* Getting a precises time takes a lot of time
	 * if a time stamp is not needed, it can be switched of
	 * by ioctl() */
	if(timestamp[board]) {
	    do_gettimeofday(&(RxFifo->data[RxFifo->head]).timestamp);
	} else {
	    (RxFifo->data[RxFifo->head]).timestamp.tv_sec  = 0;
	    (RxFifo->data[RxFifo->head]).timestamp.tv_usec  = 0;
	}
	/* check for Bus Off condition */
	if (estat & CAN_ESTAT_BOFF_INT) {
	    /* printk("BusOff\n"); */

	    (RxFifo->data[RxFifo->head]).id = 0xFFFFFFFF;
	    (RxFifo->data[RxFifo->head]).flags = MSG_BUSOFF; 
	    RxFifo->status = BUF_OK;
	    RxFifo->head = ++(RxFifo->head) % MAX_BUFSIZE;
	    if(RxFifo->head == RxFifo->tail) {
		    printk("CAN[%d] RX: FIFO overrun\n", board);
		    RxFifo->status = BUF_OVERRUN;
	    } 
	    /* tell someone that there is a new error message */
	    wake_up_interruptible(&CanWait[board]); 

	}
	if (CAN_ESTAT_TX_WARN ==
			(estat & (CAN_ESTAT_ERR_INT + CAN_ESTAT_TX_WARN))) {
	    /* printk("Tx Warning level reached\n"); */
	}
	if (CAN_ESTAT_RX_WARN ==
			(estat & (CAN_ESTAT_ERR_INT + CAN_ESTAT_RX_WARN))) {
	    /* printk("Rx Warning level reached\n"); */
	}
	/* FCS - the Fault Confinement State 
	 *   if CAN_ESTAT_FCS0 is set :  Error Passive
	 *   else Error Active   
	 */
	/* if (estat & (CAN_ESTAT_ERR_INT + CAN_ESTAT_FCS)) { */


	/* Going back to Error Active can happen without an error Int ? */
	    if(0 == (estat & CAN_ESTAT_FCS)) {
		if (!erroractive) {
		    /* printk("Going error active\n"); */
		    /* do what you like here */
	        }
	        erroractive = 1;
	    }
	    if(CAN_ESTAT_FCS0 == (estat & CAN_ESTAT_FCS)) {
		if (erroractive) {
		    /* printk("Going error passive\n"); */
		    (RxFifo->data[RxFifo->head]).id = 0xFFFFFFFF;
		    (RxFifo->data[RxFifo->head]).flags = MSG_PASSIVE; 
		    RxFifo->status = BUF_OK;
		    RxFifo->head = ++(RxFifo->head) % MAX_BUFSIZE;
		    if(RxFifo->head == RxFifo->tail) {
			    printk("CAN[%d] RX: FIFO overrun\n", board);
			    RxFifo->status = BUF_OVERRUN;
		    } 
		    wake_up_interruptible(&CanWait[board]); 
		}
	        erroractive = 0;
	    }
	/* } */
	CANresetw(board, estat,
	    (CAN_ESTAT_BOFF_INT + CAN_ESTAT_ERR_INT + CAN_ESTAT_WAKE_INT));
    }

    if(0 == (estat & CAN_ESTAT_FCS)) {
	if (!erroractive) {
	    /* printk("Going error active\n"); */
	    /* do what you like here */
	}
	erroractive = 1;
    }


    /*
     loop as long as the CAN controller shows interrupts.
     check for message object interrupts.
    */
    while(1) {
	irqsrc = CANinw(board, iflag);
	/* printk("   0x%0x\n", irqsrc); */
	if (irqsrc == 0) break;

	/*
	 * If there is any IRQ, reset all of them, the IRQ sources are stored
	 * in the irqsrc variable for later evaluation.
	 * 5282 overwrites/clears with '1'
	 */
	CANoutw(board, iflag, irqsrc /* 0xffff */);

	if(timestamp[board]) {
	    do_gettimeofday(&(RxFifo->data[RxFifo->head]).timestamp);
	} else {
	    (RxFifo->data[RxFifo->head]).timestamp.tv_sec  = 0;
	    (RxFifo->data[RxFifo->head]).timestamp.tv_usec  = 0;
	}

	/* preset flags */
	(RxFifo->data[RxFifo->head]).flags =
			    (RxFifo->status & BUF_OVERRUN ? MSG_BOVR : 0);

	/* CANresetw(board, estat,  */
	   /* (CAN_ESTAT_BOFF_INT + CAN_ESTAT_ERR_INT + CAN_ESTAT_WAKE_INT)); */


    /*========== receive interrupt */
    if( irqsrc & ((1 << RECEIVE_STD_OBJ) + (1 << RECEIVE_EXT_OBJ))) {

	unsigned oid;
	int mrobject;	/* number of the message receive object */

	DBGprint(DBG_DATA, (" => got  RX IRQ[%d]: 0x%0x\n", board, irqsrc));

        /* Handle differences between base format frames and extended frames */
        if (irqsrc & (1 << RECEIVE_STD_OBJ)) {
            mrobject = RECEIVE_STD_OBJ;
	    while (((ctrl = CAN_READ_CTRL(RECEIVE_STD_OBJ))
			   & (REC_CODE_BUSY << 4)) == (REC_CODE_BUSY << 4)) {
		/* 20 cycles maximum wait */
		/* printf1("CAN_int, rx REC_CODE_BUSY"); */
	    }
	    last_tx_id = CAN_READ_OID(TRANSMIT_OBJ);
	    oid        = CAN_READ_OID(RECEIVE_STD_OBJ);
	} else {
            mrobject = RECEIVE_EXT_OBJ;
	    while (((ctrl = CAN_READ_CTRL(RECEIVE_EXT_OBJ))
			   & (REC_CODE_BUSY << 4)) == (REC_CODE_BUSY << 4)) {
		/* 20 cycles maximum wait */
		/* printf1("CAN_int, rx REC_CODE_BUSY"); */
	    }
	    last_tx_id = CAN_READ_XOID(TRANSMIT_OBJ);
	    oid        = CAN_READ_XOID(RECEIVE_EXT_OBJ);

	}
	/* printk(" %d/%x == %d/%x\n", oid, oid, last_tx_id, last_tx_id); */
	if (oid == last_tx_id) {
            /*
	    This is obviously a message we sent ourselves recently.
            The FlexCAN receives self-transmitted frames
            if there exists a matching receive MB.
	    There is no point in delivering this to the upper layer.
	    At least for using the driver in standard applications.
	    A CAN Analyser using can4linux _is_ interested in a time stamp
	    for the sent frame too.
	    We rely on the fact
	    that no other node will ever send a message with the same OID.
            */
	    if (selfreception[board]) {
		(RxFifo->data[RxFifo->head]).flags |= MSG_SELF;
	    } else {
		goto ResetRXInt;
	    }
	}

	(RxFifo->data[RxFifo->head]).id =  oid;
	if(mrobject ==  RECEIVE_STD_OBJ) {
	    /* (RxFifo->data[RxFifo->head]).id =  CAN_READ_OID(mrobject); */
	} else {
	    /* (RxFifo->data[RxFifo->head]).id =  CAN_READ_XOID(mrobject); */
	    (RxFifo->data[RxFifo->head]).flags |= MSG_EXT;
	}
	/* printk("CAN[%d] received id %d\n", */
		    /* board, (RxFifo->data[RxFifo->head]).id); */

	/* get message length,  strip length code */
	dummy = ctrl & 0x0f;
	(RxFifo->data[RxFifo->head]).length = dummy;
	dummy %= 9;	/* limit count to 8 bytes */

#if 0
/* There was reported a problem with using memcpy resulting in corrupted data */
	memcpy(&(RxFifo->data[RxFifo->head]).data[0],\
	    &(CAN_OBJ[RECEIVE_STD_OBJ].MSG0), dummy);
#else


#if 1
	for( i = 0; i < dummy; i++) {
	    RxFifo->data[RxFifo->head].data[i] =
			    CAN_OBJ[mrobject].msg[i];
	}
#else
	(RxFifo->data[RxFifo->head]).length = 8;

	RxFifo->data[RxFifo->head].data[0] =  RxFifo->head;
	RxFifo->data[RxFifo->head].data[1] =  RxFifo->tail;

#endif
#endif

	/* mark just written entry as OK and full */
	RxFifo->status = BUF_OK;

        /* adjust write pointer for received messages */
        if (!full) {
        RxFifo->head = (++(RxFifo->head)) % MAX_BUFSIZE;
        }

        if (RxFifo->head > RxFifo->tail)  printk("+");
        if (RxFifo->head < RxFifo->tail)  printk("-");

        if(RxFifo->head == RxFifo->tail) {
	    /* This was the last message which is fitting in the RX buffer */
	    /* printk("CAN[%d] RX: FIFO overrun\n", board); */
	    RxFifo->status = BUF_OVERRUN;
	    printk("CAN RX: FIFO overrun %d/%d\n", RxFifo->head, RxFifo->tail );
	    full = 1;
        } else full = 0;

	/*---------- kick the select() call  -*/
	/* This function will wake up all processes
	   that are waiting on this event queue,
	   that are in interruptible sleep
	*/
	wake_up_interruptible(&CanWait[board]);


	/* check for CAN controller overrun */
	/* ToDo */
ResetRXInt:
	CAN_WRITE_CTRL(mrobject, REC_CODE_EMPTY, 8);
	/* reading the free running timer will unlock any message buffers */
	(void) CANinw(board, timer);

    }
    /*========== transmit interrupt */
    if( irqsrc & (1 << TRANSMIT_OBJ)) {
	DBGprint(DBG_DATA, (" => got  TX IRQ[%d]: 0x%0x\n", board, irqsrc));
	/* CAN_register_dump(); */
	if( TxFifo->free[TxFifo->tail] == BUF_EMPTY ) {
	    /* TX FIFO empty, nothing more to sent */
	    /* printk("TXE\n"); */
	    TxFifo->status = BUF_EMPTY;
            TxFifo->active = 0;
	    /* This function will wake up all processes
	       that are waiting on this event queue,
	       that are in interruptible sleep
	    */
	    wake_up_interruptible(&CanOutWait[board]); 
            goto Tx_done;
	} else {
	    /* printk("TX\n"); */
	}

        /* enter critical section */
        save_flags(flags);cli();

        while ((ctrl = CAN_READ_CTRL(TRANSMIT_OBJ)
                 & (REC_CODE_BUSY << 4)) == (REC_CODE_BUSY << 4)) {
        	/* printk("CAN_int, tx REC_CODE_BUSY"); */
	}

	/* now we can check here if an RTR was received with this TX channel */
	if ((    CAN_READ_CTRL(TRANSMIT_OBJ)
	      & (REC_CODE_FULL << 4)) == (REC_CODE_FULL << 4))
	{
            printk("CAN_int, got rx interrupt on transmit channel,  OID= %d\n",
						    CAN_READ_OID(TRANSMIT_OBJ));
	} else {
	    /* Real Transmit Interrupt */

	    /* Writing Control/Status word to hold TX Message Object inactive */
	    CAN_WRITE_CTRL(TRANSMIT_OBJ, TRANS_CODE_NOT_READY, 1);

	    /* fill the frame info and identifier fields , ID-Low and ID-High */
	    if( (TxFifo->data[TxFifo->tail]).flags & MSG_EXT ) {
		/* use ID in extended message format */
		DBGprint(DBG_DATA, ("---> send ext message \n"));
		if( (TxFifo->data[TxFifo->tail]).flags & MSG_RTR) {
		    CAN_WRITE_XOID_RTR(TRANSMIT_OBJ,
		    	(TxFifo->data[TxFifo->tail]).id);
		} else {
		    CAN_WRITE_XOID(TRANSMIT_OBJ,
		    	(TxFifo->data[TxFifo->tail]).id);
		}
	    } else {
		DBGprint(DBG_DATA, ("---> send std message \n"));
		if( (TxFifo->data[TxFifo->tail]).flags & MSG_RTR) {
		    CAN_WRITE_OID_RTR(TRANSMIT_OBJ,
		    	(TxFifo->data[TxFifo->tail]).id);
		} else {
		    CAN_WRITE_OID(TRANSMIT_OBJ,
		    	(TxFifo->data[TxFifo->tail]).id);
		}
	    }

#if 0
/* There was reported a problem with using memcpy resulting in corrupted data */
	    memcpy( (void *)&(CAN_OBJ[TRANSMIT_OBJ].MSG0),
		(const void *)&(TxFifo->data[TxFifo->tail]).data[0],
		(TxFifo->data[TxFifo->tail]).length);
#else
	    for( i = 0; i < (TxFifo->data[TxFifo->tail]).length; i++) {
		    CAN_OBJ[TRANSMIT_OBJ].msg[i] =
			TxFifo->data[TxFifo->tail].data[i];
	    }

#endif


	    /* Writing Control/Status word (active code, length) */
	    CAN_WRITE_CTRL(TRANSMIT_OBJ, TRANS_CODE_TRANSMIT_ONCE,
		    (TxFifo->data[TxFifo->tail]).length);

	    TxFifo->free[TxFifo->tail] = BUF_EMPTY; /* now this entry is EMPTY */
	    TxFifo->tail = ++(TxFifo->tail) % MAX_BUFSIZE;

	    /* leave critical section */
	    restore_flags(flags);
	}

Tx_done:
	/* Reset Interrupt pending at Transmit Object */
	/* CANsetw(board, iflag, ~(1 << TRANSMIT_OBJ)); */
	/* CANsetw(board, iflag, (1 << TRANSMIT_OBJ)); */
   }
	/* for some reason it wasn't good enough to reset iflag
	 * just after reading the iflag register in while( (irqsrc = ...)
	 */
	/* CANsetw(board, iflag, ~(1 << TRANSMIT_OBJ)); */
   }

    DBGprint(DBG_DATA, (" => leave IRQ[%d]\n", board));
    /* printk(" => leave IRQ[%d]\n", board); */
/* printk("\n"); */
    reset_led();

    return IRQ_RETVAL(IRQ_HANDLED);
}


/* dump all FlexCAN registers to printk */
void CAN_register_dump(void)
{
volatile flex_can_t *tou_can = (flex_can_t *)(MCF_MBAR + 0x1c0000);

    printk("Flex CAN register layout\n");

#define  printregister(s, name) printk(s, &name , name)

    /* printk(" %p: 0x%x \n", tou_can, *(unsigned char *)tou_can); */
    /* printk(" %p: 0x%x \n", (unsigned char *)tou_can + 1, *(((unsigned char *)tou_can) + 1)); */

    printregister
    (" CAN_ModulConfigRegister      %p %0x\n", CAN_ModulConfigRegister);
    printregister
    (" CAN_ControlReg0              %p %0x\n", CAN_ControlReg0);
    printregister
    (" CAN_ControlReg1              %p %0x\n", CAN_ControlReg1);
    printregister
    (" CAN_ControlReg2              %p %0x\n", CAN_ControlReg2);

    printregister
    (" CAN_PrescalerDividerRegister %p %0x\n", CAN_PrescalerDividerRegister);
    printregister
    (" CAN_TimerRegister            %p %0x\n", CAN_TimerRegister);
#if 0
    printregister
    (" CAN_ReceiveGlobalMask        %p %0x\n", CAN_ReceiveGlobalMask);
    printregister
    (" CAN_ReceiveBuffer14Mask      %p %0x\n", CAN_ReceiveBuffer14Mask);
    printregister
    (" CAN_ReceiveBuffer15Mask      %p %0x\n", CAN_ReceiveBuffer15Mask);
#else
    printregister
    (" CAN_ReceiveGlobalMaskHigh    %p %0x\n", CAN_ReceiveGlobalMaskHigh);
    printregister
    (" CAN_ReceiveGlobalMaskLow     %p %0x\n", CAN_ReceiveGlobalMaskLow);
    printregister
    (" CAN_ReceiveBuffer14MaskHigh  %p %0x\n", CAN_ReceiveBuffer14MaskHigh);
    printregister
    (" CAN_ReceiveBuffer14MaskLow   %p %0x\n", CAN_ReceiveBuffer14MaskLow);
    printregister
    (" CAN_ReceiveBuffer15MaskHigh  %p %0x\n", CAN_ReceiveBuffer15MaskHigh);
    printregister
    (" CAN_ReceiveBuffer15MaskLow   %p %0x\n", CAN_ReceiveBuffer15MaskLow);
#endif
    printregister
    (" CAN_ErrorStatusRegister      %p %0x\n", CAN_ErrorStatusRegister);
    printregister
    (" CAN_InterruptMasks           %p %0x\n", CAN_InterruptMasks);
    printregister
    (" CAN_InterruptFlags           %p %0x\n", CAN_InterruptFlags);
    printregister
    (" CAN_ReceiveErrorCounter      %p %0x\n", CAN_ReceiveErrorCounter);
    printregister
    (" CAN_TransmitErrorCounter     %p %0x\n", CAN_TransmitErrorCounter);
}


/* dump the content of the selected message object (MB) to printk */
void CAN_object_dump(int object)
{
int board = 0;   /* be prepared for an board paramter, if later on .. */
unsigned int vh;
unsigned int vl;

volatile unsigned char *cp =
		(unsigned char *)(MCF_MBAR + 0x1c0080 + (0x10 * object));

    printk("Flex CAN object %d\n", object);
    for(vl = 0; vl < 16; vl++) {
	printk("%2x ", *(cp + vl));
    }
    printk("\n");


    vl = CAN_OBJ[object].ctl_status;
    printk(" Ctrl/Status 0x%x\n", vl);
    vh = CAN_OBJ[object].id_high.id_high;
    printk(" Id high 0x%x\n", vh);
    vl = CAN_OBJ[object].id_low.id_low;
    printk(" Id low 0x%x\n", vl);
    printk(" StdId  %d/0x%x\n", vh >> 5, vh >> 5);
    vh =  (vl >> 1) + ((vh & 7) << 15) + ((vh & 0xffe0) << 13);
    printk(" ExtId  %d/0x%x\n", vh, vh);
}
