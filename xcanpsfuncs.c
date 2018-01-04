/* xcanpsfuncs - CAN controller Xilinx xcanps part of can4linux drivers
*
* can4linux -- LINUX CAN device driver source
* 
* This file is subject to the terms and conditions of the GNU General Public
* License.  See the file "COPYING" in the main directory of this archive
* for more details.
*
* 
* Copyright (c) 2013 Heinz-Juergen Oertel (oe@port.de)
*------------------------------------------------------------------
* $Header: $
*
*--------------------------------------------------------------------------
*
*/

/* use it for pr_info() and consorts */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include "defs.h"
#include "linux/delay.h"
#include "xcanps.h"
#include <linux/sched.h>


/* int	CAN_Open = 0; */

/* timing values 
 in this case we need for all 10 possible timings two bytes each.
 This might be different on some controllers */
u8 CanTiming[10][2]={
	{CAN_BRPR_10K,  CAN_BTR_10K},
	{CAN_BRPR_20K,  CAN_BTR_20K},
	{CAN_BRPR_50K,  CAN_BTR_50K},
	{CAN_BRPR_100K, CAN_BTR_100K},
	{CAN_BRPR_125K, CAN_BTR_125K},
	{CAN_BRPR_250K, CAN_BTR_250K},
	{CAN_BRPR_500K, CAN_BTR_500K},
	{CAN_BRPR_800K, CAN_BTR_800K},
	{CAN_BRPR_1000K,CAN_BTR_1000K}};



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
/* when ever we need while debugging some controller status information */
int CAN_ShowStat (int minor)
{
    /* Show only the first n register valuaes */
    CAN_register_dump(minor, 7);
    return 0;
}

void CAN_register_dump(int minor, int n)
{
int i;
char *register_names[] = {
 "srr", "msr", "brpr", "btr", "ecr", "esr", "sr", "isr", "ier", "icr", "tcr", "wir", "txfifo_id",
 "txfifo_dlc", "txfifo_data1", "txfifo_data2", "txhpb_id", "txhpb_dlc", "txhpb_data1",
 "txhpb_data2", "rxfifo_id", "rxfifo_dlc", "rxfifo_data1", "rxfifo_data2", "afr", "afmr1",
 "afir1", "afmr2", "afir2", "afmr3", "afir3", "afmr4", "afir4" };

    if(n > 33) n = 33;
    printk("-----------\n");
    for( i = 0; i < n; i++) {
	printk("%p %13s: 0x%08x\n",
		can_base[0] + 4*i,
		register_names[i],
		(__raw_readl ((void __iomem *)can_base[0]+4*i)  ));
    }
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
* exception:
* ERROR WARNING LIMIT REGISTER (EWLR)
* The SJA1000 defines a EWLR, reaching this Error Warning Level
* an Error Warning interrupt can be generated.
* The default value (after hardware reset) is 96. In reset
* mode this register appears to the CPU as a read/write
* memory. In operating mode it is read only.
* Note, that a content change of the EWLR is only possible,
* if the reset mode was entered previously. An error status
* change (see status register; Table 14) and an error
* warning interrupt forced by the new register content will not
* occur until the reset mode is cancelled again.
*/

int can_GetStat(
	struct inode *inode,
	struct file *file,
	CanStatusPar_t *stat
	)
{
unsigned int minor = iminor(inode);	
msg_fifo_t *Fifo;
unsigned long flags;
unsigned long ecr;
int rx_fifo = ((struct _instance_data *)(file->private_data))->rx_index;

    stat->type = CAN_TYPE_XCANPS;

    stat->baud = Baud[minor];
    stat->status = CANinl(minor, sr);
    stat->error_warning_limit = 96;

    ecr = CANinl(minor, ecr);
    stat->rx_errors = (ecr & XCANPS_ECR_REC_MASK) >> XCANPS_ECR_REC_SHIFT;
    stat->tx_errors = ecr & XCANPS_ECR_TEC_MASK; 

    stat->error_code= CANinl(minor, esr);

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

int CAN_ChipReset (int minor)
{
    DBGin();
    /* Reset CAN controller */
    /* Writing a 1 to the SRST bit in the SRR register.
     * The controller enters Configuration mode
     * immediately following the software reset.
     */
    CANoutl(minor, srr, XCANPS_SRR_SRST_MASK);

    DBGout();
    return 0;
}


/*
 * Configures bit timing registers directly. Chip must be in bus off state.
 */
int CAN_SetBTR (int minor, int btr0, int btr1)
{
    (void)minor;
    (void)btr0;
    (void)btr1;

    DBGin();

    DBGout();
    return 0;
}


/*
 * Configures bit timing. Chip must be in configuration mode.
 */
int CAN_SetTiming (int minor, int baud)
{
int i	   = 4;
int custom = 0;
int isopen;
int retval = 0;

    DBGin();

    isopen = atomic_read(&Can_isopen[minor]);
    if ((isopen > 1) && (Baud[minor] != baud)) {
	DBGprint(DBG_DATA, ("isopen = %d", isopen));
	DBGprint(DBG_DATA, ("refused baud[%d]=%d already set to %d",
					minor, baud, Baud[minor]));
	return -1;
    }

    DBGprint(DBG_DATA, ("baud[%d]=%d", minor, baud));
    switch(baud) {
	case   10: i = 0; break;
	case   20: i = 1; break;
	case   50: i = 2; break;
	case  100: i = 3; break;
	case  125: i = 4; break;
	case  250: i = 5; break;
	case  500: i = 6; break;
	case  800: i = 7; break;
	case 1000: i = 8; break;
	default  : 
		custom=1;
		break;
    }

    /* hardware depending code follows here */

    if( custom ) {
	/* set direct register values */
        CANoutl(minor, brpr, (u8) (baud >> 8) & 0xff);
        CANoutl(minor, btr,  (u8) (baud & 0xff ));
    } else {
	/* use table values, i is index */
	/* If value of BTR is 0xff - bit rate is not supported */ 
	if (CanTiming[i][1] == 0xff) {
	    retval = -1;
	} else {
	    CANoutl(minor, brpr, (u8) CanTiming[i][0]);
	    CANoutl(minor, btr,  (u8) CanTiming[i][1]);
	}
    }

    DBGout();
    return retval;
}


/*
   Reset error Counter information in /proc
   Clear pending Interrupts
   Set Interrupt sources
   Activate CAN

*/
int CAN_StartChip (int minor)
{
    (void)minor;

    DBGin();
    RxErr[minor] = TxErr[minor] = 0L;

    /* set normal operating mode */
    /* Normal mode, reset MSR: LBACK, SNOOP, SLEEP
     * that is msr content is == 0
     * For test purposes it might useful to set the CAN
     * in loop-back mode:
     * CANoutl(minor, ms, XCANPS_MSR_LBACK_MASK);
     *
     */
    CANoutl(minor, msr, 0);

    CAN_SetTiming(minor, Baud[minor]    );
    CAN_SetMask  (minor, AccCode[minor], AccMask[minor] );
    /* set the Chip ENable bit */
    CANoutl(minor, srr, XCANPS_SRR_CEN_MASK);
    // CAN_register_dump(minor, 7);

    /* now clear pending interrupts */


    /* And set used and supported interrupt sources
     * setting XCANPS_IXR_TXFEMP_MASK makes no sense now.
     * the interrupt comes always when TXFIFO is empty.
     * Much to often hoe can4linux uses TX.
     */ 
    CANoutl(minor, ier, 
	    XCANPS_IXR_TXOK_MASK
	  | XCANPS_IXR_RXOK_MASK
	  | XCANPS_IXR_RXOFLW_MASK
	  | XCANPS_IXR_BSOFF_MASK
	  | XCANPS_IXR_ERROR_MASK
	);

    DBGout();
    return 0;
}


/*
* If the driver is used by more than one application,
* one should take care that this functionality (like some others)
* can not be called by any application.
* Stopping the shared CAN will stop it for all other processes as well.
*
* can4linux blocks this function (and others)  in ioctl.c
*/
int CAN_StopChip (int minor)
{
    DBGin();

    DBGout();
    return 0;
}

/* set value of the output control register */
int CAN_SetOMode (int minor, int arg)
{
    (void)minor;
    (void)arg;

    DBGin();

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
int CAN_SetListenOnlyMode (int minor,
	int arg)	/* true - set Listen Only, false - reset */
{
    (void)minor;
    (void)arg;

    DBGin();
    if (arg) {
	/* set listen only mode */
	;
    } else {
	/* set active mode */
	;
    }

    DBGout();
    return 0;
}

/* set Acceptance Code and Mask Registers */
int CAN_SetMask (int minor, unsigned int code, unsigned int mask)
{
    (void)minor;
    (void)code;
    (void)mask;

    DBGin();
    /* set register values */

    /* put values back in global variables for sysctl */
    AccCode[minor] = code;
    AccMask[minor] = mask;
    DBGout();
    return 0;
}


/* 
Single CAN frames or the very first Message are copied into the CAN controller
using this function.
After a succesful transmission, an interrupt will be generated,
which will be handled in the CAN ISR CAN_Interrupt()
*/
int CAN_SendMessage (int minor, canmsg_t *tx)
{
int stat;
unsigned int id = 0;
int i = 0;

    DBGin();
    /* wait for transmission complete, read canstat  */
    while ( (stat=CANinl(minor, sr) & XCANPS_SR_TXFLL_MASK)) {
	cond_resched();
    }

    /* fill in message id, message data, .... */
    if(tx->flags & MSG_EXT) {
	/* Extended frame format */
	id = (tx->id & 0x3FFFF) << XCANPS_IDR_ID2_SHIFT;
	id |= (tx->id & 0x1FFC0000) << XCANPS_IDR_ID1_X_SHIFT;
	id |= 1 << XCANPS_IDR_IDE_SHIFT;
	if(tx->flags & MSG_RTR) {
	    id |= XCANPS_IDR_RTR_MASK;
	}
    } else {
	/* Base frame format */
	id = tx->id << XCANPS_IDR_ID1_SHIFT;
	id &= XCANPS_IDR_ID1_MASK;
	if(tx->flags & MSG_RTR) {
	    id |= XCANPS_IDR_SRR_MASK;
	}
    }
    CANoutl(minor, txfifo_id, id);
    CANoutl(minor, txfifo_dlc, tx->length << XCANPS_DLCR_DLC_SHIFT);

    CANoutl(minor, txfifo_data1,
	      (tx->data[0] << 24)
	    + (tx->data[1] << 16)
	    + (tx->data[2] <<  8)
	    + (tx->data[3]));
    CANoutl(minor, txfifo_data2,
	      (tx->data[4] << 24)
	    + (tx->data[5] << 16)
	    + (tx->data[6] <<  8)
	    + (tx->data[7]));

    /* issue transmission request to the CAN controller */
    /* The Xilinx xcan doesn't need a special request
     * to send the frame.
     * It is send, afetr the last of the four words is written 
     */

    /* 
     * Save last message that was sent.
     * Since can4linux 3.5 multiple processes can access
     * one CAN interface. On a CAN interrupt this message is copied into 
     * the receive queue of each process that opened this same CAN interface.
     */
    memcpy(
	(void *)&last_Tx_object[minor],
	(void *)tx,
	sizeof(canmsg_t));

    DBGout();
    return i;
}


/*
 * The plain interrupt
 *
 */

irqreturn_t CAN_Interrupt( int irq, void *dev_id)
{
int minor;
//int i = 0;
int rx_fifo;
struct timeval  timestamp;
unsigned long flags;
int ext;			/* flag for extended message format */
unsigned int id;
int irqsrc;
msg_fifo_t   *RxFifo; 
msg_fifo_t   *TxFifo;
#if CAN_USE_FILTER
msg_filter_t *RxPass;
#endif 
#if 1
int first = 0;
#endif 
//unsigned int ecc = 0;

#if CONFIG_TIME_MEASURE
    set_measure_pin();
#endif


    minor = *(int *)dev_id;

    RxFifo = &Rx_Buf[minor][0]; 
    TxFifo = &Tx_Buf[minor];
#if CAN_USE_FILTER
    RxPass = &Rx_Filter[minor];
#endif 

    /* read status if CAN has an interrupt pending */
    irqsrc = CANinl(minor, isr);

    /* not all bits in the Interrupt source masks
     * are handled Interrupt conditions.
     * Therefore mask them out
     */
#if 0
    irqsrc &=  XCANPS_IXR_TXOK_MASK
	     | XCANPS_IXR_BSOFF_MASK
	     | XCANPS_IXR_RXOK_MASK
             | XCANPS_IXR_RXOFLW_MASK 
	     ;
    // printk("CAN - ISR ; minor = %d, isr= 0x%08x\n", *(int *)dev_id, irqsrc);
#endif

    if(irqsrc == 0) {
         /* first call to ISR, it's not for me ! */
#if CONFIG_TIME_MEASURE
	reset_measure_pin();
#endif
#if LINUX_VERSION_CODE >= 0x020500 
	return IRQ_RETVAL(IRQ_NONE);
#else
	goto IRQdone_doneNothing;
#endif
    }

    /* clear interrupts */
    CANoutl(minor, icr, XCANPS_IXR_ALL);

    /* Whatever interrupt we have, update the tx error counter
     * and rx error counter information in /proc/sys/dev/Can
     */
    {
    int ecr;
	ecr = CANinl(minor, ecr);
	TxErrCounter[minor] = ecr & XCANPS_ECR_TEC_MASK;
	RxErrCounter[minor] = (ecr & XCANPS_ECR_REC_MASK) >> XCANPS_ECR_REC_SHIFT;
    }

    do {
    /* loop as long as the CAN controller shows interrupts */
    /* can_dump(minor); */
#if defined(DEBUG)
    /* how often do we loop through the ISR ? */
    /* if(first) printk("n = %d\n", first); */
    /* we can have a /proc/sys/dev/Can/irqloop
       to store the max counter value
       for debugging purposes to see how heavy the isr is used
       */
	first++;
	if (first > 10) return IRQ_RETVAL(IRQ_HANDLED);
#endif

	get_timestamp(minor, &timestamp);

	for(rx_fifo = 0; rx_fifo < CAN_MAX_OPEN; rx_fifo++) {
	    RxFifo = &Rx_Buf[minor][rx_fifo];

	    RxFifo->data[RxFifo->head].timestamp = timestamp;

	    /* preset flags */
	    (RxFifo->data[RxFifo->head]).flags =
			    (RxFifo->status & BUF_OVERRUN ? MSG_BOVR : 0);
	}


	/*========== receive interrupt */
	if( irqsrc & XCANPS_IXR_RXOK_MASK ) {
	    u32 dummy;
	    u32 length;
	    u32 data1, data2; 

     //CAN_register_dump(minor, 33);

	    // pr_info("isr: %08x\n", CANinl(minor, isr));
	    //
	    // mdelay 2 ms for testing the RX FIFO Overflow
	    // with sending fast at 125K 
	    // an overflow should happen after twice the number
	    // of RX Fifo size frames send. 
	    // mdelay(2);

	    /* read IDR */
	    //dummy  = CANinl(minor, rxfifo_id);
	    dummy = (__raw_readl ((void __iomem *)can_base[minor] + 0x50));
	    // pr_info(" id %08x\n", dummy);
	    /* get message length as received in the frame */
	    /* strip length code */ 
	    // length = CANinl(minor, rxfifo_dlc); 
	    length = (__raw_readl ((void __iomem *)can_base[minor] + 0x54));
	    // pr_info("dlc %08x\n", length);
	    length = length >> XCANPS_DLCR_DLC_SHIFT;
	    /* Read out frame data */
	    // data1 = CANinl(minor, rxfifo_data1);
	    data1 = (__raw_readl ((void __iomem *)can_base[minor] + 0x58));
	    // data2 = CANinl(minor, rxfifo_data2);
	    data2 = (__raw_readl ((void __iomem *)can_base[minor] + 0x5c));

	    /* ---------- fill frame data -------------------------------- */
	    /* handle all subscribed rx fifos */

	    for(rx_fifo = 0; rx_fifo < CAN_MAX_OPEN; rx_fifo++) {
		/* for every rx fifo */

		if (CanWaitFlag[minor][rx_fifo] == 1) {
		    /* this FIFO is in use */
		    RxFifo = &Rx_Buf[minor][rx_fifo]; /* prepare buffer to be used */
		    /* pr_info("> filling buffer [%d][%d]\n", minor, rx_fifo);  */
		    if( dummy & XCANPS_IDR_IDE_MASK) {
			/* received extended Id frame */ 
			(RxFifo->data[RxFifo->head]).flags |= MSG_EXT;
			if(dummy & XCANPS_IDR_RTR_MASK) { 
			    (RxFifo->data[RxFifo->head]).flags |= MSG_RTR;
			}
			(RxFifo->data[RxFifo->head]).id =
			      ((dummy & XCANPS_IDR_ID2_MASK) >> XCANPS_IDR_ID2_SHIFT)
			    | ((dummy & XCANPS_IDR_ID1_MASK) >> 3);
		    } else {
			/* received base Id frame */ 
			if(dummy & XCANPS_IDR_SRR_MASK) {
			    (RxFifo->data[RxFifo->head]).flags |= MSG_RTR;
			}
			(RxFifo->data[RxFifo->head]).id =
			    (dummy & XCANPS_IDR_ID1_MASK) >> XCANPS_IDR_ID1_SHIFT;
		    }

		    /* get message length as received in the frame */
		    /* strip length code */ 
		    (RxFifo->data[RxFifo->head]).length = length;

		    (RxFifo->data[RxFifo->head]).data[0] = data1 >> 24;
		    (RxFifo->data[RxFifo->head]).data[1] = data1 >> 16;
		    (RxFifo->data[RxFifo->head]).data[2] = data1 >> 8;
		    (RxFifo->data[RxFifo->head]).data[3] = data1;

		    (RxFifo->data[RxFifo->head]).data[4] = data2 >> 24;
		    (RxFifo->data[RxFifo->head]).data[5] = data2 >> 16;
		    (RxFifo->data[RxFifo->head]).data[6] = data2 >> 8;
		    (RxFifo->data[RxFifo->head]).data[7] = data2;

		    /* mark just written entry as OK and full */
		    RxFifo->status = BUF_OK;
		    /* Handle buffer wrap-around */
		    ++(RxFifo->head);
		    RxFifo->head %= MAX_BUFSIZE;
		    if(unlikely(RxFifo->head == RxFifo->tail)) {
			    pr_err("CAN[%d][%d] RX: FIFO overrun\n", minor, rx_fifo);
			    RxFifo->status = BUF_OVERRUN;
		    } 

		    /*---------- kick the select() call  -*/
		    /* This function will wake up all processes
		       that are waiting on this event queue,
		       that are in interruptible sleep
		    */
		    /* pr_info(" should wake [%d][%d]\n", minor, rx_fifo); */
		    wake_up_interruptible(&CanWait[minor][rx_fifo]); 

		}
	    } /* for( rx fifos ...) */
	    /* ---------- / fill frame data -------------------------------- */
#if 0
	    if(CANin(minor, canstat) & CAN_DATA_OVERRUN ) {
		     pr_info("CAN[%d] Rx: Overrun Status \n", minor);
		     CANout(minor, cancmd, CAN_CLEAR_OVERRUN_STATUS );
	    }
#endif
	} /* end RX Interrupt */

	/*========== transmit interrupt */
	if( irqsrc & XCANPS_IXR_TXOK_MASK ) {
	    u32 dlc;
	    /* CAN frame successfully sent */
	    // pr_info("==> TX Interrupt\n");
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
		    /* pr_info("self copy to [%d][%d]\n", * minor, rx_fifo); */
			
		    /*
		     * Don't copy the message in the receive queue
		     * of the process that sent the message unless
		     * this process requested selfreception.
		     */
		    if ((last_Tx_object[minor].cob == rx_fifo) && 
			(selfreception[minor][rx_fifo] == 0))
		    {
		    /* pr_info("CAN[%d][%d] Don't copy message in my queue\n",
		     * minor, rx_fifo); */
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
		    ++(RxFifo->head);
		    RxFifo->head %= MAX_BUFSIZE;

		    if(unlikely(RxFifo->head == RxFifo->tail)) {
			pr_err("CAN[%d][%d] RX: FIFO overrun\n", minor, rx_fifo);
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


	    if( TxFifo->free[TxFifo->tail] == BUF_EMPTY ) {
		/* TX FIFO empty, nothing more to sent */
		/* pr_info("TXE\n"); */
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
	    /* spin_lock_irqsave(&write_splock[minor], flags); */

	    local_irq_save(flags);
	    /* pr_info("CAN[%d][%d] enter\n", minor, rx_fifo); */

	    /* The TX message FIFO contains other CAN frames to be sent
	     * The next frame in the FIFO is copied into the last_Tx_object
	     * and directly into the hardware of the CAN controller
	     */
	    memcpy(
		    (void *)&last_Tx_object[minor],
		    (void *)&TxFifo->data[TxFifo->tail],
		    sizeof(canmsg_t));

	    ext = (TxFifo->data[TxFifo->tail]).flags & MSG_EXT;
	    id = (TxFifo->data[TxFifo->tail]).id;
	    if(ext) {
		u32 tmpid = id;
		DBGprint(DBG_DATA, ("---> send ext message"));
		/* Extended frame format */
		id = (id & 0x3FFFF) << XCANPS_IDR_ID2_SHIFT;
		id |= (tmpid & 0x1FFC0000) << XCANPS_IDR_ID1_X_SHIFT;
		id |= 1 << XCANPS_IDR_IDE_SHIFT;
		if( (TxFifo->data[TxFifo->tail]).flags & MSG_RTR) {
		    id |= XCANPS_IDR_RTR_MASK;
		}
	    } else {
		DBGprint(DBG_DATA, ("---> send std message"));
		/* Base frame format */
		id = id << XCANPS_IDR_ID1_SHIFT;
		id &= XCANPS_IDR_ID1_MASK;
		if( (TxFifo->data[TxFifo->tail]).flags & MSG_RTR) {
		    id |= XCANPS_IDR_SRR_MASK;
		}
	    }
	    CANoutl(minor, txfifo_id, id);

	    dlc = (TxFifo->data[TxFifo->tail]).length;
	    dlc &= 0x0f;		/* restore length only */
	    CANoutl(minor, txfifo_dlc, dlc << XCANPS_DLCR_DLC_SHIFT);

	    CANoutl(minor, txfifo_data1,
		      ((TxFifo->data[TxFifo->tail]).data[0] << 24)
		    + ((TxFifo->data[TxFifo->tail]).data[1] << 16)
		    + ((TxFifo->data[TxFifo->tail]).data[2] <<  8)
		    + ((TxFifo->data[TxFifo->tail]).data[3]));
	    CANoutl(minor, txfifo_data2,
		      ((TxFifo->data[TxFifo->tail]).data[4] << 24)
		    + ((TxFifo->data[TxFifo->tail]).data[5] << 16)
		    + ((TxFifo->data[TxFifo->tail]).data[6] <<  8)
		    + ((TxFifo->data[TxFifo->tail]).data[7]));
	    /* no transmission request needed,
	     * XCAN sends after all four TXBUF registers are filled
	     */


	    /* now this entry is EMPTY */
	    TxFifo->free[TxFifo->tail] = BUF_EMPTY;
	    ++(TxFifo->tail);
	    TxFifo->tail %= MAX_BUFSIZE;

	    /* leave critical section */
	    /* pr_info("CAN[%d][%d] leave\n", minor, rx_fifo); */
	    local_irq_restore(flags);
	    /* spin_unlock_irqrestore(&write_splock[minor], flags); */


	}
Tx_done:

	/*========== arbitration lost */
	if( irqsrc &  XCANPS_IXR_ARBLST_MASK) {
	    ArbitrationLost[minor]++; 
	}

	/*========== error status */
	if( irqsrc & (
	      XCANPS_IXR_BSOFF_MASK 
	    | XCANPS_IXR_ERROR_MASK
		)) {
	int status;

	    /* ESTAT  bits in SR Register always reflect CAN status
	     *
	     * XCANPS_SR_ESTAT_MASK
	     * XCANPS_SR_ESTAT_SHIFT
	     *
	     *
	     *
	     *
	     * */
	    status = CANinl(minor, sr) >> XCANPS_SR_ESTAT_SHIFT;
	    /*
	     *  00: Indicates Configuration Mode (CONFIG = 1).  Error State is undefined.
	     *  01: Indicates Error Active State.
	     *  11: Indicates Error Passive State.
	     *  10: Indicates Bus Off State.			
	     *
	     */

	    pr_info("CAN[%d], error Interrupt ESTAT %d\n",
		    minor, status & 0x03);
	    CANoutl(minor, icr, 
		      XCANPS_IXR_BSOFF_MASK 
		    | XCANPS_IXR_ERROR_MASK);

	}
	/*========== CAN data overrun interrupt */
	if( irqsrc & XCANPS_IXR_RXOFLW_MASK) {
	    /* This bit indicates that a message has been lost.
	     * This condition occurs when a new message is beeing received
	     * and the receive FIFO is full.
	     * This bit can be cleared by writing to the ICR .
	     * This bit is also cleared when a 0 is written
	     * to the CEN bit in the SRR.
	     *
	     * It is very difficult to test this condition.
	     * One way of doing so: delay the isr CAN reception
	     * See above = receive interrupt ===
	     */
	    pr_err("CAN[%d]: controller RX FIFO overrun!\n", minor);
	    Overrun[minor]++;

	    /* insert error */
	    RxErr[minor]++;
	    for(rx_fifo = 0; rx_fifo < CAN_MAX_OPEN; rx_fifo++) {
		/* for every rx fifo */
		if (CanWaitFlag[minor][rx_fifo] == 1) {
		    /* this FIFO is in use */
		    (RxFifo->data[RxFifo->head]).flags |= MSG_OVR; 
		    (RxFifo->data[RxFifo->head]).id = 0xFFFFFFFF;
		    (RxFifo->data[RxFifo->head]).length = 0;
		    RxFifo->status = BUF_OK;
		    ++(RxFifo->head);
		    RxFifo->head %= MAX_BUFSIZE;
		    if(unlikely(RxFifo->head == RxFifo->tail)) {
			pr_err("CAN[%d][%d] RX: FIFO overrun\n", minor, rx_fifo);
			RxFifo->status = BUF_OVERRUN;
		    } 
		    /* tell someone that there is a new error message */
		    wake_up_interruptible(&CanWait[minor][rx_fifo]); 
		}
	    }
	    /* Clear Overrun status */
	    CANoutl(minor, icr, XCANPS_IXR_RXOFLW_MASK);
	}

	/* check again for pending interrupts */
	irqsrc = CANinl(minor, isr);
    // pr_info("CAN - ISR ;  isr= 0x%08x\n",  irqsrc);
#if 0
	irqsrc &=  XCANPS_IXR_TXOK_MASK
		 | XCANPS_IXR_BSOFF_MASK
		 | XCANPS_IXR_RXOK_MASK
		 | XCANPS_IXR_RXOFLW_MASK; 
#endif
    } while( irqsrc != 0 );

    DBGprint(DBG_DATA, (" => leave IRQ[%d]", minor));

//    board_clear_interrupts(minor);

#if CONFIG_TIME_MEASURE
    reset_measure_pin();
#endif

    return IRQ_RETVAL(IRQ_HANDLED);
}
