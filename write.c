/*
 * can_write - can4linux CAN driver module
 *
 * can4linux -- LINUX CAN device driver source
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * 
 * Copyright (c) 2001 port GmbH Halle/Saale
 * (c) 2013 Heinz-Jürgen Oertel (oe@port.de)
 *          Claus Schroeter (clausi@chemie.fu-berlin.de)
 *------------------------------------------------------------------
 *
 *--------------------------------------------------------------------------
 *
 *
 *
 *
 *
 *
 *--------------------------------------------------------------------------
 */


/**
* \file write.c
* \author Heinz-Jürgen Oertel
*
*/
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include "defs.h"
#include <linux/sched.h>

/* \fn size_t can_write( __LDDK_WRITE_PARAM) */
/***************************************************************************/
/**

\brief size_t write(int fd, const char *buf, size_t count);
write CAN messages to the network
\param fd The descriptor to write to.
\param buf The data buffer to write (array of CAN canmsg_t).
\param count The number of bytes to write.

write  writes  up to \a count CAN messages to the CAN controller
referenced by the file descriptor fd from the buffer
starting at buf.



\par Errors

the following errors can occur

\li \c EBADF  fd is not a valid file descriptor or is not open
              for writing.

\li \c EINVAL fd is attached to an object which is unsuitable for
              writing.

\li \c EFAULT buf is outside your accessible address space.

\li \c EINTR  The call was interrupted by a signal before any
              data was written.



\returns
On success, the number of CAN messages written are returned
(zero indicates nothing was written).
On error, -1 is returned, and errno is set appropriately.

\internal
*/

__LDDK_WRITE_TYPE can_write( __LDDK_WRITE_PARAM )
{
unsigned int minor = __LDDK_MINOR;
msg_fifo_t *TxFifo = &Tx_Buf[minor];
canmsg_t __user *addr;
canmsg_t tx;
unsigned long flags = 0;  /* still needed for local_irq_save() ? */
int written         = 0;
int blocking;
unsigned long _cnt;
int rxfifoindex; 


    DBGin();
    /* spin_lock_irqsave(&write_splock[minor], flags ); */
#ifdef DEBUG_COUNTER
    Cnt1[minor] = Cnt1[minor] + 1;
#endif /* DEBUG_COUNTER */


    /* detect write mode */
    blocking = !(file->f_flags & O_NONBLOCK);

    DBGprint(DBG_DATA,(" -- write %d msg, blocking=%d", (int)count, blocking));
    /* printk("w[%d/%d]", minor, TxFifo->active); */
    addr = (canmsg_t __user *)buffer;
#if 0
pr_info("ID 0x%lx \n", addr->id);
{

    int i;
    printk("buffer %p, addr %p, addrp %p\n", &buffer, &addr, addr);
    for( i = 0; i < 32; i++) {
	printk("%2x ", buffer[i]);
    }
    printk("\n");
}
#endif

    if(!access_ok(VERIFY_READ, buffer, count * sizeof(canmsg_t))) {
	written = -EINVAL;
	goto can_write_exit;
    }

    /* enter critical section */
    local_irq_save(flags);

    while( written < count ) {

	if(virtual != 0) {
	/* virtual CAN write, put the frame in all RX queues only */
	int rx_fifo;
	msg_fifo_t   *RxFifo; 
	int16_t myindex = 
		(int16_t)((struct _instance_data *)(file->private_data))->rx_index;
	struct timeval  timestamp;

	DBGprint(DBG_DATA,(" -- write msg %d, virtual, blocking=%d, size=%d",
		    (int)written, blocking, (int)sizeof(canmsg_t)));

	/* depending on the number of open processes
	 * the TX data has to be copied in different
	 * RX FIFOs
	 */

	    /* get one message from the userspace buffer */
	    /* FIXME: with CANFD, does it make sense to copy only the number
	     * of data bytes specified in the length field of canmsg_t ?
	     * Instead of  sizeof(canmsg_t) it is something like
	     * sizeof(canmsg_t) - CAN_MSG_LENGTH  + length 
	     */
	    __lddk_copy_from_user(
		    (canmsg_t *) &tx, 
		    (canmsg_t __user *) &addr[written],
		    sizeof(canmsg_t) );

	/* we are taking this as receive time stamp */
	get_timestamp(minor, &timestamp);

	for(rx_fifo = 0; rx_fifo < CAN_MAX_OPEN; rx_fifo++) {
	    /* for every rx fifo */
	    if (CanWaitFlag[minor][rx_fifo] == 1) {
		/* this FIFO is in use */
		/* printk(KERN_INFO "self copy to [%d][%d]\n", minor, rx_fifo); */
		    
		/*
		 * Don't copy the message in the receive queue
		 * of the process that sent the message unless
		 * this process requested selfreception.
		 */
		if ((myindex == rx_fifo) && 
		    (selfreception[minor][rx_fifo] == 0))
		{
		    /* printk("CAN[%d][%d] Don't copy message in my queue\n",
		     * minor, rx_fifo); */
		    continue;
		}
		// printk(
		// "CAN[%d][%d] Copy message to queue %d\n",
		 //        minor, myindex, rx_fifo);

		/* prepare buffer to be used */
		RxFifo = &Rx_Buf[minor][rx_fifo];

		RxFifo->data[RxFifo->head].flags = 0;
		memcpy(  
		    (void *)&RxFifo->data[RxFifo->head],
		    (void *)&tx,
		    sizeof(canmsg_t));
		/* Now copy the time stamp to the RX FIFO */
		RxFifo->data[RxFifo->head].timestamp.tv_sec  = timestamp.tv_sec;
		RxFifo->data[RxFifo->head].timestamp.tv_usec = timestamp.tv_usec;

		/* Set software overflow flag */
		if((RxFifo->status & BUF_OVERRUN) != 0) {
		    RxFifo->data[RxFifo->head].flags |= MSG_BOVR;
		}

		/* Mark message as 'self sent/received' */
		if ((myindex == rx_fifo) && 
		    (selfreception[minor][rx_fifo] != 0))
		{
		    RxFifo->data[RxFifo->head].flags |= MSG_SELF;
		}
		/* increment write index */
		RxFifo->status = BUF_OK;
		++(RxFifo->head);
		RxFifo->head %= MAX_BUFSIZE;

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

	}



	} else {
	    /* we have a real hardware to handle */

	/* Do we really need to protect something here ????
	 * e.g. in this case the TxFifo->free[TxFifo->head] value
	 *
	 * If YES, we have to use spinlocks for synchronization
	 */

/* - new Blocking code -- */

	if(blocking) {
	    if(wait_event_interruptible(CanOutWait[minor], \
		    TxFifo->free[TxFifo->head] != BUF_FULL)) {
		written = -ERESTARTSYS;
		goto can_write_exit;
	    }
	} else {
	    /* there are data to write to the network */
	    if(TxFifo->free[TxFifo->head] == BUF_FULL) {
		/* but there is already one message at this place */;
		/* write buffer full in non-blocking mode, leave write() */
		goto can_write_exit;
	    }
	}

/* ---- */

	/*
	 * To know which process sent the message we need an index.
	 * This is used in the TX IRQ to decide in which receive queue
	 * this message has to be copied (selfreception)
	 */
	rxfifoindex = ((struct _instance_data *)&(file->private_data))->rx_index;
	put_user( rxfifoindex,  &addr[written].cob);

	if( TxFifo->active ) {
	    /* more than one data and actual data in queue,
	     * add this message to the TX queue 
	     */
	    __lddk_copy_from_user(	/* copy one message to FIFO */
		    (canmsg_t *) &(TxFifo->data[TxFifo->head]), 
		    (canmsg_t __user *) &addr[written],
		    sizeof(canmsg_t) );
	    TxFifo->free[TxFifo->head] = BUF_FULL; /* now this entry is FULL */
	    /* TxFifo->head = ++(TxFifo->head) % MAX_BUFSIZE; */
	    ++TxFifo->head;
	    (TxFifo->head) %= MAX_BUFSIZE;

	} else {
	    /* copy message into local canmsg_t structure */
	    __lddk_copy_from_user(
		    (canmsg_t *) &tx, 
		    (canmsg_t __user *) &addr[written],
		    sizeof(canmsg_t) );
	    /* f - fast -- use interrupts */
	    if( count >= 1 ) {
	        /* !!! CHIP abh. !!! */
	        TxFifo->active = 1;
	    }
	    /* write CAN msg data to the chip and enable the tx interrupt */
	    CAN_SendMessage( minor, &tx);  /* Send, no wait */
	}	/* TxFifo->active */
    }
        written++;
    }

can_write_exit:

    local_irq_restore(flags);

    /* spin_unlock_irqrestore(&write_splock[minor], flags); */
    DBGout();
    return written;
}

