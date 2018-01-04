/*
 * can_close - can4linux CAN driver module
 *
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 * 
 * Copyright (c) 2011 port GmbH Halle/Saale
 * (c) 2001 Heinz-Jürgen Oertel (oe@port.de)
 *          Claus Schroeter (clausi@chemie.fu-berlin.de)
 * derived from the the LDDK can4linux version
 *     (c) 1996,1997 Claus Schroeter (clausi@chemie.fu-berlin.de)
 *------------------------------------------------------------------
 * $Header: /z2/cvsroot/products/0530/software/can4linux/src/close.c,v 1.8 2009/06/03 14:29:53 oe Exp $
 *
 *--------------------------------------------------------------------------
 *
 *
 *
 *--------------------------------------------------------------------------
 */


/**
* \file close.c
* \author Heinz-Jürgen Oertel, port GmbH
* $Revision: 1.8 $
* $Date: 2009/06/03 14:29:53 $
*
*
*/
/*
*
*/
#include <linux/pci.h>
#include "defs.h"

#ifndef DOXYGEN_SHOULD_SKIP_THIS
#endif /* DOXYGEN_SHOULD_SKIP_THIS */

/***************************************************************************/
/**
*
* \brief int close(int fd);
* close a file descriptor
* \param fd The descriptor to close.
*
* \b close closes a file descriptor, so that it no longer
*  refers to any device and may be reused.
* \returns
* close returns zero on success, or -1 if an error occurred.
* \par ERRORS
*
* the following errors can occur
*
* \arg \c BADF \b fd isn't a valid open file descriptor 
*
*/

__LDDK_CLOSE_TYPE can_close ( __LDDK_CLOSE_PARAM )
{
unsigned int minor = iminor(inode);
int rx_fifo = ((struct _instance_data *)(file->private_data))->rx_index;

    DBGin();

    /* printk(KERN_INFO "--> closing minor %d  fifo %d \n", minor, rx_fifo); */

#ifdef CAN_USE_FILTER
    Can_FilterCleanup(minor);
#endif

    if(file->private_data) {
	kfree(file->private_data);
    }

    CanWaitFlag[minor][rx_fifo] = 0;
    selfreception[minor][rx_fifo] = 0;

    atomic_dec(&Can_isopen[minor]);		/* flag device as free */

    if(virtual != 0) {
	DBGout();
	return 0;
    }
    if(atomic_read(&Can_isopen[minor]) > 0) {
	DBGprint(DBG_BRANCH, ("leaving close() without shut down"));
	DBGout();
	return 0;
    }

    /*
     * all processes released the driver
     * now shut down the CAN controller
     */
    DBGprint(DBG_BRANCH, ("stop chip and release resources"));
    CAN_StopChip(minor);


#if !defined(PCM3680) && !defined(CPC_104) && !defined(CPC_PCM_104)
    /* call this before freeing any memory or io area.
     * this can contain registers needed by Can_FreeIrq()
     */
    /* printk(KERN_INFO "    Releasing IRQ %d\n", IRQ[minor]); */
    Can_FreeIrq(minor, IRQ[minor]);


    /* should the resources be released in a manufacturer specific file?
     * is it always depending on the hardware?
     */

#if defined(SSV_MCP2515) || defined(PCM9890) || defined(AuR_MCP2515) \
	|| defined(MCP2515SPI)
	;
#elif defined(CAN_PORT_IO) && !defined(KVASER_PCICAN)
    /* release_region(Base[minor], can_range[minor]); */

    /* printk("pci_release_region()\n"); */
    /*  code for CC_CANPCI */
    pci_release_region(Can_pcidev[minor], 1);   /* LED control */
    pci_release_region(Can_pcidev[minor], 2);	/* CAN-I/O */ 
#else
# if defined(CAN_INDEXED_PORT_IO)
    release_region(Base[minor], 2);
# else
#  ifndef CAN4LINUX_PCI
    /* This part is called for:
       Zedboard, ....
     */
    /* release I/O memory mapping -> release virtual memory */
    /* printk("iounmap %p \n", can_base[minor]); */
    iounmap(can_base[minor]);

    /* Release the memory region */
    /* printk("release mem %x \n", Base[minor]); */
    release_mem_region(Base[minor], can_range[minor]);

#  endif
# endif
#endif



#else  /* !defined(TARGETS with CAN_Release() in target.c */
    CAN_Release(minor);
#endif /* !defined(TARGETS with CAN_Release() in target.c */

    DBGout();
    return -EBADF;
}
