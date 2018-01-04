/* imx35funcs.c  - Motorola FlexCAN functions
 *
 * can4linux -- LINUX CAN device driver source
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 *
 * Copyright (c) 2003-2005 port GmbH Halle/Saale
 * (c) 2003-2013 Heinz-Jürgen Oertel (oe@port.de)
 *------------------------------------------------------------------
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
 *  	base, extended, base RTR, extended RTR frames
 *  RECEIVE_STD_OBJ - used to receice all messages in base frame format
 *  RECEIVE_EXT_OBJ - used to receice all messages in extended frame format
 *  RECEIVE_RTR_OBJ - what be nice to have, but this doesn't work
 *        the driver is not able to receive any RTR frames.
 *
 *
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include "defs.h"
#include <asm/delay.h>
#include <linux/sched.h>
/* ARM based i.MX  controllers */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
# error use this ARM  FLexCAN only with Kernel > 2.6
#endif
#if defined(__BIG_ENDIAN)
# error not tested on BIG_ENDIAn yet
#endif



#define flexcan_swab32(x)	\
	(((x) << 24) | ((x) >> 24) |\
		(((x) & (__u32)0x0000ff00UL) << 8) |\
		(((x) & (__u32)0x00ff0000UL) >> 8))

static inline void flexcan_memcpy(void *dst, void *src, int len)
{
	int i;
	unsigned int *d = (unsigned int *)dst, *s = (unsigned int *)src;
	len = (len + 3) >> 2;
	for (i = 0; i < len; i++, s++, d++)
		*d = flexcan_swab32(*s);
}

/* timing values */
static const BTR_TAB_FLEXCAN_T can_btr_tab_toucan[] = {
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
int CAN_ShowStat (int minor)
{
#if 0
    if (dbgMask && (dbgMask & DBG_DATA)) {
	pr_info(" MCR 0x%x,", CANinw(minor, canmcr));
	pr_info(" ESTAT 0x%x,", CANinw(minor, estat));
	pr_info(" IFLAGS 0x%x,", CANinw(minor, iflag));
	pr_info(" IMASK 0x%x,", CANinw(minor, imask));
	pr_info("\n");
    }
#endif
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
	struct file *file,
	CanStatusPar_t *stat
	)
{

unsigned int minor = iminor(inode);	
msg_fifo_t *Fifo;
unsigned long flags;
int rx_fifo = ((struct _instance_data *)(file->private_data))->rx_index;
u32 reg;


    stat->type = CAN_TYPE_FlexCAN;

    stat->baud = Baud[minor];
    /* pr_info(" STAT ST 0x%x\n", CANin(minor, canstat)); */
    stat->status = CANinl(minor, estat);
    /* not available in the FlexCAN, lets fill 127 here */
    stat->error_warning_limit = 96;
    reg = CANinl(minor, canecr);
    stat->rx_errors = (reg & 0xff00) >> 8;
    stat->tx_errors = (reg & 0x00ff);
    /* error code is not available, use estat again */
    stat->error_code= CANinl(minor, estat);

    /* Collect information about the RX and TX buffer usage */
    /* Disable CAN Interrupts */
    /* !!!!!!!!!!!!!!!!!!!!! */
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
    local_irq_restore(flags);
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
int CAN_ChipReset (int minor)
{
int i;
volatile u32 reg;

    DBGin();

     /* 
        assuming the resource allocation is done in the platform code */

    /* Disable the FlexCAN module */
    /*
     * go to INIT mode
     * Any configuration change/initialization requires that the FlexCAN be
     * frozen by either asserting the HALT bit in the
     * module configuration register or by reset.
     * For Init_CAN() we choose reset.
     */
    CANsetl(minor, canmcr, CAN_MCR_MDIS + CAN_MCR_SOFT_RST); /* MDIS */
    udelay(12);
    reg = CANinl(minor, canmcr);
    reg &= ~CAN_MCR_MAX_MB_MASK;		/* clear max message buffers */
    reg |= FLEXCAN_MAX_MB_USED		/* set max message buffers used */
    	  + CAN_MCR_BCC			/* switch comatibility off */
          + CAN_MCR_SUPV		/* only Supervisor access */
          + CAN_MCR_FEN;		/* FIFO Enable */
    CANoutl(minor, canmcr, reg); 
    reg = CANinl(minor, canmcr);

    /* set all bits to there reset defaults
    * but clockmode to use bus clock */
#if defined(CAN_IS_USING_BUS_CLOCK)
    CANoutl(minor, canctrl, 0 + CAN_CTRL_CLK_SRC_BUS);
#else
    CANoutl(minor, canctrl, 0 );
#endif

    CAN_SetTiming(minor, Baud[minor]);

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
     
    CANresetl(minor, canctrl, CAN_CTRL_LBUF);
    CANsetl(minor, canctrl, CAN_CTRL_LBUF);
    CANsetl(minor, canctrl, CAN_CTRL_DISABLE_BOFF_RECOVERY);

    /*
     * Initialize message buffers.
     * The control/status word of all message buffers are written
     * as an inactive receive message buffer.
     */
    for(i = 0; i < FLEXCAN_MAX_MB; i++) {
	CAN_WRITE_CTRL(i, REC_CODE_NOT_ACTIVE, 0);
    }

    /* CAN_register_dump(); */
    DBGout();
    return 0;
}

/*
 * Configures bit timing registers directly. Chip must be in configuration mode.
 */
int CAN_SetBTR (int minor, int btr0, int btr1)
{
    DBGin();
    DBGprint(DBG_DATA, ("[%d] btr0=%d, btr1=%d", minor, btr0, btr1));


    /* ToDO for FlexCAN */

    DBGout();
    return 0;
}

/* change the bit timings of the selected CAN channel */
int CAN_SetTiming (int minor, int baud)
{
u32 reg;
BTR_TAB_FLEXCAN_T * table = (BTR_TAB_FLEXCAN_T*)can_btr_tab_toucan;

    DBGin();
    DBGprint(DBG_DATA, ("baud[%d]=%d", minor, baud));
    /* enable changing of bit timings */
    /* Disable the FlexCAN module */
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
     * in control register CTRL.
     *
     * The imx35 FlexCAN module uses one 4 byte register
     * the bit timing parameters required by the CAN protocol.
     * The high part contains the the prescaler divide register (PRESDIV)
     * RJW, PSEG1 and PSEG2 which allow the user to configure
     * the bit timing parameters.
     * The prescaler divide register (PRESDIV) allows the user to select
     * the ratio used to derive the S-Clock from the system clock.
     * The time quanta clock operates at the S-clock frequency.
     */

    reg = CANinl(minor, canctrl);
    /* reset all relevant bits */
    reg &=   ~(CAN_CTRL_SMP | CANCTRL_PRESDIV(0xff)
           | CANCTRL_PSEG1(7)| CANCTRL_PSEG2(7));
    /* now use correct bit timing values */
    reg |=   CANCTRL_PRESDIV(table->presdiv)
           | CANCTRL_PSEG1(table->pseg1)
           | CANCTRL_PSEG2(table->pseg2)
           | CANCTRL_RJW(0)
           | CANCTRL_PROPSEG(table->propseg);

    CANoutl(minor, canctrl, reg);
    /*
     * Stay in configuration mode; a call to Start-CAN() is necessary to
     * activate the CAN controller with the new bit rate
     */
    DBGout();
    return 0;
}


int CAN_StartChip (int minor)
{
u32 reg;

    RxErr[minor] = TxErr[minor] = 0L;
    DBGin();

    /* 
     * go to INIT mode
     * Any configuration change/initialization requires that the FlexCAN be
     * frozen by either asserting the HALT bit in the
     * module configuration register or by reset.
     * For Init_CAN() we choose reset.
     */
    CANoutl(minor, canmcr, /* CAN_MCR_MDIS + */ CAN_MCR_SOFT_RST); /* MDIS */
    udelay(20);
    reg =  CANinl(minor, canmcr);
    if (reg & CAN_MCR_SOFT_RST) {
       pr_err("Failed to softreset FlexCAN module (mcr=0x%08x)\n", reg);
               return -ENODEV;
    }

    /* CLK_SRC:  The clock source must be selected
       while the module is in disable mode.
       After the clock source is selected and the module is enabled
       (MDIS bit cleared), FlexCAN automatically goes to freeze mode.
       The clock source bit (CLK_SRC) in the CTRL register
       defines whether the internal clock is connected
       to the output of a crystal oscillator (oscillator clock)
       or to the peripheral clock (generally from a PLL).
       In order to guarantee reliable operation,
       the clock source must be selected
       while the module is in disable mode (bit MDIS set in the MCR).

       The crystal oscillator clock must be selected
       whenever a tight tolerance (up to 0.1%) is required
       in the CAN bus timing.
       The crystal oscillator clock has better jitter performance
       than PLL-generated clocks.

	Bit 13 in CTRL:
	This bit selects the clock source to the CAN protocol interface (CPI)
	to be either the peripheral clock (driven by CLK_SRC the PLL)
	or the crystal oscillator clock.
	The selected clock is the one fed to the prescaler
	to generate the SCLK (SCLK).
	In order to guarantee reliable operation,
	this bit must only be changed while the module is in disable mode.
	See Section 24.4.8.4, "Protocol Timing," for more information.
        0 The CAN engine clock source is the oscillator clock (24.576 MHz)
        1 The CAN engine clock source is the bus clock (66.5 MHz)

        Set all bits to there reset defaults
        but clockmode to use bus clock or oscillator clock
      */


    /* this sets most bits to zero */
#if defined(CAN_IS_USING_BUS_CLOCK)
    CANoutl(minor, canctrl, 0 + CAN_CTRL_CLK_SRC_BUS);
#else
    CANoutl(minor, canctrl, 0);
#endif

    CAN_SetTiming(minor, Baud[minor]);

    reg =  CANinl(minor, canmcr);
    reg &= ~CAN_MCR_MAX_MB_MASK;		/* clear max message buffers */
    reg |= (FLEXCAN_MAX_MB_USED		/* set max message buffers used */
    	  | CAN_MCR_BCC			/* switch comatibility off */
          | CAN_MCR_SUPV		/* only Supervisor access */
          | CAN_MCR_FEN			/* FIFO Enable */
          | CAN_MCR_FRZ			/* Freeze enable */
          | CAN_MCR_HALT		/* set module in Freeze mode */
	  | CAN_MCR_FEN
          );
    CANoutl(minor, canmcr, reg); 

#if 0
    /* CANsetl(minor, canmcr, CAN_MCR_FEN); */
    pr_info("MCR Reg 0x%08x\n", reg);
    CAN_register_dump(minor);
#endif
    /* clear interrupts */
    /* IFLAG[12]
       This register defines the flags for 32 message buffer interrupts.
       It contains one interrupt flag bit per buffer.
	Each successful transmission or reception
	sets the corresponding IFLAG2 bit.
	If the corresponding IMASK? bit is set, an interrupt is generated.
	The interrupt flag must be cleared by writing a 1;
	writing 0 has no effect. */
    CANoutl(minor, iflag1, 0xffffffff);	/* overwrites with '1' */
    CANoutl(minor, iflag2, 0xffffffff);	/* overwrites with '1' */

    /* ESTAT:
      Most bits in this register are read-only,
      except TWRN_INT, RWRN_INT, BOFF_INT, WAK_INT and ERR_INT,
      which are interrupt flags
      that can be cleared by writing 1 to them (writing `0' has no effect). */
    CANoutl(minor, estat, 0xffffffff); 	

    /* Set RX FiFO mode */
    CANsetl(minor, canmcr, CAN_MCR_FEN);
    /* For the RX Fifo use Format A:
    	One full ID (base or extended frame format) per filter element. */
    CANresetl(minor, canmcr, CAN_MCR_MAX_IDAM_MASK);

    /* CAN_register_dump(minor); */

    /* set all filters */
    /* for now, don't filter at all */
    __raw_writel( 0x00000000, can_base[minor] + (0xE0     ));
    __raw_writel( 0x00000000, can_base[minor] + (0xE0 +  4));
    __raw_writel( 0x00000000, can_base[minor] + (0xE0 +  8));
    __raw_writel( 0x00000000, can_base[minor] + (0xE0 + 12));
    __raw_writel( 0x00000000, can_base[minor] + (0xE0 + 16));
    __raw_writel( 0x00000000, can_base[minor] + (0xE0 + 20));
    __raw_writel( 0x00000000, can_base[minor] + (0xE0 + 24));
    __raw_writel( 0x00000000, can_base[minor] + (0xE0 + 28));

    memset_io(can_base[minor] + 0x0E0, 0,
    	sizeof(unsigned int) * FLEXCAN_MAX_FILTER);


    /* set the individual mask registers for the FIFO entries */
    /* 24.3.2.13 Rx Individual Mask Registers (RXIMR0­RXIMR63)
       These registers are used as acceptance masks for ID filtering
       in Rx message buffers and the FIFO. If the FIFO is not
       enabled, one mask register is provided for each available
       message buffer, providing ID masking capability on a per
       message buffer basis. When the FIFO is enabled (FEN bit
       in the MCR is set), the first 8 mask registers apply to
       the 8 elements of the FIFO filter table (on a one-to-one
       correspondence), while the rest of the registers apply to
       the regular message buffers, starting from message buffer 8.
	..... Furthermore, they can only be accessed by the ARM
       while the module is in freeze mode. Outside of freeze mode,
	write accesses are blocked and read accesses return "all
	zeros". Furthermore, if the BCC bit in the MCR is cleared,
	any read or write operation to these registers results in
	an access error.
	0 the corresponding bit in the filter is "don't care"
	1 The corresponding bit in the filter is checked against
	  the one received
    */
    __raw_writel( 0x00000000, can_base[minor] + (0x0880 +  0));
    __raw_writel( 0x00000000, can_base[minor] + (0x0880 +  4));
    __raw_writel( 0x00000000, can_base[minor] + (0x0880 +  8));
    __raw_writel( 0x00000000, can_base[minor] + (0x0880 + 12));
    __raw_writel( 0x00000000, can_base[minor] + (0x0880 + 16));
    __raw_writel( 0x00000000, can_base[minor] + (0x0880 + 20));
    __raw_writel( 0x00000000, can_base[minor] + (0x0880 + 24));
    __raw_writel( 0x00000000, can_base[minor] + (0x0880 + 28));

    memset_io(can_base[minor] + 0x0880, 0,
    	sizeof(unsigned int) * FLEXCAN_MAX_MB);

    /* Interrupts on Rx, TX, any Status change and data overrun */
    /* Initialize the transmit and receive pin modes in control register 0 */



    /* enable Bus-Off Interrupt
     */
    CANsetl(minor, canctrl, CAN_CTRL_ENABLE_BOFF_INT);

    /* for driver test, Loop Back mode can be enabled here */
    CANsetl(minor, canctrl, CAN_CTRL_DISABLE_BOFF_RECOVERY 
    			    /* + CAN_CTRL_ENABLE_LOOP_BACK */
    			    );

    /*
     * - Set the required mask bits in the IMASK register (for all message
     *  buffer interrupts) in CANCTRL0 for bus off and error interrupts,
     *  and in CANMCR for WAKE interrupt.
     *
     * - And disable sefreception of the FlexCAN model.
     *  Selfreception is completely handled in software
     * - Disable Auto Bus-Off Recovery
     * instead inform the application about that event
     */
    /* dont't forget error int's ! */
    /* Enable Warning mask and disable self reception */
    CANsetl(minor, canmcr, CAN_MCR_WRN_EN + CAN_MCR_SRX_DIS);

    /* enable TX Warning Int
       enable RX Warning Int
       Both bits can be set only after CAN_MCR_WRN_EN is set in canmcr
     */
    CANsetl(minor, canctrl, CAN_CTRL_ENABLE_TWRN_INT );
    CANsetl(minor, canctrl, CAN_CTRL_ENABLE_RWRN_INT );


    /* Set interrupt mask for message buffers */
    CANoutl(minor, imask1, 
                (1 << 5)	/* three FIFO interrupts, frame received */
              + (1 << 6)  	/* FIFO nearly full */
              + (1 << 7)  	/* FIFO overflow */
              + (1 << TRANSMIT_OBJ));

    /* Start Chip now, reset MDIS, ... */
    CANresetl(minor, canmcr, CAN_MCR_MDIS | CAN_MCR_HALT | CAN_MCR_FRZ);
    erroractive[minor]	 = 1;  /* set error active true */

    DBGout();
    return 0;
}


/* Disable all CAN activities */
int CAN_StopChip (int minor)
{
    DBGin();
    CANsetl(minor, canmcr, CAN_MCR_HALT);
    /* disable all error interrupts */
    CANresetl(minor, canctrl,
    	  CAN_CTRL_ENABLE_BOFF_INT	/* disable Bus-Off Interrupt	*/
    	| CAN_CTRL_ENABLE_ERR_INT	/* disable Error Interrupt	*/
    	| CAN_CTRL_ENABLE_TWRN_INT	/* disable TX Warning Int */
    	| CAN_CTRL_ENABLE_RWRN_INT	/* disable RX Warning Int */
    	);
    /* disable all MB interrupts */
    CANoutl(minor, imask1, 0);
    DBGout();
    return 0;
}

/* set value of the output control register
 * The register is not available, nothing happens here 
 * besides printing some debug information
 */
int CAN_SetOMode (int minor, int arg)
{

    DBGin();
	DBGprint(DBG_ENTRY,("[%d] outc=0x%x, CAN_SetOMode() not supported",
		minor, arg));
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

    From the FlexCAN manual:
    LOM  bit 3 of the control register
    This bit configures FlexCAN to operate in listen-only mode.
    In this mode, transmission is disabled, all error counters
    are frozen and the module operates in a CAN error passive mode [Ref. 1].
    Only messages acknowledged by another CAN station are received.
    If FlexCAN detects a message that has not been acknowledged,
    it flags a BIT0 error (without changing the REC),
    as if it was trying to acknowledge the message.
    0 Listen-only mode is deactivated
    1 FlexCAN module operates in listen-only mode

*/
int CAN_SetListenOnlyMode (int minor,
	int arg)	/* true - set Listen Only, false - reset */
{
    DBGin();
    if (arg) {
	CANsetl(minor, canctrl, CAN_CTRL_LOM);
    } else {
	CANresetl(minor, canctrl, CAN_CTRL_LOM );
    }
    DBGout();
    return 0;
}

/* The new FlexCAN in FIFO mode knows a 'mask' value and 'code' 
   value
 */
int CAN_SetMask (int minor, int n, unsigned int code, unsigned int mask)
{
    DBGin();

    /* acceptance code
    ist stored in 8 4 bytes locations starting at base+0xe0 */
    writel( code, can_base[minor] + (0xE0 + (n * 4)));
    /* acceptance mask
    ist stored in 8 4 bytes locations starting at base+0x0880 */
    writel( mask, can_base[minor] + (0x880 + (n * 4)));

    /* put values back in global variables for sysctl */
    AccCode[n][minor] = code;
    AccMask[n][minor] = mask;

    DBGout();
    return 0;
}


int CAN_SendMessage (int minor, canmsg_t *tx /*, int isr */)
{
volatile u32 stat;
u32 ctl_status = 0;
u32 code = 0;

    DBGin();
    /*
    IDLE (bit 7) - Idle Status. The IDLE bit indicates, when there is activity
    on the CAN bus
    1 - The CAN bus is Idle

    TX/RX - Transmission/receive status.
    Indicates when the FlexCAN module is transmitting or receiving a message.
    TX/RX has no meaning, when IDLE = 1
	0 - FlexCAN is receiving when IDLE = 0
	1 - FlexCAN is transmitting when IDLE = 0


    Or check if TX mb is inactive 
    */
#if defined(OLD_MBOX_TEST)  /* old */ 
    while (
    	    ((stat = CANinl(minor, estat)) & (CAN_ESTAT_IDLE + CAN_ESTAT_TX_RX))
    	    == CAN_ESTAT_TX_RX
    	  ) {
	    cond_resched();
    }
#endif


#if defined(PROD_MBOX_TEST)
    do {

	/* Error and Status Register (HW_CAN_ESR)
	    25.6.8 Error and Status Register (HW_CAN_ESR) p.1586
TXRX This bit indicates if CAN is transmitting or receiving a message when the CAN bus is not in IDLE state.
IDLE This bit indicates when CAN bus is in IDLE state.

	*/
	pr_info("stat = 0x%08x\n", CANinl(minor, estat));
	stat = CANinl(minor, estat) & (CAN_ESTAT_IDLE + CAN_ESTAT_TX_RX);

	ctl_status = readl(&(CAN_OBJ[TRANSMIT_OBJ].ctl_status));
	code = (ctl_status >> 24) & 0x0F;
	pr_info("ctl_status: 0x%08x,  code 0x%01x\n", ctl_status, code);
	if (    stat == CAN_ESTAT_TX_RX
		/* if CAN is still in active/progress */
	     || (code != 0b1000 && code != 0b1001 && code != 0)) {
	        /* or 
		   0b1000 INACTIVE: MB does not participate in the arbitration process.
		   0b1001 ABORT: MB was configured as Tx and CPU aborted the transmission.
		    This code is only valid when AEN bit in MCR is asserted. MB does
		    not participate in the arbitration process.
		   0b0000  
		   */

		cond_resched();
		;
	} else {
	    break;
	}
    } while (1);

#endif

    /* DBGprint(DBG_DATA,( */
    		/* "CAN[%d]: tx.flags=%d tx.id=0x%lx tx.length=%d stat=0x%x", */
		/* minor, tx->flags, tx->id, tx->length, stat)); */

    tx->length %= 9;			/* limit CAN message length to 8 */

    /* Writing Control/Status word to hold TX Message Object inactive */
    CAN_WRITE_CTRL(TRANSMIT_OBJ, TRANS_CODE_NOT_READY, 1);

    /* fill the frame info and identifier fields , ID-Low and ID-High */
    if(tx->flags & MSG_EXT) {
    	/* use ID in extended message format */
	if( tx->flags & MSG_RTR) {
	    DBGprint(DBG_DATA, ("---> send rtr extended frame\n"));
	    CAN_WRITE_XOID_RTR(TRANSMIT_OBJ, tx->id);
	} else {
	    DBGprint(DBG_DATA, ("---> send data extended frame\n"));
	    CAN_WRITE_XOID(TRANSMIT_OBJ, tx->id);
	}
    } else {
	if( tx->flags & MSG_RTR) {
	    DBGprint(DBG_DATA, ("---> send rtr base frame\n"));
	    CAN_WRITE_OID_RTR(TRANSMIT_OBJ, tx->id);
	} else {
	    DBGprint(DBG_DATA, ("---> send data base frame\n"));
	    CAN_WRITE_OID(TRANSMIT_OBJ, tx->id);
	}
    }

    /* - fill data ---------------------------------------------------- */
    /* only two 4 byte access are necessarry
     * instead of a for() loop 
     * u32 data = be32_to_cpup((__be32 *)&tx->data[0]);
	flexcan_write(data, &regs->cantxfg[FLEXCAN_TX_BUF_ID].data[0]);
     * can be better
     */

    /* using 
	 flexcan_memcpy(void *dst, void *src, int len)
	*/
    /** FIXME sparse */
    flexcan_memcpy((void *)CAN_OBJ[TRANSMIT_OBJ].msg, tx->data, tx->length);


    /* Writing Control/Status word (active code, length) */
    CAN_WRITE_CTRL(TRANSMIT_OBJ, TRANS_CODE_TRANSMIT_ONCE, tx->length);

    /* - /end --------------------------------------------------------- */
    /* CAN_object_dump(minor, TRANSMIT_OBJ); */
    DBGout();return 0;
}


/* look if one of the receive message objects has something received 
 * is called by an ioctl() Call 
 * but not supported in this FlexCAN driver
 */
int CAN_GetMessage (int minor, canmsg_t *rx)
{
    DBGin();
	DBGprint(DBG_ENTRY, ("CAN_GetMessage() not implemented\n"));
    DBGout();
    return 0;
}


/* 
create an error message 
in the rx fifo of the receiving process
*/
static void fill_errorframe(int minor, int flags) {
int		rx_fifo;
int		head;
msg_fifo_t	*RxFifo;


	    for(rx_fifo = 0; rx_fifo < CAN_MAX_OPEN; rx_fifo++) {
		/* for every rx fifo */
		if (CanWaitFlag[minor][rx_fifo] == 1) {
		    /* this FIFO is in use */
		    RxFifo = &Rx_Buf[minor][rx_fifo]; /* prepare buffer to be used */
		    (RxFifo->data[RxFifo->head]).flags = flags;
		    (RxFifo->data[RxFifo->head]).id = CANDRIVERERROR;
		    (RxFifo->data[RxFifo->head]).length = 0;
		    RxFifo->status = BUF_OK;

		    /* handle fifo wrap around */
		    head = ++(RxFifo->head) % MAX_BUFSIZE;
		    RxFifo->head = head;
		    if(unlikely(RxFifo->head == RxFifo->tail)) {
			pr_info("CAN[%d][%d] Rx: FIFO overrun\n", minor, rx_fifo);
			RxFifo->status = BUF_OVERRUN;
		    } 
		    /* tell someone that there is a new error message */
		    wake_up_interruptible(&CanWait[minor][rx_fifo]); 
		}
	    } /* endo for-loop,  filling all rx fifos */
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
irqreturn_t CAN_Interrupt ( int irq, void *dev_id)
{
int		minor;
volatile unsigned int		estat;
volatile unsigned int 		irqsrc;

struct timeval  timestamp;

unsigned long	flags;
msg_fifo_t	*RxFifo;
msg_fifo_t	*TxFifo;
int		rx_fifo;

u32 errcnt;

#if CONFIG_TIME_MEASURE
    set_measure_pin();
#endif

    minor = *(int *)dev_id;
    /* pr_info("CAN - ISR ; minor = %d\n", *(int *)dev_id); */

    RxFifo = &Rx_Buf[minor][0];
    TxFifo = &Tx_Buf[minor];

    /* DBGprint(DBG_DATA, (" => got  IRQ[%d]\n", minor)); */
    /* pr_info(" => got  IRQ[%d]\n", minor); */

    /* Fill timestamp as first action. 
     * The timestamp is used with all kinds of message:
       - tx
       - rx
       - local error messages to the application
     * Getting a precises time takes a lot of time
     * (additional 7 µs of ISR time on ea reasonaable fast CPU 
     * if a time stamp is not needed, it can be switched of
     * by ioctl() */
    if (use_timestamp[minor]) {
	do_gettimeofday(&timestamp);
    } else {
	timestamp.tv_sec  = 0;
	timestamp.tv_usec = 0;
    }

    for(rx_fifo = 0; rx_fifo < CAN_MAX_OPEN; rx_fifo++) {
	RxFifo = &Rx_Buf[minor][rx_fifo];

	RxFifo->data[RxFifo->head].timestamp = timestamp;

	/* preset flags */
	(RxFifo->data[RxFifo->head]).flags =
        		(RxFifo->status & BUF_OVERRUN) ? MSG_BOVR : 0;
    }

    /* One of the following can actually occur:
    Int Reason
        BusOff				
        ErrInt - in this case:
	    bits 4-5 - ESTAT.FLT_CONF - Fault Confinment State tells us
	     00 - all ok, error active mode
	     01 - error passive mode
	     1x - Bus Off
	    bit 8 - RX warning level reached
	    bit 9 - Tx warning level reached
        WakeUp Interrupt

        At the moment we aren't interested in more information

	ESR: This register reflects various error conditions,
	some general status of the device and it is the source
	of four interrupts to the ARM.
	The reported error conditions are those
	that occurred since the last time the ARM read this register.
	The ARM read action clears bits.
	Bits are status bits.
	Most bits in this register are read-only,
	except TWRN_INT, RWRN_INT, BOFF_INT, WAK_INT and ERR_INT,
	which are interrupt flags that can be cleared by writing 1 to them
	(writing `0' has no effect).
    */
    /*
     loop as long as the CAN controller shows interrupts.
     check for message object interrupts.
    */
    while(1) {
	estat = CANinl(minor, estat); /* check for special interrupts */

/* pr_info("!"); */
	if (unlikely(estat &
	   (  CAN_ESTAT_BOFF_INT
	    | CAN_ESTAT_ERR_INT
	    | CAN_ESTAT_WAKE_INT
	    | CAN_ESTAT_TWRN_INT
	    | CAN_ESTAT_RWRN_INT
	      )))  {

	    int flags = 0;
	    /* DBGprint(DBG_DATA,
	    	(" => got ERR IRQ[%d]: estat 0x%08x\n", minor, estat)); */

	/* CAN_register_dump(minor); */

	    /* we have an error interrupt
	     * later on we can move this error handling at the end of the ISR
	     * to have better RX response times */
		/* 1111 */
	    /* pr_info(" error-1 status 0x%04x \n", estat); */
	    /* pr_info(" 0x%04x\n", estat); */
	    /* reset all error conditions, we have saved them in estat
	     * BusOff disables the Interrupt generation itself by resetting
	     * the mask in canctrl0.
	     * ErrInt - to clear this bit, first read it (already done)
	     * then write as a zero.
	     *
	     * we do reset all possible interrupts and look what we got later
	     * Interrupts are cleared by writing '1'  to them.
	     */
	/* pr_info("ESTAT 0x%08x\n", estat); */
	    /* CANoutl(minor, estat, 0); */
	    CANoutl(minor, estat, 0x030007);
	/* CAN_register_dump(minor); */

	    if(estat & (0x030004) ) {
	       /* only if it was one of the expected errors  */

		if (unlikely(estat & CAN_ESTAT_FCS1)) { 
		  /* FLT_CONF == Bus OFF */
		  flags = MSG_BUSOFF;
		}
#if CHECK_FCS
/* check the current status in
the fault confinment register.
BUT: be aware of the difference of the ERROR ACTIVE and PASSIVE
and the warning level.
The Interrupt is issued when the warning level reaches 96.
In this case, the controller is still in ERROR ACTIVE state.

Therefore for now I use reaching the Error warning level as
signalling the ERROR PASSIVE flag to the application.
*/
		if ((estat & CAN_ESTAT_FCS) == CAN_ESTAT_FCS0) {
		  /* FLT_CONF == Error Passive */
#else
		if(estat & (CAN_ESTAT_TWRN_INT | CAN_ESTAT_RWRN_INT)) {
		    /* rx or tx warning interrupt */

#endif
		  flags = MSG_PASSIVE;
		  erroractive[minor] = 0;
		}
		fill_errorframe(minor, flags);

	    } /* was an expected error */


	    /* Reset Interrupts at the beginning or the end */
	    /* CANresetl(minor, estat, */
	    /* (CAN_ESTAT_BOFF_INT | CAN_ESTAT_ERR_INT | CAN_ESTAT_WAKE_INT));*/
	    /* CANoutl(minor, estat, 0x30007); */

	} /* was one of the error interrupts */


        if((estat & CAN_ESTAT_FCS) == CAN_ESTAT_FCS0) {
	    /* CAN controller is still in error passive state */
	    erroractive[minor] = 0;
        } 

#if 0
    pr_info("erroractive %d, FCS 0x%04x, ",
    		erroractive[minor], (estat & CAN_ESTAT_FCS));
    errcnt = CANinl(minor, canecr);
    pr_info(" :%08x: rx err %d, tx err %d \n", errcnt,
	((errcnt >> 8)) & 0xff, (errcnt & 0xff));
#endif

#if CHECK_FCS
	if ((erroractive[minor] == 0) && !(estat & CAN_ESTAT_FCS)) {
#else
/* check the error counters */
    errcnt = CANinl(minor, canecr);
	if ((erroractive[minor] == 0)
	&& (((errcnt >> 8) & 0xff) < 96)
	&&  ((errcnt & 0xff) < 96)
	) {
#endif
	    /* can error status changed from passive to active */
	    /* pr_info("can error status changed from passive to active\n"); */
	    erroractive[minor] = 1;
	    fill_errorframe(minor, 0);
	}

	/* check for message interrupt */
	irqsrc = CANinl(minor, iflag1);
	/* pr_info("   0x%0x\n", irqsrc); */
	if (irqsrc == 0) break;     /* while (1) */

	/*
	 * If there is any IRQ, reset all of them, the IRQ sources are stored
	 * in the irqsrc variable for later evaluation.
	 * 5282 overwrites/clears with '1'
	 */
	/* CANoutl(minor, iflag1, irqsrc); */


    /*========== receive interrupt */
    /* FlexCAN is used in RX Fifo Mode */
    if( irqsrc & (
                (1 << 5)	/* three FIFO interrupts, frame received */
              | (1 << 6)  	/* FIFO nearly full */
              | (1 << 7) ) ) {  /* FIFO overflow */

	u32 ctl_status, id; 
	int length, ovr;
	int flags = 0;
	DBGprint(DBG_DATA, (" => got RX IRQ[%d]: 0x%08x\n", minor, irqsrc));

	if (unlikely(irqsrc & (1 << 6))) {
	    /* don't take care about this, no way to tell this event
	       the application */
	    pr_info ("CAN[%d] RX HW FIFO Nearly Full\n", minor);
	    CANoutl(minor, iflag1, (1 << 6));  /* reset fifo interrupt */
	    goto ResetRXInt;
	}

	ctl_status = readl(&(CAN_OBJ[0].ctl_status));
	if (unlikely(irqsrc & (1 << 7))) {
	    ovr = 1;
	    /* pr_err("FIFO Overrun\n"); */
	    id       = CANDRIVERERROR;
	    length   = 8;
	    flags    = MSG_OVR; 

	} else if (likely(irqsrc & (1 << 5))) {
	    /* Read out CAN data, one receive process will be there anyway
	       and we have to prepare it for every receive queue */
	    ovr	     = 0;
	    id       = readl(&(CAN_OBJ[0].id));
	    length   = (ctl_status >> 16) & 0x0f;
	    if(unlikely(ctl_status & CAN_RTR_BIT)) {
		flags |= MSG_RTR;
	    }
	    if( ctl_status & CAN_IDE_BIT ) {
		flags |= MSG_EXT;
		id &= CAN_EEF_MASK;
	    } else {
	    	id = (id >> 18) & CAN_BFF_MASK;
	    }

	} else {
	    goto ResetRXInt;
	}

	/* CAN_object_dump( 0, 0); */

 
/* ---------- fill frame data -------------------------------- */
    /* handle all subscribed rx fifos */

    for(rx_fifo = 0; rx_fifo < CAN_MAX_OPEN; rx_fifo++) {
	int head;
	/* for every rx fifo */
	if (CanWaitFlag[minor][rx_fifo] == 1) {
	    /* this FIFO is in use */
	    RxFifo = &Rx_Buf[minor][rx_fifo]; /* prepare buffer to be used */

	    (RxFifo->data[RxFifo->head]).timestamp = timestamp;
	    (RxFifo->data[RxFifo->head]).flags |= flags;
	    (RxFifo->data[RxFifo->head]).id = id;
	    /* put message length */
	    (RxFifo->data[RxFifo->head]).length = length;
	    /* copy data */
	    length %= 9;	/* limit count to 8 bytes */
    /** FIXME sparse */
	    /* void flexcan_memcpy(void *dst, void *src, int len) */
	    flexcan_memcpy(
		(void *)&(RxFifo->data[RxFifo->head]).data[0],
		(void *)CAN_OBJ[0].msg,
		length);

	    /* mark just written entry as OK and full */
	    RxFifo->status = BUF_OK;
	    /* Handle buffer wrap-around */
	    head = ++(RxFifo->head) % MAX_BUFSIZE;
	    RxFifo->head = head;
	    if(unlikely(RxFifo->head == RxFifo->tail)) {
		    pr_err("CAN[%d][%d] RX: SW FIFO overrun\n", minor, rx_fifo);
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
    }

	/* Clear the FIFO interrupt flag.
	   The act of clearing the interrupt
	   triggers the FIFO engine to replace the MB
	   with the next frame in the queue
	   and then issues another interrupt to the CPU.
	 */
	if (ovr) {
	    CANoutl(minor, iflag1, (1 << 7));  /* reset ovr interrupt */
	} else {
	    CANoutl(minor, iflag1, (1 << 5));  /* reset fifo rx interrupt */
	}
	/* reading the free running timer will unlock any message buffers */
	(void) CANinl(minor, timer);

    }
ResetRXInt:
	/* For testing the CAN FIFO Overflow function, enable the following
	   line with the udelay: slow down the driver */
	/* udelay(500); */

    /*========== transmit interrupt */
    if( irqsrc & (1 << TRANSMIT_OBJ)) {
	/* CAN frame successfully sent */
	DBGprint(DBG_DATA, (" => got TX IRQ[%d]: 0x%08x\n", minor, irqsrc));

#if 1
 /* PRODRIVE */
	/* Reset Interrupt pending at Transmit Object */
	CANsetl(minor, iflag1, (1 << TRANSMIT_OBJ));

#endif
	/* use time stamp sampled with last INT */
	last_Tx_object[minor].timestamp = timestamp;

	/* CAN_register_dump(); */
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
	} else {
	    /* pr_info("TX\n"); */
	}

        /* enter critical section */
	local_irq_save(flags);

	/* The TX message FIFO contains other CAN frames to be sent
	 * The next frame in the FIFO is copied into the last_Tx_object
	 * and directly into the hardware of the CAN controller
	 */
	memcpy(
		(void *)&last_Tx_object[minor],
		(void *)&TxFifo->data[TxFifo->tail],
		sizeof(canmsg_t));

	/* Writing Control/Status word to hold TX Message Object inactive */
	CAN_WRITE_CTRL(TRANSMIT_OBJ, TRANS_CODE_NOT_READY, 1);
	/* fill the frame info and identifier fields , ID-Low and ID-High */
	if( (TxFifo->data[TxFifo->tail]).flags & MSG_EXT ) {

	    /* use ID in extended message format */
	    if( (TxFifo->data[TxFifo->tail]).flags & MSG_RTR) {
		DBGprint(DBG_DATA, ("---> send rtr extended frame\n"));
		CAN_WRITE_XOID_RTR(TRANSMIT_OBJ,
		    	(TxFifo->data[TxFifo->tail]).id);
	    } else {
		DBGprint(DBG_DATA, ("---> send data extended frame\n"));
		CAN_WRITE_XOID(TRANSMIT_OBJ,
		    	(TxFifo->data[TxFifo->tail]).id);
	    }
	} else {
	    if( (TxFifo->data[TxFifo->tail]).flags & MSG_RTR) {
	    DBGprint(DBG_DATA, ("---> send rtr base frame\n"));
		CAN_WRITE_OID_RTR(TRANSMIT_OBJ,
		    	(TxFifo->data[TxFifo->tail]).id);
	    } else {
	    DBGprint(DBG_DATA, ("---> send data base frame\n"));
		CAN_WRITE_OID(TRANSMIT_OBJ,
		    	(TxFifo->data[TxFifo->tail]).id);
	    }
	}

	/* - fill data ---------------------------------------------------- */
	/* using 
	     flexcan_memcpy(void *dst, void *src, int len)
	    */
    /** FIXME sparse */
	flexcan_memcpy((void *)CAN_OBJ[TRANSMIT_OBJ].msg,
		(void *)&(TxFifo->data[TxFifo->tail]).data[0],
		(TxFifo->data[TxFifo->tail]).length);

	/* Writing Control/Status word (active code, length) */
	CAN_WRITE_CTRL(TRANSMIT_OBJ, TRANS_CODE_TRANSMIT_ONCE,
		(TxFifo->data[TxFifo->tail]).length);

	TxFifo->free[TxFifo->tail] = BUF_EMPTY; /* now this entry is EMPTY */
	{
	int tail;
	    tail = ++(RxFifo->head) % MAX_BUFSIZE;
	    RxFifo->tail = tail;
	}

	/* leave critical section */
	local_irq_restore(flags);

Tx_done:
	;
	/* Reset Interrupt pending at Transmit Object */
	/* CANsetl(minor, iflag1, (1 << TRANSMIT_OBJ)); */
    } /* END ===== transmit interrupt */

   }  /* END while(1), looping through interrupt sources */

    DBGprint(DBG_DATA, (" => leave IRQ[%d]\n", minor));
#if CONFIG_TIME_MEASURE
    reset_measure_pin();
#endif    
    return IRQ_RETVAL(IRQ_HANDLED);
}

#if 0
/* dump all FlexCAN module registers, use printk */
void CAN_register_dump(int minor)
{
volatile flexcan_t __iomem *flexcan = (volatile flexcan_t __iomem *)can_base[minor];
u32 reg;

    pr_info("Flex CAN register layout, size 0x%02x bytes\n", sizeof(flexcan_t));

#define  printregister(s, name) pr_info(s, &name , name)

    /* pr_info(" %p: 0x%x \n", tou_can, *(unsigned char *)tou_can); */
    /* pr_info(" %p: 0x%x \n", (unsigned char *)tou_can + 1, *(((unsigned char *)tou_can) + 1)); */

    pr_info("CAN%d ", minor);
    printregister
    ("ModulConfigRegister      %p: %08x\n", CAN_ModulConfigRegister);
    reg = readl(flexcan);
    pr_info("\t%s, Freeze %s, ",
    	reg & 0x08000000 ? "NOT_RDY":"RDY",
    	reg & 0x01000000 ? "ON" : "OFF"); 
    pr_info("Compat Mode (BCC) %s,",
    	reg & 0x10000 ? "OFF" : "ON");
    pr_info(" FIFO Mode (FEN) %s, IDAM = %d\n",
    	reg & 0x20000000 ? "ON" : "OFF",
    	(reg >> 8) & 0x03 
    	);

    pr_info("CAN%d ", minor);
    printregister
    ("ControlReg               %p: %08x\n", CAN_ControlReg);
    pr_info("\tClock: %s,  presdiv: %d, rjw: %d, "
           "propseg: %d, pseg1: %d, pseg2: %d\n",
	(CAN_ControlReg & CAN_CTRL_CLK_SRC_BUS) ? "Bus" : "Osc", 
    	(CAN_ControlReg) >> 24,
    	(CAN_ControlReg & 0x00c00000) >> 22,
    	(CAN_ControlReg & 0x00000007),
    	(CAN_ControlReg & 0x00380000) >> 19,
    	(CAN_ControlReg & 0x00070000) >> 16);
    printregister
    (" CAN_TimerRegister            %p: %08x\n", CAN_TimerRegister);
    printregister
    (" CAN_ReceiveGlobalMask        %p: %08x\n", CAN_ReceiveGlobalMask);
    printregister
    (" CAN_ReceiveBuffer14Mask      %p: %08x\n", CAN_ReceiveBuffer14Mask);
    printregister
    (" CAN_ReceiveBuffer15Mask      %p: %08x\n", CAN_ReceiveBuffer15Mask);
    printregister
    (" CAN_ErrorCounterReg          %p: %08x\n", CAN_ErrorCounterRegister);
    printregister
    (" CAN_ErrorStatusRegister      %p: %08x\n", CAN_ErrorStatusRegister);
    printregister
    (" CAN_InterruptMasks1          %p: %08x\n", CAN_InterruptMasks1);
    printregister
    (" CAN_InterruptFlags1          %p: %08x\n", CAN_InterruptFlags1);
    printregister
    (" CAN_InterruptMasks2          %p: %08x\n", CAN_InterruptMasks2);
    printregister
    (" CAN_InterruptFlags2          %p: %08x\n", CAN_InterruptFlags2);
}

/* dump the content of the selected message object (MB) to printk */
/* 4 long words, 4 bytes each */
void CAN_object_dump(int minor, int object)
{
unsigned int vh;
unsigned int vl;

volatile u32 *cpx =
		(unsigned int *)(can_base[minor] + 0x80 + (0x10 * object));

    pr_info("Flex CAN %d object %d", minor, object);
    pr_info(", at: %p: \n\t", cpx);
    for(vl = 0; vl < 4; vl++) {
	pr_info("%08x  ", readl(cpx + vl));
    }
    pr_info("\n");

    vl = CAN_OBJ[object].ctl_status;
    pr_info(" Ctrl/Status 0x%08x; CODE 0x%02x, l = %d \n",
    	vl, (vl >> 24) & 0xf, (vl >> 16) & 0x0f);
    vh = CAN_OBJ[object].id;
    if( vl & CAN_EXTID_BIT) { 
	pr_info(" ExtId  %d/0x%x\n", vh & CAN_EFF_MASK, vh & CAN_EFF_MASK);
    } else {
	pr_info(" StdId  %d/0x%x\n", (vh >> 18) & CAN_BFF_MASK,
		(vh >> 18) & CAN_BFF_MASK);
    }
}
#endif
