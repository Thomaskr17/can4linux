/*
 * imx35flexcan.h - can4linux CAN driver module
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (c) 2010 port GmbH Halle/Saale
 *------------------------------------------------------------------
 * $Header: $
 *
 *--------------------------------------------------------------------------
 * ERRATAs:
 * As of 20100601
 * ENGcm09158
 * FlexCAN: Glitch filter is not implemented
 *         does not influence can4linux
 *
 *
 * modification history
 * --------------------
 * $Log: $
 *
 *--------------------------------------------------------------------------
 */

/*
One reason is that you have to be
a bit careful with the CAN buffers. When they are read they become
blocked until a command is issued to let them be used again (maybe
not all but atleast the last one read) - there are details about this
in the users' manual.
 
For my work I added a CAN memory dump routine which can be called
via the serial interface or via TELNET on the M52235, which has
the same CAN controller). The routine copies the contents, frees
the buffers and then displays the values. This was adequate for
all debugging needs I had.
 
 
 















*/
#ifndef __FLEX_CAN_H
#define __FLEX_CAN_H

#include <mach/hardware.h>

/* Ka-Ro TX28 linux 2.6.38 */
#if defined(IMX28) 
#include <mach/mx28.h>
#endif

/* define some types, header file comes from CANopen */
#define UNSIGNED8 u8
#define UNSIGNED16 u16

/* can4linux does not use all the full CAN features, partly because it doesn't
   make sense.
 */

/* We use only one transmit object for all messages to be transmitted */
#define TRANSMIT_OBJ 		16
#define RECEIVE_STD_OBJ		0	/* RXFIFO, the first message box */
#define RECEIVE_EXT_OBJ 	18
#define RECEIVE_RTR_OBJ		19

#define FLEXCAN_MAX_FIFO_MB	8
#define FLEXCAN_MAX_FILTER	8
#define FLEXCAN_MAX_MB		64
#define FLEXCAN_MAX_MB_USED	(32-1)



/* Makros to manipulate FlexCAN control registers */
#define CAN_READ(reg)			(reg)

#define CAN_WRITE(reg, source)		((reg) = (source))

#define CAN_SET_BIT(reg, mask)		((reg) |= (mask))

#define CAN_RESET_BIT(reg, mask)	((reg) &= ~(mask))

#define CAN_TEST_BIT(breg, mask)	((breg) & (mask))


/*=== Register Layout of the FlexCAN module ==============================*/
typedef struct {
    volatile u32     canmcr;		/* module configuration */
    volatile u32     canctrl;		/* control register */
    volatile u16     timer;		/* free running timer */
    volatile u16     _0A;
    volatile u32     _0C;
    volatile u32     rxgmsk;		/* RX global mask         */
    volatile u32     rx14msk;		/* RX Buffer 14 Mask      */
    volatile u32     rx15msk;		/* RX Buffer 15 Mask      */
    volatile u16     canecr;		/* error counter register */
    volatile u32     estat;		/* error status register ESR */
    volatile u32     imask2;
    volatile u32     imask1;
    volatile u32     iflag2;
    volatile u32     iflag1;
} flexcan_t;


/* How much  is occupied by the imx35 FlexCAN module?
256 Byte ID mask storage and 1056 byte of message buffer storage
according Table 24-2 "FlexCAN Module Memory MAP - 0x97F
This is also valid for TX25, for each of the CAN channels
*/
#define CAN_RANGE 0x980


#define CAN_ModulConfigRegister		(flexcan->canmcr)
#define CAN_ControlReg			(flexcan->canctrl)
#define CAN_TimerRegister		(flexcan->timer)
#define CAN_ReceiveGlobalMask    	(flexcan->rxgmsk)
#define CAN_ReceiveBuffer14Mask		(flexcan->rx14msk)
#define CAN_ReceiveBuffer15Mask		(flexcan->rx15msk)
#define CAN_ErrorCounterRegister	(flexcan->canecr)
#define CAN_ErrorStatusRegister		(flexcan->estat)
#define CAN_InterruptMasks1		(flexcan->imask1)
#define CAN_InterruptFlags1		(flexcan->iflag1)
#define CAN_InterruptMasks2		(flexcan->imask2)
#define CAN_InterruptFlags2		(flexcan->iflag2)


/* Bit values of the FlexCAN module configuration register */

#define CAN_MCR_MDIS		0x80000000	/* MDIS Modul Disable bit */ 
#define CAN_MCR_FRZ		0x40000000	/* enable enter Freeze Mode */
#define CAN_MCR_FEN		0x20000000	/* FIFO enabled */
#define CAN_MCR_HALT		0x10000000
#define CAN_MCR_NOT_RDY		0x08000000
#define CAN_MCR_WAKE_MSK	0x04000000
#define CAN_MCR_SOFT_RST	0x02000000
#define CAN_MCR_FRZ_ACK		0x01000000
#define CAN_MCR_SUPV		0x00800000
#define CAN_MCR_SELF_WAKE	0x00400000
#define CAN_MCR_WRN_EN		0x00200000	/* Warn Interrupt enable */
#define CAN_MCR_LPM_ACK		0x00100000	/* Low Power Mode ack */
#define CAN_MCR_WAK_SRC		0x00080000
#define CAN_MCR_DOZE		0x00040000
#define CAN_MCR_SRX_DIS		0x00020000	/* Self reception enabled */
#define CAN_MCR_BCC		0x00010000	/* backwards compat. */

#define CAN_MCR_LPRIO_EN	0x00002000	/* Local Priority Enable */
#define CAN_MCR_AEN		0x00001000	/* Abort Enable */
#define CAN_MCR_MAX_IDAM	0x00000100
#define CAN_MCR_MAX_IDAM_MASK	0x00000300
#define CAN_MCR_MAX_MB_OFFSET 	0
#define CAN_MCR_MAX_MB_MASK 	(0x03F)


/* old definitions */
#define CAN_MCR_APS			0x0020
#define CAN_MCR_STOP_ACK		0x0010
#define CAN_MCR_IARB_MASK		0x000F
#define CAN_MCR_IARB3			0x0008
#define CAN_MCR_IARB2			0x0004
#define CAN_MCR_IARB1			0x0002
#define CAN_MCR_IARB0			0x0001

/* Bits of the FlexCAN control register 0 */

#define CAN_CTRL_BOFF_MSK_BIT		15
#define CAN_CTRL_ERR_MSK_BIT		14

/* Bit values of the FlexCAN control register (low)  */
#define CAN_CTRL_BOFF_MSK		0x00008000
#define CAN_CTRL_ENABLE_BOFF_INT	0x00008000
#define CAN_CTRL_DISABLE_BOFF_INT	0x00000000
#define CAN_CTRL_ERR_MSK		0x00004000
#define CAN_CTRL_ENABLE_ERR_INT		0x00004000
#define CAN_CTRL_DISABLE_ERR_INT	0x00000000
#define CAN_CTRL_CLK_SRC_BUS		0x00002000
#define CAN_CTRL_ENABLE_LOOP_BACK	0x00001000

#define CAN_CTRL_ENABLE_TWRN_INT	0x00000800
#define CAN_CTRL_ENABLE_RWRN_INT	0x00000400

#define CAN_CTRL_DISABLE_BOFF_RECOVERY	0x00000040

#define CAN_CTRL_SAMP_BIT		7	/* sampling mode */
#define CAN_CTRL_SMP			(1 << 7)
#define CAN_CTRL_TSYNC_BIT		5	/* sync timer */
#define CAN_CTRL_TSYNC			(1 << 5)
#define CAN_CTRL_LBUF_BIT		4	/* Lowest number buffer */
#define CAN_CTRL_LBUF			(1 << 4)
#define CAN_CTRL_LOM			(1 << 3) /* Listen onlx mode */

/* from Sascha Hauer, Pengutronix */
#define CANCTRL_PRESDIV(x)		(((x) & 0xff) << 24)
#define CANCTRL_RJW(x)			(((x) & 0x03) << 22)
#define CANCTRL_PSEG1(x)		(((x) & 0x07) << 19)
#define CANCTRL_PSEG2(x)		(((x) & 0x07) << 16)
#define CANCTRL_PROPSEG(x)		((x) & 0x07)


/* Bits of the FlexCAN error and status register */

#define CAN_ESTAT_BITERR1_BIT		15
#define CAN_ESTAT_BITERR0_BIT		14
#define CAN_ESTAT_ACK_ERR_BIT		13
#define CAN_ESTAT_CRC_ERR_BIT		12
#define CAN_ESTAT_FORM_ERR_BIT		11
#define CAN_ESTAT_STUFF_ERR_BIT		10
#define CAN_ESTAT_TX_WARN_BIT		9
#define CAN_ESTAT_RX_WARN_BIT		8
#define CAN_ESTAT_IDLE_BIT		7
#define CAN_ESTAT_TX_RX_BIT		6
#define CAN_ESTAT_FCS1_BIT		5
#define CAN_ESTAT_FCS0_BIT		4
#define CAN_ESTAT_BOFF_INT_BIT		2
#define CAN_ESTAT_ERR_INT_BIT		1
#define CAN_ESTAT_WAKE_INT_BIT		0

/* Bit values of the FlexCAN error and status register  ESR */

#define CAN_ESTAT_TWRN_INT		0x20000
#define CAN_ESTAT_RWRN_INT		0x10000
#define CAN_ESTAT_BITERR1		0x8000
#define CAN_ESTAT_BITERR0		0x4000
#define CAN_ESTAT_ACK_ERR		0x2000
#define CAN_ESTAT_CRC_ERR		0x1000
#define CAN_ESTAT_FORM_ERR		0x0800
#define CAN_ESTAT_STUFF_ERR		0x0400
#define CAN_ESTAT_TX_WARN		0x0200
#define CAN_ESTAT_RX_WARN		0x0100
#define CAN_ESTAT_IDLE			0x0080
#define CAN_ESTAT_TX_RX			0x0040
#define CAN_ESTAT_FCS 			0x0030
#define CAN_ESTAT_FCS0			0x0010	/* Error Passive */
#define CAN_ESTAT_FCS1			0x0020	/* Bus Off */
#define CAN_ESTAT_BOFF_INT		0x0004
#define CAN_ESTAT_ERR_INT		0x0002
#define CAN_ESTAT_WAKE_INT		0x0001

/*
 * Macros to handle CAN objects
 *
 * Structure for a single CAN object
 * A total of 64 such object structures exists (starting at CAN_BASE + 0x80)
 */
struct can_obj {
  volatile u32 ctl_status;	/* code + length, rtr, etc	*/
  volatile u32 id;		/* id and  prio	*/
  unsigned char msg[8];      	/* Message Data 0 .. 7   		*/
};

/* The firs data byte of a message */
#define MSG0 msg[0]

#define CAN_BFF_MASK		0x000007ff 	/* base frame format mask */ 
#define CAN_EEF_MASK		0x1fffffff	/* extended frame format mask */
#define CAN_RTR_BIT		0x00100000
#define CAN_EXTID_BIT		0x00600000	/* Always set SRR and IDE */
#define CAN_IDE_BIT		0x00200000	/* Always set SRR and IDE */

/* CAN object definition */
#define CAN_OBJ \
   ((struct can_obj volatile __iomem *) (((void __iomem *)can_base[minor]) + 0x80))

/*
 * the above definition can be used as follows:
 * CAN_OBJ[Msg].ctl_status = 0x5595;
 * val = CAN_OBJ[Msg].ctl_status;
 * with Msg 0....64
 */

/* ---------------------------------------------------------------------------
 * CAN_READ_OID(obj) is a macro to read the CAN-ID of the specified object.
 * It delivers the value as 16 bit from the standard ID registers.
 */

#define CAN_READ_OID(bChannel) ((CAN_OBJ[bChannel].id >> 18) & CAN_BFF_MASK)
#define CAN_READ_OID_AND_RTR(bChannel) (CAN_OBJ[bChannel].id)
#define CAN_READ_XOID(bChannel) (CAN_OBJ[bChannel].id & CAN_EFF_MASK)



/* ---------------------------------------------------------------------------
 * CAN_WRITE_OID(obj, id) is a macro to write the CAN-ID
 * of the specified object with identifier id.
 * CAN_WRITE_XOID(obj, id) is a macro to write the extended CAN-ID
 */
#define CAN_WRITE_OID(bChannel, Id) \
	do { \
	CAN_OBJ[bChannel].id = ((Id & CAN_BFF_MASK) << 18); \
	CAN_OBJ[bChannel].ctl_status &= ~(CAN_EXTID_BIT + CAN_RTR_BIT); \
	} while(0);

/* Set the IDE Bit in Control/Status */
#define CAN_WRITE_XOID(bChannel, Id)  \
	do { \
	(CAN_OBJ[bChannel].id = (Id & CAN_EFF_MASK));  \
	(CAN_OBJ[bChannel].ctl_status &= ~(CAN_RTR_BIT)); \
	(CAN_OBJ[bChannel].ctl_status |= CAN_EXTID_BIT); \
	} while(0);

/* ---------------------------------------------------------------------------
 * CAN_WRITE_OID_RTR(obj, id) is a macro to write the CAN-ID
 * of the specified object with identifier id and set the RTR Bit.
 */
#define CAN_WRITE_OID_RTR(bChannel, Id) \
	do { \
	u32 reg; \
	CAN_OBJ[bChannel].id = ((Id & CAN_BFF_MASK) << 18); \
	reg = CAN_OBJ[bChannel].ctl_status & (0x00F00000); \
	reg |= CAN_RTR_BIT; \
	reg &= ~(CAN_EXTID_BIT); \
	CAN_OBJ[bChannel].ctl_status = reg; \
	} while(0);


#define CAN_WRITE_XOID_RTR(bChannel, Id)  \
	do { \
	u32 reg; \
	(CAN_OBJ[bChannel].id = (Id & CAN_EFF_MASK));  \
	reg = CAN_OBJ[bChannel].ctl_status & (0x00F00000); \
	reg |= CAN_EXTID_BIT + CAN_RTR_BIT; \
	CAN_OBJ[bChannel].ctl_status = reg; \
	} while(0);

/* ---------------------------------------------------------------------------
 * CAN_WRITE_CTRL(obj, code, length) is a macro to write to the 
 * specified objects control register
 *
 * Writes 4 byte, TIME STAMP is overwritten with 0.
 * Leave SRR, IDE, and RTR bit to be set by CAN_WRITE_?OID
 */
#define CAN_WRITE_CTRL(bChannel, code, length) \
	do { u32 reg; \
	reg =  CAN_OBJ[bChannel].ctl_status & (0x00F00000); \
	CAN_OBJ[bChannel].ctl_status = reg | ((code << 24) + (length << 16));\
	} while (0);

/* ---------------------------------------------------------------------------
 * CAN_READ_CTRL(obj) is a macro to read the CAN-Ctrl register
 *
 * Read 2 byte
 */
/* #define CAN_READ_CTRL(bChannel) (CAN_OBJ[bChannel].ctl_status >> 4) */
#define CAN_READ_CTRL(bChannel) (CAN_OBJ[bChannel].ctl_status)


/***** receive message object codes *************************************/
/* Message buffer is not active */
#define REC_CODE_NOT_ACTIVE	0
/* Message buffer is active and empty */
#define REC_CODE_EMPTY		4
/* Message buffer is full */
#define REC_CODE_FULL		2
/* second frame was received into a full buffer before CPU read the first */
#define REC_CODE_OVERRUN	6
/* message buffer is now being filled with a new receive frame.
 * This condition will be cleared within 20 cycles.
 */
#define REC_CODE_BUSY		1

/***** transmit message object codes ************************************/
/* Message buffer not ready for transmit */
#define TRANS_CODE_NOT_READY		8
/* Data frame to be transmitted once, unconditionally */
#define TRANS_CODE_TRANSMIT_ONCE	12
/* Remote frame to be transmitted once, and message buffer becomes
 * an RX message buffer for dadat frames;
 * RTR bit must be set
 */
#define TRANS_CODE_TRANSMIT_RTR_ONCE	12
/* Data frame to be transmitted only as a response to a remote frame, always */
#define TRANS_CODE_TRANSMIT_ONLY_RTR	10
/* Data frame to be transmitted once, unconditionally
 * and then only as a response to remote frame always
 */
#define TRANS_CODE_TRANSMIT_ONCE_RTR_ALWAYS	14




/***** message object configuration register ****************************/
#define CAN_MSG_Dir		    0x08
#define CAN_MSG_Xtd		    0x04

/* Definitions for Data direction */
#define CAN_Dir_TRANSMIT 			1
#define CAN_Dir_RECEIVE 			0

/***** B I T  --  T I M I N G  ******************************************/

/* use http://www.port.de/pages/misc/bittimings.php?lang=en */
/*-- !!! The FlexCAN need a special baud rate table with more ----
 *-- !!! parameters than many other                          ----
 *-- !!! If you use Set_Baudrate you must cast the table pointer 
 */
 
typedef struct {
	UNSIGNED16 rate;
	UNSIGNED8  presdiv;
	UNSIGNED8  propseg;
	UNSIGNED8  pseg1;
	UNSIGNED8  pseg2;
} BTR_TAB_FLEXCAN_T;

#ifndef CAN_SYSCLK
#  error Please specify an CAN_SYSCLK value
#endif

/* generated bit rate table by http://www.port.de/engl/canprod/sv_req_form.html */
/* bitrate table for 10 MHz */
#if CAN_SYSCLK == 10
    #define CAN_SJW			   0

    #define CAN_PRESDIV_10K		0x63
    #define CAN_PROPSEG_10K		0x07
    #define CAN_PSEG1_10K		0x07
    #define CAN_PSEG2_10K		0x02

    #define CAN_PRESDIV_20K		0x31
    #define CAN_PROPSEG_20K		0x07
    #define CAN_PSEG1_20K		0x07
    #define CAN_PSEG2_20K		0x02

    #define CAN_PRESDIV_50K		0x13
    #define CAN_PROPSEG_50K		0x07
    #define CAN_PSEG1_50K		0x07
    #define CAN_PSEG2_50K		0x02

    #define CAN_PRESDIV_100K		0x09
    #define CAN_PROPSEG_100K		0x07
    #define CAN_PSEG1_100K		0x07
    #define CAN_PSEG2_100K		0x02

    #define CAN_PRESDIV_125K		0x07
    #define CAN_PROPSEG_125K		0x07
    #define CAN_PSEG1_125K		0x07
    #define CAN_PSEG2_125K		0x02
    
    #define CAN_PRESDIV_250K		0x03
    #define CAN_PROPSEG_250K		0x07
    #define CAN_PSEG1_250K		0x07
    #define CAN_PSEG2_250K		0x02

    #define CAN_PRESDIV_500K		0x01
    #define CAN_PROPSEG_500K		0x07
    #define CAN_PSEG1_500K		0x07
    #define CAN_PSEG2_500K		0x02

    /* bad samplepoint !!! */ 	
    #define CAN_PRESDIV_800K		0x00
    #define CAN_PROPSEG_800K		0x07
    #define CAN_PSEG1_800K		0x07
    #define CAN_PSEG2_800K		0x07
    
    #define CAN_PRESDIV_1000K		0x00
    #define CAN_PROPSEG_1000K		0x07
    #define CAN_PSEG1_1000K		0x07
    #define CAN_PSEG2_1000K		0x02
    
    #define CAN_SYSCLK_is_ok		1
#endif

/* wdh bitrate table for 20 MHz */
/* using same as for 10 MHz except for PRESDIV */
#if CAN_SYSCLK == 20000000
    #define CAN_SJW			   0

    #define CAN_PRESDIV_10K		(0x63*2+1)
    #define CAN_PROPSEG_10K		0x07
    #define CAN_PSEG1_10K		0x07
    #define CAN_PSEG2_10K		0x02

    #define CAN_PRESDIV_20K		(0x31*2+1)
    #define CAN_PROPSEG_20K		0x07
    #define CAN_PSEG1_20K		0x07
    #define CAN_PSEG2_20K		0x02

    #define CAN_PRESDIV_50K		(0x13*2+1)
    #define CAN_PROPSEG_50K		0x07
    #define CAN_PSEG1_50K		0x07
    #define CAN_PSEG2_50K		0x02

    #define CAN_PRESDIV_100K		(0x09*2+1)
    #define CAN_PROPSEG_100K		0x07
    #define CAN_PSEG1_100K		0x07
    #define CAN_PSEG2_100K		0x02

    #define CAN_PRESDIV_125K		(0x07*2+1)
    #define CAN_PROPSEG_125K		0x07
    #define CAN_PSEG1_125K		0x07
    #define CAN_PSEG2_125K		0x02
    
    #define CAN_PRESDIV_250K		(0x03*2+1)
    #define CAN_PROPSEG_250K		0x07
    #define CAN_PSEG1_250K		0x07
    #define CAN_PSEG2_250K		0x02

    #define CAN_PRESDIV_500K		(0x01*2+1)
    #define CAN_PROPSEG_500K		0x07
    #define CAN_PSEG1_500K		0x07
    #define CAN_PSEG2_500K		0x02

    /* bad samplepoint !!! */ 	
    #define CAN_PRESDIV_800K		(0x00*2+1)
    #define CAN_PROPSEG_800K		0x07
    #define CAN_PSEG1_800K		0x07
    #define CAN_PSEG2_800K		0x07
    
    #define CAN_PRESDIV_1000K		(0x00*2+1)
    #define CAN_PROPSEG_1000K		0x07
    #define CAN_PSEG1_1000K		0x07
    #define CAN_PSEG2_1000K		0x02
    
    #define CAN_SYSCLK_is_ok		1
#endif

/*
 * with Motorola FlexCAN, 24 MHz, sample point at 87.5% 
 * used with iMX35
 */ 
#if CAN_SYSCLK == 24000000
    #define CAN_SJW			   0

    #define CAN_PRESDIV_10K		(0x95)		/* not possible */
    #define CAN_PROPSEG_10K		0x04
    #define CAN_PSEG1_10K		0x07
    #define CAN_PSEG2_10K		0x01

    #define CAN_PRESDIV_20K		(0x4a)
    #define CAN_PROPSEG_20K		0x04
    #define CAN_PSEG1_20K		0x07
    #define CAN_PSEG2_20K		0x01

    #define CAN_PRESDIV_50K		(0x1d)
    #define CAN_PROPSEG_50K		0x04
    #define CAN_PSEG1_50K		0x07
    #define CAN_PSEG2_50K		0x01

    #define CAN_PRESDIV_100K		(0x0e)
    #define CAN_PROPSEG_100K		0x04
    #define CAN_PSEG1_100K		0x07
    #define CAN_PSEG2_100K		0x01

    #define CAN_PRESDIV_125K		(0x0b)
    #define CAN_PROPSEG_125K		0x04
    #define CAN_PSEG1_125K		0x07
    #define CAN_PSEG2_125K		0x01
    
    #define CAN_PRESDIV_250K		(0x05)
    #define CAN_PROPSEG_250K		0x04
    #define CAN_PSEG1_250K		0x07
    #define CAN_PSEG2_250K		0x01

    #define CAN_PRESDIV_500K		(2)
    #define CAN_PROPSEG_500K		0x04
    #define CAN_PSEG1_500K		0x07
    #define CAN_PSEG2_500K		0x01

    #define CAN_PRESDIV_800K		(1)
    #define CAN_PROPSEG_800K		0x03
    #define CAN_PSEG1_800K		0x07
    #define CAN_PSEG2_800K		0x01
    
    /* Be careful, the other values obtaint from the
       calculator: 1, 1, 7, 0  are not working */
    #define CAN_PRESDIV_1000K		(0)
    #define CAN_PROPSEG_1000K		0x07
    #define CAN_PSEG1_1000K		0x07
    #define CAN_PSEG2_1000K		0x06
    
    #define CAN_SYSCLK_is_ok		1
#endif

/*
 * with Motorola FlexCAN, 66.5 MHz, sample point at 87.5% 
 * used with iMX35
 */ 
#if CAN_SYSCLK == 66500000
    #define CAN_SJW			   0

    #define CAN_PRESDIV_10K		(0xae)		/* not possible */
    #define CAN_PROPSEG_10K		0x07
    #define CAN_PSEG1_10K		0x07
    #define CAN_PSEG2_10K		0x01

    #define CAN_PRESDIV_20K		(0xae)
    #define CAN_PROPSEG_20K		0x07
    #define CAN_PSEG1_20K		0x07
    #define CAN_PSEG2_20K		0x01

    #define CAN_PRESDIV_50K		(0x45)
    #define CAN_PROPSEG_50K		0x07
    #define CAN_PSEG1_50K		0x07
    #define CAN_PSEG2_50K		0x01

    #define CAN_PRESDIV_100K		(0x22)
    #define CAN_PROPSEG_100K		0x07
    #define CAN_PSEG1_100K		0x07
    #define CAN_PSEG2_100K		0x01

    #define CAN_PRESDIV_125K		(0x1b)
    #define CAN_PROPSEG_125K		0x07
    #define CAN_PSEG1_125K		0x07
    #define CAN_PSEG2_125K		0x01
    
    #define CAN_PRESDIV_250K		(0x0d)
    #define CAN_PROPSEG_250K		0x07
    #define CAN_PSEG1_250K		0x07
    #define CAN_PSEG2_250K		0x01

    #define CAN_PRESDIV_500K		(6)
    #define CAN_PROPSEG_500K		0x07
    #define CAN_PSEG1_500K		0x07
    #define CAN_PSEG2_500K		0x01

    #define CAN_PRESDIV_800K		(6)		/* not possible */
    #define CAN_PROPSEG_800K		0x07
    #define CAN_PSEG1_800K		0x07
    #define CAN_PSEG2_800K		0x01
    
    #define CAN_PRESDIV_1000K		(6)		/* not possible */
    #define CAN_PROPSEG_1000K		0x07
    #define CAN_PSEG1_1000K		0x07
    #define CAN_PSEG2_1000K		0x01
    
    #define CAN_SYSCLK_is_ok		1
#endif
/*
 * with Motorola FlexCAN, 32 MHz, sample point at 87.5% 
 * used with ColdFire M5282EVB
 */ 
#if CAN_SYSCLK == 32000000
    #define CAN_SJW			   0

    #define CAN_PRESDIV_10K		(199)
    #define CAN_PROPSEG_10K		0x04
    #define CAN_PSEG1_10K		0x07
    #define CAN_PSEG2_10K		0x01

    #define CAN_PRESDIV_20K		(99)
    #define CAN_PROPSEG_20K		0x04
    #define CAN_PSEG1_20K		0x07
    #define CAN_PSEG2_20K		0x01

    #define CAN_PRESDIV_50K		(39)
    #define CAN_PROPSEG_50K		0x04
    #define CAN_PSEG1_50K		0x07
    #define CAN_PSEG2_50K		0x01

    #define CAN_PRESDIV_100K		(19)
    #define CAN_PROPSEG_100K		0x04
    #define CAN_PSEG1_100K		0x07
    #define CAN_PSEG2_100K		0x01

    #define CAN_PRESDIV_125K		(15)
    #define CAN_PROPSEG_125K		0x04
    #define CAN_PSEG1_125K		0x07
    #define CAN_PSEG2_125K		0x01
    
    #define CAN_PRESDIV_250K		(7)
    #define CAN_PROPSEG_250K		0x04
    #define CAN_PSEG1_250K		0x07
    #define CAN_PSEG2_250K		0x01

    #define CAN_PRESDIV_500K		(3)
    #define CAN_PROPSEG_500K		0x04
    #define CAN_PSEG1_500K		0x07
    #define CAN_PSEG2_500K		0x01

    #define CAN_PRESDIV_800K		(1)
    #define CAN_PROPSEG_800K		0x07
    #define CAN_PSEG1_800K		0x07
    #define CAN_PSEG2_800K		0x02
    
    #define CAN_PRESDIV_1000K		(1)
    #define CAN_PROPSEG_1000K		0x04
    #define CAN_PSEG1_1000K		0x07
    #define CAN_PSEG2_1000K		0x01
    
    #define CAN_SYSCLK_is_ok		1
#endif

/* wdh: bitrate table for 40 MHz */
/* using same as for 10 MHz except for PRESDIV */
#if CAN_SYSCLK == 40000000
    #define CAN_SJW			   0

    #define CAN_PRESDIV_10K		(0x63*4+3)
    #define CAN_PROPSEG_10K		0x07
    #define CAN_PSEG1_10K		0x07
    #define CAN_PSEG2_10K		0x02

    #define CAN_PRESDIV_20K		(0x31*4+3)
    #define CAN_PROPSEG_20K		0x07
    #define CAN_PSEG1_20K		0x07
    #define CAN_PSEG2_20K		0x02

    #define CAN_PRESDIV_50K		(0x13*4+3)
    #define CAN_PROPSEG_50K		0x07
    #define CAN_PSEG1_50K		0x07
    #define CAN_PSEG2_50K		0x02

    #define CAN_PRESDIV_100K		(0x09*4+3)
    #define CAN_PROPSEG_100K		0x07
    #define CAN_PSEG1_100K		0x07
    #define CAN_PSEG2_100K		0x02

    #define CAN_PRESDIV_125K		(0x07*4+3)
    #define CAN_PROPSEG_125K		0x07
    #define CAN_PSEG1_125K		0x07
    #define CAN_PSEG2_125K		0x02
    
    #define CAN_PRESDIV_250K		(0x03*4+3)
    #define CAN_PROPSEG_250K		0x07
    #define CAN_PSEG1_250K		0x07
    #define CAN_PSEG2_250K		0x02

    #define CAN_PRESDIV_500K		(0x01*4+3)
    #define CAN_PROPSEG_500K		0x07
    #define CAN_PSEG1_500K		0x07
    #define CAN_PSEG2_500K		0x02

    /* bad samplepoint !!! */ 	
    #define CAN_PRESDIV_800K		(0x00*4+3)
    #define CAN_PROPSEG_800K		0x07
    #define CAN_PSEG1_800K		0x07
    #define CAN_PSEG2_800K		0x07
    
    #define CAN_PRESDIV_1000K		(0x00*4+3)
    #define CAN_PROPSEG_1000K		0x07
    #define CAN_PSEG1_1000K		0x07
    #define CAN_PSEG2_1000K		0x02
    
    #define CAN_SYSCLK_is_ok		1
#endif

#ifndef CAN_SYSCLK_is_ok
# error "Please specify a valid CAN_SYSCLK value"
# error "(i.e. 24000000, 66500000) or define new parameters"
#endif
/*==========================================================================*/
#endif		/* __FLEX_CAN_H  */


