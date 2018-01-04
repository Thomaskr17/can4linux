/*
 * at9263.h - can4linux CAN driver module
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (c) 2007 port GmbH Halle/Saale
 *------------------------------------------------------------------
 * $Header: /z2/cvsroot/products/0530/software/can4linux/src/at9263.h,v 1.1 2008/11/23 12:06:36 oe Exp $
 *
 *--------------------------------------------------------------------------
 *
 *
 * modification history
 * --------------------
 *
 *
 *
 *--------------------------------------------------------------------------
 */

#ifndef __AT9263_CAN_H
#define __AT9263_CAN_H

extern  upointer_t Base[];


/* can4linux does not use all the full CAN features, partly because it doesn't
   make sense.
 */

/* We use only one transmit object for all messages to be transmitted */
#define TRANSMIT_OBJ		0
#define RECEIVE_STD_OBJ 	1
#define RECEIVE_EXT_OBJ 	4
#define RECEIVE_RTR_OBJ 	8
#define RECEIVE_EXT_RTR_OBJ 	12

#define CAN_LAST_OBJ 15		/* 16 Mailboxes available */


/* Makros to manipulate CAN control registers */
#define CAN_READ(reg)			(reg)
#define CAN_WRITE(reg, source)		((reg) = (source))
#define CAN_SET_BIT(reg, mask)		((reg) |= (mask))
#define CAN_RESET_BIT(reg, mask)	((reg) &= ~(mask))
#define CAN_TEST_BIT(breg, mask)	((breg) & (mask))

/*=== Register Layout of the ATMEL CAN module ==============================*/
/* access is two byte, each register is on a 4 byte boundary             */
typedef struct {
	volatile u32	mode;		/* mode register */
	volatile u32	ier;		/* interrupt enble register */
	volatile u32	idr		/* interrupt disable register */;
	volatile u32	imr		/* interrupt mask register */;
	volatile u32	sr;		/* status register */
	volatile u32	br;		/* baud rate register */
	volatile u32	tim;
	volatile u32	timestp;
	volatile u32	ecr;		/* error counter register */
	volatile u32	tcr;
	volatile u32	acr;
} at9263can_t;


/* We have different CAN structures for different CAN Controllers
 * like BF_CAN, TouCan, FlexCan, ATMEL CAN
 * to be used with -
 * - CANopen library without OS we use global pointers
 *   bf_can, tou_can, at_can, ...
 * - in can4linux we use the generic name canregs_t
 *   (this name is used also in the sja1000 driver)
 *
 * All basic register macros /can_def.h) are based on that name
 */

/* typedef bf_can_t canregs_t; */
typedef at9263can_t canregs_t;


/* Register access macros */
#define CAN_Mode			(at_can->mode)
#define CAN_InterruptEnableRegister	(at_can->ier)
#define CAN_InterruptDisableRegister	(at_can->idr)
#define CAN_InterruptMaskRegister	(at_can->imr)
#define CAN_StatusRegister		(at_can->sr)
#define CAN_BaudrateRegister		(at_can->br)
#define CAN_TimerRegister		(at_can->tim)
#define CAN_TimeStampRegister		(at_can->timestp)
#define CAN_ErrorCounterRegister	(at_can->ecr)
#define CAN_TransferCommandRegister	(at_can->tcr)
#define CAN_AbortCommandRegister	(at_can->acr)


/* CAN register mask definitions  */

/* CAN Mode Register CANREG_MR */
# define CANBIT_MR_DRPT		0x00000080ul
# define CANBIT_MR_TIMFRZ	0x00000040ul
# define CANBIT_MR_TTM		0x00000020ul
# define CANBIT_MR_TEOF		0x00000010ul
# define CANBIT_MR_OVL		0x00000008ul
# define CANBIT_MR_ABM		0x00000004ul
# define CANBIT_MR_LPM		0x00000002ul
# define CANBIT_MR_CANEN	0x00000001ul
# define CANBIT_MR_CANDIS	0x00000000ul


/* Interrupt Enable Register CANREG_IER	*/
/* Interrupt Disable Register CANREG_IDR */
/* Interrupt Mask Register CANREG_IMR */
# define CANBIT_IR_BERR		0x10000000ul
# define CANBIT_IR_FERR		0x08000000ul
# define CANBIT_IR_AERR		0x04000000ul
# define CANBIT_IR_SERR		0x02000000ul
# define CANBIT_IR_CERR		0x01000000ul
# define CANBIT_IR_TSTP		0x00800000ul
# define CANBIT_IR_TOVF		0x00400000ul
# define CANBIT_IR_WAKEUP	0x00200000ul
# define CANBIT_IR_SLEEP	0x00100000ul
# define CANBIT_IR_BOFF		0x00080000ul
# define CANBIT_IR_ERRP		0x00040000ul
# define CANBIT_IR_WARN		0x00020000ul
# define CANBIT_IR_ERRA		0x00010000ul
# define CANBIT_IR_MB(ch)	(1ul << ch)


/* CAN Status Register CANREG_SR */
# define CANBIT_SR_OVLSY	0x80000000ul
# define CANBIT_SR_TBSY		0x40000000ul
# define CANBIT_SR_RBSY		0x20000000ul
# define CANBIT_SR_RBSY		0x20000000ul

# define CANBIT_SR_BERR		0x10000000ul
# define CANBIT_SR_FERR		0x08000000ul
# define CANBIT_SR_AERR		0x04000000ul
# define CANBIT_SR_SERR		0x02000000ul
# define CANBIT_SR_CERR		0x01000000ul
# define CANBIT_SR_TSTP		0x00800000ul
# define CANBIT_SR_TOVF		0x00400000ul
# define CANBIT_SR_WAKEUP	0x00200000ul
# define CANBIT_SR_SLEEP	0x00100000ul
# define CANBIT_SR_BOFF		0x00080000ul
# define CANBIT_SR_ERRP		0x00040000ul
# define CANBIT_SR_WARN		0x00020000ul
# define CANBIT_SR_ERRA		0x00010000ul
# define CANBIT_SR_MB_MSK	0x0000FFFFul
# define CANBIT_SR_MB(ch)	(1ul << ch)

/* CAN Message Mode Register CAN_CH_MMR */
# define CANBIT_MMR_MOT_MSK		0x07000000ul
# define CANBIT_MMR_MOT_DISABLE		0x00000000ul
# define CANBIT_MMR_MOT_RX		0x01000000ul
# define CANBIT_MMR_MOT_RX_OVL		0x02000000ul
# define CANBIT_MMR_MOT_TX		0x03000000ul
# define CANBIT_MMR_MOT_RX_RTR		0x04000000ul
# define CANBIT_MMR_MOT_TX_RTR		0x05000000ul

# define CANBIT_MMR_PRIOR_MSK		0x000F0000ul
# define CANBIT_MMR_MTIMEMARK_MSK	0x0000FFFFul

/* CAN Message Status Register CAN_CH_MSR */
# define CANBIT_MSR_MMI		0x01000000ul
# define CANBIT_MSR_MRDY	0x00800000ul
# define CANBIT_MSR_MABT	0x00400000ul
# define CANBIT_MSR_MRTR	0x00100000ul
# define CANBIT_MSR_MDLC_MSK	0x000F0000ul
# define CANBIT_MSR_MDLC_POS	16

/* CAN Message Control Register CAN_CH_MCR */
# define CANBIT_MCR_MTCR	0x00800000ul	/* Mailbox Transfer Command   */
# define CANBIT_MCR_MACR	0x00400000ul	/* Mailbox Abort Request      */
# define CANBIT_MCR_MRTR	0x00100000ul	/* Mailbox RTR Request        */
# define CANBIT_MCR_MDLC_POS	16		/* DLC code bit position      */





#if 0
/* Mailbox acceptance mask registers */
typedef struct {
    u16 aml;		/* mailbox acceptance mask low		*/
    u16 dummy1;
    u16 amh;		/* mailbox acceptance mask high		*/
    u16 dummy2;
} mask_t;

#define CAN_MASK \
   ((mask_t volatile *) ((void *)(can_base[minor] + 0x100)) )

#endif

/*
 * Macros to handle CAN message objects
 *
 * Structure for a single CAN object
 * A total of 16 such object structures exists
 * (starting at AT91SAM9263_BASE_CAN + 0x200)
 * ( 0xfffac200 )
 */

struct can_obj {
    u32 mmr;		/* Mailbox Mode Register */
    u32 mam;		/* Mailbox Acceptance Register */
    u32 mid;		/* Mailbox ID Register */
    u32 mfid;		/* Mailbox Family ID Register */
    u32 msr;		/* Mailbox Status Register */
    u32 mdl;		/* Mailbox Data Low Register */
    u32 mdh;		/* Mailbox Data High Register */
    u32 mcr;		/* Mailbox Control Register */
};


/* #define CAN_ID_RTR_BIT	0x4000 */
#define CAN_ID_EXT_BIT	0x20000000ul
/* #define	CAN_AME		0x8000 */
/* Acceptance Mask Enable		*/ 


/* The firs data byte of a message */
#define MSG0 msg[0]
/* CAN object definition */
#define CAN_OBJ \
   ((struct can_obj volatile *) ((void *)(can_base[minor] + 0x200)) )

/* ---------------------------------------------------------------------------
 * CAN_READ_OID(obj) is a macro to read the CAN-ID of the specified object.
 * It delivers the value as 16 bit from the standard ID registers.
 */
#define CAN_READ_OID(bChannel) ((CAN_OBJ[bChannel].mid & 0x03fc0000) >> 18)

#define CAN_READ_XOID(bChannel) (CAN_OBJ[bChannel].mid & 0x1fffffff) 



/* ---------------------------------------------------------------------------
 * CAN_WRITE_OID(obj, id) is a macro to write the CAN-ID
 * of the specified object with identifier id.
 * CAN_WRITE_XOID(obj, id) is a macro to write the extended CAN-ID
 */
#define CAN_WRITE_OID(bChannel, Id) \
	    (CAN_OBJ[bChannel].mid = ((Id) << 18))


#define CAN_WRITE_XOID(bChannel, Id)  \
	    (CAN_OBJ[bChannel].mid = ((Id) + CAN_ID_EXT_BIT))

/* ---------------------------------------------------------------------------
 * CAN_WRITE_OID_RTR(obj, id) is a macro to write the CAN-ID
 * of the specified object with identifier id and set the RTR Bit.
 */
#define CAN_WRITE_OID_RTR(bChannel, Id) \

#define CAN_WRITE_XOID_RTR(bChannel, Id)  \


/* ---------------------------------------------------------------------------
 * CAN_WRITE_CTRL(obj, code, length) is a macro to write to the 
 * specified objects control register
 *
 * Writes 2 byte, TIME STAMP is overwritten with 0.
 */
#define CAN_WRITE_CTRL(bChannel, length) \
	(CAN_OBJ[bChannel].dlc = (length))


/*
 * CAN Bit Timing definitions
 */
/* Bittiming Table 
---------------------------------------------------------------------------*/
typedef struct {
	u16 rate;
	u32 btr;
} BTR_TAB_AT91_T;


enum can_state {active, passive, busoff};

/*
* Bittiming Table 
*
* e.g. AT91SAM7A3 the CONFIG_CAN_T_CLK is the CORECLK
*
*/

/* BRP .. 7bit Prescaler , SMP == 1, SYNC == 1 
 * PROP > 400ns (2x Receiver delay) not checked
 * (depends on your hardware)
 */


#define CALC_MR(BRP, PROP, PSEG1, PSEG2)   \
	( ((u32)0 << 24) /* SMP */| \
	  ((u32)(BRP - 1) << 16) |  \
	  ((u32)0 << 12) /* SYNC */ | \
	  ((u32)(PROP - 1) << 8) |  \
	  ((u32)(PSEG1 - 1) << 4) | \
	  ((u32)(PSEG2 - 1) << 0) )


#ifndef CAN_SYSCLK
#  error Please specify a valid CAN_SYSCLK value (i.e. 48) \
or define new parameters
#endif

    /* http://www.port.de/engl/canprod/sv_req_form.html
     *  Freescale TouCAN is used */
#if CAN_SYSCLK == 24
#   define CAN_BTR_20K		CALC_MR(60, 8, 8, 3)
#   define CAN_BTR_50K		CALC_MR(30, 6, 7, 2)
#   define CAN_BTR_100K		CALC_MR(15, 6, 7, 2)
#   define CAN_BTR_125K		CALC_MR(12, 6, 7, 2)
#   define CAN_BTR_250K		CALC_MR( 6, 6, 7, 2)
#   define CAN_BTR_500K		CALC_MR( 3, 6, 7, 2)
#   define CAN_BTR_800K		CALC_MR( 2, 6, 6, 1)
#   define CAN_BTR_1000K	CALC_MR( 3, 3, 3, 1) 
#   define CAN_SYSCLK_is_ok		1

#elif (CAN_SYSCLK == 48)
#   define CAN_BTR_20K		CALC_MR(120, 8, 8, 3)
#   define CAN_BTR_50K		CALC_MR( 60, 6, 7, 2)
#   define CAN_BTR_100K		CALC_MR( 30, 6, 7, 2)
#   define CAN_BTR_125K		CALC_MR( 24, 6, 7, 2)
#   define CAN_BTR_250K		CALC_MR( 12, 6, 7, 2)
#   define CAN_BTR_500K		CALC_MR(  6, 6, 7, 2)
#   define CAN_BTR_800K		CALC_MR(  4, 6, 6, 2)
#   define CAN_BTR_1000K	CALC_MR(  3, 6, 7, 2) 
#   define CAN_SYSCLK_is_ok		1
#
#elif (CAN_SYSCLK == 50)
#   define CAN_BTR_20K		CALC_MR(120, 8, 8, 3)
#   define CAN_BTR_50K		CALC_MR( 60, 6, 7, 2)
#   define CAN_BTR_100K		CALC_MR( 30, 6, 7, 2)
#   define CAN_BTR_125K		CALC_MR( 25, 6, 7, 2)
#   define CAN_BTR_250K		CALC_MR( 12, 6, 7, 2)
#   define CAN_BTR_500K		CALC_MR(  6, 6, 7, 2)
#   define CAN_BTR_800K		CALC_MR(  4, 6, 6, 2)
#   define CAN_BTR_1000K	CALC_MR(  3, 6, 7, 2) 
#   define CAN_SYSCLK_is_ok		1
#
#elif (CAN_SYSCLK == 100)
#   define CAN_BTR_20K		CALC_MR(250, 8, 8, 3)
#   define CAN_BTR_50K		CALC_MR(125, 6, 7, 2)
#   define CAN_BTR_100K		CALC_MR( 50, 8, 8, 3)
#   define CAN_BTR_125K		CALC_MR( 50, 6, 7, 2)
#   define CAN_BTR_250K		CALC_MR( 25, 6, 7, 2)
#   define CAN_BTR_500K		CALC_MR( 10, 8, 8, 3)
#   define CAN_BTR_800K		CALC_MR(  5, 8, 8, 8)
#   define CAN_BTR_1000K	CALC_MR(  5, 8, 8, 3) 
#   define CAN_SYSCLK_is_ok		1

#endif /* CONFIG_CAN_T_CLK == 24 */

#ifndef CAN_SYSCLK_is_ok
#  error Please specify a valid CAN_SYSCLK value (i.e. 48) or define new parameters
#endif


/* #define IODEBUG */

/* Access Macros for register access.
 * can4linux tries to use always (if possible)  the same syntax
 * CANin(), CANout(), CANset(), ....
 * this makes it easiert to use code from the embedded CANopen 
 * drivers
 */

/* using memory acces with readb(), writeb() */
/* #error  memory I/O access */
/* #define can_base Base */
#ifdef IODEBUG
#  define CANout(bd,adr,v)	\
	(printk("Cout: (%x)=%x\n", (u32)&((canregs_t *)can_base[bd])->adr, v), \
		writeb(v, (u32) &((canregs_t *)can_base[bd])->adr ))

#define CANset(bd,adr,m)     do	{\
	unsigned char v;	\
        v = (readb((void __iomem *) &((canregs_t *)can_base[bd])->adr)); \
	printk("CANset %x |= %x\n", (v), (m)); \
	writeb( v | (m) , (u32) &((canregs_t *)can_base[bd])->adr ); \
	} while (0)

#define CANreset(bd,adr,m)	do {\
	unsigned char v; \
        v = (readb((u32) &((canregs_t *)can_base[bd])->adr)); \
	printk("CANreset %x &= ~%x\n", (v), (m)); \
	writeb( v & ~(m) , (u32) &((canregs_t *)can_base[bd])->adr ); \
	} while (0)

#define CANoutl(bd,adr,v)	\
	(printk("Cout: (%x)=%lx\n", (u32)&((canregs_t *)can_base[bd])->adr, v),\
		(writel(v, (u32) &((canregs_t *)can_base[bd])->adr )))
#else
   /* Memory Byte access */
#define CANout(bd,adr,v)	\
		(writeb(v, (void __iomem *) &((canregs_t *)can_base[bd])->adr ))
#define CANset(bd,adr,m)	\
	writeb((readb((void __iomem *) &((canregs_t *)can_base[bd])->adr)) \
		| (m) , (void __iomem *) &((canregs_t *)can_base[bd])->adr )
#define CANreset(bd,adr,m)	\
	writeb((readb((void __iomem *) &((canregs_t *)can_base[bd])->adr)) \
		& ~(m), (void __iomem *) &((canregs_t *)can_base[bd])->adr )
#endif  /* IODEBUG */

#define CANin(bd,adr)		\
		(readb ((void __iomem *) &((canregs_t *)can_base[bd])->adr  ))
#define CANtest(bd,adr,m)	\
	(readb((void __iomem *) &((canregs_t *)can_base[bd])->adr  ) & (m) )

   /* Memory word access */
#define CANoutw(bd,adr,v)	\
		(writew((v), (u32) &((canregs_t *)can_base[bd])->adr ))


#define CANoutwd(bd,adr,v)	\
	(printk("Cout: (%x)=%x\n", (u32)&((canregs_t *)can_base[bd])->adr, v), \
		writew((v), (u32) &((canregs_t *)can_base[bd])->adr ))


#define CANsetw(bd,adr,m)	\
	writew((readw((u32) &((canregs_t *)can_base[bd])->adr)) \
		| (m) , (u32) &((canregs_t *)can_base[bd])->adr )
#define CANresetw(bd,adr,m)	\
	writew((readw((u32) &((canregs_t *)can_base[bd])->adr)) \
		& ~(m), (u32) &((canregs_t *)can_base[bd])->adr )
#define CANinw(bd,adr)		\
		(readw ((u32) &((canregs_t *)can_base[bd])->adr  ))
#define CANinwd(bd,adr)		\
	(printk("Cin: (%x)\n", (u32)&((canregs_t *)can_base[bd])->adr), \
		readw ((u32) &((canregs_t *)can_base[bd])->adr  ))
#define CANtestw(bd,adr,m)	\
	(readw((u32) &((canregs_t *)can_base[bd])->adr  ) & (m) )


   /* Memory long word access */
#ifndef IODEBUG
#define CANoutl(bd,adr,v)	\
		(writel(v, (u32) &((canregs_t *)can_base[bd])->adr ))
#endif
#define CANsetl(bd,adr,m)	\
	writel((readl((u32) &((canregs_t *)can_base[bd])->adr)) \
		| (m) , (u32) &((canregs_t *)can_base[bd])->adr )
#define CANresetl(bd,adr,m)	\
	writel((readl((u32) &((canregs_t *)can_base[bd])->adr)) \
		& ~(m), (u32) &((canregs_t *)can_base[bd])->adr )
#define CANinl(bd,adr)		\
		(readl ((u32) &((canregs_t *)can_base[bd])->adr  ))
#define CANtestl(bd,adr,m)	\
	(readl((u32) &((canregs_t *)can_base[bd])->adr  ) & (m) )

#endif 		/* __AT9263_CAN_H */
