/*
 * pcm9890 - hardware access functions or macros
 *
 ************************************************************************
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (c) 2009 port GmbH Halle/Saale
 *------------------------------------------------------------------
 * $Header: /z2/cvsroot/products/0530/software/can4linux/src/pcm9890.h,v 1.1 2009/06/03 14:29:20 oe Exp $
 *
 *--------------------------------------------------------------------------
 *
 *
 *
 *--------------------------------------------------------------------------
 */


/**
* \file pcm9890.h
* \author Name, port GmbH
* $Revision: 1.1 $
* $Date: 2009/06/03 14:29:20 $
*
* Module Description 
* see Doxygen Doc for all possibilites
*
*
*
*/

/* using memory acces with readb(), writeb() */
/* #error  memory I/O access */
/* #define can_base Base */

/* board parameters 
    16MHz CAN controller frequency
    4KB addresswindows, from D000H to EF00H, 32 address selection
    support wide interrupt range IRQ3,4,5,7,9
    	Jumper 1 for IRQ settings                     IRQ 3 		OK
    Direct memory addressmap
    	Jumper 3  0xD000						OK

    Jumper 2 CAN bus operating mode selection
    	using: high speed mode						ok

     # define CAN_OUTC_VAL           0xfa

*/



extern void board_clear_interrupts(int minor);


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
#define CANoutl(bd,adr,v)	\
		(writel(v, (u32) &((canregs_t *)can_base[bd])->adr ))
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


