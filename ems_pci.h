/************************************************************************/
/* hardware access functions or macros */
/************************************************************************/

/* using memory acces with readb(), writeb() */
/* #error  memory I/O access */
/* #define can_base Base */



/* Both CPC_PCI boards have the CAN Controllers memory mapped */


/* CPC_PCI2 */
#define CPC_PCI_CHANNEL_BASE 	0x400
#define CPC_PCI_CHANNEL_WIDTH	0x200


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
		(writeb(v, (void __iomem *) &((canregs_t __iomem *)can_base[bd])->adr ))
#define CANset(bd,adr,m)	\
	writeb((readb((void __iomem *) &((canregs_t __iomem *)can_base[bd])->adr)) \
		| (m) , (void __iomem *) &((canregs_t __iomem *)can_base[bd])->adr )
#define CANreset(bd,adr,m)	\
	writeb((readb((void __iomem *) &((canregs_t __iomem *)can_base[bd])->adr)) \
		& ~(m), (void __iomem *) &((canregs_t __iomem *)can_base[bd])->adr )
#endif  /* IODEBUG */

#define CANin(bd,adr)		\
		(readb ((void __iomem *) &((canregs_t __iomem *)can_base[bd])->adr  ))
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


