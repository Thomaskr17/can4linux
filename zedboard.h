/************************************************************************/
/* hardware access functions or macros */
/************************************************************************/

/* using memory acces with readb(), writeb() */
/* #error  memory I/O access */
/* #define can_base Base */

#if defined(CAN_PORT_IO) || defined(CAN_INDEXED_PORT_IO)
# error "Only memory access allowed, "
#endif

/* Memory long word access */
#define CANoutl(bd,adr,v)	\
		(writel(v, (void __iomem *) &((canregs_t *)can_base[bd])->adr ))
#define CANsetl(bd,adr,m)	\
	writel((readl((u32) &((canregs_t *)can_base[bd])->adr)) \
		| (m) , (u32) &((canregs_t *)can_base[bd])->adr )
#define CANresetl(bd,adr,m)	\
	writel((readl((u32) &((canregs_t *)can_base[bd])->adr)) \
		& ~(m), (u32) &((canregs_t *)can_base[bd])->adr )
#define CANinl(bd,adr)		\
		(readl ((void __iomem *) &((canregs_t *)can_base[bd])->adr  ))
#define CANtestl(bd,adr,m)	\
	(readl((u32) &((canregs_t *)can_base[bd])->adr  ) & (m) )

