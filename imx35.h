/************************************************************************/
/* hardware access functions or macros */
/************************************************************************/

/* we are using the following functions
 static inline void __raw_writel(u32 b, volatile void __iomem *addr)
 static inline u32 __raw_readl(const volatile void __iomem *addr)

 do acess the iomem
*/

#ifndef  _IMX35_H_
#define  _IMX35_H_


/* Memory word access, 16 bit */
#define CANinw(bd,adr)          \
                (__raw_readw ((void const volatile __iomem *)&((flexcan_t __iomem *)can_base[bd])->adr  ))


/* Memory long word access */
#if 0  /*debug version using printk to inform the programmer */
#define CANoutl(bd,adr,v)       do { \
	printk(" write 0x%08x to %p\n", v, &((flexcan_t *)can_base[bd])->adr);\
                (__raw_writel(v, (void volatile __iomem *)&((flexcan_t __iomem *)can_base[bd])->adr ));\
                } while(0)
#endif

#define CANoutl(bd,adr,v)	\
                (__raw_writel(v, (void volatile __iomem *)&((flexcan_t __iomem *)can_base[bd])->adr ))

#define CANsetl(bd,adr,m)       \
        __raw_writel((__raw_readl((void const volatile __iomem *)&((flexcan_t __iomem *)can_base[bd])->adr)) \
                | (m) , (void volatile __iomem *)&((flexcan_t __iomem *)can_base[bd])->adr )

#define CANresetl(bd,adr,m)     \
        __raw_writel((__raw_readl((void const volatile __iomem *)&((flexcan_t __iomem *)can_base[bd])->adr)) \
                & ~(m) , (void volatile __iomem *)&((flexcan_t __iomem *)can_base[bd])->adr )

#define CANinl(bd,adr)  ( __raw_readl(       		\
	  (void const volatile __iomem *)&((flexcan_t __iomem *)(can_base[bd]))->adr)  			\
	)

#define CANtestl(bd,adr,m)      \
        (__raw_readl( (void const volatile __iomem *)&((flexcan_t *)can_base[bd])->adr  ) & (m) )

#endif          /* _IMX35_H_ */

