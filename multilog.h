/************************************************************************/
/* hardware access functions or macros */
/************************************************************************/

/* Index memory adress
 * Two meory adresses are defined
 * first the first address hast to be written with the adress of
 *    the CAN register to be addressed
 * second on the second adress read or write the value
 *
 * multilog is using CAN_INDEXED_MEM_IO
 */


#ifdef CAN_INDEXED_MEM_IO

#if defined(MMC_SJA1000)
# define REGOFFSET 1
#elif defined(MULTILOG_SJA1000)
# define REGOFFSET 4
#endif

static inline unsigned Indexed_Inb(void __iomem *base, u32 adr) {
unsigned val;
    writeb(adr, (void __iomem *)base);
    val = readb(base + REGOFFSET);

# ifdef IODEBUG
  printk("CANin: base: %x adr: %x, got: %x\n",
  	(u32)base, (u8)adr, (u8)val);
# endif
    return val;
}

# ifdef IODEBUG

#  define CANout(bd,adr,v)	do {\
        printk("CANout bd:%x base:%p reg:%x val:%x\n", \
                bd, (void __iomem *)can_base[bd], \
		(u32) &regbase->adr, v); \
        writeb((u32) &regbase->adr, (void __iomem *)can_base[bd]); \
        writeb(v, (void __iomem *)can_base[bd] + REGOFFSET); \
  } while(0)

#  define CANin(bd,adr) 		\
	Indexed_Inb((void __iomem *)can_base[bd], (u32)&regbase->adr)

#  define CANset(bd,adr,m)  {\
        unsigned val; \
        val = Indexed_Inb((void __iomem *)can_base[bd], (u32) &regbase->adr);\
        writeb((u32) &regbase->adr, (void __iomem *)can_base[bd]); \
        writeb(val | m, (void __iomem *)can_base[bd] + REGOFFSET); \
	}

#  define CANreset(bd,adr,m)  {\
        unsigned val; \
        val = Indexed_Inb((void __iomem *)can_base[bd], (u32) &regbase->adr);\
        writeb((u32) &regbase->adr, (void __iomem *)can_base[bd]); \
        writeb(val & ~m, (void __iomem *)can_base[bd] + REGOFFSET); \
}
/* not used, not filled, causes error at compile time */
#  define CANtest(bd,adr,m) (x)

# else   /* IODEBUG */

#  define CANout(bd,adr,v)	do {\
        writeb((u32) &regbase->adr, (void __iomem *)can_base[bd]); \
        writeb(v, (void __iomem *)can_base[bd] + REGOFFSET); \
  } while(0)

#  define CANin(bd,adr) 		\
	Indexed_Inb((void __iomem *)can_base[bd], (u32)&regbase->adr)

#  define CANset(bd,adr,m)  {\
        unsigned val; \
        val = Indexed_Inb((void __iomem *)can_base[bd], (u32) &regbase->adr);\
        writeb((u32) &regbase->adr, (void __iomem *)can_base[bd]); \
        writeb(val | m, (void __iomem *)can_base[bd] + REGOFFSET); \
	}

#  define CANreset(bd,adr,m)  {\
        unsigned val; \
        val = Indexed_Inb((void __iomem *)can_base[bd], (u32) &regbase->adr);\
        writeb((u32) &regbase->adr, (void __iomem *)can_base[bd]); \
        writeb(val & ~m, (void __iomem *)can_base[bd] + REGOFFSET); \
}

/* not used, not filled, causes error at compile time */
#  define CANtest(bd,adr,m) (x)

# endif /* IODEBUG */
#endif /* CAN_INDEXED_MEM_IO */

