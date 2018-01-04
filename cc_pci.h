/************************************************************************/
/* hardware access functions or macros					*/
/* for the Contemporary Controls board CANPCI-CO/DN using SJA1000	*/
/************************************************************************/


extern void board_clear_interrupts(int minor);

#ifdef  CAN_PORT_IO

/* using port I/O with inb()/outb() for Intel architectures */

#ifdef IODEBUG
#  define CANout(bd,adr,v)      \
	(printk("Cout: (%x)=%x\n", (int)&((canregs_t *)Base[bd])->adr, v), \
		outb(v, (int) &((canregs_t *)Base[bd])->adr ))
#else
#  define CANout(bd,adr,v)      \
	(outb(v, (long) &((canregs_t *)Base[bd])->adr ))
#endif  /* IODEBUG */


#define CANin(bd,adr)           \
	(inb ((long) &((canregs_t *)Base[bd])->adr  ))
#define CANset(bd,adr,m)        \
	outb((inb((long) &((canregs_t *)Base[bd])->adr)) \
		| m ,(long) &((canregs_t *)Base[bd])->adr )
#define CANreset(bd,adr,m)      \
	outb((inb((long) &((canregs_t *)Base[bd])->adr)) \
		& ~m,(long) &((canregs_t *)Base[bd])->adr )
#define CANtest(bd,adr,m)       \
	(inb((long) &((canregs_t *)Base[bd])->adr  ) & m )

#endif 	/* CAN_PORT_IO */


