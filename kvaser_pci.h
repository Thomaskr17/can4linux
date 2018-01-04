/************************************************************************/
/* hardware access functions or macros */
/* for Kvaser 4 channel passive board using SJA1000 */
/************************************************************************/

#ifdef  CAN_PORT_IO
/* #warning "Using port-IO access macros" */
/* #error Intel port I/O access */
/* using port I/O with inb()/outb() for Intel architectures like 
   AT-CAN-MINI ISA board */
#ifdef IODEBUG
#  define CANout(bd,adr,v)      \
	(printk("Cout: (%x)=%x\n", (int)&((canregs_t *)Base[bd])->adr, v), \
		outb(v, (int) &((canregs_t *)Base[bd])->adr ))
#else
#ifdef CONFIG_X86_64
#  define CANout(bd,adr,v)      \
	(outb(v, (long) &((canregs_t *)((u64)Base[bd]))->adr ))
#endif
#ifdef CONFIG_X86_32
#  define CANout(bd,adr,v)      \
	(outb(v, (u32) &((canregs_t *)((u32)Base[bd]))->adr ))
#endif
#endif  /* IODEBUG */


#ifdef CONFIG_X86_64

#define CANin(bd,adr)           \
	(inb ((long) &((canregs_t *)((u64)Base[bd]))->adr  ))
	
#define CANset(bd,adr,m)        \
	outb((inb((long) &((canregs_t *)((u64)Base[bd]))->adr)) \
		| m ,(long) &((canregs_t *)((u64)Base[bd]))->adr )

#define CANreset(bd,adr,m)      \
	outb((inb((long) &((canregs_t *)((u64)Base[bd]))->adr)) \
		& ~m,(long) &((canregs_t *)((u64)Base[bd]))->adr )

#define CANtest(bd,adr,m)       \
	(inb((long) &((canregs_t *)((u64)Base[bd]))->adr  ) & m )



#endif 	/* CONFIG_X86_64 */

#ifdef CONFIG_X86_32

#define CANin(bd,adr)           \
	(inb ((u32) &((canregs_t *)((u32)Base[bd]))->adr  ))
	
#define CANset(bd,adr,m)        \
	outb((inb((u32) &((canregs_t *)((u32)Base[bd]))->adr)) \
		| m ,(u32) &((canregs_t *)((u32)Base[bd]))->adr )

#define CANreset(bd,adr,m)      \
	outb((inb((u32) &((canregs_t *)((u32)Base[bd]))->adr)) \
		& ~m,(u32) &((canregs_t *)((u32)Base[bd]))->adr )

#define CANtest(bd,adr,m)       \
	(inb((u32) &((canregs_t *)((u32)Base[bd]))->adr  ) & m )


#endif 	/* CONFIG_X86_32 */

#endif 	/* CAN_PORT_IO */


