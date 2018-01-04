/************************************************************************/
/* hardware access functions or macros */
/************************************************************************/

/* Index memory adress
 * Two meory adresses are defined
 * first the first address hast to be written with the adress of
 *    the CAN register to be addressed
 * second on the second adress read or write the value
 */

#if CAN_INDEXED_PORT_IO
/* #error Indexed Intel port I/O access */
/* using port I/O with indexed inb()/outb() for Intel architectures like 
   SSV TRM/816 DIL-NET-PC */

#ifdef IODEBUG
#define CANout(bd,adr,v) {\
        printk("CANout bd:%x base:%x reg:%x val:%x\n", \
                bd, (u32) Base[bd], \
		(u32) &regbase->adr,v); \
        outb((u32) &regbase->adr,(u32) Base[bd]);\
        outb(v,((u32) Base[bd])+1);\
  }
#else
#define CANout(bd,adr,v) {\
        outb((u32) &regbase->adr,(u32) Base[bd]);\
        outb(v,((u32) Base[bd])+1);\
}
#endif
#define CANin(bd,adr) \
        Indexed_Inb((u32) Base[bd],(u32) &regbase->adr)

#define CANset(bd,adr,m) {\
        unsigned val; \
        val=Indexed_Inb((u32) Base[bd],(u32) &regbase->adr);\
        outb((u32) &regbase->adr,(u32) Base[bd]);\
        outb(val | m,((u32) Base[bd])+1);\
}
#define CANreset(bd,adr,m) {\
        unsigned val; \
        val=Indexed_Inb((u32) Base[bd],(u32) &regbase->adr);\
        outb((u32) &regbase->adr,(u32) Base[bd]);\
        outb(val & ~m,((u32) Base[bd])+1);\
}
#define CANtest(bd,adr,m) \
        (Indexed_Inb((u32) Base[bd],(u32) &regbase->adr) & m)

#endif
