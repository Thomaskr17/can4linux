/************************************************************************/
/* hardware access functions or macros */
/************************************************************************/

/* using memory acces with readb(), writeb() */
/* #error  memory I/O access */
/* #define can_base Base */

/* /sbin/lspci:
   CANBUS: PLX Technology, Inc. PCI9030 32-bit 33 MHz PCI <-> IOBus Bridge
   PCI 2.2
 */

#define PCI_SUBVENDOR_ID 0x13C3		/* JANZ AG */



/* Local Address Space 0 CAN starts at 0 */
#define JANZ_PCIL_CHANNEL_BASE 	0x0
#define JANZ_PCIL_CHANNEL_WIDTH	0x200   /* distance between two CAN */

/*

The CAN controllers are attached to the local bus of the PLX9030,
address mappings are arranged so that the registers of each CAN
controller are visible to the PCIbus memory space. Additionally,
there are some local status registers (i.e. interrupt status
register) that can also be accessed via PCI memory space.

Address Offset accesses:
    0x000..0x0ff   CAN controller 0 registers
    0x100..0x102   CAN controller 0 control

    0x200..0x2ff   CAN controller 1 registers
    0x300..0x302   CAN controller 1 control

CAN_TERM0,1             0x100, 0x300 (byte, rw)
 7    6    5    4     3     2      1       0
             reserved                 | TERM

TERM Set to zero to disable the line-termination,
     Set to one to enable the termination resistor.

     If jumpers are used to turn the CAN line termination on,
     any software settings are overwritten.


CAN_LED0,1           0x102, 0x302 (byte, wo)
 7    6     5     4 3    2      1       0
           reserved           |LEDG | LEDR |

LEDR Write 1 to turn red LED on, write 0 to disable.
LEDG Write 1 to turn green LED on, write 0 to disable.


--------------------

Clock 16MHz.			8 MHZ Table				OK

OCR ist auf folgendes zu setzen:
#define OCR_PUSHPULL    0xfa    // push/pull				OK

Clock Divider register
Außderdem ist der RX1 pin auf GND gelegt, daher müssen Sie
das CBP bit in CDR Register setzen.					OK

*/


#define CAN_LED_ADDRESS 0x102
#define CAN_LED_RED	0x01
#define CAN_LED_GREEN	0x02

#define CAN_TERM_ADDRESS 0x100
#define CAN_TERM_ON	0x01
#define CAN_TERM_OFF	0x00


/* CAN conrtroller register access macros */
#ifdef IODEBUG
#  define CANout(bd, adr, v)	\
	(printk("Cout: (%x)=%x\n", (u32)&((canregs_t *)can_base[bd])->adr, v), \
		writeb(v, (u32) &((canregs_t *)can_base[bd])->adr ))

#define CANset(bd, adr, m)     do	{\
	unsigned char v;	\
        v = (readb((void __iomem *)&((canregs_t *)can_base[bd])->adr)); \
	printk("CANset %x |= %x\n", (v), (m)); \
	writeb( v | (m), (u32) &((canregs_t *)can_base[bd])->adr ); \
	} while (0)

#define CANreset(bd, adr, m)	do {\
	unsigned char v; \
        v = (readb((u32) &((canregs_t *)can_base[bd])->adr)); \
	printk("CANreset %x &= ~%x\n", (v), (m)); \
	writeb( v & ~(m), (u32) &((canregs_t *)can_base[bd])->adr); \
	} while (0)

#define CANoutl(bd, adr, v)	\
	(printk("Cout: (%x)=%lx\n", (u32)&((canregs_t *)can_base[bd])->adr, v),\
		(writel(v, (u32) &((canregs_t *)can_base[bd])->adr)))
#else
   /* Memory Byte access */
#define CANout(bd, adr, v)	\
		(writeb(v, (void __iomem *)&((canregs_t *)can_base[bd])->adr))
#define CANset(bd, adr, m)	\
	writeb((readb((void __iomem *)&((canregs_t *)can_base[bd])->adr)) \
		| (m), (void __iomem *)&((canregs_t *)can_base[bd])->adr)
#define CANreset(bd, adr, m)	\
	writeb((readb((void __iomem *)&((canregs_t *)can_base[bd])->adr)) \
		& ~(m), (void __iomem *)&((canregs_t *)can_base[bd])->adr)
#endif  /* IODEBUG */

#define CANin(bd, adr)		\
		(readb ((void __iomem *)&((canregs_t *)can_base[bd])->adr))
#define CANtest(bd, adr, m)	\
	(readb((void __iomem *) &((canregs_t *)can_base[bd])->adr) & (m))
