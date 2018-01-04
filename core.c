/*
 * can_core - can4linux CAN driver module
   set tagprg="global -t $1"
 *
 * can4linux -- LINUX CAN device driver source
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (c) 2001-2011 port GmbH Halle/Saale
 * (c) 2001-2013 Heinz-Jürgen Oertel (oe@port.de)
 *          Claus Schroeter (clausi@chemie.fu-berlin.de)
 *------------------------------------------------------------------
 *
 *--------------------------------------------------------------------------
 *
 *
 *
 *
 *--------------------------------------------------------------------------
 */


/****************************************************************************/
/**
* \mainpage  can4linux - CAN network device driver
*
The LINUX CAN driver
can be used to control the CAN bus (http://www.can-cia.org)
connected to a PC running LINUX or embedded LINUX systems using uClinux.
Different interface boards and target micro controllers are supported
(see TARGET=VARIABLE in Makefile).
The most popular interfaces are the
AT-CAN-MINI http://www.port.de/pages/products/can/canopen/hardware/at_canmini.php?lang=en
and
CPC-PCI  http://www.port.de/0665
or PCI-express

This project was born in cooperation with the  LINUX LLP Project
to control laboratory or automation devices via CAN.
It started already in 1995 and is now considered as mature.


\attention
The former and older can4linux version 1.x
did support many different interface boards.
It was possible to use different kinds of boards at the same time.
Up to four boards could be placed in one computer.
With this feature it was possible to use /dev/can0 and
/dev/can2 for two boards AT-CAN-MINI with SJA1000
and /dev/can1 and /dev/can3 with two CPC-XT equipped with Intel 82527.
\attention
\b Attention: This can4linux version isn't supported anymore \b !

Instead the \b new version has to be compiled for the target hardware.
It was unlikely in the past that a PC or embedded device
was equipped with different CAN controllers.

\par Virtual CAN
In a special mode which is selected by setting the kernel parameter \e virtual
to true,
no hardware at all is needed.
The driver implements something like a virtual CAN network,
where producer and consumer exchange CAN frames only virtually.
\code
/sbin/insmod can4linux.ko virtual=1
\endcode

In all these configurations
the programmer sees the same driver interface with
open(), close(), read(), write() and ioctl() calls
( can_open(), can_close(), can_read(), can_write(), can_ioctl() ).

The driver itself is highly configurable
using the /proc interface of the LINUX kernel.

The following listing shows a typical configuration with three boards:

\code
$ grep . /proc/sys/dev/Can/\*
/proc/sys/dev/Can/AccCode:  -1       -1      -1      -1
/proc/sys/dev/Can/AccMask:  -1       -1      -1      -1
/proc/sys/dev/Can/ArbitrationLost   0	    0	    0	    0
/proc/sys/dev/Can/Base:     800      672     832     896
/proc/sys/dev/Can/Baud:     125      125     125     250
/proc/sys/dev/Can/CAN clock:8000000
/proc/sys/dev/Can/Chipset:  SJA1000
/proc/sys/dev/Can/dbgMask:  0
/proc/sys/dev/Can/framelength:8
/proc/sys/dev/Can/IOModel:  pppp
/proc/sys/dev/Can/IRQ:      5     7       3       5
/proc/sys/dev/Can/Outc:     250   250     250     0
/proc/sys/dev/Can/Overrun:  0     0       0       0
/proc/sys/dev/Can/RxErr:    0     0       0       0
/proc/sys/dev/Can/RxErrCounter:0  0       0       0
/proc/sys/dev/Can/Speedfactor:1   1       1       1
/proc/sys/dev/Can/Timeout:  100   100     100     100
/proc/sys/dev/Can/TxErr:    0     0       0       0
/proc/sys/dev/Can/TxErrCounter:    0     0       0       0
/proc/sys/dev/Can/version:  4.0_ATCANMINI_PELICAN SVN $Revision: 255 $
\endcode

This above mentioned full flexibility
is not needed in embedded applications.
For this applications, a stripped-down version exists.
It uses the same programming interface
but does the most configurations at compile time.
That means especially that only one CAN controller support with
a special register access method is compiled into the driver.
Actually the only CAN controller supported by this version
is the Philips SJA 1000 in both the compatibility mode
\b BasicCAN and the Philips \b PeliCAN mode (compile time selectable).

The version of can4linux currently available at SourceForge
http://sourceforge.net/projects/can4linux
is also supporting the Motorola FlexCAN module as ist is implemented
on Motorolas ColdFire 5282 CPU,
the FlexCAN controllers found on the Freescale ARM i.MX series controllers,
the Analog Devices BlackFin DSP with CAN,
Atmels ARM AT91SAM9263 with integrated CAN and also Microchips
stand alone CAN CAN controller MCP2515 connected via SPI.
One version of the SPI controlled MCP2515 is using direct register access
to the SPI controller found on Atmels AT91 CPUs.
This design was chosen to improve the performance of this special
can4linux version.
Care has to be taken not to use the Linux SPI driver at the same time.
Another possible make target, TARGET)MCP2515SPI,
is using the kernels SPI master driver to control the CAN MCP2515.

Since version 3.4.6 can4linux
assumes that your distribution uses \b udev to have the device
`/dev/can[0-9]' automatically created.
It is usually necessary to change the device access rights set by \b udev .
With the Fedora Core >= 4 or SuSE/novell you can do:

\code
echo 'KERNEL=="[Cc]an*", NAME="%k", MODE="0666"' \
     > /etc/udev/rules.d/91-Can.rules
\endcode

Alternatively create the device inodes in
/lib/udev/devices .
At system start-up,
the contents of that directory is copied to the /dev directory
with the same ownership and permissions as the files in /lib/udev/devices.

The driver creates class Can,
with information in /sys/class/Can/

See also udev (7)

\par The following sections are describing the \e sysctl entries.

\par AccCode/AccMask
contents of the message acceptance mask and acceptance code registers
of 82x200/SJA1000 compatible CAN controllers (see can_ioctl()).

\par Base
CAN controllers base address for each board.
Depending of the \e IOModel entry that can be a memory or I/O address.
(read-only for PCI boards)
\par Baud
used bit rate for this board in Kbit/s
\par Chipset
name of the supported CAN chip used with this boards
Read only for this version.
\par IOModel
one letter for each port. Readonly.
Read the CAN register access model.
The following models are currently supported:
\li b - special mode for the B&R CAN card,
     two special I/O addresses for register addressing and access
\li f - fast register access, special mode for the 82527
     uses memory locations for register addresses
     (ELIMA)
\li i - indexed access, one address serves as register selector,
     the next one as register access (MMC_SJA1000)
\li m - memory access, the registers are directly mapped into memory
\li p - port I/O,  80x86 specific I/O address range
	(AT-CAN-MINI,CTI_CANPRO, KVASER_PCICAN)
\li s - access via SPI (the only CAN controller supported so far
	is the Microchip MCP2515)

Since version 2.4 the IOModel is set at compile time.
\par IRQ
used IRQ numbers, one value for each board.
(read-only for PCI boards)
\par Outc
value of the output control register of the CAN controller
Since version 2.4 set at compile time.
A board specific value is used when the module the first time is loaded.
This board specific value can be reloaded by writing the value 0
to \e Outc .
\par
With the most boards using a Philips SJA1000,
by changing the value of the \e Outc it is possible
to inhibit generating the CAN Acknowledge.
Using this feature, it is possible to implement a
\b listen \b only
mode.
Please refer the CAN controller documentation for more details.
\par
Another way is implementing access to the \b mode register with an
\e ioctl () call in later \e can4linux versions.

\par Overrun
counter for overrun conditions in the CAN controller
\par RxErr
counter for CAN controller RX error conditions - CAN controller RX buffer hardware overflow
\par RxErrCounter
CAN controllers RX error counter
\par Timeout
time out value for waiting for a successful transmission

\par TxErr
counter for CAN controller TX error conditions
\par TxErrCounter
CAN controllers TX error counter

\par dbgMask
if compiled with debugging support, writing a value greater then 0
enables debugging to \b syslogd .
The value is bit coded.
\code
Bit 0 print all debug messages
Bit 1 print function entry message
Bit 2 print function exit message
Bit 3 print if a function branches in two different branches
Bit 4 print debug data statements
\endcode

\par version
read only entry containing the drivers version number and hardware acronym

\par framelength
Number of bytes which are used for the data section of the CAN frame.
This is typical 8 for classic CAN an 64 for CAN FD frames.

\par Speedfactor
Factor to be used for the bit timing in the optional High Bit Rate section (data phase)
of an CAN FD frame. Data type is integer, the default is 1.

\par CAN errors
In case the driver detects internal or CAN controller related errors
it reports this on two ways.
The \e flags field of a received message is used
by signaling common CAN errors like ERROR PASSIVE or Buffer overflows.
This can be combined with a real received message.
Or the driver uses a special error signaling message with an invalid
message  \e id 0f 0xFFFF.FFFF together with the \e flags.

If the driver is using the NXP SJA1000 it is possible to detect
CAN error frames caused by bit errors, crc errors, stuff errors, and so on.
In this case the driver reports an error signaling message to the read() caller
with \id =xFFFF.FFF and two data bytes containing the content of the
Error Code Register.
		data[0] = ecc;
		data[1] = ecc & 0x1f;

This special error diagnosis feature
must be enabled at driver load time
by setting the module parameter \e errint .
\code
/sbin/insmod can4linux.ko errint=1
\endcode




\par miscellaneous

Since 2010 the driver is hosted at SourceForge.
The used svn version number can be obtained by asking /sbin/modinfo.

Please see also at can_ioctl() for some additional descriptions.

For initially writing these sysctl entries after loading the driver
(or at any time) a shell script utility does exist.
It uses a board configuration file that is written over \e /proc/sys/dev/Can .
\code
utils/cansetup port.conf
\endcode
or, like used in the Makefile:
\code
CONFIG := $(shell uname -n)

# load host specific CAN configuration
load:
	@echo "Loading etc/$(CONFIG).conf CAN configuration"
	utils/cansetup etc/$(CONFIG).conf
	echo 0 >/proc/sys/dev/Can/dbgMask
\endcode
Example *.conf files are located in the \e etc/ directory.

\note
This documentation was created using the wonderful tool
\b Doxygen http://www.doxygen.org/index.html .
Die Dokumentation wurde unter Verwendung von
\b Doxygen http://www.doxygen.org/index.html
erstellt

*/

/* used for the pr_ () macros */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/fs.h>			/* register_chrdev() */
#include <linux/pci.h>

#include "defs.h"
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0)
# include <linux/device.h>
#endif

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0)
MODULE_VERSION("$Revision: 255 $");
#endif
#if 0
/* would be nice to have, but compiler says:
error: invalid application of 'sizeof' to incomplete type 'int[]'
*/
module_param_array(IRQ, int, NULL, S_IRUGO);
module_param_array(Baud, int, NULL, S_IRUGO);
#endif


/* This name is used to register the char device */
#define CANREGDEVNAME "can4linux" CAN_MODULE_POSTFIX

int IRQ_requested[MAX_CHANNELS]             = { 0 };
int Can_minors[MAX_CHANNELS]                = { 0 }; /* used as IRQ dev_id */

int virtual = 0;
module_param(virtual, int, S_IRUGO);
module_param(errint,  int, S_IRUGO);

static int Can_major 			    = CAN_MAJOR;

#endif /* DOXYGEN_SHOULD_SKIP_THIS */

/*
There's a C99 way of assigning to elements of a structure,
and this is definitely preferred over using the GNU extension.
gcc 2.95 (and later versions) supports the new C99 syntax.
The meaning is clear, and you should be aware
that any member of the structure which you don't explicitly assign
will be initialized to NULL by gcc.
*/

static struct file_operations can_fops = {
    .owner	=	THIS_MODULE,
    .open	=	can_open,
    .release	=	can_close,
    .read	=	can_read,
    .write	=	can_write,
    .poll	=	can_select,
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35)
    .unlocked_ioctl	=	can_ioctl,
#else
    .ioctl	=	can_ioctl,
#endif
    .fasync	=	can_fasync,
};

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0)
static struct class *can_class;
#endif


#ifndef DOXYGEN_SHOULD_SKIP_THIS


#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0)
static int __init can_init(void)
#else
static int init_module(void)
#endif
{
int i, j, err = 0;
int minor = -1;

    /* do you want do see debug message already while loading the driver ?
     * Then enable this line and set the mask != 0
     */
    /* dbgMask = 7; */

    DBGin();

    /* try udev support */
    if( (i = register_chrdev(Can_major, CANREGDEVNAME, &can_fops))) {
	printk(KERN_ERR "-> can't get Major %d = %d\n", Can_major, i);
	return(-EIO);
    }

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0)
    /* udev support */
    can_class = class_create(THIS_MODULE, CANREGDEVNAME);
    if( IS_ERR(can_class)) {
	printk("No udev support.\n");
	err = PTR_ERR(can_class);
	goto out_devfs;
    }

    /*
    The kernel API for device_create() in 2.6.26 and previous versions was:

    extern struct device *device_create(
    		struct class *cls, struct device *parent,
		dev_t devt, const char *fmt, ...)
		__attribute__((format(printf, 4, 5)));

    and starting in 2.6.27 it changed to:


    struct device *device_create(
    		struct class *cls, struct device *parent,
		dev_t devt, void *drvdata,
		const char *fmt, ...)

    */
    for (i = 0; i < MAX_CHANNELS; i++) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
	device_create(can_class, NULL, MKDEV(Can_major, i),
		"can%d", i);
#else
	device_create(can_class, NULL, MKDEV(Can_major, i),
		NULL, "can%d", i);
#endif
    }


#endif

    printk(KERN_INFO __CAN_TYPE__ "CAN Driver " VERSION " (c) "
    					__DATE__ " " __TIME__ "\n");
    if(virtual == 0) {
#if defined(MCF5282)
    printk(KERN_INFO " FlexCAN port by H.J. Oertel (oe@port.de)\n");
#elif defined(AD_BLACKFIN)
    printk(KERN_INFO " BlackFin port by H.J. Oertel (oe@port.de)\n");
#elif defined(ATMEL_SAM9)
    printk(KERN_INFO " Atmel AT91SAM9263 port by H.J. Oertel (oe@port.de)\n");
#elif defined(SSV_MCP2515) || defined(AuR_MCP2515)
    printk(KERN_INFO " Atmel AT91 and MCP2515 port by H.J. Oertel (oe@port.de) - QF by mha@ist1.de\n");
#elif defined(MCP2515SPI)
    printk(KERN_INFO " MCP2515 SPI port by H.J. Oertel (oe@port.de)\n");
#elif defined(IMX35) || defined(IMX25) || defined(IMX28)
    printk(KERN_INFO " Freescale FlexCAN port by H.J. Oertel (oe@port.de)\n");
#else
    printk(KERN_INFO " H.J. Oertel (oe@port.de)\n");
#endif
    } else {
    printk(KERN_INFO " virtual CAN network, (c) H.J. Oertel (oe@port.de)\n");
    }

    printk(KERN_INFO " MAX_CHANNELS %d\n", MAX_CHANNELS);
    printk(KERN_INFO " CAN_MAX_OPEN %d\n", CAN_MAX_OPEN);

    /*
    initialize the variables laid down in /proc/sys/dev/Can
    ========================================================
    */
    for (i = 0; i < MAX_CHANNELS; i++) {
        atomic_set(&Can_isopen[i], 0);
	for(j=0; j < CAN_MAX_OPEN; j++) {
	    selfreception[i][j] = 1;
	}
	use_timestamp[i] = 1;
	IOModel[i]       = IO_MODEL;
	Baud[i]          = 125;

#if !defined(IMX35) && !defined(IMX25) && !defined(IMX28)
	AccCode[i]       = AccMask[i] =  STD_MASK;
#endif
	Timeout[i]       = 100;
	Outc[i]          = CAN_OUTC_VAL;
	IRQ_requested[i] = 0;
	Can_minors[i]    = i;		/* used as IRQ dev_id */

	spin_lock_init(&write_splock[i]);

#if defined(MCF5282)
	/* we have a really fixed address here */
	Base[i] = (MCF_MBAR + 0x1c0000);
	/* Because the MCF FlexCAN is using more then 1 Interrupt vector,
	 * what should be specified here ?
	 * For information purpose let's only specify  the first used here
	 */
	IRQ[i] = 136;
#endif
#if defined(IMX35) || defined(IMX25) || defined(IMX28)
#if defined(IMX28)
#define CAN1_BASE_ADDR MX28_CAN0_BASE_ADDR
#define MXC_INT_CAN1 MX28_INT_CAN0
#endif
	/* we have a really fixed address here */
	/* better would be using platform device pdev.resource.start */

#if defined(IMX25)
	Base[i] = CAN1_BASE_ADDR + (0x4000 * i);
#endif
#if defined(IMX28)
	Base[i] = CAN1_BASE_ADDR + (0x2000 * i);
#endif
#if defined(IMX35)
	/* CAN1 0x53FE_4000 */
	/* CAN2 0x53FE_8000 */
	Base[i] = CAN1_BASE_ADDR + (0x4000 * i);
#endif
	/* printk("can%d: 0x%08lx\n", i, Base[i]); */
	IRQ[i] = MXC_INT_CAN1 + (i * 1);
	/* printk("can%d: %d\n", i, IRQ[i]); */
	init_imx35_hw(i);
	{
	int masks;
	    for(masks = 0; masks < 8; masks++) {
		AccCode[masks][i] = AccMask[masks][i] =  STD_MASK;
	    }
	}

	erroractive[i]	 = 1;  /* set error active true */
#endif


#if defined(AD_BLACKFIN)
	/* we have a really fixed address here */
	/* starting with Mailbox config reg 1  */
	Base[i] = BFCAN_BASE;
	/* Because the AD BlackFin CAN is using more then 1 Interrupt vector,
	 * what should be specified here ?
	 * For information purpose let's only specify  the first used here.
	 * Next one is +1
	 */
	IRQ[i] = IRQ_CAN_RX;
#endif

#if defined(ATMEL_SAM9)
	/* we have a really fixed address here */
	Base[i] = (AT91SAM9263_BASE_CAN);
	/*
	 */
	IRQ[i] = AT91SAM9263_ID_CAN;
#endif

#if defined(VCMA9)
	Base[i] = 0x28000000;
	can_sysctl_table[CAN_SYSCTL_IRQ  - 1].mode = 0444;
	can_sysctl_table[CAN_SYSCTL_BASE - 1].mode = 0444;
# if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0))
	IRQ[i] = 37 + 16;
# else
	IRQ[i] = 37;
# endif

#endif


#if defined(CCPC104)
        pc104_irqsetup();
        IRQ[i]           = 67;          /* The only possible vector on CTRLink's 5282 CPU */
        Base[i]          = 0x40000280;
#endif

/*
static struct spi_board_info sbc_spi_devices[].irq
*/
#if defined(SSV_MCP2515) || defined(AuR_MCP2515) || defined(MCP2515SPI)
	Base[i]          = 0xFFFFFFFF;	/* not used, SPI ? */
	/* IRQ[i]           = gpio_to_irq(CAN_SPI_INTERRUPT_PIN);  */
	/* set the used IRQ via /proc/sys/dev/Can/IRQ */
	IRQ[i]           = 0;
#endif

#if defined(MMC_SJA1000)

	Base[i]          = 0x30000000 + (2 * i);
        /* IRQ[i]           = gpio_to_irq(CAN_IRQ_PIN); */
        IRQ[i]           =  AT91SAM9260_ID_IRQ0;        /* 29 bei Heyfra */
#endif


#if defined(ZEDBOARD)
	/* platform date have to come from the device tree in recent kernels */
	/* can0: 0xE0008000	irq 60
	 * can1: 0xE0009000	irq 83 */
	Base[i]          = 0xE0008000 + (0x1000 * i);
        IRQ[i]           = 60 + (23 * i);
	init_zynq_hw(i);
#endif
    } /* end of for loop initializing all CAN channels */


    if(virtual == 0) {
    /*
    ========== Begin HW initialization ==================
    */




#if defined(MULTILOG_SJA1000)
    /* multilog32 addresses and interrupts of the both sja1000s are:

	1: addr=0x30000000, IRQ4 -> AT91_PIN_PA2
	2: addr=0x40000000, IRQ5 -> AT91_PIN_PA3
    */

    Base[0] = 0x30000000;
    Base[1] = 0x40000000;

    IRQ[0]  = AT91RM9200_ID_IRQ4;
    IRQ[1]  = AT91RM9200_ID_IRQ5;
#endif

#if defined(VCMA9)
    /* only one SJA1000 available
     * we can check if it is available when loading the module
     */
     if(!controller_available(0x28000000, 1)) {
	err =  -EIO;
	goto out_class;
     }
#endif

    /* after initializing channel based parameters
     * finish some entries
     * and do drivers specific initialization
     */
    IOModel[i] = '\0';

#if defined(CAN4LINUX_PCI)
    /* make some syctl entries read only
     * IRQ number
     * Base address
     * and access mode
     * are fixed and provided by the PCI BIOS
     */
    can_sysctl_table[CAN_SYSCTL_IRQ  - 1].mode = 0444;
    can_sysctl_table[CAN_SYSCTL_BASE - 1].mode = 0444;
    /* printk(KERN_INFO "CAN pci test loaded\n"); */
    /* dbgMask = 0; */
    if(pcimod_scan()) {
        pr_info("no valid PCI CAN found\n");
	err = -EIO;
	goto out_class;
    }

    pr_info("pci scan success\n");
#endif

#if defined(CCPC104)
    /* The only possible interrupt could be IRQ4 on the PC104 Board */
    can_sysctl_table[CAN_SYSCTL_IRQ - 1].mode = 0444;
#endif

#if defined(MCF5282) || defined(IMX35) || defined(IMX25) || defined(IMX28)\
    || defined(SSV_MCP2515) || defined(AuR_MCP2515) || defined(MCP2515SPI)
    can_sysctl_table[CAN_SYSCTL_BASE - 1].mode = 0444;
    can_sysctl_table[CAN_SYSCTL_IRQ - 1].mode = 0444;
#endif

#if defined(MMC_SJA1000) || defined(MULTILOG_SJA1000)
    init_mmc_hw();
#endif


#if defined(MCP2515SPI)
    /* SPI driver is calling the probe() function */
    printk(KERN_INFO " %s() register CAN SPI driver\n", __func__);
    err = spi_register_driver(&mcp251x_can_driver);
#endif
#if defined(SSV_MCP2515) || defined(AuR_MCP2515)
    /* call the probe function direct */
    err = mcp251x_can_probe();
    if (err) {
	goto out_class;
    }
#endif


    /* end of hardware specific part */
    } else {
	/* do nothing hardware related but overwrite
	   /proc/sys/.../Chipset */
	strncpy(Chipset, "virtual CAN", PROC_CHIPSET_LENGTH);
    }

    /*
    ========== end HW initialization ==================
    */

#if LDDK_USE_PROCINFO
    register_procinfo();
#endif
#if LDDK_USE_SYSCTL
    register_systables();
#endif

    DBGout();
    return 0;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0)
#if defined(VCMA9) || defined(CAN4LINUX_PCI) \
	|| defined(SSV_MCP2515) || defined(AuR_MCP2515)
out_class:
#endif
    class_destroy(can_class);
out_devfs:
#endif
    unregister_chrdev(Can_major, CANREGDEVNAME);
    DBGout();
    return err;
}


#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0)
static void __exit can_exit(void)
#else
static void cleanup_module(void)
#endif
{
#if defined(KVASER_PCICAN)
int i;
void *ptr;
#endif
int minor = -1;

    DBGin();

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
  if (MOD_IN_USE) {
    printk(KERN_WARNING "Can : device busy, remove delayed\n");
  }
#endif


#ifdef KVASER_PCICAN

    i = 0;
    ptr = NULL;
    /* The pointer to dev can be used up to four times,
     * but we have to release the region only once */
    while(Can_pcidev[i]) {
	if( ptr !=  Can_pcidev[i]) {

    printk(KERN_INFO "Can[-1]: - : Kvaser: release PCI resources\n");
	    /* disable PCI board interrupts */
	    disable_pci_interrupt(pci_resource_start(Can_pcidev[i], 0));
	    /* printk(KERN_DEBUG "release Kvaser CAN region 2 (XILINX)\n"); */
	    pci_release_region(Can_pcidev[i], 2);   /*release xilinx */
	    /* printk(KERN_DEBUG "release Kvaser CAN region 1 (CAN)\n"); */
	    pci_release_region(Can_pcidev[i], 1); /*release i/o */
	    /* printk(KERN_DEBUG "release Kvaser CAN region 0 (PCI)\n"); */
	    pci_release_region(Can_pcidev[i], 0);   /*release pci */

	}
	ptr = Can_pcidev[i];
	i++;
    }

#endif

    unregister_chrdev(Can_major, CANREGDEVNAME);
    printk(KERN_INFO "Can[-1]: - : char device \"" CANREGDEVNAME
	    "\" removed\n");

#if defined(IMX35) || defined(IMX25) || defined(IMX28)
    exit_imx35_hw();
#endif

#if defined(ZEDBOARD)
    exit_zynq_hw();
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0)
    if( !IS_ERR(can_class)) {
	int i;
	for (i = 0; i < MAX_CHANNELS; i++) {
	    device_destroy(can_class, MKDEV(Can_major, i));
	}
	class_destroy(can_class);
    }
#endif
#if LDDK_USE_PROCINFO
    unregister_procinfo();
#endif

#if defined(MCP2515SPI)
    {
    extern struct mcp251x_priv realone;
    struct mcp251x_priv *priv = &realone;
    /* printk(KERN_INFO "%s(): calling unregister spi\n", __func__); */
    spi_unregister_driver(&mcp251x_can_driver);
    destroy_workqueue(priv->wq);
    }
#endif
#if defined(SSV_MCP2515) || defined(AuR_MCP2515)
    {
    mcp251x_can_remove();
    /* only with direct using SPI */
    release_mem_region(0xfffa4000ul, 0x4000);
    }
#endif

#if LDDK_USE_SYSCTL
    unregister_systables();
#endif
    DBGout();
}

#endif /* DOXYGEN_SHOULD_SKIP_THIS */


#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0)
module_init(can_init);
module_exit(can_exit);
#endif

MODULE_AUTHOR("H.-J.Oertel <hj.oertel@t-online.d");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CAN fieldbus driver " __CAN_TYPE__);

