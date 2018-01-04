/*
 * defs.h  - can4linux CAN driver module, common definitions
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (c) 2001-2008 port GmbH Halle/Saale
 *------------------------------------------------------------------
 * $Header: /z2/cvsroot/products/0530/software/can4linux/src/defs.h,v 1.16 2009/06/18 09:15:56 oe Exp $
 *
 *--------------------------------------------------------------------------
 *
 *
 *
 *--------------------------------------------------------------------------
 */


/**
* \file defs.h
* \author Name, port GmbH
* $Revision: 1.16 $
* $Date: 2009/06/18 09:15:56 $
*
* Module Description 
* see Doxygen Doc for all possibilites
*
*
*
*/


#ifdef __KERNEL__
#include <linux/version.h>
#include <linux/init.h>
#include <linux/module.h>


/* needed for 2.4 */
#ifndef NOMODULE
# define __NO_VERSION__
# ifndef MODULE
#  define MODULE
# endif
#endif

#ifdef __KERNEL__
#ifndef KERNEL_VERSION
#define KERNEL_VERSION(a,b,c) (((a) << 16) + ((b) << 8) + (c))
#endif


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
# include <linux/config.h>
#endif

#include <linux/kernel.h>
#include <linux/tty.h>
#include <linux/errno.h>
#include <linux/major.h>


#if LINUX_VERSION_CODE > KERNEL_VERSION(2,4,12)
# include <linux/slab.h>
#else
# include <linux/malloc.h>
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,1,0)
#include <linux/poll.h>
#endif

#include <asm/io.h>
#include <asm/segment.h>
#include <asm/irq.h>
#include <asm/dma.h>

#include <linux/mm.h>
#include <linux/signal.h>
#include <linux/timer.h>

#include <asm/uaccess.h>

#define __lddk_copy_from_user(a,b,c) _cnt = copy_from_user(a,b,c)
#define __lddk_copy_to_user(a,b,c) _cnt =copy_to_user(a,b,c)


#include <linux/ioport.h>

#if !defined TRUE
# define TRUE  1
# define FALSE 0
#endif

#if !defined(__iomem)
# define __iomem
#endif
#endif

/* Unsigned value which later should be converted to an integer */
#if defined(__x86_64__)

#  if defined(CAN_PORT_IO)
typedef unsigned long int upointer_t;
/* typedef unsigned int upointer_t; */
#  else
typedef unsigned long upointer_t;
#  endif

#else  /* 32 bit system */

#  if defined(CAN_PORT_IO)
typedef unsigned int upointer_t;
#  else
typedef unsigned long upointer_t;
#  endif
#endif /* defined(__x86_64__) */


#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0) 
# include <linux/interrupt.h> 

#else
 /* For 2.4.x compatibility, 2.4.x can use */
typedef void irqreturn_t;
#define IRQ_NONE
#define IRQ_HANDLED
#define IRQ_RETVAL(x)

/* not available in k2.4.x */
static inline unsigned iminor(struct inode *inode)
{
	return MINOR(inode->i_rdev);
}
#endif

#if CAN4LINUX_PCI
# define _BUS_TYPE "PCI-"
/* 
Since 2008 we hav two versions of the CPC PCI
The old one, using the SIemens PITA chip with up to two CAN controllers
The new one using A PLX PCI Bridge with up to 4 CAN controllers
the only one supported: EMS CPC-PCI */

/* Old CPC-PCI
/sbin/lspci:
Network controller: Siemens Nixdorf AG Eicon Diva 2.02 compatible passive ISDN card (rev 02)
*/
# define PCI_VENDOR_CAN_EMS	0x110a
# define PCI_DEVICE_CAN 	0x2104
/* PSB4610 PITA-2 bridge control registers */
#define PITA2_ICR           0x00	    /* Interrupt Control Register */
#define PITA2_ICR_INT0      0x00000002	/* [RC] INT0 Active/Clear */
#define PITA2_ICR_INT0_EN   0x00020000	/* [RW] Enable INT0 */

#define PITA2_MISC          0x1c	    /* Miscellaneous Register */
#define PITA2_MISC_CONFIG   0x04000000	/* Multiplexed parallel interface */

/* New CPC_PCI2 */
/* #define PCI_VENDOR_ID_PLX  : in pci_ids.h */
#ifndef PCI_DEVICE_ID_PLX_9030
#define PCI_DEVICE_ID_PLX_9030 0x9030
#endif
/* PLX 9030 PCI-to-IO Bridge control registers */
#define PLX9030_ICR             0x4c  /* Interrupt Control Register */
#define PLX9030_ICR_CLEAR_IRQ0  0x400
#define PLX9030_ICR_ENABLE_IRQ0 0x41



/* and CANPCI manufactured by Contemporary Controls Inc. */
# define PCI_VENDOR_CAN_CC	0x1571
# define PCI_DEVICE_CC_MASK 	0xC001
# define PCI_DEVICE_CC_CANopen 	0xC001
# define PCI_DEVICE_CC_CANDnet 	0xC002

/* and PCICAN manufactured by Kvaser */
# define PCI_VENDOR_CAN_KVASER	0x10e8
# define PCI_DEVICE_CAN_KVASER	0x8406


#elif CAN4LINUX_PCCARD
# define _BUS_TYPE "PC-Card-"
#elif  defined(SSV_MCP2515) || defined(AuR_MCP2515) || defined(MCP2515SPI) || defined(RASPI)
# define _BUS_TYPE "SPI-"
#else
# define _BUS_TYPE "ISA-"
#endif

#if    defined(ATCANMINI_PELICAN) \
    || defined(CCPC104)		\
    || defined(CPC_PCI)		\
    || defined(CPC_PCI2)	\
    || defined(JANZ_PCIL)	\
    || defined(CC_CANPCI)	\
    || defined(IXXAT_PCI03)	\
    || defined(PCM3680)		\
    || defined(PCM9890)		\
    || defined(CPC104)		\
    || defined(CPC104_200)	\
    || defined(CPC_PCM_104)	\
    || defined(CPC_CARD)	\
    || defined(KVASER_PCICAN)	\
    || defined(VCMA9)		\
    || defined(MMC_SJA1000)	\
    || defined(MULTILOG_SJA1000)\
    || defined(CTI_CANPRO)	\
    || defined(ECAN1000)
/* ---------------------------------------------------------------------- */

# ifdef  CAN_PORT_IO
#  define __CAN_TYPE__ _BUS_TYPE "PeliCAN-port I/O "
# else
#  ifdef  CAN_INDEXED_PORT_IO
#   define __CAN_TYPE__ _BUS_TYPE "PeliCAN-indexed port I/O "
#  else
#  define __CAN_TYPE__ _BUS_TYPE "PeliCAN-memory mapped "
#  endif
# endif

#elif defined(MCF5282)
#   define __CAN_TYPE__ _BUS_TYPE "FlexCAN "
/* ---------------------------------------------------------------------- */
#elif defined(IMX35) || defined(IMX25) || defined(IMX28)
#   define __CAN_TYPE__ _BUS_TYPE "FlexCAN "
/* ---------------------------------------------------------------------- */
#elif defined(UNCTWINCAN)
#   define __CAN_TYPE__ _BUS_TYPE "TwinCAN "
/* ---------------------------------------------------------------------- */
#elif defined(AD_BLACKFIN)
#   define __CAN_TYPE__ _BUS_TYPE "BlackFin-CAN "
/* ---------------------------------------------------------------------- */
#elif defined(ATMEL_SAM9)
#   define __CAN_TYPE__ _BUS_TYPE "Atmel-CAN "
/* ---------------------------------------------------------------------- */
#elif defined(SSV_MCP2515) || defined(AuR_MCP2515) || defined(MCP2515SPI) || defined(RASPI)
#   define __CAN_TYPE__ _BUS_TYPE "Microchip MCP2515 "
/* ---------------------------------------------------------------------- */
#elif defined(ZEDBOARD)
#   define __CAN_TYPE__ _BUS_TYPE "Xilinx XPSCAN "
/* ---------------------------------------------------------------------- */
#elif defined(ZEDBOARD)
#   define __CAN_TYPE__ _BUS_TYPE "Xilinx XPSCAN "
/* ---------------------------------------------------------------------- */
#else
/* ---------------------------------------------------------------------- */
#endif

/* Length of the "version" string entry in /proc/.../version */
#define PROC_VER_LENGTH 80
/* Length of the "Chipset" string entry in /proc/.../version */
#define PROC_CHIPSET_LENGTH 30

/* kernels higher 2.3.x have a new kernel interface.
*  Since can4linux V3.x only kernel interfaces 2.4 an higher are supported.
*  This simplfies alot of things
* ******************/
#define __LDDK_WRITE_TYPE	ssize_t
#define __LDDK_CLOSE_TYPE	int
#define __LDDK_READ_TYPE	ssize_t
#define __LDDK_OPEN_TYPE	int

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35) 
# define __LDDK_IOCTL_TYPE	long
#else
# define __LDDK_IOCTL_TYPE	int
#endif

#define __LDDK_SELECT_TYPE	unsigned int
#define __LDDK_FASYNC_TYPE	int

#define __LDDK_SEEK_PARAM 	struct file *file, loff_t off, size_t count
#define __LDDK_READ_PARAM 	struct file *file, char __user *buffer, size_t count, \
					loff_t *loff
#define __LDDK_WRITE_PARAM 	struct file *file, const char __user *buffer, \
					size_t count, loff_t *loff
#define __LDDK_READDIR_PARAM 	struct file *file, void *dirent, filldir_t count
#define __LDDK_SELECT_PARAM 	struct file *file, \
					struct poll_table_struct *wait
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35) 
#define __LDDK_IOCTL_PARAM 	struct file *file, \
					unsigned int cmd, unsigned long arg
#else
#define __LDDK_IOCTL_PARAM 	struct inode *inode, struct file *file, \
					unsigned int cmd, unsigned long arg
#endif
#define __LDDK_MMAP_PARAM 	struct file *file, struct vm_area_struct * vma
#define __LDDK_OPEN_PARAM 	struct inode *inode, struct file *file 
#define __LDDK_FLUSH_PARAM	struct file *file 
#define __LDDK_CLOSE_PARAM 	struct inode *inode, struct file *file 
#define __LDDK_FSYNC_PARAM 	struct file *file, struct dentry *dentry, \
					int datasync
#define __LDDK_FASYNC_PARAM 	int fd, struct file *file, int count 
#define __LDDK_CCHECK_PARAM 	kdev_t dev
#define __LDDK_REVAL_PARAM 	kdev_t dev

#define __LDDK_MINOR MINOR(file->f_dentry->d_inode->i_rdev)


#ifndef SLOW_DOWN_IO
# define SLOW_DOWN_IO __SLOW_DOWN_IO
#endif





/************************************************************************/
#include "debug.h"
/************************************************************************/
#ifdef __KERNEL__

extern __LDDK_READ_TYPE   can_read   (__LDDK_READ_PARAM);
extern __LDDK_WRITE_TYPE can_write (__LDDK_WRITE_PARAM);
extern __LDDK_SELECT_TYPE can_select (__LDDK_SELECT_PARAM);
extern __LDDK_IOCTL_TYPE  can_ioctl  (__LDDK_IOCTL_PARAM);
extern __LDDK_OPEN_TYPE   can_open   (__LDDK_OPEN_PARAM);
extern __LDDK_CLOSE_TYPE can_close (__LDDK_CLOSE_PARAM);
extern __LDDK_FASYNC_TYPE can_fasync (__LDDK_FASYNC_PARAM);

#endif 


/*---------- Default Outc and other value for some known boards
 * this depends on the transceiver configuration
 * some embedded CAN controllers even don't have this configuration option
 * 
 * the AT-CAN-MINI board uses optocoupler configuration as denoted
 * in the Philips application notes, so the OUTC value is 0xfa
 *
 * CAN_OUTC_VAL  - is a register value, mostly used for SJA1000
 * IO_Modeli     'p'  port IO
 *               'm'  memory IO
 *               's'  spi bus connected
 *               'i'  indexed meory or indexed port access
 */

#if   defined(ATCANMINI_BASIC) || defined(ATCANMINI_PELICAN)
# define CAN_OUTC_VAL           0xfa
# define IO_MODEL		'p'
# define STD_MASK		0xFFFFFFFF 
# include "sja1000.h"

#include "port_atcanmini.h"

#elif defined(IME_SLIMLINE)
# define CAN_OUTC_VAL           0xda
# define IO_MODEL		'm'
# define STD_MASK		0xFFFFFFFF 
# include "sja1000.h"

#elif defined(CPC_PCI) || defined(CPC_PCI2)
# define CAN_OUTC_VAL           0xda
# define IO_MODEL		'm'
# define STD_MASK		0xFFFFFFFF 
# include "sja1000.h"

# include "ems_pci.h"
extern void board_clear_interrupts(int minor);

#elif defined(JANZ_PCIL)
# define CAN_OUTC_VAL           0xfa
# define IO_MODEL		'm'
# define STD_MASK		0xFFFFFFFF 
# include "sja1000.h"

# include "janz_pcil.h"
extern void board_clear_interrupts(int minor);

#elif defined(KVASER_PCICAN)
# define CAN_OUTC_VAL           0xda
# define IO_MODEL		'p'
# define STD_MASK		0xFFFFFFFF 
# include "sja1000.h"

# include "kvaser_pci.h"
extern void disable_pci_interrupt(unsigned int base);
extern void board_clear_interrupts(int minor);


#elif defined(IXXAT_PCI03)
# define CAN_OUTC_VAL           0x5e
# define IO_MODEL		'm'
# define STD_MASK		0xFFFFFFFF 
# include "sja1000.h"

#elif defined(CC_CANPCI)
# define CAN_OUTC_VAL           0xfa
# define IO_MODEL		'p'
# define STD_MASK		0xFFFFFFFF 
# include "sja1000.h"

# include "cc_pci.h"
/* extern void board_clear_interrupts(int minor); */

#elif defined(PCM3680)
# define CAN_OUTC_VAL           0x5e
# define IO_MODEL		'm'
# define STD_MASK		0xFFFFFFFF 
# include "sja1000.h"
# include "pcm3680.h"

#elif defined(PCM9890)
# define CAN_OUTC_VAL           0xfa
# define IO_MODEL		'm'
# define STD_MASK		0xFFFFFFFF 
# include "sja1000.h"
# include "pcm9890.h"

#elif defined(CPC104)
# define CAN_OUTC_VAL           0xda
# define IO_MODEL		'm'
# define STD_MASK		0xFFFFFFFF 
# include "sja1000.h"

#elif defined(CPC_PCM_104)
  /* dual board configuration */
  /* fortunately both boards use the same settings */
# define CAN_OUTC_VAL           0xda
# define IO_MODEL		'm'
# define STD_MASK		0xFFFFFFFF 
# include "sja1000.h"

#elif defined(CCPC104)
# define CAN_OUTC_VAL           0xfa
# define IO_MODEL		'm'
# define STD_MASK		0xFFFFFFFF 
# include "sja1000.h"


#elif defined(MMC_SJA1000)
# define CAN_OUTC_VAL           0xfa
# define IO_MODEL		'i'
# define STD_MASK		0xFFFFFFFF 
# define CAN_IRQ_PIN		AT91_PIN_PC12
# include "sja1000.h"
# include "mmc.h"
# include <linux/platform_device.h>
# include <asm/gpio.h>
extern void init_mmc_hw(void);
extern void board_clear_interrupts(int minor);

#elif defined(MULTILOG_SJA1000)
# define CAN_OUTC_VAL           0xfa
# define IO_MODEL		'i'
# define STD_MASK		0xFFFFFFFF 
# define CAN_IRQ_PIN1		AT91_PIN_PA2
# define CAN_IRQ_PIN2           AT91_PIN_PA3
# include "sja1000.h"
# include "multilog.h"
# include <linux/platform_device.h>
# include <asm/gpio.h>
extern void init_mmc_hw(void);
extern void board_clear_interrupts(int minor);

#elif defined(MCF5282)
# define CAN_OUTC_VAL           0x5e
# define IO_MODEL		'm'
# define STD_MASK		0
# include "mcf5282.h"
/* can_mcf5282funcs.c */
void mcf_irqreset(void);
void mcf_irqsetup(void);

#elif defined(IMX35) || defined(IMX25) || defined(IMX28)
# define CAN_OUTC_VAL           0x00
# define IO_MODEL		'm'
# define STD_MASK		0x00000000
# include "imx35flexcan.h"
# include "imx35.h"
/* imx35funcs.c */
void imx_irqreset(void);
void imx_irqsetup(void);
void init_imx35_hw(int n);
void exit_imx35_hw(void);
void board_clear_interrupts(int minor);

#elif defined(AD_BLACKFIN)
# define CAN_OUTC_VAL           0x00
# define IO_MODEL		'm'
# define STD_MASK		0
# include "bf537.h"

#elif defined(ATMEL_SAM9)
# define CAN_OUTC_VAL           0x00
# define IO_MODEL		'm'
# define STD_MASK		0
# include "at9263.h"

#elif defined(VCMA9)
# define CAN_OUTC_VAL           0x1a
# define IO_MODEL		'm'
# define STD_MASK		0xFFFFFFFF 
# include "sja1000.h"

#elif defined(RASPI)
# define IO_MODEL		's'
# define CAN_OUTC_VAL           0x00
# define STD_MASK		0x00000000 
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <asm/gpio.h>

#define CAN_SPI_INTERRUPT_PIN	AT91_PIN_PB22
/* #define CAN_SPI_CS0		AT91_PIN_PA3 */
# include "mcp251x.h"
# include "mcp2515.h"

extern struct spi_driver mcp251x_can_driver;
extern int loopback;
extern int enable_can_dma;

#elif defined(MCP2515SPI)
# define IO_MODEL		's'
# define CAN_OUTC_VAL           0x00
# define STD_MASK		0x00000000 
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <asm/gpio.h>

#define CAN_SPI_INTERRUPT_PIN	AT91_PIN_PB22
/* #define CAN_SPI_CS0		AT91_PIN_PA3 */
# include "mcp251x.h"
# include "mcp2515.h"

extern struct spi_driver mcp251x_can_driver;
extern int loopback;
extern int enable_can_dma;

#elif defined(SSV_MCP2515)
# define IO_MODEL		's'
# define CAN_OUTC_VAL           0x00
# define STD_MASK		0x00000000 
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <asm/gpio.h>

#define CAN_SPI_INTERRUPT_PIN	AT91_PIN_PD2
#define CAN_SPI_CS0		AT91_PIN_PA3
# include "mcp251x.h"
# include "mcp2515.h"

extern struct spi_driver mcp251x_can_driver;
extern int loopback;
extern int enable_can_dma;

#elif defined(AuR_MCP2515)
# define IO_MODEL		's'
# define CAN_OUTC_VAL           0x00
# define STD_MASK		0x00000000 
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <asm/gpio.h>

#define CAN_SPI_INTERRUPT_PIN	AT91_PIN_PB22
#define CAN_SPI_CS0		AT91_PIN_PB3
/* #define CAN_SPI_CS0		AT91_PIN_PB19 */
# include "mcp251x.h"
# include "mcp2515.h"

extern struct spi_driver mcp251x_can_driver;
extern int loopback;
extern int enable_can_dma;

#elif defined(ECAN1000)
# define IO_MODEL       'i'
# define CAN_OUTC_VAL   0xda
# define STD_MASK       0xFFFFFFFF
# include "sja1000.h"
# include "ecan1000.h"

#elif defined(ZEDBOARD)
# define IO_MODEL       'm'
# define CAN_OUTC_VAL   0x00
# define STD_MASK       0xFFFFFFFF

//#include <mach/zynq_soc.h>
#include "zedboard.h"
#include "xcanps.h"

void zynq_irqreset(void);
void zynq_irqsetup(void);
void init_zynq_hw(int n);
void exit_zynq_hw(void);

#else 
# define CAN_OUTC_VAL           0x00
# define IO_MODEL		'm'
# define STD_MASK		0xFFFFFFFF 
# include "sja1000.h"
/* #error no CAN_OUTC_VAL */
#endif

/************************************************************************/
#include "can4linux.h"
/************************************************************************/
 /* extern volatile int irq2minormap[]; */
 /* extern volatile int irq2pidmap[]; */
#if defined (CPC_PCI2)
extern void __iomem *Can_pitapci_control[];
#else
extern upointer_t Can_pitapci_control[];
#endif

extern void __iomem *Can_ob_control[];

/* extern void __iomem *Can_pitapci_control[]; */
extern struct	pci_dev *Can_pcidev[];

/* number of supported CAN channels */
#ifndef MAX_CHANNELS
# define MAX_CHANNELS 4
#endif

/* number of processes allowed to open a CAN channel */
#ifndef CAN_MAX_OPEN
# define CAN_MAX_OPEN 1
#endif

/* Max number of CAN frames for RX and TX FIFO */
/* A typical can frame contains around 24 bytes + the data bytes
 * resulting in 32 byte per frame for the traditional CAN with 8 data bytes
 * or 88 bytes for the maximum CAN FD data byte length.
 * The typical size of an RX  or TX buffer with 256 entries results in 8KiB.
 * The driver compiled for 4 CAN each foreseen for 4 open processes
 * consumes already 128KiB.
 * CAN FD in this case consumes 352KiB.
 */
#ifndef MAX_BUFSIZE
# ifdef CANFD
#  define MAX_BUFSIZE 128
# else
#  define MAX_BUFSIZE 256
# endif
#endif

/* if not given a module postfix set to empty string to prevent syntax errors */
#if !defined  CAN_MODULE_POSTFIX
# define CAN_MODULE_POSTFIX ""
#endif 

/* highest supported interrupt number */
#define MAX_IRQNUMBER	32767 /* SHRT_MAX */
/* max number of bytes used for the device name in the inode 'can%d' */
#define MAXDEVNAMELENGTH 10

#define BUF_EMPTY    0
#define BUF_OK       1
#define BUF_FULL     BUF_OK
#define BUF_OVERRUN  2
#define BUF_UNDERRUN 3


typedef struct {
	int head;
        int tail;
        int status;
	int active;
	char free[MAX_BUFSIZE];
        canmsg_t data[MAX_BUFSIZE];
 } msg_fifo_t;


struct _instance_data {
	int rx_index;				/* index of receive queue used */
	int su;					/* first process opened the device */
						/* it will have 'super user' privileges */ 
	wait_queue_head_t CanRxWait;
    };


#ifdef CAN_USE_FILTER
 #define MAX_ID_LENGTH 11
 #define MAX_ID_NUMBER (1<<11)

 typedef struct {
	unsigned    use;
	unsigned    signo[3];
	struct {
		unsigned    enable    : 1;
		unsigned    timestamp : 1;
		unsigned    signal    : 2;
		canmsg_t    *rtr_response;
	} filter[MAX_ID_NUMBER];
 } msg_filter_t;



 extern msg_filter_t Rx_Filter[];
#endif

extern msg_fifo_t Tx_Buf[MAX_CHANNELS];
extern msg_fifo_t Rx_Buf[MAX_CHANNELS][CAN_MAX_OPEN];
extern canmsg_t last_Tx_object[MAX_CHANNELS];    /* used for selfreception of messages */


extern atomic_t Can_isopen[MAX_CHANNELS];   /* device minor already opened */


#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18) 
extern int Can_RequestIrq(int minor, int irq, 
	irqreturn_t (*handler)(int, void *));
#else
extern int Can_RequestIrq(int minor, int irq, 
	irqreturn_t (*handler)(int, void *, struct pt_regs *));
#endif


extern wait_queue_head_t CanWait[MAX_CHANNELS][CAN_MAX_OPEN];
extern wait_queue_head_t CanOutWait[MAX_CHANNELS];
extern int CanWaitFlag[MAX_CHANNELS][CAN_MAX_OPEN];

extern spinlock_t write_splock[MAX_CHANNELS];



#if defined (CAN_PORT_IO)
/* can_base holds a pointer, in order to use the acess macros
   for adress calculation with the given CAN register structure.
   before acessing the registers, the pointer is csted to int */
extern unsigned char *can_base[];
#else
/* Memory access to CAN */
extern void __iomem *can_base[];
#endif /* defined (CAN_PORT_IO) */



extern unsigned int   can_range[];
#endif		/*  __KERNEL__ */

extern int virtual;	/* only virtuual CAN */

extern int IRQ_requested[];
extern int Can_minors[];			/* used as IRQ dev_id */
extern int selfreception[MAX_CHANNELS][CAN_MAX_OPEN];			/* flag  1 = On */
extern int use_timestamp[MAX_CHANNELS];			/* flag  1 = On */
extern int wakeup[];				/* flag  1 = On */
extern int listenmode;				/* true for listen only */

/************************************************************************/
#define LDDK_USE_SYSCTL 1
#ifdef __KERNEL__
#include <linux/sysctl.h>

extern ctl_table can_sysctl_table[];


/* ------ /proc/sys/dev/Can accessible global variables */

extern char version[];
extern char Chipset[];
extern char IOModel[];
extern  int IRQ[];
extern  upointer_t Base[];
extern  int Baud[];
extern  int Speedfactor[];
extern  unsigned int Clock;
extern  unsigned int framelength;

#if defined(IMX35) || defined(IMX25) || defined(IMX28)
extern  unsigned int AccCode[FLEXCAN_MAX_FILTER][MAX_CHANNELS];
extern  unsigned int AccMask[FLEXCAN_MAX_FILTER][MAX_CHANNELS];
#else
extern  unsigned int AccCode[];
extern  unsigned int AccMask[];
#endif

extern  int Timeout[];
extern  int Outc[];
extern  int TxErr[];
extern  int RxErr[];
extern  int TxErrCounter[];
extern  int RxErrCounter[];
extern  int ArbitrationLost[];
extern  int Overrun[];
extern  unsigned int dbgMask;
extern  int Cnt1[];
extern  int Cnt2[];
extern	int erroractive[];	/* CAN controller state */

/* Hardware dependant sysctl entries */
#if defined JANZ_PCIL
 /* the hex switch on the board */
extern	int proc_board_id[];
#endif
/* /end Hardware dependant sysctl entries */

/* this is a global module parameter */
extern  int errint;
 
enum {
	CAN_SYSCTL_VERSION  = 1,
	CAN_SYSCTL_CHIPSET  = 2,
	CAN_SYSCTL_IOMODEL  = 3,
	CAN_SYSCTL_IRQ      = 4,
	CAN_SYSCTL_BASE     = 5,
	CAN_SYSCTL_BAUD     = 6,
	CAN_SYSCTL_ACCCODE  = 7,
	CAN_SYSCTL_ACCMASK  = 8,
	CAN_SYSCTL_TIMEOUT  = 9,
	CAN_SYSCTL_OUTC     = 10,
	CAN_SYSCTL_TXERR    = 11,
	CAN_SYSCTL_RXERR    = 12,
	CAN_SYSCTL_TXERRCNT = 13,
	CAN_SYSCTL_RXERRCNT = 14,
	CAN_SYSCTL_OVERRUN  = 15,
	CAN_SYSCTL_DBGMASK  = 16,
	CAN_SYSCTL_CNT1     = 17,
	CAN_SYSCTL_CNT2     = 18,
	CAN_SYSCTL_CLOCK
};
 
#endif
/************************************************************************/



#ifndef CAN_MAJOR
#define CAN_MAJOR 91
#endif

extern int Can_errno;

#ifdef USE_LDDK_RETURN
#define LDDK_RETURN(arg) DBGout();return(arg)
#else
#define LDDK_RETURN(arg) return(arg)
#endif


/************************************************************************/
/* function prototypes */
/************************************************************************/
extern int CAN_ChipReset(int);
extern int CAN_SetTiming(int, int);
extern int CAN_StartChip(int);
extern int CAN_StopChip(int);
#if defined(IMX35) || defined(IMX25) || defined(IMX28) 
extern int CAN_SetMask(int, int, unsigned int, unsigned int);
#else
extern int CAN_SetMask(int, unsigned int, unsigned int);
#endif
extern int CAN_SetOMode(int,int);
extern int CAN_SetListenOnlyMode(int, int);
extern int CAN_GetMessage (int minor, canmsg_t *rx);
extern int CAN_SendMessage(int minor, canmsg_t *tx);
extern int CAN_SetBTR(int, int, int);
extern void CAN_control_led(int minor, Command_par_t * argp);
extern void can_control_termination(int minor, Command_par_t * argp);
extern int can_GetStat(struct inode *inode, struct file *file,
	CanStatusPar_t *stat);

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18) 
extern irqreturn_t CAN_Interrupt( int irq, void *dev_id);
#else
extern irqreturn_t CAN_Interrupt( int irq, void *dev_id, struct pt_regs *);
#endif

extern int CAN_VendorInit(int);
extern int CAN_Release(int);

extern void register_systables(void);
extern void unregister_systables(void);


/* util.c */
extern int Can_RxFifoInit(int minor, int fifo);
extern int Can_TxFifoInit(int minor);
extern int Can_FilterCleanup(int minor);
extern int Can_FilterInit(int minor);
extern int Can_FilterMessage(int minor, unsigned message, unsigned enable);
extern int Can_FilterOnOff(int minor, unsigned on);
extern int Can_FilterSigNo(int minor, unsigned signo, unsigned signal);
extern int Can_FilterSignal(int minor, unsigned id, unsigned signal);
extern int Can_FilterTimestamp(int minor, unsigned message, unsigned stamp);
extern int Can_FreeIrq(int minor, int irq );
extern int Can_WaitInit(int minor);
extern void Can_StartTimer(unsigned long v);
extern void Can_StopTimer(void);
extern void Can_TimerInterrupt(unsigned long unused);
extern void can_dump(int minor);
extern void dump_CAN(unsigned long adress, int n, int offset);
#if defined(ZEDBOARD)
extern void CAN_register_dump(int minor, int n);
#else
extern void CAN_register_dump(int minor);
#endif
extern void CAN_object_dump(int minor, int object);
extern void print_tty(const char *fmt, ...);
void get_timestamp(int minor, struct timeval *timestamp);
//extern inline unsigned int adjustlength(int length);
static inline unsigned int adjustlength(int length)
{
int i;
static const unsigned int  t[8] = {
   8, 12, 16, 20, 24, 32, 48, 64 };

   if(length > 64) length = 64;
    i = 0;
    while(length > t[i++]);
    i -= 1;
    i == 0 ?
	(length = length)
	:
	(length = t[i]);
    return length;
}
// extern inline unsigned int len2dlc(unsigned int length);
/**
*
\brief  convert byte number to a valid data length code 
 
    dlc	    bytes
    9	    12
    10	    16
    11	    20
    12	    24
    13	    32
    14	    48
    15	    64 */
static inline unsigned int len2dlc(unsigned int cnt)
{   
unsigned  int i;

static const unsigned int  t[7][2] = {
	{  9, 12},
	{ 10, 16},
	{ 11, 20},
	{ 12, 24},
	{ 13, 32},
	{ 14, 48},
	{ 15, 64}
    };

    if( cnt <= 8) {
	return cnt;
    }
    if( cnt > 64) {
	 cnt = 64;
    }
    for(i = 0; i <= 7; i++) {
	if( t[i][1] >= cnt) {
	    break;
	}
    }
    return  t[i][0];
}

#if defined(CAN_PORT_IO) 
extern int controller_available(unsigned long a, int offset);
#else
# if defined(JANZ_PCIL)
extern int controller_available(void __iomem *ptr, int offset);
# else
extern int controller_available(upointer_t address, int offset);
# endif
#endif

// FIXME
#if defined(MCP2515SPI)
# ifdef RASPI
//static int __devinit mcp251x_can_probe(struct spi_device *spi);
//static int __devexit mcp251x_can_remove(struct spi_device *spi);

# else
#  if defined(CANSPI_USEKTHREAD)

#  else
//extern int __devinit mcp251x_can_probe(void);
//extern int __devexit mcp251x_can_remove(void);
#  endif
# endif
#endif

/* PCI support */
extern int pcimod_scan(void);

/* debug support */
extern void init_measure(void);
extern void set_measure_pin(void);
extern void reset_measure_pin(void);

#ifndef pci_pretty_name
#define pci_pretty_name(dev) ""
#endif


/* ----------------------------------------
 *
 * TARGET specific function declarations
 *
 * ---------------------------------------
 */

/* can_82c200funcs.c */
extern int CAN_ShowStat (int board);





/*________________________E_O_F_____________________________________________*/
