/* mcp2515funcs
 * CAN bus driver for Microchip 251x CAN Controller with SPI Interface
 *
 * can4linux -- LINUX CAN device driver source
 * 
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 *
 * 
 * Copyright (c) 2008 port GmbH Halle/Saale
 * (c) 2008-2013 Heinz-Juergen Oertel (hj.oertel@t-online.de)
 *
 * Large parts are derived from code based on
 * Copyright 2007 Raymarine UK, Ltd. All Rights Reserved.
 * Written under contract by:
 *   Chris Elston, Katalix Systems, Ltd.
 * which in turn is based on
 * Microchip MCP251x CAN controller driver written by
 * David Vrabel, Copyright 2006 Arcom Control Systems Ltd.
 *
 * Large parts using direct register access to the SPI controller
 * to impover speed/performance where contributed by M.Hasewinkel 
 *
*/

/*
 * Notes:
 * This driver interacts with the SPI subsystem.
 * - To the SPI subsystem it is a 'protocol driver'
 *
 * Because it is an SPI device, it's probing is handled via the SPI device
 * mechanisms (which are very similar to platform devices).
 *
 */

/*
 * Platform data/file currently not used
 * Platform file can specify something like:
 *
 *  	static struct mcp251x_platform_data mcp251x_info = {
 *		.oscillator_frequency = 19000000,
 *		.board_specific_setup = myboard_mcp251x_initfunc,
 *		.device_reset = myboard_mcp251x_reset,
 *		.transceiver_enable = NULL,
 *	};
 *
 *	static struct spi_board_info spi_board_info[] __initdata = {
 *	{
 *		.modalias	= "mcp251x",
 *		.platform_data	= &mcp251x_info,
 *		.irq		= 10,
 *		.max_speed_hz	= 8000000,
 *		.bus_num	= 1,
 *		.chip_select	= 0
 *	};
 *
 * (See Documentation/spi/spi-summary for more info)
 *
 * The first implementation was done for an ATMEL AT91SAm9263 board
 * see at arch/arm/mach-at91/board-sbc9263.c
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include "defs.h"
#include "linux/delay.h"

#ifdef RASPI
# include <linux/spi/spi.h>
#else
# include <asm/arch/at91_pmc.h>
# include "atmel_spi.h"
#endif

#include <linux/clk.h>


/* Defining DEBUG before including <linux/device.h>
enables dev_dbg to work, should be done in the Makefile
*/
/* #define DEBUG */
#include <linux/device.h>

//#define CAN_FILTERSUPPORT       1       /* not usable for CANopen devices */
#undef CAN_FILTERSUPPORT

#define CPUMCLK         96      /* 96 MHz */
#define SPICANCLK       10      /* max 10 MHz */

#define MAX_PRINTK_LINES	10	/* limit problem reports */

//#define SPIIRQ_IRQF_DISABLED    1     /* disable IRQs in ISR handler */
#undef SPIIRQ_IRQF_DISABLED

//#define SPIIRQ_TIME_MEASUREMENT 1       /* DEBUG: ISR time measurement */
#undef SPIIRQ_TIME_MEASUREMENT

#define SPIDEBUG	1
#undef SPIDEBUG

#if defined(SSV_MCP2515) || defined(AuR_MCP2515)
# if 1
#  define updownhelper(x) unsigned long L_flags
#  define up(x)           do { local_irq_restore(L_flags); } while (0)
#  define down(x)         do { local_irq_save(L_flags); } while (0)
# else
#  define updownhelper(x) do {} while (0)
#  define up(x)           do {} while (0)
#  define down(x)         do {} while (0)
# endif
#endif

#if defined(MCP2515SPI)
#  define updownhelper(x) do {} while (0)    /* don't need it */
#endif

#if defined(SSV_MCP2515)
# define IRQREGDEVNAME "CanSpi"
#elif defined(AuR_MCP1515)
# define IRQREGDEVNAME "Can"
#elif defined(MCP2515SPI)
# define IRQREGDEVNAME "Can"
#else
# error "#define IRQREGDEVNAME"
#endif

/* bit timing table */
u16 CanTiming[10][2]={
	/*    CNF1    ,  CNF3 << 8 +CNF2 */
	{ CAN_BCR1_10K,  CAN_BCR2_10K  },
	{ CAN_BCR1_20K,  CAN_BCR2_20K  },
	{ CAN_BCR1_50K,  CAN_BCR2_50K  },
	{ CAN_BCR1_100K, CAN_BCR2_100K },
	{ CAN_BCR1_125K, CAN_BCR2_125K },
	{ CAN_BCR1_250K, CAN_BCR2_250K },
	{ CAN_BCR1_500K, CAN_BCR2_500K },
	{ CAN_BCR1_800K, CAN_BCR2_800K },
	{ CAN_BCR1_1000K,CAN_BCR2_1000K} };

int enable_can_dma = 0;
struct spi_device *spidevice[MAX_CHANNELS]; 
int listenmode = 0;		/* true if listen only mode (mcp2515 only) */

/* We need possibly a structure for the driverdata */
struct mcp251x_priv realone;
#define SPIIOMEM ((&realone)->spi_iomem)

struct device_driver mcp251x_priv_drivername = { "can4linuxspi" };

#if !defined(MCP2515SPI)
int at91_spihw_init(void)
{
int minor = -1;

    struct mcp251x_priv *priv = &realone;
    DBGin();

    /* init for device printk */
    (&priv->dev)->driver =  &mcp251x_priv_drivername;
    /* strcpy((&priv->dev)->bus, "spihw0.1"); */
    dev_set_name(&priv->dev, "spihw0.2");

#if !defined(MCP2515SPI)    /* than initialize SPI pins ! */ 
    /* Chip select */
    at91_set_gpio_output(CAN_SPI_CS0, 1);	    /* SPI0_CS0 */

    /* we need to set these lines ?? already initialized in the SPI driver*/
    /* now set the MISO, MOSI and CLK Pins */
#if defined(SSV_MCP2515) 
    at91_set_B_periph(AT91_PIN_PA0, 0);     /* SPI0_MISO */
    at91_set_B_periph(AT91_PIN_PA1, 0);     /* SPI0_MOSI */
    at91_set_B_periph(AT91_PIN_PA2, 0);     /* SPI0_SPCK */

    /* does SSV need this one ? */
    at91_set_gpio_output(AT91_PIN_PA5, 1);  /* SPI0_CS0 */
    at91_sys_write(AT91_PMC_PCER, 1 << 14);

#elseif defined(AuR_MCP1515)
    at91_set_A_periph(AT91_PIN_PB0, 0);     /* SPI0_MISO */
    at91_set_A_periph(AT91_PIN_PB1, 0);     /* SPI0_MOSI */
    at91_set_A_periph(AT91_PIN_PB2, 0);     /* SPI0_SPCK */

    at91_sys_write(AT91_PMC_PCER, 1 << 14);
#else
#error "please check your Target definition, must be SSV_MCP2515 or AuR_MCP1515"
#endif
    /* at91_clock_associate("spi0_clk", NULL, "spi_clk"); */
    {
	struct clk *clk = clk_get(NULL, "spi0_clk");

	if (!clk ) {
	    printk("Can: can't get clock\n");

	}
	/* clk->function = func; */
	/* clk->dev = dev; */
    }

    /* Request the controllers address space */
    if(NULL == request_mem_region(0xfffa4000ul, 0x4000, "SPI-CAN-IO")) {
	printk("Request_mem_region CAN-IO failed at %lx\n",
	    0xfffa4000ul);
	return -EBUSY;
    }


    SPIIOMEM = ioremap(0xfffa4000ul, 0x4000);
    if (!SPIIOMEM) {
        dev_err(&priv->dev, "ioremap() for SPI mem failed.\n");
        return ~0;
    }
    dev_info(&priv->dev, "ioremap: 0x%p\n", SPIIOMEM);
#endif

#if defined(AuR_MCP2515)
#if 0
/* changing CSR0 to CSR2 lead to a kernel oops,
   device is properly configured by spidev anyway,
   and the future code will get rid of direct SPI-register-writing anyway!
 */
    /* initializing  SPI interface */
    spi_writel(SPIIOMEM, CR, SPI_BIT(SPIEN));
    spi_writel(SPIIOMEM, MR, SPI_BIT(MSTR) | SPI_BIT(MODFDIS));


    spi_writel(SPIIOMEM, CSR2, SPI_BIT(NCPHA) |
                               SPI_BIT(CSAAT) |
                               SPI_BF(SCBR, CPUMCLK / SPICANCLK + 1)
                               | SPI_BF(BITS, 0)); /* 8bit tx */
#endif
#endif

#if defined(SSV_MCP1515)

    /* initializing  SPI interface */
    spi_writel(SPIIOMEM, CR, SPI_BIT(SPIEN));
    spi_writel(SPIIOMEM, MR, SPI_BIT(MSTR) | SPI_BIT(MODFDIS));

    spi_writel(SPIIOMEM, CSR0, SPI_BIT(NCPHA) |
                               SPI_BIT(CSAAT) |
                               SPI_BF(SCBR, CPUMCLK / SPICANCLK + 1)
                               | SPI_BF(BITS, 0)); /* 8bit tx */
    spi_writel(SPIIOMEM, CSR1, SPI_BIT(NCPHA) |
                               SPI_BIT(CSAAT) |
                               SPI_BF(SCBR, CPUMCLK / SPICANCLK + 1)
                               | SPI_BF(BITS, 8)); /* 16bit tx */
#endif
    DBGout();
    return 0;
}

void at91_spihw_deinit(void)
{
    if (SPIIOMEM) {
        iounmap(SPIIOMEM);
    }
}

/* SPI read-write one byte, without setting CS */
static inline unsigned char at91_spihw_rw_data8(unsigned char data)
{
int loopcount = 10000;

    while (!(spi_readl(SPIIOMEM, SR) & SPI_BIT(TDRE)) && (--loopcount)) ;
    if (loopcount <= 0) {
	printk(KERN_ERR
	"loop count exceeded (#1) while waiting for SPI, SR is %08x\n",
	spi_readl(SPIIOMEM, SR));
	return -1;
    }

    spi_writel(SPIIOMEM, TDR, data & 0xFF);

    while (!(spi_readl(SPIIOMEM, SR) & SPI_BIT(RDRF)) && (--loopcount)) ;
    if (loopcount <= 0) {
	printk(KERN_ERR
	"loop count exceeded (#2) while waiting for SPI, SR is %08x\n",
	spi_readl(SPIIOMEM, SR));
	return -1;
    }

    data = spi_readl(SPIIOMEM, RDR) & 0xff;
    return data;
}

/* SPI read-write n bytes, with activating CS  */
static inline void at91_spihw_rwn(
		    unsigned char *out,
		    unsigned char *in,
		    unsigned char len
		    )
{
int i = 0;

    /* printk("%s()\n", __func__); */

    at91_set_gpio_output(CAN_SPI_CS0, 0);
    for (i = 0; i < len; i++) {
	in[i] = at91_spihw_rw_data8(out[i]);
    }
    at91_set_gpio_output(CAN_SPI_CS0, 1);
}


/* SPI read-write n bytes, with activating CS, but leaves it on  */
static inline void at91_spihw_rwn_CSon(
		    unsigned char *out,
		    unsigned char *in,
		    unsigned char len
		    )
{
int i = 0;

    at91_set_gpio_output(CAN_SPI_CS0, 0);
    for (i = 0; i < len; i++) {
	in[i] = at91_spihw_rw_data8(out[i]);
    }
}
#endif /* ! MCP2515SPI */

/* SPI write one byte, avtivating CS */
#if defined(MCP2515SPI)
static inline void at91_spihw_wr8(unsigned char out)
{
struct mcp251x_priv *priv = &realone;
int minor = -1;
int ret;

    DBGin();
    priv->spi_tx_buf[0] = INSTRUCTION_RESET;
    ret = spi_write(priv->spi, priv->spi_tx_buf, 1);
    if (ret < 0)
	    printk("%s(): failed: ret = %d\n", __FUNCTION__, ret);
    DBGout();
}

#else

static inline void at91_spihw_wr8(unsigned char out)
{
    at91_set_gpio_output(CAN_SPI_CS0, 0);
    at91_spihw_rw_data8(out);
    at91_set_gpio_output(CAN_SPI_CS0, 1);
}
#endif



#if defined(MCP2515SPI)
static u8 mcp251x_read_reg(/* struct spi_device *spi,*/ uint8_t reg)
{
struct mcp251x_priv *priv = &realone;
struct spi_device *spi = priv->spi;
struct spi_transfer t = {
	    .tx_buf = priv->spi_tx_buf,
	    .rx_buf = priv->spi_rx_buf,
	    .len = 3,
	    .cs_change = 0,
    };

struct spi_message m;
u8 val = 0;
int ret;
/* int minor = 0; */

    /* DBGin(); */
    /* dev_dbg(&spi->dev, "%s()\n", __FUNCTION__); */

    down(&priv->spi_lock);

    priv->spi_tx_buf[0] = INSTRUCTION_READ;
    priv->spi_tx_buf[1] = reg;

    spi_message_init(&m);

#if 0   /* enable if dma is enabled */
    if (enable_can_dma) {
	    t.tx_dma = priv->spi_tx_dma;
	    t.rx_dma = priv->spi_rx_dma;
	    m.is_dma_mapped = 1;
    }
#endif
    spi_message_add_tail(&t, &m);

    /* start the transfer and wait for finishing it 
     * To be in _sync_ ith the requests
     */
    ret = spi_sync(spi, &m);
    if (ret < 0) {
	    dev_dbg(&spi->dev, "%s: failed: ret = %d\n", __FUNCTION__, ret);
    } else
	    val = priv->spi_rx_buf[2];

    up(&priv->spi_lock);

    /* dev_info(&spi->dev, "%s: ret = 0x%0x\n", __FUNCTION__, val); */
    /* DBGout(); */
    return val;
}
#else
/* SPI read one byte, one MCP1525 register */
static u8 mcp251x_read_reg(uint8_t reg)
{
	struct mcp251x_priv *priv = &realone;
	u8 val = 0;
	updownhelper();

	down(&priv->spi_lock);

	priv->spi_tx_buf[0] = INSTRUCTION_READ;
	priv->spi_tx_buf[1] = reg;
	
        at91_spihw_rwn(priv->spi_tx_buf, priv->spi_rx_buf, 3);
	val = priv->spi_rx_buf[2];

	up(&priv->spi_lock);

	return val;
}
#endif /* MCP2515SPI */


/* SPI write one byte, one MCP1525 register */
#if defined(MCP2515SPI)

static void mcp251x_write_reg(/* struct spi_device *spi,*/ u8 reg, uint8_t val)
{

    struct mcp251x_priv *priv = &realone;
    struct spi_device *spi = priv->spi;
    struct spi_transfer t = {
		.tx_buf = priv->spi_tx_buf,
		.rx_buf = priv->spi_rx_buf,
		.len = 3,
		.cs_change = 0,
	};
    struct spi_message m;
    int ret;
#if 0  /* enable debug message */
    dev_info(&spi->dev, "%s: reg = 0x%02x, val = 0x%0x\n", __FUNCTION__,
		reg, val);
#endif
    down(&priv->spi_lock);

    priv->spi_tx_buf[0] = INSTRUCTION_WRITE;
    priv->spi_tx_buf[1] = reg;
    priv->spi_tx_buf[2] = val;

    spi_message_init(&m);

#if 0   /* enable if dma is enabled */
    if (enable_can_dma) {
	    t.tx_dma = priv->spi_tx_dma;
	    t.rx_dma = priv->spi_rx_dma;
	    m.is_dma_mapped = 1;
    }
#endif
	spi_message_add_tail(&t, &m);

    /* start the transfer and wait for finishing it 
     * To be in _sync_ ith the requests
     */
    ret = spi_sync(spi, &m);

    up(&priv->spi_lock);

    if (ret < 0)
	    dev_dbg(&spi->dev, "%s: failed\n", __FUNCTION__);
}

#else /* MCP2515SPI */

static void mcp251x_write_reg(u8 reg, uint8_t val)
{
    struct mcp251x_priv *priv = &realone;


    /* printk("%s()\n", __func__); */


    updownhelper();

    down(&priv->spi_lock);

    priv->spi_tx_buf[0] = INSTRUCTION_WRITE;
    priv->spi_tx_buf[1] = reg;
    priv->spi_tx_buf[2] = val;

    at91_spihw_rwn(priv->spi_tx_buf, priv->spi_rx_buf, 3);

    up(&priv->spi_lock);
}
#endif /* MCP2515SPI */


#if defined(MCP2515SPI)

static void mcp251x_write_bits(/* struct spi_device *spi,*/ u8 reg,
                               u8 mask, uint8_t val)
{
struct mcp251x_priv *priv =  &realone;
struct spi_device *spi = realone.spi;
struct spi_transfer t = {
	.tx_buf = priv->spi_tx_buf,
	.rx_buf = priv->spi_rx_buf,
	.len = 4,
	.cs_change = 0,
};
struct spi_message m;
int ret;
int minor = -1;

	DBGin();
#if 0  /* enable to debug this function */
	dev_info(&spi->dev, "%s: reg = 0x%02x, mask = 0x%0x, val = 0x%0x\n",
		__FUNCTION__, reg, mask, val);
#endif
	down(&priv->spi_lock);

	priv->spi_tx_buf[0] = INSTRUCTION_BIT_MODIFY;
	priv->spi_tx_buf[1] = reg;
	priv->spi_tx_buf[2] = mask;
	priv->spi_tx_buf[3] = val;

	spi_message_init(&m);

#if 0   /* enable if dma is enabled */
	if (enable_can_dma) {
		t.tx_dma = priv->spi_tx_dma;
		t.rx_dma = priv->spi_rx_dma;
		m.is_dma_mapped = 1;
	}
#endif
	spi_message_add_tail(&t, &m);

	ret = spi_sync(spi, &m);

	up(&priv->spi_lock);

	if (ret < 0)
		dev_dbg(&spi->dev, "%s: failed\n", __FUNCTION__);
	DBGout();
}
#else	/* MCP2515SPI */
static void mcp251x_write_bits(u8 reg, u8 mask, uint8_t val)
{
    struct mcp251x_priv *priv =  &realone;
    updownhelper();

    down(&priv->spi_lock);

    priv->spi_tx_buf[0] = INSTRUCTION_BIT_MODIFY;
    priv->spi_tx_buf[1] = reg;
    priv->spi_tx_buf[2] = mask;
    priv->spi_tx_buf[3] = val;

    at91_spihw_rwn(priv->spi_tx_buf, priv->spi_rx_buf, 4);

    up(&priv->spi_lock);
}
#endif	/* MCP2515SPI */ 

static void mcp251x_hw_reset(int minor, struct spi_device *spi)
{
struct mcp251x_priv *priv =  &realone;

    updownhelper();
    DBGin();


#ifdef RASPI

#else
#if defined(AuR_MCP2515) || defined(MCP2515SPI)

/* oe: must be moved to some board specific source code */

    /* auch0r mod:
       we add an additional HW-reset first if this is a CANSUB-module 
       (which we know by looking at IRQ, which is then 86)
     */
    if(IRQ[minor] == 86)
    {
	printk(KERN_INFO "doing A&R specific hardware reset\n");
	at91_set_gpio_output(irq_to_gpio(85), 1);
	udelay(5);	    
	at91_set_gpio_output(irq_to_gpio(85), 0);
	udelay(5);		/* datasheet says 2us, but let's make sure...*/
	at91_set_gpio_output(irq_to_gpio(85), 1);
	udelay(100);
    }
#endif
#endif

    down(&priv->spi_lock);

    at91_spihw_wr8(INSTRUCTION_RESET);

    up(&priv->spi_lock);
    DBGout();
}

#ifdef DEBUG
int CAN_ShowStat (int board)
{
    if (dbgMask && (dbgMask & DBG_DATA)) {
    printk(KERN_INFO " %s: CTRL 0x%x,", __func__, CANin(board, CANCTRL));
    printk(          " STAT 0x%x,",  CANin(board, CANSTAT));
    printk(          " INTF 0x%x,",  CANin(board, CANINTF));
    printk(          " INTE 0x%x\n", CANin(board, CANINTE));
    printk(KERN_INFO "mask   %0x %0x %0x %0x\n",
			CANin(board, RXM0SIDH),
			CANin(board, RXM0SIDL),
			CANin(board, RXM0EID8),
			CANin(board, RXM0EID0));

    printk(KERN_INFO "filter %0x %0x %0x %0x\n",
			CANin(board, RXFnSIDH),
			CANin(board, RXFnSIDL),
			CANin(board, RXFnEID8),
			CANin(board, RXFnEID0));
    }
    return 0;
}
#endif

/* can_GetStat - read back as many status information as possible 
*
* Because the CAN protocol itself describes different kind of information
* already, and the driver has some generic information
* (e.g about it's buffers)
* we can define a more or less hardware independent format.
*
*
* exception:
* ERROR WARNING LIMIT REGISTER (EWLR)
* The SJA1000 defines a EWLR, reaching this Error Warning Level
* an Error Warning interrupt can be generated.
* The default value (after hardware reset) is 96. In reset
* mode this register appears to the CPU as a read/write
* memory. In operating mode it is read only.
* Note, that a content change of the EWLR is only possible,
* if the reset mode was entered previously. An error status
* change (see status register; Table 14) and an error
* warning interrupt forced by the new register content will not
* occur until the reset mode is cancelled again.
*/

int can_GetStat(
	struct inode *inode,
	struct file *file,
	CanStatusPar_t *stat
	)
{
unsigned int board = iminor(inode);	
msg_fifo_t *Fifo;
unsigned long flags;
int rx_fifo = ((struct _instance_data *)(file->private_data))->rx_index;

    /* Disable CAN (All !!) Interrupts */
    /* !!!!!!!!!!!!!!!!!!!!! */
    /* FIXME: better use spin lock ??? */
    local_irq_save(flags);      /* MH: no race with irqs please */
    stat->type = CAN_TYPE_MCP2515;

    stat->baud = Baud[board];
    /* printk(" STAT ST %d\n", CANin(board, canstat)); */
    stat->status = CANin(board, CANSTAT);
    /* printk(" STAT EWL %d\n", CANin(board, errorwarninglimit)); */
    stat->error_warning_limit = 0;
    stat->rx_errors = CANin(board, REC);
    stat->tx_errors = CANin(board, TEC);
    stat->error_code= 0; //CANin(board, errorcode);


    Fifo = &Rx_Buf[board][rx_fifo];
    stat->rx_buffer_size = MAX_BUFSIZE;	/**< size of rx buffer  */
    /* number of messages */
    stat->rx_buffer_used =
    	(Fifo->head < Fifo->tail)
    	? (MAX_BUFSIZE - Fifo->tail + Fifo->head) : (Fifo->head - Fifo->tail);
    Fifo = &Tx_Buf[board];
    stat->tx_buffer_size = MAX_BUFSIZE;	/* size of tx buffer  */
    /* number of messages */
    stat->tx_buffer_used = 
    	(Fifo->head < Fifo->tail)
    	? (MAX_BUFSIZE - Fifo->tail + Fifo->head) : (Fifo->head - Fifo->tail);
    /* Enable CAN Interrupts */
    /* !!!!!!!!!!!!!!!!!!!!! */
    /* restore_flags(flags); */
    local_irq_restore(flags);
    return 0;
}



int CAN_ChipReset(int minor)
{
struct spi_device *spi = spidevice[minor];
int loopcount;

    DBGin();

    /* Reset CAN controller */
    mcp251x_hw_reset(minor, spi);

    udelay(100);
    /* Put device into config mode */
    mcp251x_write_reg(CANCTRL, CANCTRL_REQOP_CONF);

    /* Wait for the device to enter config mode */
    loopcount = 1000; /* circa 10ms */
    while (((mcp251x_read_reg(CANSTAT) & 0xE0)
            != CANCTRL_REQOP_CONF) && (--loopcount)) {
	udelay(10);
    }
    if (loopcount <= 0) {
	printk(KERN_ERR
	    "loop count exceeded (#1) while waiting for can status, "
	    "CANSTAT is %02x\n", mcp251x_read_reg(CANSTAT));
	return -1;
    }

    /* Set initial baudrate */
    CAN_SetTiming(minor, Baud[minor]);

    CAN_SetMask(minor, AccCode[minor], AccMask[minor]);
    /* Enable RX0->RX1 buffer roll over */
    /* RXM  Receive Buffer Operating Mode bits are 0
            -  Receive all valid messages using either base or
            extended format identifiers that meet filter criteria
     */       
    mcp251x_write_bits(RXBCTRL(0), RXBCTRL_BUKT, RXBCTRL_BUKT);


    mcp251x_write_reg(CANCTRL, CANCTRL_REQOP_NORMAL);
    loopcount = 1000; /* circa 10ms */
    while (((mcp251x_read_reg(CANSTAT) & 0xE0) 
            != CANCTRL_REQOP_NORMAL) && (--loopcount)) {
	udelay(10);
    }
    if (loopcount <= 0) {
	printk(KERN_ERR
	    "loop count exceeded (#2) while waiting for can status, "
	    "CANSTAT is %02x\n", mcp251x_read_reg(CANSTAT));
	return -1;
    }

    /* Reset error conditions */
    mcp251x_write_reg(EFLG, 0);

    DBGout();
    return 0;
}


/*
 * Configures bit timing registers directly. Chip must be in configuration mode.
 * MCP2515 specific
 *      btr0 == CNF1
 *      btr1 == (CNF3 << 8) + CNF2 
 */
int CAN_SetBTR (int minor, int btr0, int btr1)
{
int isopen;

    DBGin();

    isopen = atomic_read(&Can_isopen[minor]);
    if (isopen > 1)  {
	DBGprint(DBG_DATA, ("refused to change bit rate, driver already in use"
	"by another process"));
	return -1;
    }
    DBGprint(DBG_DATA, ("[%d] btr0=%d, btr1=%d", minor, btr0, btr1));
    mcp251x_write_reg(CNF1, (u8)btr0);
    mcp251x_write_reg(CNF2, (u8)btr1);
    mcp251x_write_reg(CNF3, (u8)(btr1 >> 8));

    DBGout();
    return 0;
}

/*
 * Configures bit timing. Chip must be in configuration state.
 */
int CAN_SetTiming(int minor, int baud)
{
struct mcp251x_priv *priv = &realone;
int i      = 4;
int custom = 0;
int isopen;
int ret	   = -1; 

    DBGin();

    isopen = atomic_read(&Can_isopen[minor]);
    if ((isopen > 1) && (Baud[minor] != baud)) {
	DBGprint(DBG_DATA, ("isopen = %d", isopen));
	DBGprint(DBG_DATA, ("refused baud=%d already set to %d",
			baud, Baud[minor]));
    } else {

	DBGprint(DBG_DATA, ("baud=%d", baud));
	switch(baud)
	{
	    case   10: i = 0; break;
	    case   20: i = 1; break;
	    case   50: i = 2; break;
	    case  100: i = 3; break;
	    case  125: i = 4; break;
	    case  250: i = 5; break;
	    case  500: i = 6; break;
	    case  800: i = 7; break;
	    case 1000: i = 8; break;
	    default  : 
		    custom = 1;
	}

	if(custom) {
	    dev_info(&priv->dev, "%s, no custom bit register setting yet\n",
				__FUNCTION__);
	} else {
	    if((CanTiming[i][0] == 0) && (CanTiming[i][1] == 0)) {
	    	/* values not valid */
	    } else {
	    	/* write values to the three registers */
		mcp251x_write_reg(CNF1, (u8)CanTiming[i][0]);
		mcp251x_write_reg(CNF2, (u8)CanTiming[i][1]);
		mcp251x_write_reg(CNF3, (u8)(CanTiming[i][1] >> 8));
		ret = 0;  /* return value now OK */
	    }
	}
    }
    DBGout();
    return ret;
}


int CAN_StartChip(int minor)
{
int loopcount = 1000;

    DBGin();
    RxErr[minor] = TxErr[minor] = 0L;   /* empty error counters */
    /* clear interrupts */
    /* nothing to do ? */

    /* Interrupts on Rx, TX, any Status change and data overrun */
    /* The CANINTE register contains the individual interrupt enable bits
     * for each interrupt source.
     */
    mcp251x_write_reg(CANINTE, CANINTE_ERRIE |
    			       /* CANINTE_TX2IE | */
    			       /* CANINTE_TX1IE | */
    			       CANINTE_TX0IE |
    			       CANINTE_RX1IE |
    			       CANINTE_RX0IE);

    if (listenmode == 1) {
	mcp251x_write_reg(CANCTRL, CANCTRL_REQOP_LISTEN_ONLY);
	while ((mcp251x_read_reg(CANSTAT) & 0xE0) != CANCTRL_REQOP_LISTEN_ONLY
	&& (--loopcount)) {
	    udelay(10);
	}
    } else {
	mcp251x_write_reg(CANCTRL, CANCTRL_REQOP_NORMAL);
	while ((mcp251x_read_reg(CANSTAT) & 0xE0) != CANCTRL_REQOP_NORMAL
	&& (--loopcount)) {
	    udelay(10);
	}
    }

    /* Reset error conditions */
    mcp251x_write_reg(EFLG, 0);

    DBGout();
    return 0;
}


int CAN_StopChip(int minor)
{
int loopcount = 1000;
    DBGin();
    /* FIXME: Issue  CAN controller reset */

    /* Disable and clear pending interrupts */
    mcp251x_write_reg(CANINTE, 0x00);
    mcp251x_write_reg(CANINTF, 0x00);

    /* Abort All Pending Transmisson ?  */
    /* CANCTRL_ABAT */
    mcp251x_write_reg(CANCTRL, CANCTRL_REQOP_CONF /* | CANCTRL_ABAT */);
    /* Wait for the device to enter config mode */
    while ((mcp251x_read_reg(CANSTAT) & 0xE0) != CANCTRL_REQOP_CONF
	&& (--loopcount)) {
	    udelay(10);
    }

    DBGout();
    return 0;
}

/* set value of the output control register */
int CAN_SetOMode (int minor, int arg)
{
    DBGin();
	DBGprint(DBG_DATA,("[%d] outc=0x%x", minor, arg));
	printk(KERN_INFO "no \"output mode\" to set\n");
    DBGout();
    return 0;
}

/*
Listen-Only Mode
In listen-only mode, the CAN module is able to receive messages
without giving an acknowledgment.
Since the module does not influence the CAN bus in this mode
the host device is capable of functioning like a monitor
or for automatic bit-rate detection.

 must be done after CMD_START (CAN_StopChip)
 and before CMD_START (CAN_StartChip)

 MCP2515:

Listen-only mode provides a means for the MCP2515 to receive
all messages (including messages with errors) by configuring the
RXBnCTRL.RXM<1:0> bits.
The Listen-only mode is activated by setting the mode request bits
in the CANCTRL register.
    bit 7-5 REQOP: Request Operation Mode bits <2:0>
        000 = Set Normal Operation mode		CANCTRL_REQOP_NORMAL
        001 = Set Sleep mode
        010 = Set Loopback mode
        011 = Set Listen-only mode		CANCTRL_REQOP_LISTEN_ONLY
        100 = Set Configuration mode		CANCTRL_REQOP_CONF 
        All other values for REQOP bits are invalid and should not be used
*/
int CAN_SetListenOnlyMode(int minor, int arg)
{
int loopcount = 1000;

    DBGin();

    if (arg) {
	/* switch to listen only mode */
	mcp251x_write_reg(CANCTRL, CANCTRL_REQOP_LISTEN_ONLY);
	listenmode = 1;
        while ((mcp251x_read_reg(CANSTAT) & 0xE0) != CANCTRL_REQOP_LISTEN_ONLY
        && (--loopcount)) {
            udelay(10);
	}
    } else {
	/* switch to normal mode */
	mcp251x_write_reg(CANCTRL, CANCTRL_REQOP_NORMAL);
	listenmode = 0;
       while ((mcp251x_read_reg(CANSTAT) & 0xE0) != CANCTRL_REQOP_NORMAL
        && (--loopcount)) {
	    udelay(10);
	}
    }

    DBGout();
    return 0;
}

/*
The mask and filter registers can only be modified
when the MCP2515 is in Configuration mode (see
Section 10.0 "Modes of Operation").

Used is only Mask1 and Filter1 

                     Extended Frame
+---------------+-------------------------------------------------+
|ID10       ID0 | EID17                                       EID0|
+---------------+-------------------------------------------------+
     Masks and Filters apply to the entire 29-bit ID field

*/
int CAN_SetMask(int minor, unsigned int code, unsigned int mask)
{
#if CAN_FILTERSUPPORT
int n;

    DBGin();
    DBGprint(DBG_DATA, ("[%d] acc=0x%x mask=0x%x", minor, code, mask));

    mcp251x_write_reg(RXM0SIDH, (u8)(mask >> 24));
    mcp251x_write_reg(RXM0SIDL, (u8)(mask >> 16));
    mcp251x_write_reg(RXM0EID8, (u8)(mask >>  8));
    mcp251x_write_reg(RXM0EID0, (u8)(mask >>  0));

    mcp251x_write_reg(RXM1SIDH, (u8)(mask >> 24)); 
    mcp251x_write_reg(RXM1SIDL, (u8)(mask >> 16)); 
    mcp251x_write_reg(RXM1EID8, (u8)(mask >>  8)); 
    mcp251x_write_reg(RXM1EID0, (u8)(mask >>  0)); 

    for (n = 0; n < 6; n++) {
        mcp251x_write_reg(RXFnSIDH, (u8)(code >> 24));
        mcp251x_write_reg(RXFnSIDL, (u8)(code >> 16));
        mcp251x_write_reg(RXFnEID8, (u8)(code >>  8));
        mcp251x_write_reg(RXFnEID0, (u8)(code >>  0));
    }
#else
    DBGin();
    DBGprint(DBG_DATA, ("%s: not implemented\n", __FUNCTION__));
#endif

   /* put values back in global variables for sysctl */
    AccCode[minor] = code;
    AccMask[minor] = mask;

    DBGout();
    return 0;
}


/*
 CAN_SendMessage
 send a single CAN frame using SPI Blocktransfer
 */
#if defined(MCP2515SPI)
int CAN_SendMessage(int minor, canmsg_t *tx)
{
struct spi_device *spi = spidevice[minor];
struct mcp251x_priv *priv = &realone;
struct spi_transfer t = {
	.tx_buf = priv->spi_tx_buf,
	.rx_buf = priv->spi_rx_buf,
	.cs_change = 0,
	.len = 6 + CAN_FRAME_MAX_DATA_LEN,  /* 1 CTRL + 4 ID + 1 DLC */
    };
u8 *tx_buf = priv->spi_tx_buf;
struct spi_message m;
int ret;
u32 sid, eid, ext, rtr;

    DBGin();

    sid  = tx->id & CAN_SFF_MASK; /* Base format ID */
    eid  = tx->id & CAN_EFF_MASK; /* Extended format ID */
    ext  = (tx->flags & MSG_EXT) ? 1 : 0; /* Extended ID Enable */
    rtr  = (tx->flags & MSG_RTR) ? 1 : 0; /* Remote transmission */

    down(&priv->spi_lock);

    tx_buf[0] = INSTRUCTION_LOAD_TXB(TRANSMIT_OBJ);
    if(ext) {
	tx_buf[1] = (eid >> 21) & 0xff;
	tx_buf[2] = ((eid >> 13) & 0xe0) + 0x08 + ((eid & 0x30000) >> 16);
	tx_buf[3] = (eid & 0xff00) >> 8;
	tx_buf[4] = (eid & 0xff);
    } else {
	tx_buf[1] = sid >> 3;
	tx_buf[2] = (sid << 5);
    }
    tx_buf[5] = (rtr << 6) | tx->length;


    /* copy data to spi buffer */
    memcpy(tx_buf + 6, tx->data, tx->length);

    spi_message_init(&m);

#if 0   /* enable if dma is enabled */
    if (enable_can_dma) {
	    t.tx_dma = priv->spi_tx_dma;
	    t.rx_dma = priv->spi_rx_dma;
	    m.is_dma_mapped = 1;
    }
#endif
    spi_message_add_tail(&t, &m);

    ret = spi_sync(spi, &m);

    up(&priv->spi_lock);

    if (ret < 0)
	dev_dbg(&spi->dev, "%s: failed: ret = %d\n", __FUNCTION__, ret);

    /* FIXME: Should we exit with an error here? */

    /* this is the last step to tell the contrller
       sending out the frame configured with the prviuos spi
       INSTRUCTION_LOAD_TXB(TRANSMIT_OBJ)
      */
    mcp251x_write_reg(/* spi,*/ TXBCTRL(TRANSMIT_OBJ), TXBCTRL_TXREQ);


  /* 
     * Save last message that was sent.
     * Since can4linux 3.5 multiple processes can access
     * one CAN interface. On a CAN interrupt this message is copied into 
     * the receive queue of each process that opened this same CAN interface.
     */
    memcpy(
	(void *)&last_Tx_object[minor],
	(void *)tx,
	sizeof(canmsg_t));


    DBGout();
    return 0;
}

#else	/* MCP2515 */

int CAN_SendMessage(int minor, canmsg_t *tx)
{
    struct mcp251x_priv *priv = &realone;
    u8 *tx_buf = priv->spi_tx_buf;
    u32 sid, eid, ext, rtr;

    DBGin();

    updownhelper();

    /* DBGin(); */

    sid  = tx->id & CAN_SFF_MASK; /* Base format ID */
    eid  = tx->id & CAN_EFF_MASK; /* Extended format ID */
    ext  = (tx->flags & MSG_EXT) ? 1 : 0; /* Extended ID Enable */
    rtr  = (tx->flags & MSG_RTR) ? 1 : 0; /* Remote transmission */

    down(&priv->spi_lock);

    tx_buf[0] = INSTRUCTION_LOAD_TXB(TRANSMIT_OBJ);
    if(ext) {
	tx_buf[1] = (eid >> 21) & 0xff;
	tx_buf[2] = ((eid >> 13) & 0xe0) + 0x08 + ((eid & 0x30000) >> 16);
	tx_buf[3] = (eid & 0xff00) >> 8;
	tx_buf[4] = (eid & 0xff);
    } else {
	tx_buf[1] = sid >> 3;
	tx_buf[2] = (sid << 5);
    }
    tx->length = tx->length > 8 ? 8 : tx->length;
    tx_buf[5] = (rtr << 6) | tx->length;

    /* copy data to spi buffer */
    memcpy(tx_buf + 6, tx->data, tx->length);

    at91_spihw_rwn(priv->spi_tx_buf, priv->spi_rx_buf, 6 + tx->length);

    at91_spihw_wr8(MCP_RTS_TX0);
    up(&priv->spi_lock);
   /* 
     * Save last message that was sent.
     * Since can4linux 3.5 multiple processes can access
     * one CAN interface. On a CAN interrupt this message is copied into 
     * the receive queue of each process that opened this same CAN interface.
     */
    memcpy(
	(void *)&last_Tx_object[minor],
	(void *)tx,
	sizeof(canmsg_t));


    /* DBGout(); */
    return 0;
}
#endif	/* MCP2515SPI */


/*
 * Perform Vendor, that means sometimes CAN controller
 * or only board manufacturer specific initialization.
 *
 * Mainly it gets needed IO and IRQ ressources and initilaizes 
 * special hardware functions.
 *
 * This code should normally be in the CAN_VendorInit() function
 * in a TARGET specific file  target.c
 */
int CAN_VendorInit (int minor)
{
    DBGin();


    /* SPI specific Tasks:
     * register the ISR
     * enable the PIO pins wher the CAN is connected to
     *
     * CAN interrupts are later enabled in
     * CAN_StartChip(minor)
     * according to can4linux rules
     */
    if( IRQ[minor] > 0 && IRQ[minor] < MAX_IRQNUMBER ){
        if( Can_RequestIrq(minor, IRQ[minor] , CAN_Interrupt) ) {
	     printk("Can[%d]: Can't request IRQ %d \n", minor, IRQ[minor]);
	     DBGout(); return -EBUSY;
        }
    } else {
	/* Invalid IRQ number in /proc/.../IRQ */
	DBGout(); return -EBUSY;
    }

    listenmode = 0; /* set NORMAL mode when opening the driver */

    DBGout(); return 0;
}



int Can_RequestIrq(int minor, int irq,
    irqreturn_t (*handler)(int, void *))
{
int err = 0;

    DBGin();
    /*

    int request_irq(unsigned int irq,			// interrupt number  
              void (*handler)(int, void *, struct pt_regs *), // pointer to ISR
		              irq, dev_id, registers on stack
              unsigned long irqflags, const char *devname,
              void *dev_id);

       dev_id - The device ID of this handler (see below).       
       This parameter is usually set to NULL,
       but should be non-null if you wish to do  IRQ  sharing.
       This  doesn't  matter when hooking the
       interrupt, but is required so  that,  when  free_irq()  is
       called,  the  correct driver is unhooked.  Since this is a
       void *, it can point to anything (such  as  a  device-spe-
       cific  structure,  or even empty space), but make sure you
       pass the same pointer to free_irq().

    */


    /* From include/linux/irq.h
       IRQ line status.
	IRQ types
	IRQ_TYPE_NONE		Default, unspecified type
	IRQ_TYPE_EDGE_RISING	Edge rising type
	IRQ_TYPE_EDGE_FALLING	Edge falling type
	IRQ_TYPE_EDGE_BOTH (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING)
	IRQ_TYPE_LEVEL_HIGH	Level high type
	IRQ_TYPE_LEVEL_LOW	Level low type
	IRQ_TYPE_SENSE_MASK	Mask of the above
	IRQ_TYPE_PROBE		Probing in progress
       From include/linux/irq.h
       IRQF_DISABLED - keep irqs disabled when calling the action handler


    */

   /* We don't need the interrupt to be shared for now */
   /* On AT91 
   	IRQF_DISABLED		- recognized
   	IRQF_NOBALANCING	- not
   	IRQF_TRIGGER_FALLING,	- not
    */


    disable_irq(irq);
    /* Interrupt Line */
#ifdef RASPI
#else
    /* set GPIO intterupt lines */
    at91_set_gpio_input(irq_to_gpio(irq), 1);  /* input with pull-up */
    at91_set_deglitch(irq_to_gpio(irq), 1);    /* deglitch is_on */
#endif

#if defined(SPIIRQ_IRQF_DISABLED)
    err = request_irq(irq, handler,
    	  IRQF_DISABLED
    	| IRQF_NOBALANCING
    	| IRQF_TRIGGER_FALLING
    	| IRQF_TRIGGER_RISING,
    	IRQREGDEVNAME, &Can_minors[minor]);
#else  /* SPIIRQ_IRQF_DISABLED */
    err = request_irq(irq, handler,
#  if defined(SSV_MCP2515)
	  IRQF_NOBALANCING
	| IRQF_TRIGGER_FALLING,
#  elseif defined(AuR_MCP2515) || defined(MCP2515SPI)
    	  IRQF_NOBALANCING
    	/* | IRQF_TRIGGER_FALLING, */,
#  else
	    0, 
#  endif  /* SPIIRQ_IRQF_DISABLED */
    	IRQREGDEVNAME, &Can_minors[minor]);
#endif  /* SPIIRQ_IRQF_DISABLED */

    if( !err ){
      DBGprint(DBG_BRANCH,("Requested IRQ: %d @ 0x%lx",
      				irq, (unsigned long)handler));
      IRQ_requested[minor] = 1;
    }
    DBGout(); return err;
}

int Can_FreeIrq(int minor, int irq )
{
    DBGin();
    IRQ_requested[minor] = 0;

    /* printk(" Free IRQ %d  minor %d\n", irq, minor);  */
    free_irq(irq, &Can_minors[minor]);

    DBGout();
    return 0;
}


/* insert_rxfifo
 * insert a CAN frame, pointed to by 'frame' 
 * into the active rx_fifos
 * The driver can handle an rx_fifo for each
 * process which open() the driver
 */

static void insert_rxfifo(int minor, canmsg_t *frame, int *printk_limit)
{
msg_fifo_t   *RxFifo; 	/* pointer to the active FIFO */
int rx_fifo;		/* looping through these fifo indicies */

    DBGin();
    /* handle all subscribed rx fifos */
    for(rx_fifo = 0; rx_fifo < CAN_MAX_OPEN; rx_fifo++) {
	/* for every rx fifo */
	if (CanWaitFlag[minor][rx_fifo] == 1) {
	    /* this FIFO is in use */
	    RxFifo = &Rx_Buf[minor][rx_fifo]; /* prepare buffer to be used */
	    (RxFifo->data[RxFifo->head]).flags  = frame->flags;
	    (RxFifo->data[RxFifo->head]).id     = frame->id;
	    (RxFifo->data[RxFifo->head]).length = frame->length;
	    (RxFifo->data[RxFifo->head]).timestamp = frame->timestamp;
	    memcpy( &(RxFifo->data[RxFifo->head]).data[0],
		    &frame->data[0], frame->length);

	    /* mark just written entry as OK and full */
	    RxFifo->status = BUF_OK;
	    /* Handle buffer wrap-around */
	    ++(RxFifo->head);
	    RxFifo->head %= MAX_BUFSIZE;
	    if(RxFifo->head == RxFifo->tail) {
		if (*printk_limit) { /* restrict number of overall msgs */
		    printk("CAN[%d][%d] RX: FIFO overrun\n", minor, rx_fifo);
		    (*printk_limit)--;
		}
		RxFifo->status = BUF_OVERRUN;
	    } 

	    /*---------- kick the select() call  -*/
	    /* This function will wake up all processes
	       that are waiting on this event queue,
	       that are in interruptible sleep
	    */
// FIXME     wake_up_interruptible(&CanWait[minor][rx_fifo]); 
	}
    } /* end loop fill rx buffers */
    DBGout();
}


/* Read an CAN Frame from the MCP2515 hardware 
from the numbered RX hardware buffer
and store it in the drivers rx queues
*/
static void mcp251x_hw_rx(
	/* struct spi_device *spi, */
	int buf_idx,
	struct timeval *tv,
	int *printk_limit
	)
{
/* struct mcp251x_priv *priv = dev_get_drvdata(&spi->dev); */
struct mcp251x_priv *priv = &realone;
struct spi_device *spi = realone.spi;
u8 *tx_buf = priv->spi_tx_buf;
u8 *rx_buf = priv->spi_rx_buf;
msg_fifo_t   *RxFifo; 
canmsg_t rx;
int minor = 0;		/* only one device at the moment */
int ret;


struct spi_transfer t = {
	.tx_buf = priv->spi_tx_buf,
	.rx_buf = priv->spi_rx_buf,
	.cs_change = 0,
	.len = 14, /* RX buffer: RXBnCTRL to RXBnD7 =
		  1 RXBnCTRL
		+ 4 ID
		+ 1 DLC
		+ 8 Data */
};
struct spi_message m;


    updownhelper();

    RxFifo = &Rx_Buf[minor][0]; 

    down(&priv->spi_lock);

#if defined(MCP2515SPI)
    /* The Read RX Buffer instruction provides
     * a means to quickly address a receive buffer for reading.
     * This instruction further reduces the SPI overhead
     * by automatically clearing the associated receive flag
     * (CANINTF.RXnIF) when CS is raised at the end of the command.
     */
    tx_buf[0] = INSTRUCTION_READ_RXB(buf_idx);
    spi_message_init(&m);

#if 0   /* enable if dma is enabled */
    if (enable_can_dma) {
	    t.tx_dma = priv->spi_tx_dma;
	    t.rx_dma = priv->spi_rx_dma;
	    m.is_dma_mapped = 1;
    }
#endif
    spi_message_add_tail(&t, &m);
    ret = spi_sync(spi, &m);
    if (ret < 0)
	    dev_info(&spi->dev, "%s: failed: ret = %d\n", __FUNCTION__, ret);
    /* FIXME: Should we exit with an error here? */

    rx.length = rx_buf[5] & 0x0f;
    rx.length = rx.length > 8 ? 8 : rx.length;
#else   /* use hard register access */

    tx_buf[0] = INSTRUCTION_READ_RXB(buf_idx);
    at91_spihw_rwn_CSon(priv->spi_tx_buf, priv->spi_rx_buf, 6);
    /* Data length */
    rx.length = rx_buf[5] & 0x0f;
    rx.length = rx.length > 8 ? 8 : rx.length;
    at91_spihw_rwn(priv->spi_tx_buf + 6, priv->spi_rx_buf + 6, rx.length);

#endif	/* MCP2515SPI */

    rx.flags = 0;
    if ((rx_buf[2] >> 3) & 0x1) {
	    /* Extended ID format */
	    rx.flags |= MSG_EXT;
	    rx.id = 
	    (rx_buf[1] << 21)
	    + ((rx_buf[2] & 0xe0) << 13)
	    + ((rx_buf[2] & 3) << 16) 
	    + (rx_buf[3] << 8) + rx_buf[4];
    } else {
	    /* Base ID format */
	    rx.id = (rx_buf[1] << 3) | (rx_buf[2] >> 5);
    }

    if ((rx_buf[5] >> 6) & 0x1) {
	    /* Remote transmission request */
	    rx.flags |= MSG_RTR;
    }
    rx.timestamp.tv_sec = tv->tv_sec;
    rx.timestamp.tv_usec = tv->tv_usec;
    memcpy(&rx.data[0], rx_buf + 6, rx.length);

    insert_rxfifo(minor, &rx, printk_limit); /* put the received frame in all rx queues */

    up(&priv->spi_lock);
}

/* handling the CAN if ISR was called is done here

The following SPI actions are performed:

	Read interrupt sources for actions
	irqsrc = mcp251x_read_reg(CANINTF);
    In case of an RX interrupt:
	read frame
	ret = spi_sync(spi, &m);        14 byte
    In case of an TX interrupt:
	mcp251x_write_bits(CANINTF, irqstate, 0);
	ret = spi_sync(spi, &m);	6 to 14 bytes
	mcp251x_write_reg(TXBCTRL(TRANSMIT_OBJ), TXBCTRL_TXREQ);




do we need:
mcp251x_write_reg(TXBCTRL(TRANSMIT_OBJ), TXBCTRL_TXREQ);
in CAN_SendMessage(int minor, canmsg_t *tx) 










*/
#if defined(MCP2515SPI)
static void mcp251x_irq_work_handler(struct work_struct *ws)
#else
static int mcp251x_irq_work_handler(int irq)
#endif
{
struct mcp251x_priv *priv = &realone;
/* struct spi_device *spi = priv->spi; */
int irqsrc;
int irqstate;
msg_fifo_t  *TxFifo;		/* pointer to the active tx fifo used */
msg_fifo_t   *RxFifo; 		/* pointer to the active rx fifo used */
struct timeval  timestamp;
int rx_fifo;			/* loopindex for different rx queues */
int rxc = 0;

#if 0
static int state    = 0;        /* CAN Controller state */
       int newstate = 0;
#endif
int minor = 0;	/* only one device supported */
int irqmaxloops = 100;          /* maximum loops in IRQ handler */
int printk_limit = MAX_PRINTK_LINES;          /* limit printk lines */

    /* RxFifo = &Rx_Buf[minor][0];  */
    TxFifo = &Tx_Buf[minor];

    /* DBGin(); */
    for (;;) {
	/* Read interrupt sources for actions */
	irqsrc = mcp251x_read_reg(CANINTF);
	irqstate = 0;

	/* fill timestamp as first action. 
	 * Getting a precises time takes a lot of time
	 * (additional 7 µs of ISR time )
	 * if a time stamp is not needed, it can be switched off
	 * by ioctl() */
	if (use_timestamp[minor]) {
	    do_gettimeofday(&timestamp);
	} else {
	    timestamp.tv_sec  = 0;
	    timestamp.tv_usec = 0;
	}
#if 0
	dev_info(&priv->dev, "interrupt:%s%s%s%s%s%s%s%s\n",
		(irqsrc & CANINTF_MERRF) ? " MERR" : "",
		(irqsrc & CANINTF_WAKIF) ? " WAK"  : "",
		(irqsrc & CANINTF_ERRIF) ? " ERR"  : "",
		(irqsrc & CANINTF_TX2IF) ? " TX2"  : "",
		(irqsrc & CANINTF_TX1IF) ? " TX1"  : "",
		(irqsrc & CANINTF_TX0IF) ? " TX0"  : "",
		(irqsrc & CANINTF_RX1IF) ? " RX1"  : "",
		(irqsrc & CANINTF_RX0IF) ? " RX0"  : "");
#endif
#if 0
        if (irqsrc & (CANINTF_MERRF | CANINTF_ERRIF)) {
            dev_info(&priv->dev,
                     "ERR: IR:0x%02X EF:0x%02X TEC:0x%02X REC:0x%02X\n",
                     irqsrc,
                     mcp251x_read_reg(EFLG),
                     mcp251x_read_reg(TEC),
                     mcp251x_read_reg(REC));
        }
#endif	
        if (!irqsrc) {
            /* No interrupt from MCP2515 */
            break;
        }

        if (irqmaxloops <= 0) {
            dev_info(&priv->dev, "TOO MANY IRQ LOOPS\n");
            break;
        }
        irqmaxloops--;

	if (irqsrc & CANINTF_WAKIF) {
	    irqstate |= CANINTF_WAKIF;
            if (printk_limit) {
                dev_info(&priv->dev, "WAKIF:0x%02X\n", (u8)irqsrc);
                printk_limit--;
            }
	}

	/* message error interrupt flag
	 * any error while transmitting or receiving */
	if (irqsrc & CANINTF_MERRF) {
	    irqstate |= CANINTF_MERRF;
	}
	
	/* When the error interrupt is enabled
	   (CANINTE.ERRIE = 1), an interrupt is generated on the
	   INT pin if an overflow condition occurs or if the error
	   state of the transmitter or receiver has changed. The
	   Error Flag (EFLG) register will indicate one of the
	   following conditions.  */
	if (irqsrc & CANINTF_ERRIF) {
	    u8 eflag = mcp251x_read_reg(EFLG);
	    canmsg_t frame;	/* temporary CAN from for error signaling */
	    frame.flags  = MSG_ACTIVE;	/* CAN frame flags */

	    /* Set error frame flags according to bus state */
	    if (eflag & EFLG_TXBO) {
		frame.flags |= MSG_BUSOFF;
	    } else if (eflag & EFLG_TXEP) {
		frame.flags |= MSG_PASSIVE;
	    } else if (eflag & EFLG_RXEP) {
		frame.flags |= MSG_PASSIVE;
	    } else if (eflag & EFLG_TXWAR) {
		frame.flags |= MSG_WARNING;
	    } else if (eflag & EFLG_RXWAR) {
		frame.flags |= MSG_WARNING;
	    }
	    if (eflag & (EFLG_RX0OVR | EFLG_RX1OVR)) {
		frame.flags |= MSG_OVR;
		mcp251x_write_reg(EFLG, 0x00);

	    }

	    /* generate an error CAN frame for the rx queue */
	    frame.id = CANDRIVERERROR;
	    frame.length = 0;
	    frame.timestamp = timestamp;
	    insert_rxfifo(minor, &frame, &printk_limit);
	    irqstate |= CANINTF_ERRIF;
	}
      /*========== receive interrupt */
        if (irqsrc & CANINTF_RX0IF) {
            /* DBGprint(DBG_BRANCH, ("RX 0\n")); */
            mcp251x_hw_rx(0, &timestamp, &printk_limit);
            /* CANINTF_RXnIF is cleared already automatically */
            rxc++;
        }
        if (irqsrc & CANINTF_RX1IF) {
            /* DBGprint(DBG_BRANCH, ("RX 1\n")); */
            mcp251x_hw_rx(1, &timestamp, &printk_limit);
            /* CANINTF_RXnIF is cleared already automatically */
            rxc++;
        }
        if (irqsrc & CANINTF_TX0IF) {
            irqstate |= CANINTF_TX0IF;
        }
        if (irqstate) {
            mcp251x_write_bits(CANINTF, irqstate, 0);
        }
    /*========== transmit interrupt */
	if (irqsrc & (/*CANINTF_TX2IF | CANINTF_TX1IF |*/ CANINTF_TX0IF)) {
	    /* CAN frame successfully sent */
	    /* use time stamp sampled with last INT */
	    last_Tx_object[minor].timestamp = timestamp;

	    /* depending on the number of open processes
	     * the TX data has to be copied in different
	     * rx fifos
	     */
	    for(rx_fifo = 0; rx_fifo < CAN_MAX_OPEN; rx_fifo++) {
		/* for every rx fifo */
		if (CanWaitFlag[minor][rx_fifo] == 1) {
		    /* this FIFO is in use */
			
		    /*
		     * Don't copy the message in the receive queue
		     * of the process that sent the message unless
		     * this process requested selfreception.
		     */
		    if ((last_Tx_object[minor].cob == rx_fifo) && 
			(selfreception[minor][rx_fifo] == 0)) {
			continue;
		    }

		    /* prepare buffer to be used */
		    RxFifo = &Rx_Buf[minor][rx_fifo];
		    memcpy((void *)&RxFifo->data[RxFifo->head],
			(void *)&last_Tx_object[minor],
			sizeof(canmsg_t));
		    
		    /* Mark message as 'self sent/received' */
		    RxFifo->data[RxFifo->head].flags |= MSG_SELF;

		    /* increment write index */
		    RxFifo->status = BUF_OK;
		    /* Handle buffer wrap-around */
		    ++(RxFifo->head);
		    RxFifo->head %= MAX_BUFSIZE;
		    if(RxFifo->head == RxFifo->tail) {
			if (printk_limit) {
			    printk("CAN[%d][%d] RX: FIFO overrun\n", minor, rx_fifo);
			    printk_limit--;
                        }
			RxFifo->status = BUF_OVERRUN;
		    } 
		    /*---------- kick the select() call  -*/
		    /* This function will wake up all processes
		       that are waiting on this event queue,
		       that are in interruptible sleep */
// FIXME	    wake_up_interruptible(&CanWait[minor][rx_fifo]); 
		} /* this FIFO is in use */
	    } /* end for loop filling all rx-fifos */

	    if( TxFifo->free[TxFifo->tail] == BUF_EMPTY ) {
		/* TX FIFO empty, nothing more to sent */
		/* printk("TXE\n"); */
		TxFifo->status = BUF_EMPTY;
		TxFifo->active = 0;
		/* This function will wake up all processes
		   that are waiting on this event queue,
		   that are in interruptible sleep */
// FIXME	wake_up_interruptible(&CanOutWait[minor]); 
	    } else {
	    /* The TX message FIFO contains other CAN frames to be sent
	     * The next frame in the FIFO is copied into the last_Tx_object
	     * and directly into the hardware of the CAN controller */

	    /* dev_info(&priv->dev, "schedule next frame for transmission\n"); */
	    memcpy((void *)&last_Tx_object[minor],
		    (void *)&TxFifo->data[TxFifo->tail],
		    sizeof(canmsg_t));

	    CAN_SendMessage(0, &last_Tx_object[minor]);

	    /* now this entry is EMPTY */
	    TxFifo->free[TxFifo->tail] = BUF_EMPTY;
	    ++(TxFifo->tail);
	    TxFifo->tail %= MAX_BUFSIZE;
	    }
	} /* tx interrupt */
    } /* for (;;) */
    /* DBGout(); */
 
#if defined(CANSPI_USEKTHREAD) || defined(MCP2515SPI)	
    return;
#else
    return(rxc);
#endif
}

/*
  On the SSV Board, CAN2, the MCP2515, INT_2 is connected to PIO PB24
  another signal, CAN2_SOF is connected to PB25, which (hopefully)
  is not needed (yet)
 
  From the Manual

  The MCP2515 has eight sources of interrupts. The
  CANINTE register contains the individual interrupt
  enable bits for each interrupt source.

  - If the message started to transmit but encoun-
  tered an error condition, the TXBnCTRL.TXERR
  and the CANINTF.MERRF bits will be set and an
  interrupt will be generated on the INT pin if the
  CANINTE.MERRE bit is set
  - If the CANINTE.RXnIE bit is set, an interrupt will be
  generated on the INT pin to indicate that a valid message has
  been received.
  - Transmit Interrupt
  When the transmit interrupt is enabled
  (CANINTE.TXnIE = 1), an interrupt will be generated on
  the INT pin once the associated transmit buffer
  becomes empty and is ready to be loaded with a new
  message. The CANINTF.TXnIF bit will be set to indicate
  the source of the interrupt. The interrupt is cleared by
  clearing the TXnIF bit.
  - Receive Interrupt
  When receive interrupt is enabled
  (CANINTE.RXnIE = 1), an interrupt will be generated
  on the INT pin once a message has been successfully
  received and loaded into the associated receive buffer.
  This interrupt is activated immediately after receiving
  the EOF field. The CANINTF.RXnIF bit will be set to
  indicate the source of the interrupt. The interrupt is
  cleared by clearing the RXnIF bit.
  - Message Error Interrupt
  When an error occurs during the transmission or reception of a
  message, the message error flag (CANINTF.MERRF) will be set and,
  if the CANINTE.MERRE bit is set, an interrupt will be generated
  on the INT pin. This is intended to be used to facilitate baud
  rate determination when used in conjunction with Listen-only mode.
  When the error interrupt is enabled
  (CANINTE.ERRIE = 1), an interrupt is generated on
  the INT pin if an overflow condition occurs or if the error
  state of the transmitter or receiver has changed. The
  Error Flag (EFLG) register will indicate one of the
  following conditions.
  - Interrupt Acknowledge
  Interrupts are directly associated with one or more status flags
  in the CANINTF register. Interrupts are pending as long as one
  of the flags is set. Once an interrupt flag is set by the device,
  the flag can not be reset by the MCU until the interrupt condition
  is removed.

  The SSV board is using a standard GPIO pin with edge sensitive interrupt.
  The CAN controller is issuing an interrupt with a high/low edge.
  The kernel is not able to differnciate, therefore we handle both issues.

  The A&R board is using a standard GPIO ...
  I don't know more about it, currently.

 */

irqreturn_t CAN_Interrupt(int irq, void *dev_id) 
{
#if defined(MCP2515SPI)
    struct mcp251x_priv *priv = &realone;
#endif
#ifdef SPIIRQ_TIME_MEASUREMENT
    struct timeval timestamp1,timestamp2;
    int mytime, rx;
    static int oldtime = 0, oldrx = 0;
    unsigned long flags = 0;
    
    local_irq_save(flags);
    do_gettimeofday(&timestamp1);
    rx = mcp251x_irq_work_handler(irq);
    do_gettimeofday(&timestamp2);
    local_irq_restore(flags);
    mytime = timestamp2.tv_usec - timestamp1.tv_usec;
    if (mytime < 0)
         mytime = 1000000 - timestamp1.tv_usec + timestamp2.tv_usec;
    if ((oldtime < mytime) || (oldrx < rx)) {
        printk(KERN_ERR "IRQTIME:%dus RX:%d\n", mytime, rx);
        oldtime = mytime;
        oldrx = rx;
    }
#else
#  if defined(MCP2515SPI)
	/* Schedule bottom half */
	/* printk("=====> handle CAN IRQ / %s()\n", __func__); */
	queue_work(priv->wq, &priv->irq_work);
#  else
	(void)mcp251x_irq_work_handler(irq);
#  endif
#endif
    return IRQ_HANDLED;
}

#if defined(MCP2515SPI)
/* This is the basic function called to check if SPI is available
   and initialize the SPI layer
   struct spi_device 
   is decribed in include/linux/spi/spi.h

   In probe/remove functions we take care of any resource allocation/setup
   that is needed for each chip.
   Often times, this may involve setting up a per-device spinlock.
 */
static int __devinit mcp251x_can_probe(struct spi_device *spi)
{
struct mcp251x_priv *priv;
/* struct mcp251x_platform_data *pdata = spi->dev.platform_data; */
int ret = -ENODEV;
int err;
int minor = -1;


    DBGin();

#if 0
/* IRQ and platform data are not set in the board package */
	/* check the correct installed SPI */
	if (!spi->irq) {
		dev_dbg(&spi->dev, "no IRQ?\n");
		/* return -ENODEV; */
	}
	if (!pdata) {
		dev_dbg(&spi->dev, "no platform data?\n");
		/*
		may be needed for some devive specific parameters
		set in the bsp.
		But instead, it can be compiled here into this
		loadabel module
		*/
		/* return -ENODEV; */
	}
#endif

    /* for all CAN spi devices, but only one supported yet */
    spidevice[0] = spi; 

    /* At the moment we use ONE global defined structure for the
       mcp251x SPI data, as a struct mcp251x_priv, defined in mcp2515.h.
       It contains e.g. the rx/tx buffers usewd, a semaphore,
       workqueue structures and more.
     */  

    priv = &realone;            /* realone is a global defined structure */
    priv->spi = spi;
// FIXME init_MUTEX(&priv->spi_lock);

#if 0
    dev_dbg(&spi->dev, "%s, allocate %d bytes memory for tx and rx\n",
			__FUNCTION__, SPI_TRANSFER_BUF_LEN);
#endif
    /* If requested, allocate DMA buffers */
    /* FIXME: not implemented yet consistently ! */
#if 0   /* enable if dma is enabled */
   if (enable_can_dma) {
	spi->dev.coherent_dma_mask = DMA_32BIT_MASK;

	/* Minimum coherent DMA allocation is PAGE_SIZE, so allocate
	   that much and share it between Tx and Rx DMA buffers. */
	priv->spi_tx_buf = dma_alloc_coherent(&spi->dev,
		PAGE_SIZE, &priv->spi_tx_dma, GFP_DMA);

	if (priv->spi_tx_buf) {
		priv->spi_rx_buf = (u8 *)(priv->spi_tx_buf +
			(PAGE_SIZE / 2));
		priv->spi_rx_dma = (dma_addr_t)(priv->spi_tx_dma +
			(PAGE_SIZE / 2));
	} else {
		/* Fall back to non-DMA */
		enable_can_dma = 0;
	}
    }
#endif


    /* Allocate non-DMA buffers */
    if ( !enable_can_dma ) {
    dev_dbg(&spi->dev, "%s, don't use DMA buffers\n", __FUNCTION__);
	
    priv->spi_tx_buf = kmalloc(SPI_TRANSFER_BUF_LEN, GFP_KERNEL);

	if (!priv->spi_tx_buf) {
		ret = -ENOMEM;
		goto error_tx_buf;
	}
	priv->spi_rx_buf = kmalloc(SPI_TRANSFER_BUF_LEN, GFP_KERNEL);
	if (!priv->spi_tx_buf) {
		ret = -ENOMEM;
		goto error_rx_buf;
	}
    }


#if 0
    /* Call out to platform specific setup */
    if (pdata->board_specific_setup)
	    pdata->board_specific_setup(spi);

    /* Call out to platform specific hardware reset */
    if (pdata->device_reset)
	    pdata->device_reset(spi);
#endif

    /* printk("Can[-1]: - : create work queue\n"); */
    /* ISR only schedules a task to the work queu */
    priv->wq = create_singlethread_workqueue("mcp251x_wq");
    /* printk("Can[-1]: - : create work queue returned \n"); */

    INIT_WORK(&priv->irq_work, mcp251x_irq_work_handler);
    /* printk("Can[-1]: - : work queues initialized \n"); */

    /* http://lwn.net/Articles/23993/
    Completions are a simple synchronization mechanism
    that is preferable to sleeping and waking up in some situations.
    If you have a task that must simply sleep
    until some process has run its course,
    completions can do it easily and without race conditions.

    This feature is currently not used. Only initialized.

    */
    init_completion(&priv->awake);

    /* Configure the SPI bus */
    spi->mode = SPI_MODE_0;	/* select the clock mode */
    spi->bits_per_word = 8;
    err = spi_setup(spi);
    if (err < 0)
	    return err;

#if 0
    /* The following are only usage examples on how to send a CAN frame */
    {
    canmsg_t tx;
    int i = 4; 	/* bit timing table index 4 is 125 k */ 
	dev_info(&spi->dev, "%s, set bit timing index %d\n", __func__, i);

	mcp251x_write_reg(CNF1, (u8)CanTiming[i][0]);
	mcp251x_write_reg(CNF2, (u8)CanTiming[i][1]);
	mcp251x_write_reg(CNF3, (u8)(CanTiming[i][1] >> 8));

	mcp251x_write_reg(/* spi,*/ CANCTRL, CANCTRL_REQOP_NORMAL);



	/* First writing one by one to the CAN tx registers */
	mcp251x_write_reg(/* spi,*/ TXBSIDH(1), (u8) (200 >> 3));
	mcp251x_write_reg(/* spi,*/ TXBSIDL(1), (u8) (200 << 5));
	mcp251x_write_reg(/* spi,*/ TXBDLC(1), (u8) 1);
	mcp251x_write_reg(/* spi,*/ TXBD0(1), (u8) 0x55);
	mcp251x_write_reg(/* spi,*/ TXBCTRL(1), (u8) TXBCTRL_TXREQ);

	CAN_ShowStat(0); 

	/* and using the CAN_SendMessage() function */
	tx.id = 111;
	tx.flags = MSG_BASE;
	tx.length = 7;
	tx.data[0] = 1;
	tx.data[1] = 2;
	tx.data[2] = 3;
	tx.data[3] = 4;
	tx.data[4] = 5;
	tx.data[5] = 6;
	tx.data[6] = 7;
	tx.data[7] = 8;

	CAN_SendMessage(0, &tx);

	/* 1 - 29 do get each identifier bit touched */
	for(i = 0; i < 4; i++) {
	    tx.id = 1 << i;
	    tx.flags = MSG_EXT;
	    tx.length = 5;
	    tx.data[0] = 0xaa;
	    tx.data[1] = 2;
	    tx.data[2] = 3;
	    tx.data[3] = 4;
	    tx.data[4] = 0x55;
	    tx.data[5] = 6;
	    tx.data[6] = 7;
	    tx.data[7] = 8;
	    CAN_SendMessage(0, &tx);
	}
    }
#endif

    /* error free return */
    return 0;

error_rx_buf:
    if (1 /* !enable_dma */) {
	    kfree(priv->spi_tx_buf);
    }
error_tx_buf:
    dev_err(&spi->dev, "probe failed\n");
/* error_out: */
    return  ret;
}

static int mcp251x_can_resume(struct spi_device *spi)
{
int minor = -1;

    DBGin();
    printk("%s() not implemented yet.\n", __FUNCTION__);
    return 0;
}

static int mcp251x_can_suspend(struct spi_device *spi, pm_message_t state)
{
int minor = -1;

    DBGin();
    printk("%s() not implemented yet.\n", __func__);
    return 0;
}


static int __devexit mcp251x_can_remove(struct spi_device *spi)
{
struct mcp251x_priv *priv = &realone;
int minor = -1;

    DBGin();

    kfree(priv->spi_tx_buf);
    kfree(priv->spi_rx_buf);

    DBGout();
    return 0;
}
 

/* can4linux is acting like a SPI protocol driver
   (see Documentation/spi/spi-summary)

   The driver core  will automatically attempt to bind this driver
   to any SPI device whose board_info gave a modalias of "canspi".
   Have a look at the board*.c file
*/
struct spi_driver mcp251x_can_driver = {
	.driver = {
		.name		= "canspi",
		.bus		= &spi_bus_type,
		.owner		= THIS_MODULE,
	},

	.probe		= mcp251x_can_probe,
	.remove		= __devexit_p(mcp251x_can_remove),
	.suspend	= mcp251x_can_suspend,
	.resume		= mcp251x_can_resume,
};
#endif

#if defined(SSV_MCP2515) || defined(AuR_MCP2515)
int __devinit mcp251x_can_probe(void)
{
    struct mcp251x_priv *priv;
    int ret = -ENODEV;
    int minor = -1;

    DBGin();

    priv = &realone;
    init_MUTEX(&priv->spi_lock);

    dev_dbg(&priv->dev, "%s, allocate memory %d for tx and rx\n",
                        __FUNCTION__, SPI_TRANSFER_BUF_LEN);

    priv->spi_tx_buf = kmalloc(SPI_TRANSFER_BUF_LEN, GFP_KERNEL);
    if (!priv->spi_tx_buf) {
	ret = -ENOMEM;
	goto error_tx_buf;
    }
    priv->spi_rx_buf = kmalloc(SPI_TRANSFER_BUF_LEN, GFP_KERNEL);
    if (!priv->spi_tx_buf) {
	ret = -ENOMEM;
	goto error_rx_buf;
    }
    if (at91_spihw_init()) {
	ret = -ENOMEM;
	goto error_rx_buf;
    }

    /* error free return */
    return 0;

error_rx_buf:
    kfree(priv->spi_tx_buf);
error_tx_buf:
    dev_err(&priv->dev, "probe failed\n");
    return  ret;
}

int __devexit mcp251x_can_remove(void)
{
    struct mcp251x_priv *priv = &realone;
    int minor = -1;
    DBGin();

    kfree(priv->spi_tx_buf);
    kfree(priv->spi_rx_buf);
    at91_spihw_deinit();
    return 0;
}
#endif
 
/* ---------------- E N D E ---------------------------------------------- */
