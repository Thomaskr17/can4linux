/* mcp2515.h
 * registrer and other definitions for the Microchip MCP2515
 * used with can4linux
 */

#ifndef __CAN_MCP2515__
#define __CAN_MCP2515__

/* SPI interface instruction set */
#define INSTRUCTION_RESET		0xC0
#define INSTRUCTION_WRITE		0x02
#define INSTRUCTION_READ		0x03
#define INSTRUCTION_BIT_MODIFY	0x05
/* one of:
	TX buffer 0, Start at TXB0SIDH
	TX buffer 1, Start at TXB1SIDH
	TX buffer 2, Start at TXB2SIDH
*/
#define INSTRUCTION_LOAD_TXB(n)	(0x40 + 2 * (n))
/* one of:
	Receive Buffer 0, Start at RXB0SIDH
	Receive Buffer 1, Start at RXB1SIDH
*/
#define INSTRUCTION_READ_RXB(n)	(((n) == 0) ? 0x90 : 0x94)

/* MCP251x registers */
#define CANSTAT       0x0e
#define CANCTRL       0x0f
#  define CANCTRL_REQOP_MASK        0xe0
#  define CANCTRL_REQOP_CONF        0x80
#  define CANCTRL_REQOP_LISTEN_ONLY 0x60
#  define CANCTRL_REQOP_LOOPBACK    0x40
#  define CANCTRL_REQOP_SLEEP       0x20
#  define CANCTRL_REQOP_NORMAL      0x00
#  define CANCTRL_OSM               0x08
#  define CANCTRL_ABAT              0x10
#define TEC           0x1c
#define REC           0x1d
 /* Mask and Filter Register */
#define RXFnSIDH	0x00
#define RXFnSIDL	0x01
#define RXFnEID8	0x02
#define RXFnEID0	0x03

#define RXM0SIDH	0x20
#define RXM0SIDL	0x21
#define RXM0EID8	0x22
#define RXM0EID0	0x23

#define RXM1SIDH	0x24
#define RXM1SIDL	0x25
#define RXM1EID8	0x26
#define RXM1EID0	0x27

 /* conf */
#define CNF1          0x2a
#define CNF2          0x29
#  define CNF2_BTLMODE  0x80
#define CNF3          0x28
#  define CNF3_SOF      0x08
#  define CNF3_WAKFIL   0x04
#  define CNF3_PHSEG2_MASK 0x07
#define CANINTE       0x2b
#  define CANINTE_MERRE 0x80
#  define CANINTE_WAKIE 0x40
#  define CANINTE_ERRIE 0x20
#  define CANINTE_TX2IE 0x10
#  define CANINTE_TX1IE 0x08
#  define CANINTE_TX0IE 0x04
#  define CANINTE_RX1IE 0x02
#  define CANINTE_RX0IE 0x01
#define CANINTF       0x2c
#  define CANINTF_MERRF 0x80
#  define CANINTF_WAKIF 0x40
#  define CANINTF_ERRIF 0x20
#  define CANINTF_TX2IF 0x10
#  define CANINTF_TX1IF 0x08
#  define CANINTF_TX0IF 0x04
#  define CANINTF_RX1IF 0x02
#  define CANINTF_RX0IF 0x01
#define EFLG          0x2d
#  define EFLG_EWARN    0x01
#  define EFLG_RXWAR    0x02
#  define EFLG_TXWAR    0x04
#  define EFLG_RXEP     0x08
#  define EFLG_TXEP     0x10
#  define EFLG_TXBO     0x20
#  define EFLG_RX0OVR   0x40
#  define EFLG_RX1OVR   0x80
#define TXBCTRL(n)  ((n * 0x10) + 0x30)
#  define TXBCTRL_TXREQ 0x08
#  define TXBCTRL_ABTF	0x40
#  define TXBCTRL_MLOA	0x20
#  define TXBCTRL_TXERR 0x10

#define TXBSIDH(n)  (((n) * 0x10) + 0x30 + TXBSIDH_OFF)
#define TXBSIDL(n)  (((n) * 0x10) + 0x30 + TXBSIDL_OFF)
#define TXBEID8(n)  (((n) * 0x10) + 0x30 + TXBEID8_OFF)
#define TXBEID0(n)  (((n) * 0x10) + 0x30 + TXBEID0_OFF)
#define TXBDLC(n)   (((n) * 0x10) + 0x30 + TXBDLC_OFF)
#define TXBD0(n)    (((n) * 0x10) + 0x30 + TXBDAT_OFF)


#define TXBCTRL_OFF 0
#define TXBSIDH_OFF 1
#define TXBSIDL_OFF 2
#define TXBEID8_OFF 3
#define TXBEID0_OFF 4
#define TXBDLC_OFF  5
#define TXBDAT_OFF  6




#define RXBCTRL(n)  ((n * 0x10) + 0x60)
#  define RXBCTRL_BUKT  0x04
#  define RXBCTRL_RXM0	0x20
#  define RXBCTRL_RXM1	0x40

#define MCP_RTS_TX0     0x81
#define MCP_RTS_TX1     0x82
#define MCP_RTS_TX2     0x84
#define MCP_RTS_ALL     0x87

/* Buffer size required for the largest SPI transfer (i.e., reading a
 * frame). */
#define CAN_FRAME_MAX_DATA_LEN	8
#define SPI_TRANSFER_BUF_LEN	(2*(6 + CAN_FRAME_MAX_DATA_LEN))
#define CAN_FRAME_MAX_BITS      128

#define DEVICE_NAME "mcp251x_can"

/* MCP2515 has three transmit buffers, we are using only one for now */
#define TRANSMIT_OBJ 0

/* structure need to hold the SPI buffers für 
 * communication with the CAN using the Linux kernel SPI layer
 *
 * FIX: needed for each CAN controller connected to the SPI bus
 */
struct mcp251x_priv {
	struct spi_device *spi;

	struct semaphore spi_lock; /* SPI buffer lock */
	u8 *spi_tx_buf;
	u8 *spi_rx_buf;
	dma_addr_t spi_tx_dma;
	dma_addr_t spi_rx_dma;

	struct sk_buff *tx_skb;
	struct workqueue_struct *wq;
	struct work_struct tx_work;
	struct work_struct irq_work;
	struct completion awake;
	int wake;

        volatile void *spi_iomem;
        struct device dev;		/* include/linux/device.h */
};

/*=== Bit Timing Registers for 25 MHz ( = Fcan) ===*/
/* BCR1 == CNF1 - CONFIGURATION 1 (ADDRESS: 2Ah)
           BRP: Baud Rate Prescaler bits <5:0>
	   SJW: Synchronization Jump Width Length bits <1:0>

   BCR2 
       bits 0-7 (low part) 
       CNF2 - CONFIGURATION 1 (ADDRESS: 29h)
	    BTLMODE:  should be 1 in our configuration
	    SAM: Sample point configuration bit<6> : 1 - three samples
	                                             0 - sampled once 
	    PHSEG1: PS1 Length bits<5:3>
	    PRSEG: Propagation Segment Length bits <2:0>
       bits 8-15 (high part)
            PHSEG2: PS2 Length bits<2:0>
            Minimum valid setting for PS2 is 2 TQ == 1 
*/
#if CAN_SYSCLK == 25		
#   define CAN_BCR1_10K         0x0031
#   define CAN_BCR2_10K         0x07BF

#   define CAN_BCR1_20K         0x0018
#   define CAN_BCR2_20K         0x07BF

#   define CAN_BCR1_50K         0x0009
#   define CAN_BCR2_50K         0x07BF

#   define CAN_BCR1_100K        0x0004
#   define CAN_BCR2_100K        0x07BF

#   define CAN_BCR1_125K        0x0004
#   define CAN_BCR2_125K        0x02BF

#   define CAN_BCR1_250K        0x0001
#   define CAN_BCR2_250K        0x07BF

#   define CAN_BCR1_500K        0x0000
#   define CAN_BCR2_500K        0x07BF


/* 800K not possible */
#   define CAN_BCR1_800K        0x0000
#   define CAN_BCR2_800K        0x0000

/* 1000K not possable */
#   define CAN_BCR1_1000K       0x0000
#   define CAN_BCR2_1000K       0x0000

/*=== Bit Timing Registers for 16 MHz ( = Fcan) ===*/
#elif CAN_SYSCLK == 16		
#   define CAN_BCR1_10K		0x0031
#   define CAN_BCR2_10K		0x01FC
#   define CAN_BCR1_20K		0x0018
#   define CAN_BCR2_20K		0x01FC
#   define CAN_BCR1_50K		0x0009
#   define CAN_BCR2_50K		0x01FC
#   define CAN_BCR1_100K	0x0004
#   define CAN_BCR2_100K	0x01FC
#   define CAN_BCR1_125K	0x0003
#   define CAN_BCR2_125K	0x01FC
#   define CAN_BCR1_250K	0x0001
#   define CAN_BCR2_250K	0x01FC
#   define CAN_BCR1_500K	0x0000
#   define CAN_BCR2_500K	0x01FC
#   define CAN_BCR1_800K	0x0000
#   define CAN_BCR2_800K	0x00F0 /* ! only one TQ in phase 2 */
#   define CAN_BCR1_1000K	0x0000
#   define CAN_BCR2_1000K	0x00E0 /* ! only one TQ in phase 2 */

/*=== Bit Timing Registers for 12 MHz ( = Fcan) ===*/
/* used http://www.port.de/cgi-bin/tq.cgi?ctype=Pic18&CLK=6&sample_point=87.5 */
/* CANopen suggested sample point 0f 87.5% mostly not reached */
#elif CAN_SYSCLK == 12	                /* getestet */	
#   define CAN_BCR1_10K		0x001D  /* OK */
#   define CAN_BCR2_10K		0x02BF
#   define CAN_BCR1_20K		0x0013  /* OK */
#   define CAN_BCR2_20K		0x01BB
#   define CAN_BCR1_50K		0x0005  /* OK */
#   define CAN_BCR2_50K		0x02BF
#   define CAN_BCR1_100K	0x0002  /* OK */
#   define CAN_BCR2_100K	0x02BF
#   define CAN_BCR1_125K	0x0002  /* OK */
#   define CAN_BCR2_125K	0x01FC
#   define CAN_BCR1_250K	0x0000  /* OK */
#   define CAN_BCR2_250K	0x06BF
#   define CAN_BCR1_500K	0x0000  /* OK */
#   define CAN_BCR2_500K	0x01B8  /* ! only one TQ in phase 2 */
#   define CAN_BCR1_800K	0x0000  /* -  */
#   define CAN_BCR2_800K	0x0000
#   define CAN_BCR1_1000K	0x0000  /* -  */
#   define CAN_BCR2_1000K	0x0000

#else /* CAN_SYSCLK */
#   error "Please specify an CAN_SYSCLK value of 12, 16 or 25 MHz!"
#   error "or extend table in mpc2515.h"
#endif /* CAN_SYSCLK */




/* Access Macros for register access.
 * can4linux tries to use always (if possible)  the same syntax
 * CANin(), CANout(), CANset(), ....
 * this makes it easiert to use code from the embedded CANopen 
 * drivers
 */


# define CANin(bd,adr)		mcp251x_read_reg(adr)
# define CANout(bd,adr,v)	mcp251x_read_reg(adr, v)



#endif /* __CAN_MCP2515__ */
