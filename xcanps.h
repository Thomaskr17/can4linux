/* Xilinx XCANPS  CAN register definitions
 * For each CAN controller hardware depending register definitions
 */

#ifndef __CAN_XCANPS_
#define __CAN_XCANPS_

/* number of bytes allocated by the CAN controller */
#define CAN_RANGE 0x1000

/* That is hoe the CAN controller looks like for us programmers */
typedef struct canregs {
    u32	    srr;	/* Software Reset Register			    */
    u32	    msr;	/* Mode Select Register				    */
    u32	    brpr;	/* Baud Rate Prescaler Register			    */
    u32	    btr;	/* Bit Timing Register				    */
    u32	    ecr;	/* Error Counter Register			    */
    u32	    esr;	/* Error Status Register			    */
    u32	    sr;		/* Status Register				    */
    u32	    isr;	/* Interrupt Status Register			    */
    u32	    ier;	/* Interrupt Enable Register			    */
    u32	    icr;	/* Interrupt Clear Register			    */
    u32	    tcr;	/* Timestamp Control Register			    */
    u32	    wir;	/* Watermark Interrupt Register			    */
    u32	    txfifo_id;	/* transmit message fifo message identifier	    */
    u32	    txfifo_dlc;	/* transmit message fifo data length code	    */
    u32	    txfifo_data1;   /* transmit message fifo data word 1	    */
    u32	    txfifo_data2;   /* transmit message fifo data word 2	    */
    u32	    txhpb_id;	/* transmit high priority buffer message identifier */
    u32	    txhpb_dlc;	/* transmit high priority buffer data length code   */
    u32	    txhpb_data1; /* transmit high priority buffer data word 1	    */
    u32	    txhpb_data2; /* transmit high priority buffer data word 2	    */
    u32	    rxfifo_id;	/* receive message fifo message identifier	    */
    u32	    rxfifo_dlc;	/* receive message fifo data length code	    */
    u32	    rxfifo_data1;    /* receive message fifo data word 1	    */
    u32	    rxfifo_data2;    /* receive message fifo data word 2	    */
    u32	    afr;	/* Acceptance Filter Register			    */
    u32	    afmr1;	/* Acceptance Filter Mask Register 1		    */
    u32	    afir1;	/* Acceptance Filter ID Register 1		    */
    u32	    afmr2;	/* Acceptance Filter Mask Register 2		    */
    u32	    afir2;	/* Acceptance Filter ID Register 2		    */
    u32	    afmr3;	/* Acceptance Filter Mask Register 3		    */
    u32	    afir3;	/* Acceptance Filter ID Register 3		    */
    u32	    afmr4;	/* Acceptance Filter Mask Register 4		    */
    u32	    afir4;	/* Acceptance Filter ID Register 4		    */
} __attribute__((packed)) canregs_t;



/* Register definitions */

/* The following register bit definition are taken from Xilinx xcanps_hw.h  */
/* comments are taken from the manual UG585(v1.3) October 30, 2012	    */

/* Software Reset Register (SRR) Bit Definitions and Masks		    */
/* It really has only two bits */
#define XCANPS_SRR_CEN_MASK	0x00000002  /**< Can Enable		    */
/* 1: The CAN controller is in Loop Back, Sleep or Normal mode
   depending on the LBACK and SLEEP bits in the MSR.
   0: The CAN controller is in the Configuration mode.			    */
#define XCANPS_SRR_SRST_MASK	0x00000001  /**< Reset			    */
/* 1: CAN controller is reset.
   If a 1 is written to this bit, all the CAN controller
   configuration registers (including the SRR) are reset.
   Reads to this bit always return a 0.					    */ 


/* Mode Select Register (MSR) Bit Definitions and Masks			    */
#define XCANPS_MSR_SNOOP_MASK	0x00000004 /**< Snoop Mode Select	    */
#define XCANPS_MSR_LBACK_MASK	0x00000002 /**< Loop Back Mode Select	    */
#define XCANPS_MSR_SLEEP_MASK	0x00000001 /**< Sleep Mode Select	    */

/* Baud Rate Prescaler register (BRPR) Bit Definitions and Masks	    */
/* The BRPR can be programmed to any value in the range 0-255.
   The actual value is 1 more than the value written into the register.
   The actual value ranges from 1 to 256.				    */ 
#define XCANPS_BRPR_BRP_MASK	0x000000FF /**< Baud Rate Prescaler	    */


/* Bit Timing Register (BTR) Bit Definitions and Masks			    */
#define XCANPS_BTR_SJW_MASK	0x00000180 /**< Synchronization Jump Width  */
#define XCANPS_BTR_SJW_SHIFT	7
#define XCANPS_BTR_TS2_MASK	0x00000070 /**< Time Segment 2		    */
/* value 0...7 results in TS2 values of 1...8				    */
#define XCANPS_BTR_TS2_SHIFT	4
#define XCANPS_BTR_TS1_MASK	0x0000000F /**< Time Segment 1		    */
/* value 0...15 results in TS2 values of 1...16				    */




/* Error Counter Register (ECR) Bit Definitions and Masks		    */
#define XCANPS_ECR_REC_MASK	0x0000FF00 /**< Receive Error Counter	    */
#define XCANPS_ECR_REC_SHIFT	8
#define XCANPS_ECR_TEC_MASK	0x000000FF /**< Transmit Error Counter	    */


/* Status Register (SR) Bit Definitions and Masks			    */
/* The CAN Status Register provides a status of all conditions of the Core.
   Specifically, FIFO status, Error State, Bus State
   and Configuration mode are reported.					    */
#define XCANPS_SR_SNOOP_MASK	0x00001000 /**< Snoop Mask		    */
#define XCANPS_SR_ACFBSY_MASK	0x00000800 /**< Acceptance Filter busy	    */
#define XCANPS_SR_TXFLL_MASK	0x00000400 /**< TX FIFO is full		    */
#define XCANPS_SR_TXBFLL_MASK	0x00000200 /**< TX High Priority Buffer full*/
#define XCANPS_SR_ESTAT_MASK	0x00000180 /**< Error Status		    */
#define XCANPS_SR_ESTAT_SHIFT	7
/* Indicates the error status of the CAN controller.
   00: Indicates Configuration Mode (CONFIG = 1).  Error State is undefined.
   01: Indicates Error Active State.
   11: Indicates Error Passive State.
   10: Indicates Bus Off State.						    */
#define XCANPS_SR_ERRWRN_MASK	0x00000040 /**< Error Warning		    */
/* Indicates that either the Transmit Error counter
   or the Receive Error counter has exceeded a value of 96.		    */
#define XCANPS_SR_BBSY_MASK	0x00000020 /**< Bus Busy		    */
#define XCANPS_SR_BIDLE_MASK	0x00000010 /**< Bus Idle		    */
#define XCANPS_SR_NORMAL_MASK	0x00000008 /**< Normal Mode		    */
#define XCANPS_SR_SLEEP_MASK	0x00000004 /**< Sleep Mode		    */
#define XCANPS_SR_LBACK_MASK	0x00000002 /**< Loop Back Mode		    */
#define XCANPS_SR_CONFIG_MASK	0x00000001 /**< Configuration Mode	    */


/* Error Status Register (ESR) Bit Definitions and Masks		    */
/* The Error Status Register (ESR)
   indicates the type of error that has occurred on the bus.
   If more than one error occurs,
   all relevant error flag bits are set in this register.
   The ESR is a write-to-clear register.
   Writes to this register will not set any bits,
   but will clear the bits that are set.				    */ 
#define XCANPS_ESR_ACKER_MASK	0x00000010 /**< ACK Error		    */
#define XCANPS_ESR_BERR_MASK	0x00000008 /**< Bit Error		    */
#define XCANPS_ESR_STER_MASK	0x00000004 /**< Stuff Error		    */
#define XCANPS_ESR_FMER_MASK	0x00000002 /**< Form Error		    */
#define XCANPS_ESR_CRCER_MASK	0x00000001 /**< CRC Error		    */


/* Interrupt Status/Enable/Clear Register Bit Definitions and Masks	    */
/* The Interrupt Status Register (ISR) contains bits
   that are set when a particular interrupt condition occurs.
   If the corresponding mask bit in the Interrupt Enable Register is set,
   an interrupt is generated.
   Interrupt bits in the ISR can be cleared
   by writing to the Interrupt Clear Register.
   For all bits in the ISR,
   a set condition takes priority over the clear condition
   and the bit continues to remain '1.'					    */
#define XCANPS_IXR_TXFEMP_MASK   0x00004000 /**< Tx Fifo Empty Interrupt    */
#define XCANPS_IXR_TXFWMEMP_MASK 0x00002000 /**< Tx Fifo Watermark Empty    */
#define XCANPS_IXR_RXFWMFLL_MASK 0x00001000 /**< Rx FIFO Watermark Full	    */
#define XCANPS_IXR_WKUP_MASK     0x00000800 /**< Wake up Interrupt	    */
#define XCANPS_IXR_SLP_MASK	 0x00000400 /**< Sleep Interrupt	    */
#define XCANPS_IXR_BSOFF_MASK	 0x00000200 /**< Bus Off Interrupt	    */
#define XCANPS_IXR_ERROR_MASK	 0x00000100 /**< Error Interrupt	    */
#define XCANPS_IXR_RXNEMP_MASK	 0x00000080 /**< RX FIFO Not Empty Interrupt*/
#define XCANPS_IXR_RXOFLW_MASK	 0x00000040 /**< RX FIFO Overflow Interrupt */
#define XCANPS_IXR_RXUFLW_MASK	 0x00000020 /**< RX FIFO Underflow Interrupt*/
#define XCANPS_IXR_RXOK_MASK	 0x00000010 /**< New Message Received Intr  */
#define XCANPS_IXR_TXBFLL_MASK	 0x00000008 /**< TX High Priority Buf Full  */
#define XCANPS_IXR_TXFLL_MASK	 0x00000004 /**< TX FIFO Full Interrupt	    */
#define XCANPS_IXR_TXOK_MASK	 0x00000002 /**< TX Successful Interrupt    */
#define XCANPS_IXR_ARBLST_MASK	 0x00000001 /**< Arbitration Lost Interrupt */
#define XCANPS_IXR_ALL		(XCANPS_IXR_RXFWMFLL_MASK | \
				XCANPS_IXR_WKUP_MASK   | \
				XCANPS_IXR_SLP_MASK	| \
				XCANPS_IXR_BSOFF_MASK  | \
				XCANPS_IXR_ERROR_MASK  | \
				XCANPS_IXR_RXNEMP_MASK | \
 				XCANPS_IXR_RXOFLW_MASK | \
				XCANPS_IXR_RXUFLW_MASK | \
	 			XCANPS_IXR_RXOK_MASK   | \
				XCANPS_IXR_TXBFLL_MASK | \
				XCANPS_IXR_TXFLL_MASK  | \
				XCANPS_IXR_TXOK_MASK   | \
				XCANPS_IXR_ARBLST_MASK)

/* CAN Timestamp Control Register (TCR) Bit Definitions and Masks	    */
#define XCANPS_TCR_CTS_MASK	0x00000001 /**< Clear Timestamp counter mask*/

/* CAN Watermark Register (WIR) Bit Definitions and Masksa		    */
#define XCANPS_WIR_FW_MASK   	0x0000003F /**< Rx Full Threshold mask	    */
#define XCANPS_WIR_EW_MASK 	0x00003F00 /**< Tx Empty Threshold mask	    */
#define XCANPS_WIR_EW_SHIFT 	0x00000008 /**< Tx Empty Threshold shift    */


/* CAN Frame Identifier
   (TX High Priority Buffer/TX/RX/Acceptance Filter Mask/Acceptance Filter ID)	
 */
#define XCANPS_IDR_ID1_MASK	0xFFE00000 /**< Standard Messg Identifier   */
#define XCANPS_IDR_ID1_SHIFT	21
#define XCANPS_IDR_ID1_X_SHIFT	3
#define XCANPS_IDR_SRR_MASK	0x00100000 /**< Substitute Remote TX Req    */
#define XCANPS_IDR_SRR_SHIFT	20
#define XCANPS_IDR_IDE_MASK	0x00080000 /**< Identifier Extension	    */
#define XCANPS_IDR_IDE_SHIFT	19
#define XCANPS_IDR_ID2_MASK	0x0007FFFE /**< Extended Message Ident	    */
#define XCANPS_IDR_ID2_SHIFT	1
#define XCANPS_IDR_RTR_MASK	0x00000001 /**< Remote TX Request	    */

/* CAN Frame Data Length Code (TX High Priority Buffer/TX/RX)		    */
#define XCANPS_DLCR_DLC_MASK	 0xF0000000	/**< Data Length Code	    */
#define XCANPS_DLCR_DLC_SHIFT	 28
#define XCANPS_DLCR_TIMESTAMP_MASK 0x0000FFFF /**< Timestamp Mask (Rx only) */


/* CAN Frame Data Word 1 (TX High Priority Buffer/TX/RX)		    */
#define XCANPS_DW1R_DB0_MASK	0xFF000000 /**< Data Byte 0		    */
#define XCANPS_DW1R_DB0_SHIFT	24
#define XCANPS_DW1R_DB1_MASK	0x00FF0000 /**< Data Byte 1		    */
#define XCANPS_DW1R_DB1_SHIFT	16
#define XCANPS_DW1R_DB2_MASK	0x0000FF00 /**< Data Byte 2		    */
#define XCANPS_DW1R_DB2_SHIFT	8
#define XCANPS_DW1R_DB3_MASK	0x000000FF /**< Data Byte 3		    */

/* CAN Frame Data Word 2 (TX High Priority Buffer/TX/RX)		    */
#define XCANPS_DW2R_DB4_MASK	0xFF000000 /**< Data Byte 4		    */
#define XCANPS_DW2R_DB4_SHIFT	24
#define XCANPS_DW2R_DB5_MASK	0x00FF0000 /**< Data Byte 5		    */
#define XCANPS_DW2R_DB5_SHIFT	16
#define XCANPS_DW2R_DB6_MASK	0x0000FF00 /**< Data Byte 6		    */
#define XCANPS_DW2R_DB6_SHIFT	8
#define XCANPS_DW2R_DB7_MASK	0x000000FF /**< Data Byte 7		    */

/* Acceptance Filter Register (AFR) Bit Definitions and Masks		    */
#define XCANPS_AFR_UAF4_MASK	0x00000008 /**< Use Acceptance Filter No.4  */
#define XCANPS_AFR_UAF3_MASK	0x00000004 /**< Use Acceptance Filter No.3  */
#define XCANPS_AFR_UAF2_MASK	0x00000002 /**< Use Acceptance Filter No.2  */
#define XCANPS_AFR_UAF1_MASK	0x00000001 /**< Use Acceptance Filter No.1  */
#define XCANPS_AFR_UAF_ALL_MASK	(XCANPS_AFR_UAF4_MASK | \
					XCANPS_AFR_UAF3_MASK | \
					XCANPS_AFR_UAF2_MASK | \
					XCANPS_AFR_UAF1_MASK)


/* CAN frame length constants (4 words a 4 bytes)			    */
#define XCANPS_MAX_FRAME_SIZE 16 /**< Maximum CAN frame length in bytes	    */

/* For backwards compatibilty */
#define XCANPS_TXBUF_ID_OFFSET   XCANPS_TXHPB_ID_OFFSET
#define XCANPS_TXBUF_DLC_OFFSET  XCANPS_TXHPB_DLC_OFFSET
#define XCANPS_TXBUF_DW1_OFFSET  XCANPS_TXHPB_DW1_OFFSET
#define XCANPS_TXBUF_DW2_OFFSET  XCANPS_TXHPB_DW2_OFFSET

#define XCANPS_RXFWIR_RXFLL_MASK XCANPS_WIR_FW_MASK
#define XCANPS_RXWIR_OFFSET 	 XCANPS_WIR_OFFSET
#define XCANPS_IXR_RXFLL_MASK 	 XCANPS_IXR_RXFWMFLL_MASK





/*---------- Timing values */
/* generated bit rate table by
 * http://www.port.de/engl/canprod/sv_req_form.html
 *
 *
 * BTRO == prescaler BRPR
 * BTR1 == Phase segments BTR
 *
 *
 * XCANPS register BTR contains JJW, TS2 and TS1
 * in the lowest bits of the 32 bit register.
 *
 * XCANps register BRPR contains the Baud Rate Prescaler
 * in the lowest byte (0..255) of the 32 bit register.
 *
 * The value of the CAN_SYSCLK is the same value used 
 * as 'clock rate' in the calculation form.
 */
#if CAN_SYSCLK == 10
#  define CAN_BRPR_10K		0x31
#  define CAN_BTR_10K		0x2f
#  define CAN_BRPR_20K		0x18	
#  define CAN_BTR_20K		0x2f
#  define CAN_BRPR_50K		0x09
#  define CAN_BTR_50K		0x2f
#  define CAN_BRPR_100K         0x04 
#  define CAN_BTR_100K          0x2f
#  define CAN_BRPR_125K		0x04
#  define CAN_BTR_125K		0x1c
#  define CAN_BRPR_250K		   1
#  define CAN_BTR_250K		0x2f
#  define CAN_BRPR_500K		   0
#  define CAN_BTR_500K		0x2f
#  define CAN_BRPR_800K		   0	/* not supported */
#  define CAN_BTR_800K		0xFF
#  define CAN_BRPR_1000K	   0
#  define CAN_BTR_1000K		0x07

#define CAN_SYSCLK_is_ok            1

#endif

#if CAN_SYSCLK == 24000000
#  define CAN_BRPR_10K		0x95
#  define CAN_BTR_10K		0x1c
#  define CAN_BRPR_20K		0x4a
#  define CAN_BTR_20K		0x1c
#  define CAN_BRPR_50K		0x1d
#  define CAN_BTR_50K		0x1c
#  define CAN_BRPR_100K         0x0e 
#  define CAN_BTR_100K          0x1c
#  define CAN_BRPR_125K		0x0b
#  define CAN_BTR_125K		0x1c
#  define CAN_BRPR_250K		0x05
#  define CAN_BTR_250K		0x1c
#  define CAN_BRPR_500K		0x02
#  define CAN_BTR_500K		0x1c
#  define CAN_BRPR_800K		   1
#  define CAN_BTR_800K		0x1b
#  define CAN_BRPR_1000K	   1
#  define CAN_BTR_1000K		0x09

#define CAN_SYSCLK_is_ok            1
#endif

#if CAN_SYSCLK == 20000000		/* 20 MHz */
#  define CAN_BRPR_10K		0x7c
#  define CAN_BTR_10K		0x1c
#  define CAN_BRPR_20K		0x31
#  define CAN_BTR_20K		0x2f
#  define CAN_BRPR_50K		0x18
#  define CAN_BTR_50K		0x1c
#  define CAN_BRPR_100K         0x09 
#  define CAN_BTR_100K          0x2f
#  define CAN_BRPR_125K		0x09
#  define CAN_BTR_125K		0x1c
#  define CAN_BRPR_250K		0x04
#  define CAN_BTR_250K		0x1c
#  define CAN_BRPR_500K		0x01
#  define CAN_BTR_500K		0x2f
#  define CAN_BRPR_800K		0x00
#  define CAN_BTR_800K		0x7f
#  define CAN_BRPR_1000K	0x00
#  define CAN_BTR_1000K		0x2f

#define CAN_SYSCLK_is_ok            1
#endif

#if CAN_SYSCLK == 22000000
#  define CAN_BRPR_10K		0x00	/* not possible */
#  define CAN_BTR_10K		0xFF
#  define CAN_BRPR_20K		0x36
#  define CAN_BTR_20K		0x2f
#  define CAN_BRPR_50K		0x15
#  define CAN_BTR_50K		0x2f
#  define CAN_BRPR_100K         0x0a 
#  define CAN_BTR_100K          0x2f
#  define CAN_BRPR_125K		0x0a
#  define CAN_BTR_125K		0x1c
#  define CAN_BRPR_250K		0x03
#  define CAN_BTR_250K		0x4f
#  define CAN_BRPR_500K		0x01
#  define CAN_BTR_500K		0x4f
#  define CAN_BRPR_800K		   0
#  define CAN_BTR_800K		0xFF
#  define CAN_BRPR_1000K	   0
#  define CAN_BTR_1000K		0x4f

#define CAN_SYSCLK_is_ok            1
#endif

#if CAN_SYSCLK == 11000000
#  define CAN_BRPR_10K		0x36
#  define CAN_BTR_10K		0x2f
#  define CAN_BRPR_20K		0x18
#  define CAN_BTR_20K		0x4f
#  define CAN_BRPR_50K		0x0a
#  define CAN_BTR_50K		0x2f
#  define CAN_BRPR_100K         0x04 
#  define CAN_BTR_100K          0x4f
#  define CAN_BRPR_125K		0x03
#  define CAN_BTR_125K		0x4f
#  define CAN_BRPR_250K		0x01
#  define CAN_BTR_250K		0x4f
#  define CAN_BRPR_500K		0x00
#  define CAN_BTR_500K		0x4f
#  define CAN_BRPR_800K		   0
#  define CAN_BTR_800K		0xFF
#  define CAN_BRPR_1000K	   0
#  define CAN_BTR_1000K		0x08

#define CAN_SYSCLK_is_ok            1
#endif

#if CAN_SYSCLK == 10000000
#  define CAN_BRPR_10K		0x31
#  define CAN_BTR_10K		0x2f
#  define CAN_BRPR_20K		0x18
#  define CAN_BTR_20K		0x2f
#  define CAN_BRPR_50K		0x09
#  define CAN_BTR_50K		0x2f
#  define CAN_BRPR_100K         0x04 
#  define CAN_BTR_100K          0x2f
#  define CAN_BRPR_125K		0x04
#  define CAN_BTR_125K		0x1c
#  define CAN_BRPR_250K		0x01
#  define CAN_BTR_250K		0x2f
#  define CAN_BRPR_500K		0x00
#  define CAN_BTR_500K		0x2f
#  define CAN_BRPR_800K		   0
#  define CAN_BTR_800K		0xFF	/* not */
#  define CAN_BRPR_1000K	0x00
#  define CAN_BTR_1000K		0x1b

#define CAN_SYSCLK_is_ok            1
#endif
/* for more CAN_SYSCLK values */


#ifndef CAN_SYSCLK_is_ok
#  error Please specify a valid CAN_SYSCLK value (i.e. 8, 10) or define new parameters
#endif

#endif 	/* __CAN_XCANPS__ */
