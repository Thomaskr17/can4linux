#include "defs.h"
#include <asm/types.h>

#if defined(CAN_PORT_IO) || defined(CAN_INDEXED_PORT_IO)
# error "Only memeory access allowed, "
#endif


typedef struct canregs {
    u32	    srr;	/* Software Reset Register			    */
    u32	    msr;	/* Mode Select Register				    */
    u32	    brpr;	/* Baud Rate Prescaler Register			    */
    u32	    btr;	/* Bit Timing Register				    */
    u32	    ecr;	/* Error Counter Register			    */
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


#if 0
/* Memory long word access */
#define CANoutl(bd,adr,v)	\
		(writel(v, (u32) &((canregs_t *)can_base[bd])->adr ))
#define CANsetl(bd,adr,m)	\
	writel((readl((u32) &((canregs_t *)can_base[bd])->adr)) \
		| (m) , (u32) &((canregs_t *)can_base[bd])->adr )
#define CANresetl(bd,adr,m)	\
	writel((readl((u32) &((canregs_t *)can_base[bd])->adr)) \
		& ~(m), (u32) &((canregs_t *)can_base[bd])->adr )
#define CANinl(bd,adr)		\
		(readl ((u32) &((canregs_t *)can_base[bd])->adr  ))
#define CANtestl(bd,adr,m)	\
	(readl((u32) &((canregs_t *)can_base[bd])->adr  ) & (m) )




/* Reset CAN controller


	XCanPs_WriteReg(InstancePtr->CanConfig.BaseAddr, XCANPS_SRR_OFFSET, \
			   XCANPS_SRR_SRST_MASK);

*/
#endif
