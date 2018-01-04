/*
 * can_i82527funcs.c - i82527 related code
 *
 * Original version Written by Arnaud Westenberg email:arnaud@wanadoo.nl
 * This software is released under the GPL-License.
 *
 * Modified and extended to support the esd elctronic system
 * design gmbh PC/104-CAN Card (www.esd-electronics.com)
 * by Jean-Jacques Tchouto (tchouto@fokus.fraunhofer.de), 2003
 *
 * Major Refactoring and Integration into can4linux version 3.1 by
 * Henrik W Maier of FOCUS Software Engineering Pty Ltd <www.focus-sw.com>
 *
 * Modified and extended to support the SBS PC7compact DINrail mounted
 * industry PC by FOCUS Software Engineering Pty Ltd <www.focus-sw.com>
 *
 * Updated for 2.6 kernel by Henrik W Maier of FOCUS Software
 * Engineering Pty Ltd <www.focus-sw.com>.
 * Bugfix in CAN_SendMessage, added wake_up_interruptible for irq_write_handler
 * Disabled redundant status change interrupt for RX/TX to decrease int
 * load on CPU.
 * Fixed issue of a second iteration in the ISR when sending because the
 * send interrupt didn't sometimes get acknowledged.
 *
 * $Id: i82527funcs.c,v 1.1 2006/04/21 16:26:51 oe Exp $
 *
 */


#include "defs.h"
#include "i82527.h"


#if defined (PC104_200)
inline void CANactivateIRQline(int bd)
{
    unsigned long canIoPort;
    canIoPort = (unsigned long)Base[bd];
    outb_p(0x86, canIoPort+3);
}
#endif


/*
 * Timing values
 */
uint8_t iCanTiming[10][2]={
	{CAN_TIM0_10K,  CAN_TIM1_10K},
	{CAN_TIM0_20K,  CAN_TIM1_20K},
	{CAN_TIM0_40K,  CAN_TIM1_40K},
	{CAN_TIM0_50K,  CAN_TIM1_50K},
	{CAN_TIM0_100K, CAN_TIM1_100K},
	{CAN_TIM0_125K, CAN_TIM1_125K},
	{CAN_TIM0_250K, CAN_TIM1_250K},
	{CAN_TIM0_500K, CAN_TIM1_500K},
	{CAN_TIM0_800K, CAN_TIM1_800K},
	{CAN_TIM0_1000K,CAN_TIM1_1000K}};


/*
 * Clear and invalidate message objects
 */
int CAN_ClearObjects(int board)
{
    int i;
    int id;
    int data;

    DBGin("CAN_ClearObjects");

    for (i = 1; i <= 15; i++) {
        CANout(board, msgArr[i].messageReg.msgCtrl0Reg,
            INTPD_RES | RXIE_RES | TXIE_RES | MVAL_RES);
        CANout(board, msgArr[i].messageReg.msgCtrl1Reg,
            NEWD_RES | MLST_RES | TXRQ_RES | RMPD_RES);
        for (data = 0; data < 8; data++)
            CANout(board, msgArr[i].messageReg.dataReg[data], 0);
        for (id = 0; id < 4; id++)
            CANout(board, msgArr[i].messageReg.idReg[id], 0);
        CANout(board, msgArr[i].messageReg.messageConfigReg, 0);
    }

    DBGout();
    return 0;
}


/*
 * Board and chip reset
 *
 * Resets and fully configures the chip. Sets baud rate and masks.
 *
 * Note: The chip remains bus-off. All interrupts remain disabled.
 */
int CAN_ChipReset(int board)
{
    DBGin("CAN_ChipReset");

    // Configure cpu interface
    CANout(board, cpuInterfaceReg, (iCPU_DMC | iCPU_DSC | iCPU_CEN));

    // Enable configuration and puts chip in bus-off, disable interrupts
    CANout(board, controlReg, (iCTL_CCE | iCTL_INI));

    // Set clock out slew rates
    CANout(board, clkOutReg, (iCLK_SL1 | iCLK_CD1));

    // Bus configuration
    CANout(board, busConfigReg, (iBUS_CBY));

    // Clear interrupts
    CANin(board, interruptReg);

    // Clear status register
    CANout(board, statusReg, 0);

    // Write test pattern
    CANout(board, message1Reg.dataReg[1], 0x25);
    CANout(board, message2Reg.dataReg[3], 0x52);
    CANout(board, message10Reg.dataReg[6], 0xc3);

    // Read back test pattern
    if ((CANin(board, message1Reg.dataReg[1]) != 0x25 ) ||
        (CANin(board, message2Reg.dataReg[3]) != 0x52 ) ||
        (CANin(board, message10Reg.dataReg[6]) != 0xc3 )) {
        DBGprint(DBG_DATA,("Could not read back from the hardware."));
        DBGprint(DBG_DATA,("This probably means that your hardware is not correctly configured!"));
        return -1;
    }
    else {
        DBGprint(DBG_DATA,("Could read back, hardware is probably configured correctly"));
    }

    CAN_ClearObjects(board);
    CAN_SetTiming(board, Baud[board]);

    CANout(board, globalMaskStandardReg[0], 0);	
    CANout(board, globalMaskStandardReg[1], 0);
    CANout(board, globalMaskExtendedReg[0], 0);
    CANout(board, globalMaskExtendedReg[1], 0);
    CANout(board, globalMaskExtendedReg[2], 0);
    CANout(board, globalMaskExtendedReg[3], 0);
    // Set message 15 mask, we are not using it, only standard mask is used
    // We set all bits to one because this mask is anded with the global mask.
    CANout(board, message15MaskReg[0], 0xFF);
    CANout(board, message15MaskReg[1], 0xFF);
    CANout(board, message15MaskReg[2], 0xFF);
    CANout(board, message15MaskReg[3], 0xFF);

    DBGprint(DBG_DATA, ("[%d] CAN_CON 0x%x\n", board, CANin(board, controlReg)));
    DBGprint(DBG_DATA, ("[%d] CAN_CPU 0x%x\n", board, CANin(board, cpuInterfaceReg)));

    // Note: At this stage the CAN ship is still in bus-off condition
    // and must be started using StartChip()

    DBGout();
    return 0;
}




/*
 * Configures bit timing registers directly. Chip must be in configuration mode.
 */
int CAN_SetBTR (int board, int btr0, int btr1)
{
    DBGin("i82527_CAN_SetBTR");
    DBGprint(DBG_DATA, ("[%d] btr0=%d, btr1=%d", board, btr0, btr1));

    CANout(board, bitTiming0Reg, (uint8_t) (btr0 & 0xff ));
    CANout(board, bitTiming1Reg, (uint8_t) (btr1 & 0xff ));
    DBGprint(DBG_DATA,("[%d] CAN_BTIME0 0x%x CAN_BTIME1 0x%x", baud,
                       CANin(board, bitTiming0Reg),
                       CANin(board, bitTiming1Reg)));

    DBGout();
    return 0;
}


/*
 * Configures bit timing. Chip must be in configuration mode.
 */
int CAN_SetTiming(int board, int baud)
{
    int i = 5;
    int custom = 0;

    DBGin("i82527_CAN_SetTiming");
    DBGprint(DBG_DATA, ("baud[%d]=%d", board, baud));

    switch(baud)
    {
        case   10: i = 0; break;
        case   20: i = 1; break;
        case   40: i = 2; break;
        case   50: i = 3; break;
        case  100: i = 4; break;
        case  125: i = 5; break;
        case  250: i = 6; break;
        case  500: i = 7; break;
        case  800: i = 8; break;
        case 1000: i = 9; break;
        default:
            custom = 1;
    }
    if( custom ) {
        CANout(board, bitTiming0Reg, (uint8_t) (baud >> 8) & 0xff);
        CANout(board, bitTiming1Reg, (uint8_t) (baud & 0xff ));
    } else {
        CANout(board, bitTiming0Reg, (uint8_t) iCanTiming[i][0]);
        CANout(board, bitTiming1Reg, (uint8_t) iCanTiming[i][1]);
    }
    DBGprint(DBG_DATA,("[%d] CAN_BTIME0 0x%x CAN_BTIME1 0x%x", baud,
                       CANin(board, bitTiming0Reg),
                       CANin(board, bitTiming1Reg)));

    DBGout();
    return 0;
}


/*
 * Enables interrupts and put chip out off bus-off mode
 * Configures message objects for receiption.
 */
int CAN_StartChip(int board)
{
    DBGin("CAN_StartChip");

    RxErr[board] = 0;
    TxErr[board] = 0;

    // Clear interrupts
    CANin(board, interruptReg);

    // Clear status register
    CANout(board, statusReg, 0);

    // Configure message object for receiption
    CANout(board, message15Reg.msgCtrl1Reg,
           NEWD_RES | MLST_RES | TXRQ_RES | RMPD_RES);
    CANout(board, message15Reg.msgCtrl0Reg,
           MVAL_SET | TXIE_RES | RXIE_SET | INTPD_RES);

    CAN_SetMask(board, AccCode[board], AccMask[board]);

    // Clear message object for send
    CANout(board, message1Reg.msgCtrl1Reg,
           RMPD_RES | TXRQ_RES | CPUU_RES | NEWD_RES);
    CANout(board, message1Reg.msgCtrl0Reg,
           MVAL_RES | TXIE_RES | RXIE_RES | INTPD_RES);

    // Clear bus-off, Interrupts only for errors, not for status change
    CANout(board, controlReg, iCTL_IE | iCTL_EIE);

    DBGprint(DBG_DATA, ("[%d] CAN_CON 0x%x\n", board, CANin(board, controlReg)));

    DBGout();
    return 0;
}


/*
 * Puts chip in bus-off mode
 *
 * Enable configuration and puts chip in bus-off, disable interrupts.
 * Invalidates message objects.
 */
int CAN_StopChip(int board)
{
    DBGin("CAN_StopChip");

    // Enable configuration and puts chip in bus-off, disable interrupts
    CANout(board, controlReg, iCTL_CCE | iCTL_INI);

    // Clear interrupts
    CANin(board, interruptReg);

    // Clear status register
    CANout(board, statusReg, 0);

    // Clear message object for receiption
    CANout(board, message15Reg.msgCtrl1Reg,
           NEWD_RES | MLST_RES | TXRQ_RES | RMPD_RES);
    CANout(board, message15Reg.msgCtrl0Reg,
           MVAL_RES | TXIE_RES | RXIE_RES | INTPD_RES);

    // Clear message object for send
    CANout(board, message1Reg.msgCtrl1Reg,
           RMPD_RES | TXRQ_RES | CPUU_RES | NEWD_RES);
    CANout(board, message1Reg.msgCtrl0Reg,
           MVAL_RES | TXIE_RES | RXIE_RES | INTPD_RES);

    DBGprint(DBG_DATA, ("[%d] CAN_CON 0x%x\n", board, CANin(board, controlReg)));

    DBGout();
    return 0;
}


int CAN_SendMessage(int board, canmsg_t *tx)
{
    int i = 0;
    int ext;
    uint8_t id0, id1, id2, id3;

    DBGin("i82527_CAN_SendMessage");

    DBGin("Wait if there is a transmission in progress");
    // Wait if there is a transmission in progress
    while ((CANin(board, message1Reg.msgCtrl1Reg) & TXRQ_UNC) == TXRQ_SET) {
	    #if LINUX_VERSION_CODE >= 131587
	    # if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
	    cond_resched();
	    # else
	    if( current->need_resched )
        schedule();
	    # endif
	    #else
	    if( need_resched ) schedule();
	    #endif
    }
    DBGout();

    CANout(board, message1Reg.msgCtrl1Reg,
           RMPD_RES | TXRQ_RES | CPUU_SET | NEWD_SET);
    CANout(board,message1Reg.msgCtrl0Reg,
           MVAL_SET | TXIE_SET | RXIE_RES | INTPD_RES);

    tx->length %= 9; // Limit CAN message length to 8
    ext = (tx->flags & MSG_EXT); // Extended ID?
    if ( ext ) {
        CANout(board, message1Reg.messageConfigReg,
               ( tx->length << 4 ) + ( MCFG_DIR | MCFG_XTD ));
        id0 = (uint8_t)( tx->id << 3 );
        id1 = (uint8_t)( tx->id >> 5 );
        id2 = (uint8_t)( tx->id >> 13 );
        id3 = (uint8_t)( tx->id >> 21 );
        CANout(board, message1Reg.idReg[3], id0);
        CANout(board, message1Reg.idReg[2], id1);
        CANout(board, message1Reg.idReg[1], id2);
        CANout(board, message1Reg.idReg[0], id3);
    }
    else {
        CANout(board, message1Reg.messageConfigReg,
               ( tx->length << 4 ) + MCFG_DIR);
        id1 = (uint8_t)( tx->id << 5 );
        id0 = (uint8_t)( tx->id >> 3 );
        CANout(board, message1Reg.idReg[1], id1);
        CANout(board, message1Reg.idReg[0], id0);
    }

    for ( i=0; i < tx->length; i++ ) {
        CANout(board, message1Reg.dataReg[i], tx->data[i]);
    }

    if ( (tx->flags & MSG_RTR) == MSG_RTR ) {
        CANout(board,message1Reg.msgCtrl1Reg,
               RMPD_RES | TXRQ_RES | CPUU_RES | NEWD_UNC);
    }
    else {
        CANout(board,message1Reg.msgCtrl1Reg,
               RMPD_RES | TXRQ_SET | CPUU_RES | NEWD_UNC);

    }

    DBGout();
    return i;
}


/*
 * ioctl poll
 */
int CAN_GetMessage(int board, canmsg_t *rx)
{
    int i = 0;
    uint8_t msgctlreg;
    uint8_t ctl1reg;

    DBGin("i82527_CAN_GetMessage");

    ctl1reg = CANin(board, message15Reg.msgCtrl1Reg);
    rx->flags = 0;
    rx->length = 0;

    if( ctl1reg & MLST_SET ) {
        Overrun[board]++;
        DBGprint(DBG_DATA,("i82527: Previous message lost!\n"));
    }

    if(ctl1reg & NEWD_SET) {

        do_gettimeofday(&rx->timestamp);

        if (ctl1reg & RMPD_SET) {
            rx->flags |= MSG_RTR;
        }

        msgctlreg = CANin(board, message15Reg.messageConfigReg);
        if( msgctlreg & MCFG_XTD ) {
            int id0, id1, id2, id3;

            id0 = (unsigned int)(CANin(board, message15Reg.idReg[3]));
            id1 = (unsigned int)(CANin(board, message15Reg.idReg[2])) << 8;
            id2 = (unsigned int)(CANin(board, message15Reg.idReg[1])) << 16;
            id3 = (unsigned int)(CANin(board, message15Reg.idReg[0])) << 24;
            rx->flags |= MSG_EXT;
            rx->id =(id0 | id1 | id2 | id3) >> 3;

        } else {
            int id0, id1;

            id0 = (unsigned int)(CANin(board, message15Reg.idReg[1]));
            id1 = (unsigned int)(CANin(board, message15Reg.idReg[0])) << 8;
            rx->id =(id0 | id1 ) >> 5;
        }

        msgctlreg  &= 0xf0;/* strip length code */
        msgctlreg  = msgctlreg >>4;

        rx->length = msgctlreg;
        msgctlreg %= 9;	/* limit count to 8 bytes */

        for (i = 0; i < msgctlreg; i++) {
            rx->data[i] = CANin(board, message15Reg.dataReg[i]);
            DBGprint(DBG_DATA,("rx[%d]: 0x%x",i, rx->data[i]));
        }

        // Make the chip ready to receive the next message
        CANout(board,message15Reg.msgCtrl0Reg,
               MVAL_SET | TXIE_RES | RXIE_SET | INTPD_RES);
        CANout(board,message15Reg.msgCtrl1Reg,
               RMPD_RES | TXRQ_RES | MLST_RES |  NEWD_RES);
    }
    else {
        i = 0;
    }

    DBGout();
    return i;
}


/*
 * Subroutine of ISR for RX interrupts.
 *
 * Note: This code depends on using message object 15 for receiving.
 * Object 15 has a double buffer and using this routine would not work
 * reliably on other message objects!
 */
static void i82527_irq_read_msg15_handler(int board, msg_fifo_t *RxFifo)
{
    int i;
    uint8_t msgctlreg;
    uint8_t ctl1reg;

    DBGin("i82527_irq_read_msg15_handler");

    ctl1reg = CANin(board, message15Reg.msgCtrl1Reg);
    while (ctl1reg & NEWD_SET) {

        do_gettimeofday(&(RxFifo->data[RxFifo->head]).timestamp);

        if (ctl1reg & MLST_SET) {
            Overrun[board]++;
            DBGprint(DBG_DATA,("i82527: Previous message lost!\n"));
        }

        if (ctl1reg & RMPD_SET) {
            (RxFifo->data[RxFifo->head]).flags |= MSG_RTR;
        }

        msgctlreg = CANin(board, message15Reg.messageConfigReg);
        if( msgctlreg & MCFG_XTD ) {
            int id0, id1, id2, id3;

            id0 = (unsigned int)(CANin(board, message15Reg.idReg[3]));
            id1 = (unsigned int)(CANin(board, message15Reg.idReg[2])) << 8;
            id2 = (unsigned int)(CANin(board, message15Reg.idReg[1])) << 16;
            id3 = (unsigned int)(CANin(board, message15Reg.idReg[0])) << 24;
            (RxFifo->data[RxFifo->head]).flags |= MSG_EXT;
            (RxFifo->data[RxFifo->head]).id =(id0 | id1 | id2 | id3) >> 3;

        } else {
            int id0, id1;

            id0 = (unsigned int)(CANin(board, message15Reg.idReg[1]));
            id1 = (unsigned int)(CANin(board, message15Reg.idReg[0])) << 8;
            (RxFifo->data[RxFifo->head]).id =(id0 | id1 ) >> 5;
        }

        msgctlreg  &= 0xf0;/* strip length code */
        msgctlreg  = msgctlreg >>4;

        (RxFifo->data[RxFifo->head]).length = msgctlreg;
        msgctlreg %= 9;	/* limit count to 8 bytes */

        for (i = 0; i < msgctlreg; i++) {
            (RxFifo->data[RxFifo->head]).data[i] =
                CANin(board, message15Reg.dataReg[i]);
            DBGprint(DBG_DATA,("rx[%d]: 0x%x",i,
                               (RxFifo->data[RxFifo->head]).data[i]));
        }
        RxFifo->status = BUF_OK;
        RxFifo->head = ++(RxFifo->head) % MAX_BUFSIZE;

        if(RxFifo->head == RxFifo->tail) {
            RxFifo->status = BUF_OVERRUN;
        }

        // Make the chip ready to receive the next message
        CANout(board, message15Reg.msgCtrl0Reg,
               MVAL_SET | TXIE_RES | RXIE_SET | INTPD_RES);
        CANout(board, message15Reg.msgCtrl1Reg,
               RMPD_RES | TXRQ_RES | MLST_RES | NEWD_RES);

        // Notify user app
        wake_up_interruptible(  &CanWait[board] );

        ctl1reg = CANin(board, message15Reg.msgCtrl1Reg);
    }

    DBGout();
}


/*
 * Subroutine of ISR for TX interrupts
 */
static void i82527_irq_write_handler(int board, msg_fifo_t *TxFifo)
{
    unsigned long flags;
    uint8_t tx2reg;
    unsigned int id;
    int ext;
    int i;
    uint8_t id0, id1, id2, id3;

    DBGin("i82527_irq_write_handler");

    // Enter critical section
    save_flags(flags);
    cli();

    if( TxFifo->free[TxFifo->tail] == BUF_EMPTY ) {
        // Nothing more to send, switch off interrupts
        CANout(board, message1Reg.msgCtrl0Reg,
               (MVAL_RES | TXIE_RES | RXIE_RES | INTPD_RES));
        TxFifo->status = BUF_EMPTY;
        TxFifo->active = 0;
        // We had some cases of repeated IRQ, so make sure the INT is acknowledged
        CANout(board, message1Reg.msgCtrl0Reg,
               (MVAL_UNC | TXIE_UNC | RXIE_UNC | INTPD_RES));
        /* leave critical section */
        restore_flags(flags);
        // Notify user app
        wake_up_interruptible(  &CanOutWait[board] );
        DBGout();
        return;
    }
    CANout(board, message1Reg.msgCtrl1Reg,
                 RMPD_RES | TXRQ_RES | CPUU_SET | NEWD_RES);
    CANout(board, message1Reg.msgCtrl0Reg,
                 MVAL_SET | TXIE_SET | RXIE_RES | INTPD_RES);

    tx2reg = (TxFifo->data[TxFifo->tail]).length;

    ext = (TxFifo->data[TxFifo->tail]).flags & MSG_EXT;
    id = (TxFifo->data[TxFifo->tail]).id;

    if ( ext ) {
        CANout(board, message1Reg.messageConfigReg,
                     (tx2reg << 4 ) + ( MCFG_DIR | MCFG_XTD ));
        id0 = (uint8_t)( id << 3 );
        id1 = (uint8_t)( id >> 5 );
        id2 = (uint8_t)( id >> 13 );
        id3 = (uint8_t)( id >> 21 );
        CANout(board, message1Reg.idReg[3], id0);
        CANout(board, message1Reg.idReg[2], id1);
        CANout(board, message1Reg.idReg[1], id2);
        CANout(board, message1Reg.idReg[0], id3);
    }
    else {
        CANout(board, message1Reg.messageConfigReg,
               ( tx2reg << 4 ) + MCFG_DIR);
        id1 = (uint8_t)( id << 5 );
        id0 = (uint8_t)( id >> 3 );
        CANout(board, message1Reg.idReg[1], id1);
        CANout(board, message1Reg.idReg[0], id0);
    }

    tx2reg &= 0x0f; //restore length only
    for ( i=0; i < tx2reg; i++ ) {
        CANout(board, message1Reg.dataReg[i],
               (TxFifo->data[TxFifo->tail]).data[i]);
    }

    if ( ((TxFifo->data[TxFifo->tail]).flags  & MSG_RTR) == MSG_RTR ) {
        CANout(board, message1Reg.msgCtrl1Reg,
               (RMPD_RES | TXRQ_RES | CPUU_RES | NEWD_UNC));
    }
    else {
        CANout(board, message1Reg.msgCtrl1Reg,
               (RMPD_RES | TXRQ_SET | CPUU_RES | NEWD_UNC));

    }

    TxFifo->free[TxFifo->tail] = BUF_EMPTY; /* now this entry is EMPTY */
    TxFifo->tail = ++(TxFifo->tail) % MAX_BUFSIZE;

    // HM: We had some cases of repeated IRQs, so make sure the INT is acknowledged
    // I know it's already further up, but doing again fixed the issue
    CANout(board, message1Reg.msgCtrl0Reg,
           (MVAL_UNC | TXIE_UNC | RXIE_UNC | INTPD_RES));
    /* leave critical section */
    restore_flags(flags);

    DBGout();
}


/*
 * The plain i82527 interrupt
 *
 *				RX ISR           TX ISR
 *                              8/0 byte
 *                               _____            _   ___
 *                         _____|     |____   ___| |_|   |__
 *---------------------------------------------------------------------------
 * 1) CPUPentium 75 - 200
 *  75 MHz, 149.91 bogomips
 *  AT-CAN-MINI                 42/27µs            50 µs
 *  CPC-PCI		        35/26µs		
 * ---------------------------------------------------------------------------
 * 2) AMD Athlon(tm) Processor 1M
 *    2011.95 bogomips
 *  AT-CAN-MINI  std            24/12µs            ?? µs
 *               ext id           /14µs
 *
 *
 * 1) 1Byte = (42-27)/8 = 1.87 µs
 * 2) 1Byte = (24-12)/8 = 1.5  µs
 *
 *
 *
 * RX Int with to Receive channels:
 * 1)                _____   ___
 *             _____|     |_|   |__
 *                   30    5  20  µs
 *   first ISR normal length,
 *   time between the two ISR -- short
 *   sec. ISR shorter than first, why? it's the same message
 */
#if LINUX_VERSION_CODE >= 0x020500
irqreturn_t CAN_Interrupt ( int irq, void *dev_id, struct pt_regs *ptregs )
#else
void CAN_Interrupt ( int irq, void *dev_id, struct pt_regs *ptregs )
#endif
{
    int board = *(int *) dev_id;
    msg_fifo_t *RxFifo = &Rx_Buf[board];
    msg_fifo_t *TxFifo = &Tx_Buf[board];
    uint8_t irqreg;
    uint8_t lastIrqreg;
     
    // Read the highest pending interrupt request
    irqreg = CANin(board, interruptReg);
    lastIrqreg = irqreg;
    
    while ( irqreg ) {
        switch (irqreg)
        {
            case 1: // Status register
            {
                uint8_t status;

                // Read the STATUS reg
                status = CANin(board, statusReg);
                CANout (board, statusReg, 0);

                if ( status & iSTAT_RXOK ) {
                    // Intel datasheet: Software must clear this bit in ISR
                    CANout (board, statusReg, status & ~iSTAT_RXOK);
                }
                if ( status & iSTAT_TXOK ) {
                    // Intel datasheet: Software must clear this bit in ISR
                    CANout (board, statusReg, status & ~iSTAT_TXOK);
                }
                if ( status & iSTAT_WARN ) {
                    // Note: status bit is read-only, don't clear
                    // TODO must be implemented here for chip statistic
                    (RxFifo->data[RxFifo->head]).flags += MSG_PASSIVE;
                    DBGprint(DBG_DATA,("i82527_CAN_Interrupt: Bus warning\n" ));
                }
                if ( status & iSTAT_BOFF ) {
                    long flags;

                    // Note: status bit is read-only, don't clear
                    // TODO must be implemented here for chip statistic
                    (RxFifo->data[RxFifo->head]).flags += MSG_BUSOFF;

                    // Clear init flag and reenable interrupts
                    flags = CANin(board, controlReg) | ( iCTL_IE | iCTL_EIE );
                    flags &= ~iCTL_INI; // Reset init flag
                    CANout(board, controlReg, flags);

                    (RxFifo->data[RxFifo->head]).id = 0xFFFFFFFF;
                    RxFifo->status = BUF_OK;
                    RxFifo->head = ++(RxFifo->head) % MAX_BUFSIZE;
                    if(RxFifo->head == RxFifo->tail) {
                        RxFifo->status = BUF_OVERRUN;
                    }
                    // Notify user app
                    wake_up_interruptible(  &CanWait[board] );
                    DBGprint(DBG_DATA,("i82527_CAN_Interrupt: Bus off\n" ));
                }
             }
             break;
             case 2: // Receiption, message object 15
                 i82527_irq_read_msg15_handler(board, RxFifo);
             break;
             case 3: // Write, message object 1
                i82527_irq_write_handler(board, TxFifo);
             break;
             case 4: // message object 2
                 //printk("*********** Unexpected i82527_CAN_Interrupt: irqreq2=0x%X\n", irqreg);
                 DBGprint(DBG_DATA,("Unexpected i82527_CAN_Interrupt: irqreq=0x%X\n", irqreg));
                 CANout(board, message2Reg.msgCtrl0Reg, 
                        (MVAL_RES | TXIE_RES | RXIE_RES | INTPD_RES));
             break;
             default: // Unexpected
                 //printk("*********** Unexpected i82527_CAN_Interrupt: irqreq2=0x%X\n", irqreg);
                 DBGprint(DBG_DATA,("Unexpected i82527_CAN_Interrupt: irqreq=0x%X\n", irqreg));
             break;
        }
        // Get irq status again for next loop iteration
        irqreg = CANin(board, interruptReg);
        if (irqreg == lastIrqreg)
        {
           //printk("i82527_CAN_Interrupt: irqreq repeated!!!! 0x%X\n", irqreg);
           DBGprint(DBG_DATA,("i82527_CAN_Interrupt: irqreq repeated!!!! 0x%X\n", irqreg));
        }
        lastIrqreg =     irqreg;
    } /* end while (irqreq) */
    DBGout();
#if LINUX_VERSION_CODE >= 0x020500
    return IRQ_RETVAL(IRQ_HANDLED);
#endif
}


int CAN_ShowStat(int board)
{
  // TODO: Implement
  return -1;
}


/*
 * This is currently an attempt to support acceptance code and mask
 * for i82527. However the interpretation of mask and code is currently
 * different to the SJA1000 function.
 *
 * This MUST change in the future to have a common exchangable API for
 * both chips.
 *
 * Currently if code and mask are both 0 OR both 0xFFFFFFFF the
 * acceptance filtering is disabled.
 *
 * If the highest bit of mask or code is set OR the value is bigger than
 * 0x7FF (11 bit), an extended mask is assumed.
 */
int CAN_SetMask(int board, unsigned int code, unsigned int mask)
{
    unsigned char mask0, mask1, mask2, mask3;
    unsigned char code0, code1, code2, code3;

    DBGin("CAN_SetMask");

    // 0xfffff is a magic value and means no mask set.
    // We have to change this to 0 to make this work with i82527, here 0
    // means don't care.
    if (code == 0xffffffff)
        code = 0;
    if (mask == 0xffffffff)
        mask = 0;

    // Extended (29-bit) or basic (11-bit) mask?
    if (((code & 0x8000000) == 0x8000000) ||
        ((mask & 0x8000000) == 0x8000000) ||
        (code > 0x7FF) ||
        (mask > 0x7FF))
    {
        mask0 = (unsigned char) (mask >> 21);
        mask1 = (unsigned char) (mask >> 13);
        mask2 = (unsigned char) (mask >> 5);
        mask3 = (unsigned char) (mask << 3);
        code0 = (unsigned char) (code >> 21);
        code1 = (unsigned char) (code >> 13);
        code2 = (unsigned char) (code >> 5);
        code3 = (unsigned char) (code << 3);

        CANout(board, globalMaskExtendedReg[0], mask0);
        CANout(board, globalMaskExtendedReg[1], mask1);
        CANout(board, globalMaskExtendedReg[2], mask2);
        CANout(board, globalMaskExtendedReg[3], mask3);
        CANout(board, message15Reg.idReg[0], code0);
        CANout(board, message15Reg.idReg[1], code1);
        CANout(board, message15Reg.idReg[2], code2);
        CANout(board, message15Reg.idReg[3], code3);
        CANout(board, message15Reg.messageConfigReg, MCFG_XTD);

        DBGprint(DBG_DATA,("[%d] CAN_EGMSK 0x%x ==> CAN_EGMSK0 0x%x "
                           "CAN_EGMSK1 0x%x CAN_EGMSK2 0x%x CAN_EGMSK3 0x%x" ,
                           board, mask, mask0, mask1,mask2, mask3));
        DBGprint(DBG_DATA,("[%d] CAN_EGMSK 0x%x ==> CAN_MSGID0 0x%x "
                           "CAN_MSGID1 0x%x CAN_MSGID2 0x%x CAN_MSGID3 0x%x" ,
                           board, code, code0, code1,code2, code3));
    }
    else
    {
        mask0 = (unsigned char) (mask >> 3);
        mask1 = (unsigned char) (mask << 5);
        code0 = (unsigned char) (code >> 3);
        code1 = (unsigned char) (code << 5);

        CANout(board, globalMaskStandardReg[0], mask0);	
        CANout(board, globalMaskStandardReg[1], mask1);
        CANout(board, message15Reg.idReg[0], code0);
        CANout(board, message15Reg.idReg[1], code1);
        CANout(board, message15Reg.messageConfigReg, 0x00);

        DBGprint(DBG_DATA,("[%d] CAN_SGMSK0 0x%x CAN_SGMSK1 0x%x",
                           board, mask0, mask1));
        DBGprint(DBG_DATA,("[%d] CAN_MSGID0 0x%x CAN_MSGID1 0x%x",
                           board, code0, code1));
    }

    /* put values back in global variables for sysctl */
    AccCode[minor] = code;
    AccMask[minor] = mask;

    DBGout();
    return 0;
}


int CAN_SetOMode(int board,int arg)
{
  // Does not exist for i82527
  return -1;
}

int CAN_SetListenOnlyMode(int board,int arg)
{
  // Does not exist for i82527
  return -1;
}

/*
 * Read back as many status information as possible
 *
 * Defined only to be API compatible with SJA1000 code.
 * Refer to SJA1000 code documentation.
 */
int can_GetStat(struct inode *inode, CanStatusPar_t *stat)
{
  // TODO: Implement
    return -1;
}


/*
 * Vendor/hardware specific initilisation
 *
 * Note: release region is done by can_close !
 */
int CAN_VendorInit(int board)
{
    DBGin("i82527_CAN_VendorInit");

    if ((sizeof(canmessage_t) != 15) || (sizeof(canregs_t) != 256))
    {
        DBGprint(DBG_DATA,("Wrong sizes: %d %d ",
                           sizeof(canmessage_t), sizeof(canregs_t)));
        return -EBUSY;
    }

/* Defaults valid for most vendors/configurations ------------------------- */

#ifdef CAN_INDEXED_PORT_IO
    can_range[board] = 2; /* Note: This value is hard coded in can_close.c! */
#else
    can_range[board] = 0x100; /* i82527 has 256 registers */
#endif

/* 1. Vendor specific part ------------------------------------------------ */

#if defined(PC104_200)
    can_range[board] = 0x08; /* pc104/200 board */
#elif defined(SBS_PC7)
    outb(5, 0x169);          /* Unlock special function register */
#else
#endif

/* End: 1. Vendor specific part ------------------------------------------- */


#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,3,11)

#if !defined(CAN4LINUX_PCI)
    /* Request the controllers address space */
#if defined(CAN_PORT_IO)
    /* It's port I/O */
    if(NULL == request_region(Base[board], can_range[board], "CAN-IO")) {
	return -EBUSY;
    }
#else
#if defined(CAN_INDEXED_PORT_IO)
    /* It's indexed port I/O */
    if(NULL == request_region(Base[board], can_range[board], "CAN-IO")) {
      return -EBUSY;
    }
#else
    /* It's Memory I/O */
    if(NULL == request_mem_region(Base[board], can_range[board], "CAN-IO")) {
	return -EBUSY;
    }
#endif
#endif
#endif 	/* !defined(CAN4LINUX_PCI) */

#if defined(CAN4LINUX_PCI)
	/* PCI scan has already remapped the address */
	can_base[board] = (unsigned char *)Base[board];
#else
	can_base[board] = ioremap(Base[board], can_range[board]);
#endif
#else 	  /*  LINUX_VERSION_CODE >= KERNEL_VERSION(2,3,11) */

/* both drivers use high memory area */
#if !defined(CONFIG_PPC) && !defined(CAN4LINUX_PCI)
    if( check_region(Base[board], CAN_RANGE ) ) {
		     DBGout();
		     return -EBUSY;
    } else {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,0,0)
          request_region(Base[board], can_range[board], "CAN-IO" );
#else
          request_region(Base[board], can_range[board] );
#endif  /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,0,0) */
    }
#endif /* !defined  ... */

    /* we don't need ioremap in older Kernels */
    can_base[board] = Base[board];
#endif  /*  LINUX_VERSION_CODE >= KERNEL_VERSION(2,3,11) */

    /* now the virtual address can be used for the register address macros */


/* 2. Vendor specific part ------------------------------------------------ */

#if defined (PC104_200)
    if( IRQ[board] > 0 ) {
      CANactivate_irq(board, IRQ[board]);
      int i;
      for(i=0;i<500;i++) SLOW_DOWN_IO;
    }
#endif

/* End: 2. Vendor specific part ------------------------------------------- */

    if( IRQ[board] > 0 ) {
        if( Can_RequestIrq( board, IRQ[board] , CAN_Interrupt) ) {
	     printk("Can[%d]: Can't request IRQ %d \n", board, IRQ[board]);
	     DBGout(); return -EBUSY;
        }
    }

    DBGout();
    return 0;
}

