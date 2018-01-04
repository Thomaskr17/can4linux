/*
 * i82527.h - Header file with definition for i82527
 *
 * Original version Written by Arnaud Westenberg email:arnaud@wanadoo.nl
 * This software is released under the GPL-License.
 *
 * Major Refactoring and Integration into can4linux version 3.1 by
 * Henrik W Maier of FOCUS Software Engineering Pty Ltd <www.focus-sw.com>
 *
 * $Id: i82527.h,v 1.1 2006/04/21 16:26:51 oe Exp $
 *
 */


typedef struct canmessage {
	uint8_t	msgCtrl0Reg;	
	uint8_t	msgCtrl1Reg;	
	uint8_t	idReg[4];
	uint8_t	messageConfigReg;
	uint8_t	dataReg[8];	
} canmessage_t __attribute__ ((packed));


typedef struct canregs {
    union
    {
        struct
        {
            canmessage_t messageReg;
            uint8_t someOtherReg; // padding
        } msgArr[16];
        struct {
            uint8_t      controlReg;               // Control Register
            uint8_t      statusReg;                // Status Register
            uint8_t      cpuInterfaceReg;          // CPU Interface Register
            uint8_t      reserved1Reg;
            uint8_t      highSpeedReadReg[2];      // High Speed Read
            uint8_t      globalMaskStandardReg[2]; // Standard Global Mask byte 0
            uint8_t      globalMaskExtendedReg[4]; // Extended Global Mask bytes
            uint8_t      message15MaskReg[4];      // Message 15 Mask bytes
            canmessage_t message1Reg;
            uint8_t      clkOutReg;                // Clock Out Register
            canmessage_t message2Reg;
            uint8_t      busConfigReg;             // Bus Configuration Register
            canmessage_t message3Reg;
            uint8_t      bitTiming0Reg;            // Bit Timing Register byte 0
            canmessage_t message4Reg;
            uint8_t      bitTiming1Reg;            // Bit Timing Register byte 1
            canmessage_t message5Reg;
            uint8_t      interruptReg;             // Interrupt Register
            canmessage_t message6Reg;
            uint8_t      reserved2Reg;
            canmessage_t message7Reg;
            uint8_t      reserved3Reg;
            canmessage_t message8Reg;
            uint8_t      reserved4Reg;
            canmessage_t message9Reg;
            uint8_t      p1ConfReg;
            canmessage_t message10Reg;
            uint8_t      p2ConfReg;
            canmessage_t message11Reg;
            uint8_t      p1InReg;
            canmessage_t message12Reg;
            uint8_t      p2InReg;
            canmessage_t message13Reg;
            uint8_t      p1OutReg;
            canmessage_t message14Reg;
            uint8_t      p2OutReg;
            canmessage_t message15Reg;
            uint8_t      serialResetAddressReg;
        };
    };
} canregs_t __attribute__ ((packed));


/* Control Register (0x00) */
enum i82527_iCTL {
	iCTL_INI = 1,		// Initialization
	iCTL_IE  = 1<<1,	// Interrupt Enable
	iCTL_SIE = 1<<2,	// Status Interrupt Enable
	iCTL_EIE = 1<<3,	// Error Interrupt Enable
	iCTL_CCE = 1<<6		// Change Configuration Enable
};

/* Status Register (0x01) */
enum i82527_iSTAT {
	iSTAT_TXOK = 1<<3,	// Transmit Message Successfully
	iSTAT_RXOK = 1<<4,	// Receive Message Successfully
	iSTAT_WAKE = 1<<5,	// Wake Up Status
	iSTAT_WARN = 1<<6,	// Warning Status
	iSTAT_BOFF = 1<<7	// Bus Off Status
};

/* CPU Interface Register (0x02) */
enum i82527_iCPU {
	iCPU_CEN = 1,		// Clock Out Enable
	iCPU_MUX = 1<<2,	// Multiplex
	iCPU_SLP = 1<<3,	// Sleep
	iCPU_PWD = 1<<4,	// Power Down Mode
	iCPU_DMC = 1<<5,	// Divide Memory Clock
	iCPU_DSC = 1<<6,	// Divide System Clock
	iCPU_RST = 1<<7,	// Hardware Reset Status
};

/* Clock Out Register (0x1f) */
enum i82527_iCLK {
	iCLK_CD0 = 1,		// Clock Divider bit 0
	iCLK_CD1 = 1<<1,
	iCLK_CD2 = 1<<2,
	iCLK_CD3 = 1<<3,
	iCLK_SL0 = 1<<4,	// Slew Rate bit 0
	iCLK_SL1 = 1<<5
};

/* Bus Configuration Register (0x2f) */
enum i82527_iBUS {
	iBUS_DR0 = 1,		// Disconnect RX0 Input
	iBUS_DR1 = 1<<1,	// Disconnect RX1 Input
	iBUS_DT1 = 1<<3,	// Disconnect TX1 Output
	iBUS_POL = 1<<5,	// Polarity
	iBUS_CBY = 1<<6		// Comparator Bypass
};

#define RESET 1			// Bit Pair Reset Status
#define SET 2			// Bit Pair Set Status
#define UNCHANGED 3		// Bit Pair Unchanged

/* Message Control Register 0 (Base Address + 0x0) */
enum i82527_iMSGCTL0 {
	INTPD_SET = SET,		// Interrupt pending
	INTPD_RES = RESET,		// No Interrupt pending
	INTPD_UNC = UNCHANGED,
	RXIE_SET  = SET<<2,		// Receive Interrupt Enable
	RXIE_RES  = RESET<<2,		// Receive Interrupt Disable
	RXIE_UNC  = UNCHANGED<<2,
	TXIE_SET  = SET<<4,		// Transmit Interrupt Enable
	TXIE_RES  = RESET<<4,		// Transmit Interrupt Disable
	TXIE_UNC  = UNCHANGED<<4,
	MVAL_SET  = SET<<6,		// Message Valid
	MVAL_RES  = RESET<<6,		// Message Invalid
	MVAL_UNC  = UNCHANGED<<6
};

/* Message Control Register 1 (Base Address + 0x01) */
enum i82527_iMSGCTL1 {
	NEWD_SET = SET,			// New Data
	NEWD_RES = RESET,		// No New Data
	NEWD_UNC = UNCHANGED,
	MLST_SET = SET<<2,		// Message Lost
	MLST_RES = RESET<<2,		// No Message Lost
	MLST_UNC = UNCHANGED<<2,
	CPUU_SET = SET<<2,		// CPU Updating
	CPUU_RES = RESET<<2,		// No CPU Updating
	CPUU_UNC = UNCHANGED<<2,
	TXRQ_SET = SET<<4,		// Transmission Request
	TXRQ_RES = RESET<<4,		// No Transmission Request
	TXRQ_UNC = UNCHANGED<<4,
	RMPD_SET = SET<<6,		// Remote Request Pending
	RMPD_RES = RESET<<6,		// No Remote Request Pending
	RMPD_UNC = UNCHANGED<<6
};

/* Message Configuration Register (Base Address + 0x06) */
enum i82527_iMSGCFG {
	MCFG_XTD = 1<<2,		// Extended Identifier
	MCFG_DIR = 1<<3			// Direction is Transmit
};

#if CAN_SYSCLK == 8
/* The timings are valid for clock 8Mhz */
#define CAN_TIM0_10K		  49
#define CAN_TIM1_10K		0x1c
#define CAN_TIM0_20K		  24	
#define CAN_TIM1_20K		0x1c
#define CAN_TIM0_40K		0x89	/* Old Bit Timing Standard of port */
#define CAN_TIM1_40K		0xEB	/* Old Bit Timing Standard of port */
#define CAN_TIM0_50K		   9
#define CAN_TIM1_50K		0x1c
#define CAN_TIM0_100K              4    /* sp 87%, 16 abtastungen, sjw 1 */
#define CAN_TIM1_100K           0x1c
#define CAN_TIM0_125K		   3
#define CAN_TIM1_125K		0x1c
#define CAN_TIM0_250K		   1
#define CAN_TIM1_250K		0x1c
#define CAN_TIM0_500K		   0
#define CAN_TIM1_500K		0x1c
#define CAN_TIM0_800K		   0
#define CAN_TIM1_800K		0x16
#define CAN_TIM0_1000K		   0
#define CAN_TIM1_1000K		0x14

#define CAN_SYSCLK_is_ok            1
#endif


#ifndef CAN_SYSCLK_is_ok
#  error Please specify a valid CAN_SYSCLK value (i.e. 8, 10) or define new parameters
#endif

