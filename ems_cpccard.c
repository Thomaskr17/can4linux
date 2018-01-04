#ifdef CAN4LINUX_PCCARD
# ifndef CONFIG_CARDBUS
#  error "trying to compile a PC-Card driver for kernel without CONFIG_CARDBUS"
# endif

# include "../cdkl-1.10/cpc-card/cpccard.h"

/* Room for two CPC-Card PC Card adress information
 * 
 * not very flexible, but most computers only have two slots 
 */
CPC_CARD_T * CPC_CARD_Table[2];

/*
write_byte macro
parameters: (b)ase, (r)egister, (v)alue
*/
#define write_byte(b,r,v)  writeb((v), ((b) + (r)));
/*
read_byte macro
parameters: (b)ase, (r)egister
*/
#define read_byte(b,r)     (readb((b) + (r)))

/* CPCCardRegister is called from cpc-card_cs, if CPCCard is inserted */
/*
*  Works at the moment only for the first inserted Card /dev/can0 and can1
*/
int CPCCardRegister(unsigned long base, unsigned int irq)
{ 
CPC_CARD_T * card = &CPC_CARD_Table[0];
unsigned int timeout;

    DBGin("CPCCardRegister");

    card->base = (unsigned long)ioremap(base, 0x200);
    /* See if we can read the predefined registers at Base */
    printk("0: %x\n", read_byte(card->base, LOGIC_BASE) );	/* 0x55 */
    printk("1: %x\n", read_byte(card->base, LOGIC_BASE + 1) );	/* 0xaa */
    printk("2: %d\n", read_byte(card->base, LOGIC_BASE + 2) );	/* 1 */
    printk("3: %x\n", read_byte(card->base, LOGIC_BASE + 3) );	/* 0xcb */
    printk("4: %d\n", read_byte(card->base, LOGIC_BASE + 4) );	/* 5 */
    printk("6: %d\n", read_byte(card->base, LOGIC_BASE + 6) );	/* 2 */

    /* Very Basic initailazation of two CAN channels  */
    IRQ[0] = IRQ[1] = irq;
    Base[0] = card->base + 0x100;
    Base[1] = card->base + 0x180;
	
    writeb(0, card->base + LOGIC_BASE); /* reset */
    timeout = 0;
    do {
	udelay(1);         
	timeout++;
	if(timeout > 100000) {
	    break; 
	}
    } while(!(read_byte(card->base, LOGIC_BASE) & 0x01));



    printk(KERN_INFO "CPCCard Driver v%s successfully installed!\n",
	    CPC_DRIVER_VERSION);
    printk(KERN_INFO "   using IRQ %d, Base at %Xh\n", irq, card->base);

    DBGout();
    return 0;
}

/******************************************************************************/
/**
* CPCCardUnRegister is called from cpc-card_cs, if CPCCard is removed
*/
void CPCCardUnRegister(void)
{
CPC_CARD_T * card = CPC_CARD_Table[0];

    DBGin("CPCCardUnRegister");
    card->locked = 1;
    DBGout();
}



#endif
/*----------------------------------------------------------------------------*/
