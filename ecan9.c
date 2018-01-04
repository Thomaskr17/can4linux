/* EMS Wünsche ECAN9 (TQM28 module, with Pengutronix kernel)
 * featuring the FlexCAN CAN module
 * 
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include "defs.h"
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <mach/hardware.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>

#include <linux/of.h>
#include <linux/pinctrl/consumer.h>

/* three ressouces needed, allocated it at load time
   free it at remove time */
static struct clk *clk[MAX_CHANNELS];
static struct regulator *regulator[MAX_CHANNELS];
static struct flexcan_platform_data *plat_data[MAX_CHANNELS];


# define DRV_NAME               "FlexCAN"

#if defined(CAN_IS_USING_BUS_CLOCK)
#error "For some reasons clocking CAN from the Bus PLL is not supported"
#endif

/*
* The probe() function checks for all defined CAN controllers if they are
* available, switches the core regulator and clock source on 
*
*/
static int /* __devinit */ flexcan_probe(struct platform_device *pdev)
{
int minor = -1;			/* only for DBGin();DBGout(); */
int err = 0;
int devid;

struct clk *clk_ipg = NULL, *clk_per = NULL;
struct pinctrl *pinctrl;
u32 clock_freq = 0;

    DBGin();
    devid = pdev->id;		/* id of used CAN, starts 0 .... */
    pr_info("Probe: Modul %d\n", devid);
    if (devid > (MAX_CHANNELS - 1) ) {
    	pr_err("\tFlexCAN module > %d not supported yet\n", devid - 1);
    	return -ENODEV;    /* test only FlexCAN.1 */
    }

    plat_data[devid] = (pdev->dev).platform_data;


    /* before accessing the 24 MHz clock
	Bit OSC_AUDIO_DOWN in the PMCR2 register
	must be set to 0 to enable the external clock.
	crm_regs.h
	   MXC_CCM_PMCR2
	   MXC_CCM_PMCR2_OSC_AUDIO_DOWN

	this normally should be done in 
	by calling
	    clk_enable(&ckie_clk);
    */

    pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
    if (IS_ERR(pinctrl))
	    return PTR_ERR(pinctrl);

    if (pdev->dev.of_node) {
	    const __be32 *clock_freq_p;

	    clock_freq_p = of_get_property(pdev->dev.of_node,
					    "clock-frequency", NULL);
	    if (clock_freq_p)
		    clock_freq = be32_to_cpup(clock_freq_p);
    }

    /* Clock frequency got from the device tree information */
    pr_info(" clock_freq %u\n", clock_freq);

    if (!clock_freq) {
	    clk_ipg = clk_get(&pdev->dev, "ipg");
	    if (IS_ERR(clk_ipg)) {
		    dev_err(&pdev->dev, "no ipg clock defined\n");
		    err = PTR_ERR(clk_ipg);
		    goto clock_failed;
	    }
	    clock_freq = clk_get_rate(clk_ipg);
	    pr_info(" clock_freq ipg %u\n", clock_freq);
	    clk[devid] = clk_ipg;

	    clk_per = clk_get(&pdev->dev, "per");
	    if (IS_ERR(clk_per)) {
		    dev_err(&pdev->dev, "no per clock defined\n");
		    err = PTR_ERR(clk_per);
		    goto clock_failed;
	    }
	    pr_info(" clock_freq per %lu\n", clk_get_rate(clk_per));
    }

    {
	/* only informative, values ar set static in core.c */
	int irq;
	irq = platform_get_irq(pdev, 0);
	pr_info("IRQ = %d\n", irq);
    }

    /* enable both clocks */
    clk_prepare_enable(clk_ipg);
    clk_prepare_enable(clk_per);

    /* put value into /proc/sys/dev/Can */
    Clock = clock_freq; 

    /* Check if the correct static bit timing table is used */
    if ((Clock) != CAN_SYSCLK) {
	pr_err("\tPlease check the Bit Rate table defined in imx35flexcan.h\n");
	pr_err("\tCurrently set to %d\n", CAN_SYSCLK);
        err = -1;
	goto clock_failed;
    }

    DBGout(); return err;

clock_failed:
	if (clk_per)
		clk_put(clk_per);
	if (clk_ipg)
		clk_put(clk_ipg);

    DBGout(); return err;
}

static int __devexit flexcan_remove(struct platform_device *pdev)
{
int devid = pdev->id;

    if (devid > (MAX_CHANNELS - 1)) return 0;
    pr_info("FlexCAN Remove Module %d\n", devid);
    clk_put(clk[devid]);
    regulator_put(regulator[devid]);
    /* free used io pins */
    /* plat_data[devid]->inactive(pdev);     */
    return 0;
}


static struct of_device_id flexcan_of_match[] = {
	{
		.compatible = "fsl,p1010-flexcan",
	},
	{},
};

static struct platform_driver flexcan_driver = {
       .driver = {
	   .name  = DRV_NAME,
	   .owner = THIS_MODULE,
	   .of_match_table = flexcan_of_match,
       },
       .probe = flexcan_probe,
       .remove = __devexit_p(flexcan_remove),
};


void exit_imx35_hw(void)
{
    platform_driver_unregister(&flexcan_driver);
}

/* Called from __init,  once when driver is loaded
   set up physical adresses, irq number
   and initalize clock source for the CAN module

   take care it will be called only once
*/
void init_imx35_hw(int n)
{
static int already_called = 0;
int ret = 0;

    if(!already_called) {
	pr_info("register \"%s\" can4linux driver\n", DRV_NAME);
	ret = platform_driver_register(&flexcan_driver);
	already_called = 1;
    }

#if CONFIG_TIME_MEASURE
    init_measure();
#endif
}

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

   /* We don't ned the interrupt to be shared for now */

void board_clear_interrupts(int minor)
{
    ;
}



/*
 * Perform Vendor, that means sometimes CAN controller
 * or only board manufacturer specific initialization.
 *
 * Mainly it gets needed IO and IRQ ressources and initilaizes 
 * special hardware functions.
 *
 * This code should normally be in the CAN_VendorInit() function
 * in a TARGET specific file  target.c
 *
 * special code for initializing the IRQ input Pin is done already
 * in init_mmc_hw() 
 * if the module is loaded.
*/

int CAN_VendorInit (int minor)
{
/* struct resource *res; */
/* struct platform_device *pdev; */

    DBGin();
    can_range[minor] = CAN_RANGE;

    /* For the moment get the ressources out of the sources

    ./arch/arm/mach-mx35/mx35_3stack.c
    ./arch/arm/mach-mx35/devices.c
    ./arch/arm/plat-mxc/include/mach/mx35.h

	 .start = CAN1_BASE_ADDR,
	 .end = CAN1_BASE_ADDR + 0x97F,
	 .flags = IORESOURCE_MEM,},
	 .start = MXC_INT_CAN1,
	 .end = MXC_INT_CAN1,
	 .flags = IORESOURCE_IRQ,}


	 should use like this:
	 	platform_get_resource(pdev, IORESOURCE_MEM, 0);
	including
	#include <mach/hardware.h>
	seems to be enough

     */

    /* Request the controllers address space */
    if(NULL == request_mem_region(Base[minor], can_range[minor], "CAN-IO")) {
	DBGprint(DBG_DATA,("Request_mem_region CAN-IO failed at %lx\n",
		Base[minor]));
	return -EBUSY;
    }

    can_base[minor] = ioremap(Base[minor], can_range[minor]);
    /* pr_info(" 0x%08lx remapped to 0x%08lx\n", */
    /* Base[minor], (long unsigned int)can_base[minor]); */

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

       If the kernel receives an interrupt he knows
       how often it was registered,
       and calls the ISR for each registered interrupt.
       In this case, can4linux, for example
       every time with a different pointer to Can_minors[]
       containing the device minor which has registered.
       The ISR should now check this CAN controller related to the minor
       number, if it was the cause of the interrupt.

    */

    /* Each CAN on the i.MX35 has it's own Interrupt
       no need to share Interrupts between modules. */

    /* test for valid IRQ number in /proc/.../IRQ */
    if( IRQ[minor] > 0 && IRQ[minor] < MAX_IRQNUMBER ){
        int err;
	err = request_irq(IRQ[minor], CAN_Interrupt,
		IRQF_SAMPLE_RANDOM,
		"can4linux", &Can_minors[minor]);

	if( !err ){
	    DBGprint(DBG_BRANCH,("Requested IRQ: %d @ %p",
				IRQ[minor], CAN_Interrupt));

	    IRQ_requested[minor] = 1;
	    /* we are her if all is OK */
	    DBGout(); return 0;
	}
	pr_err("Can[%d]: Can't request IRQ %d \n", minor, IRQ[minor]);
    } else {
	;
	DBGprint(DBG_DATA,("Can[%d]: invalid IRQ number %d\n",
						minor, IRQ[minor]));
    }

    /* if something fails, we are here */
    /* release I/O memory mapping -> release virtual memory */
    /* pr_info("iounmap %p \n", can_base[minor]); */
    iounmap(can_base[minor]);

    /* Release the memory region */
    /* pr_info("release mem %x \n", Base[minor]); */
    release_mem_region(Base[minor], can_range[minor]);
    DBGout(); return -EBUSY;
}


int Can_FreeIrq(int minor, int irq )
{
    DBGin();
    IRQ_requested[minor] = 0;
    free_irq(irq, &Can_minors[minor]);
    DBGout();
    return 0;
}


#if CONFIG_TIME_MEASURE
/* Functions to Switch the stat of output pins
   used for time measurement within the CAN ISR
 */
#include <mach/gpio.h>
#include <linux/../../arch/arm/mach-mx35/iomux.h>
#include <linux/../../arch/arm/mach-mx35/mx35_pins.h>


/*
  gpio_* functions are defined in ./drivers/gpio/gpiolib.c

  after setting the output direction,
  void gpio_set_value(unsigned gpio, int value)


  The GPIO pins which are controlled
  are located on the MLB connector (P3) pin 2,4,6

  Before toggling the GPIO the following steps have to be performed.
  - Connect the MLB connector pin 20 to MLB connector pin 15

*/

void init_measure(void)
{
    /* set port to output and initially to low */
    /* set mux */
    mxc_request_iomux(MX35_PIN_MLB_CLK, MUX_CONFIG_GPIO);
    mxc_request_iomux(MX35_PIN_MLB_DAT, MUX_CONFIG_GPIO);
    mxc_request_iomux(MX35_PIN_MLB_SIG, MUX_CONFIG_GPIO);
    mxc_free_iomux(MX35_PIN_MLB_CLK, MUX_CONFIG_GPIO);
    mxc_free_iomux(MX35_PIN_MLB_DAT, MUX_CONFIG_GPIO);
    mxc_free_iomux(MX35_PIN_MLB_SIG, MUX_CONFIG_GPIO);

    /* set gpio */
    gpio_direction_output(67, 1);
    gpio_direction_output(68, 1);
    gpio_direction_output(69, 1);

}

void set_measure_pin(void) 
{
    /* set port to output and initially to high */
    gpio_set_value(67, 1);
}

void reset_measure_pin(void) 
{
    /* set port to output and initially to high */
    gpio_set_value(67, 0);
}
#endif
