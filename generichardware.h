/* generichardware.h 
 * For each CAN controller hardware depending register definitions
 * As an example look at sja1000.h
 */

#ifndef __CAN_GENERIC_
#define __CAN_GENERIC_


/* Register definitions like these: 
 */

#define CAN_BUS_STATUS 				(1<<7)
#define CAN_ERROR_STATUS			(1<<6)
#define CAN_TRANSMIT_STATUS			(1<<5)
#define CAN_RECEIVE_STATUS			(1<<4)


/*--- Interrupt Register -----------------------------------*/
 
#define CAN_BUS_ERR_INT				(1<<7)
#define CAN_ARBITR_LOST_INT			(1<<6)
#define CAN_ERROR_PASSIVE_INT			(1<<5)
#define CAN_WAKEUP_INT				(1<<4)
#define CAN_OVERRUN_INT				(1<<3)
#define CAN_ERROR_WARN_INT			(1<<2)
#define CAN_TRANSMIT_INT			(1<<1)
#define CAN_RECEIVE_INT 			(1<<0)


/* generated bit rate table by
 * http://www.port.de/engl/canprod/sv_req_form.html
 */
#if CAN_SYSCLK == 5
/* these timings are valid for XXXXX, using 10.0 Mhz*/
#  define CAN_TIM0_10K		0x18
#  define CAN_TIM1_10K		0x2f
#  define CAN_TIM0_20K		0x18	
#  define CAN_TIM1_20K		0x07
#  define CAN_TIM0_40K		   0	/* not supported */
#  define CAN_TIM1_40K		   0
#  define CAN_TIM0_50K		0x09
#  define CAN_TIM1_50K		0x07
#  define CAN_TIM0_100K         0x04 
#  define CAN_TIM1_100K         0x07
#  define CAN_TIM0_125K		0x04
#  define CAN_TIM1_125K		0x05
#  define CAN_TIM0_250K		   0
#  define CAN_TIM1_250K		0x2f
#  define CAN_TIM0_500K		   0
#  define CAN_TIM1_500K		0x07
#  define CAN_TIM0_800K		   0	/* not supported */
#  define CAN_TIM1_800K		0x00
#  define CAN_TIM0_1000K	   0	/* not supported */
#  define CAN_TIM1_1000K	0x00

#define CAN_SYSCLK_is_ok            1

#endif

#if CAN_SYSCLK == 24
/* these timings are valid for XXXXX, using 48.0 Mhz*/



#define CAN_SYSCLK_is_ok            1

#endif




/* for more CAN_SYSCLK values */


#ifndef CAN_SYSCLK_is_ok
#  error Please specify a valid CAN_SYSCLK value (i.e. 8, 10) or define new parameters
#endif



#endif 	/* __CAN_GENERIC__ */
