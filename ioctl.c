/*
 * ioctl - can4linux CAN driver module
 *
 * can4linux -- LINUX CAN device driver source
 * 
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 * 
 * Copyright (c) 2001-2011 port GmbH Halle/Saale
 * (c) 2011 Heinz-Jürgen Oertel (oe@port.de)
 *          Claus Schroeter (clausi@chemie.fu-berlin.de)
 *------------------------------------------------------------------
 * $Header: /z2/cvsroot/products/0530/software/can4linux/src/ioctl.c,v 1.8 2009/06/03 14:34:11 oe Exp $
 *
 *--------------------------------------------------------------------------
 *
 *
 *
 *
 *--------------------------------------------------------------------------
 */


/**
* \file ioctl.c
* \author Heinz-Jürgen Oertel, port GmbH
* $Revision: 1.8 $
* $Date: 2009/06/03 14:34:11 $
*
*
*/

#include "defs.h"

int can_Command(struct inode *inode, struct file *file, Command_par_t * argp);
int can_Send(struct inode *inode, canmsg_t __user *Tx);
int can_GetStat(struct inode *inode, struct file *file, CanStatusPar_t *s);
int can_GetStatVirt(struct inode *inode, struct file *file, CanStatusPar_t *s);
int can_Config(struct inode *inode, struct file *file, int target,
		unsigned long val1, unsigned long val2);

/***************************************************************************/
/**
*
\brief int ioctl(int fd, int request, ...);
the CAN controllers io-control interface
\param fd The descriptor to change properties
\param request special configuration request
\param ...  traditional a \a char *argp

The \a ioctl function manipulates the underlying device
parameters of the CAN special device.
In particular, many operating characteristics of
character CAN driver may be controlled with \a ioctl requests.
The argument \a fd must be an open file descriptor.

An ioctl request has encoded in it whether the argument is
an \b in parameter or \b out parameter,
and the size of the argument argp in bytes.
Macros and defines used in specifying an \a ioctl request
are located  in  the  file can4linux.h .

The following \a requests are defined:

\li \c CAN_IOCTL_COMMAND some commands for
start, stop and reset the CAN controller chip
\li \c CAN_IOCTL_CONFIG configure some of the device properties
like acceptance filtering, bit timings, mode of the output control register
or the optional software message filter configuration(not implemented yet).
\li \c CAN_IOCTL_STATUS request the CAN controllers status
\li \c CAN_IOCTL_SEND a single message over the \a ioctl interface 
\li \c CAN_IOCTL_RECEIVE poll a receive message
\li \c CAN_IOCTL_CONFIGURERTR configure automatic RTR responses(not implemented)

The third argument is a parameter structure depending on the request.
These are
\code
struct Command_par
struct Config_par
struct CanStatusPar
struct ConfigureRTR_par
struct Receive_par
struct Send_par
\endcode
described in can4linux.h

The following commands are available
\li \c CMD_START calls the target specific CAN_StartChip function. 
	This normally clears all pending interrupts, enables interrupts
	and starts the CAN controller by releasing the RESET bit.
\li \c CMD_STOP  calls the target specific CAN_StopChip function. 
	This sets only the RESET bit of the CAN controller
	which will stop working.
\li \c CMD_RESET calls the target specific CAN_ChipReset function. 
	This command also sets the RESET bit of the CAN controller,
	but additionally initializes CAN bit timing 
	the output control register and acceptance and mask registers.
	The CAN controller itself stays in the RESET mode until 
	CMD_START is called.
\li \c CMD_CLEARBUFFERS clears/empties both
	the RX fifo of the associated process
	and the one and only global TX fifo.
\li \c CMD_CTRL_LED control on board LEDs
	The driver defines different LEDs
	red, gree, yellow are standard, but may be more
	and each of the LEDS can have a state \e on or \e off.

The normal way of reinitializing  CAN
is the following ioctl()-command sequence:
\li \c CMD_STOP
\li \c CMD_CLEARBUFFERS
\li \c CMD_RESET
\li \c CMD_START

If the driver is used by more than one application,
one should take care that this functionality (like some others)
can not be called by any application.
Stopping the shared CAN will stop it for all other processes as well.
In \e can4linux 
the first process opening a device like /dev/canX gets some more privileges
marked in the private structure \e .su as TRUE.
	
\par Bit Timing
The bit timing can be set using the \a ioctl(CONFIG,.. )
and the targets CONF_TIMING or CONF_BTR.
CONFIG_TIMING should be used only for the predefined Bit Rates
(given in kbit/s).
With CONF_BTR it is possible to set the CAN controllers bit timing registers
individually by providing the values in \b val1 (BTR0)
and \b val2 (BTR1).


\par Acceptance Filtering

\b Basic \b CAN.
In the case of using base format identifiers in Basic CAN mode
for receiving CAN messages
only the low bytes are used to set acceptance code and mask
for bits ID.10 ... ID.3

\par
\b PeliCAN.
For acceptance filtering the entries \c AccCode and \c AccMask are used
like specified in the controllers manual for
\b Single \b Filter \b Configuration .
Both are 4 byte entries.
In the case of using base format identifiers for receiving CAN messages
also all 4 bytes can be used.
In this case two bytes are used for acceptance code and mask
for all 11 identifier bits plus additional the first two data bytes.
The SJA1000 is working in the \b Single \b Filter \ Mode .

Example for extended message format
\code
       Bits
 mask  31 30 .....           4  3  2  1  0
 code
 -------------------------------------------
 ID    28 27 .....           1  0  R  +--+-> unused
                                   T
                                   R

  acccode =  (id << 3) + (rtr << 2) 
\endcode

Example for base message format
\code
       Bits
 mask  31 30 .....           23 22 21 20 ... 0
 code
 -------------------------------------------
 ID    11 10 .....           1  0  R  +--+-> unused
                                   T
                                   R
\endcode

You have to shift the CAN-ID by 5 bits and two bytes to shift them
into ACR0 and ACR1 (acceptance code register)
\code
  acccode =  (id << 21) + (rtr << 20) 
\endcode
In case of the base format match the content of bits 0...20
is of no interest, it can be 0x00000 or 0xFFFFF.
\returns
On success, zero is returned.
On error, -1 is returned, and \e errno is set appropriately.

\par Example
\code
Config_par_t  cfg;
volatile Command_par_t cmd;


    cmd.cmd = CMD_STOP;
    ioctl(can_fd, CAN_IOCTL_COMMAND, &cmd);

    cfg.target = CONF_ACCM; 
    cfg.val    = acc_mask;
    ioctl(can_fd, CAN_IOCTL_CONFIG, &cfg);
    cfg.target = CONF_ACCC; 
    cfg.val    = acc_code;
    ioctl(can_fd, CAN_IOCTL_CONFIG, &cfg);

    cmd.cmd = CMD_START;
    ioctl(can_fd, CAN_IOCTL_COMMAND, &cmd);

\endcode

\par Setting the bit timing register

can4linux provides direct access to the bit timing registers,
besides an implicit setting using the \e ioctl \c CONF_TIMING
and fixed values in Kbit/s.
In this case ioctl(can_fd, CAN_IOCTL_CONFIG, &cfg);
is used with configuration target \c CONF_BTR
The configuration structure contains two values, \e val1 and \e val2 .
The following relation to the bit timing registers is used regarding
the CAN controller:

\code
                           val1            val2
SJA1000                    BTR0            BTR1
BlackFin                   CAN_CLOCK       CAN_TIMING
FlexCAN	(to implement) 
\endcode

\par
Bit timings are coded in a table in the <hardware>funcs.c file.
The values for the bit timing registers are calculated based on a
fixed CAN controller clock.
This can lead to wrong bit timings if the processor (or CAN)
uses another clock as assumed at compile time.
Please check carefully.
Depending on the clock,
it might be possible that not all bit rates can be generated.
(e.g. th Blackfin only supports 100, 125, 250, 500 and 1000 Kbit/s 
(currently!))


\par Other CAN_IOCTL_CONFIG configuration targets

(see can4linux.h)
\code
CONF_LISTEN_ONLY_MODE   if set switch to listen only mode
                        (default false)
CONF_SELF_RECEPTION     if set place sent messages back in the RX queue
                        (default false)
CONF_BTR		configure bit timing registers directly
CONF_TIMESTAMP          if set fill time stamp value in message structure
                        (default value 1)
			Different values are possible and are selecting the
			time stamp format
			0 - no time stamp (time stamp is zero)
			1 - absolute time as gettimeofday()
			2 - absolute rate monotonic time
			3 - time difference to the last event (received message)
CONF_WAKEUP             if set wake up waiting processes (default true) 
\endcode
*/

__LDDK_IOCTL_TYPE can_ioctl( __LDDK_IOCTL_PARAM )
{
void *argp;
int retval = -EIO;
unsigned long _cnt;
int ret = 0;
int Can_errno = 0;
#if defined(DEBUG) 
unsigned int minor = __LDDK_MINOR;
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35)
  struct inode *inode = file->f_path.dentry->d_inode;
#endif

  DBGin();
  DBGprint(DBG_DATA,("cmd=%d", cmd));
  
  switch(cmd){

        case CAN_IOCTL_COMMAND:
	  if( !access_ok(VERIFY_READ, (void __user *)arg, sizeof(Command_par_t))) {
	     DBGout(); return(retval); 
	  }
	  if( !access_ok(VERIFY_WRITE, (void __user *)arg, sizeof(Command_par_t))) {
	     DBGout(); return(retval); 
	  }
	  argp = (void *) kmalloc( sizeof(Command_par_t) +1 , GFP_KERNEL );
	  __lddk_copy_from_user( (void *)argp, (Command_par_t __user *)arg,
	  					sizeof(Command_par_t));
	  ((Command_par_t *) argp)->retval =
	  		can_Command(inode, file, (Command_par_t *)argp);
	  ((Command_par_t *) argp)->error = Can_errno;
	  __lddk_copy_to_user( (Command_par_t __user *)arg, (void *)argp,
	  					sizeof(Command_par_t));
	  kfree(argp);
	  ret = 0;
	  break;
      case CAN_IOCTL_CONFIG:
	  if( !access_ok(VERIFY_READ, (void __user *)arg, sizeof(Config_par_t))) {
	     DBGout(); return(retval); 
	  }
	  if( !access_ok(VERIFY_WRITE, (void __user *)arg, sizeof(Config_par_t))) {
	     DBGout(); return(retval); 
	  }
	  argp = (void *) kmalloc( sizeof(Config_par_t) +1 ,GFP_KERNEL);
	  __lddk_copy_from_user( (void *)argp, (Config_par_t __user *)arg,
	  					sizeof(Config_par_t));
	  retval = can_Config(inode, file, ((Config_par_t *)argp)->target, 
			     ((Config_par_t *)argp)->val1,
			     ((Config_par_t *)argp)->val2 );
	  ((Config_par_t *) argp)->retval = retval;
	  ((Config_par_t *) argp)->error = Can_errno;
	  __lddk_copy_to_user( (Config_par_t __user *)arg, (void *)argp,
	  					sizeof(Config_par_t));
	  kfree(argp);
	  if (0 != retval) {
	    ret = retval;
	  } else {
	      ret = 0;
	  }
	  break;
      case CAN_IOCTL_SEND:
	  if( !access_ok(VERIFY_READ, (void __user *)arg, sizeof(Send_par_t))) {
	     DBGout(); return(retval); 
	  }
	  if( !access_ok(VERIFY_WRITE, (void __user *)arg, sizeof(Send_par_t))) {
	     DBGout(); return(retval); 
	  }
	  argp = (void *)kmalloc( sizeof(Send_par_t) +1 ,GFP_KERNEL );
	  __lddk_copy_from_user( (void *)argp, (Send_par_t __user *)arg,
	  				sizeof(Send_par_t));
	  if(virtual == 0)  {
	      ((Send_par_t *) argp)->retval =
			    can_Send(inode, 
				    (canmsg_t __user *)((Send_par_t *)argp)->Tx );
	  } else {

	      /* FIXME */


	  }
	  ((Send_par_t *) argp)->error = Can_errno;
	  __lddk_copy_to_user( (Send_par_t __user *)arg, (void *)argp,
	  				sizeof(Send_par_t));
	  kfree(argp);
	  ret = 0;
	  break;
      case CAN_IOCTL_STATUS:
	  if( !access_ok(VERIFY_READ, (void __user *)arg,
	  				sizeof(CanStatusPar_t))) {
	     DBGout(); return(retval); 
	  }
	  if( !access_ok(VERIFY_WRITE, (void __user *)arg,
	  			sizeof(CanStatusPar_t))) {
	     DBGout(); return(retval); 
	  }
	  argp = (void *)kmalloc( sizeof(CanStatusPar_t) +1 ,GFP_KERNEL );
	  
	  if(virtual == 0)  {

	      ((CanStatusPar_t *) argp)->retval =
	  		can_GetStat(inode, file, ((CanStatusPar_t *)argp));
	  } else {

	      ((CanStatusPar_t *) argp)->retval =
	  		can_GetStatVirt(inode, file, ((CanStatusPar_t *)argp));

	  }

	  __lddk_copy_to_user( (CanStatusPar_t __user *)arg, (void *)argp,
	  				sizeof(CanStatusPar_t));
	  kfree(argp);
	  ret  = 0;
	  break;

#ifdef CAN_RTR_CONFIG
      case CAN_IOCTL_CONFIGURERTR:
	  if( !access_ok(VERIFY_READ, (void *)arg,
	  			sizeof(ConfigureRTR_par_t))){
	     DBGout(); return(retval); 
	  }
	  if( !access_ok(VERIFY_WRITE, (void *)arg,
	  			sizeof(ConfigureRTR_par_t))){
	     DBGout(); return(retval); 
	  }
	  argp = (void *)kmalloc( sizeof(ConfigureRTR_par_t) +1 ,GFP_KERNEL );
	  __lddk_copy_from_user( (void *)argp, (ConfigureRTR_par_t *) arg,
	  			sizeof(ConfigureRTR_par_t));
	  ((ConfigureRTR_par_t *) argp)->retval =
	  	can_ConfigureRTR(inode,
	  			((ConfigureRTR_par_t *)argp)->message, 
				((ConfigureRTR_par_t *)argp)->Tx );
	  ((ConfigureRTR_par_t *) argp)->error = Can_errno;
	  __lddk_copy_to_user( (ConfigureRTR_par_t *)arg, (void *)argp,
	  			sizeof(ConfigureRTR_par_t));
	  kfree(argp);
	  ret = 0;
	  break;

#endif  	/* CAN_RTR_CONFIG */
  
      default:
        DBGout();
	ret = -EINVAL;
    }
    DBGout();
    return ret;
}



#ifndef DOXYGEN_SHOULD_SKIP_THIS


/* 
* ioctl functions are following here 
*/


int can_Command(struct inode *inode, struct file *file, Command_par_t * argp)
{
unsigned int minor = iminor(inode);
int cmd;
int rx_fifo = ((struct _instance_data *)(file->private_data))->rx_index;
int su = ((struct _instance_data *)(file->private_data))->su;

    cmd =  argp->cmd;

    DBGprint(DBG_DATA,("%s: cmd=%d", __func__, cmd));
    switch (cmd) {
	case CMD_START:
	    if(virtual != 0)  break;
	    if(su == TRUE) {
		CAN_StartChip(minor);
	    } else {
		return -EPERM;
	    }
	    break;
	case CMD_STOP:
	    if(virtual != 0)  break;
	    if(su == TRUE) {
		CAN_StopChip(minor);
		// printk("do Stop CAN\n");
	    } else {
		// printk("I'm not allowed to Stop CAN\n");
		return -EPERM;
	    }
	    break;
	case CMD_RESET:
	    if(virtual != 0)  break;
	    if(su == TRUE) {
		CAN_ChipReset(minor);
	    } else {
		return -EPERM;
	    }
	    break;
        case CMD_CLEARBUFFERS:
	    {
	    if(su == TRUE) {
		Can_TxFifoInit(minor);
	    }
	    Can_RxFifoInit(minor, rx_fifo);
#if 0
	    if( argp->target = CMD_CLEAR_RX) {
	    } else
	    if( argp->target = CMD_CLEAR_TX) {
	    } else {
		DBGout();
		return(-EINVAL);
	    }
#endif
	    }
	    break;
	case CMD_CTRL_LED:
	    if (virtual != 0)
		    break;
	    /* This is very target specific */
#if defined(CC_CANPCI) || defined(JANZ_PCIL)
	    CAN_control_led(minor, argp);
#endif
	    break;
	case CMD_CTRL_TERM:
	    if (virtual != 0)
		    break;
#if defined(JANZ_PCIL)
	    can_control_termination(minor, argp);
#endif
	    break;
	default:
	    DBGout();
	    return(-EINVAL);
    }
    return 0;
}

/* is not very useful! use it if you are sure the tx queue is empty */
int can_Send(struct inode *inode, canmsg_t __user *Tx)
{
unsigned int minor = iminor(inode);	
canmsg_t tx;
unsigned long _cnt;

    if(virtual != 0)  return 0;

    if( !access_ok(VERIFY_READ, Tx, sizeof(canmsg_t)) ) {
	    return -EINVAL;
    }
    __lddk_copy_from_user((canmsg_t *)&tx, (canmsg_t __user *)Tx, sizeof(canmsg_t));
    return CAN_SendMessage(minor, &tx);
}

int can_Config(
	struct inode *inode,
	struct file *file,
	int target,
	unsigned long val1,
	unsigned long val2
	)
{
unsigned int minor = iminor(inode);
int rx_fifo = ((struct _instance_data *)(file->private_data))->rx_index;
int su = ((struct _instance_data *)(file->private_data))->su;
int ret = -EINVAL;

    DBGin();
    switch(target) {
	case CONF_ACC:		/* set the first code/mask pair */
	    if(virtual != 0)  break;
#if defined(IMX35) || defined(IMX25) || defined(IMX28)
	    ret = CAN_SetMask(minor, 0, val2, val1);
#else
	    ret = CAN_SetMask(minor, val2, val1);		
#endif
	    break;
	case CONF_ACCM:		/* set the first mask only */
	    if(virtual != 0)  break;
#if defined(IMX35) || defined(IMX25) || defined(IMX28)
	    ret = CAN_SetMask(minor, 0, AccCode[minor][0], val1);
#else
	    ret = CAN_SetMask(minor, AccCode[minor], val1);		
#endif
	    break;
	case CONF_ACCC:		/* the first code only */
	    if(virtual != 0)  break;
#if defined(IMX35) || defined(IMX25) || defined(IMX28)
	    ret = CAN_SetMask(minor, 0, val1, AccMask[minor][0]);
#else
	    ret = CAN_SetMask(minor, val1, AccMask[minor]);		
#endif
	    break;

#if defined(IMX35) || defined(IMX25) || defined(IMX28)
	case CONF_ACC1:		/* set the first additional code/mask pair */
	    if(virtual != 0)  break;
	    ret = CAN_SetMask(minor, 1, val2, val1);
	    break;
	case CONF_ACC2:
	    if(virtual != 0)  break;
	    ret = CAN_SetMask(minor, 2, val2, val1);
	    break;
	case CONF_ACC3:
	    if(virtual != 0)  break;
	    ret = CAN_SetMask(minor, 3, val2, val1);
	    break;
	case CONF_ACC4:
	    if(virtual != 0)  break;
	    ret = CAN_SetMask(minor, 4, val2, val1);
	    break;
	case CONF_ACC5:
	    if(virtual != 0)  break;
	    ret = CAN_SetMask(minor, 5, val2, val1);
	    break;
	case CONF_ACC6:
	    if(virtual != 0)  break;
	    ret = CAN_SetMask(minor, 6, val2, val1);
	    break;
	case CONF_ACC7:
	    if(virtual != 0)  break;
	    ret = CAN_SetMask(minor, 7, val2, val1);
	    break;
#endif
	case CONF_TIMING:
	    if(virtual != 0)  break;
	    /* check if the same bit rate as already active should be set */
	    if(Baud[minor] == val1) {ret = 0; break;}
	    if(su == TRUE) {
		ret = CAN_SetTiming(minor,(int) val1);   
		if (0 == ret) {
		    Baud[minor] = val1;
		} else {
		    ret = -EINVAL;
		}
	    } else {
		ret = -EPERM;
	    }
	   break;
	case CONF_SPEEDFACTOR:
	    if(virtual != 0)  break;
	    /* check if the same speedfactor as already active should be set */
	    if(Speedfactor[minor] == val1) {ret = 0; break;}
	    if(su == TRUE) {
		/* Temporay workaround */
                ret = 0;
#ifdef CAN4LINUX_SET_SPEEDFACTOR_FUNC
		ret = CAN4LINUX_SET_SPEEDFACTOR_FUNC(minor,(int) val1);
#endif /*CAN4LINUX_SET_SPEEDFACTOR_FUNC*/
		if (0 == ret) {
		    Speedfactor[minor] = val1;
		} else {
		    ret = -EINVAL;
		}
	    } else {
		ret = -EPERM;
	    }

	    break;			
	case CONF_OMODE:
	    if(virtual != 0)  break;
	    ret = CAN_SetOMode( minor, (int) val1);
	    break;			
#if CAN_USE_FILTER
	case CONF_FILTER:
	    Can_FilterOnOff( minor, (int) val1 );
	    break;
	case CONF_FENABLE:
	    Can_FilterMessage( minor, (int) val1, 1);
	    break;
	case CONF_FDISABLE:
	    Can_FilterMessage( minor, (int) val1, 0);
	    break;
#endif
	case CONF_LISTEN_ONLY_MODE:
	    if(virtual != 0)  break;
	    ret = CAN_SetListenOnlyMode( minor, (int) val1);
	    break;
	case CONF_SELF_RECEPTION:
	    DBGprint(DBG_DATA,
	    ("setting selfreception of minor %d to %d\n", minor, (int)val1));
	    selfreception[minor][rx_fifo] = (int)val1;
	    ret = 0;
	    break;
	case CONF_TIMESTAMP:
	    use_timestamp[minor] = (int)val1;
	    ret = 0;
	    break;
	case CONF_WAKEUP:
	    wakeup[minor] = (int)val1;
	    ret = 0;
	    break;
	case CONF_BTR:
	    if(virtual != 0)  break;
	    ret = CAN_SetBTR(minor, (int)val1, (int)val2);
	    break;

	default:
	    ret = -EINVAL;
    }
    DBGout();
    return ret;
}

int can_GetStatVirt(
	struct inode *inode,
	struct file *file,
	CanStatusPar_t *stat
	)
{
unsigned int minor = iminor(inode);	
msg_fifo_t *Fifo;
unsigned long flags;
int rx_fifo = ((struct _instance_data *)(file->private_data))->rx_index;


    stat->type = CAN_TYPE_VIRTUAL;

    stat->baud = Baud[minor];
    stat->status = 0; /*CANin(minor, canstat); */
    stat->error_warning_limit = 96; /* CANin(minor, errorwarninglimit); */
    stat->rx_errors = 0; /* CANin(minor, rxerror); */
    stat->tx_errors = 0; /* CANin(minor, txerror); */
    stat->error_code= 0; /* CANin(minor, errorcode); */ /* should reset this register */

    /* Disable CAN (All !!) Interrupts */
    /* !!!!!!!!!!!!!!!!!!!!! */
    /* save_flags(flags); cli(); */
    local_irq_save(flags);

    Fifo = &Rx_Buf[minor][rx_fifo];
    stat->rx_buffer_size = MAX_BUFSIZE;	/**< size of rx buffer  */
    /* number of messages */
    stat->rx_buffer_used =
    	(Fifo->head < Fifo->tail)
    	? (MAX_BUFSIZE - Fifo->tail + Fifo->head) : (Fifo->head - Fifo->tail);
    Fifo = &Tx_Buf[minor];
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

#endif /* DOXYGEN_SHOULD_SKIP_THIS */
