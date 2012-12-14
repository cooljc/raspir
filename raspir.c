/*
 * raspir.c
 * 
 * Copyright 2012 Jon Cross <joncross.cooljc@gmail.com>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */
#include <linux/module.h>
#include <linux/init.h>

#include <linux/sched.h>  /* current and everything */
#include <linux/kernel.h> /* printk() */
#include <linux/fs.h>     /* everything... */
#include <linux/types.h>  /* size_t */
#include <linux/completion.h>
#include <linux/uaccess.h>

#include "raspir.h"

#define RASPIR_DRIVER_NAME "raspir"

#define dprintk(fmt, args...)					\
  do {								\
    if (debug)							\
      printk(KERN_INFO RASPIR_DRIVER_NAME ": "			\
	     fmt, ## args);					\
  } while (0)

/* module parameters */

/* set the default GPIO input pin */
static int gpio_in_pin = 18;
/* set the default GPIO output pin */
static int gpio_out_pin = 4;
/* enable debugging messages */
static int debug = 1;


static int raspir_major = 0;
static unsigned long driver_open;

/*
 * Open and close.
 */

static int raspir_open(struct inode *inode, struct file *file)
{
  if (test_and_set_bit(0, &driver_open))
    return -EBUSY;
  
  return nonseekable_open(inode, file);
}

static int raspir_release(struct inode *inode, struct file *file)
{
  clear_bit(0, &driver_open);
  return 0;
}

static long raspir_ioctl (struct file *file, unsigned int cmd, 
			  unsigned long arg)
{
  void __user *argp = (void __user *)arg;
  long ret = -ENOTTY; /* unknown command */
  raspir_status_t status;
  raspir_command_t ir_cmd;
  

  /* check command magic code matches */
  if(_IOC_TYPE(cmd) != RASPIR_IOC_MAGIC) {
    return ret;
  }

  switch (cmd) {
  case RASPIR_IOC_GETSTATUS:
    /* add some dummy data to test interface */
    status.m_mode = 0x55;
    status.m_status = 0xaa;
    ret = copy_to_user(argp, &status, sizeof(status)) ? -EFAULT : 0;
    break;
  case RASPIR_IOC_SENDCMD:
    ret = copy_from_user(&ir_cmd, argp, sizeof(ir_cmd)) ? -EFAULT : 0;
    if (ret == 0) {
      /* add command to send queue */
      dprintk ("send: %04x %04x %02x %02x\n",
	       ir_cmd.m_manufacturer_id,
	       ir_cmd.m_equipment_code,
	       ir_cmd.m_ir_code,
	       ir_cmd.m_repeat);
    }
    break;
  }

  return ret;
}


struct file_operations raspir_fops = {
  .owner          = THIS_MODULE,
  .open           = raspir_open,
  .release        = raspir_release,
  .unlocked_ioctl = raspir_ioctl,
};


int raspir_init(void)
{
  int result;

  /*
   * Register your major, and accept a dynamic number
   */
  result = register_chrdev(raspir_major, RASPIR_DRIVER_NAME, &raspir_fops);
  if (result < 0)
    return result;
  if (raspir_major == 0)
    raspir_major = result; /* dynamic */

  /* Print so info out */
  dprintk ("I was assigned major number %d. To talk to\n", raspir_major);
  dprintk ("the driver, create a dev file with\n");
  dprintk ("'mknod /dev/%s c %d 0'.\n", RASPIR_DRIVER_NAME, raspir_major);
  
  return 0;
}

void raspir_cleanup(void)
{
  unregister_chrdev(raspir_major, RASPIR_DRIVER_NAME);
}

module_init(raspir_init);
module_exit(raspir_cleanup);

MODULE_DESCRIPTION("Panasonic Infra-red receiver and blaster driver for Raspberry Pi GPIO.");
MODULE_AUTHOR("Jon Cross <joncross.cooljc@gmail.com>");
MODULE_LICENSE("GPL");

module_param(gpio_out_pin, int, S_IRUGO);
MODULE_PARM_DESC(gpio_out_pin, "GPIO output/transmitter pin number of the BCM"
		 " processor. Valid pin numbers are: 0, 1, 4, 8, 7, 9, 10, 11,"
		 " 14, 15, 17, 18, 21, 22, 23, 24, 25, default 17");

module_param(gpio_in_pin, int, S_IRUGO);
MODULE_PARM_DESC(gpio_in_pin, "GPIO input pin number of the BCM processor."
		 " Valid pin numbers are: 0, 1, 4, 8, 7, 9, 10, 11, 14, 15,"
		 " 17, 18, 21, 22, 23, 24, 25, default 18");

module_param(debug, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Enable debugging messages");

/* EOF */
