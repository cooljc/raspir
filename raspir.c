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
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include "raspir.h"

#define RASPIR_DRIVER_NAME "raspir"

#ifndef MAX_UDELAY_MS
#define MAX_UDELAY_US 5000
#else
#define MAX_UDELAY_US (MAX_UDELAY_MS*1000)
#endif

/* ------------------------------------------------------------------ */
/* Debug MACRO used to print data to syslog. */
/* ------------------------------------------------------------------ */
#define dprintk(fmt, args...)					\
  do {								\
    if (debug)							\
      printk(KERN_INFO RASPIR_DRIVER_NAME ": "			\
	     fmt, ## args);					\
  } while (0)


/* ------------------------------------------------------------------ */
/* module parameters passed in via modprobe or insmod when module is
 * loaded */
/* ------------------------------------------------------------------ */
/* set the default GPIO input pin */
static int gpio_in_pin = 18;
/* set the default GPIO output pin */
static int gpio_out_pin = 4;
/* enable debugging messages */
static int debug = 1;

/* ------------------------------------------------------------------ */
/* Global variables required for driver operation. */
/* ------------------------------------------------------------------ */
static int raspir_major = 0;
static unsigned long driver_open;
static struct class *raspir_class = 0;
struct gpio_chip *gpiochip;

/* =================================================================
 * Matsushita Kaseikyo protocol:
 *
 * +--------+--------------+---------+
 * | Leader | 48 Data Bits | Trailer |
 * +--------+--------------+---------+
 *
 *         +----------+
 * Leader: |    8T    |  4T
 *         +          +------+
 *
 *          +---+
 * Trailer: | T |  171T
 *          +   +----------....----+
 *
 * Data bits
 *          +---+
 * Logic 1: | T |   3T
 *          +   +---------+
 *
 *          +---+
 * Logic 0: | T | T
 *          +   +---+
 *
 * T = 436uS
 * Pulse: 50/50 duty cycle
 * ================================================================== */
#define T_DELAY 14

static void safe_udelay(unsigned long usecs)
{
  while (usecs > MAX_UDELAY_US) {
    udelay(MAX_UDELAY_US);
    usecs -= MAX_UDELAY_US;
  }
  udelay(usecs);
}

static void raspir_t_on (unsigned char t)
{
  unsigned char loop=0;
  unsigned char pulse=0;

  for (loop=0; loop<t; loop++) {
    for (pulse=0; pulse<16; pulse++) {
      /* IR On */
      gpiochip->set(gpiochip, gpio_out_pin, 1);
      /* Wait T/32 */
      safe_udelay (T_DELAY);
      /* IR Off */
      gpiochip->set(gpiochip, gpio_out_pin, 0);
      /* Wait T/32 */
      safe_udelay (T_DELAY);
    }
  }
}

static void raspir_t_off (unsigned char t)
{
  unsigned long delay = (t * (T_DELAY*32));
  /* delay */
  safe_udelay (delay);
}

static void raspir_send_bit (unsigned char bit)
{
  if ( (bit & 0x01) == 1) {
    raspir_t_on  (1);
    raspir_t_off (3);
  }
  else {
    raspir_t_on  (1);
    raspir_t_off (1);
  }
}

static void raspir_send_byte (unsigned char byte, unsigned char lsbFirst)
{
  unsigned char loop=0;
  for (loop=0; loop<8; loop++) {
    if (!lsbFirst)
      raspir_send_bit ( ((byte >> (8-(loop+1)) ) & 0x01) );
    else
      raspir_send_bit ( ((byte >> loop)& 0x01) );
  }
}

void raspir_send_command (unsigned short manid, unsigned short equip_code, unsigned char data_code)
{
  unsigned char bytes[6];
  unsigned char loop;

  bytes[1] = (unsigned char)(manid & 0xff);              /* MAKER_CODE_PANA_HB */
  bytes[0] = (unsigned char)((manid >> 8) & 0xff);       /* MAKER_CODE_PANA_LB */

  bytes[3] = (unsigned char)(equip_code & 0xff);         /* parity+equip_code */
  bytes[2] = (unsigned char)((equip_code >> 8) & 0xff);  /* equip_code */
  bytes[4] = data_code;                                  /* data code */
  bytes[5] = (bytes[2] ^ bytes[3] ^ bytes[4]);           /* checksum */

  /* send leader */
  raspir_t_on (8);
  raspir_t_off (4);

  /* send Bytes */
  for (loop=0; loop<6; loop++) {
    raspir_send_byte (bytes[loop], 1);
  }
  /* send trailer */
  raspir_t_on (1);
  raspir_t_off (171);
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static int raspir_is_right_chip(struct gpio_chip *chip, void *data)
{
  dprintk("raspir_is_right_chip %s %d\n", chip->label,
	  strcmp(data, chip->label));

  if (strcmp(data, chip->label) == 0)
    return 1;
  return 0;
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static int raspir_init_port(void)
{
  int ret;

  gpiochip = gpiochip_find("bcm2708_gpio", raspir_is_right_chip);

  if (!gpiochip)
    return -ENODEV;

  if (gpio_request(gpio_out_pin, RASPIR_DRIVER_NAME " ir/out")) {
    printk(KERN_ALERT RASPIR_DRIVER_NAME
	   ": cant claim gpio pin %d\n", gpio_out_pin);
    ret = -ENODEV;
    goto exit_init_port;
  }

  if (gpio_request(gpio_in_pin, RASPIR_DRIVER_NAME " ir/in")) {
    printk(KERN_ALERT RASPIR_DRIVER_NAME
	   ": cant claim gpio pin %d\n", gpio_in_pin);
    ret = -ENODEV;
    goto exit_gpio_free_out_pin;
  }

  gpiochip->direction_input(gpiochip, gpio_in_pin);
  gpiochip->direction_output(gpiochip, gpio_out_pin, 1);
  gpiochip->set(gpiochip, gpio_out_pin, 0);

#ifdef RECEIVER_ENABLED
  irq = gpiochip->to_irq(gpiochip, gpio_in_pin);
  dprintk("to_irq %d\n", irq);
  irqdata = irq_get_irq_data(irq);

  if (irqdata && irqdata->chip) {
    irqchip = irqdata->chip;
  } else {
    ret = -ENODEV;
    goto exit_gpio_free_in_pin;
  }
#endif

  return 0;

#ifdef RECEIVER_ENABLED
 exit_gpio_free_in_pin:
  gpio_free(gpio_in_pin);
#endif

 exit_gpio_free_out_pin:
  gpio_free(gpio_out_pin);

 exit_init_port:
  return ret;
}

/* ------------------------------------------------------------------ */
/* raspir_open()
 * This function is called form userland using open(). If successful
 * it will return a file descriptor that can be used with ioctl() and
 * close() system functions. */
/* ------------------------------------------------------------------ */
static int raspir_open(struct inode *inode, struct file *file)
{
  if (test_and_set_bit(0, &driver_open))
    return -EBUSY;
  
  return nonseekable_open(inode, file);
}

/* ------------------------------------------------------------------ */
/* raspir_release()
 * This function is called from userland using close() on an open
 * file descriptor. */
/* ------------------------------------------------------------------ */
static int raspir_release(struct inode *inode, struct file *file)
{
  clear_bit(0, &driver_open);
  return 0;
}

/* ------------------------------------------------------------------ */
/* raspir_ioctl()
 * This function is called from userland using ioctl() on an open
 * file descriptor to /dev/raspir. */
/* ------------------------------------------------------------------ */
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
      raspir_send_command (ir_cmd.m_manufacturer_id,
			   ir_cmd.m_equipment_code,
			   ir_cmd.m_ir_code);
    }
    break;
  }

  return ret;
}

/* ------------------------------------------------------------------ */
/* Define modules file operations. */
/* ------------------------------------------------------------------ */
struct file_operations raspir_fops = {
  .owner          = THIS_MODULE,
  .open           = raspir_open,
  .release        = raspir_release,
  .unlocked_ioctl = raspir_ioctl,
};

/* ------------------------------------------------------------------ */
/* raspir_cleanup()
 * This function is called when the module is removed or init fails.  */
/* ------------------------------------------------------------------ */
void raspir_cleanup(void)
{
  /* remove /dev/raspir */
  if (raspir_class) {
    device_destroy(raspir_class, MKDEV(raspir_major, 0));
    class_destroy(raspir_class);
  }

  /* free GPIO pins */
  gpio_free(gpio_out_pin);
  gpio_free(gpio_in_pin);

  /* unregister char device */
  unregister_chrdev(raspir_major, RASPIR_DRIVER_NAME);
}

/* ------------------------------------------------------------------ */
/* raspir_init()
 * This function is called when the module is inserted/loaded. */
/* ------------------------------------------------------------------ */
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

  /* claim GPIO pins */
  result = raspir_init_port();
  if (result < 0)
    goto fail;

  raspir_class = class_create(THIS_MODULE, RASPIR_DRIVER_NAME);
  if (IS_ERR(raspir_class)) {
    result = PTR_ERR(raspir_class);
    goto fail;
  }

  /* create device in /dev/ directory */
  device_create(raspir_class, NULL, MKDEV(raspir_major, 0), NULL, "%s", RASPIR_DRIVER_NAME);
  dprintk ("device created: /dev/raspir\n");

  return 0;

 fail:
  raspir_cleanup ();
  return result;
}

/* ------------------------------------------------------------------ */
/* register module load/exit functions */
/* ------------------------------------------------------------------ */
module_init(raspir_init);
module_exit(raspir_cleanup);

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
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
