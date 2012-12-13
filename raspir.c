#include <linux/module.h>
#include <linux/init.h>

#include <linux/sched.h>  /* current and everything */
#include <linux/kernel.h> /* printk() */
#include <linux/fs.h>     /* everything... */
#include <linux/types.h>  /* size_t */
#include <linux/completion.h>

#define RASPIR_DRIVER_NAME "raspir"

#define dprintk(fmt, args...)					\
  do {								\
    if (debug)							\
      printk(KERN_DEBUG RASPIR_DRIVER_NAME ": "			\
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

/*
 * Open and close.
 */

static int raspir_open(struct inode *inode, struct file *filp)
{
  return 0;
}

static int raspir_release(struct inode *inode, struct file *filp)
{
  return 0;
}

int raspir_ioctl (struct file *filp,
                 unsigned int cmd, unsigned long arg)
{
  return -ENOTTY; /* unknown command */
}


struct file_operations raspir_fops = {
  .owner          = THIS_MODULE,
  .open           = raspir_open,
  .release        = raspir_release,
  .unlocked_ioctl = raspir_ioctl,
#ifdef CONFIG_COMPAT
  .compat_ioctl   = raspir_ioctl,
#endif
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

module_param(debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Enable debugging messages");

/* EOF */
