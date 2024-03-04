#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/kfifo.h>
#include <linux/cdev.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/io.h>

#include "rpigpio-rec.h"

#define DEVICE_NAME  "rpigpio-rec"
#define FIFO_SIZE    1024 // shell be power of 2
#define READ_TIMEOUT msecs_to_jiffies(33)
#define MAX_GPIOS    32

struct rpigrec_pin_info {
	int gpio;
	uint32_t last_value;
	struct kfifo change_times_fifo;
	struct cdev cdev;
	int major;
	int minor;
	int irq;
	int timeout;
	wait_queue_head_t queue;
};

static int gpios[MAX_GPIOS] = {17}; // default GPIO pins (!)
static int num_gpios = 1;

static int read_timeout = 0;

module_param(read_timeout, int, S_IRUGO);
module_param_array(gpios, int, &num_gpios, S_IRUGO);
MODULE_PARM_DESC(gpios, "GPIO pin numbers");

struct rpigrec_pin_info *rpigrec_pins[MAX_GPIOS] = {0};

static long ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static ssize_t read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
static int open(struct inode *inode, struct file *file);

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .read = read,
    .open = open,
    .unlocked_ioctl = ioctl,
};

#define BCM2708_PERI_BASE 0x3F000000
#define GPIO_BASE	  (BCM2708_PERI_BASE + 0x200000) /* GPIO Controller */
#define GPPUD		  (GPIO_BASE + 0x94)		 /* GPIO Pull-up/down Register */
#define GPPUDCLK0	  (GPIO_BASE + 0x98)		 /* GPIO Pull-up/down Clock Register 0 */

#define GPIO_PULLOFF  0
#define GPIO_PULLDOWN 1
#define GPIO_PULLUP   2

void set_gpio_pullupdown(int gpio, int pud)
{
	uint32_t *gppud, *gppudclk;
	void __iomem *ioaddr;

	ioaddr = ioremap(GPIO_BASE, SZ_4K);
	gppud = (uint32_t *)(ioaddr + GPPUD);
	gppudclk = (uint32_t *)(ioaddr + GPPUDCLK0 + (gpio / 32) * 4);

	writel(pud & 3, gppud);
	udelay(5);

	writel(1 << (gpio % 32), gppudclk);
	udelay(5);

	writel(0, gppud);
	writel(0, gppudclk);

	iounmap(ioaddr);
}

static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
	struct rpigrec_pin_info *pin_info = (struct rpigrec_pin_info *)dev_id;
	uint64_t time = (uint64_t)ktime_get_real_ns() & (uint64_t)(~0x1);
	int value = gpio_get_value(pin_info->gpio);
	pin_info->last_value = value;

	if (value)
		time |= 1;

	if (!kfifo_is_full(&pin_info->change_times_fifo)) {
		kfifo_in(&pin_info->change_times_fifo, &time, sizeof(uint64_t));
		wake_up_interruptible(&pin_info->queue);
	}

	return IRQ_HANDLED;
}

static ssize_t read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct rpigrec_pin_info *pin_info = filp->private_data;
	int channel = MINOR(filp->f_inode->i_rdev);
	u64 *tmp_buf = NULL;
	int ret;

	if (kfifo_is_empty(&pin_info->change_times_fifo)) {
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		ret = wait_event_interruptible_timeout(pin_info->queue, !kfifo_is_empty(&pin_info->change_times_fifo), pin_info->timeout);
		if (ret == 0)
			return -ETIMEDOUT;
		if (ret < 0)
			return -ERESTARTSYS;
	}

	count = min(count, (size_t)kfifo_len(&pin_info->change_times_fifo));

	tmp_buf = kmalloc(count, GFP_KERNEL);
	if (!tmp_buf) {
		return -ENOMEM;
	}

	count = kfifo_out(&pin_info->change_times_fifo, tmp_buf, count);

	if (copy_to_user(buf, tmp_buf, count)) {
		kfree(tmp_buf);
		return -EFAULT;
	}

	kfree(tmp_buf);

	return count;
}

static long ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct rpigrec_pin_info *pin_info = filp->private_data;
	int channel = MINOR(filp->f_inode->i_rdev);
	uint32_t gpio_value = (pin_info->last_value ^ (1 << channel)) ? 1 : 0;

	switch (cmd) {
	case RPIGREC_GET_GPIO_VALUE:
		if (copy_to_user((int __user *)arg, &gpio_value, sizeof(gpio_value))) {
			return -EFAULT;
		}
		break;
	case RPIGREC_BUFFER_RESET:
		kfifo_reset_out(&pin_info->change_times_fifo);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int open(struct inode *inode, struct file *file)
{
	struct rpigrec_pin_info *pin_info;
	int index = iminor(inode) % num_gpios;

	pin_info = container_of(inode->i_cdev, struct rpigrec_pin_info, cdev);

	file->private_data = pin_info;
	return 0;
}

static int __init gpio_module_init(void)
{
	int i;
	int result;
	struct rpigrec_pin_info *pin_info;
	dev_t pin_no;
	int success_devs = 0;

	if ((result = alloc_chrdev_region(&pin_no, 0, num_gpios, DEVICE_NAME)) < 0) {
		goto error_alloc_chrdev;
	}

	for (i = 0; i < num_gpios; i++) {
		pin_info = kzalloc(sizeof(*pin_info), GFP_KERNEL);
		if (!pin_info) {
			goto error_mem_alloc;
		}

		pin_info->major = MAJOR(pin_no);
		pin_info->minor = i;
		pin_info->gpio = gpios[i];
		pin_info->timeout = msecs_to_jiffies(read_timeout);

		init_waitqueue_head(&pin_info->queue);
		if (kfifo_alloc(&pin_info->change_times_fifo, FIFO_SIZE * sizeof(uint64_t), GFP_KERNEL)) {
			goto error_fifo_alloc;
		}

		cdev_init(&pin_info->cdev, &fops);
		pin_info->cdev.owner = THIS_MODULE;
		pin_no = MKDEV(pin_info->major, i);
		if ((result = cdev_add(&pin_info->cdev, pin_no, 1)) < 0) {
			goto error_cdev;
		}

		gpio_request(pin_info->gpio, "sysfs");
		gpio_direction_input(pin_info->gpio);
		gpio_export(pin_info->gpio, false);

		pin_info->irq = gpio_to_irq(pin_info->gpio);
		if ((result = request_irq(pin_info->irq, gpio_irq_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "rpi_gpio_log_irq", (void *)pin_info)) <
		    0) {
			goto error_irq;
		}

		rpigrec_pins[i] = pin_info;

		success_devs++;
		continue;

	error_irq:
		printk(KERN_ERR "Failed to request IRQ for GPIO %d: %d\n", gpios[i], result);
		free_irq(pin_info->irq, (void *)pin_info);
	error_cdev:
		cdev_del(&pin_info->cdev);
		gpio_unexport(pin_info->gpio);
		gpio_free(pin_info->gpio);
	error_fifo_alloc:
		kfifo_free(&pin_info->change_times_fifo);
	error_mem_alloc:
		kfree(pin_info);
		rpigrec_pins[i] = NULL;
	}

	if (success_devs == 0)
		goto error_alloc_chrdev;

	printk(KERN_INFO "GPIO module with multiple GPIO support loaded\n");
	return 0;

error_alloc_chrdev:
	unregister_chrdev_region(MKDEV(pin_info->major, 0), num_gpios);
	return -EIO;
}

static void __exit gpio_module_exit(void)
{
	int i;
	for (i = 0; i < num_gpios; i++) {
		if (rpigrec_pins[i] != NULL) {
			struct rpigrec_pin_info *pin_info = rpigrec_pins[i];

			free_irq(pin_info->irq, (void *)pin_info);
			cdev_del(&pin_info->cdev);
			gpio_unexport(pin_info->gpio);
			gpio_free(pin_info->gpio);
			kfifo_free(&pin_info->change_times_fifo);
			kfree(pin_info);
			rpigrec_pins[i] = NULL;
		}
	}
	printk(KERN_INFO "GPIO module with multiple GPIO support unloaded\n");
}

module_init(gpio_module_init);
module_exit(gpio_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alexander Krapivnyy ");
MODULE_DESCRIPTION("GPIO time recording Kernel Module for Rasperry PI");
