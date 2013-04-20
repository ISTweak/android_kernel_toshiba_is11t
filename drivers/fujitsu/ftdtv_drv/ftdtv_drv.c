/*
 * Copyright(C) 2011 FUJITSU LIMITED
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <asm/atomic.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <asm/current.h>
#include <mach/vreg.h>
#include <linux/wakelock.h>

/* IOCTL CMD */
#define FTDTV_IOCTL_MAGIC		'f'

#define FTDTV_IOCTL_GET_SIGNAL	_IOC(_IOC_NONE, FTDTV_IOCTL_MAGIC, 0, 0)
#define FTDTV_IOCTL_POWER_ON	_IOC(_IOC_NONE, FTDTV_IOCTL_MAGIC, 1, 0)
#define FTDTV_IOCTL_POWER_OFF	_IOC(_IOC_NONE, FTDTV_IOCTL_MAGIC, 2, 0)

#define TSIF_CLK_ID					"TSIF_CLK"
#define TSIF_CLK_PORT				34
#define TSIF_CLK_POLLING_INTERVAL	10
#define TSIF_CLK_ANS_BIT			0x1

#define TSIF_EN_ID					"TSIF_EN"
#define TSIF_EN_PORT				35
#define TSIF_EN_POLLING_INTERVAL	1000
#define TSIF_EN_ANS_BIT				0x2

#define TSIF_DATA_ID				"TSIF_DATA"
#define TSIF_DATA_PORT				36
#define TSIF_DATA_POLLING_INTERVAL	10
#define TSIF_DATA_ANS_BIT			0x4

#define TSIF_CHECK_MAX_LOOP			1000
#define HIGH_VALUE					1
#define LOW_VALUE					0

int get_fsif_signal_value(char *id, int port, int interval, int anser);

static struct cdev ftdtvDevs;
static int ftdtv_major = 0;
static struct class* ftdtv_class;

#define VREG_TABLE  2
static struct vreg_info {
    char    *name;
    unsigned int    lvl;
    struct vreg *vreg;
} vreg_info[VREG_TABLE] = {
    {"wlan2",  2800, NULL},		//L19
    {"gp17",  1200, NULL},		//L25
};

#define GPIO_DTV_RST  37
#define DTV_RST_LOW 0
#define DTV_RST_HIGH 1
#define POWER_ON_DELAY_1	20
#define POWER_ON_DELAY_2	1

struct wake_lock ftdtv_wake_lock;

/*---------------------------------------------------------------------------
    open
---------------------------------------------------------------------------*/
int ftdtv_open(struct inode* inode, struct file* file)
{
	int ret;

	ret = 0;
	wake_lock(&ftdtv_wake_lock);
	gpio_set_value(GPIO_DTV_RST,DTV_RST_LOW);
	if (vreg_enable(vreg_info[0].vreg)) {
		printk("%s: * vreg %s enable failed !\n", __func__, vreg_info[0].name);
    	ret = -EIO;
	}
	if (vreg_enable(vreg_info[1].vreg)) {
		printk("%s: * vreg %s enable failed !\n", __func__, vreg_info[0].name);
    	ret = -EIO;
	}
	mdelay(POWER_ON_DELAY_1);
	gpio_set_value(GPIO_DTV_RST,DTV_RST_HIGH);
	mdelay(POWER_ON_DELAY_2);
	if ( ret != 0 )
		wake_unlock(&ftdtv_wake_lock);

	return ret;
}


/*---------------------------------------------------------------------------
    release
---------------------------------------------------------------------------*/
int ftdtv_release(struct inode *inode, struct file *file)
{
	int ret;

	ret = 0;
	gpio_set_value(GPIO_DTV_RST,DTV_RST_LOW);
	if (vreg_disable(vreg_info[1].vreg)) {
		printk("%s: * vreg %s enable failed !\n", __func__, vreg_info[0].name);
    	ret = -EIO;
	}
	if (vreg_disable(vreg_info[0].vreg)) {
		printk("%s: * vreg %s enable failed !\n", __func__, vreg_info[0].name);
    	ret = -EIO;
	}
	wake_unlock(&ftdtv_wake_lock);

	return ret;
}

/*---------------------------------------------------------------------------
    ftdtv_ioctl
---------------------------------------------------------------------------*/
static long ftdtv_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret;
	int anser;

	switch (cmd) {
	case FTDTV_IOCTL_GET_SIGNAL:
		anser = 0;
		anser |= get_fsif_signal_value(TSIF_CLK_ID, TSIF_CLK_PORT, TSIF_CLK_POLLING_INTERVAL, TSIF_CLK_ANS_BIT);
		anser |= get_fsif_signal_value(TSIF_EN_ID, TSIF_EN_PORT, TSIF_EN_POLLING_INTERVAL, TSIF_EN_ANS_BIT);
		anser |= get_fsif_signal_value(TSIF_DATA_ID, TSIF_DATA_PORT, TSIF_DATA_POLLING_INTERVAL, TSIF_DATA_ANS_BIT);
		if (copy_to_user((int __user *)arg, &anser, sizeof(int))) {
			printk(KERN_DEBUG "ftdtv_drv:%s: copy_from_user failed\n", __func__);
			ret = (-EFAULT);
		} else {
			ret = 0;
		}
		break;
/****
	case FTDTV_IOCTL_POWER_ON:
		ret = 0;
		gpio_set_value(GPIO_DTV_RST,DTV_RST_LOW);
		if (vreg_enable(vreg_info[0].vreg)) {
			printk("%s: * vreg %s enable failed !\n", __func__, vreg_info[0].name);
        	ret = -1;
		}
		if (vreg_enable(vreg_info[1].vreg)) {
			printk("%s: * vreg %s enable failed !\n", __func__, vreg_info[0].name);
        	ret = -1;
		}
		mdelay(POWER_ON_DELAY_1);
		gpio_set_value(GPIO_DTV_RST,DTV_RST_HIGH);
		mdelay(POWER_ON_DELAY_2);
		break;
	case FTDTV_IOCTL_POWER_OFF:
		ret = 0;
		gpio_set_value(GPIO_DTV_RST,DTV_RST_LOW);
		if (vreg_disable(vreg_info[1].vreg)) {
			printk("%s: * vreg %s enable failed !\n", __func__, vreg_info[0].name);
        	ret = -1;
		}
		if (vreg_disable(vreg_info[0].vreg)) {
			printk("%s: * vreg %s enable failed !\n", __func__, vreg_info[0].name);
        	ret = -1;
		}
		break;
****/
	default:
		printk(KERN_DEBUG "ftdtv_drv:%s: Command Failed cmd=0x%04x", __func__, cmd);
		ret = (-EINVAL);
		break;
	}
	return ret;
}

/*---------------------------------------------------------------------------
    get_fsif_signal_value
---------------------------------------------------------------------------*/
int get_fsif_signal_value(char *id, int port, int interval, int anser)
{
	long loop;
	int sig, pre_sig, count;
	
	pre_sig = HIGH_VALUE;
	count = 0;
	for (loop=0; loop<TSIF_CHECK_MAX_LOOP; loop++) {
		udelay(interval);
		sig = gpio_get_value(port);
		if (sig == pre_sig) {
			pre_sig = !sig;
			count++;
			if (count == 3) return anser;
		}
	}
	return 0;
}

static struct file_operations ftdtv_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.open = ftdtv_open,
	.unlocked_ioctl = ftdtv_ioctl,
	.release = ftdtv_release,
};
/*---------------------------------------------------------------------------
    ftdtv_setup_cdev
---------------------------------------------------------------------------*/
static void ftdtv_setup_cdev(struct cdev *dev, int minor, struct file_operations *fops)
{
	int err, devno;

	devno = MKDEV(ftdtv_major, minor);
	cdev_init(dev, fops);
	dev->owner = THIS_MODULE;
	dev->ops = fops;
	err = cdev_add(dev, devno, 1);
	if (err) {
		printk(KERN_DEBUG "ftdtv_setup_cdev Error %d adding rfs%d\n", err, minor);
	}
	if (IS_ERR(device_create(ftdtv_class, NULL, devno, NULL, "ftdtv"))) {
		printk(KERN_DEBUG "ftdtv_setup_cdev can't create device\n");
	}
}

/*---------------------------------------------------------------------------
    init
---------------------------------------------------------------------------*/
static int __init ftdtv_init(void)
{
	int result = 0;
	dev_t dev;
	int i;
	int rc;

	dev = MKDEV(ftdtv_major, 0);

	if (ftdtv_major) {
		result = register_chrdev_region(dev, 2, "ftdtv");
	} else {
		result = alloc_chrdev_region(&dev, 0, 2, "ftdtv");
		ftdtv_major = MAJOR(dev);
	}
	if (result < 0) {
		printk(KERN_DEBUG "ftdtv error fail to get major %d\n", ftdtv_major);
		return result;
	}
	ftdtv_class = class_create(THIS_MODULE, "ftdtv");
	if (IS_ERR(ftdtv_class)) {
	    printk(KERN_DEBUG "ftdtv IS_ERR(ftdtv_class)\n");
		return PTR_ERR(ftdtv_class);
	}

	ftdtv_setup_cdev(&ftdtvDevs, 0, &ftdtv_fops);

	rc = gpio_tlmm_config(GPIO_CFG(GPIO_DTV_RST,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    if (rc) {
        pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
            __func__, GPIO_CFG(GPIO_DTV_RST,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), rc);
        return -1;
    }

	for (i=0;i<VREG_TABLE;++i) {
	    vreg_info[i].vreg = vreg_get(NULL, vreg_info[i].name);
		printk("name = %s Level = %d\n",vreg_info[i].name,vreg_info[i].lvl);
		if (IS_ERR(vreg_info[i].vreg)) {
	        printk("%s: * vreg_get(%s) failed (%ld) !\n",
	            __func__, vreg_info[i].name, PTR_ERR(vreg_info[i].vreg));
	        break;
	    }
	}
	for (i=0;i<VREG_TABLE;++i) {
        if (vreg_set_level(vreg_info[i].vreg, vreg_info[i].lvl)) {
            printk("%s: * %s set level failed !\n", __func__, vreg_info[i].name);
            return -1;
        }
	}
	wake_lock_init(&ftdtv_wake_lock, WAKE_LOCK_SUSPEND,"ft_dtv");
	return 0;
}

/*---------------------------------------------------------------------------
    exit
---------------------------------------------------------------------------*/
static void __exit ftdtv_exit(void)
{

	wake_lock_destroy(&ftdtv_wake_lock);
	cdev_del(&ftdtvDevs);
	unregister_chrdev_region(MKDEV(ftdtv_major, 0), 2);
	device_destroy(ftdtv_class, MKDEV(ftdtv_major, 0));
	class_destroy(ftdtv_class);
}

module_init(ftdtv_init);
module_exit(ftdtv_exit);
MODULE_LICENSE("GPL");
