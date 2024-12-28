#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>

#include <linux/kernel.h> /* printk() */
#include <linux/slab.h>   /* kmalloc() */
#include <linux/fs.h>     /* everything... */
#include <linux/errno.h>  /* error codes */
#include <linux/types.h>  /* size_t */
#include <linux/proc_fs.h>
#include <linux/fcntl.h> /* O_ACCMODE */
#include <linux/seq_file.h>
#include <linux/cdev.h>
#include <linux/mutex.h>
#include <linux/semaphore.h>
#include <linux/sched.h>

// #include <asm/system.h>         /* cli(), *_flags */
#include <asm/uaccess.h>

int simple_major = 0;
int simple_minor = 0;
int memsize = 255;

module_param(simple_major, int, S_IRUGO);
module_param(simple_minor, int, S_IRUGO);
module_param(memsize, int, S_IRUGO);

MODULE_AUTHOR("Shady Abdelmalek");
MODULE_LICENSE("Dual BSD/GPL");

struct simple_dev
{
        char *data;           /* Pointer to data area */
        struct semaphore sem; /* Mutual exclusion semaphore */
        struct cdev cdev;     /* Char device structure */
};

struct simple_dev simple_device; /* Instance of the device structure */

int simple_open(struct inode *inode, struct file *filp)
{
        struct simple_dev *dev;

        dev = container_of(inode->i_cdev, struct simple_dev, cdev);
        filp->private_data = dev; /* For other methods */

        return 0;
}

int simple_release(struct inode *inode, struct file *filp)
{
        return 0;
}

ssize_t simple_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
        struct simple_dev *dev = filp->private_data;
        ssize_t retval = -ENOMEM;

        if (down_interruptible(&dev->sem))
                return -ERESTARTSYS;

        /* Adjust count if it exceeds available memory */
        if (*f_pos + count > memsize)
                count = memsize - *f_pos;

        /* Copy data from user-space */
        if (copy_from_user(dev->data + *f_pos, buf, count))
        {
                retval = -EFAULT;
                goto out;
        }

        *f_pos += count;
        retval = count;

        /* Log the written data */
        printk(KERN_INFO "Written data: %.*s\n", (int)count, dev->data);

out:
        up(&dev->sem);
        return retval;
}

struct file_operations simple_fops = {
    .owner = THIS_MODULE,
    .write = simple_write,
    .open = simple_open,
    .release = simple_release,
};

void simple_cleanup_module(void)
{
        dev_t devno = MKDEV(simple_major, simple_minor);

        /* Free the char dev */
        cdev_del(&simple_device.cdev);

        /* Free the memory */
        kfree(simple_device.data);

        /* Unregister the device */
        unregister_chrdev_region(devno, 1);
}

int simple_init_module(void)
{
        int result;
        dev_t dev = 0;

        if (simple_major)
        {
                dev = MKDEV(simple_major, simple_minor);
                result = register_chrdev_region(dev, 1, "simple");
        }
        else
        {
                result = alloc_chrdev_region(&dev, simple_minor, 1, "simple");
                simple_major = MAJOR(dev);
        }
        if (result < 0)
        {
                printk(KERN_WARNING "simple: can't get major %d\n", simple_major);
                return result;
        }

        /* Memory allocation for the device */
        simple_device.data = kmalloc(memsize * sizeof(char), GFP_KERNEL);
        if (!simple_device.data)
        {
                result = -ENOMEM;
                goto fail_malloc;
        }
        memset(simple_device.data, 0, memsize * sizeof(char));

        sema_init(&simple_device.sem, 1);
        cdev_init(&simple_device.cdev, &simple_fops);
        simple_device.cdev.owner = THIS_MODULE;
        result = cdev_add(&simple_device.cdev, dev, 1);

        if (result)
        {
                printk(KERN_NOTICE "Error %d adding simple device", result);
                goto fail_add;
        }

        return 0;

fail_add:
        kfree(simple_device.data);
fail_malloc:
        unregister_chrdev_region(dev, 1);
        return result;
}

module_init(simple_init_module);
module_exit(simple_cleanup_module);
