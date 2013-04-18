/*
 * lunix-chrdev.c
 *
 * Implementation of character devices
 * for Lunix:TNG
 *
 * Marios Kogias
 *
 */

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mmzone.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>
#include <linux/vmalloc.h>

#include "lunix.h"
#include "lunix-chrdev.h"
#include "lunix-lookup.h"

/*
 * Global data
 */
struct cdev lunix_chrdev_cdev;

/*
 * Just a quick [unlocked] check to see if the cached
 * chrdev state needs to be updated from sensor measurements.
 */
static int lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	
	WARN_ON ( !(sensor = state->sensor));
	if (state->buf_timestamp < sensor->msr_data[state->type]->last_update) {
		debug("need to update");
		return 1;
	}
	else {
		debug("no need to update");
 		return 0;
	} 
}

/*
 * Updates the cached state of a character device
 * based on sensor data. Must be called with the
 * character device state lock held.
 */
static int lunix_chrdev_state_update(struct lunix_chrdev_state_struct *state)
{
	if (!lunix_chrdev_state_needs_refresh(state))
		return  -EAGAIN;
	
	struct lunix_sensor_struct *sensor;

	debug("updating\n");

	sensor = state->sensor;
	/*check if update needed*/
		
	/*dictionaries for each measurement*/	
	long * dictionary[N_LUNIX_MSR];
	dictionary[BATT] = lookup_voltage;
	dictionary[TEMP] = lookup_temperature;
	dictionary[LIGHT] = lookup_light;

	uint32_t value;
	

	spin_lock(&sensor->lock);
	/*start of critical segment*/
	value = sensor->msr_data[state->type]->values[0];
	
	/*end of critical segment*/
	spin_unlock(&sensor->lock);
	debug("before formation\n");
	/*formation*/	
	long long_value = dictionary[state->type][value];
	
	int ak = long_value / 1000;
	int dec = long_value % 1000;

	state->buf_lim=sprintf(state->buf_data,"%d.%d\n",ak,dec);
        debug("DOULEPSE, %s\n",state->buf_data);
	state->buf_timestamp = get_seconds();
		/*
		 * Grab the raw data quickly, hold the
		 * spinlock for as little as possible.
		 */
		/* ? */
		/* Why use spinlocks? See LDD3, p. 119 */

		/*
		 * Any new data available?
		 */
		/* ? */

		/*
		 * Now we can take our time to format them,
		 * holding only the private state semaphore
		 */

		/* ? */

		debug("leaving\n");
	return 0;
}

/*************************************
 * Implementation of file operations
 * for the Lunix character device
 *************************************/

static int lunix_chrdev_open(struct inode *inode, struct file *filp)
{
	/* Declarations */
	/* ? */
	int ret;

	debug("entering\n");
	ret = -ENODEV;
	if ((ret = nonseekable_open(inode, filp)) < 0)
		goto out;

	/*
	 * Associate this open file with the relevant sensor based on
	 * the minor number of the device node [/dev/sensor<NO>-<TYPE>]
	 */
	
	/* Allocate a new Lunix character device private state structure */
	/* ? */
	struct lunix_chrdev_state_struct * state =  vmalloc(sizeof(struct lunix_chrdev_state_struct));
	state->sensor = lunix_sensors+iminor(inode)/8; // connect with a sensor
	
	debug("the sensor is %d\n",iminor(inode)/8);
	debug("the mesurement is %d\n",iminor(inode)%8);	
	switch (iminor(inode)%8) { // set the type
	
	case 0:
		state->type = BATT;
		break;	
	case 1:
		state->type = TEMP;
		break;
	case 2:
		state->type = LIGHT;
		break;

	}
	
	state->buf_timestamp = 0; // set the timestamp for the first time
	sema_init(&state->lock,1); //initialize the semaphore
	filp->private_data=state;
	ret = 0;
out:
	debug("leaving, with ret = %d\n", ret);
	return ret;
}

static int lunix_chrdev_release(struct inode *inode, struct file *filp)
{
	
	struct lunix_chrdev_state_struct *state;

	state = filp->private_data;
	if (state == NULL)
		debug("in release null");
	else debug("in release not NULL");
	vfree(state);
	return 0;
}

static long lunix_chrdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	/* Why? */
	return -EINVAL;
}

static ssize_t lunix_chrdev_read(struct file *filp, char __user *usrbuf, size_t cnt, loff_t *f_pos)
{
	ssize_t ret;

	struct lunix_sensor_struct *sensor;
	struct lunix_chrdev_state_struct *state;

	state = filp->private_data;
	WARN_ON(!state);

	sensor = state->sensor;
	WARN_ON(!sensor);


	/* Lock? */
	/*
	 * If the cached character device state needs to be
	 * updated by actual sensor data (i.e. we need to report
	 * on a "fresh" measurement, do so
	 */
	/*lock the semaphore*/
	if (down_interruptible(&state->lock))
 		return -ERESTARTSYS;
	/*read now*/
	if (*f_pos == 0) {
		while (lunix_chrdev_state_update(state) == -EAGAIN) { // EAGAIN = try again
			/* ? */
			/* The process needs to sleep */
			/* See LDD3, page 153 for a hint */
			if (wait_event_interruptible(sensor->wq,lunix_chrdev_state_needs_refresh(state)))
            			return -ERESTARTSYS;
		}
	}
	debug("before copying");
	int char_no = *f_pos / sizeof(unsigned char);
	if (cnt>(state->buf_lim-char_no)*sizeof(unsigned char))
		cnt = (state->buf_lim - char_no)*sizeof(unsigned char);
	
	char * temp = state->buf_data;
	copy_to_user(usrbuf,temp+char_no,cnt);
	
	*f_pos = *f_pos + cnt;	
	if ((cnt/sizeof(unsigned char) + char_no) == state->buf_lim)
		*f_pos = 0;

	
	ret = cnt; //need to return how much I've read
	/* End of file */
	/* ? */
	
	/* Determine the number of cached bytes to copy to userspace */
	/* ? */

	/* Auto-rewind on EOF mode? */
	/* ? */
out:
	/* Unlock? */
	debug("up the semaphore");
	up(&state->lock);

	return ret;
}

static int lunix_chrdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	return -EINVAL;
}

static struct file_operations lunix_chrdev_fops = 
{
        .owner          = THIS_MODULE,
	.open           = lunix_chrdev_open,
	.release        = lunix_chrdev_release,
	.read           = lunix_chrdev_read,
	.unlocked_ioctl = lunix_chrdev_ioctl,
	.mmap           = lunix_chrdev_mmap
};

int lunix_chrdev_init(void)
{
	/*
	 * Register the character device with the kernel, asking for
	 * a range of minor numbers (number of sensors * 8 measurements / sensor)
	 * beginning with LINUX_CHRDEV_MAJOR:0
	 */
	int ret;
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;
	
	debug("initializing character device\n");
	cdev_init(&lunix_chrdev_cdev, &lunix_chrdev_fops);
	lunix_chrdev_cdev.owner = THIS_MODULE;
	
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	register_chrdev_region(LUNIX_CHRDEV_MAJOR,16*3,"lunix");	

	if (ret < 0) {
		debug("failed to register region, ret = %d\n", ret);
		goto out;
	}	
	/* ? */
	/* cdev_add? */
	cdev_add(&lunix_chrdev_cdev,dev_no,16*3);	
	if (ret < 0) {
		debug("failed to add character device\n");
		goto out_with_chrdev_region;
	}
	debug("completed successfully\n");
	return 0;

out_with_chrdev_region:
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
out:
	return ret;
}

void lunix_chrdev_destroy(void)
{
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;
		
	debug("entering\n");
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	cdev_del(&lunix_chrdev_cdev);
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
	debug("leaving\n");
}
