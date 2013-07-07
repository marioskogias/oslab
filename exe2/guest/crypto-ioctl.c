/*
 * crypto-ioctl.c
 *
 * Implementation of ioctl for 
 * virtio-crypto (guest side) 
 *
 * Stefanos Gerangelos <sgerag@cslab.ece.ntua.gr>
 *                                                                               
 */

#include <linux/cdev.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/freezer.h>
#include <linux/list.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/virtio.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/ioctl.h>

#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include "crypto.h"
#include "crypto-vq.h"
#include "crypto-chrdev.h"

long crypto_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct crypto_device *crdev;
	struct crypto_data *cr_data;
	ssize_t ret;
	struct session_op __user * sess;
	struct crypt_op __user * crypt;
	int i;
	unsigned long int flags;

	crdev = filp->private_data;

	cr_data = kzalloc(sizeof(crypto_data), GFP_KERNEL);
	if (!cr_data)
		return -ENOMEM;

	cr_data->cmd = cmd;

	switch (cmd) {

        /* get the metadata for every operation 
	 * from userspace and send the buffer 
         * to the host */

	case CIOCGSESSION:
		/* ? */
		/*we need to copy session_op at the arg pointer*/
		
		sess = (struct session_op __user *) arg;	
	
		/*copy session_op*/
		ret = copy_from_user(&cr_data->op.sess,sess,sizeof(struct session_op));
		debug("i didn't copy %lu\n",ret);	
		/*copy key*/	
		ret = copy_from_user(cr_data->keyp,sess->key,sess->keylen*sizeof(unsigned char));
		debug("i didn't copy %lu\n",ret);	
	
		debug("the key len is : %d\n",cr_data->op.sess.keylen);
		debug("in ciocgsession before send the data");	
		
		/*lock the whole device*/
		spin_lock_irqsave(&crdev->general,flags);
	
		send_buf(crdev,cr_data,sizeof(crypto_data),true);		
			
		/*unlock the whole device*/
		spin_unlock_irqrestore(&crdev->general,flags);
		
		if (!device_has_data(crdev)) {
			debug("sleeping in CIOCGSESSION\n");
			if (filp->f_flags & O_NONBLOCK)
				return -EAGAIN;

			/* Go to sleep unti we have data. */
			ret = wait_event_interruptible(crdev->i_wq,
			                               device_has_data(crdev));

			if (ret < 0) 
				goto free_buf;
		}
		printk(KERN_ALERT "woke up in CIOCGSESSION\n");

		if (!device_has_data(crdev))
			return -ENOTTY;

		/*lock the whole device*/
		spin_lock_irqsave(&crdev->general,flags);
	
		ret = fill_readbuf(crdev, (char *)cr_data, sizeof(crypto_data));
		
		/*unlock the whole device*/
		spin_unlock_irqrestore(&crdev->general,flags);
		

		debug("after fill readbuf\n");
		/* copy the response to userspace */
		/* ? */

		ret = copy_to_user((void __user *)arg, &cr_data->op.sess, 
		                   sizeof(struct session_op));
		break;

	case CIOCCRYPT: 
		/* ? */
		/*need to copy the crypt_op*/
		
		crypt = (struct crypt_op __user * ) arg;
		
		ret = copy_from_user(&cr_data->op.crypt,crypt,sizeof(struct crypt_op));

		/*copy vector*/
		ret = copy_from_user(cr_data->ivp,crypt->iv,sizeof(cr_data->ivp));
	
		/*copy data in*/
		
		ret = copy_from_user(cr_data->srcp,crypt->src,crypt->len);
		
		/*lock the whole device*/
		spin_lock_irqsave(&crdev->general,flags);	
	
		/*send the data to the host*/	
		send_buf(crdev,cr_data,sizeof(crypto_data),true);		
		
		/*unlock the whole device*/
		spin_unlock_irqrestore(&crdev->general,flags);
	
		if (!device_has_data(crdev)) {
			printk(KERN_WARNING "sleeping in CIOCCRYPTO\n");	
			if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
			
			/* Go to sleep until we have data. */
			ret = wait_event_interruptible(crdev->i_wq,
			device_has_data(crdev));
			
			if (ret < 0)
			goto free_buf;
		}

		/*lock the whole device*/
		spin_lock_irqsave(&crdev->general,flags);	
	
		ret = fill_readbuf(crdev, (char *)cr_data, sizeof(crypto_data));
		
		/*unlock the whole device*/
		spin_unlock_irqrestore(&crdev->general,flags);
		

		/*fix the correct data place*/
		cr_data->op.crypt.dst = crypt->dst;

		cr_data->op.crypt.iv = crypt->iv;

		cr_data->op.crypt.src = crypt->src;

		/*now copy to user*/

		ret = copy_to_user((void __user *)arg, &cr_data->op.crypt, 
		                   sizeof(struct crypt_op));
		if (ret)
			goto free_buf;

		debug("after check and before copying the encrypted\n");
		for (i=0;i<crypt->len;i++)
			printk("%x",cr_data->dstp[i]);
		
		printk("\n\n");
		/* copy the response to userspace */
		/* ? */
		/*copy the encrypted data to the correct user space buffer*/
		ret = copy_to_user((void __user*)crypt->dst,cr_data->dstp,crypt->len);
		break;

	case CIOCFSESSION:

		/* ? */
		/*we need to copy sess.ses*/
		ret = copy_from_user(&cr_data->op.sess_id,(void __user*)arg,sizeof(uint32_t));

		/*lock the whole device*/
		spin_lock_irqsave(&crdev->general,flags);	

		/*send the data to the host*/	
		send_buf(crdev,cr_data,sizeof(crypto_data),true);		

		/*unlock the whole device*/
		spin_unlock_irqrestore(&crdev->general,flags);

		if (!device_has_data(crdev)) {
			printk(KERN_WARNING "PORT HAS NOT DATA!!!\n");
			if (filp->f_flags & O_NONBLOCK)
				return -EAGAIN;

			/* Go to sleep until we have data. */
			ret = wait_event_interruptible(crdev->i_wq,
			                               device_has_data(crdev));

			if (ret < 0)
				goto free_buf;
		}

		/*lock the whole device*/
		spin_lock_irqsave(&crdev->general,flags);	

		ret = fill_readbuf(crdev, (char *)cr_data, sizeof(crypto_data));

		/*unlock the whole device*/
		spin_unlock_irqrestore(&crdev->general,flags);
		
		/* copy the response to userspace */
		/* ? */
		/*have nothing to copy*/
		debug("ended the session\n");	
		break;

	default:
		return -EINVAL;
	}

	return 0;

free_buf:
	kfree(cr_data);
	return ret;	
}
