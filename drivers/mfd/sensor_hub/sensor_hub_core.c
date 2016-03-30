#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <soc/gpio.h>
#include <linux/fs.h>
#include <linux/kthread.h>
#include <asm/uaccess.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>


#include <linux/sensor_hub.h>

#include "sensor_hub.h"
#include "pack_buff.h"


#define SENSOR_HUB_VERSION           "1.0"

#define SENSOR_HUB_BUST_RECV          0x01
#define SENSOR_HUB_BUST_TX            0x02
#define SENSOR_HUB_RX_BUFF_EMPTY      0x03
#define SENSOR_HUB_TX_BUFF_EMPTY      0x04
#define SENSOR_HUB_TX_BUFF_FULL       0x05

static volatile unsigned long        g_flags;


static int sensor_hub_reset(sensor_hub_data_t *s_data, unsigned int level)
{
	struct device *dev = s_data->dev;
	int pin_level = !!level;

	gpio_direction_output(s_data->pdata->gpio_reset, pin_level);
	mdelay(5);
	gpio_direction_output(s_data->pdata->gpio_reset, !pin_level);
	mdelay(10);

	return 0;
}

static void wait_sensor_hub_ack(sensor_hub_data_t *s_data)
{
	int ret = 0;

	do {
		ret = gpio_get_value(s_data->pdata->gpio_irq);

	} while(ret == 0);
}

void wait_sensor_hub_end(sensor_hub_data_t *s_data)
{
	int ret = 0;

	do {
		ret = gpio_get_value(s_data->pdata->gpio_irq);

	} while(ret == 1);

}

static void sensor_hub_set_state(sensor_hub_data_t *s_data, int mode)
{
	gpio_direction_output(s_data->pdata->gpio_wakeup_sensorhub, mode);
}

static int sensor_hub_get_state(sensor_hub_data_t *s_data)
{
	int ret = 0;

	ret = gpio_get_value(s_data->pdata->gpio_wakeup_host);

	return 	ret;
}

static int sensor_hub_gpio_init(sensor_hub_data_t *s_data)
{
	struct device *dev = s_data->dev;
	int err;

	err = gpio_request(s_data->pdata->gpio_reset, "sensor_hub reset");
	if (err < 0) {
		dev_err(dev, "%s failed to request gpio reset.\n", __func__);
		return err;
	}

	err = gpio_request(s_data->pdata->gpio_wakeup_sensorhub, "sensor_hub wake up");
	if (err < 0) {
		dev_err(dev, "%s failed to request gpio response for boot mode.\n", __func__);
		gpio_free(s_data->pdata->gpio_reset);
		return err;
	}

	err = gpio_request(s_data->pdata->gpio_irq, "sensor_hub irq");
	if (err < 0) {
		dev_err(dev, "%s failed to request gpio irq.\n", __func__);
		gpio_free(s_data->pdata->gpio_reset);
		gpio_free(s_data->pdata->gpio_wakeup_sensorhub);
		return err;
	}
	s_data->irq = gpio_to_irq(s_data->pdata->gpio_irq );

	gpio_direction_input(s_data->pdata->gpio_irq);

	return 0;
}

static int sensor_hub_gpio_free(sensor_hub_data_t *s_data)
{
	gpio_free(s_data->pdata->gpio_irq);
	gpio_free(s_data->pdata->gpio_reset);
	gpio_free(s_data->pdata->gpio_wakeup_sensorhub);

	return 0;
}

void sync(sensor_hub_data_t *s_data)
{
	int ret = 0;

	do {
		ret = gpio_get_value(s_data->pdata->gpio_irq);

	} while(ret == 1);

}

static void sensor_hub_work_read_handler(struct work_struct *work)
{
	int ret = 0;
	char temp_buff[EACH_BLOCK_SIZE] = {0};

	sensor_hub_data_t *s_data = container_of(work, struct sensor_hub_data, work_read);
	struct device *dev = s_data->dev;

	msleep(10);
	sensor_hub_set_state(s_data, 1);

	if( if_buff_is_full(&s_data->rx_buff, 1) == 0 )
	{
		printk("rx_buff have no space to recv pack\n");
		goto err;
	}

	msleep(30);

	if (down_interruptible(&s_data->sem)) {
		dev_err(dev, "%s get sem failed .\n", __func__);
		goto err;
	}

	ret = sensor_hub_read(s_data, temp_buff , EACH_BLOCK_SIZE);
	if(ret < 0)
	{
		up(&s_data->sem);
		goto err;
	}

	up(&s_data->sem);

	spin_lock(&s_data->lock);
	write_block_to_buff(temp_buff, 1, &s_data->rx_buff);
	spin_unlock(&s_data->lock);

	spin_lock(&s_data->lock);
	if( if_buff_hold_enough_block(&s_data->rx_buff, MAX_PACK_BLOCK) == 0 )
	{
		spin_unlock(&s_data->lock);
		clear_bit(SENSOR_HUB_RX_BUFF_EMPTY, &g_flags);

	} else {
		spin_unlock(&s_data->lock);
		set_bit(SENSOR_HUB_RX_BUFF_EMPTY, &g_flags);

	}

	clear_bit(SENSOR_HUB_BUST_RECV, &g_flags);

	sensor_hub_set_state(s_data, 0);
	sync(s_data);

	wake_up_interruptible(&s_data->tx_waiter);
	wake_up_interruptible(&s_data->rx_waiter);

	return ;

err:
	clear_bit(SENSOR_HUB_BUST_RECV, &g_flags);
	sensor_hub_set_state(s_data, 0);
	sync(s_data);
	return ;
}

static void sensor_hub_flush_buff(void *arg)
{
	int ret = 0;
	char temp_buff[EACH_BLOCK_SIZE];
	sensor_hub_data_t *s_data = (sensor_hub_data_t *)arg;
	struct device *dev = s_data->dev;

	while(1)
	{
		spin_lock(&s_data->lock);
		if ( if_buff_is_empty(&s_data->tx_buff) == 0)
		 {
			spin_unlock(&s_data->lock);
			while( !wait_event_interruptible(s_data->tx_waiter, !test_bit(SENSOR_HUB_TX_BUFF_EMPTY, &g_flags)))
			{
				spin_lock(&s_data->lock);
				if( if_buff_is_empty(&s_data->tx_buff) < 0)
				{
					spin_unlock(&s_data->lock);
					break;
				}
				spin_unlock(&s_data->lock);
			}
		 } else {
			 spin_unlock(&s_data->lock);

		 }

		while(!wait_event_interruptible(s_data->tx_waiter, !test_bit(SENSOR_HUB_BUST_RECV, &g_flags)))
		{
			if(sensor_hub_get_state(s_data) == 0)
			{
				break;
			}
		}

		sensor_hub_set_state(s_data, 1);
		set_bit(SENSOR_HUB_BUST_TX, &g_flags);
		disable_irq(s_data->irq);

		spin_lock(&s_data->lock);
		read_block_from_buff(temp_buff, 1, &s_data->tx_buff);
		spin_unlock(&s_data->lock);

		wait_sensor_hub_ack(s_data);
		msleep(10);

		if (down_interruptible(&s_data->sem)) {
			dev_err(dev, "%s get sem failed .\n", __func__);
			goto final;
		}

		ret = sensor_hub_write(s_data, temp_buff, EACH_BLOCK_SIZE);
		if(ret < 0)
		{
			up(&s_data->sem);
			wait_sensor_hub_end(s_data);
			goto final;
		}

		up(&s_data->sem);

		spin_lock(&s_data->lock);
		cut_down_block_count(&s_data->tx_buff, 1);
		spin_unlock(&s_data->lock);

		spin_lock(&s_data->lock);
		if(if_buff_is_empty(&s_data->tx_buff) == 0)
		{
			set_bit(SENSOR_HUB_TX_BUFF_EMPTY, &g_flags);

		} else {
			if(if_buff_is_full(&s_data->tx_buff, MAX_PACK_BLOCK) == 0)
			{
				set_bit(SENSOR_HUB_TX_BUFF_FULL, &g_flags);
			}

		}

		spin_unlock(&s_data->lock);
final:
		wait_sensor_hub_end(s_data);
		enable_irq(s_data->irq);
		sensor_hub_set_state(s_data, 0);
		clear_bit(SENSOR_HUB_BUST_TX, &g_flags);
		msleep(10);

	}

	return;
}

static irqreturn_t sensor_hub_interrupt(int irq, void *dev_id)
{
	sensor_hub_data_t *s_data = dev_id;
	struct device *dev = &s_data->dev;

	set_bit(SENSOR_HUB_BUST_RECV, &g_flags);

    if(work_pending(&s_data->work_read) == 0) {
        return queue_work(s_data->workqueue, &s_data->work_read);
    }

	return IRQ_HANDLED;
}

static int sensor_hub_core_open(struct inode *inode, struct file *filp)
{

	sensor_hub_data_t *s_data = container_of(inode->i_cdev, struct sensor_hub_data, cdev);
	filp->private_data = s_data;


	return 0;
}

static int sensor_hub_core_close(struct inode *inode, struct file *filp)
{

	return 0;
}

static ssize_t sensor_hub_core_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	int retval = 0, length = 0 , blocks = 0;
	sensor_hub_data_t *s_data = filp->private_data;
	sensor_hub_packet_t *packet;
	char temp_buff[MAX_PACK_SIZE];

	if(buf == NULL || count > MAX_PACK_SIZE)
	{
		printk("sensor_hub_core_read err param!\n");
		return -1;
	}

	spin_lock(&s_data->lock);
	if ( if_buff_hold_enough_block(&s_data->rx_buff, MAX_PACK_BLOCK) < 0 )
	{
		spin_unlock(&s_data->lock);
		while(!wait_event_interruptible(s_data->rx_waiter, !test_bit(SENSOR_HUB_RX_BUFF_EMPTY, &g_flags)) )
		{
			spin_lock(&s_data->lock);
			if( if_buff_hold_enough_block(&s_data->rx_buff, MAX_PACK_BLOCK) == 0 )
			{
				spin_unlock(&s_data->lock);
				break;
			}
			set_bit(SENSOR_HUB_RX_BUFF_EMPTY, &g_flags);
			spin_unlock(&s_data->lock);
		}
	} else {
		spin_unlock(&s_data->lock);
	}


	spin_lock(&s_data->lock);
	blocks = read_pack_from_buff(temp_buff, MAX_PACK_SIZE, &s_data->rx_buff );
	cut_down_block_count(&s_data->rx_buff, blocks);
	spin_unlock(&s_data->lock);

	length = ( (hub_format_header_t *)temp_buff )->length + HEAD_SIZE;

	copy_to_user(buf, temp_buff, length);

	return length;
}

static ssize_t sensor_hub_core_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
	int ret = 0;
	sensor_hub_data_t *s_data = filp->private_data;
	struct device *dev = s_data->dev;

	char temp_buff[MAX_PACK_SIZE] = {0};

	if( buf == NULL || count > MAX_PACK_SIZE )
	{
		printk("sensor_hub_core_write err param!\n");
		return -1;
	}
	spin_lock(&s_data->lock);
	if(if_buff_is_full(&s_data->tx_buff, MAX_PACK_BLOCK) == 0)
	{
		spin_unlock(&s_data->lock);

		while(!wait_event_interruptible(s_data->rx_waiter, !test_bit(SENSOR_HUB_TX_BUFF_FULL, &g_flags)))
		{
			spin_lock(&s_data->lock);
			if( if_buff_is_full(&s_data->tx_buff, MAX_PACK_BLOCK) < 0 )
			{
				spin_unlock(&s_data->lock);
				break;
			}
			spin_unlock(&s_data->lock);
			set_bit(SENSOR_HUB_TX_BUFF_FULL, &g_flags);
		}
	} else {
		spin_unlock(&s_data->lock);
	}

	copy_from_user(temp_buff, buf, count);
	spin_lock(&s_data->lock);
	ret = write_pack_to_buff(temp_buff, count, &s_data->tx_buff);
	spin_unlock(&s_data->lock);
	if(ret < 0)
		return ret;

	clear_bit(SENSOR_HUB_TX_BUFF_EMPTY, &g_flags);
	wake_up_interruptible(&s_data->tx_waiter);

	return count;
}

static long sensor_hub_core_ioctl(struct file *filp, unsigned int cmd, unsigned long args)
{
	sensor_hub_data_t *s_data = filp->private_data;

	switch(cmd) {
		case 1:
			printk("sensor_hub_core_ioctl\n");
			break;
		default:
			break;
	}

	return 0;
}

static struct file_operations sensor_hub_ops = {
	.owner          = THIS_MODULE,
	.open           = sensor_hub_core_open,
	.release        = sensor_hub_core_close,
	.read           = sensor_hub_core_read,
	.write          = sensor_hub_core_write,
	.unlocked_ioctl = sensor_hub_core_ioctl,

};

static int sensor_hub_init_cdev(sensor_hub_data_t *s_data, struct file_operations *s_ops)
{
	int err;

	s_data->devt = MKDEV(0, 0);
	err = alloc_chrdev_region(&s_data->devt, 0, DEVICE_NUM, DEVICE_NAME);

	if (err < 0) {
		printk(KERN_ERR "sensor_hub:alloc_chrdev_region() failed. err=%d \n", err);
		goto error_alloc_chrdev;
	}

	s_data->class = class_create(THIS_MODULE, DEVICE_NAME);
    if (IS_ERR(s_data->class)) {
		printk(KERN_ERR "sensor_hub:class_create failed\n");
		err = -1;
		goto error_create_cls;
	}

	if (!(device_create(s_data->class, NULL, s_data->devt, NULL, DEVICE_NAME))) {
		printk(KERN_ERR" sensor_hub: create device failed\n");
		err = -1;
		goto error_create_device;
	}

	cdev_init(&s_data->cdev, s_ops);
	s_data->cdev.owner = THIS_MODULE;
	err = cdev_add(&s_data->cdev, s_data->devt, DEVICE_NUM);
	if (err) {
		printk(KERN_ERR "sensor_hub:cdev_add() failed. err=%d\n", err);
		err = -1;
		goto error_add_cdev;
	}

	return 0;

error_add_cdev:
	device_destroy(s_data->class, s_data->devt);
error_create_device:
	class_destroy(s_data->class);
error_create_cls:
	unregister_chrdev_region(s_data->devt, DEVICE_NUM);
error_alloc_chrdev:
	return err;
}

int sensor_hub_core_init(const struct sensor_hub_bus_ops *bus_ops, struct device *dev)
{
	sensor_hub_data_t *s_data;
	struct sensor_hub_platform_data *pdata = dev_get_platdata(dev);
	int err;

	if (!pdata) {
		printk("sensor_hub data platform_data is NULL\n");
		err = -1;
		goto error_no_pdata;
	}

	s_data = kzalloc(sizeof(struct sensor_hub_data), GFP_KERNEL);
	if (!s_data) {
		dev_err(dev, "failed to allocate memory for sensor_hub data\n");
		err = -ENOMEM;
		goto error_alloc_data;
	}

	err = sensor_hub_init_cdev(s_data, &sensor_hub_ops);
	if (err < 0) {
		printk("sensor_hub:init_cdev failed\n");
		err = -1;
		goto error_init_cdev;
	}

	s_data->dev      = dev;
	s_data->pdata    = pdata;
	s_data->bus_ops  = bus_ops;

	dev_set_drvdata(dev, s_data);


	mutex_init(&s_data->lock);
	sema_init(&s_data->sem, 1);
	init_waitqueue_head(&s_data->rx_waiter);
	init_waitqueue_head(&s_data->tx_waiter);

	err = sensor_hub_gpio_init(s_data);
	if(err < 0){
		dev_err(dev, "failed to request gpio\n");
		goto error_gpio_init;
	}

	sensor_hub_reset(s_data, BOOT_MODE_FLASH);

	INIT_WORK(&s_data->work_read, sensor_hub_work_read_handler);

	s_data->workqueue = create_singlethread_workqueue("sensor_hub_wq");
	s_data->workqueue = create_workqueue("sensor_hub_wq");
	if (!s_data->workqueue) {
		dev_err(dev, "sensor_hub:create singlethread workqueue failed\n");
		goto error_create_wq;
	}

	err = request_irq(s_data->irq, sensor_hub_interrupt, IRQF_TRIGGER_RISING | IRQF_DISABLED,
			DEVICE_NAME, s_data);
	if (err < 0) {
		dev_err(dev, "sensor_hub:request irq failed\n");
		goto error_irq_request;
	}

	s_data->my_thread = kthread_run(sensor_hub_flush_buff, s_data, "sensor_hub_flush_buff");
	if (IS_ERR(s_data->my_thread)) {
		printk("error create thread_name thread");
		goto err_kthread_run;
	}

	memset(&s_data->rx_buff, 0, sizeof(BUFF_T));
	memset(&s_data->tx_buff, 0, sizeof(BUFF_T));

	set_bit(SENSOR_HUB_RX_BUFF_EMPTY, &g_flags);
	set_bit(SENSOR_HUB_TX_BUFF_EMPTY, &g_flags);

	clear_bit(SENSOR_HUB_TX_BUFF_FULL, &g_flags);
	clear_bit(SENSOR_HUB_BUST_RECV, &g_flags);
	clear_bit(SENSOR_HUB_BUST_TX, &g_flags);

	return 0;

err_kthread_run:
error_irq_request:
	destroy_workqueue(s_data->workqueue);
error_create_wq:
	sensor_hub_gpio_free(s_data);
error_gpio_init:
	cdev_del(&s_data->cdev);
	device_destroy(s_data->class, s_data->devt);
	class_destroy(s_data->class);
	unregister_chrdev_region(s_data->devt, DEVICE_NUM);
	dev_set_drvdata(dev, NULL);
error_init_cdev:
	kfree(s_data);
error_alloc_data:
error_no_pdata:
	return err;
}


int sensor_hub_core_release(struct device *dev)
{
	sensor_hub_data_t *s_data = dev_get_drvdata(dev);

	if (s_data != NULL) {
		flush_workqueue(s_data->workqueue);
		destroy_workqueue(s_data->workqueue);

		free_irq(s_data->irq, s_data);
		sensor_hub_gpio_free(s_data);
		cdev_del(&s_data->cdev);
		device_destroy(s_data->class, s_data->devt);
		class_destroy(s_data->class);
		unregister_chrdev_region(s_data->devt, DEVICE_NUM);
		dev_set_drvdata(dev, NULL);
		kfree(s_data);
	}

	return 0;
}
