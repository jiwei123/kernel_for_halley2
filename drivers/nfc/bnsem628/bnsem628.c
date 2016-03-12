/*
 * drivers/nfc/bnsem628/bnsem628.c
 *
 *	add by haijie.hong@ingenic.com
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
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/poll.h>
#include <linux/version.h>
#include <linux/nfc/bnsem628.h>

struct bnsem628_dev {
	wait_queue_head_t read_wq;
	struct mutex read_mutex;
	struct i2c_client *client;
	struct miscdevice bnsem628_device;
	/*ENVDDSE 使能脚*/
	unsigned int en_gpio;
};


/*
* 读取nfc芯片数据
* 1、读出可读数据的大小（2个字节表示）
* 2、根据 1 读出的大小，读取芯片返回的数据
*/
int bnsem628_i2c_Read(struct i2c_client *client, unsigned char *readbuf)
{
	int ret;
	int readlen = 0;
	unsigned char lenbuf[2];

	/*receive lenth*/
	struct i2c_msg msgs1[] = {
		{
		 .addr = client->addr,
		 .flags = 1,
		 .len = 2,
		 .buf = lenbuf,
		 },
	};

	ret = i2c_transfer(client->adapter, msgs1, 1);
	if (ret < 0){
		dev_err(&client->dev, "%s: i2c read lenth error. %d\n", __func__, ret);
		return ret;
	}

	/*set read lenth*/
	readlen = (lenbuf[0] << 8) + lenbuf[1];
	/*receive data*/
	struct i2c_msg msgs2[] = {
		{
			.addr = client->addr,
			.flags = 1,
			.len = readlen,
			.buf = readbuf,
		},
	};

	ret = i2c_transfer(client->adapter, msgs2, 1);
	if (ret < 0){
		dev_err(&client->dev, "%s: i2c read data error. %d\n", __func__, ret);
		return ret;
	}

	return readlen;
}

/*
* 向nfc芯片发送数据
* 1、前2个字节表示要发送的数据长度
* 2、接下来的数据是真正要发送的数据
*/
int bnsem628_i2c_Write(struct i2c_client *client, unsigned char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = writelen,
			.buf = writebuf,
		 },
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0){
		dev_err(&client->dev, "%s i2c write error %d.\n", __func__, ret);
		return ret;
	}

	return writelen;
}

/*
* ENVDDSE GPIO控制
* mode：1 ----> 接触模式
* mode：0 ----> 非接模式
*/
int bnsem628_set_mode(struct bnsem628_dev *bnsem628_dev, int mode)
{
	int ret;

	if(mode == BNSEM628_MODE_CONNECT){
		ret = gpio_direction_output(bnsem628_dev->en_gpio, 1);
		if (ret < 0) {
			return ret;
		}
	}else if(mode == BNSEM628_MODE_UNCONNECT){
		ret = gpio_direction_output(bnsem628_dev->en_gpio, 0);
		if (ret < 0) {
			return ret;
		}
	}

	return ret;
}

static int bnsem628_dev_open(struct inode *inode, struct file *filp)
{
	int ret = 0;

	struct bnsem628_dev *bnsem628_dev = container_of(filp->private_data,
							 struct bnsem628_dev,
							 bnsem628_device);
	filp->private_data = bnsem628_dev;

	printk("%s device node major=%d, minor=%d, flags=%x, address=%x\n",
		__func__, imajor(inode), iminor(inode),
		bnsem628_dev->client->flags, bnsem628_dev->client->addr);

	/*开启接触模式*/
	ret = bnsem628_set_mode(bnsem628_dev, BNSEM628_MODE_CONNECT);

	return ret;
}

int bnsem628_dev_release(struct inode *inode, struct file *filp)
{
	struct bnsem628_dev *bnsem628_dev = filp->private_data;

	/*关闭接触模式*/
	bnsem628_set_mode(bnsem628_dev, BNSEM628_MODE_UNCONNECT);

	return 0;
}

static ssize_t bnsem628_dev_write(struct file *filp, const char __user * buf,
				  size_t count, loff_t * offset)
{
	struct bnsem628_dev *bnsem628_dev = filp->private_data;
	char tmp[MAX_BUFFER_SIZE+2];
	int ret;

	if (count > MAX_BUFFER_SIZE) {
		printk("%s out of memory\n", __func__);
		return -ENOMEM;
	}

	/*2 bytes of lenth*/
	tmp[0] = ((count >> 8) & 0xff);
	tmp[1] = (count & 0xff);

	if (copy_from_user(&tmp[2], buf, count)) {
		printk("%s failed to copy from user space\n", __func__);
		return -EFAULT;
	}

	mutex_lock(&bnsem628_dev->read_mutex);
	/* send lenth and data */
	ret = bnsem628_i2c_Write(bnsem628_dev->client, tmp, count+2);
	if (ret != count+2) {
		printk("%s failed to write %d\n", __func__, ret);
		ret = -EIO;
	}
	mutex_unlock(&bnsem628_dev->read_mutex);

	return ret;
}

static ssize_t bnsem628_dev_read(struct file *filp, char __user * buf,
				 size_t count, loff_t * offset)
{
	struct bnsem628_dev *bnsem628_dev = filp->private_data;
	unsigned char tmp[MAX_BUFFER_SIZE];
	int total;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	mutex_lock(&bnsem628_dev->read_mutex);
	/* Receive data */
	total = bnsem628_i2c_Read(bnsem628_dev->client, tmp);
	if(total < 0){
		printk("%s failed to read %d\n", __func__, total);
		total = -EIO;
	}
	mutex_unlock(&bnsem628_dev->read_mutex);

	if(total > count ||copy_to_user(buf, tmp, total)){
		printk("%s failed to copy to user space\n", __func__);
		total = -EFAULT;
	}

	return total;
}

static long bnsem628_dev_ioctl(struct file *filp,unsigned int cmd, unsigned long arg)
{
	struct bnsem628_dev *bnsem628_dev = filp->private_data;
	int ret = 0;

	switch(cmd){
	/*打开电源*/
	case CMD_NFC_BNSEM628_POWER_ON:
		printk("CMD_NFC_BNSEM628_POWER_ON\n");
		break;
	/*关闭电源*/
	case CMD_NFC_BNSEM628_POWER_OFF:
		printk("CMD_NFC_BNSEM628_POWER_OFF\n");
		break;
	/*设置为接触模式*/
	case CMD_NFC_BNSEM628_MODE_CONNECT:
		ret = bnsem628_set_mode(bnsem628_dev, BNSEM628_MODE_CONNECT);
		printk("CMD_NFC_BNSEM628_MODE_CONNECT\n");
		break;
	/*设置为非接模式*/
	case CMD_NFC_BNSEM628_MODE_UNCONNECT:
		ret = bnsem628_set_mode(bnsem628_dev, BNSEM628_MODE_UNCONNECT);
		printk("CMD_NFC_BNSEM628_MODE_UNCONNECT\n");
		break;
	default:
		break;
	}

	return ret;
}

static const struct file_operations bnsem628_dev_fops = {
	.owner = THIS_MODULE,
	.read = bnsem628_dev_read,
	.write = bnsem628_dev_write,
	.open = bnsem628_dev_open,
	.release = bnsem628_dev_release,
	.unlocked_ioctl = bnsem628_dev_ioctl
};


static int bnsem628_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	int ret;
	struct bnsem628_platform_data *platform_data;
	struct bnsem628_dev *bnsem628_dev;

	platform_data = client->dev.platform_data;
	printk("%s, probing bnsem628 driver flags = %x, name= %s, address=%x\n",
		 __func__, client->flags, client->name, client->addr);

	if (platform_data == NULL) {
		printk( "%s nfc probe fail\n", __func__);
		return -ENODEV;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk("%s need I2C_FUNC_I2C\n", __func__);
		return -ENODEV;
	}

	bnsem628_dev = kzalloc(sizeof(*bnsem628_dev), GFP_KERNEL);
	if (bnsem628_dev == NULL) {
		printk("%s failed to allocate memory for module data\n", __func__);
		return -ENOMEM;
	}

	bnsem628_dev->en_gpio = platform_data->en_gpio;
	bnsem628_dev->client = client;

	ret = gpio_request(platform_data->en_gpio, "nfc_ven");
	if (ret) {
		printk("%s failed to request gpio.\n", __func__);
		goto gpio_err;
	}

	/*启动时关闭接触模式*/
	ret = bnsem628_set_mode(bnsem628_dev, BNSEM628_MODE_UNCONNECT);
	if(ret < 0){
		printk("%s failed to set nfc unconnected\n", __func__);
		goto mode_err;
	}

	/* init mutex */
	mutex_init(&bnsem628_dev->read_mutex);

	bnsem628_dev->bnsem628_device.minor = MISC_DYNAMIC_MINOR;
	bnsem628_dev->bnsem628_device.name = BNSEM628_NAME;
	bnsem628_dev->bnsem628_device.fops = &bnsem628_dev_fops;

	ret = misc_register(&bnsem628_dev->bnsem628_device);
	if (ret) {
		printk( "%s misc_register failed\n", __func__);
		goto err_misc_register;
	}

	i2c_set_clientdata(client, bnsem628_dev);

	printk("%s, probing bnsem628 driver exited successfully\n", __func__);
	return 0;

err_misc_register:
	mutex_destroy(&bnsem628_dev->read_mutex);
mode_err:
	gpio_free(platform_data->en_gpio);
gpio_err:
	kfree(bnsem628_dev);

	return ret;
}


static int bnsem628_remove(struct i2c_client *client)
{
	struct bnsem628_dev *bnsem628_dev;

	bnsem628_dev = i2c_get_clientdata(client);
	misc_deregister(&bnsem628_dev->bnsem628_device);
	mutex_destroy(&bnsem628_dev->read_mutex);
	gpio_free(bnsem628_dev->en_gpio);
	kfree(bnsem628_dev);

	return 0;
}

static const struct i2c_device_id bnsem628_id[] = {
	{BNSEM628_NAME, 0},
	{}
};


static struct i2c_driver bnsem628_driver = {
	.id_table = bnsem628_id,
	.probe = bnsem628_probe,
	.remove = bnsem628_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = BNSEM628_NAME,
	},
};

/*
 * module load/unload record keeping
 */

static int __init bnsem628_dev_init(void)
{
	return i2c_add_driver(&bnsem628_driver);
}

module_init(bnsem628_dev_init);

static void __exit bnsem628_dev_exit(void)
{
	i2c_del_driver(&bnsem628_driver);
}

module_exit(bnsem628_dev_exit);

MODULE_AUTHOR("SzIngenic");
MODULE_DESCRIPTION("NFC bnsem628 driver");
MODULE_LICENSE("GPL");
