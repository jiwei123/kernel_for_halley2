#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <uapi/linux/input.h>
#include <linux/sensor_hub.h>

#include "sensor_hub.h"


int sensor_hub_i2c_read(struct device *dev, void *buf, size_t size)
{

	int ret = 0;
	struct i2c_client *client = to_i2c_client(dev);

	if (!buf) {
		dev_err(dev, "%s buf is NULL.\n", __func__);
		return -1;
	}

	ret = i2c_master_recv(client, buf, size);

	if(ret < 0)
	{
		return ret;
	}

	return 0;
}

int sensor_hub_i2c_write(struct device *dev, void *buf, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret = 0;

	if (!buf) {
		dev_err(dev, "%s buf is NULL.\n", __func__);
		return -1;
	}

	ret = i2c_master_send(client, buf, size);

	if(ret < 0)
	{
		return ret;
	}

	return 0;
}

static struct sensor_hub_bus_ops sensor_hub_i2c_bus_ops = {
	.bustype = BUS_I2C,
	.read    = sensor_hub_i2c_read,
	.write   = sensor_hub_i2c_write,
};


#ifdef CONFIG_PM
static int sensor_hub_i2c_suspend(struct device *dev)
{
	printk("---stm32_i2c_suspend");

	return 0;
}

static int sensor_hub_i2c_resume(struct device *dev)
{
	printk("---stm32_i2c_resume");

	return 0;
}
#else
#define sensor_hub_i2c_suspend	NULL
#define sensor_hub_i2c_resume	NULL
#endif

static int sensor_hub_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct sensor_hub_platform_data *pdata = (struct sensor_hub_platform_data *)client->dev.platform_data;
	int err = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "check functionality failed. %s %d\n", __func__, __LINE__);
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	err = sensor_hub_core_init(&sensor_hub_i2c_bus_ops, &client->dev);
	if (err < 0) {
		dev_err(&client->dev, "sensor_hub_core_init failed. %s %d\n", __func__, __LINE__);
	}
	return 0;

exit_check_functionality_failed:
	return err;
}


static int sensor_hub_i2c_remove(struct i2c_client *client)
{
	dev_err(&client->dev, "sensor_hub_remove.\n");

	struct sensor_hub_data *s_data = i2c_get_clientdata(client);
	sensor_hub_core_release(s_data->dev);

	return 0;
}


static const struct i2c_device_id sensor_hub_i2c_id[] = {
		{DEVICE_NAME, 0},
		{}
};
MODULE_DEVICE_TABLE(i2c, sensor_hub_id);


static struct i2c_driver sensor_hub_driver = {
	.probe = sensor_hub_i2c_probe,
	.remove = sensor_hub_i2c_remove,
    .suspend  = sensor_hub_i2c_suspend,
    .resume   = sensor_hub_i2c_resume,
	.id_table = sensor_hub_i2c_id,
	.driver = {
			.name = DEVICE_NAME,
			.owner = THIS_MODULE,
	},
};

static int __init sensor_hub_i2c_init(void)
{
	int ret;

	ret = i2c_add_driver(&sensor_hub_driver);
	return ret;
}

static void __exit sensor_hub_i2c_exit(void)
{
	i2c_del_driver(&sensor_hub_driver);
}

module_init(sensor_hub_i2c_init);
module_exit(sensor_hub_i2c_exit);

MODULE_AUTHOR("<Ingenic>");
MODULE_DESCRIPTION("Sensor Hub driver");
MODULE_LICENSE("GPL");
