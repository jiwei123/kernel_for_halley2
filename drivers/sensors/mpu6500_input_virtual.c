/*
	$License:
	Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
	$
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/pagemap.h>
#include <linux/wakelock.h>
#include <linux/hrtimer.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>

#include <linux/sensor/mpu6500_platformdata.h>
#include "./mpu6500_input_virtual.h"
#include "sensors_core.h"

#define DEBUG 0

#define MAX_GYRO	32767
#define MIN_GYRO	-32768

#define MPU6500_LOGTIME		10000
#define LOG_RESULT_LOCATION(x) {\
	printk(KERN_ERR "%s:%s:%d result=%d\n",__FILE__,__func__,__LINE__, x);\
}\

#define CHECK_RESULT(x) {\
		result = x;\
		if (unlikely(result)) \
			LOG_RESULT_LOCATION(result);\
	}

#define MPU6500_CALIB_FILENAME	"//data//mpu6500.cal"
#define MPU6500_ACCEL_CAL_PATH	"/efs/calibration_data"
#define MPU6500_GYRO_CAL_PATH	"/efs/gyro_cal_data"
#define MODULE_NAME_ACCEL	"accelerometer_sensor"
#define MODULE_NAME_GYRO	"gyro_sensor"

struct mpu6500_v {
	union {
		s16 v[3];
		struct {
			s16 x;
			s16 y;
			s16 z;
		};
	};
};

struct motion_int_data {
	unsigned char pwr_mnt[2];
	unsigned char cfg;
	unsigned char accel_cfg;
	unsigned char gyro_cfg;
	unsigned char int_cfg;
	unsigned char smplrt_div;
	bool is_set;
	unsigned char accel_cfg2;
};

struct mpu6500_input_data {
	struct i2c_client *client;
	struct input_dev *accel_input;
	struct input_dev *gyro_input;
	struct motion_int_data mot_data;
	struct mutex mutex;
	struct wake_lock reactive_wake_lock;
	atomic_t accel_enable;
	atomic_t accel_delay;

	atomic_t gyro_enable;
	atomic_t gyro_delay;
	atomic_t reactive_state;
	atomic_t reactive_enable;

	atomic_t motion_recg_enable;
	unsigned long motion_recg_st_time;

	unsigned char gyro_pwr_mgnt[2];
	unsigned char int_pin_cfg;

	u16 enabled_sensors;
	u16 sleep_sensors;
	u32 chip_pos;
	int current_delay;
	int irq;
	int count_logtime;
	int count_logtime_gyro;

	int gyro_bias[3];

	u8 mode;

	struct device *accel_sensor_device;
	struct device *gyro_sensor_device;
	s16 acc_cal[3];
	bool factory_mode;
};

static ssize_t mpu6500_input_accel_enable_show(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct mpu6500_input_data *data = input_get_drvdata(input_data);

	return sprintf(buf, "%d\n", atomic_read(&data->accel_enable));

}

static ssize_t mpu6500_input_accel_enable_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct mpu6500_input_data *data = input_get_drvdata(input_data);

	int value = simple_strtoul(buf, NULL, 10);

	atomic_set(&data->accel_enable, value);

	return count;
}

static ssize_t mpu6500_input_accel_delay_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct mpu6500_input_data *data = input_get_drvdata(input_data);

	return sprintf(buf, "%d\n", atomic_read(&data->accel_delay));

}

static ssize_t mpu6500_input_accel_delay_store(struct device *dev,
					       struct device_attribute *attr,
					       const char *buf, size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct mpu6500_input_data *data = input_get_drvdata(input_data);
	int value = 0;
	int ret = 0;

	ret = kstrtoint(buf, 10, &value);
	if (ret) {
		pr_err("[SENSOR]: %s - Invalid Argument\n", __func__);
		return ret;
	}

	pr_info("[SENSOR] %s : delay = %d\n", __func__, value);

	mutex_lock(&data->mutex);

	atomic_set(&data->accel_delay, value);

	mutex_unlock(&data->mutex);

	return count;
}

static ssize_t mpu6500_input_gyro_enable_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct mpu6500_input_data *data = input_get_drvdata(input_data);

	return sprintf(buf, "%d\n", atomic_read(&data->gyro_enable));

}

static ssize_t mpu6500_input_gyro_enable_store(struct device *dev,
					       struct device_attribute *attr,
					       const char *buf, size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct mpu6500_input_data *data = input_get_drvdata(input_data);

	int value = simple_strtoul(buf, NULL, 10);
	pr_info("[SENSOR] %s : enable = %d\n", __func__, value);

	mutex_lock(&data->mutex);
	atomic_set(&data->gyro_enable, value);
	mutex_unlock(&data->mutex);

	return count;
}

static ssize_t mpu6500_input_gyro_delay_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct mpu6500_input_data *data = input_get_drvdata(input_data);

	return sprintf(buf, "%d\n", atomic_read(&data->gyro_delay));

}

static ssize_t mpu6500_input_gyro_delay_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct mpu6500_input_data *data = input_get_drvdata(input_data);
	int value = 0;
	int ret = 0;

	ret = kstrtoint(buf, 10, &value);
	if (ret) {
		pr_err("[SENSOR]: %s - Invalid Argument\n", __func__);
		return ret;
	}

	pr_info("[SENSOR] %s : delay = %d\n", __func__, value);

	mutex_lock(&data->mutex);

	atomic_set(&data->gyro_delay, value);

	mutex_unlock(&data->mutex);

	return count;
}

static ssize_t mpu6500_input_motion_recg_enable_show(struct device *dev,
						     struct device_attribute
						     *attr, char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct mpu6500_input_data *data = input_get_drvdata(input_data);

	return sprintf(buf, "%d\n", atomic_read(&data->motion_recg_enable));
}

static ssize_t mpu6500_input_motion_recg_enable_store(struct device *dev,
						      struct device_attribute
						      *attr, const char *buf,
						      size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct mpu6500_input_data *data = input_get_drvdata(input_data);

	int value = simple_strtoul(buf, NULL, 10);

	mutex_lock(&data->mutex);

	atomic_set(&data->motion_recg_enable, value);

	mutex_unlock(&data->mutex);

	return count;
}

static ssize_t mpu6500_input_gyro_self_test_show(struct device *dev,
						 struct device_attribute *attr,
						 char *buf)
{
	return sprintf(buf, "%d "
		       "%d %d %d "
		       "%d.%03d %d.%03d %d.%03d "
		       "%d.%03d %d.%03d %d.%03d ",
		       0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1);
}

static struct device_attribute dev_attr_acc_enable =
	__ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
	mpu6500_input_accel_enable_show, mpu6500_input_accel_enable_store);
static struct device_attribute dev_attr_gyro_enable =
	__ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
	mpu6500_input_gyro_enable_show, mpu6500_input_gyro_enable_store);
static struct device_attribute dev_attr_acc_delay =
	__ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	mpu6500_input_accel_delay_show, mpu6500_input_accel_delay_store);
static struct device_attribute dev_attr_gyro_delay =
	__ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	mpu6500_input_gyro_delay_show, mpu6500_input_gyro_delay_store);
static DEVICE_ATTR(self_test, S_IRUGO, mpu6500_input_gyro_self_test_show, NULL);
static DEVICE_ATTR(mot_recg_enable, S_IRUGO | S_IWUSR | S_IWGRP,
		   mpu6500_input_motion_recg_enable_show,
		   mpu6500_input_motion_recg_enable_store);

static struct attribute *accel_attributes[] = {
	&dev_attr_acc_enable.attr,
	&dev_attr_acc_delay.attr,
	&dev_attr_mot_recg_enable.attr,
	NULL
};

static struct attribute *gyro_attributes[] = {
	&dev_attr_gyro_enable.attr,
	&dev_attr_gyro_delay.attr,
	&dev_attr_self_test.attr,
	NULL
};

static struct attribute_group accel_attribute_group = {
	.attrs = accel_attributes
};

static struct attribute_group gyro_attribute_group = {
	.attrs = gyro_attributes
};

static ssize_t mpu6500_input_reactive_enable_show(struct device *dev,
					struct device_attribute
						*attr, char *buf)
{
	return sprintf(buf, "%d\n", 1);
}

static ssize_t mpu6500_input_reactive_enable_store(struct device *dev,
					struct device_attribute
						*attr, const char *buf,
							size_t count)
{
	return count;
}

static struct device_attribute dev_attr_reactive_alert =
	__ATTR(reactive_alert, S_IRUGO | S_IWUSR | S_IWGRP,
		mpu6500_input_reactive_enable_show,
			mpu6500_input_reactive_enable_store);

static ssize_t mpu6500_power_on(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int count = 0;

	count = sprintf(buf, "%d\n",1);

	return count;
}

static struct device_attribute dev_attr_power_on =
	__ATTR(power_on, S_IRUSR | S_IRGRP, mpu6500_power_on,
		NULL);

static ssize_t mpu6500_get_temp(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	short temperature = 1;
	int count = 0;

	count = sprintf(buf, "%d\n", temperature);

	return count;
}

static struct device_attribute dev_attr_temperature =
	__ATTR(temperature, S_IRUSR | S_IRGRP, mpu6500_get_temp,
		NULL);

static ssize_t mpu6500_input_gyro_selftest_show(struct device *dev,
						 struct device_attribute *attr,
						 char *buf)
{
	return sprintf(buf, "%d,"
			"%d.%03d,%d.%03d,%d.%03d,"
			"%d.%03d,%d.%03d,%d.%03d,"
			"%d.%01d,%d.%01d,%d.%01d,"
			"%d.%03d,%d.%03d,%d.%03d\n",
			0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1);
}

static struct device_attribute dev_attr_selftest =
	__ATTR(selftest, S_IRUSR | S_IRGRP,
		mpu6500_input_gyro_selftest_show,
		NULL);

static ssize_t acc_data_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mpu6500_v acc;
	acc.x = 1;
	acc.y = 1;
	acc.z = 1;

	return sprintf(buf, "%d, %d, %d\n", acc.x, acc.y, acc.z);
}

static struct device_attribute dev_attr_raw_data =
	__ATTR(raw_data, S_IRUSR | S_IRGRP, acc_data_read,
		NULL);

static ssize_t accel_calibration_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct mpu6500_input_data *data = dev_get_drvdata(dev);
	int count = 0;
	s16 x, y, z;
	int err;

	x = data->acc_cal[0] = 1;
	y = data->acc_cal[1] = 1;
	z = data->acc_cal[2] = 1;
	pr_info("[SENSOR]: accel_calibration_show %d %d %d\n",
		x, y, z);

	if (!x && !y && !z)
		err = -1;
	else
		err = 1;

	count = sprintf(buf, "%d %d %d %d\n", err, x, y, z);

	return count;
}

static ssize_t accel_calibration_store(struct device *dev,
				struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct mpu6500_input_data *data = dev_get_drvdata(dev);
	int err = 0;
	int count;
	unsigned long enable;
	s16 x;
	s16 y;
	s16 z;
	char tmp[64];

	if (strict_strtoul(buf, 10, &enable))
		return -EINVAL;

	x = data->acc_cal[0];
	y = data->acc_cal[1];
	z = data->acc_cal[2];

	pr_info("[SENSOR]: accel_calibration_store %d %d %d\n",
		x, y, z);
	count = sprintf(tmp, "%d\n", err);

	return count;
}

static struct device_attribute dev_attr_calibration =
	__ATTR(calibration, S_IRUGO | S_IWUSR | S_IWGRP,
		accel_calibration_show, accel_calibration_store);

static ssize_t accel_vendor_show(struct device *dev,
				struct device_attribute *attr,
					char *buf)
{
	return sprintf(buf, "%s\n", "INVENSENSE");
}

static struct device_attribute dev_attr_accel_sensor_vendor =
	__ATTR(vendor, S_IRUSR | S_IRGRP, accel_vendor_show, NULL);
static struct device_attribute dev_attr_gyro_sensor_vendor =
	__ATTR(vendor, S_IRUSR | S_IRGRP, accel_vendor_show, NULL);

static ssize_t accel_name_show(struct device *dev,
				struct device_attribute *attr,
					char *buf)
{
	return sprintf(buf, "%s\n", "MPU6500");
}

static struct device_attribute dev_attr_accel_sensor_name =
	__ATTR(name, S_IRUSR | S_IRGRP, accel_name_show, NULL);
static struct device_attribute dev_attr_gyro_sensor_name =
	__ATTR(name, S_IRUSR | S_IRGRP, accel_name_show, NULL);

static struct device_attribute *gyro_sensor_attrs[] = {
	&dev_attr_power_on,
	&dev_attr_temperature,
	&dev_attr_gyro_sensor_vendor,
	&dev_attr_gyro_sensor_name,
	&dev_attr_selftest,
	NULL,
};

static struct device_attribute *accel_sensor_attrs[] = {
	&dev_attr_raw_data,
	&dev_attr_calibration,
	&dev_attr_reactive_alert,
	&dev_attr_accel_sensor_vendor,
	&dev_attr_accel_sensor_name,
	NULL,
};

static int mpu6500_accel_input_init(struct mpu6500_input_data *data)
{
	struct input_dev *idev;
	int error;

	idev = input_allocate_device();
	if (!idev)
		return -ENOMEM;

	data->accel_input = idev;
	idev->name = MODULE_NAME_ACCEL;
	idev->id.bustype = BUS_VIRTUAL;

	input_set_capability(data->accel_input, EV_REL, REL_X);
	input_set_capability(data->accel_input, EV_REL, REL_Y);
	input_set_capability(data->accel_input, EV_REL, REL_Z);
	input_set_drvdata(data->accel_input, data);

	error = input_register_device(data->accel_input);
	if (error) {
		input_free_device(data->accel_input);
		return error;
	}

	error = sensors_create_symlink(&data->accel_input->dev.kobj, idev->name);
	if (error < 0) {
		input_unregister_device(data->accel_input);
		return error;
	}

	error = sysfs_create_group(&idev->dev.kobj,
				&accel_attribute_group);
	if (error) {
		pr_err("%s: could not create sysfs group\n", __func__);
		sensors_remove_symlink(&data->accel_input->dev.kobj,
			data->accel_input->name);
		input_unregister_device(data->accel_input);
		return error;
	}

	atomic_set(&data->accel_enable, 0);
	atomic_set(&data->accel_delay, 10);
	atomic_set(&data->motion_recg_enable, 0);
	atomic_set(&data->reactive_state, 0);
	return 0;
}

static int mpu6500_gyro_input_init(struct mpu6500_input_data *data)
{
	struct input_dev *idev;
	int error;

	idev = input_allocate_device();
	if (!idev)
		return -ENOMEM;

	data->gyro_input = idev;
	idev->name = MODULE_NAME_GYRO;
	idev->id.bustype = BUS_I2C;

	input_set_capability(data->gyro_input, EV_REL, REL_RX);
	input_set_capability(data->gyro_input, EV_REL, REL_RY);
	input_set_capability(data->gyro_input, EV_REL, REL_RZ);
	input_set_drvdata(data->gyro_input, data);

	error = input_register_device(data->gyro_input);
	if (error) {
		input_free_device(data->gyro_input);
		return error;
	}

	error = sensors_create_symlink(&data->gyro_input->dev.kobj, idev->name);
	if (error < 0) {
		input_unregister_device(data->gyro_input);
		return error;
	}

	error = sysfs_create_group(&idev->dev.kobj,
				&gyro_attribute_group);
	if (error) {
		pr_err("%s: could not create sysfs group\n", __func__);
		sensors_remove_symlink(&data->gyro_input->dev.kobj,
			data->gyro_input->name);
		input_unregister_device(data->gyro_input);
		return error;
	}

	return 0;
}

static int  mpu6500_input_probe(struct platform_device *pdev)
{
	struct sensor_virtual_platform_data *pdata = pdev->dev.platform_data;
	struct mpu6500_input_data *data = NULL;
	int error = 0;

	if (!pdata)
		return -EBUSY;

	pr_info("[SENSOR] %s: IN\n", __func__);

	data = kzalloc(sizeof(struct mpu6500_input_data), GFP_KERNEL);
	if (!data)
		goto err_kzalloc;

	platform_set_drvdata(pdev, data);

	mutex_init(&data->mutex);

	error = mpu6500_accel_input_init(data);
	if (error)
		goto err_accel_input_init;

	error = mpu6500_gyro_input_init(data);
	if (error)
		goto err_gyro_input_init;

	error = sensors_register(data->accel_sensor_device,
		data, accel_sensor_attrs,
			"accelerometer_sensor");
	if (error) {
		pr_err("[SENSOR] %s: cound not register\
			accelerometer sensor device(%d).\n",
			__func__, error);
		goto acc_sensor_register_failed;
	}

	error = sensors_register(data->gyro_sensor_device,
		data, gyro_sensor_attrs,
			"gyro_sensor");
	if (error) {
		pr_err("[SENSOR] %s: cound not register\
			gyro sensor device(%d).\n",
			__func__, error);
		goto gyro_sensor_register_failed;
	}

	return 0;

gyro_sensor_register_failed:
	sensors_unregister(data->accel_sensor_device, accel_sensor_attrs);
acc_sensor_register_failed:
	sensors_remove_symlink(&data->gyro_input->dev.kobj,
			data->gyro_input->name);
	input_unregister_device(data->gyro_input);
err_gyro_input_init:
	sensors_remove_symlink(&data->accel_input->dev.kobj,
			data->accel_input->name);
	input_unregister_device(data->accel_input);
err_accel_input_init:
	mutex_destroy(&data->mutex);
	wake_lock_destroy(&data->reactive_wake_lock);
	kfree(data);
err_kzalloc:
	pr_err("[SENSOR] %s: - Probe fail!\n", __func__);
	return error;
}

static int mpu6500_input_remove(struct platform_device *pdev)
{
	struct mpu6500_input_data *data = platform_get_drvdata(pdev);

	if (data == NULL)
		return 0;

	input_unregister_device(data->accel_input);
	input_unregister_device(data->gyro_input);

	kfree(data);

	return 0;
}

static struct platform_driver mpu6500_virtual_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "sensor_virtual",
		   },
	.probe = mpu6500_input_probe,
	.remove = mpu6500_input_remove,
};

struct sensor_virtual sensor_virtual = {
	.name = "accelerometer_sensor",
};

struct sensor_virtual_platform_data sensor_virtual_platform_data = {
	.num_sensor = 1,
	.sensor_virtual = &sensor_virtual,
};

struct platform_device mpu6500_virtual_device = {
        .name   = "sensor_virtual",
        .id     = 0,
        .dev    = {
                .platform_data  = &sensor_virtual_platform_data,
        },
};

static int __init sensor_virtual_init(void)
{
	platform_device_register(&mpu6500_virtual_device);
	platform_driver_register(&mpu6500_virtual_driver);
	return 0;
}

static void __exit sensor_virtual_cleanup(void)
{
	platform_driver_unregister(&mpu6500_virtual_driver);
	platform_device_unregister(&mpu6500_virtual_device);
}

module_init(sensor_virtual_init);
module_exit(sensor_virtual_cleanup);

MODULE_AUTHOR("Tae-Soo Kim <tskim@invensense.com>");
MODULE_DESCRIPTION("MPU6500 virtual driver");
MODULE_LICENSE("GPL");
