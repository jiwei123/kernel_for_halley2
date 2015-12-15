/*
	$License:
	Copyright (C) 2015 Ingenic  Corporation, All Rights Reserved.

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
#include <linux/i2c/frizz.h>
#include <linux/sensor/mpu6500_platformdata.h>
#include "./frizz_input.h"
#include "../sensors_core.h"
#include "inc/sensors/libsensors_id.h"
#include "inc/hub_mgr/hub_mgr_if.h"
#include "frizz_debug.h"
#include "frizz_sensor_list.h"
#include "frizz_hal_if.h"
#include "sensors.h"
#include "frizz_file_id_list.h"
#include "frizz_reg.h"
#include "frizz_connection.h"
#include "frizz_chip_orientation.h"
#define ERR_FILE_BAD_FORM -77
int frizz_irq, frizz_reset, frizz_wakeup;
struct mutex burn_lock;
 const uint8_t Firmware_hex[] ={
 #include "frizz_firmware.h"
};

int enabled_accel = 0;

int on_isq = 0;
int config_verify = 0;
#define MAX_GYRO	32767
#define MIN_GYRO	-32768
#define I2C_BUFF_SIZE             (TRANS_BUFF_SIZE)   /*!< Max transfer buffer size (Byte)*/

static unsigned char *tx_buff = 0;
/*!< read fifo data from i2c */
static unsigned char *rx_buff = 0;
#define MPU6500_LOGTIME		10000
#define LOG_RESULT_LOCATION(x) {\
	printk(KERN_ERR "%s:%s:%d result=%d\n",__FILE__,__func__,__LINE__, x);\
}\

#define CHECK_RESULT(x) {\
		result = x;\
		if (unlikely(result)) \
			LOG_RESULT_LOCATION(result);\
	}

#define MODULE_NAME_ACCEL	"accelerometer_sensor"
#define MODULE_NAME_GYRO	"gyro_sensor"

unsigned int fifo_data[MAX_HWFRIZZ_FIFO_SIZE];
#define MAX_ACCESS_LIMIT        (32)            /*!< max transfer size         */
#define I2C_RW_TRY_TIMES    20

struct frizz_input_data  frizz_i2c;

#define GEN_CMD_CODE(sensor_id, on) \
    (uint32_t) (((uint32_t)((on)&0xFF)<<24) | \
    ((uint32_t)((sensor_id)&0xFF)<<16) | \
    ((uint32_t)((0x00)&0xFF)<<8) | \
    ((uint32_t)((0x01)&0xFF)))

static void print_fifo_data(unsigned int data[], int num) {
    int i;

    for(i = 0; i < num; i++) {
        printk("%08x ", data[i]);
    }
    printk("\n");
}

static int __frizz_i2c_read(struct i2c_client *i2c_client,unsigned int reg_addr, unsigned char * const rx_buff, int buff_size)
{
	const unsigned char tx_buff = (unsigned char)reg_addr;
	int data_left = buff_size;
	int times_out = I2C_RW_TRY_TIMES;

	// (1) set address
	int status = i2c_master_send(i2c_client, &tx_buff, sizeof(tx_buff));
	if(status < 0){
		dump_stack();
		return status;
	}
	else{
		// receive data.
		do {
			status = i2c_master_recv(i2c_client, rx_buff, buff_size);
			if(status < 0) {
				return status;
			}
			data_left -= status;
			times_out -= 1;
		}while(data_left != 0 && times_out != 0);
		if(!times_out && data_left != 0) {
			status = data_left;
			kprint("%s rereceive data failed!!!, bytes_left = %d", __func__, data_left);
		}
	}
	//if all data receive, data_left should be Zero;
	status = data_left;
	return status;
}

static int __frizz_i2c_write(struct i2c_client *i2c_client,const unsigned char * const data, int data_size)
{
	int data_left = data_size;
	int status = 0;
	int times_out = I2C_RW_TRY_TIMES;

	do{
		status = i2c_master_send(i2c_client, data + (data_size - data_left), data_left);
		if(status < 0) {
			if(data_size == 5 && data[0]==0 && data[1]==0 && data[2]==0 && data[3]==0 && data[4]==1){
				DEBUG_PRINT("frizz: __frizz_i2c_write() ignore ctrl reset\n" );
				status = 0;
			}
			if(status < 0) {
				dump_stack();
			}
			return status;
		}
		data_left -= status;
		times_out -= 1;
	}while(data_left != 0 && times_out != 0);

	if(!times_out && data_left) {
		status = data_left;
		kprint("%s resend data failed!!!, bytes_left = %d", __func__, data_left);
	}

	//if all data receive, data_left should be Zero;
	status = data_left;
	return status;
}

static int __i2c_write_reg_32(struct i2c_client *i2c_client,unsigned int reg_addr, unsigned int data)
{
	unsigned char tx_buff[5];
	int status = 0;

	tx_buff[0] = (unsigned char)reg_addr;
	tx_buff[1] = (data >> 24) & 0xff;
	tx_buff[2] = (data >> 16) & 0xff;
	tx_buff[3] = (data >>  8) & 0xff;
	tx_buff[4] = (data >>  0) & 0xff;

	status = __frizz_i2c_write(i2c_client,tx_buff, sizeof(tx_buff));

	return status;
}

int i2c_write_reg_32(struct i2c_client *i2c_client,unsigned int reg_addr, unsigned int data)
{
	int status = 0;
	if (!i2c_client) {
		return -ENODEV;
	}

	status = __i2c_write_reg_32(i2c_client,reg_addr, data);
	return status;
}

static int __i2c_read_reg_32(struct i2c_client *i2c_client,unsigned int reg_addr, unsigned int* data)
{
	unsigned char rx_buff[4];

	int status = __frizz_i2c_read(i2c_client,reg_addr, rx_buff, sizeof(rx_buff));

	if (!status) {
		*data = CONVERT_UINT(rx_buff[0], rx_buff[1] , rx_buff[2] , rx_buff[3] );
	}
	return status;
}

int i2c_read_reg_32(struct i2c_client *i2c_client,unsigned int reg_addr, unsigned int* data)
{
	int status = 0;
	if (!i2c_client) {
		return -ENODEV;
	}
	status = __i2c_read_reg_32(i2c_client, reg_addr, data);
	return status;
}

static int __i2c_read_reg_uint_array(struct i2c_client *i2c_client,unsigned int reg_addr, unsigned int *array, int array_size)
{
	int status = 0;
	unsigned char * const rx_buff = (unsigned char*)array;
	const int rx_size             = (array_size * 4);

	status = __frizz_i2c_read(i2c_client,reg_addr, rx_buff, rx_size);

	if (!status) {
		const unsigned char* src         = rx_buff;
		unsigned int* dst                = array;
		const unsigned int* const beyond = dst + array_size;

		for(; dst < beyond; dst++, src+=4 ){
			*dst = CONVERT_UINT(src[0], src[1], src[2], src[3] );
		}
		status = 0;
	}
	return status;
}

int i2c_read_reg_uint_array(struct i2c_client *i2c_client,unsigned int reg_addr, unsigned int *array, int array_size)
{
	int status = 0;
	int rest_size = (array_size * 4);
	unsigned char* p = (unsigned char*)array;

	if (!i2c_client) {
		return -ENODEV;
	}


	while(0 < rest_size && status == 0){
		const int read_size = (rest_size < MAX_ACCESS_LIMIT ? rest_size : MAX_ACCESS_LIMIT );

		status = __i2c_read_reg_uint_array(i2c_client,reg_addr, (unsigned int *)p, read_size/4 );
		if(status)
			break;
		rest_size -= read_size;
		p += read_size;
	}

	return status;
}

static int __i2c_write_reg_ram_data(struct i2c_client *i2c_client,unsigned int reg_addr, unsigned int ram_addr, unsigned char *ram_data, int ram_size)
{
	int status = 0;

	const int send_size = ram_size + 5;

	unsigned char * const tx_reg_addr    = tx_buff;
	unsigned char * const tx_ram_addr    = tx_buff + 1;
	unsigned char * const tx_data        = tx_buff + 5;

	memset(tx_buff, 0, send_size);

	*tx_reg_addr   = (unsigned char)reg_addr;				// register address
	tx_ram_addr[0] = (ram_addr >> 24 ) & 0xff;				// ram address
	tx_ram_addr[1] = (ram_addr >> 16 ) & 0xff;
	tx_ram_addr[2] = (ram_addr >>  8 ) & 0xff;
	tx_ram_addr[3] = (ram_addr >>  0 ) & 0xff;

	memcpy(tx_data, ram_data, ram_size);					// data

	status = __frizz_i2c_write(i2c_client,tx_buff, send_size );

	return status;
}

int i2c_write_reg_ram_data(struct i2c_client *i2c_client,unsigned int reg_addr, unsigned int ram_addr, unsigned char *ram_data, int ram_size)
{
	int status = 0;
	int rest_size = ram_size;
	unsigned int   dst = ram_addr;
	unsigned char* src = ram_data;

	if (!i2c_client) {
		return -ENODEV;
	}

	for(rest_size = ram_size; 0 < rest_size && status == 0;){
		const int write_size = (rest_size < MAX_ACCESS_LIMIT ? rest_size : MAX_ACCESS_LIMIT );
		status = __i2c_write_reg_ram_data(i2c_client,reg_addr, dst, src, write_size );
		if(status)
			break;

		rest_size -= write_size;
		dst += (write_size/4);
		src += write_size;
	}

	return status;
}

static int __i2c_read_reg_array(struct i2c_client *i2c_client,unsigned int reg_addr, unsigned char *array, int array_size)
{
	int status=0;

	status = __frizz_i2c_read(i2c_client,reg_addr, array, array_size);

	return status;
}

int i2c_read_reg_array(struct i2c_client *i2c_client,unsigned int reg_addr, unsigned char *array, int array_size)
{
	int status=0;
	int rest_size = array_size;
	unsigned char* p = array;
	unsigned int access_addr = 0;

	if (!i2c_client) {
		return -ENODEV;
	}

	if(MAX_ACCESS_LIMIT < rest_size ){
		__i2c_read_reg_32(i2c_client,RAM_ADDR_REG_ADDR, &access_addr);
		__i2c_write_reg_32(i2c_client,RAM_ADDR_REG_ADDR, access_addr);
	}
	for(rest_size = array_size; 0 < rest_size && status == 0;){
		const int read_size = (rest_size < MAX_ACCESS_LIMIT ? rest_size : MAX_ACCESS_LIMIT );

		status = __i2c_read_reg_array(i2c_client,reg_addr, p, read_size );
		if(status)
			break;

		rest_size -= read_size;
		p += read_size;

		if(0 < rest_size ){
			access_addr += (read_size/4);
			__i2c_write_reg_32(i2c_client,RAM_ADDR_REG_ADDR, access_addr);
		}
	}

	return status;
}

static int serial_read_fifo_size(struct i2c_client *i2c_client)
{
	int status;
	unsigned int fifo_cnr;
	status = i2c_read_reg_32(i2c_client,FIFO_CNR_REG_ADDR, &fifo_cnr);
	if(status == 0) {
		return (int)(fifo_cnr >> 16);
	} else {
		return -1;
	}
}

static int enable_sensor_id(struct frizz_input_data *data,uint32_t sensor_id,uint32_t enable_value)
{
	int ret;
	int iii;
	unsigned int fifo_size = 0,timeout;
	ret = i2c_write_reg_32(data->client,MES_REG_ADDR, 0xFF81FF01);
	if (ret != 0)
		printk("frizz fw command write MES_REG_ADDR failed.\n");
	for(timeout = 0;timeout < 25;timeout ++){
		fifo_size = serial_read_fifo_size(data->client);
		if(fifo_size > 0){
			memset(fifo_data, 0, MAX_HWFRIZZ_FIFO_SIZE);
			i2c_read_reg_uint_array(data->client,FIFO_REG_ADDR, fifo_data, fifo_size);

			for(iii = 0; iii<fifo_size; iii++){
				if(fifo_data[iii] == MESSAGE_ACK){
						ret = 1;// sensor enable pass
						break;
				}
			}
			if(ret == 1) {
				enabled_accel = 1;
				break;
			}else{
				msleep(2);
			}
		}else{
			msleep(2);
		}
	}
	ret = i2c_write_reg_32(data->client,MES_REG_ADDR, GEN_CMD_CODE(sensor_id, enable_value));
	if (ret != 0){
		printk("frizz fw command write MES_REG_ADDR failed.\n");
		return -1;
	}

	for(timeout = 0; timeout<50; timeout++){
		msleep(2);
		fifo_size = serial_read_fifo_size(data->client);
		if(fifo_size > 0){
			memset(fifo_data, 0, MAX_HWFRIZZ_FIFO_SIZE);
			i2c_read_reg_uint_array(data->client,FIFO_REG_ADDR, fifo_data, fifo_size);
			for(iii = 0; iii<fifo_size; iii++){
				if(fifo_data[iii] == RESPONSE_HUB_MGR){
					if(fifo_data[iii+2] != 0)
						return -1;		//sensor enable fail
					else{
						ret = 1;// sensor enable pass
						break;
					}
				}
			}
			if(ret == 1) {
				enabled_accel = 1;
				break;
			}
		}
	}
	return 0;
}

static int set_delay_sensor_id(struct frizz_input_data *data,uint32_t sensor_id,uint32_t handle_num,unsigned int times)
{
	int ret;
	int iii;
	unsigned int fifo_size = 0,timeout;
	// ===================== Sensor Interval =========================
	ret = i2c_write_reg_32(data->client,MES_REG_ADDR, 0xFF81FF02);
	for(timeout = 0;timeout < 25;timeout ++){
		fifo_size = serial_read_fifo_size(data->client);
		if(fifo_size > 0){
			memset(fifo_data, 0, MAX_HWFRIZZ_FIFO_SIZE);
			i2c_read_reg_uint_array(data->client,FIFO_REG_ADDR, fifo_data, fifo_size);
			for(iii = 0; iii<fifo_size; iii++){
				if(fifo_data[iii] == MESSAGE_ACK){
						ret = 1;// sensor enable pass
						break;
				}
			}
			if(ret == 1) {
				enabled_accel = 1;
				break;
			}else{
				msleep(2);
			}
		}else{
			msleep(2);
		}
	}
	ret = i2c_write_reg_32(data->client,MES_REG_ADDR, GEN_CMD_CODE(sensor_id, handle_num));
	for(timeout = 0;timeout < 25;timeout ++){
		fifo_size = serial_read_fifo_size(data->client);
		if(fifo_size > 0){
			memset(fifo_data, 0, MAX_HWFRIZZ_FIFO_SIZE);
			i2c_read_reg_uint_array(data->client,FIFO_REG_ADDR, fifo_data, fifo_size);

			for(iii = 0; iii<fifo_size; iii++){
				if(fifo_data[iii] == MESSAGE_ACK){
						ret = 1;// sensor enable pass
						break;
				}
			}
			if(ret == 1) {
				enabled_accel = 1;
				break;
			}else{
				msleep(2);
			}
		}else{
			msleep(2);
		}
	}
	ret = i2c_write_reg_32(data->client,MES_REG_ADDR, times);
	ret = 0;
	for(timeout = 0; timeout<50; timeout++){
		fifo_size = serial_read_fifo_size(data->client);
		if(fifo_size > 0){
			memset(fifo_data, 0, MAX_HWFRIZZ_FIFO_SIZE);
			i2c_read_reg_uint_array(data->client,FIFO_REG_ADDR, fifo_data, fifo_size);

			for(iii = 0; iii<fifo_size; iii++){
				if(fifo_data[iii] == RESPONSE_HUB_MGR){
					if(fifo_data[iii+2] != 0)
						return -1;		//sensor enable fail
					else{
						printk("sensor interval pass.\n");
						ret = 1;// sensor enable pass
						break;
					}
				}
			}
			if(ret == 1)
				break;
		}
		msleep(2);
	}
	return 0;
}

static ssize_t enable_show(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct frizz_input_data *data = input_get_drvdata(input_data);

	return sprintf(buf, "%09u\n", atomic_read(&data->enable));

}

static ssize_t enable_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct frizz_input_data *data = input_get_drvdata(input_data);

	int ret;
	int value = simple_strtoul(buf, NULL, 10);

	pr_info("[SENSOR] %s : enable = %d  read:%d  value&0x02:%x)\n", __func__, value,atomic_read(&data->enable),value & 0x02);

	mutex_lock(&data->mutex);

#ifdef CONFIG_INPUT_FRIZZ_POLLING
	if (value && !atomic_read(&data->enable)) {
		if(value && !atomic_read(&data->enable) && (value&0x01) ){
			schedule_delayed_work(&data->accel_work, msecs_to_jiffies(5));
			ret = enable_sensor_id(data,0x80,0x01);
			if(ret < 0)
				return -1;
			udelay(1000);
		}
	}
#if 0
	if (value && atomic_read(&data->enable)) {
		if(value && atomic_read(&data->enable) && (value&0x02) ){
			schedule_delayed_work(&data->gyro_work, msecs_to_jiffies(5));
			enable_sensor_id(data,0x82,0x01);
		}
	}
#endif
	if (!value && atomic_read(&data->enable)) {
		enable_sensor_id(data,0x80,0x00);
		cancel_delayed_work_sync(&data->accel_work);
		cancel_delayed_work_sync(&data->gyro_work);
		enable_sensor_id(data,0x82,0x00);
		if (atomic_read(&data->reactive_enable)){}
	}
#else
	if (value && !atomic_read(&data->enable)) {
	//	mpu6500_input_activate_devices(data, MPU6500_SENSOR_ACCEL, true);
	}
	if (!value && atomic_read(&data->enable)) {
	//	mpu6500_input_activate_devices(data, MPU6500_SENSOR_ACCEL, false);
	}
#endif

	atomic_set(&data->enable, value);
	mutex_unlock(&data->mutex);

	return count;
}

static ssize_t mpu6500_input_accel_delay_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct frizz_input_data *data = input_get_drvdata(input_data);

	return sprintf(buf, "%d\n", atomic_read(&data->accel_delay));

}

static ssize_t mpu6500_input_accel_delay_store(struct device *dev,
					       struct device_attribute *attr,
					       const char *buf, size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct frizz_input_data *data = input_get_drvdata(input_data);
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

static ssize_t mpu6500_input_gyro_delay_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct frizz_input_data *data = input_get_drvdata(input_data);

	return sprintf(buf, "%d\n", atomic_read(&data->gyro_delay));

}

static ssize_t mpu6500_input_gyro_delay_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct frizz_input_data *data = input_get_drvdata(input_data);
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
	struct frizz_input_data *data = input_get_drvdata(input_data);

	return sprintf(buf, "%d\n", atomic_read(&data->motion_recg_enable));
}

static ssize_t mpu6500_input_motion_recg_enable_store(struct device *dev,
						      struct device_attribute
						      *attr, const char *buf,
						      size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct frizz_input_data *data = input_get_drvdata(input_data);

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
	&dev_attr_acc_delay.attr,
	&dev_attr_mot_recg_enable.attr,
	NULL
};

static struct attribute *gyro_attributes[] = {
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
	struct frizz_input_data *data = dev_get_drvdata(dev);;
	frizz_read_accel(data,&acc);
	return sprintf(buf, "%d, %d, %d\n", acc.x, acc.y, acc.z);
}

static struct device_attribute dev_attr_raw_data =
	__ATTR(raw_data, S_IRUSR | S_IRGRP, acc_data_read,
		NULL);

static ssize_t accel_calibration_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct frizz_input_data *data = dev_get_drvdata(dev);
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
	struct frizz_input_data *data = dev_get_drvdata(dev);
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
	return sprintf(buf, "%s\n", "Frizz");
}

static struct device_attribute dev_attr_accel_sensor_vendor =
	__ATTR(vendor, S_IRUSR | S_IRGRP, accel_vendor_show, NULL);
static struct device_attribute dev_attr_gyro_sensor_vendor =
	__ATTR(vendor, S_IRUSR | S_IRGRP, accel_vendor_show, NULL);

static ssize_t accel_name_show(struct device *dev,
				struct device_attribute *attr,
					char *buf)
{
	return sprintf(buf, "%s\n", "FRIZZ-MPU6500");
}

static ssize_t show_acc_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct frizz_input_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", atomic_read(&data->accel_delay));
}
static ssize_t set_acc_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct frizz_input_data *data = input_get_drvdata(input_data);
	int value = 0;
	int ret = 0;

	ret = kstrtoint(buf, 10, &value);
	if (ret) {
		pr_err("[SENSOR]: %s - Invalid Argument\n", __func__);
		return ret;
	}

	value = 10;
	pr_info("[SENSOR] %s : delay = %d\n", __func__, value);
	mutex_lock(&data->mutex);

	set_delay_sensor_id(data,0x80,0x02,10);

	atomic_set(&data->accel_delay, value);

	mutex_unlock(&data->mutex);

	return size;
}

static ssize_t show_gyro_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct frizz_input_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", atomic_read(&data->gyro_delay));
}

static ssize_t set_gyro_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct frizz_input_data *data = input_get_drvdata(input_data);
	int value = 0;
	int ret = 0;

	ret = kstrtoint(buf, 10, &value);
	if (ret) {
		pr_err("[SENSOR]: %s - Invalid Argument\n", __func__);
		return ret;
	}

	value = 10;
	pr_info("[SENSOR] %s : delay = %d\n", __func__, value);
	mutex_lock(&data->mutex);


	set_delay_sensor_id(data,0x82,0x2,10);

	atomic_set(&data->gyro_delay, value);
	mutex_unlock(&data->mutex);
	return size;
}

static struct device_attribute dev_attr_accel_sensor_name =
	__ATTR(name, S_IRUSR | S_IRGRP, accel_name_show, NULL);
static struct device_attribute dev_attr_gyro_sensor_name =
	__ATTR(name, S_IRUSR | S_IRGRP, accel_name_show, NULL);

static struct device_attribute dev_attr_enable =
	__ATTR(enable,S_IRUGO | S_IWUSR | S_IWGRP, enable_show,
		enable_store);
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

static void frizz_fw_command_test(struct i2c_client *i2c_client,uint32_t sensor_id, uint32_t test_loop)
{
	int ret;
	int iii;
	int fifo_size;
	unsigned int fifo_data[MAX_HWFRIZZ_FIFO_SIZE];

    /*  send command to frizz
        prefix    kind    sen_id    num
        0xFF      0x81    0xFF      0x01
    */
	ret = i2c_write_reg_32(i2c_client,MES_REG_ADDR, 0xFF81FF01);
	if (ret == 0)
		printk("frizz fw command write MES_REG_ADDR successful.\n");
	else
		printk("frizz fw command write MES_REG_ADDR failed.\n");
	udelay(100);

    /* SET_SENSOR_ACTIVE, sensor_id, 0x00, 0x00 */
	if(sensor_id == 0x9F)     //SENSOR_ID_ACCEL_MOVE
		ret = i2c_write_reg_32(i2c_client,MES_REG_ADDR, 0x019F0001);
	ret = i2c_write_reg_32(i2c_client,MES_REG_ADDR, GEN_CMD_CODE(sensor_id, 0x01));
	if (ret == 0)
		printk("frizz fw command write MES_REG_ADDR successful.\n");
	else
		printk("frizz fw command write MES_REG_ADDR failed.\n");
	while(test_loop--) {
		fifo_size = serial_read_fifo_size(i2c_client);

		if(fifo_size > 0) {
			memset(fifo_data, 0, MAX_HWFRIZZ_FIFO_SIZE);
			i2c_read_reg_uint_array(i2c_client,FIFO_REG_ADDR, fifo_data, fifo_size);
			print_fifo_data(fifo_data, fifo_size);
			printk("git (fifo_size > 0)fifo_size = %d\n", fifo_size);
			for(iii=0; iii<fifo_size; iii++)
				printk("CQF git (fifo_size > 0)fifo_data[%d] = [%8d]=[0x%08x].\n", iii, fifo_data[iii], fifo_data[iii]);
		}else {
			printk("git fifo_size %d\n",fifo_size);
		}
		msleep(1000);
	}
	udelay(100);
}

int analysis_fifo(unsigned int *fifo_data, int *index,unsigned int *f32_data)
{
    int i, num;
    unsigned int data[10];
    if (fifo_data[*index] == OUTPUT_INTERVAL_TIME) {
        data[0] = fifo_data[*index + 1]; //timestamp
        data[1] = fifo_data[*index + 2]; //update interval
        data[2] = fifo_data[*index + 3];
        *index = *index + 2;
        return 0;
    } else {
        num = fifo_data[*index] & 0xFF;
        for (i = 0; i < num; i++) {
            data[i] = fifo_data[i + *index + 1];
        }
        for (i = 0; i < num-1; i++) {
		f32_data[i] = (data[i+1]);
        }
        return 0;
    }
    return 0;
}

int frizz_read_accel(struct frizz_input_data *data,struct mpu6500_v *acc_raw)
{
	struct mpu6500_v acc;
	int result;
	unsigned int fifo_size = 0;
	unsigned int f32_data[10] = {0};
	int i = 0;
	result = i2c_write_reg_32(data->client,MES_REG_ADDR ,0xff818007);
	if (result) {
		pr_err("[SENSOR] %s: i2c_read err= %d\n", __func__, result);
		return -1;
	}
	udelay(100);
	result = i2c_write_reg_32(data->client,MES_REG_ADDR ,0x01800001);
	if (result) {
		pr_err("[SENSOR] %s: i2c_read err= %d\n", __func__, result);
		return -1;
	}
	udelay(100);
	result = i2c_read_reg_32(data->client,FIFO_CNR_REG_ADDR , &fifo_size);
	if (result) {
		pr_err("[SENSOR] %s: i2c_read err= %d\n", __func__, result);
		return -1;
	}
	fifo_size = (int)(fifo_size >> 16);

	if (fifo_size > 0) {
		memset(fifo_data, 0, MAX_HWFRIZZ_FIFO_SIZE);
		result = i2c_read_reg_uint_array(data->client,FIFO_REG_ADDR, fifo_data, fifo_size);
		if(result) {
			dump_stack();
			pr_info("%s serial failed. result = %d \n", __func__, result);
		}

		for (i = 0; i < fifo_size; i++) {
			if (((fifo_data[i] >> 16) == 0xFF80) && ((fifo_data[i] & 0xff) <= (fifo_size - i))) {
				analysis_fifo(fifo_data, &i,f32_data);
			}
		}
		acc.x = (s16) (f32_data[0])*10;
		acc.y = (s16) (f32_data[1])*10;
		acc.z = (s16) (f32_data[2])*10;

		acc_raw->x = acc.x - data->acc_cal[0];
		acc_raw->y = acc.y - data->acc_cal[1];
		acc_raw->z = acc.z - data->acc_cal[2];
	}
	return 0;

}

static void mpu6500_input_report_accel_xyz(struct frizz_input_data *data)
{
	unsigned int fifo_size = 0;
	int i;
	unsigned int f32_data[10] = {0};
	mutex_lock(&data->mutex);
	if(enabled_accel){
		fifo_size = serial_read_fifo_size(data->client);
		if(fifo_size > 0){
			memset(fifo_data, 0, MAX_HWFRIZZ_FIFO_SIZE);
			i2c_read_reg_uint_array(data->client,FIFO_REG_ADDR, fifo_data, fifo_size);
			for (i = 0; i < fifo_size; i++) {
				if (((fifo_data[i] >> 16) == 0xFF80) && ((fifo_data[i] & 0xff) <= (fifo_size - i))) {
					memset(f32_data, 0, 10);
					analysis_fifo(fifo_data, &i,f32_data);

					input_report_rel(data->accel_input, REL_X, f32_data[0]);
					input_report_rel(data->accel_input, REL_Y, f32_data[1]);
					input_report_rel(data->accel_input, REL_Z, f32_data[2]);
					if (atomic_read(&data->accel_delay) * data->count_logtime > MPU6500_LOGTIME) {
						data->count_logtime = 0;
					} else
						data->count_logtime++;
					input_sync(data->accel_input);
				}
			}
		}else{
			printk("git fifo_size = 0\n");
		}
	}
	mutex_unlock(&data->mutex);
}

static void mpu6500_input_report_gyro_xyz(struct frizz_input_data *data)
{
	struct mpu6500_v acc;
	unsigned int fifo_size = 0;
	int i;
	unsigned int f32_data[10] = {0};
	mutex_lock(&data->mutex);
	if(enabled_accel){
		fifo_size = serial_read_fifo_size(data->client);
		if(fifo_size > 0){
			memset(fifo_data, 0, MAX_HWFRIZZ_FIFO_SIZE);
			i2c_read_reg_uint_array(data->client,FIFO_REG_ADDR, fifo_data, fifo_size);
			for (i = 0; i < fifo_size; i++) {
				if (((fifo_data[i] >> 16) == 0xFF80) && ((fifo_data[i] & 0xff) <= (fifo_size - i))) {
					memset(f32_data, 0, 10);
					analysis_fifo(fifo_data, &i,f32_data);
					acc.x = (s16) (f32_data[0]);
					acc.y = (s16) (f32_data[1]);
					acc.z = (s16) (f32_data[2]);
					pr_info("[SENSOR] %s, %u, %u %u\n",
				__func__, f32_data[0], f32_data[1], f32_data[2]);

					input_report_rel(data->gyro_input, REL_X, f32_data[0]);
					input_report_rel(data->gyro_input, REL_Y, f32_data[1] );
					input_report_rel(data->gyro_input, REL_Z, f32_data[2]);
					if (atomic_read(&data->gyro_delay) * data->count_logtime > MPU6500_LOGTIME) {
						pr_info("[SENSOR] %s, %u, %u %u\n",
						__func__, f32_data[0], f32_data[1], f32_data[2]);
						data->count_logtime = 0;
					} else
						data->count_logtime++;
						input_sync(data->gyro_input);
				}
			}
		}else{
			printk("git fifo_size = 0\n");
		}
	}
	mutex_unlock(&data->mutex);
}

static irqreturn_t mpu6500_input_irq_thread(int irq, void *dev)
{
	struct frizz_input_data *data = (struct frizz_input_data *)dev;

	mpu6500_input_report_accel_xyz(data);
	mpu6500_input_report_gyro_xyz(data);
			/* disable motion int */
	return IRQ_HANDLED;
}
#ifdef CONFIG_INPUT_FRIZZ_POLLING
static void mpu6500_work_func_acc(struct work_struct *work)
{
	struct frizz_input_data *data =
		container_of((struct delayed_work *)work,
			struct frizz_input_data, accel_work);
	mpu6500_input_report_accel_xyz(data);

	if (atomic_read(&data->accel_delay) < 60) {
		usleep_range(atomic_read(&data->accel_delay) * 1000,
			atomic_read(&data->accel_delay) * 1100);
		schedule_delayed_work(&data->accel_work, 0);
	} else {
		schedule_delayed_work(&data->accel_work,
			msecs_to_jiffies(
			atomic_read(&data->accel_delay)));
	}
}

static void mpu6500_work_func_gyro(struct work_struct *work)
{
	struct frizz_input_data *data =
		container_of((struct delayed_work *)work,
			struct frizz_input_data, gyro_work);

	mpu6500_input_report_gyro_xyz(data);

	if (atomic_read(&data->gyro_delay) < 60) {
		usleep_range(atomic_read(&data->gyro_delay) * 1000,
			atomic_read(&data->gyro_delay) * 1100);
		schedule_delayed_work(&data->gyro_work, 0);
	} else {
		schedule_delayed_work(&data->gyro_work,
			msecs_to_jiffies(
			atomic_read(&data->gyro_delay)));
	}
}
#endif
static DEVICE_ATTR(accel_poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_acc_delay, set_acc_delay);
static DEVICE_ATTR(gyro_poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_gyro_delay, set_gyro_delay);
#ifdef CONFIG_SENSORS_SSP_ADPD142
static DEVICE_ATTR(hrm_poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_hrm_delay, set_hrm_delay);
#endif

static struct device_attribute *frizz_sensor_attrs[] = {
	&dev_attr_enable,
	&dev_attr_accel_poll_delay,
	&dev_attr_gyro_poll_delay,

	NULL,
};

static int mpu6500_accel_input_init(struct frizz_input_data *data)
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

static int mpu6500_gyro_input_init(struct frizz_input_data *data)
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

static int frizz_ioctl_hardware_stall(struct i2c_client *i2c_client)
{
	int ret = 0;
	ret = i2c_write_reg_32(i2c_client,CTRL_REG_ADDR, CTRL_REG_SYSTEM_RESET);
	if(ret)
		kprint("frizz i2c bus error, when reset the hardware!!, ret = %d", ret);
	msleep(100);
	ret = i2c_write_reg_32(i2c_client,CTRL_REG_ADDR, CTRL_REG_STALL);
	if(ret)
		kprint("frizz i2c bus error, when stall the hardware!!, ret = %d", ret);
	return 0;
}

int frizz_ioctl_hardware_run(struct i2c_client *i2c_client)
{
	i2c_write_reg_32(i2c_client,CTRL_REG_ADDR, CTRL_REG_RUN);
	return 0;
}

int init_g_sensor_chip_orientation(struct i2c_client *i2c_client,int type,unsigned int position)
{
	int ret;
	int iii;
	unsigned int fifo_size = 0,timeout;
	int index_position;
	switch(type){
		case 0:
			ret = i2c_write_reg_32(i2c_client,MES_REG_ADDR, acc_sensor_orientation[position][0]);
			break;
		case 1:
			ret = i2c_write_reg_32(i2c_client,MES_REG_ADDR, gyro_sensor_orientation[position][0]);
			break;
		case 2:
			ret = i2c_write_reg_32(i2c_client,MES_REG_ADDR, magn_sensor_orientation[position][0]);
			break;
		default:
			ret = -1;
			break;
	}
	if (ret != 0)
		printk("frizz fw command write MES_REG_ADDR failed.\n");
	for(timeout = 0;timeout < 25;timeout ++){
		fifo_size = serial_read_fifo_size(i2c_client);
		if(fifo_size > 0){
			memset(fifo_data, 0, MAX_HWFRIZZ_FIFO_SIZE);
			i2c_read_reg_uint_array(i2c_client,FIFO_REG_ADDR, fifo_data, fifo_size);

			for(iii = 0; iii<fifo_size; iii++){
				if(fifo_data[iii] == MESSAGE_ACK){
					printk("[sensorhub] position 0 ack\n");
						ret = 1;// sensor enable pass
						break;
				}
			}
			if(ret == 1) {
				enabled_accel = 1;
				break;
			}else{
				msleep(2);
			}
		}else{
			msleep(2);
		}
	}

	for(index_position = 1; index_position < 7; index_position ++){
		switch(type){
		case 0:
			ret = i2c_write_reg_32(i2c_client,MES_REG_ADDR, acc_sensor_orientation[position][index_position]);
			break;
		case 1:
			ret = i2c_write_reg_32(i2c_client,MES_REG_ADDR, gyro_sensor_orientation[position][index_position]);
			break;
		case 2:
			ret = i2c_write_reg_32(i2c_client,MES_REG_ADDR, magn_sensor_orientation[position][index_position]);
			break;
		}
		if (ret != 0){
			printk("frizz fw command write MES_REG_ADDR failed.\n");
			return -1;
		}
		for(timeout = 0; timeout<50; timeout++){
			fifo_size = serial_read_fifo_size(i2c_client);
			if(fifo_size > 0){
				memset(fifo_data, 0, MAX_HWFRIZZ_FIFO_SIZE);
				i2c_read_reg_uint_array(i2c_client,FIFO_REG_ADDR, fifo_data, fifo_size);

				for(iii = 0; iii<fifo_size; iii++){
					if(fifo_data[iii] == MESSAGE_ACK){
					printk("[sensorhub] position index_position ack\n");
							ret = 1;// sensor enable pass
							break;
					}
				}
				if(ret == 1) {
					break;
				}
			}
			msleep(2);
		}
	}
	switch(type){
		case 0:
			ret = i2c_write_reg_32(i2c_client,MES_REG_ADDR, acc_sensor_orientation[position][7]);
			break;
		case 1:
			ret = i2c_write_reg_32(i2c_client,MES_REG_ADDR, gyro_sensor_orientation[position][7]);
			break;
		case 2:
			ret = i2c_write_reg_32(i2c_client,MES_REG_ADDR, magn_sensor_orientation[position][7]);
			break;
	}
	if (ret != 0)
		printk("frizz fw command write MES_REG_ADDR failed.\n");
	for(timeout = 0;timeout < 25;timeout ++){
		fifo_size = serial_read_fifo_size(i2c_client);
		if(fifo_size > 0){
			memset(fifo_data, 0, MAX_HWFRIZZ_FIFO_SIZE);
			i2c_read_reg_uint_array(i2c_client,FIFO_REG_ADDR, fifo_data, fifo_size);

			for(iii = 0; iii<fifo_size; iii++){
				if(fifo_data[iii] == RESPONSE_HUB_MGR || fifo_data[iii]==MESSAGE_ACK){
					printk("[sensorhub] position 7 response\n");
						ret = 1;// sensor enable pass
						break;
				}
			}
			if(ret == 1) {
				enabled_accel = 1;
				break;
			}else{
				msleep(2);
			}
		}else{
			msleep(2);
		}
	}

	return 0;
}

int init_g_chip_orientation(struct i2c_client *i2c_client,unsigned int position)
{
	init_g_sensor_chip_orientation(i2c_client,0,position);
	init_g_sensor_chip_orientation(i2c_client,1,position);
	return 0;
}

int frizz_ioctl_hardware_reset(struct i2c_client *i2c_client,unsigned int chip_orientation)
{
	unsigned int xdata = 0;
	on_isq = 0;
	gpio_direction_output(frizz_wakeup, 1);
	gpio_direction_output(frizz_reset, 1);
	if(i2c_write_reg_32(i2c_client,CTRL_REG_ADDR, CTRL_REG_SYSTEM_RESET))
		kprint("frizz i2c bus error, when stall the hardware!!");
	i2c_write_reg_32(i2c_client,CTRL_REG_ADDR, CTRL_REG_RUN);
	msleep(100);
	i2c_read_reg_32(i2c_client,MODE_REG_ADDR, &xdata);
	xdata &= ~((unsigned int)1 << 10);
	i2c_write_reg_32(i2c_client,MODE_REG_ADDR, xdata);

	init_g_chip_orientation(i2c_client,chip_orientation);
	gpio_direction_output(frizz_wakeup, 1);

	return 0;
}

void read_file(struct file *filep, unsigned char* read_data, int read_data_size, int *read_file_size)
{
	mm_segment_t fs;
	memset(read_data, 0, sizeof(read_data));
	fs = get_fs();
	set_fs(get_ds());
	*read_file_size = filep->f_op->read(filep, read_data, read_data_size, &filep->f_pos);
	set_fs(fs);
}
//this download function is depends on the *.bin which in the file-system.
//u can use this function only when the file-system regist.
static int get_data(int offset, unsigned char *data, unsigned long length)
{
	unsigned long count;
	unsigned long length_left;
	length_left = sizeof(Firmware_hex)/sizeof(unsigned char) - offset;
	length = length_left >= length ? length : length_left;
	for(count = 0; count < length; count++ )
	{
		*(data + count) = *(Firmware_hex + offset + count);
	}
	return length;
}

int frizz_download_firmware(struct i2c_client *i2c_client,unsigned int g_chip_orientation)
{
	int ret = ERR_FILE_BAD_FORM;
	unsigned long offset = 0;
	unsigned long length = 0;

	unsigned int header;
	int modified_ram_size;
	int data_size;

	unsigned int cmd;
	unsigned int ram_addr;
	unsigned char *write_ram_data;
	unsigned char *read_ram_data;
	unsigned char read_data[4];
	unsigned char tmp = 0;

	int i;

    //when redownload the firmware, the on_isq must been close and wait for open again.
	on_isq = 0;

	length = sizeof(Firmware_hex)/sizeof(unsigned char);

    //before download the firmeware, must keep the frizz GPIO2 high to wakeup the frizz.
	printk("frizz start download firmware by hex \n");
	frizz_ioctl_hardware_stall(i2c_client);
	do {
		offset += get_data(offset, read_data, 2);
		header = (read_data[0] << 8);
		tmp = read_data[1];
		if (header == 0xC900) {
            //get file data size. remove cmd (2byte) and ram addr (4byte)
			offset += get_data(offset, read_data, 2);
			data_size = ((tmp << 16) | (read_data[0] << 8) | (read_data[1])) - 6;
            //get frizz command (command of writting ram)
			offset += get_data(offset, read_data, 2);
			cmd = (read_data[0] << 8) | (read_data[1]);
            //get ram address
			offset += get_data(offset, read_data, 4);
			ram_addr = (read_data[0] << 24) | (read_data[1] << 16)
			| (read_data[2] << 8) | (read_data[3]);

            //keep memory
			write_ram_data = kmalloc(data_size, GFP_KERNEL | GFP_DMA);
			offset += get_data(offset, write_ram_data, data_size);

			modified_ram_size = data_size + 3;
			modified_ram_size &= 0xFFFFFFFC;

			ret = i2c_write_reg_ram_data(i2c_client,RAM_ADDR_REG_ADDR, ram_addr,
					write_ram_data, modified_ram_size);
			if(ret){
				kfree(write_ram_data);
				break;
			}
			if(config_verify){
                //keep memory (verify)
				read_ram_data = kmalloc(data_size, GFP_KERNEL | GFP_DMA);
				ret = i2c_write_reg_32(i2c_client,RAM_ADDR_REG_ADDR, ram_addr);
				if(ret){
					kfree(write_ram_data);
					kfree(read_ram_data);
					break;
				}
				ret = i2c_read_reg_array(i2c_client,RAM_DATA_REG_ADDR, read_ram_data,
					modified_ram_size);
				if(ret){
					kfree(write_ram_data);
					kfree(read_ram_data);
					break;
				}
				for (i = 0; i < modified_ram_size; i++) {
					if (write_ram_data[i] != read_ram_data[i]) {
						break;
					}
				}
				if (i == modified_ram_size) {
					kprint("Verify Success. start address %x ", ram_addr);
				} else {
					printk("[%d] write ram data %x read ram data %x", i,write_ram_data[i], read_ram_data[i]);
					kprint("Verify Failed. start address %x ", ram_addr);
					kfree(write_ram_data);
					kfree(read_ram_data);
					ret = -1;
					break;
				}
				kfree(read_ram_data);
			}
			kfree(write_ram_data);
		} else if (header == 0xED00) {

            //don't execute command and skip data.
			offset += get_data(offset, read_data, 2);
			data_size = ((read_data[0] << 8) | (read_data[1]));

			offset += get_data(offset, read_data, 2);
			cmd = (read_data[0] << 8) | (read_data[1]);

			offset += get_data(offset, read_data, 4);
		}

	} while (offset < length);
	if(!ret) {
		printk("frizz download firmware by hex done \n");
	} else {
		kprint("frizz download firmware by hex failed!, ret = %d", ret);
	}
	frizz_ioctl_hardware_reset(i2c_client,g_chip_orientation);
	return 0;
}

int init_frizz_gpio(struct frizz_platform_data *frizz_pdata)
{
	static int irq_number = 0;
	static char *irq_button = "irq_button";
	static char *reset_button = "reset_button";
	static char *wakeup_button = "wakeup_button";
	int irq_error = 0;

	int ret = 0;
	frizz_irq = frizz_pdata->irq_gpio;
	irq_number = gpio_to_irq(frizz_irq);
	printk("frizz interrupt gpio is %d\n", frizz_irq);
	gpio_request(frizz_irq, irq_button);
	gpio_direction_input(frizz_irq);
	frizz_reset = frizz_pdata->reset_gpio;
	printk("frizz reset gpio is %d \n", frizz_reset);
	ret = gpio_request(frizz_reset, reset_button);
	if(ret)
	{
		printk("frizz request failed (%d) \n", ret);
	}
	gpio_direction_output(frizz_reset, 0);
	mdelay(2);
	gpio_direction_output(frizz_reset, 1);
	frizz_wakeup = frizz_pdata->wakeup_frizz_gpio;
	printk("frizz wakeup gpio is %d \n", frizz_wakeup);
	ret = gpio_request(frizz_wakeup, wakeup_button);
	gpio_direction_output(frizz_wakeup, 1);
	irq_error = request_irq(irq_number, mpu6500_input_irq_thread, IRQF_TRIGGER_FALLING, "frizz_input", irq_button);

	enable_irq_wake(irq_number);

	return ret;
}

static int frizz_input_probe(struct i2c_client *client,
					 const struct i2c_device_id *id)
{
	struct frizz_input_data *data = NULL;
	struct frizz_platform_data *frizz_pdata;
	int error = 0;
	int ret = -ENODEV;

	pr_info("[SENSOR] %s: IN\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("[SENSOR] %s: - i2c_check_functionality error\n",
			__func__);
		goto exit;
	}

	data = kzalloc(sizeof(struct frizz_input_data), GFP_KERNEL);
	if (!data)
		goto err_kzalloc;

	/* get platform data */

	frizz_pdata = client->dev.platform_data;
	pr_info("########### frizz_pdata-<irq_gpio:%x \n",frizz_pdata->irq_gpio);// = FRIZZ_IRQ;
	pr_info("########### frizz_pdata->reset_gpio :%x \n" ,frizz_pdata->reset_gpio );//= FRIZZ_RESET,
	pr_info("########### frizz_pdata->wakeup_frizz_gpio:%x \n",frizz_pdata->wakeup_frizz_gpio);// = FRIZZ_WAKEUP,
	pr_info("########### frizz_pdata->g_chip_orientation:%x \n" ,frizz_pdata->g_chip_orientation);// = GSENSOR_CHIP_ORIENTATION,

	data->client = client;
	client->addr = 0x1c;
	i2c_set_clientdata(client, data);

	data->pdata = kzalloc(sizeof(struct frizz_i2c_dev_data), GFP_KERNEL);
	if (!data->pdata) {
		dev_err(&client->dev,
					"alloc memory error %s\n", __func__);
		return -ENODEV;
	}
	mutex_init(&data->mutex);
	init_frizz_gpio(frizz_pdata);
	tx_buff = kmalloc(I2C_BUFF_SIZE, GFP_KERNEL | GFP_DMA);
	rx_buff = kmalloc(I2C_BUFF_SIZE, GFP_KERNEL | GFP_DMA);


	frizz_download_firmware(client,frizz_pdata->g_chip_orientation) ;
#ifdef CONFIG_INPUT_FRIZZ_FW_COMMAND_TEST
	frizz_fw_command_test(client,0x80, 1);
#endif
#ifdef CONFIG_INPUT_FRIZZ_POLLING
	INIT_DELAYED_WORK(&data->accel_work, mpu6500_work_func_acc);
	INIT_DELAYED_WORK(&data->gyro_work, mpu6500_work_func_gyro);
#endif
	error = mpu6500_accel_input_init(data);
	if (error)
		goto err_accel_input_init;

	error = mpu6500_gyro_input_init(data);
	if (error)
		goto err_gyro_input_init;


	error = sensors_register(data->mcu_device,data,frizz_sensor_attrs,"ssp_sensor");
	if (error) {
		pr_err("[SENSOR] %s: cound not register\
			frizz sensor device(%d).\n",
			__func__, error);
		goto acc_sensor_register_failed;
	}
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


	ret = request_threaded_irq(client->irq, NULL, mpu6500_input_irq_thread,
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "frizz_input", data);
	if (ret < 0) {
		pr_err("[SENSOR] %s: - can't allocate irq.\n", __func__);
		goto exit_reactive_irq;
	}

	disable_irq(client->irq);

	pr_info("[SENSOR] %s: success\n", __func__);
	return 0;

exit_reactive_irq:
	sensors_unregister(data->gyro_sensor_device, accel_sensor_attrs);
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
err_kzalloc:
exit:
	pr_err("[SENSOR] %s: - Probe fail!\n", __func__);
	return error;
}
static int  frizz_input_remove(struct i2c_client *client)
{
	struct frizz_input_data *data = i2c_get_clientdata(client);
	if (data == NULL)
		return 0;

	if (client->irq > 0) {
		free_irq(client->irq, data);
		input_unregister_device(data->accel_input);
		input_unregister_device(data->gyro_input);
	}

	kfree(data);

	return 0;
}

static const struct i2c_device_id frizz_input_id[] = {
	{"frizz_input", 0},
	{}
};

static struct i2c_driver frizz_input_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "frizz_input",
		   },
	.class = I2C_CLASS_HWMON,
	.id_table = frizz_input_id,
	.probe = frizz_input_probe,
	.remove = frizz_input_remove,
};




static int __init sensor_frizz_init(void)
{
	int result = i2c_add_driver(&frizz_input_driver);

	printk(KERN_INFO "[SENSOR] mpu6500_init()\n");

	return result;
}

static void __exit sensor_frizz_exit(void)
{
	printk(KERN_INFO "[SENSOR] mpu6500_exit()\n");

	i2c_del_driver(&frizz_input_driver);
}

module_init(sensor_frizz_init);
module_exit(sensor_frizz_exit);

MODULE_AUTHOR("Kelly Kang <jianyun.kang@ingenic.com>");
MODULE_DESCRIPTION("FRIZZ Input Driver");
MODULE_LICENSE("GPL");
