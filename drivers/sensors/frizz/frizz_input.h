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

/**
 * @defgroup
 * @brief
 *
 * @{
 *      @file     mpu6500_input.h
 *      @brief
 */

#ifndef __MPU6500_INPUT_H_
#define __MPU6500_INPUT_H_

#define MPU_NAME "mpu6500"
#define DEFAULT_MPU_SLAVEADDR		0x68

#define MPU6500_INPUT_DRIVER "mpu6500_input"

#define MPU6500_ID	0x70
#define MPU9250_ID      0x71      /* unique WHOAMI */
#define MPU6515_ID	0x74

#define FIFO_HW_SIZE		(1024)

#define NUM_EXT_SLAVES		(4)

#define ACC_CAL_TIME 20
#define ACC_IDEAL 1024
#define ACC_CAL_DIV 16

#define MPU6500_MODE_NORMAL	0
#define MPU6500_MODE_SLEEP	2
#define MPU6500_MODE_WAKE_UP	3

#ifndef MIN
#define	MIN(a, b)		(((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#define	MAX(a, b)		(((a) > (b)) ? (a) : (b))
#endif

#define MPU6500_SENSOR_ACCEL	(0x0f)
#define MPU6500_SENSOR_GYRO	(0xf0)
#define MPU6500_SENSOR_LPSO	(0x0100)

#define IS_LP_ENABLED(sensors) (sensors & (MPU6500_SENSOR_LPSO ))
#define LP_MASK(sensors) (sensors & (MPU6500_SENSOR_LPSO ))
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
struct frizz_i2c_dev_data {
  int gpio_interrupt;  /*!< frizz interrupt to host CPU */
  int gpio_reset;      /*!< frizz reset */
  struct regulator    *vio; /*!< frizz power source 1.8v */
};

struct frizz_input_data {
    struct semaphore   semaphore; /*!< Exclusive access control */
	struct i2c_client *client;
	struct input_dev *accel_input;
	struct input_dev *gyro_input;
	struct input_dev *mcu_input;
	struct motion_int_data mot_data;
	struct mutex mutex;
	struct wake_lock reactive_wake_lock;
	atomic_t enable;
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

#ifdef CONFIG_INPUT_FRIZZ_POLLING
	struct delayed_work accel_work;
	struct delayed_work gyro_work;
#endif
	struct device *accel_sensor_device;
	struct device *gyro_sensor_device;
	struct device *mcu_device;
	s16 acc_cal[3];
	bool factory_mode;
	struct frizz_i2c_dev_data *pdata;
};

struct sensor_virtual {
	const char *name;
};

struct sensor_virtual_platform_data {
	int		num_sensor;
	struct sensor_virtual *sensor_virtual;
};
int frizz_read_accel(struct frizz_input_data *data,struct mpu6500_v *);
#endif
