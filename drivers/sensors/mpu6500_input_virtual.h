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

struct sensor_virtual {
	const char *name;
};

struct sensor_virtual_platform_data {
	int		num_sensor;
	struct sensor_virtual *sensor_virtual;
};

#endif
