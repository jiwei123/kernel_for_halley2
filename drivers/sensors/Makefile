#
# Makefile for the sensors drivers.
#

# Each configuration option enables a list of files.

obj-$(CONFIG_SENSORS)		+= sensors_core.o

obj-$(CONFIG_SENSORS_TMD27723)	+= tmd27723.o
obj-$(CONFIG_SENSORS_AK09911C)	+= ak09911c.o
obj-$(CONFIG_SENSOR_VIRTUAL)	+= mpu6500_input_virtual.o
# 6-axis_sensor
obj-$(CONFIG_INPUT_MPU6500)     += mpu6500/
obj-$(CONFIG_INPUT_FRIZZ)     	+= frizz/
