#ifndef __SENSOR_HUB_H
#define __SENSOR_HUB_H
#include <linux/cdev.h>

#include "sensorhub_common.h"

#define   DEVICE_NAME                              "sensor_hub"
#define   DEVICE_NUM                               (1)       /*!< Number of device */
#define   SENSORHUB_STM32_FLASH_MEMORY             0x08000000
#define   SENSORHUB_STM32_COMMANDS_NUM             0x11


#define   SENSORHUB_ACK                            (0x79)
#define   SENSORHUB_NACK                           (0x1F)
#define   SENSORHUB_BUSY                           (0x76)

//#define	    EACH_CYCLE_SIZE                           5

enum boot_mode{
	BOOT_MODE_FLASH = 0,
	BOOT_MODE_SYSTEM_MEMRY,

};

typedef enum {
	SENSORHUB_ERR_OK = 0,
	SENSORHUB_ERR_UNKNOWN,	/* Generic error */
	SENSORHUB_ERR_NACK,
	SENSORHUB_ERR_NO_CMD,	/* Command not available in bootloader */

} sensor_hub_err_t;

typedef enum {
	PORT_ERR_OK = 0,
	PORT_ERR_NODEV,		/* No such device */
	PORT_ERR_TIMEDOUT,	/* Operation timed out */
	PORT_ERR_UNKNOWN,

} port_err_t;

struct sensor_hub_cmd {
	uint8_t get;
	uint8_t gvr;
	uint8_t gid;
	uint8_t rm;
	uint8_t go;
	uint8_t wm;
	uint8_t er; /* this may be extended erase */
	uint8_t wp;
	uint8_t uw;
	uint8_t rp;
	uint8_t ur;
	uint8_t	crc;
};

typedef struct sensor_hub_cmd       sensor_hub_cmd_t;
typedef struct sensor_hub_data      sensor_hub_data_t;

struct sensor_hub_data {
	struct semaphore                  sem;   /*< Exclusive access control */
	struct mutex                      lock;
	struct input_dev                  *input_dev;  /*< represents an input device */
	struct device                     *dev;
	struct sensorhub_platform_data   *pdata;
	const struct sensor_hub_bus_ops   *bus_ops;

	struct cdev                       cdev;       /*!< cdev information */
	struct class                      *class;     /*!< class information */
	dev_t                             devt;      /*!< device information */
	wait_queue_head_t                 rx_waiter;
	wait_queue_head_t                 tx_waiter;
	struct work_struct                work_read;
	struct work_struct                work_write;
	struct workqueue_struct           *workqueue;

	sensor_hub_cmd_t                   *cmd;
	unsigned int                      irq;
	unsigned int                      irq_state;
	char                              *vcc_name;
	struct regulator                  *vcc_regulator;

	struct task_struct * my_thread;

	BUFF_T	rx_buff;
	BUFF_T	tx_buff;

};


struct sensor_hub_bus_ops {
	u16 bustype;
	int (*read)(struct device *dev, void *buf, size_t size);
	int (*write)(struct device *dev,void *buf, size_t size);
};

static inline int sensor_hub_read(struct sensor_hub_data *s_data, void *buf, size_t size)
{
	return s_data->bus_ops->read(s_data->dev, buf, size);
}

static inline int sensor_hub_write(struct sensor_hub_data *s_data, void *buf, size_t size)
{
	return s_data->bus_ops->write(s_data->dev, buf, size);
}

int sensor_hub_core_init(const struct sensor_hub_bus_ops *bus_ops, struct device *dev);
int sensor_hub_core_release(struct device *dev);


#endif /* __SENSOR_HUB_H */
