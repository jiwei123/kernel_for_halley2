#ifndef __SENSORHUB_CORE_H__
#define __SENSORHUB_CORE_H__



struct sensorhub_platform_data {
	unsigned int gpio_irq;
	unsigned int gpio_reset;
	unsigned int gpio_response;
	unsigned int gpio_wakeup_sensorhub;
	unsigned int gpio_wakeup_host;
	char * vcc_name;

};
#endif
