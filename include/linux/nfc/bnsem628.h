#ifndef _BNSEM628_H
#define _BNSEM628_H

#define BNSEM628_NAME "bnsem628"
/*两个字节的数据长度, 最多支持261字节的数据读写*/
#define MAX_BUFFER_SIZE 261

struct bnsem628_platform_data {
	unsigned int en_gpio;	/*for enable nfc*/
	void (*power_down_gpio_set)(void);	/*for setting the gpio when power down*/
	void (*power_on_gpio_set)(void);	/*for setting the gpio when powen on*/
};

enum{
	CMD_NFC_BNSEM628_POWER_ON,
	CMD_NFC_BNSEM628_POWER_OFF,
	CMD_NFC_BNSEM628_MODE_CONNECT,
	CMD_NFC_BNSEM628_MODE_UNCONNECT,
	CMD_NFC_BNSEM628_COUNT,
};

enum{
	BNSEM628_MODE_CONNECT,
	BNSEM628_MODE_UNCONNECT,
};

#endif
