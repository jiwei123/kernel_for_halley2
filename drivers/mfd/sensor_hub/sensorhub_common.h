#ifndef __SENSORHUB_COMMON_H
#define __SENSORHUB_COMMON_H

#define MAX_PACK_SIZE		50
#define EACH_BLOCK_SIZE	 	5
#define MAX_PACK_BLOCK		(MAX_PACK_SIZE / EACH_BLOCK_SIZE)

#define MAX_BLOCK_COUNT	 	200
#define MAX_BUFF_SIZE 		(MAX_BLOCK_COUNT * EACH_BLOCK_SIZE)

#define   SENSORHUB_MAX_PACK_DATA_SIZE                  (0xFF)

typedef struct {
	char buff[MAX_BUFF_SIZE];
	unsigned int head;
	unsigned int tail;
	unsigned int block_count;

} BUFF_T;


typedef struct {
	unsigned char	type;
	unsigned char	length;
	unsigned short	uuid;
	unsigned char	reserved[2];

} hub_format_header_t;


struct sensor_hub_packet {
	hub_format_header_t hdr;
	unsigned char data[SENSORHUB_MAX_PACK_DATA_SIZE];

};

typedef struct sensor_hub_packet    sensor_hub_packet_t;


#define HEAD_SIZE		sizeof(hub_format_header_t)

#endif /* __SENSORHUB_COMMON_H */
