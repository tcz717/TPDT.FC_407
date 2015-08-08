#ifndef TFCR_H
#define TFCR_H
#include "stm32f4xx.h"

#define TFCR_TYPE_PING       0x00
#define TFCR_TYPE_TASK       0x01
#define TFCR_TYPE_ACK        0xFF

#define TFCR_HEAD            0xABCD

#define TFCR_IS_PACK_TYPE(type) 	(((type)==TFCR_TYPE_PING)||		\
									((type)==TFCR_TYPE_TASK)||		\
									((type)==TFCR_TYPE_ACK))

#pragma pack(push)
#pragma pack(1)
//#pragma pack is not supputed in cubesuite++
struct tfcr_common
{
    uint16_t head;
    uint8_t type;
    uint16_t index;
};


//remote only
struct tfcr_ping
{
    uint16_t head;
    uint8_t type;
    uint16_t index;
    uint8_t checksum;
};

struct tfcr_task
{
	uint16_t head;
	uint8_t type;
	uint16_t index;
	
	char name[8];//8 bytes task name, "default" is main task
	uint8_t state;//1 for start and 0 for stop
	
	uint8_t checksum;
};

//controller only
struct tfcr_ack
{
	uint16_t head;
	uint8_t type;
	uint16_t index;
	uint8_t code;//0 for ok
	uint8_t checksum;
};
#pragma pack(pop)

#define TFCR_ACK_TIMEOUT			300/*ms*/
#define TFCR_PING_TIME				100/*ms*/
#define TFCR_PING_TIMEOUT			500/*ms*/
//the interval between two packages should be at least TFCR_PACKAGE_INTERVAL_MIN ms
#define TFCR_PACKAGE_INTERVAL_MIN	50/*ms*/


#endif
