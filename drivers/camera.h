#ifndef __CAMERA_H__
#define __CAMERA_H__

#include "stm32f4xx.h"
#include "board.h"
#include <rtdevice.h>
#include <rthw.h>
#include <rtthread.h>
#include "ahrs.h"

enum
{
	LINE_STRAIGHT,//直线
				  //	LINE_TURN_LEFT_45,
	LINE_TURN_LEFT_90,
	//	LINE_TURN_RIGHT_45,
	LINE_TURN_RIGHT_90,
	LINE_END,
	LINE_LOSTfromLEFT,
	LINE_LOSTfromRIGHT,
	LINE_LOSTfromBOTTOM,
	LINE_LOST_ERROR,
};
#define REPORT_PACKAGE_HEAD 0x23
typedef struct
{
	uint8_t head;
	uint8_t frame_cnt;//帧计数
	uint8_t linestate;
	uint8_t dummy0;//为了对齐32位用
	float angle_error;
	//朝向误差
	//                  0 degree
	//                  A
	//                  A
	//                  A
	//                  A
	//                  A
	//                  A
	//-90degree ←←←←←←←←A→→→→→→→→→→→→ +90degree
	int8_t middle_error;
	//离中心线的距离//负值代表线在飞机左边（即飞机需要左移）
	uint8_t dummy1;
	uint8_t dummy2;
	uint8_t checksum;
} report_package_type;

union data_pack
{
	report_package_type pack;
	char data[sizeof(report_package_type)];
};

extern union data_pack recv;

rt_err_t check_safe(rt_uint32_t checklist);

#endif
