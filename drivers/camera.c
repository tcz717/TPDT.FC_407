#include "stm32f4xx.h"
#include "board.h"
#include <rtdevice.h>
#include <rthw.h>
#include <rtthread.h>
#include "ahrs.h"

#define crBegin static int state=0; switch(state) { case 0:
#define crReturn(x) do { state=__LINE__; return x; \
                         case __LINE__:; } while (0)
#define crFinish }

rt_device_t uart;

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

static struct rt_semaphore cam_sem;

union data_pack
{
	report_package_type pack;
	char data[sizeof(report_package_type)];
}recv;

u8 read_state=0;
rt_err_t byte_recv(rt_device_t dev, rt_size_t size)
{
	rt_sem_release(&cam_sem);
	return RT_EOK;;
}

static rt_thread_t camera_thread;

rt_bool_t pack_parser(char ch)
{
	static u8 i = 0;
	static u8 sum = 0;
	crBegin
		while(1)
		{
			if (ch == REPORT_PACKAGE_HEAD)
			{
				i = 0;
				sum = recv.data[i];
				recv.data[i] = ch;
				//rt_kprintf("$ ");
				i++;
				crReturn(RT_FALSE);
				for (;i < sizeof(recv) - 1;i++)
				{
					recv.data[i] = ch;
					sum += recv.data[i];
					//rt_kprintf("* ");
					crReturn(RT_FALSE);
				}
				recv.data[i] = ch;
				//rt_kprintf("~ ");
				crReturn(recv.pack.checksum == sum);
			}
			else
				crReturn(RT_FALSE);
		}
	crFinish
	return RT_FALSE;
}

void camera_thread_entry(void* parameter)
{
	char re;
	while (1)
	{
		rt_sem_take(&cam_sem, RT_WAITING_FOREVER);
		rt_device_read(uart, 0, &re, 1);

//		if (re == REPORT_PACKAGE_HEAD&&read_state == 0)
//		{
//			rt_kprintf("get packet:");
//			rt_kprintf("%02x ", re);
//			read_state++;
//		}
//		else if (read_state)
//		{
//			read_state++;
//			rt_kprintf("%02x ", re);
//			if (read_state == sizeof(report_package_type))
//			{
//				read_state = 0;
//				rt_kprintf("\n");
//			}
//		} 
		
		if (pack_parser(re))
		{
//			rt_kprintf("get packet:#%03d\tstate:%4d\tangle:%4d\tmid:%4d\n"
//				,recv.pack.frame_cnt
//				,recv.pack.linestate
//				,(s32)recv.pack.angle_error
//				,recv.pack.middle_error);
			ahrs_state.camera=RT_EOK;
			rt_event_send(&ahrs_event,AHRS_EVENT_CARMERA);
		}
	}
}

rt_err_t camera_init(const char * uart_name)
{
	uart = rt_device_find(uart_name);

	RT_ASSERT(uart != RT_NULL);

	rt_device_set_rx_indicate(uart, byte_recv);

	rt_device_open(uart, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_STREAM | RT_DEVICE_FLAG_INT_RX);
	
	rt_sem_init(&cam_sem,"cam",0,RT_IPC_FLAG_FIFO);
	
	camera_thread = rt_thread_create("camera",camera_thread_entry,RT_NULL,512,10,3);

	rt_thread_startup(camera_thread);
	
	return RT_EOK;
}
