#include "stm32f4xx.h"
#include "board.h"
#include <rtdevice.h>
#include <rthw.h>
#include <rtthread.h>
#include "ahrs.h"
#include "camera.h"

#define crBegin static int state=0; switch(state) { case 0:
#define crReturn(x) do { state=__LINE__; return x; \
                         case __LINE__:; } while (0)
#define crFinish }

rt_device_t uart;

static struct rt_semaphore cam_sem;

union data_pack recv;

s16 line[8];
s16 angle[8];

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
	ahrs_state.camera=RT_ENOSYS;
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
			if(ahrs_state.camera!=RT_EOK)
				rt_kprintf("find camera.\n");
			ahrs_state.camera=RT_EOK;
			ahrs.line_err=MoveAve_WMA(recv.pack.middle_error,line,4);
			ahrs.angle_err=MoveAve_WMA(recv.pack.angle_error,angle,4);
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
	
	camera_thread = rt_thread_create("camera",camera_thread_entry,RT_NULL,512,6,3);

	rt_thread_startup(camera_thread);
	
	return RT_EOK;
}
