#include "remote.h"
#include "control.h"
#include "safe.h"
#include "camera.h"

#define crBegin static int state=0; switch(state) { case 0:
#define crReturn(x) do { state=__LINE__; return x; \
                         case __LINE__:; } while (0)
#define crFinish }

#define get(ptr) if(!getc((ptr))) goto timeout;

//#define TFCR_DEBUG
#ifdef TFCR_DEBUG
#define debug(fmt, ...)   rt_kprintf(fmt, ##__VA_ARGS__)
#else
#define debug(fmt, ...)
#endif

union tfcr_pack
{
	struct tfcr_ack ack;
	struct tfcr_common head;
	struct tfcr_ping ping;
	struct tfcr_task task;
	struct tfcr_get_value get;
	struct tfcr_value value;
	char buf[20];
}pack,send;

const uint16_t head=TFCR_HEAD;

static rt_thread_t remote_thread;
static rt_device_t uart;
static struct rt_semaphore remote_sem;

rt_bool_t tfrc_con=RT_FALSE;

static rt_err_t byte_recv(rt_device_t dev, rt_size_t size)
{
	rt_sem_release(&remote_sem);
	return RT_EOK;;
}

static rt_bool_t getc(char * ch)
{
	if((rt_sem_take(&remote_sem,RT_TICK_PER_SECOND/2))==RT_EOK)
	{
		rt_device_read(uart,0,ch,1);
		debug("%02X ",*ch);
		return RT_TRUE;
	}
	return RT_FALSE;
}

static uint8_t checksum(char * data,rt_size_t size)
{
	uint8_t sum=0;
	for(int j=0;j<size;j++)
		sum+=data[j];
	return sum;
}

static void send_error(uint16_t index,uint8_t code)
{
	send.ack.head=head; 
	send.ack.index=index;
	send.ack.type=TFCR_TYPE_ACK;
	send.ack.code=code;
	send.ack.checksum=checksum(send.buf,sizeof(struct tfcr_ack)-1);
	
	rt_device_write(uart,0,send.buf,sizeof(struct tfcr_ack));
}

static void send_ack(uint16_t index)
{
	send_error(index,0);
}

static void send_value(uint16_t index,uint8_t id)
{
	send.value.head=head;
	send.value.index=index;
	send.value.type=TFCR_TYPE_VALUE;
	send.value.id=id;
	
	switch(id)
	{
		default:
		case 0:
			send.value.value1=ahrs.degree_pitch;
			send.value.value2=ahrs.degree_roll;
			send.value.value3=ahrs.degree_yaw;
			break;
		case 1:
			send.value.value1=ahrs.x;
			send.value.value2=ahrs.y;
			send.value.value3=ahrs.height;
		case 2:
			send.value.value1=recv.pack.angle_error;
			send.value.value2=recv.pack.middle_error;
			send.value.value3=recv.pack.linestate;
			break;
	}
	
	send.value.checksum=checksum(send.buf,sizeof(struct tfcr_value)-1);
	
	rt_device_write(uart,0,send.buf,sizeof(struct tfcr_value));
}

void remote_thread_entry(void* parameter)
{
	char * ptr;
	u8 lost=0;
	rt_tick_t last=rt_tick_get();
	while(1)
	{
		ptr=pack.buf;
		
		get(ptr++);
		if(pack.buf[0]!=(head&0xFF))
			goto wrong;
		
		get(ptr++);
		if(pack.head.head!=head)
			goto wrong;
		
		while(ptr-pack.buf<sizeof(struct tfcr_common))
			get(ptr++);
		
		if(!TFCR_IS_PACK_TYPE(pack.head.type))
			goto wrong;
		//rt_kprintf("head ok %d\n",sizeof(struct tfcr_common));
			
		switch(pack.head.type)
		{
			case TFCR_TYPE_PING:
				while(ptr-pack.buf<sizeof(struct tfcr_ping))
					get(ptr++);
				
				if(checksum(pack.buf,sizeof(struct tfcr_ping)-1)!=pack.ping.checksum)
					goto wrong;
				
				send_ack(pack.ping.index);
				debug("ping %05d\n",pack.ping.index);
				
				if(!tfrc_con)
					rt_kprintf("tfcr connected.\n");
				
				tfrc_con=RT_TRUE;
				break;
			case TFCR_TYPE_TASK:
				while(ptr-pack.buf<sizeof(struct tfcr_task))
					get(ptr++);
				
				if(checksum(pack.buf,sizeof(struct tfcr_task)-1)!=pack.task.checksum)
					goto wrong;
				
				if(rt_strcasecmp(pack.task.name,"mayday")==0)
				{
					excute_task("mayday");
					disarm();
					send_ack(pack.task.index);
					break;
				}
				
				if(pack.task.state==1)
				{
					fc_task * task=find_task(pack.task.name);
					
					if(task==RT_NULL)
					{
						send_error(pack.task.index,0xff);
						rt_kprintf("no task %s!\n",pack.task.name);
						break;
					}
					
					u8 result= check_safe(task->depend);
					if(result!=0)
					{
						send_error(pack.task.index,result);
						break;
					}
					
					if(!excute_task(pack.task.name))
					{
						send_error(pack.task.index,0xff);
						break;
					}
					arm(task->depend);
					send_ack(pack.task.index);
				}
				else
				{
					excute_task("wait");
					send_ack(pack.task.index);
					rt_kprintf("stop task %s.\n",pack.task.name);
				}
				break;
			case TFCR_TYPE_GET_VALUE:
				while(ptr-pack.buf<sizeof(struct tfcr_get_value))
					get(ptr++);
				
				if(checksum(pack.buf,sizeof(struct tfcr_get_value)-1)!=pack.get.checksum)
					goto wrong;
				
				send_value(pack.get.index,pack.get.id);
				
				break;
		}
		
		last=rt_tick_get();
		lost=0;
		continue;
		
		timeout:
		if(tfrc_con)
		{
			if(lost<3)
			{
				rt_kprintf("tfcr time out.\n");
				lost++;
			}
			else
			{
				rt_kprintf("tfcr lost connect.\n");
				tfrc_con=RT_FALSE;
			}
		}
		continue;
		
		wrong:
		rt_kprintf("wrong\n");
		if(rt_tick_get()-RT_TICK_PER_SECOND/2>last)
		{
			rt_tick_t last=rt_tick_get();
			goto timeout;
		}
	}
}

rt_err_t remote_init(const char * uart_name)
{
	uart = rt_device_find(uart_name);

	RT_ASSERT(uart != RT_NULL);

	rt_device_set_rx_indicate(uart, byte_recv);

	rt_device_open(uart, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_STREAM | RT_DEVICE_FLAG_INT_RX);
	
	rt_sem_init(&remote_sem,"remote",0,RT_IPC_FLAG_FIFO);
	
	remote_thread = rt_thread_create("remote",remote_thread_entry,RT_NULL,512,9,3);

	rt_thread_startup(remote_thread);
	
	return RT_EOK;
}
