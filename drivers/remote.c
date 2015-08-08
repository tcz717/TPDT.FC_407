#include "remote.h"

#define crBegin static int state=0; switch(state) { case 0:
#define crReturn(x) do { state=__LINE__; return x; \
                         case __LINE__:; } while (0)
#define crFinish }

#define get(ptr) if(!getc((ptr))) goto timeout;

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
	char buf[16];
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

static void send_ack(uint16_t index)
{
	send.ack.head=head;
	send.ack.index=index;
	send.ack.type=TFCR_TYPE_ACK;
	send.ack.code=0;
	send.ack.checksum=checksum(send.buf,sizeof(struct tfcr_ack)-1);
	
	rt_device_write(uart,0,send.buf,sizeof(struct tfcr_ack));
}

void remote_thread_entry(void* parameter)
{
	char * ptr;
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
		}
		
		last=rt_tick_get();
		continue;
		
		timeout:
		if(tfrc_con)
			rt_kprintf("tfcr time out.\n");
		tfrc_con=RT_FALSE;
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
