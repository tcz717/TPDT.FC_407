#include "adns3080.h"
#include "stm32_spi.h"
#include "srom.h"
#include "hardtimer.h"
#include "ahrs.h"

#define CS0 GPIO_Pin_3
#define IO6 GPIO_Pin_6
#define TEST(EX) if((EX)!=RT_EOK){rt_kprintf("(%s) test failed at %s:%d \n", #EX, __FUNCTION__, __LINE__);has_adns3080=RT_FALSE;return -RT_EIO;}

struct rt_spi_device spi_adns3080;
rt_bool_t has_adns3080 = RT_FALSE;
rt_thread_t adns3080_thread;

s32 opx, opy;
s32 optc_dx, optc_dy;

FINSH_VAR_EXPORT(opx, finsh_type_int, optical flow sum x)
FINSH_VAR_EXPORT(opy, finsh_type_int, optical flow sum y)

rt_err_t adns3080_send_then_recv(struct rt_spi_device *device,
	const void           *send_buf,
	rt_size_t             send_length,
	void                 *recv_buf,
	rt_size_t             recv_length,
	u16					us)
{
	rt_err_t result;
	struct rt_spi_message message;

	RT_ASSERT(device != RT_NULL);
	RT_ASSERT(device->bus != RT_NULL);

	result = rt_mutex_take(&(device->bus->lock), RT_WAITING_FOREVER);
	if (result == RT_EOK)
	{
		if (device->bus->owner != device)
		{
			/* not the same owner as current, re-configure SPI bus */
			result = device->bus->ops->configure(device, &device->config);
			if (result == RT_EOK)
			{
				/* set SPI bus owner */
				device->bus->owner = device;
			}
			else
			{
				/* configure SPI bus failed */
				result = -RT_EIO;
				goto __exit;
			}
		}

		/* send data */
		message.send_buf = send_buf;
		message.recv_buf = RT_NULL;
		message.length = send_length;
		message.cs_take = 1;
		message.cs_release = 0;
		message.next = RT_NULL;

		result = device->bus->ops->xfer(device, &message);
		if (result == 0)
		{
			result = -RT_EIO;
			goto __exit;
		}

		delay_us(TIM6, us);

		/* recv data */
		message.send_buf = RT_NULL;
		message.recv_buf = recv_buf;
		message.length = recv_length;
		message.cs_take = 0;
		message.cs_release = 1;
		message.next = RT_NULL;

		result = device->bus->ops->xfer(device, &message);
		if (result == 0)
		{
			result = -RT_EIO;
			goto __exit;
		}

		result = RT_EOK;
	}
	else
	{
		return -RT_EIO;
	}

__exit:
	rt_mutex_release(&(device->bus->lock));

	return result;
}

static rt_err_t write_reg(u8 reg, u8 value)
{
	u8 wr = reg | 0x80;
	delay_us(TIM6, 50);
	return rt_spi_send_then_send(&spi_adns3080, &wr, 1, &value, 1);
}

static u8 read_reg(u8 reg)
{
	u8 value;
	delay_us(TIM6, 50);
	if (adns3080_send_then_recv(&spi_adns3080, &reg, 1, &value, 1, 75) == RT_EOK)
		return value;
	else
		return 0;
}

static void adns3080_select(u8 state)
{
	GPIO_WriteBit(GPIOD, CS0, state ? Bit_RESET : Bit_SET);
}
static struct spi_cs adns3080_select_cs = { adns3080_select };

static void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	GPIO_InitStructure.GPIO_Pin = CS0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = IO6;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_ResetBits(GPIOE, IO6);
	adns3080_select(0);
}

static void reset()
{
	GPIO_ResetBits(GPIOE, IO6);
	rt_thread_delay(RT_TICK_PER_SECOND * 5 / 1000 + 1);
	GPIO_SetBits(GPIOE, IO6);
	rt_thread_delay(RT_TICK_PER_SECOND * 5 / 1000 + 1);
	GPIO_ResetBits(GPIOE, IO6);//Âö³åÐÅºÅ
}

static rt_err_t write_srom()
{
	TEST(write_reg(0x20, 0x44));
	TEST(write_reg(0x23, 0x07));
	TEST(write_reg(0x24, 0x88));

	rt_thread_delay(1);

	TEST(write_reg(SROM_Enable, 0x18));

	delay_us(TIM6, 40);
	for (u16 i = 0;i < sizeof(SROM);i++)
	{
		TEST(write_reg(SROM_Load, SROM[i]));
		delay_us(TIM6, 11);
	}
	delay_us(TIM6, 100);
	return RT_EOK;
}

u8 read_busy(void)
{
	return read_reg(Extended_Config) & 0x80;
}

void adns3080_thread_entry(void* parameter)
{
	rt_kprintf("start adns3080\n");

	opx = 0;
	opy = 0;

	while (1)
	{
		const u8 cmd = Motion_Burst;
		u8 data[3];
		adns3080_send_then_recv(&spi_adns3080, &cmd, 1, data, 3, 75);

		optc_dx = (s8)data[1];
		optc_dy = (s8)data[2];

		opx += optc_dx;
		opy += optc_dy;
		
		ahrs.x=opx;
		ahrs.y=opy;
		ahrs.dx=optc_dx*0.5f+ahrs.dx*0.5f;
		ahrs.dy=optc_dy*0.5f+ahrs.dy*0.5f;

		//		if((!(data[0]&0x10))&&(data[0]&0x80))
		//		{
		//			rt_kprintf("optc:\t%d\t%d\t%d\t%d\n",optc_dx,optc_dy,optc_sumx,optc_sumy);
		//		}
		rt_event_send(&ahrs_event, AHRS_EVENT_ADNS3080);
		rt_thread_delay(RT_TICK_PER_SECOND / 100);
	}
}

rt_err_t adns3080_Init(void)
{
	u8 id;
	GPIO_Configuration();

	timer_init(TIM6);

	if (rt_spi_bus_attach_device(&spi_adns3080, "cs_d00", "spi3", &adns3080_select_cs) != RT_EOK)
	{
		rt_kprintf("cs_d03 device attach wrong!\r\n");
		has_adns3080 = RT_FALSE;
		return -RT_ENOSYS;
	}

	reset();
	rt_thread_delay(RT_TICK_PER_SECOND * 10 / 1000 + 1);

	if (write_srom() != RT_EOK)
	{
		rt_kprintf("srom load faild!\r\n");
		has_adns3080 = RT_FALSE;
		return -RT_EIO;
	}

	TEST(write_reg(Configuration_bits, 0x10));
	rt_thread_delay(2);

	TEST(write_reg(Extended_Config, 0x01));
	rt_thread_delay(2);

	if (!read_busy())
	{
		rt_thread_delay(1);

		TEST(write_reg(Frame_Period_Max_Bound_Lower, 0x40));
		TEST(write_reg(Frame_Period_Max_Bound_Upper, 0x1f));
	}

	TEST(write_reg(Motion_Clear, 0xff));

	id = read_reg(Product_ID);
	rt_kprintf("adns3080:0x%02x\n", id);

	id = read_reg(SROM_ID);
	rt_kprintf("adns3080 srom:0x%02x\n", id);
 
	if (id != 0x51)
	{
		has_adns3080 = RT_FALSE;
		rt_kprintf("adns3080 not find\n");
		return RT_EEMPTY;
	}

	adns3080_thread = rt_thread_create("adns3080", adns3080_thread_entry, RT_NULL, 768, 6, 2);

	if (adns3080_thread == RT_NULL)
	{
		adns3080_thread = RT_FALSE;
		rt_kprintf("HMC5883 thread no Memory\n");
		return RT_ENOMEM;
	}

	rt_thread_startup(adns3080_thread);
	has_adns3080 = RT_TRUE;
	ahrs_state.adns3080=RT_EOK;
	return RT_EOK;
}
