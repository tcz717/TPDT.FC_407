//bmp085
#include "stm32f4xx.h"
#include "bmp085.h"
#include "board.h"
#include "i2c1.h"
#include "math.h"
#include <rtthread.h>

#define	BMP085_SlaveAddress   0xee	 
#define BMP085_CMD_Register   0xF4   
#define BMP085_CMD_ReadTemp   0x2E
#define BMP085_CMD_ReadPressure   0x34
#define BMP085_Val   0xF6

#define BMP085_EOC_PIN          GPIO_Pin_1
#define BMP085_EOC_GPIO_PORT    GPIOC
#define BMP085_EOC_GPIO_CLK     RCC_APB2Periph_GPIOC

#define BMP085_EOC_EXIT_IRQn			EXTI1_IRQn
#define BMP085_EOC_GPIO_PortSource		GPIO_PortSourceGPIOC
#define BMP085_EOC_GPIO_PinSource     	GPIO_PinSource1
#define BMP085_EOC_EXTI_Line       		EXTI_Line1

#define OSS 0	// Oversampling Setting (note: code is not set up to use other OSS values)

s16 ac1;
s16 ac2; 
s16 ac3; 
u16 ac4;
u16 ac5;
u16 ac6;
s16 b1; 
s16 b2;
s16 mb;
s16 mc;
s16 md;

static struct rt_i2c_bus_device * i2c_device;
static struct rt_semaphore bmp085_eoc;

void get_bmp085()
{
	struct bmp085_data data;
	
	if(bmp085_read(&data)==RT_EOK)
	{
		rt_kprintf("p:%d t:%d h:%d\n",
					data.pressure,data.temperature/10,(u16)data.height);
		return;
	}
	rt_kprintf("read time out!\n");
}
FINSH_FUNCTION_EXPORT(get_bmp085, get bmp085 data)

u16 Multiple_read(u8 ST_Address)
{
	u8 tmp[2];
	if(i2c_register_read(i2c_device,BMP085_SlaveAddress,ST_Address,tmp,2)!=RT_EOK)
		return 0;
	return tmp[0]<<8 | tmp[1];
}
static void RCC_Configuration(void)
{
	RCC_APB2PeriphClockCmd(BMP085_EOC_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);
}
static void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	GPIO_InitStructure.GPIO_Pin = BMP085_EOC_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(BMP085_EOC_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_EXTILineConfig(BMP085_EOC_GPIO_PortSource,BMP085_EOC_GPIO_PinSource);
	EXTI_InitStructure.EXTI_Line=EXTI_Line1;
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising;
	
	EXTI_Init(&EXTI_InitStructure);
}
static void NVIC_Configuration()
{
    NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = BMP085_EOC_EXIT_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

rt_err_t bmp085_read(struct bmp085_data *data)
{
	u8 tmp;
	u32 ut;
	u32 up;
	s32 x1, x2, b5, b6, x3, b3, p;
	u32 b4, b7;
	
	tmp=BMP085_CMD_ReadTemp;
	rt_enter_critical();
	bmp085_eoc.value=0;
	i2c_register_write(i2c_device,BMP085_SlaveAddress,
						BMP085_CMD_Register,&tmp,1);
	rt_exit_critical();
	
	if(rt_sem_take(&bmp085_eoc,RT_TICK_PER_SECOND)!=RT_EOK)
		return RT_ETIMEOUT;
	
	data->temperature=Multiple_read(BMP085_Val);
	
	tmp=BMP085_CMD_ReadPressure;
	rt_enter_critical();
	bmp085_eoc.value=0;
	i2c_register_write(i2c_device,BMP085_SlaveAddress,
						BMP085_CMD_Register,&tmp,1);
	rt_exit_critical();
	
	if(rt_sem_take(&bmp085_eoc,RT_TICK_PER_SECOND)!=RT_EOK)
		return RT_ETIMEOUT;
	
	data->pressure=Multiple_read(BMP085_Val);
	
	ut=data->temperature;
	up=data->pressure;
	x1 = (((s32)ut - (s32)ac6)*(s32)ac5) >> 15;
	x2 = ((s32) mc << 11) / (x1 + md);
	b5 = x1 + x2;
	data->temperature = ((b5 + 8) >> 4);

	b6 = b5 - 4000;
	// Calculate B3
	x1 = (b2 * (b6 * b6)>>12)>>11;
	x2 = (ac2 * b6)>>11;
	x3 = x1 + x2;
	b3 = (((((s32)ac1)*4 + x3)<<OSS) + 2)>>2;

	// Calculate B4
	x1 = (ac3 * b6)>>13;
	x2 = (b1 * ((b6 * b6)>>12))>>16;
	x3 = ((x1 + x2) + 2)>>2;
	b4 = (ac4 * (u32)(x3 + 32768))>>15;

	b7 = ((u32)(up - b3) * (50000>>OSS));
	if (b7 < 0x80000000)
	p = (b7<<1)/b4;
	else
	p = (b7/b4)<<1;

	x1 = (p>>8) * (p>>8);
	x1 = (x1 * 3038)>>16;
	x2 = (-7357 * p)>>16;
	data->pressure = p+((x1 + x2 + 3791)>>4);
	data->height=44330.0*(1.0-pow(data->pressure/101325.0,0.1903));
	
	return RT_EOK;
}

rt_err_t bmp085_init(const char * i2c_bus_device_name)
{
    i2c_device = rt_i2c_bus_device_find(i2c_bus_device_name);
    if(i2c_device == RT_NULL)
    {
        rt_kprintf("i2c bus device %s not found!\r\n", i2c_bus_device_name);
        return -RT_ENOSYS;
    }
	
	RCC_Configuration();
	NVIC_Configuration();
	GPIO_Configuration();
	
	ac1 = Multiple_read(0xAA);
	if(!ac1)
	{
		rt_kprintf("Not found BMP085\n");
		return RT_EIO;
	}
	ac2 = Multiple_read(0xAC);
	ac3 = Multiple_read(0xAE);
	ac4 = Multiple_read(0xB0);
	ac5 = Multiple_read(0xB2);
	ac6 = Multiple_read(0xB4);
	b1 =  Multiple_read(0xB6);
	b2 =  Multiple_read(0xB8);
	mb =  Multiple_read(0xBA);
	mc =  Multiple_read(0xBC);
	md =  Multiple_read(0xBE);
	
	if(!(ac2|ac3|ac4|ac5|ac6|b1|b2|mb|mc|md))
	{
		rt_kprintf("Wrong BMP085 Value\n");
		return RT_EIO;
	}
	
	rt_sem_init(&bmp085_eoc,"BMP_EOC",0,RT_IPC_FLAG_FIFO);
	
	return RT_EOK;
}

void EXTI1_IRQHandler()
{
	if(EXTI_GetITStatus(BMP085_EOC_EXTI_Line)==SET)
	{
		rt_interrupt_enter();
		
		rt_sem_release(&bmp085_eoc);
		EXTI_ClearITPendingBit(BMP085_EOC_EXTI_Line);
		
		rt_interrupt_leave();
	}
}
