#include "spi2.h"
#include "stm32f4xx.h"
#include "board.h"
#include <rtdevice.h>
#include <rthw.h>
#include <rtthread.h>
#include "spi_flash_w25qxx.h"

#define SPI_FLASH 1

struct rt_spi_bus spi2_bus;

//rt_inline void spi2_select_cs(u16 cs,u8 state)
//{
//	switch(cs)
//	{
//		case SPI_FLASH:
//			PAout(12)=!state;
//			break;
//	}
//}

void flash_select(u8 state)
{
	//PAout(12)=!state;
	GPIO_WriteBit(GPIOD,GPIO_Pin_11,state?Bit_RESET:Bit_SET);
}
static struct spi_cs flash_cs = {flash_select};

struct rt_spi_device spi_flash;

void spi_flash_init(void)
{
	rt_spi_bus_attach_device(&spi_flash,"cs_d11","spi2",&flash_cs);
	
	w25qxx_init("flash0","cs_d11"); 
}

rt_err_t spi2_configure(struct rt_spi_device *device, struct rt_spi_configuration *configuration)
{
	return RT_EOK;
}
rt_uint32_t spi2_xfer(struct rt_spi_device *device, struct rt_spi_message *message)
{
	u32 i;
	const rt_uint8_t * send_ptr = message->send_buf;
	rt_uint8_t * recv_ptr = message->recv_buf;
	
	if(message->cs_take)
	{
		((struct spi_cs *)(device->parent.user_data))->spi_select(1);
	}
	
	for(i=0;i<message->length;i++)
	{
		rt_uint8_t data = 0xFF;

		if(send_ptr != RT_NULL)
		{
			data = *send_ptr++;
		}

		//Wait until the transmit buffer is empty
		while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
		// Send the byte
		SPI_I2S_SendData(SPI2, data);

		//Wait until a data is received
		while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
		// Get the received data
		data = SPI_I2S_ReceiveData(SPI2);

		if(recv_ptr != RT_NULL)
		{
			*recv_ptr++ = data;
		}
	}
	
	if(message->cs_release)
	{
		((struct spi_cs *)(device->parent.user_data))->spi_select(0);
	}
	
	return message->length;
}

static const struct rt_spi_ops spi2_ops=
{
    spi2_configure,
    spi2_xfer,
};

void CS_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD , ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	flash_select(0);
}

void rt_hw_spi2_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	
	CS_Configuration();
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource2,GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource3,GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	//GPIO_SetBits(GPIOB,GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);

	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;   // ?????
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;                        // ???
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;                    // ????8?
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;                           // ????,?????
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;    
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;                            // NSS???????
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;   // 8??,9MHz
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;                   // ????
	SPI_InitStruct.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStruct);

	SPI_Cmd(SPI2, ENABLE);
	
	rt_spi_bus_register(&spi2_bus,"spi2",&spi2_ops);
}
