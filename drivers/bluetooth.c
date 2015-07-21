#include "stm32f10x.h"
#include "board.h"
#include "bluetooth.h"

#include <rtdevice.h>
#include <components.h> 


struct rt_serial_device bluetooth;

static rt_err_t bluetooth_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    USART_InitTypeDef USART_InitStructure;

    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    USART_InitStructure.USART_BaudRate = cfg->baud_rate;

    if (cfg->data_bits == DATA_BITS_8)
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;

    if (cfg->stop_bits == STOP_BITS_1)
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
    else if (cfg->stop_bits == STOP_BITS_2)
        USART_InitStructure.USART_StopBits = USART_StopBits_2;

    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART4, &USART_InitStructure);

    /* Enable USART */
    USART_Cmd(UART4, ENABLE);
    /* enable interrupt */
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);

    return RT_EOK;
}
static rt_err_t bluetooth_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    RT_ASSERT(serial != RT_NULL);
    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        UART_DISABLE_IRQ(UART4_IRQn);
        break;
    case RT_DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        UART_ENABLE_IRQ(UART4_IRQn);
        break;
    }

    return RT_EOK;
}

static int bluetooth_getc(struct rt_serial_device *serial)
{
    int ch;

    RT_ASSERT(serial != RT_NULL);
    ch = -1;
    if (UART4->SR & USART_FLAG_RXNE)
    {
        ch = UART4->DR & 0xff;
    }

    return ch;
}

static int bluetooth_putc(struct rt_serial_device *serial, char c)
{
    RT_ASSERT(serial != RT_NULL);

    while (!(UART4->SR & USART_FLAG_TXE));
    UART4->DR = c;

    return 1;
}

static rt_size_t bluetooth_dma_transmit(struct rt_serial_device *serial, const rt_uint8_t *buf, rt_size_t size, int direction)
{
	DMA_InitTypeDef DMA_InitStructure;
    RT_ASSERT(serial != RT_NULL);
	
	DMA_Cmd(DMA2_Channel5,DISABLE);  
	
	DMA_DeInit(DMA2_Channel5); 
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&UART4->DR);  
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)buf;  
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  
	DMA_InitStructure.DMA_BufferSize = size;  
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;  
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;  
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  
	DMA_Init(DMA2_Channel5,&DMA_InitStructure);  
	DMA_ITConfig(DMA2_Channel5,DMA_IT_TC,ENABLE);
	
	USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);  
	DMA_Cmd(DMA2_Channel5,ENABLE); 

    return size;
}

static const struct rt_uart_ops bluetooth_ops =
{
    bluetooth_configure,
    bluetooth_control,

    bluetooth_putc,
    bluetooth_getc,

    bluetooth_dma_transmit
};

//void bt_set(rt_uint32_t baund)
//{
//	char * result;
//    struct rt_device *device;
//	
//	device=rt_device_find("bt1");
//	
//	GPIO_SetBits(GPIOA,GPIO_Pin_4);
//	
//	rt_kprintf("AT+UART=%d,0,0\n",baund);
//	
//	GPIO_ResetBits(GPIOA,GPIO_Pin_4);
//}

//FINSH_FUNCTION_EXPORT(bt_set, set blutooth rate)

void rt_hw_bluetooth_init(){
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

    GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC| RCC_APB2Periph_AFIO, ENABLE);	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);  
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);  
	
	GPIO_ResetBits(GPIOA,GPIO_Pin_4);

    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		
	NVIC_Init(&NVIC_InitStructure);	

	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel4_5_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
	NVIC_Init(&NVIC_InitStructure);
	
	
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
    config.baud_rate = BAUD_RATE_9600;
	
	bluetooth.config = config;
	bluetooth.ops    = &bluetooth_ops;
	
    rt_hw_serial_register(&bluetooth, "bt1",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                          NULL);
}
void UART4_IRQHandler(void)
{
	rt_interrupt_enter();
    if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
    {
        rt_hw_serial_isr(&bluetooth, RT_SERIAL_EVENT_RX_IND);
        /* clear interrupt */
        USART_ClearITPendingBit(UART4, USART_IT_RXNE);
    }
    if (USART_GetITStatus(UART4, USART_IT_TC) != RESET)
    {
        /* clear interrupt */
        USART_ClearITPendingBit(UART4, USART_IT_TC);
    }

    /* leave interrupt */
    rt_interrupt_leave();
} 
void DMA2_Channel4_5_IRQHandler(void)  
{  
	rt_interrupt_enter();
    if(DMA_GetITStatus(DMA2_IT_TC5) != RESET)
    {
        rt_hw_serial_isr(&bluetooth, RT_SERIAL_EVENT_TX_DMADONE);
        /* clear interrupt */
		DMA_ClearITPendingBit(DMA2_IT_TC5);
    }
    DMA_Cmd(DMA2_Channel5,DISABLE); 

    /* leave interrupt */
    rt_interrupt_leave();
}  
