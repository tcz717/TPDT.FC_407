#include "stm32_iic.h"
#include "stm32f4xx.h"                  // Device header
#include <rtthread.h>

#define  SCL_H         GPIOB->BSRRL = GPIO_Pin_6  
#define  SCL_L         GPIOB->BSRRH = GPIO_Pin_6  
#define  SDA_H         GPIOB->BSRRL = GPIO_Pin_7  
#define  SDA_L         GPIOB->BSRRH = GPIO_Pin_7 

#define  SCL_read      GPIOB->IDR  & GPIO_Pin_6  
#define  SDA_read      GPIOB->IDR  & GPIO_Pin_7  




//IIC专用延时函数
static void I2C_delay(void)
{
    volatile int i = 7;
    while (i)
         i--;
}

 
//IIC接口初始化
void i2cInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	SCL_H;
	SDA_H;
}



static bool I2C_Start(void)
{
    SDA_H;
    SCL_H;
    I2C_delay();
    if (!SDA_read)
        return false;
    SDA_L;
    I2C_delay();
    if (SDA_read)
        return false;
    SDA_L;
    I2C_delay();
    return true;
}


static void I2C_Stop(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SDA_H;
    I2C_delay();
}


static void I2C_Ack(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
}


static void I2C_NoAck(void)
{
    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
}


static bool I2C_WaitAck(void)
{
    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();
    if (SDA_read) 
    {
        SCL_L;
        return false;
    }
    SCL_L;
    return true;
}


static void I2C_SendByte(uint8_t byte)
{
    uint8_t i = 8;
    while (i--) 
    {
        SCL_L;
        I2C_delay();
        if (byte & 0x80)
            SDA_H;
        else
            SDA_L;
        byte <<= 1;
        I2C_delay();
        SCL_H;
        I2C_delay();
    }
    SCL_L;
}

 
static uint8_t I2C_ReceiveByte(void)
{
    uint8_t i = 8;
    uint8_t byte = 0;

    SDA_H;
    while (i--)
    {
        byte <<= 1;
        SCL_L;
        I2C_delay();
        SCL_H;
        I2C_delay();
        if (SDA_read) 
        {
            byte |= 0x01;
        }
    }
    SCL_L;
    return byte;
}


bool i2cWriteBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data)
{
    int i;
    if (!I2C_Start())
        return false;
    I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
    if (!I2C_WaitAck()) 
    {
        I2C_Stop();
        return false;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    for (i = 0; i < len; i++) 
    {
        I2C_SendByte(data[i]);
        if (!I2C_WaitAck()) 
        {
            I2C_Stop();
            return false;
        }
    }
    I2C_Stop();
    return true;
}


int8_t i2cwrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data)
{
	//rt_enter_critical();
	if(i2cWriteBuffer(addr,reg,len,data))
	{
		//rt_critical_level();
		return TRUE;
	}
	else
	{
		//rt_critical_level();
		return FALSE;
	}
	//return FALSE;
}


int8_t i2cread(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	//rt_enter_critical();
	if(i2cRead(addr,reg,len,buf))
	{
		//rt_critical_level();
		return TRUE;
	}
	else
	{
		//rt_critical_level();
		return FALSE;
	}
	//return FALSE;
}


bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
{
    if (!I2C_Start())
        return false;
    I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
    if (!I2C_WaitAck()) 
    {
        I2C_Stop();
        return false;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    I2C_SendByte(data);
    I2C_WaitAck();
    I2C_Stop();
    return true;
}


bool i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!I2C_Start())
        return false;
    I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
    if (!I2C_WaitAck()) 
    {
        I2C_Stop();
        return false;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(addr << 1 | I2C_Direction_Receiver);
    I2C_WaitAck();
    while (len) 
    {
        *buf = I2C_ReceiveByte();
        if (len == 1)
            I2C_NoAck();
        else
            I2C_Ack();
        buf++;
        len--;
    }
    I2C_Stop();
    return true;
}


uint16_t i2cGetErrorCounter(void)
{
    // TODO maybe fix this, but since this is test code, doesn't matter.
    return 0;
}




