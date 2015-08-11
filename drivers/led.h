#include <stm32f4xx.h>
#ifndef LED
#define LED
#include <rtthread.h>

#define LED1(TIME) led_period[0]=(TIME)
#define LED2(TIME) led_period[1]=(TIME)
#define LED3(TIME) led_period[2]=(TIME)
#define LED4(TIME) led_period[3]=(TIME)

extern u8 led_period[4];

void LED_init(void);
void LED_set1(uint16_t state);
void LED_set2(uint16_t state);
void LED_set3(uint16_t state);
void LED_set4(uint16_t state);
#endif
