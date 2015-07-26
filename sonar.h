#ifndef _SONAR
#define _SONAR
#include <stm32f4xx.h>

void sonar_init(void);
extern float sonar_h;
extern u8 sonar_state;
extern struct rt_semaphore sonar_sem;

#endif
