#ifndef _SONAR
#define _SONAR
#include <stm32f4xx.h>

void sonar_init(void);
extern u8 sonar_state;
extern float sonar_h;

#endif
