#include "safe.h"

uint8_t check_safe()
{
	if(ahrs_state.mpu6050)
		return AHRS_EVENT_MPU6050;
	if(ahrs_state.hmc5883)
		return AHRS_EVENT_HMC5883;
	if(ahrs_state.sonar)
		return AHRS_EVENT_SONAR;
	if(ahrs_state.adns3080)
		return AHRS_EVENT_ADNS3080;
	return 0;
}
