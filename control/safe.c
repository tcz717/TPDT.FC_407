#include "safe.h"
#include "PWM.h"
#include "remote.h"

rt_err_t check_safe(rt_uint32_t checklist)
{
	if(ahrs_state.mpu6050 && (checklist & SAFE_MPU6050))
		return SAFE_MPU6050;
	if(ahrs_state.hmc5883 && (checklist & SAFE_HMC5883))
		return SAFE_HMC5883;
	if(ahrs_state.sonar && (checklist & SAFE_SONAR))
		return SAFE_SONAR;
	if(ahrs_state.adns3080 && (checklist & SAFE_ADNS3080))
		return SAFE_ADNS3080;
	
	if(checklist & SAFE_PWM)
	{
		if(!(PWM1_Time>500&&PWM2_Time>500&&PWM3_Time>500&&PWM4_Time>500&&PWM5_Time>500))
			return SAFE_PWM;
	}
	
	if(tfrc_con && checklist & SAFE_TFCR)
		return SAFE_TFCR;
	return 0;
}
