#include "pid.h"
#include "machine.h"

Pid_s gimbal_pid[GIMBAL_NUM];

void pid_parameter_init(void)
{						
	pid_param_init(&gimbal_pid[GIMBAL0], 10000, 10000, 
				   0.9f, 0.04f, 0.0f);
	pid_param_init(&gimbal_pid[GIMBAL1], 10000, 10000, 
				   1.0f, 0.0f, 0.0f);
}
void pid_clear(Pid_s * pid)
{
	if (pid == NULL)
	{
		return;
	}
	
	pid->pout = pid->iout = pid->dout = 0.0f;
	pid->delta_out = pid->delta_u = pid->last_delta_out = 0.0f;
	pid->err[0] = pid->err[1] = pid->err[2] = 0.0f;
	pid->set[NOW] = 0.0f;
	pid->get[NOW] = 0.0f;
}	

float pid_regulator(Pid_s* pid, float set_speed, float get_speed)
{
	pid->set[NOW] = set_speed;
	pid->get[NOW] = get_speed;	
	pid->err[NOW] = set_speed - get_speed;
	pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
	pid->iout = pid->i * pid->err[NOW];
	pid->dout = pid->d * (pid->err[NOW] - 2 * pid->err[LAST] + pid->err[LLAST]);
		
	abs_limit(&(pid->iout), pid->IntegralLimit);
	pid->delta_u = pid->pout + pid->iout + pid->dout;
	pid->delta_out = pid->last_delta_out + pid->delta_u;
	abs_limit(&(pid->delta_out), pid->MaxOutput);
	pid->last_delta_out = pid->delta_out;
	
	
	pid->err[LLAST] = pid->err[LAST];
	pid->err[LAST] = pid->err[NOW];
	pid->get[LLAST] = pid->get[LAST];
	pid->get[LAST] = pid->get[NOW];
	pid->set[LLAST] = pid->set[LAST];
	pid->set[LAST] = pid->set[NOW];
	
	return pid->delta_out;
}


void abs_limit(float * x, float ABS_MAX)
{
	if (*x > ABS_MAX)
	{
		*x = ABS_MAX;
	}
	if (*x < (ABS_MAX * (-1.0)))
	{
		*x = ABS_MAX * (-1.0);
	}
}


void pid_param_init(
	Pid_s	  *pid, 	
	uint32_t  intergral_limit,
	uint32_t  max_output,
	float 	kp, 
	float	ki, 
	float	kd)
{    
	pid->IntegralLimit = intergral_limit;
	pid->MaxOutput = max_output;
	
	pid->p = kp;
	pid->i = ki;
	pid->d = kd;    
}
