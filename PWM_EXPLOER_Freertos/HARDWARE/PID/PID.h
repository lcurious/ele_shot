#ifndef __PID_H
#define __PID_H

#include "stdint.h"
#include "usart.h"
#include "machine.h"

enum{
	LLAST	= 0,
	LAST 	= 1,
	NOW 	= 2,
};

typedef struct 
{
	float p;
	float i;
	float d;
}__test_pid_t;

typedef struct __pid_t
{
	
	float p;
	float i;
	float d;
	
	float pout;							
	float iout;							
	float dout;	
	
	float set[3];				
	float get[3];				
	float err[3];					
					
	float delta_u;					
	float delta_out;/* last_delta_out + delta_u */
	float last_delta_out;
	
	float max_err;
	float deadband;						//err < deadband return
	uint32_t IntegralLimit;	
	uint32_t MaxOutput;				
	
	void (*f_param_init)(struct __pid_t *pid,
									uint32_t maxOutput,
								  uint32_t integralLimit,
									float p,
									float i,
									float d);
    void (*f_pid_reset)(struct __pid_t *pid, float p, float i, float d);		//pid??????
}Pid_s;

float pid_regulator(Pid_s* pid, float set_speed, float get_speed);

void abs_limit(float * x, float ABS_MAX);
void pid_param_init(
	Pid_s *pid, 	
	uint32_t intergral_limit,
    uint32_t max_output,
	float kp, 
	float	ki, 
	float	kd);

void pid_clear(Pid_s * pid);
void pid_parameter_init(void);	
	
extern Pid_s gimbal_pid[GIMBAL_NUM];

#endif
	
	