#include "stm32f4xx.h"

float Left_Kp = 0.15;
float Left_Ki = 0.02;
float Left_Kd;

float left_error=0.0;
float left_lasterror=0.0;
float left_error_sum=0.0;
float filt_left_error;
float filt_left_lasterror;

#define left_max 10000
#define left_min -10000


float Right_Kp = 0.15;
float Right_Ki = 0.02;
float Right_Kd;

float right_error=0.0;
float right_lasterror=0.0;
float right_error_sum=0.0;
float filt_right_error;
float filt_right_lasterror;

#define right_max 10000
#define right_min -10000

#define dead_zone 2


int16_t Left_PID_Calculate(int left_actual_speed,int left_target_speed)                     
{
	float a =  0.3;
	left_error = left_target_speed - left_actual_speed;   //误差
	filt_left_error= a * left_error + (1-a)  * filt_left_lasterror;  //滤波
/*
    if((filt_left_error > -dead_zone) && (filt_left_error < dead_zone))
    {
        filt_left_lasterror = filt_left_error;
        return 0;
    }
*/
	left_error_sum += filt_left_error;                      //误差累加
	//if(left_error_sum > left_max*10)   left_error_sum = left_max*10;
	//if(left_error_sum < left_min*10)   left_error_sum = left_min*10;
	
	//PID计算
	float PID_out=Left_Kp*filt_left_error + Left_Ki*left_error_sum;
	
	filt_left_lasterror=filt_left_error;                       //更新误差
	
	//输出限幅
	if(PID_out > left_max)   PID_out=left_max;
	if(PID_out < left_min)   PID_out=left_min;
	
	return (int16_t)PID_out;
}

int16_t Right_PID_Calculate(int right_actual_speed,int right_target_speed)                     
{
	float a =  0.3;
	right_error = right_target_speed - right_actual_speed;   //误差
	filt_right_error= a * right_error + (1-a)  * filt_right_lasterror;  //滤波
/*
    if((filt_right_error > -dead_zone) && (filt_right_error < dead_zone))
    {
        filt_right_lasterror = filt_right_error;
        return 0;
    }
*/
	right_error_sum += filt_right_error;                      //误差累加
	//if(right_error_sum > right_max*10)   right_error_sum = right_max*10;
	//if(right_error_sum < right_min*10)   right_error_sum = right_min*10;
	
	//PID计算
	float PID_out=Right_Kp*filt_right_error + Right_Ki*right_error_sum;
	
	filt_right_lasterror=filt_right_error;                       //更新误差
	
	//输出限幅
	if(PID_out > right_max)   PID_out=right_max;
	if(PID_out < right_min)   PID_out=right_min;
	
	return (int16_t)PID_out;
}