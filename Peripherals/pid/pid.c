/*
 * pid.c
 *
 *  Created on: Apr 9, 2024
 *      Author: zeng
 */

#include "pid.h"

//定义一个结构体类型变量
tPid pidMotor1Speed;
tPid pidMotor2Speed;
tPid pidOpenMV_Tracking;
tPid pidMPU6050YawMovement; //利用6050偏航角 进行姿态控制的PID
//给结构体类型变量赋初值
void PID_init()
{
	pidMotor1Speed.actual_val=0.0;
	pidMotor1Speed.target_val=1.00;
	pidMotor1Speed.err=0.0;
	pidMotor1Speed.err_last=0.0;
	pidMotor1Speed.err_sum=0.0;
	pidMotor1Speed.Kp=10;
	pidMotor1Speed.Ki=2;
	pidMotor1Speed.Kd=0.1;

	pidMotor2Speed.actual_val=0.0;
	pidMotor2Speed.target_val=1.00;
	pidMotor2Speed.err=0.0;
	pidMotor2Speed.err_last=0.0;
	pidMotor2Speed.err_sum=0.0;
	pidMotor2Speed.Kp=10;
	pidMotor2Speed.Ki=2;
	pidMotor2Speed.Kd=0.1;

	pidOpenMV_Tracking.actual_val=0.0;
	pidOpenMV_Tracking.target_val=0.00;
	pidOpenMV_Tracking.err=0.0;
	pidOpenMV_Tracking.err_last=0.0;
	pidOpenMV_Tracking.err_sum=0.0;
	pidOpenMV_Tracking.Kp=-0.35;
	pidOpenMV_Tracking.Ki=0;
	pidOpenMV_Tracking.Kd=0;

	pidMPU6050YawMovement.actual_val=0.0;
	pidMPU6050YawMovement.target_val=0.00;//设定姿态目标值
	pidMPU6050YawMovement.err=0.0;
	pidMPU6050YawMovement.err_last=0.0;
	pidMPU6050YawMovement.err_sum=0.0;
	pidMPU6050YawMovement.Kp=0.02;//定距离跟随的Kp大小通过估算PID输入输出数据，确定大概小，然后在调试
	pidMPU6050YawMovement.Ki=0;
	pidMPU6050YawMovement.Kd=0;

}
//比例p调节控制函数
float P_realize(tPid * pid,float actual_val)
{
	pid->actual_val = actual_val;//传递真实值
	pid->err = pid->target_val - pid->actual_val;//当前误差=目标值-真实值
	//比例控制调节   输出=Kp*当前误差
	pid->actual_val = pid->Kp*pid->err;
	return pid->actual_val;
}
//比例P 积分I 控制函数
float PI_realize(tPid * pid,float actual_val)
{
	pid->actual_val = actual_val;//传递真实值
	pid->err = pid->target_val - pid->actual_val;//当前误差=目标值-真实值
	pid->err_sum += pid->err;//误差累计值 = 当前误差累计和
	//使用PI控制 输出=Kp*当前误差+Ki*误差累计值
	pid->actual_val = pid->Kp*pid->err + pid->Ki*pid->err_sum;

	return pid->actual_val;
}
// PID控制函数
float PID_realize(tPid * pid,float actual_val)
{
	pid->actual_val = actual_val;//传递真实值
	pid->err = pid->target_val - pid->actual_val;////当前误差=目标值-真实值
	pid->err_sum += pid->err;//误差累计值 = 当前误差累计和
	//使用PID控制 输出 = Kp*当前误差  +  Ki*误差累计值 + Kd*(当前误差-上次误差)
	pid->actual_val = pid->Kp*pid->err + pid->Ki*pid->err_sum + pid->Kd*(pid->err - pid->err_last);
	//保存上次误差: 这次误差赋值给上次误差
	pid->err_last = pid->err;

	return pid->actual_val;
}
