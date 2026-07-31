#ifndef __PID_H__
#define __PID_H__

#include "main.h"

typedef struct
{
  float kp, ki, kd;
  int last_err;       // 上次误差
  int err_sum;        // 积分
  float last_current; //上次实际值 用于微分先行
  float last_speed;
  int int_limit;    // 积分限幅阈值 最大PWM输出除以ki再乘以0.95
} pid_t;

extern pid_t pid_fl, pid_fr, pid_bl, pid_br;
extern int target_speed_fl, target_speed_fr, target_speed_bl, target_speed_br;
extern int current_speed_fl, current_speed_fr, current_speed_bl, current_speed_br;

int speed_pid(int target, int current, pid_t* pid);   //速度环PID

void PID_Init(void);
void PID_Task(void);

#endif
