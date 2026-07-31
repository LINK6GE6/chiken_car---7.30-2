#include "pid.h"
#include "pwm.h"
#include "encoder.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#define PWM_DUTY_MAX 1680
#define PWM_DUTY_MIN -1680
// #define TARGET_SPEED_MAX 415    // 目标速度值范围 与50ms读取编码器差值对应
// #define TARGET_SPEED_MIN -412
#define INT_LIMIT 1

pid_t pid_fl, pid_fr, pid_bl, pid_br;
int target_speed_fl = 0, target_speed_fr = 0, target_speed_bl = 0, target_speed_br = 0;
int current_speed_fl = 0, current_speed_fr = 0, current_speed_bl = 0, current_speed_br = 0;
int current_speed_fl_10 = 0, current_speed_fr_10 = 0, current_speed_bl_10 = 0, current_speed_br_10 = 0;//调试用
extern osMutexId_t speed_mutexHandle;

/**
 * @brief 初始化四个电机的PID参数
 */
void PID_Init(void)
{
    // 左前电机
//    pid_fl.kp = 1.18f;
//    pid_fl.ki = 0.45f;
//    pid_fl.kd = 1.15f;
//    pid_fl.int_limit = 3547;

		pid_fl.kp = 5.18f;
		pid_fl.ki = 0.45f;
		pid_fl.kd = 0.45f;
		pid_fl.int_limit = 3547;

		pid_fr.kp = 5.18f;
    pid_fr.ki = 0.45f;
    pid_fr.kd = 0.45f;
    pid_fr.int_limit = 3547;

		pid_bl.kp = 5.18f;
    pid_bl.ki = 0.45f;
    pid_bl.kd = 0.45f;
    pid_bl.int_limit = 3547;

		pid_br.kp = 5.18f;
    pid_br.ki = 0.45f;
    pid_br.kd = 0.45f;
    pid_br.int_limit = 3547;

//    // 右前电机
//    pid_fr.kp = 3.5f;
//    pid_fr.ki = 1.0f;
//    pid_fr.kd = 0.15f;
//    pid_fr.int_limit = 1596;

//    // 左后电机
//    pid_bl.kp = 3.5f;
//    pid_bl.ki = 1.0f;
//    pid_bl.kd = 0.15f;
//    pid_bl.int_limit = 1596;

//    // 右后电机
//    pid_br.kp = 3.5f;
//    pid_br.ki = 1.0f;
//    pid_br.kd = 0.15f;
//    pid_br.int_limit = 1596;
}

int speed_pid(int target, int current, pid_t *pid)
{
  int err = 0;                  // 误差
  int diff = 0;                 // 微分

  float result = 0;

  // 更新变量
  err = target - current;
  pid->err_sum += err;
  diff = err - pid->last_err;

  pid->last_err = err;

  // if(target > TARGET_SPEED_MAX)    // 限制目标值 保证目标值与当前值同个物理单位
  // {
  //   target = TARGET_SPEED_MAX;
  // }
  // else if(target < TARGET_SPEED_MIN)
  // {
  //   target = TARGET_SPEED_MIN;
  // }

#if INT_LIMIT
  // 积分限幅
  if(pid->err_sum > pid->int_limit)
  {
    pid->err_sum = pid->int_limit;
  }
  else if(pid->err_sum < -pid->int_limit)
  {
    pid->err_sum = -pid->int_limit;
  }
#endif

  // 计算PID结果
  result = pid->kp * err + pid->ki * pid->err_sum + pid->kd * diff;
  // 输出限幅
  if(result > PWM_DUTY_MAX)
  {
    result = PWM_DUTY_MAX;
  }
  else if(result < PWM_DUTY_MIN)
  {
    result = PWM_DUTY_MIN;
  }

  return (int)result;
}

void PID_Task(void)
{
  if(osMutexAcquire(speed_mutexHandle, 100) == osOK)
  {
    // 1. 读取当前速度
    current_speed_fl = encoder_read_speed(1);
    current_speed_fr = encoder_read_speed(2);
    current_speed_bl = encoder_read_speed(3);
    current_speed_br = encoder_read_speed(4);

    // 2. 打印调试信息
//    printf("%d,%d\r\n", current_speed_fl, target_speed_fl);
//    printf("%d,%d\r\n", current_speed_fr, target_speed_fr);
//    printf("%d,%d\r\n", current_speed_bl,target_speed_bl);
//    printf("%d,%d\r\n", current_speed_br,target_speed_br);

    // 3. 计算 PID 并输出给电机
    int result_fl = speed_pid(target_speed_fl, current_speed_fl, &pid_fl);
    motor_set_speed(1, result_fl);

    int result_fr = speed_pid(target_speed_fr, current_speed_fr, &pid_fr);
    motor_set_speed(2, result_fr);

    int result_bl = speed_pid(target_speed_bl, current_speed_bl, &pid_bl);
    motor_set_speed(3, result_bl);

    int result_br = speed_pid(target_speed_br, current_speed_br, &pid_br);
    motor_set_speed(4, result_br);

    osMutexRelease(speed_mutexHandle);
  }
}
