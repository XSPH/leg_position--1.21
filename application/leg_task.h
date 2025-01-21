/**
  * @file       leg_task.c/.h
  * @brief      腿部运动实现函数，包括pid初始化
  * @note       
  * 
  */
#ifndef LEG_TASK_H
#define LEG_TASK_H
#include "struct_typedef.h"
#include "pid.h"
#include "Variables.h"
#include "CAN_receive.h"

// 减速比
#define GEAR_RATIO 3591.0f / 187.0f
// 编码器分辨率
#define ENCODER_RESOLUTION 8192
extern long receive;
typedef struct
{
    const s_Dji_motor_data_t *leg_motor_measure; // 电机数据结构体
    s_pid_absolute_t M3508_pid_speed;
    s_pid_absolute_t M3508_pid_angle;  
    uint16_t offset_ecd;

    int64_t serial_position;//电机连续编码值（刻度）
    int64_t serial_position_set; // 电机连续编码值设定值

    float motor_speed;
    float motor_speed_set;
    float raw_cmd_current;
    float current_set;
    float accel;

    int16_t given_current;

} motor_3508_t;

typedef struct
{
    motor_3508_t M3508[4];
} leg_motor_t;

typedef struct
{
    leg_motor_t leg_motor;
} leg_control_t;

// 将角度转换为编码器脉冲数
int set_target_angle(float angle_degrees);
// 电机数据的初始化
void motor_init(leg_control_t *init);
// 四个电机控制函数
void motor_control_send(leg_control_t *control_loop);

#endif
