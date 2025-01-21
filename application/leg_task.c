/**
  * @file       leg_task.c/.h
  * @brief      腿部运动实现函数，包括pid初始化
  * @note       
  * 
  */
#include "main.h"
#include "leg_task.h"
#include "pid.h"
#include "CAN_receive.h"
#include "Variables.h"

//双环pid
float pos_pid_kp = 8.0f;
float pos_pid_ki = 0.2f;
float pos_pid_kd = 0.0f;
float pos_pid_max_out = 20000.0f;
float pos_pid_max_iout = 10000.0f;

float spd_pid_kp = 0.13f;
float spd_pid_ki = 0.0f;
float spd_pid_kd = 2.5;
float spd_pid_max_out = 10000.0f;
float spd_pid_max_iout = 10000.0f;

//编码器角度
float current_pos = 0;
float target_pos = 0;
//编码器脉冲
float target_position = 0;
float current_position = 0;
float last_position = 0;

s_pid_absolute_t pos_pid;
s_pid_absolute_t spd_pid;

// 将角度转换为编码器脉冲数
int set_target_angle(float angle_degrees){
    target_position = (angle_degrees / 360.0f) * ENCODER_RESOLUTION * GEAR_RATIO;
    return target_position;
}

// 电机数据的初始化
void motor_init(leg_control_t *init)
{
    // 电机数据指针获取
    init->leg_motor.M3508[0].leg_motor_measure = get_3508_M1_motor_measure_point();
    init->leg_motor.M3508[1].leg_motor_measure = get_3508_M2_motor_measure_point();
    init->leg_motor.M3508[2].leg_motor_measure = get_3508_M3_motor_measure_point();
    // 电机pid初始化
    for (uint8_t i = 0; i < 4; i++)
    {
        pid_abs_param_init(&pos_pid, pos_pid_kp, pos_pid_ki, pos_pid_kd, pos_pid_max_out, pos_pid_max_iout);
        pid_abs_param_init(&spd_pid, spd_pid_kp, spd_pid_ki, spd_pid_kd, spd_pid_max_out, spd_pid_max_iout);
    }   
}

// 四个电机控制函数
void motor_control_send(leg_control_t *control_loop)
{
    int target_pos = set_target_angle((float)receive);
     for (uint8_t i = 0; i < 4; i++)
    {
        control_loop->leg_motor.M3508[i].motor_speed = control_loop->leg_motor.M3508[i].leg_motor_measure->back_motor_speed;
        control_loop->leg_motor.M3508[i].serial_position = control_loop->leg_motor.M3508[i].leg_motor_measure->serial_position;
        motor_double_loop_PID(&pos_pid, &spd_pid, control_loop->leg_motor.M3508[i].serial_position, target_pos, control_loop->leg_motor.M3508[i].motor_speed);
    }
    CAN_cmd_chassis(control_loop->leg_motor.M3508[0].M3508_pid_speed.PIDout, control_loop->leg_motor.M3508[1].M3508_pid_speed.PIDout, control_loop->leg_motor.M3508[2].M3508_pid_speed.PIDout, control_loop->leg_motor.M3508[3].M3508_pid_speed.PIDout);
}
