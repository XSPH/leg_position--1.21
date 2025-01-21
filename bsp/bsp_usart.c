/**
  * @file      bsp_usart.c
  * @version   0.0
  * @data      2024/12/28
  * @author    XSPH
  *
  * @usage     DMA receive interrupt callback
  * @hardware  STM32 F4 Series
  * @note      In this version no Interrupt was used
  */

 /****** Macros used in this file ******/
#include "main.h"
#include "usart.h"
#include "bsp_usart.h"
#include "struct_typedef.h"   
#include "stdlib.h"
#include "stdio.h"
#include "string.h" 
#include "pid.h"
/*----------------* define declaration --------------*/
#define MAX_RECEIVE_LENGTH 20
uint8_t receivedata[MAX_RECEIVE_LENGTH];  // 接收数据缓冲区
long receive = 0;
extern s_pid_absolute_t pos_pid;
extern s_pid_absolute_t spd_pid;
extern float pos_pid_kp;
extern float pos_pid_ki;
extern float pos_pid_kd;
extern float pos_pid_max_out;
extern float pos_pid_max_iout;
extern float spd_pid_kp;
extern float spd_pid_ki;
extern float spd_pid_kd;
extern float spd_pid_max_out;
extern float spd_pid_max_iout;
//DMA初始化
void dma_init(void){
     HAL_UARTEx_ReceiveToIdle_DMA(&huart6, receivedata, MAX_RECEIVE_LENGTH);
}

// 当DMA或中断完成接收数据后会被调用
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    if (huart == &huart6) {    
        if (strncmp((const char *)receivedata, "pos_pid", 3) == 0) {
            if (sscanf((const char *)receivedata, "pos_pid %f %f %f ", &pos_pid_kp, &pos_pid_ki, &pos_pid_kd) == 3) {
                pid_abs_evaluation(&pos_pid, pos_pid_kp, pos_pid_ki, pos_pid_kd, pos_pid_max_iout, pos_pid_max_out);
            } 
        }
        
        else if (strncmp((const char *)receivedata, "spd_pid", 3) == 0) {
                if (sscanf((const char *)receivedata, "spd_pid %f %f %f ", &spd_pid_kp, &spd_pid_ki, &spd_pid_kd) == 3) {
                pid_abs_evaluation(&spd_pid, spd_pid_kp, spd_pid_ki, spd_pid_kd, spd_pid_max_iout, spd_pid_max_out);
            } 
        }
        
        else {
            receive = strtol((const char *)receivedata, NULL, 10); // 使用 strtol 处理负数
            }
        //接收应答
        HAL_UART_Transmit_DMA(&huart6, receivedata, strlen((const char *)receivedata));
        HAL_UART_Transmit_DMA(&huart6, (uint8_t *)"\r\n", 2);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, receivedata, MAX_RECEIVE_LENGTH);
        
    }
}
