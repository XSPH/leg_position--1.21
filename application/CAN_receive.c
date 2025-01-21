#include "CAN_receive.h"
#include "main.h"
#include "variables.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern s_Dji_motor_data_t motor_Date[4]; // RM电机回传数据结构体

static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];


// 发送电机控制电流  范围 [-16384,16384]
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header_can1, rx_header_can2;
    uint8_t rx_data_can1[8];
    uint8_t rx_data_can2[8];
    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header_can1, rx_data_can1);

        switch (rx_header_can1.StdId)
        {
        case CAN_3508_M1_ID:
        {
		    motor_Date[0].back_position    =		rx_data_can1[0]<<8 | rx_data_can1[1];
			motor_Date[0].back_motor_speed =		rx_data_can1[2]<<8 | rx_data_can1[3];
			motor_Date[0].back_current     = 	    rx_data_can1[4]<<8 | rx_data_can1[5];
            continue_motor_pos(&motor_Date[0]);
            break;
        }
        case CAN_3508_M2_ID:
        {
            motor_Date[1].back_position    =		rx_data_can1[0]<<8 | rx_data_can1[1];
			motor_Date[1].back_motor_speed =		rx_data_can1[2]<<8 | rx_data_can1[3];
			motor_Date[1].back_current     = 	    rx_data_can1[4]<<8 | rx_data_can1[5];
            continue_motor_pos(&motor_Date[1]);
            break;
        }
        case CAN_3508_M3_ID:
        {
		    motor_Date[2].back_position    =		rx_data_can1[0]<<8 | rx_data_can1[1];
			motor_Date[2].back_motor_speed =		rx_data_can1[2]<<8 | rx_data_can1[3];
			motor_Date[2].back_current     = 	    rx_data_can1[4]<<8 | rx_data_can1[5];
            continue_motor_pos(&motor_Date[2]);
            break;
        }
        case CAN_3508_M4_ID:
        {
            motor_Date[3].back_position    =		rx_data_can1[0]<<8 | rx_data_can1[1];
			motor_Date[3].back_motor_speed =		rx_data_can1[2]<<8 | rx_data_can1[3];
			motor_Date[3].back_current     = 	    rx_data_can1[4]<<8 | rx_data_can1[5];
            continue_motor_pos(&motor_Date[3]);            
            break;
        }
        default:
        {
            break;
        }
        }
    }
}
//dji电机连续编码器数据处理 
void continue_motor_pos(s_Dji_motor_data_t *s_motor)
{
    // 如果（当前电机返回值-上一次电机返回值）值大于4096，因为电机不可能在几毫秒内转过半圈
    if (s_motor->back_position - s_motor->back_pos_last > 4096)
    {
        s_motor->circle_num--; // 圈数--
    }
    else if (s_motor->back_position - s_motor->back_pos_last < -4096) // 同上，只不过方向是反的
    {
       s_motor->circle_num++; // 圈数++
    }
    s_motor->back_pos_last = s_motor->back_position;                                // 将上一次进入该函数的电机返回值赋值，方便计算连续值
    s_motor->serial_position = s_motor->back_position + s_motor->circle_num * 8191; // 返回的电机连续编码器值
    s_motor->back_motor_ang = s_motor->back_position / 8191.0f * 360.0f;            // 返回的电机绝对角度
    s_motor->serial_motor_ang = s_motor->serial_position / 8191.0f * 360.0f;        // 返回的电机连续角度
}

const s_Dji_motor_data_t *get_3508_M1_motor_measure_point(void)
{
    return &motor_Date[0];
}
const s_Dji_motor_data_t *get_3508_M2_motor_measure_point(void)
{
    return &motor_Date[1];
}
const s_Dji_motor_data_t *get_3508_M3_motor_measure_point(void)
{
    return &motor_Date[2];
}
const s_Dji_motor_data_t *get_3508_M4_motor_measure_point(void)
{
    return &motor_Date[3];
}
