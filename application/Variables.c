#include "Variables.h"
#include "CAN_receive.h"
#include "leg_task.h"

/********************************** global variable *************************************/
leg_control_t motor_control;
/********************************** DATA variable ****************************************/
s_Dji_motor_data_t motor_Date[4];      // RM电机回传数据结构体
