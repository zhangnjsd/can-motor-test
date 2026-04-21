#include "CAN_receive.h"


extern CAN_HandleTypeDef hcan2;
extern CAN_HandleTypeDef hcan1;

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
	
motor_measure_t             shoot_motor[2];          //摩擦轮
motor_measure_t             damiao_motor;            //达妙电机一拖四模式
motor_measure_t             triger_motor;            //拨弹电机
motor_measure_t		          yaw_motor;               //yaw轴电机
		
dm_motor_t                  damiao_motor_mit;        //达妙mit模式
superCap_measure_t          superCap;		             //超级电容
Get_Board_data              get_board_t;
Send_Key_Data               send_key_data_t;

static CAN_TxHeaderTypeDef  board_tx_message;
static CAN_TxHeaderTypeDef  key_tx_message;
static CAN_TxHeaderTypeDef  yaw_tx_message;
static CAN_TxHeaderTypeDef  shoot_tx_message;
static CAN_TxHeaderTypeDef  damiao_tx_message;

//static CAN_TxHeaderTypeDef  DM_PITCH_tx_message;
		
static uint8_t              shoot_can_send_data[8];
static uint8_t              yaw_can_send_data[8];
static uint8_t              board_can_send_data[8];
static uint8_t              damiao_can_send_data[8];
static uint8_t              key_can_send_data;
//static uint8_t              DM_PITCH_can_send_data[8];
		
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t rx_data[8];
	CAN_RxHeaderTypeDef rx_header;
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	if(hcan == &hcan1)
	{
		switch (rx_header.StdId)
		{
			
				case CAN_3508_SHOOT1_ID:
				{
					get_motor_measure(&shoot_motor[0],rx_data);
					Device_Status_Update(LINE_CHECK_3508_1);
					break;
				}
				case CAN_3508_SHOOT2_ID:
				{
					get_motor_measure(&shoot_motor[1],rx_data);
					Device_Status_Update(LINE_CHECK_3508_2);
					break;
				}
				case CAN_TRIGER_MOTOR_ID:
				{
					get_motor_measure(&triger_motor,rx_data);
					Device_Status_Update(LINE_CHECK_2006);
					break;
				}
				case CAN_YAW_MOTOR_ID:
				{
					get_motor_measure(&yaw_motor,rx_data);
					Device_Status_Update(LINE_CHECK_YAW);
					break;
				}
				case CAN_BOARD1_ID:
				{
					get_board_t.robot_id=(int8_t)(rx_data[0]);
					get_board_t.robot_level=(int8_t)(rx_data[1]);
					get_board_t.robot_shootpower_flag=(int8_t)(rx_data[2]);
					get_board_t.roll_flag=(int8_t)(rx_data[3]);
					Device_Status_Update(LINE_CHECK_BOARD);
					break;
				}
				default:
				{
					break;
				}
			}

	}
	if(hcan == &hcan2)
	{
			switch (rx_header.StdId)
			{
				case CAN_DAMIAO_ID_MIT:
				{
					dm4310_fbdata(&damiao_motor_mit,rx_data);
					Device_Status_Update(LINE_CHECK_PITCH);
					break;
				}
				default:
				{
					break;
				}
			}
	}
}


void CAN_cmd_shoot(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    shoot_tx_message.StdId = CAN_SHOOT_ALL_ID;
    shoot_tx_message.IDE = CAN_ID_STD;
    shoot_tx_message.RTR = CAN_RTR_DATA;
    shoot_tx_message.DLC = 0x08;
    shoot_can_send_data[0] = motor1 >> 8;
    shoot_can_send_data[1] = motor1;
    shoot_can_send_data[2] = motor2 >> 8;
    shoot_can_send_data[3] = motor2;
    shoot_can_send_data[4] = motor3 >> 8;
    shoot_can_send_data[5] = motor3;
    shoot_can_send_data[6] = motor4 >> 8;
    shoot_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CAN_1, &shoot_tx_message, shoot_can_send_data, &send_mail_box);
}

void CAN_cmd_yaw(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    yaw_tx_message.StdId = CAN_YAW_ALL_ID;
    yaw_tx_message.IDE = CAN_ID_STD;
    yaw_tx_message.RTR = CAN_RTR_DATA;
    yaw_tx_message.DLC = 0x08;
    yaw_can_send_data[0] = motor1 >> 8;
    yaw_can_send_data[1] = motor1;
    yaw_can_send_data[2] = motor2 >> 8;
    yaw_can_send_data[3] = motor2;
    yaw_can_send_data[4] = motor3 >> 8;
    yaw_can_send_data[5] = motor3;
    yaw_can_send_data[6] = motor4 >> 8;
    yaw_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CAN_1, &yaw_tx_message, yaw_can_send_data, &send_mail_box);
}

void CAN1_send_to_board(Send_Board_Data* data) 
{
    uint32_t send_mail_box;
    board_tx_message.StdId = CAN_BOARD2_ID; // 0x09 标识符
    board_tx_message.IDE = CAN_ID_STD;
    board_tx_message.RTR = CAN_RTR_DATA;
    board_tx_message.DLC = 0x08;

    board_can_send_data[0] = (data->chassis_x>>8);
    board_can_send_data[1] = data->chassis_x;
	  board_can_send_data[2] = (data->chassis_y>>8);
    board_can_send_data[3] = data->chassis_y;
	  board_can_send_data[4] = (data->yaw_ecd>>8);
    board_can_send_data[5] = data->yaw_ecd;
    board_can_send_data[6] = data->chassis_mode;
    board_can_send_data[7] = data->chassis_w;
	
    HAL_CAN_AddTxMessage(&CAN_1, &board_tx_message, board_can_send_data, &send_mail_box);
}

void Key_send_to_board(Send_Key_Data* data) 
{
    uint32_t send_mail_box;
    key_tx_message.StdId = CAN_BOARD2_ID_1; // 0x12 标识符
    key_tx_message.IDE = CAN_ID_STD;
    key_tx_message.RTR = CAN_RTR_DATA;
    key_tx_message.DLC = 0x01;
	  key_can_send_data = data->Key;

    HAL_CAN_AddTxMessage(&CAN_1, &key_tx_message, &key_can_send_data, &send_mail_box);
}

//测试达妙电机
//1to4
void CAN1_send_to_damiao(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) 
{
    uint32_t send_mail_box;
    damiao_tx_message.StdId = CAN_PITCH_ID; // 0x3FE 标识符
    damiao_tx_message.IDE = CAN_ID_STD;
    damiao_tx_message.RTR = CAN_RTR_DATA;
    damiao_tx_message.DLC = 0x08;

    damiao_can_send_data[0] = motor1;
    damiao_can_send_data[1] = motor1 >> 8;
	  damiao_can_send_data[2] = motor2;
    damiao_can_send_data[3] = motor2 >> 8;
	  damiao_can_send_data[4] = motor3;
    damiao_can_send_data[5] = motor3 >> 8;
	  damiao_can_send_data[6] = motor4;
    damiao_can_send_data[7] = motor4 >> 8;
	
    HAL_CAN_AddTxMessage(&CAN_2, &damiao_tx_message, damiao_can_send_data, &send_mail_box);
}

const motor_measure_t *get_shoot1_motor_measure_point(void)
{
    return &shoot_motor[0];
}

const motor_measure_t *get_shoot2_motor_measure_point(void)
{
    return &shoot_motor[1];
}

const motor_measure_t *get_damiao_motor_measure_point(void)
{
    return &damiao_motor;
}

const motor_measure_t *get_yaw_motor_measure_point(void)
{
    return &yaw_motor;
}

const motor_measure_t *get_triger_motor_measure_point(void)
{
    return &triger_motor;
}
