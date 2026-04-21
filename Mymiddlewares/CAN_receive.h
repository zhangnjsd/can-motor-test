#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"
#include "cmsis_os.h"
#include "main.h"
// #include "dm_driver.h"  // Missing file - commenting out
// #include "Check_Online_task_fun.h"  // Missing file - commenting out

#define CAN_2         hcan2
#define CAN_1         hcan1

typedef enum 
{
	
	CAN_3508_SHOOT1_ID = 0x201,
	
	CAN_3508_SHOOT2_ID = 0x202,
	
	CAN_TRIGER_MOTOR_ID = 0x203,
	
	CAN_YAW_MOTOR_ID = 0x206,
	
	CAN_DAMIAO_ID = 0x301,      // 1to4
	
	CAN_DAMIAO_ID_MIT = 0x002,  // MIT
	
	CAN_BOARD1_ID = 0X10,      //下板给上板
	
}can_rec_id_e;

typedef enum 
{
	CAN_SHOOT_ALL_ID = 0x200,   //发射机构
	
  CAN_YAW_ALL_ID   = 0x1FF,  
	
	CAN_BOARD2_ID    = 0x09,   //上板给下板
	
	CAN_PITCH_ID     = 0x3FE,
	
	CAN_BOARD2_ID_1    =0x12,
//	C_data_ID = 0x211,
	
}can_tra_id_e;

//电机数据结构体
typedef struct
{
	uint16_t ecd;
	int16_t speed_rpm;
	int16_t given_current;
	uint8_t temperate;
	int16_t last_ecd;
} motor_measure_t;

//达妙电机数据结构体
typedef struct
{
	uint8_t err;
	uint8_t id;
	int16_t pos;
	int16_t vel;
	int16_t tor;
  float mos;
	float rot;
	
} damiao_motor_measure_t;

//超电结构体
typedef struct
{
    uint8_t remain_power;
    uint8_t current_power_IO;
} superCap_measure_t;

//双板通信结构体

/*上板发送结构体*/
typedef struct
{
	int16_t chassis_x;
	int16_t chassis_y;
	int8_t  chassis_w;
	int16_t yaw_ecd;
	int8_t chassis_mode;
//	int16_t chassis_mode;
}Send_Board_Data;
typedef struct
{
	int16_t Key;
}Send_Key_Data;
/*上板接收结构体*/
typedef struct
{
	int16_t robot_id;
	int16_t robot_level;
	int16_t robot_shootpower_flag;
	int16_t roll_flag;
}Get_Board_data;

extern void CAN_cmd_shoot(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN_cmd_yaw(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4); 
extern const motor_measure_t *get_shoot1_motor_measure_point(void);
extern const motor_measure_t *get_shoot2_motor_measure_point(void);
extern const motor_measure_t *get_damiao_motor_measure_point(void);
extern const motor_measure_t *get_yaw_motor_measure_point(void);
extern const motor_measure_t *get_triger_motor_measure_point(void);
extern void CAN1_send_to_damiao(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN1_send_to_board(Send_Board_Data* data);
extern void Key_send_to_board(Send_Key_Data* data);
extern dm_motor_t                  damiao_motor_mit;
extern Get_Board_data              get_board_t;
extern Send_Key_Data               send_key_data_t;
#endif
