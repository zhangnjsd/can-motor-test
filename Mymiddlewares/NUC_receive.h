#ifndef __NUC_RECEIVE_H__
#define __NUC_RECEIVE_H__
#include "struct_typedef.h"
#include <stdbool.h>

/*发送数据包的联合体*/
typedef union 
{
			struct
	{
		uint8_t header;
		
		uint8_t robot_color:1;
		uint8_t rest_tracker:1;
//		uint8_t id_filter_1:1;
//		uint8_t id_filter_2:1;
//		uint8_t id_filter_3:1;
//		uint8_t id_filter_4:1;
//		uint8_t id_filter_5:1;
//		uint8_t reserved:1;
		uint8_t reserved:6;
		
		float sendroll;
//		float current_v;
		float sendpitch;
		float sendyaw;
		
		float aim_x;
		float aim_y;
		float aim_z;
//		uint16_t game_time;  // (s) game time [0, 450]
//    uint32_t timestamp;  // (ms) board time
		uint16_t checksum;
		
	}__attribute__((packed))Send_pack_t;
	
   uint8_t nuc_pack_arry[28];

}Send_Nuc_Data_u;
//接收到的数据包
typedef union 
{
	struct
	{
		uint8_t header;
		bool tracking : 1;
		uint8_t id:3;
		uint8_t armors_num :3;
		uint8_t reserved:1;
		float xw;
		float yw;
		float zw;
		float tar_yaw;
		float vxw;
		float vyw;
		float vzw;
		float v_yaw;
		float r1;
		float r2;
		float dz;
		uint16_t checksum;
	} __attribute__((packed))receive_pack_t;
	uint8_t nuc_receive_pack_arry[48];
}Receive_Nuc_Data_u;
//typedef union 
//{
//	struct
//	{
//		uint8_t header;
//		uint8_t tracking : 2;// 0-untracking 1-tracking-aim 2-tracking-buff
//		uint8_t id:3;
//		uint8_t armors_num :3;
//		float xw;
//		float yw;
//		float zw;
//		float tar_yaw;
//		
//		float vxw;
//		float vyw;
//		float vzw;
//		float v_yaw;
//		float r1;
//		float r2;
//		float dz;
//		
//		uint32_t cap_timestamp;  // (ms) frame capture time
//    uint16_t t_offset;       // (ms) speed t offset
//		uint16_t checksum;
//	} __attribute__((packed))receive_pack_t;
//	uint8_t nuc_receive_pack_arry[54];
//}Receive_Nuc_Data_u;

/*开放函数*/
extern void send_to_nuc(void);
extern void CDC_Receive_FS_fun(uint8_t* Buf);
extern void NUC_Recive_data(void);
extern void kalman_update_and_getvalue(void);
extern float nuc_get_pitch(void);
extern float nuc_get_yaw(void);
extern void target_lost(void);
extern Send_Nuc_Data_u nuc_tx_u;
extern Receive_Nuc_Data_u nuc_rx_u;
extern uint8_t target_lost_flag;
#endif
