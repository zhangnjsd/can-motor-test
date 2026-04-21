#include "NUC_receive.h"
#include "ins_task.h"
#include "crc.h"
// #include "usbd_cdc_if.h"  // Missing file - commenting out
#include <stdbool.h>
#include "solvetrajectory.h"
#include "kalman_filter_task.h"
#include "CAN_receive.h"
//#include "Check_Online_task_fun.h"

//#define ERROR_PITCH 0.2
//#define ERROR_YAW 0.2

Send_Nuc_Data_u nuc_tx_u;
Receive_Nuc_Data_u nuc_rx_u;
uint16_t misstime=300;
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
uint8_t nuc_rec[sizeof(nuc_rx_u)];//缓存接收小电脑的数据
float tx_aim_x=0;
float tx_aim_y=0;
float tx_aim_z=0;
float st_yaw=0,st_yaw_rad=0,st_pitch_rad=0,st_pitch=0;
float temp_nuc_yaw=0,temp_mid_con=0,nuc_yaw_result=0;
uint8_t target_lost_flag;
uint8_t header_count;
uint8_t position[10];             //存储帧头位置
uint8_t first_data;               //第一帧数据的位置

/*向小电脑发送数据*/
void send_to_nuc(void)
{
	
  nuc_tx_u.Send_pack_t.header = 0x5A;
	//0识别红色，1识别蓝色
		if(get_board_t.robot_id<= 9)
	{
		nuc_tx_u.Send_pack_t.robot_color = 1;
	}
	else
	{
		nuc_tx_u.Send_pack_t.robot_color = 0;
	}
	
	nuc_tx_u.Send_pack_t.rest_tracker =0;
//	nuc_tx_u.Send_pack_t.id_filter_1=0;
//	nuc_tx_u.Send_pack_t.id_filter_2=0;
//	nuc_tx_u.Send_pack_t.id_filter_3=0;
//	nuc_tx_u.Send_pack_t.id_filter_4=0;
//	nuc_tx_u.Send_pack_t.id_filter_5=0;
	nuc_tx_u.Send_pack_t.aim_x = tx_aim_x;
	nuc_tx_u.Send_pack_t.aim_y = tx_aim_y;
	nuc_tx_u.Send_pack_t.aim_z = tx_aim_z;
	
	nuc_tx_u.Send_pack_t.sendroll = INS.Roll*0.01745329252f;
	nuc_tx_u.Send_pack_t.sendpitch = INS.Pitch*0.01745329252f;
	nuc_tx_u.Send_pack_t.sendyaw = INS.Yaw*0.01745329252f;
	
	Append_CRC16_Check_Sum(nuc_tx_u.nuc_pack_arry,sizeof(nuc_tx_u.nuc_pack_arry));
	CDC_Transmit_FS(nuc_tx_u.nuc_pack_arry,sizeof(nuc_tx_u.nuc_pack_arry));
}
/**
 * @brief 进行USB接受到的数据的转运,在usbd_cdc.c 中被使用
 * 
 * @param Buf  接受数据的BUF长度，从usbd——cdc_if给出
 */

void CDC_Receive_FS_fun(uint8_t* Buf)
{
		header_count=0;
		memset(position,0,10);
  	for(int i=0;i<sizeof(Buf);i++)
	  {
		    if(Buf[i]==0xa5)
		   {
			   header_count++;
			   position[header_count-1]=i;
		   }
	  }
		for(int i=0;i<header_count;i++)
		{
			first_data=position[i];
			memmove(nuc_rec,&Buf[first_data],sizeof(nuc_rec));
			if(Verify_CRC16_Check_Sum(nuc_rec,sizeof(nuc_rec))==1)
			{
				memmove(&nuc_rx_u,&nuc_rec,sizeof(nuc_rx_u));
				solve_t.tracking=nuc_rx_u.receive_pack_t.tracking;
				solve_t.receive_flag=1;

			}
		}
		memset(nuc_rec,0,sizeof(nuc_rx_u));
}
//void CDC_Receive_FS_fun(uint8_t* Buf)
//{

//	if(Verify_CRC16_Check_Sum(Buf,sizeof(nuc_rx_u)))
//		{
//		   memmove(nuc_rec,Buf,sizeof(nuc_rec));
//			
//		   for(int nuc_rx_len = 0; nuc_rx_len < sizeof(nuc_rx_u) - 1;nuc_rx_len ++)
//		  {
//				if(Buf[nuc_rx_len]==0xA5 )
//				{   
//					memmove(&nuc_rx_u,&nuc_rec[nuc_rx_len],sizeof(nuc_rx_u));
//					solve_t.tracking=nuc_rx_u.receive_pack_t.tracking;
//					memset(nuc_rec,0,sizeof(nuc_rx_u));
//					solve_t.receive_flag=1;
//				}
//			}
//	  }	
//}
void NUC_Recive_data()
{
	if(solve_t.receive_flag==1&&nuc_rx_u.receive_pack_t.xw!=0&&nuc_rx_u.receive_pack_t.yw!=0&&nuc_rx_u.receive_pack_t.tar_yaw!=0)
	{
		solve_t.armor_id=(enum ARMOR_ID)nuc_rx_u.receive_pack_t.id;
		solve_t.armor_num=(enum ARMOR_NUM)nuc_rx_u.receive_pack_t.armors_num;
		solve_t.xw_nfilter=nuc_rx_u.receive_pack_t.xw;
		solve_t.yw_nfilter=nuc_rx_u.receive_pack_t.yw;
		solve_t.tar_yaw_nfilter=nuc_rx_u.receive_pack_t.tar_yaw;
		solve_t.vxw_nfilter=nuc_rx_u.receive_pack_t.vxw;
		solve_t.vyw_nfilter=nuc_rx_u.receive_pack_t.vyw;
		solve_t.v_yaw_nfilter=nuc_rx_u.receive_pack_t.v_yaw;
		solve_t.zw=nuc_rx_u.receive_pack_t.zw;
		solve_t.vzw=nuc_rx_u.receive_pack_t.vzw;
		solve_t.r1=nuc_rx_u.receive_pack_t.r1;
		solve_t.r2=nuc_rx_u.receive_pack_t.r2;
		solve_t.dz=nuc_rx_u.receive_pack_t.dz;
//		solve_t.tracking=nuc_rx_u.receive_pack_t.tracking;
		Kalman_measureValue_update();
		solve_t.receive_flag=0;	
	}
}
void kalman_update_and_getvalue(void)
{
	kalman_filter_updata(&kalman_x_t);
	kalman_filter_updata(&kalman_y_t);
	kalman_filter_updata(&kalman_yaw_t);
	
	solve_t.xw=kalman_x_t.x_had.pData[0];
	solve_t.vxw=kalman_x_t.x_had.pData[1];
	solve_t.yw=kalman_y_t.x_had.pData[0];
  solve_t.vyw=kalman_y_t.x_had.pData[1];
  solve_t.tar_yaw=kalman_yaw_t.x_had.pData[0]; 
  solve_t.v_yaw=kalman_yaw_t.x_had.pData[1];
	
	if(solve_t.v_yaw<-0.06)
	{
		solve_t.direction_flag=0;
	}
	else solve_t.direction_flag=1;
	
	autoSolveTrajectory(&st_pitch_rad,&st_yaw_rad,&tx_aim_x,&tx_aim_y,&tx_aim_z);	
	
  st_yaw=st_yaw_rad*57.2957795f;
	st_pitch=st_pitch_rad*57.2957795f;
}

/*获取自瞄解算的yaw*/
float nuc_get_yaw()
{
		temp_nuc_yaw=st_yaw;//[17];
	if(temp_nuc_yaw<INS.Yaw)
	{
		temp_mid_con=360*QEKF_INS.YawRoundCount+temp_nuc_yaw+180;
	}
	else
	{
		temp_mid_con=360*QEKF_INS.YawRoundCount+temp_nuc_yaw-180;
	}
	//比较大小，判断究竟哪个点更近
	
	if(INS.YawTotalAngle < temp_mid_con)
	{
		nuc_yaw_result=temp_mid_con-180;
	}
	else
	{
		nuc_yaw_result=temp_mid_con+180;
	}
	return nuc_yaw_result;
}

/*获取自瞄解算的pitch*/
float nuc_get_pitch(void)
{
	return st_pitch_rad;
}

/*
*判断目标是否丢失
*暂时以是否接受到数据为标志位
*return 目标丢失标志位
*/
void target_lost(void)
{
	if(solve_t.tracking == true )
	{
		misstime=0;
//		solve_t.target_lost_flag=1;
	}
	else 
	{
		misstime++;
		if(misstime>10000)
		{
			misstime=10000;
		}
//		solve_t.target_lost_flag=0;
	}
	if(misstime>20)
	{
		target_lost_flag=0;
	}
	else target_lost_flag=1;
	
}
