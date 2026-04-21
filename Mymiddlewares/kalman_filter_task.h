#ifndef __KALMAN_FILTER_TASK__
#define __KALMAN_FILTER_TASK__
#include "kalman_filter.h"

typedef struct
{
	float A_array[4];
	float A_T_array[4];
	float A_yaw_array[4];
	float A_yaw_T_array[4];
	float Q_array[4];
	float H_array[4];
	float I_array[4];
	float R_array[4];
	float Q_v_array[4];
	float R_v_array[4];
	
	float x_had_estimate_array[2];
	float x_had_array[2];
	float temp_x_had_array[2];
	
	float p_array[4];
	float temp_p_array[4];
	float p_estimate_array[4];
	
	float k_k_array[4];
	float z_array[2];
	
	float temp_1_array[4];
	float temp_2_array[4];
	float temp_3_2x1_array[2];
	float temp_4_2x1_array[2];
	
	mat A;
	mat A_T;
	mat Q;
	mat H;
	mat I;
	mat R;
//	mat Q_v;
//	mat R_v;
	
	mat temp_x_had;
	mat x_had_estimate;
	mat x_had;
	
  mat p_estimate;
	mat p;
	mat temp_p;
	
	mat k_k;
	mat z;
	
	mat temp_1;
	mat temp_2;
	mat temp_3_2x1;
	mat temp_4_2x1;
	
  uint8_t updata_flag;	
}kalman_t;

extern kalman_t kalman_x_t;
extern kalman_t kalman_y_t;
extern kalman_t kalman_yaw_t;

extern void kalman_init(kalman_t*kf);
extern void Kalman_measureValue_update(void);
extern void kalman_filter_updata(kalman_t*kf);
#endif
