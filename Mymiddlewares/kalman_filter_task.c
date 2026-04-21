#include "kalman_filter_task.h"
#include "solvetrajectory.h"
/*卡尔曼参数结构体初始化*/
kalman_t kalman_x_t;
kalman_t kalman_y_t;
kalman_t kalman_yaw_t;

void kalman_matrix_init(kalman_t *kf);

void kalman_init(kalman_t*kf)
{
	static float A_init[4]=
	{
		1,0,
		0,1
	};
	static float A_T_init[4]=
	{
		1,0,
		0,1
	};
	static float H_init[4]=
	{
		1,0,
		0,1
	};
	static float R_init[4]=
	{
		1,0,
		0,1
	};//测量噪声的协方差矩阵
	static float Q_init[4]=
	{
		0.1,0,
		0,0.1
	};//过程噪声的协方差矩阵
	static float I_init[4]=
	{
		1,0,
		0,1
	};//单位矩阵
	static float R_v_init[4]=
	{
		1,0,
		0,1
	};	
	static float Q_v_init[4]=
	{
		0.01,0,
		0,0.01
	};
	
	static float x_had_init[2]=
	{
		0,0
	};
	static float temp_x_had_init[2]=
	{
		0.1,0.1
	};//k-1时刻的xhad
	static float x_had_estimate_init[2]=
	{
		0,0
	};//先验估计值
	
	static float z_init[2]=
	{
		0,0
	};//测量值
	
	static float p_init[4]=
	{
		1,0,
		0,1
	};
	static float temp_p_init[4]=
	{
		0.1,0,
		0,0.1
	};//k-1时刻的误差协方差矩阵
	static float p_estimate_init[4]=
	{
		0,0,
		0,0
	};//先验误差协方差矩阵
	
	
	static float k_k_init[4]=
	{
		0,0,
		0,0
	};
	
	
	
	static float temp_1_init[4]=
	{
		0,0,
		0,0
	};
	static float temp_2_init[4]=
	{
		0,0,
		0,0
	};
	static float temp_3_2x1_init[2]=
	{
		0,0
	};
	static float temp_4_2x1_init[2]=
	{
		0,0
	};
	memcpy(kf->A_array,A_init,sizeof(kf->A_array));
	memcpy(kf->A_T_array,A_T_init,sizeof(kf->A_T_array));
	memcpy(kf->H_array,H_init,sizeof(kf->H_array));
	memcpy(kf->R_array,R_init,sizeof(kf->R_array));
	memcpy(kf->Q_array,Q_init,sizeof(kf->Q_array));
	
	memcpy(kf->Q_v_array,Q_v_init,sizeof(kf->Q_v_array));
	memcpy(kf->R_v_array,R_v_init,sizeof(kf->R_v_array));
	
	memcpy(kf->I_array,I_init,sizeof(kf->I_array));
	memcpy(kf->temp_x_had_array,temp_x_had_init,sizeof(kf->temp_x_had_array));
	memcpy(kf->x_had_estimate_array,x_had_estimate_init,sizeof(kf->x_had_estimate_array));
	memcpy(kf->x_had_array,x_had_init,sizeof(kf->x_had_array));
	memcpy(kf->p_array,p_init,sizeof(kf->p_array));
	memcpy(kf->p_estimate_array,p_estimate_init,sizeof(kf->p_estimate_array));
	memcpy(kf->temp_p_array,temp_p_init,sizeof(kf->temp_p_array));
	memcpy(kf->k_k_array,k_k_init,sizeof(kf->k_k_array));
	memcpy(kf->z_array,z_init,sizeof(kf->z_array));
	memcpy(kf->temp_1_array,temp_1_init,sizeof(kf->temp_1_array));
	memcpy(kf->temp_2_array,temp_2_init,sizeof(kf->temp_2_array));
  memcpy(kf->temp_3_2x1_array,temp_3_2x1_init,sizeof(kf->temp_3_2x1_array));
	memcpy(kf->temp_4_2x1_array,temp_4_2x1_init,sizeof(kf->temp_4_2x1_array));
//	memcpy(kalman_y_t.A_array,A_init,sizeof(kalman_y_t.A_array));
//	memcpy(kalman_y_t.A_T_array,A_T_init,sizeof(kalman_y_t.A_T_array));
//	memcpy(kalman_y_t.H_array,H_init,sizeof(kalman_y_t.H_array));
//	memcpy(kalman_y_t.R_array,R_init,sizeof(kalman_y_t.R_array));
//	memcpy(kalman_y_t.Q_array,Q_init,sizeof(kalman_y_t.Q_array));
//	memcpy(kalman_y_t.I_array,I_init,sizeof(kalman_y_t.I_array));
//  memcpy(kalman_y_t.temp_x_had_array,temp_x_had_init,sizeof(kalman_x_t.temp_x_had_array));
//	memcpy(kalman_y_t.x_had_estimate_array,x_had_estimate_init,sizeof(kalman_x_t.x_had_estimate_array));
//	memcpy(kalman_y_t.x_had_array,x_had_init,sizeof(kalman_x_t.x_had_array));
//	memcpy(kalman_y_t.p_array,p_init,sizeof(kalman_x_t.p_array));
//	memcpy(kalman_y_t.p_estimate_array,p_estimate_init,sizeof(kalman_x_t.p_estimate_array));
//	memcpy(kalman_y_t.temp_p_array,temp_p_init,sizeof(kalman_x_t.temp_p_array));
//	memcpy(kalman_y_t.temp_1_array,temp_1_init,sizeof(kalman_x_t.temp_1_array));
//	memcpy(kalman_y_t.temp_2_array,temp_2_init,sizeof(kalman_x_t.temp_2_array));


//	memcpy(kalman_yaw_t.A_array,A_init,sizeof(kalman_yaw_t.A_array));
//	memcpy(kalman_yaw_t.A_T_array,A_T_init,sizeof(kalman_yaw_t.A_T_array));
//	memcpy(kalman_yaw_t.H_array,H_init,sizeof(kalman_yaw_t.H_array));
//	memcpy(kalman_yaw_t.R_array,R_init,sizeof(kalman_yaw_t.R_array));
//	memcpy(kalman_yaw_t.Q_array,Q_init,sizeof(kalman_yaw_t.Q_array));
//	memcpy(kalman_yaw_t.I_array,I_init,sizeof(kalman_yaw_t.I_array));
//	memcpy(kalman_yaw_t.temp_x_had_array,temp_x_had_init,sizeof(kalman_x_t.temp_x_had_array));
//	memcpy(kalman_yaw_t.x_had_estimate_array,x_had_estimate_init,sizeof(kalman_x_t.x_had_estimate_array));
//	memcpy(kalman_yaw_t.x_had_array,x_had_init,sizeof(kalman_x_t.x_had_array));
//	memcpy(kalman_yaw_t.p_array,p_init,sizeof(kalman_x_t.p_array));
//	memcpy(kalman_yaw_t.p_estimate_array,p_estimate_init,sizeof(kalman_x_t.p_estimate_array));
//	memcpy(kalman_yaw_t.temp_p_array,temp_p_init,sizeof(kalman_x_t.temp_p_array));
//	memcpy(kalman_yaw_t.temp_1_array,temp_1_init,sizeof(kalman_x_t.temp_1_array));
//	memcpy(kalman_yaw_t.temp_2_array,temp_2_init,sizeof(kalman_x_t.temp_2_array));
	
	
//	kalman_matrix_init(kf);
//	kalman_matrix_init(kf);
//	kalman_matrix_init(kf);
}

void pos_kalman_matrix_init(kalman_t *kf)
{
	//矩阵初始化
	Matrix_Init(&kf->A,2,2,kf->A_array);
	Matrix_Init(&kf->A_T,2,2,kf->A_T_array);
	Matrix_Init(&kf->H,2,2,kf->H_array);
	Matrix_Init(&kf->R,2,2,kf->R_array);
	Matrix_Init(&kf->Q,2,2,kf->Q_array);
	Matrix_Init(&kf->I,2,2,kf->I_array);
	
	Matrix_Init(&kf->temp_x_had,2,1,kf->temp_x_had_array);
	Matrix_Init(&kf->x_had_estimate,2,1,kf->x_had_estimate_array);
	Matrix_Init(&kf->x_had,2,1,kf->x_had_array);
	
	Matrix_Init(&kf->p,2,2,kf->p_array);
	Matrix_Init(&kf->p_estimate,2,2,kf->p_estimate_array);
	Matrix_Init(&kf->temp_p,2,2,kf->temp_p_array);
	
	Matrix_Init(&kf->k_k,2,2,kf->k_k_array);
	Matrix_Init(&kf->z,2,1,kf->z_array);
	
	Matrix_Init(&kf->temp_1,2,2,kf->temp_1_array);
	Matrix_Init(&kf->temp_2,2,2,kf->temp_2_array);
	Matrix_Init(&kf->temp_3_2x1,2,1,kf->temp_3_2x1_array);
	Matrix_Init(&kf->temp_4_2x1,2,1,kf->temp_4_2x1_array);

}
void kalman_matrix_init(kalman_t*kf)
{
		//矩阵初始化
	Matrix_Init(&kf->A,2,2,kf->A_array);
	Matrix_Init(&kf->A_T,2,2,kf->A_T_array);
	Matrix_Init(&kf->H,2,2,kf->H_array);
	Matrix_Init(&kf->R,2,2,kf->R_v_array);
	Matrix_Init(&kf->Q ,2,2,kf->Q_v_array);
	Matrix_Init(&kf->I,2,2,kf->I_array);
	
	Matrix_Init(&kf->temp_x_had,2,1,kf->temp_x_had_array);
	Matrix_Init(&kf->x_had_estimate,2,1,kf->x_had_estimate_array);
	Matrix_Init(&kf->x_had,2,1,kf->x_had_array);
	
	Matrix_Init(&kf->p,2,2,kf->p_array);
	Matrix_Init(&kf->p_estimate,2,2,kf->p_estimate_array);
	Matrix_Init(&kf->temp_p,2,2,kf->temp_p_array);
	
	Matrix_Init(&kf->k_k,2,2,kf->k_k_array);
	Matrix_Init(&kf->z,2,1,kf->z_array);
	
	Matrix_Init(&kf->temp_1,2,2,kf->temp_1_array);
	Matrix_Init(&kf->temp_2,2,2,kf->temp_2_array);
	Matrix_Init(&kf->temp_3_2x1,2,1,kf->temp_3_2x1_array);
	Matrix_Init(&kf->temp_4_2x1,2,1,kf->temp_4_2x1_array);
}
void priori_estimate_x(kalman_t*kf)
{
	//step.1先验估计
	Matrix_Multiply(&kf->A,&kf->temp_x_had,&kf->x_had_estimate);
	
}
void priori_error_p(kalman_t*kf)
{
	//step.2误差的先验协方差矩阵
	Matrix_Multiply(&kf->A,&kf->temp_p,&kf->temp_1);
	Matrix_Multiply(&kf->temp_1,&kf->A_T,&kf->temp_2);
	Matrix_Add(&kf->temp_2,&kf->Q,&kf->p_estimate);
}
void kalman_gain(kalman_t*kf)
{
	//step.3卡尔曼增益
	Matrix_Multiply(&kf->H,&kf->p_estimate,&kf->temp_1);
	Matrix_Multiply(&kf->temp_1,&kf->H,&kf->temp_2);
	Matrix_Add(&kf->temp_2,&kf->R,&kf->temp_1);
	Matrix_Inverse(&kf->temp_1,&kf->temp_2);
	Matrix_Multiply(&kf->p_estimate,&kf->temp_2,&kf->temp_1);
	Matrix_Multiply(&kf->temp_1,&kf->H,&kf->k_k);
}
void x_had_updata(kalman_t*kf)
{
	//step.4后验估计
	Matrix_Multiply(&kf->H,&kf->x_had_estimate,&kf->temp_3_2x1);
	Matrix_Subtract(&kf->z,&kf->temp_3_2x1,&kf->temp_4_2x1);
	Matrix_Multiply(&kf->k_k,&kf->temp_4_2x1,&kf->temp_3_2x1);
	Matrix_Add(&kf->x_had_estimate,&kf->temp_3_2x1,&kf->x_had);
}
void error_p_updata(kalman_t*kf)
{
	//step.5误差更新
	Matrix_Multiply(&kf->k_k,&kf->H,&kf->temp_1);
	Matrix_Subtract(&kf->I,&kf->temp_1,&kf->temp_2);
	Matrix_Multiply(&kf->temp_2,&kf->p_estimate,&kf->p);
}
void kalman_filter_updata(kalman_t*kf)
{
	if(kf==&kalman_yaw_t)
	{
		kalman_matrix_init(kf);
	}
	else pos_kalman_matrix_init(kf);
//	pos_kalman_matrix_init(kf);
	
	if(kf->updata_flag)
	{
		priori_estimate_x(kf);//step.1先验估计
		priori_error_p(kf);//step.2误差的先验协方差矩阵
		kalman_gain(kf);//step.3卡尔曼增益
		x_had_updata(kf);//step.4后验估计
		error_p_updata(kf);//step.5误差更新
	}
	else 
	{
		priori_estimate_x(kf);//step.1先验估计
	}
 if(kf->z_array[0]>kf->temp_x_had_array[0])
 {
	 priori_estimate_x(kf);
 }
	memcpy (kf->temp_x_had_array,kf->x_had_array,sizeof(kf->temp_x_had_array));
	memcpy (kf->temp_p_array,kf->p_array,sizeof(kf->temp_p_array));
	
}
void Kalman_measureValue_update(void)
{
	kalman_x_t.z.pData[0]=solve_t.xw_nfilter;
	kalman_x_t.z.pData[1]=solve_t.vxw_nfilter;
	kalman_y_t.z.pData[0]=solve_t.yw_nfilter;
	kalman_y_t.z.pData[1]=solve_t.vyw_nfilter;
	kalman_yaw_t.z.pData[0]=solve_t.tar_yaw_nfilter;
	kalman_yaw_t.z.pData[1]=solve_t.v_yaw_nfilter;
	kalman_x_t.updata_flag=1;
	kalman_y_t.updata_flag=1;
	kalman_yaw_t.updata_flag=1;
}
