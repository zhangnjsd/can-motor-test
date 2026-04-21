#include "solvetrajectory.h"

struct SolveTrajectoryParams solve_t;

float fly_time;
uint8_t  fire_flag;
uint8_t  shoot_flag;
uint32_t error_time;
uint32_t first_fire_time;
uint32_t now_time;
uint16_t debug_a;
uint16_t debug_b;

double  z_2,z_1;
Ordinary_Least_Squares_t my_OLS_t;
My_Resolve my_resolve_t;
Aimerro_data aimerro_data_t;

/*计算空间距离*/
float target_space_distance(double x,double y,double z)
{
	return sqrt(pow(x,2)+pow(y,2)+pow(z,2));
}
/*计算平面距离*/
float target_plane_distance(double x,double y)
{
	return sqrt(x*x+y*y);
}

void resolve(uint16_t order,float y,float last_y,uint8_t mode)
{
	my_resolve_t.dt=DWT_GetDeltaT((void *)&my_resolve_t.my_cnt);
	my_OLS_t.Order=order;
	if(mode==0)
	{
		if((y-last_y)<0)
		{
			OLS_Update(&my_OLS_t,my_resolve_t.dt,y);
			solve_t.yaw_forecast=solve_t.tar_yaw;
			my_resolve_t.m=20;
		}
		if((y-last_y)>0)
		{
			solve_t.resolve_yaw=my_OLS_t.k*my_resolve_t.m*my_resolve_t.dt+my_OLS_t.b;
			solve_t.yaw_forecast=solve_t.resolve_yaw;
			my_resolve_t.m++;
		}
	}
	else if(mode==1)
	{
		if((y-last_y)>0)
		{
			OLS_Update(&my_OLS_t,my_resolve_t.dt,y);
			solve_t.yaw_forecast=solve_t.tar_yaw;
			my_resolve_t.m=20;
		}
		if((y-last_y)<0)
		{
			solve_t.resolve_yaw=my_OLS_t.k*my_resolve_t.m*my_resolve_t.dt+my_OLS_t.b;
			solve_t.yaw_forecast=solve_t.resolve_yaw;
			my_resolve_t.m++;
		}
	}
}
/*解算所需参数初始化*/
void SolveTrajectoryInit(void)
{
	solve_t.k_1 = 0.038;
	solve_t.k_2 = 0.0001;
	solve_t.bullet_type =  BULLET_17;
	// solve_t.current_v = current_bullet_v;
	solve_t.current_v = 23;
	solve_t.tar_yaw=0;	
	solve_t.current_pitch = 0;
	solve_t.current_yaw = 0;
	solve_t.s_bias = 0.1040;
	solve_t.z_bias = 0.0000;
	solve_t.bias_time = 31.5;
	solve_t.m=0.0032;
	OLS_Init(&my_OLS_t,20);
	my_resolve_t.m=20;
	solve_t.target_lost_flag=0;
}

/*
@brief 单方向空气阻力弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/

float monoDirectionalAirResistanceModel(float s, float v, float angle)
{

    //t为给定v与angle时的飞行时
    fly_time = (float)((exp(solve_t.k_1 * s) - 1) / (solve_t.k_1 * v * arm_cos_f32(angle)));

    //z为给定v与angle时的高度
//		z_2=(((solve_t.m*GRAVITY+solve_t.k_2*v*arm_sin_f32(angle))*(solve_t.k_2*s-v*arm_cos_f32(angle)*solve_t.m))/(solve_t.k_2*solve_t.k_2*v*arm_cos_f32(angle)))+
//		((GRAVITY*solve_t.m*solve_t.m*log(-((solve_t.k_2*s-v*arm_cos_f32(angle)*solve_t.m)/(v*arm_cos_f32(angle)*solve_t.m))))/(solve_t.k_2*solve_t.k_2))+
//		(((solve_t.m*GRAVITY+solve_t.k_2*v*arm_sin_f32(angle))*solve_t.m)/(solve_t.k_2*solve_t.k_2));
		
    z_1 = (float)(v * arm_sin_f32(angle) * (fly_time) - (float)GRAVITY * (fly_time) * (fly_time) / 2);
	
//		if(nuc_tx_u.Send_pack_t.aim_x<2.5)
//			return z_1;
//    else
			return z_1;
}
/*
@brief 解算打击目标所需要的pitch
@param x:目标水平距离
@param z:目标高度
@param v:当前射速
@return angle_pitch:所需角度
*/
float GimbalControlGetPitch(float x, float z, float v)
{
	  float z_temp, z_actual, dz;
    float angle_pitch;
    z_temp = z;
    for (int i = 0; i < 20; i++)
    {
        angle_pitch = (float)atan2(z_temp, x); // rad
        z_actual = monoDirectionalAirResistanceModel(x, v, angle_pitch);
        dz = 0.3f*(z - z_actual);
        z_temp = z_temp + dz;
        if (fabsf(dz) < 0.00001f)
        {
            break;
        }
    }
    return angle_pitch;
}
/*
@brief 根据最优决策得出被击打装甲板 自动解算弹道
@param pitch:rad  传出pitch
@param yaw:rad    传出yaw
@param aim_x:传出aim_x  打击目标的x
@param aim_y:传出aim_y  打击目标的y
@param aim_z:传出aim_z  打击目标的z
*/
void autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z)
{
	//线性预测
	float timeDelay= solve_t.bias_time/1000.0f + fly_time;
	solve_t.tar_yaw+=solve_t.v_yaw*timeDelay;

	resolve(20,solve_t.tar_yaw,solve_t.last_yaw,solve_t.direction_flag);
	solve_t.xw_forecast=solve_t.xw-solve_t.r1*cos(solve_t.yaw_forecast)+solve_t.vxw*timeDelay;
	solve_t.yw_forecast=solve_t.yw-solve_t.r1*sin(solve_t.yaw_forecast)+solve_t.vyw*timeDelay;
	float temp_z=solve_t.zw+solve_t.vzw*timeDelay;
	get_Control_coefficient(fabs(solve_t.v_yaw));
//	*aim_z=solve_t.zw+solve_t.vzw*timeDelay;
	
	  /*方案一：将坐标系划分为四个象限*/
	  if(solve_t.yw<0&&solve_t.xw>0)
	{
		solve_t.quadrant_flag=FIRST_QUADRANT;
	}
	  else if(solve_t.yw>0&&solve_t.xw>0)
	{
		solve_t.quadrant_flag=SECOND_QUADRANT;
	}
		else if(solve_t.yw>0&&solve_t.xw<0)
	{
		solve_t.quadrant_flag=THIRD_QUADRANT;
	}
		else if(solve_t.yw<0&&solve_t.xw<0)
	{
		solve_t.quadrant_flag=FOURTH_QUADRANT;
	}
	
	switch(solve_t.quadrant_flag)
	{
		case FIRST_QUADRANT:
		{
			solve_t.spin_yaw = solve_t.yaw_forecast+atanf(fabs(solve_t.yw)/fabs(solve_t.xw));
			break;
		}
		case SECOND_QUADRANT:
		{
			solve_t.spin_yaw = solve_t.yaw_forecast-atanf(fabs(solve_t.yw)/fabs(solve_t.xw));
			break;
		}
		case THIRD_QUADRANT:
		{
			solve_t.spin_yaw = solve_t.yaw_forecast-(PI-atanf(fabs(solve_t.yw)/fabs(solve_t.xw)));
			break;
		}
		case FOURTH_QUADRANT:
		{
			solve_t.spin_yaw = solve_t.yaw_forecast+(PI-atanf(fabs(solve_t.yw)/fabs(solve_t.xw)));
			break;
		}
	}

  solve_t.yaw_max=acosf(solve_t.r1/target_plane_distance(solve_t.xw,solve_t.yw))/*-atanf(fabs(solve_t.yw)/fabs(solve_t.xw))*/;
	solve_t.debug_a=acosf(solve_t.r1/target_plane_distance(solve_t.xw,solve_t.yw));
//	solve_t.debug_b=atanf(fabs(solve_t.yw)/fabs(solve_t.xw));
	solve_t.debug_b=solve_t.yaw_max-solve_t.spin_yaw;
	if(solve_t.direction_flag==1)
	{
//		if((solve_t.yaw_max-solve_t.yaw_forecast)<0.2f*solve_t.yaw_max)
		if((solve_t.yaw_max-solve_t.spin_yaw)<solve_t.Control_coefficient*solve_t.yaw_max)
		{
			switch (solve_t.armor_num)
			{
				case ARMOR_NUM_BALANCE:
				{
					*aim_x=solve_t.xw-solve_t.r1*cos(solve_t.yaw_forecast-PI)+solve_t.vxw*timeDelay;
					*aim_y=solve_t.yw-solve_t.r1*sin(solve_t.yaw_forecast-PI)+solve_t.vyw*timeDelay;
					*aim_z=temp_z+solve_t.dz;
					*yaw=atan2f(*aim_y,*aim_x);
					break ;
				}
				case ARMOR_NUM_OUTPOST:
				{
					*aim_x=solve_t.xw-solve_t.r1*cos(solve_t.yaw_forecast-2*PI/3)+solve_t.vxw*timeDelay;
					*aim_y=solve_t.yw-solve_t.r1*sin(solve_t.yaw_forecast-2*PI/3)+solve_t.vyw*timeDelay;
					*aim_z=temp_z+solve_t.dz;
					*yaw=atan2f(*aim_y,*aim_x);
					break ;
				}
				case ARMOR_NUM_NORMAL:
				{
					*aim_x=solve_t.xw-solve_t.r1*cos(solve_t.yaw_forecast-PI/2)+solve_t.vxw*timeDelay;
					*aim_y=solve_t.yw-solve_t.r1*sin(solve_t.yaw_forecast-PI/2)+solve_t.vyw*timeDelay;
					*aim_z=temp_z+solve_t.dz;
					*yaw=atan2f(*aim_y,*aim_x);
					break ;
				}
			}
		}
		else 
		{
			*yaw=atan2f(solve_t.yw_forecast,solve_t.xw_forecast);
			*aim_x=solve_t.xw_forecast;
			*aim_y=solve_t.yw_forecast;
			*aim_z=temp_z;
		}
	}
	if(solve_t.direction_flag==0)
	{
//		if(((-solve_t.yaw_max)-solve_t.yaw_forecast)>-0.2f*solve_t.yaw_max)
		 if(((-solve_t.yaw_max)-solve_t.spin_yaw)>-solve_t.Control_coefficient*solve_t.yaw_max)
		{
			switch (solve_t.armor_num)
			{
				case ARMOR_NUM_BALANCE:
				{
					
					*aim_x=solve_t.xw-solve_t.r1*cos(solve_t.yaw_forecast+PI)+solve_t.vxw*timeDelay;
					*aim_y=solve_t.yw-solve_t.r1*sin(solve_t.yaw_forecast+PI)+solve_t.vyw*timeDelay;
					*aim_z=temp_z+solve_t.dz;
					*yaw=atan2f(*aim_y,*aim_x);
					break ;
				}
				case ARMOR_NUM_OUTPOST:
				{
//					solve_t.yaw_forecast=solve_t.tar_yaw+2*PI/3;
					*aim_x=solve_t.xw-solve_t.r1*cos(solve_t.yaw_forecast+2*PI/3)+solve_t.vxw*timeDelay;
					*aim_y=solve_t.yw-solve_t.r1*sin(solve_t.yaw_forecast+2*PI/3)+solve_t.vyw*timeDelay;
					*aim_z=temp_z+solve_t.dz;
					*yaw=atan2f(*aim_y,*aim_x);
					break ;
				}
				case ARMOR_NUM_NORMAL:
				{
					
					*aim_x=solve_t.xw-solve_t.r1*cos(solve_t.yaw_forecast+PI/2)+solve_t.vxw*timeDelay;
					*aim_y=solve_t.yw-solve_t.r1*sin(solve_t.yaw_forecast+PI/2)+solve_t.vyw*timeDelay;
					*aim_z=temp_z+solve_t.dz;
					*yaw=atan2f(*aim_y,*aim_x);
					break ;
				}
			}
		}
		else 
		{
			*yaw=atan2f(solve_t.yw_forecast,solve_t.xw_forecast);
			*aim_x=solve_t.xw_forecast;
			*aim_y=solve_t.yw_forecast;
			*aim_z=temp_z;
		}
	}
	*pitch=-GimbalControlGetPitch(target_plane_distance(*aim_x,*aim_y)-solve_t.s_bias,*aim_z+solve_t.z_bias,solve_t.current_v);
	solve_t.last_yaw=solve_t.tar_yaw;
}

/*
*  开火逻辑
*/
uint8_t Autofire_Control(float pitch,float yaw,INS_t *ins)
{
	aimerro_data_t.error_pitch=fabs(fabs(pitch*57.2957795f)-fabs(ins->Pitch));
	aimerro_data_t.error_yaw=fabs(fabs(yaw)-fabs(ins->YawTotalAngle));
	now_time = HAL_GetTick();
	error_time=now_time-first_fire_time;
	if((shoot_3508_ref-abs(shoot_3508_motor_t[0].Gimbal_motor_measure_t->speed_rpm))<100&&(shoot_3508_ref-abs(shoot_3508_motor_t[1].Gimbal_motor_measure_t->speed_rpm))<100)
	{
		shoot_flag=1;
	}
	else shoot_flag=0;
	
	if(aimerro_data_t.error_pitch<ERROR_PITCH&&
		 aimerro_data_t.error_yaw<ERROR_YAW&&
	   shoot_flag==1)
	{
        fire_flag=1;
		first_fire_time=now_time;
	}
	
	else if(error_time<FIRE_CONTINUE_TIME)
	{
		fire_flag=1;
	}
	else fire_flag=0;
	
	return fire_flag;
}

//void resolve(uint16_t order,float y)
//{
//	my_resolve_t.dt=DWT_GetDeltaT((void *)&my_resolve_t.my_cnt);
//	my_OLS_t.Order=order;
//	OLS_Init(&my_OLS_t,order);
//	OLS_Update(&my_OLS_t,my_resolve_t.dt,y);
//}
void get_Control_coefficient(float w)
{
	if(solve_t.armor_num==ARMOR_NUM_OUTPOST)
	{
     solve_t.Control_coefficient=0.3f;
	}
	else
	{	
		if(w>0&&w<0.5f)
		{
			solve_t.Control_coefficient=0.45f;
		}
		else if(w>0.5f&&w<1)
		{
			solve_t.Control_coefficient=0.48f;
		}
		else if(w>1&&w<2)
		{
			solve_t.Control_coefficient=0.5f;
		}
		else if(w>2&&w<3)
		{
			solve_t.Control_coefficient=0.53f;
		}
		else if(w>3)
		{
			solve_t.Control_coefficient=0.65f;
		}
	}

}
