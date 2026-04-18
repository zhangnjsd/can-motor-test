#ifndef __SOLVETRAJECTORY__
#define __SOLVETRAJECTORY__
#include "arm_math.h"
#include "controller.h"
#include "stdint.h"
#include "math.h"
#include "ins_task.h"
#include "NUC_receive.h"
#include "Gimbal_task_fun.h"


#define GRAVITY 9.78f
#define ERROR_PITCH 0.4f
#define ERROR_YAW 0.4f
#define FIRE_CONTINUE_TIME 10

enum ARMOR_ID
{
    ARMOR_OUTPOST = 0,
    ARMOR_HERO = 1,
    ARMOR_ENGINEER = 2,
    ARMOR_INFANTRY3 = 3,
    ARMOR_INFANTRY4 = 4,
    ARMOR_INFANTRY5 = 5,
    ARMOR_GUARD = 6,
    ARMOR_BASE = 7
};

enum ARMOR_NUM
{
    ARMOR_NUM_BALANCE = 2,
    ARMOR_NUM_OUTPOST = 3,
    ARMOR_NUM_NORMAL = 4
};

enum BULLET_TYPE
{
    BULLET_17 = 0,
    BULLET_42 = 1
};

enum TARGET_QUADRANT
{
	FIRST_QUADRANT = 1,
	SECOND_QUADRANT,
	THIRD_QUADRANT,
	FOURTH_QUADRANT
};

struct SolveTrajectoryParams
{
    float k_1;             //弹道系数
    double  k_2;
    //自身参数
    enum BULLET_TYPE bullet_type;  //自身机器人类型 0-步兵 1-英雄
    float current_v;      //当前弹速
    float current_pitch;  //当前pitch
    float current_yaw;    //当前yaw

    //目标参数
    float xw;             //ROS坐标系下的x
    float yw;             //ROS坐标系下的y
    float zw;             //ROS坐标系下的z
    float vxw;            //ROS坐标系下的vx
    float vyw;            //ROS坐标系下的vy
    float vzw;            //ROS坐标系下的vz
	
    float xw_nfilter;             //ROS坐标系下的x
    float yw_nfilter;             //ROS坐标系下的y
    float vxw_nfilter;            //ROS坐标系下的vx
    float vyw_nfilter;            //ROS坐标系下的vy

    float tar_yaw;          //目标yaw
    float v_yaw;            //目标yaw速度
		float tar_yaw_nfilter;	//目标yaw     没有Karman的原始数据 
		float v_yaw_nfilter;		//目标yaw速度 没有Karman的原始数据 
	  float yaw_forecast;     //预测yaw
		float xw_forecast;      //预测x
		float yw_forecast;      //预测y
		float last_yaw;         //上一次的yaw
		float yaw_max;          //yaw阈值
		float resolve_yaw;      //利用最小二乘法修正后的yaw
		float spin_yaw;
			
    float r1;             //目标中心到前后装甲板的距离
    float r2;             //目标中心到左右装甲板的距离
    float dz;             //另一对装甲板的相对于被跟踪装甲板的高度差
    float bias_time;        //偏置时间
    float s_bias;         //枪口前推的距离
    float z_bias;         //yaw轴电机到枪口水平面的垂直距离
    enum ARMOR_ID armor_id;     //装甲板类型  0-outpost 6-guard 7-base
                                //1-英雄 2-工程 3-4-5-步兵 
    enum ARMOR_NUM armor_num;   //装甲板数字  2-balance 3-outpost 4-normal
		float m;                    //弹丸质量
	  bool tracking;                //识别标志
		
		uint8_t target_lost_flag;
		uint8_t receive_flag;
		uint8_t direction_flag;
		uint8_t quadrant_flag;
		float Control_coefficient;
		
		double debug_a;
		double debug_b;
};

typedef  struct
{
	float dt;
	uint32_t my_cnt;
	float k;
	float b;
	float m;
}My_Resolve;
typedef  struct
{
  float error_pitch;
  float error_yaw;
}Aimerro_data;
extern struct SolveTrajectoryParams solve_t;
extern void SolveTrajectoryInit(void);
extern void autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z);
extern uint8_t Autofire_Control(float pitch,float yaw,INS_t *ins);
extern void get_Control_coefficient(float w);
extern Aimerro_data aimerro_data_t;
#endif
