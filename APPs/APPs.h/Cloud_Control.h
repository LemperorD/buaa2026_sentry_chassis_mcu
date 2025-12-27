/**
 * @file Cloud_control.c
 * @author Cyx
 * @brief 
 * @version 0.1
 * @date 2023-08-15
 * 
 * @copyright 
 * 
 */
#ifndef __CLOUD_CONTROL_H
#define __CLOUD_CONTROL_H

#include "PID.h"
#include "kalman_filter.h"
#include "DM_Motor.h"
#include "typedef.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "FuzzyPID.h"
#include "FeedForward.h"
#include "Steer_Omni_Chassis.h"
#include "BSP_Can.h"
#include "Extern_Handles.h"
#include "BSP_BoardCommunication.h"
#include "Saber_C3.h"
#include "Gimbal_Chassis_Pitch_Angle.h"
// ? Cloud_Control.h ?????????
extern uint8_t chassis_mode;        // ????
extern uint8_t cloud_mode;          // ????  
extern bool lack_blood_son_mode;    // ???????
extern float Big_Yaw_Angle;

#define CHASSIS_GYRO_MODE       1   // ?????
#define CHASSIS_RETURN_MODE     2   // ??????

#define CLOUD_SEARCH_MODE       0   // ??????
#define CLOUD_AIM_MODE          1   // ??????
//重新安装电机或移用代码时需要重新测量这些值（toalAngle）后再允许运动。

#define Cloud_Yaw_Center 4060
#define Cloud_Yaw_ReverseCenter 7809
#define follow_mode_angle 0;


/* 云台电机角度结构体 */
typedef struct
{
    float Yaw_Raw; 			      //yaw轴原始数据
    float Pitch_Raw;   		    //pitch轴原始数据
    float Target_Yaw; 		    //云台目标yaw轴
    float Target_Pitch;   	  //云台目标pitch轴
	  float Vision_Yaw_Delta;	  //视觉Yaw轴数据(差值)
	  float Vision_Pitch_Delta;	//视觉Pitch轴数据(差值)
} Cloud_t;


typedef struct
{
    void (*Cloud_Init)(void);
    void (*Cloud_Sport_Out)(void);
		void (*Cloud_Yaw_Angle_Set)(void);
		void (*PID_Clear_Yaw)(void);
} Cloud_FUN_t;

void Cloud_Init(void);

extern Cloud_t Cloud;
extern Cloud_FUN_t Cloud_FUN;
void Gimbal_Pitch_Translate(void);


#define Cloud_FUNGroundInit               \
    {                                     \
        &Cloud_Init,                      \
		&Cloud_Sport_Out,			  	  \
			&Cloud_Yaw_Angle_Set,			\
			&PID_Clear_Yaw,					\
    }

/********函数声明********/
void Cloud_Init(void);
void Cloud_Yaw_Angle_Set(void);
void Cloud_Sport_Out(void);
void Cloud_Self_Yaw(void);
void PID_Clear_Yaw(void);
	
#endif /* __CLOUD_CONTROL_H */
