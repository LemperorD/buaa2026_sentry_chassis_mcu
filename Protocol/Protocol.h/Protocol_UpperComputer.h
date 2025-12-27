/**
 * @file Protocol_UpperComputer.c
 * @author Why
 * @brief 跟上位机通信的协议
 * @version 0.1
 * @date 2023-10-02
 *
 */

#ifndef PROTOCOL_UPPERCOMPUTER_H
#define PROTOCOL_UPPERCOMPUTER_H

#include "CRC.h"
#include "usart.h"
#include "PID.h"
#include "BSP_BoardCommunication.h"
#include "Cloud_Control.h"
#include "usbd_cdc_if.h"
#include <math.h>  

void UpperCom_Receive_From_Up(uint8_t Rec[]);
void UpperCom_Send_To_Up(uint8_t COM);

#define UpperCom_MAX_BUF 31

// 底盘模式定义（统一使用一套）
#define CHASSIS_FOLLOW_MODE   1  // 底盘跟随模式
#define SPINNING_MODE         2  // 小陀螺模式  
#define BLOOD_RETURN_MODE     3  // 缺血回城模式
//cloud_mode
#define ENEMY_LOCKED 0
#define ENEMY_SEARCH 1
// 缺血回城子模式定义
#define PATH_CLEAR            1  // 路径通畅
#define PATH_BLOCKED          0  // 路径堵死

#define M_PI 3.14159265358979323846

// 外部变量声明
extern uint8_t chassis_mode;//底盘跟随/小陀螺/缺血回城模式
extern uint8_t cloud_mode;//自瞄锁敌/扫描锁敌模式
extern bool lack_blood_son_mode;//缺血回城模式下的子模式

extern float32_t chassis_world_angle_offset_init;//初始底盘角度偏移值
extern float32_t big_yaw_coordinate_vx;//大YAW坐标系下目标vx
extern float32_t big_yaw_coordinate_vy;//大YAW坐标系下目标vy
extern float32_t chassis_coordinate_vx;//底盘坐标系下目标vx
extern float32_t chassis_coordinate_vy;//底盘坐标系下目标vy
extern float32_t chassis_coordinate_vz;//底盘坐标系下目标vz

#endif
