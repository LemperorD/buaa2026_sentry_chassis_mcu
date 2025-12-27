/**
 * @file BSP_BoardCommunication.h
 * @author lxr(784457420@qq.com)
 * @brief 
 * @version 1.0
 * @date 2023-9-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef BSP_BOARDCOMMUNICATION_H 
#define	BSP_BOARDCOMMUNICATION_H

#include "main.h"
#include "BSP_Can.h"
#include "Extern_Handles.h"
#include "queue.h"
#include "Protocol_Judgement.h"
#include "Steer_Omni_Chassis.h"
#include "Cloud_Control.h"
#include "Saber_C3.h"
#include "M2006_Motor.h"


// CAN���ĵı�ʶ�������ݳ���
#define CAN_ID_CHASSIS 0x10f // ����CAN���ĵ�������IDΪ0x10f
#define CAN_ID_GIMBAL  0x11f // ��̨����IDΪ0x11f
#define CAN_ID_KEYCOMMAND 0x22f // ͼ���������IDλ0x22f
#define CAN_ID_6006_BIG_YAW 0X12f

/*����ģʽ 0Ϊ������1Ϊ��¼����Ϊmid����Ϊdown��*/
#define model_Normal 0
#define model_Record 1
extern uint16_t big_yaw_ememy_position[2];

#define Board2_FunGroundInit   \
	{                          \
		&Board2_getChassisInfo,       \
		&Board2_getGimbalInfo,		  \
		&Board2_To_1,                 \
	}


// ����CAN���ĵĽṹ��
typedef struct {
    int16_t x_velocity;
    int16_t y_velocity;
    int16_t z_rotation_velocity;
	int16_t pitch_velocity;
	int16_t big_yaw_velocity;
	uint8_t shoot_state;
	int16_t yaw_realAngle;
	uint8_t modelFlag;
	uint8_t shoot_Speed;
	uint8_t AutoAimFlag;         // ���鿪��
	uint8_t change_Flag;				//����
	uint8_t fric_Flag;					//Ħ����
	uint8_t tnndcolor;
	int16_t Gimbal_Chassis_Pitch_Angle;
	int8_t feipo_Flag;            //���¿���
	uint16_t Blood_Volume;         //???????
	uint8_t game_start;
	uint8_t yaw_choose;
	float Speed_Bullet;
} ControlMessge;

extern ControlMessge ControlMes;
extern float Auto_Aim_Yaw;

typedef struct
{
	void (*Board2_getChassisInfo)(Can_Export_Data_t RxMessage);
	void (*Board2_getGimbalInfo)(Can_Export_Data_t RxMessage);
	void (*Board2_To_1)(void);
}Board2_FUN_t;

extern Board2_FUN_t Board2_FUN;

#endif
