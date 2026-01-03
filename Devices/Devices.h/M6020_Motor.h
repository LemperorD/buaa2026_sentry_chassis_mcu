/**
 * @file M6020_Motor.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __M6020_MOTOR_H
#define __M6020_MOTOR_H

#include "can.h"
#include "typedef.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "PID.h"
#include "BSP_Can.h"

#define M6020_READID_START 0x206 //��IDΪ1ʱ�ı���ID
#define M6020_READID_END 0x207
#define M6020_SENDID 0x1FF //1~4�ĵ����0x2FFΪ5~7

#define M6020_CurrentRatio 0f //���ⶨ
#define M6020_MaxOutput 30000 //���͸������������ֵ
#define M6020_mAngle 8191     //6020�Ļ�е�Ƕ����ֵ0~8191��MachineAngle

#define M6020_mAngleRatio 22.7527f //��е�Ƕ�����ʵ�Ƕȵı���

#define M6020_getRoundAngle(rAngle) rAngle / M6020_mAngleRatio //��е�Ƕ�����ʵ�Ƕȵı���

#define M6020_FunGroundInit        \
    {                              \
		        &M6020_setVoltage,		   \
		      	&M6020_getInfo,		   \
            &M6020_setTargetAngle, \
            &M6020_Reset,          \
    }

typedef struct
{
	uint16_t motor_id;
    uint16_t realAngle;  //�������Ļ�е�Ƕ�
    int32_t realSpeed;   //���������ٶ�
    int16_t realCurrent; //��������ʵ��ת�ص���
    uint8_t temperture;  //�������ĵ���¶�
    uint16_t lastAngle;  //�ϴεĽǶ�
	
    float targetSpeed; //Ŀ���ٶ�
    int32_t targetAngle; //Ŀ��Ƕ�
	
    int16_t turnCount;   //ת����Ȧ��
    float totalAngle;  //�ۻ��ܹ��Ƕ�

    int16_t outCurrent; //�������

    uint8_t InfoUpdateFlag;   //��Ϣ��ȡ���±�־
    uint16_t InfoUpdateFrame; //֡��
    uint8_t OffLineFlag;      //�豸���߱�־
	
	positionpid_t l_pid_object;
	positionpid_t v_pid_object;
		
} M6020s_t;

typedef enum
{
    //��Ҫע���뱨�Ľ��ܺ�������Ӧ����
    //M6020_PITCH_Right = 0,
	M6020_Chassis1 = 0,
	M6020_Chassis2,
	Totalnum
}M6020Name_e;

typedef struct
{
	void (*M6020_setVoltage)(int16_t uq1, int16_t uq2, int16_t uq3, int16_t uq4, uint8_t *data);
    void (*M6020_getInfo)(Can_Export_Data_t RxMessage);
    void (*M6020_setTargetAngle)(M6020s_t *M6020, int32_t angle);
    void (*M6020_Reset)(M6020s_t *m6020);
	
} M6020_Fun_t;

/********ȫ�ֱ�������********/
extern M6020s_t M6020s_Chassis1;
extern M6020s_t M6020s_Chassis2;
extern M6020_Fun_t M6020_Fun;

/********��������********/
void M6020_Init(M6020s_t *motor, uint16_t _motor_id);
void M6020_setVoltage(int16_t uq1, int16_t uq2, int16_t uq3, int16_t uq4, uint8_t *data);
void M6020_getInfo(Can_Export_Data_t RxMessage);
void M6020_setTargetAngle(M6020s_t *M6020, int32_t angle);
void M6020_Reset(M6020s_t *m6020);
void M6020_velocity_change(M6020s_t *motor,pid_control model,CAN_HandleTypeDef *hcan,float target);
void M6020_location_change(M6020s_t *motor,pid_control model,int16_t target,int16_t real);

#endif /* __M3508_MOTOR_H */
