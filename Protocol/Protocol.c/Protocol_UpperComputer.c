/**
 * @file Protocol_UpperComputer.c
 * @author Why
 * @brief 跟上位机通信的协议
 * @frame 帧头 0x42 0x52  —— ASCII是BR
 *  	  对于弹道数据，第三个字节是命令码,0xCD；
 *  	  第四个字节是数据帧的长度；
 *  	  之后是数据帧，弹道有关的有一个字节
 *		  下位机发给上位机是射速，上位机发给下位机是云台100倍的俯仰*弧度*；
 *	  	  帧尾 一字节的CRC8
 * @version 0.2
 * @date 2024-1-28
 *
 */
#include "Protocol_UpperComputer.h"

float Auto_Aim_Yaw;
float Auto_Aim_Pitch;
uint8_t chassis_mode;//底盘跟随/小陀螺/缺血回城模式  
uint8_t cloud_mode = CLOUD_SEARCH_MODE;//自瞄锁敌/扫描索敌模式		
bool lack_blood_son_mode ;//缺血回城模式下的子模式	
uint8_t Rec_flag;
float Big_Yaw_Angle = 0;//大YAW当前角度					//仍待解算
float chassis_world_angle_offset_init = 0;
float32_t big_yaw_coordinate_vx = 0;//大YAW坐标系下目标vx
float32_t big_yaw_coordinate_vy = 0;//大YAW坐标系下目标vy
float32_t chassis_coordinate_vx = 0;//底盘坐标系下目标vx
float32_t chassis_coordinate_vy = 0;//底盘坐标系下目标vy
float32_t chassis_coordinate_vz = 0;//底盘坐标系下目标vz
float32_t spinning_vz = 0;//当前小陀螺速度，单位rad/s	

// 测试debug显示用的变量
float a, b;

/**
 * @brief  UpperCom下位机与上位机通信，向上位机发送信息。使用USB
 * @param  void
 * @retval void
 */
void UpperCom_Send_To_Up(uint8_t COM)
{
	uint8_t UpperCom_Send_Buffer[UpperCom_MAX_BUF];

	static uint16_t mark = 0;//时间戳

	/* 先录入帧头 */
	UpperCom_Send_Buffer[0] = 0x42;
	UpperCom_Send_Buffer[1] = 0x52;
	UpperCom_Send_Buffer[2] = COM;
	/* 再根据命令码加上数据帧和CRC8的校验 */
	if (COM == 0xCD)
	{
		if (mark++ >= 200) mark = 0;
		UpperCom_Send_Buffer[3] = 23; // 数据包包含的字节数
		memcpy(&UpperCom_Send_Buffer[4], &cloud_mode, sizeof(uint8_t));				//云台模式
		memcpy(&UpperCom_Send_Buffer[5], &mark, sizeof(uint16_t));					//时间戳
		memcpy(&UpperCom_Send_Buffer[7], &spinning_vz, sizeof(float32_t));		    //当前小陀螺速度，单位rad/s
		memcpy(&UpperCom_Send_Buffer[11], &Big_Yaw_Angle, sizeof(float32_t));		//大YAW当前角度(世界坐标系下)，单位rad

		Append_CRC8_Check_Sum(UpperCom_Send_Buffer, 5 + UpperCom_Send_Buffer[3]); // 5+x，x代表数据包包含的数据字节数。
	}
	CDC_Transmit_FS(UpperCom_Send_Buffer, sizeof(UpperCom_Send_Buffer)); // usb发送
	memset(UpperCom_Send_Buffer, 0, UpperCom_MAX_BUF);
}

/**
 * @brief 十六进制转float
 */
static float R2float(uint8_t *p)
{
	float r;
	memcpy(&r, p, 4);
	return r;
}

/**
 * @brief 十六进制转int16_t
 */
static float R2int16(uint8_t *p)
{
	int16_t r;
	memcpy(&r, p, 2);
	return r;
}

/**
 * @brief  UpperCom下位机与上位机通信，接受上位机的信息。使用USB
 * @param  *Rec 接收到的一帧数据
 * @retval void
 */
void UpperCom_Receive_From_Up(uint8_t Rec[])
{
	/* 先检验帧头 */
	if (Rec[0] != 0x42 || Rec[1] != 0x52)
		return;
	/* 再根据命令码CRC校验 */
	switch (Rec[2])
	{
	case 0xFF:
	case 0x00:
		break;
	case 0xCD:
		if (!Verify_CRC8_Check_Sum(Rec, 5 + Rec[3]))
			return;

		chassis_mode = Rec[4];			//底盘模式
		memcpy(&chassis_world_angle_offset_init, &Rec[5], sizeof(float32_t));//初始底盘角度偏移值,单位rad
		Rec_flag=Rec[9];
		lack_blood_son_mode = Rec[9];	//缺血回城子模式
		memcpy(&big_yaw_coordinate_vx, &Rec[10], sizeof(float32_t));//大YAW坐标系下目标vx
		memcpy(&big_yaw_coordinate_vy, &Rec[14], sizeof(float32_t));//大YAW坐标系下目标vy
		
		memcpy(&chassis_coordinate_vx, &Rec[18], sizeof(float32_t));//底盘坐标系下目标vx
		memcpy(&chassis_coordinate_vy, &Rec[22], sizeof(float32_t));//底盘坐标系下目标vy
		memcpy(&chassis_coordinate_vz, &Rec[26], sizeof(float32_t));//底盘坐标系下目标vz

		break;
	default:
		return;
	}
}
