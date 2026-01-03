/**
 * @file BSP_BoardCommunication.c
 * @author lxr(784457420@qq.com)/ZS(2729511164@qq.com)
 * @brief
 * @version 1.0
 * @date 2023-9-15
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "BSP_BoardCommunication.h"

ControlMessge ControlMes;
void Board2_To_1(void);
void Board2_getChassisInfo(Can_Export_Data_t RxMessage);
void Board2_getGimbalInfo(Can_Export_Data_t RxMessage);

float big_yaw_velocity = 0;
float vx = 0;
float vy = 0;
float vw = 0;
uint16_t big_yaw_ememy_position[2] = {0, 0};
uint8_t BIG_YAW_MODE=1;
Board2_FUN_t Board2_FUN = Board2_FunGroundInit;

extern Saber_Angle_t Saber_Angle;

////�˺����������ձ��Ĺ����������ݲ����͡�
//void Board2_To_1(void)
//{
//	int16_t bullet_speed;
//	uint8_t IT_keycommand[8]={0};
//	uint8_t data[8] = {0};

//	// �ֱ���yaw��Ƕȣ�Saber��pitch��Ƕȣ���һ���ĵ��٣�װ�����?
//	data[0] = ControlMes.yaw_realAngle >> 8;
//	data[1] = ControlMes.yaw_realAngle;	
//	data[2] = ControlMes.Blood_Volume >> 8;
//	data[3] = ControlMes.Blood_Volume;
//	bullet_speed = (int16_t)(g_referee.shoot_data_.initial_speed * 1000);
//	data[4] = bullet_speed >> 8;
//	data[5] = bullet_speed;
//  data[6] |= (uint8_t)(ControlMes.tnndcolor & 0x01) <<0;
//	data[6] |= (uint8_t)(ControlMes.game_start & 0x01) <<1;
//	
//	data[7] = (uint8_t)J6006s_Yaw.realAngle6006;  // ?YAW?????8?
//	
//	//ͼ������ӳ�䣬�ֱ������x�ᡢy�ᡢz�ᣨz���ǹ��֣���������Ҽ�����ʱֻ�����������?
//	//��ֻ̨��Ҫ���ٿأ�������ϰ����ʱ��Ҫ��չIT_keycommand[8]��IT_keycommand[12]������λֱ�Ӹ�0����
//	//����������ң��������ӳ��ĸ�ʽһ��??
//	//	IT_keycommand[0] = ext_robot_keycommand.data.mouse_x >> 8;
//	//	IT_keycommand[1] = ext_robot_keycommand.data.mouse_x;
//	//	IT_keycommand[2] = ext_robot_keycommand.data.mouse_y >> 8;
//	//	IT_keycommand[3] = ext_robot_keycommand.data.mouse_y;
//	//	IT_keycommand[4] = ext_robot_keycommand.data.mouse_z >> 8;
//	//	IT_keycommand[5] = ext_robot_keycommand.data.mouse_z;
//	//	IT_keycommand[6] = ext_robot_keycommand.data.left_button_down;
//	//	IT_keycommand[7] = ext_robot_keycommand.data.right_button_down;
//	
//  //���ݷ���
//  Can_Fun.CAN_SendData(CAN_SendHandle, &hcan2, CAN_ID_STD, CAN_ID_GIMBAL, data);
//  //Can_Fun.CAN_SendData(CAN_SendHandle, &hcan2, CAN_ID_STD, CAN_ID_KEYCOMMAND, IT_keycommand);
//}
void Board2_To_1(void)
{
	int16_t bullet_speed;
	uint8_t data[8] = {0};
    uint8_t data2[8] = {0};

	//�������??
	data[0] = ControlMes.yaw_realAngle >> 8;
	data[1] = ControlMes.yaw_realAngle;	
	data[2] = ControlMes.Blood_Volume >> 8;
	data[3] = ControlMes.Blood_Volume;
	bullet_speed = (int16_t)(g_referee.shoot_data_.initial_speed * 1000);
	data[4] = bullet_speed >> 8;
	data[5] = bullet_speed;
    data[6] = 0;
    data[6] |= (uint8_t)(ControlMes.tnndcolor & 0x01) << 0;        // ?0:?????
    data[6] |= (uint8_t)(ControlMes.game_start & 0x01) << 1;       // ?1:??????
    data[6] |= (uint8_t)(chassis_mode & 0x03) << 4;                // ?2-3:????
    data[6] |= (uint8_t)(lack_blood_son_mode & 0x01) << 5;         // ?5:???????
    //��������
    Can_Fun.CAN_SendData(CAN_SendHandle, &hcan2, CAN_ID_STD, CAN_ID_GIMBAL, data);

    //�������??
    memcpy(data, &Big_Yaw_Angle, sizeof(float));

    //��������
    Can_Fun.CAN_SendData(CAN_SendHandle, &hcan2, CAN_ID_STD, CAN_ID_6006_BIG_YAW, data);
}


/**
  * @brief ����CAN���ݣ�ͬʱ�����ֱ�Ӹ�ֵ������??
  * @param RxMessage ���յ�������
  * @retval None
  */
void Board2_getChassisInfo(Can_Export_Data_t RxMessage)
{
    vx = (int16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1]);
    vy = (int16_t)(RxMessage.CANx_Export_RxMessage[2] << 8 | RxMessage.CANx_Export_RxMessage[3]);
    vw = (int16_t)(RxMessage.CANx_Export_RxMessage[4] << 8 | RxMessage.CANx_Export_RxMessage[5]);
    big_yaw_velocity = (int16_t)(RxMessage.CANx_Export_RxMessage[6] << 8 | RxMessage.CANx_Export_RxMessage[7]);//�??YAW�??标�?�度

    Steer_Omni_Data.Speed_ToCloud.vx = vx; //前进速度
    Steer_Omni_Data.Speed_ToCloud.vy = vy; //平移速度
    Steer_Omni_Data.Speed_ToCloud.wz = vw / 100; //旋转

    // 仅在遥控模式 + �??YAW模式下使用大YAW�??标�?�度数据
    // if(!ControlMes.AutoAimFlag && ControlMes.yaw_choose == BIG_YAW_MODE)
    // {
    //     Cloud.Target_Yaw += big_yaw_velocity * 0.02f; // 手动控制
    // }
}


float debug_temp_little_yaw = 0.0f;
float speed_nonlinear_function(float little_yaw_bias) {
    float little_yaw_bias_degree = little_yaw_bias * 360 / 8192;
    float little_yaw_bias_degree_sign = little_yaw_bias_degree > 0 ? 1.0 : -1.0;
    float little_yaw_bias_degree_abs = fabs(little_yaw_bias_degree);
    if (little_yaw_bias_degree_abs < 10) {
        return 0.0;
    }
    if (little_yaw_bias_degree_abs < 20) {
        return ((little_yaw_bias_degree_abs - 10) * 1 * little_yaw_bias_degree_sign) * 8192 / 360;
    }
    if (little_yaw_bias_degree_abs < 30) {
        return (((little_yaw_bias_degree_abs - 20) * 4 + 10) * little_yaw_bias_degree_sign) * 8192 / 360;
    }
    return (50 * little_yaw_bias_degree_sign) * 8192 / 360;
}
void Board2_getGimbalInfo(Can_Export_Data_t RxMessage)
{
    // static float AutoAim_Offset = 0;
    
    float little_yaw_position = (uint16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1]);//СYAW��ǰ�Ƕ�
    debug_temp_little_yaw = little_yaw_position;

    ControlMes.fric_Flag   = (uint8_t)(RxMessage.CANx_Export_RxMessage[3] >> 0) & 0x01;  // ?0:?????
    // ControlMes.AutoAimFlag = (uint8_t)(RxMessage.CANx_Export_RxMessage[3] >> 1) & 0x01;  // ?1:????
    // ControlMes.change_Flag = (uint8_t)(RxMessage.CANx_Export_RxMessage[3] >> 2) & 0x01;  // ?2:???????
    // ControlMes.modelFlag   = (uint8_t)(RxMessage.CANx_Export_RxMessage[3] >> 3) & 0x01;  // ?3:??????
    // ControlMes.yaw_choose  = (uint8_t)(RxMessage.CANx_Export_RxMessage[3] >> 5) & 0x03;  // ?5-6:??YAW??
    // uint8_t cloud_mode = (uint8_t)(RxMessage.CANx_Export_RxMessage[3] >> 6) & 0x01;
    
    int16_t big_yaw_enemy_x = (int16_t)(RxMessage.CANx_Export_RxMessage[4] << 8 | RxMessage.CANx_Export_RxMessage[5]);
    int16_t big_yaw_enemy_y = (int16_t)(RxMessage.CANx_Export_RxMessage[6] << 8 | RxMessage.CANx_Export_RxMessage[7]);
    
    big_yaw_ememy_position[0] = big_yaw_enemy_x;
    big_yaw_ememy_position[1] = big_yaw_enemy_y;

    float little_yaw_bias = -(little_yaw_position - 1350);
	while (little_yaw_bias > 4096)
	{
		little_yaw_bias -= 8192;
	}
	while (little_yaw_bias < -4096)
	{
		little_yaw_bias += 8192;
	}

    // Cloud.Target_Yaw += speed_nonlinear_function(little_yaw_bias) * 0.02;
    
    // if(ControlMes.AutoAimFlag == 1)
    // {
    //     if(little_yaw_position == 0.0f)
    //         little_yaw_position = Cloud.Target_Yaw;
            
    //     AutoAim_Offset += -1 * big_yaw_velocity * 0.05f;
    //     Cloud.Target_Yaw = little_yaw_position + AutoAim_Offset;
    // }
    // else
    // {
    //     AutoAim_Offset = 0;
    // }
}
