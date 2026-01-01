/**
 * @file Steer_Omni_Chassis.c
 * @author xhf
 * @brief
 * @version 0.1
 * @date 2025-08-15
 * @copyright
 */

#include "Steer_Omni_Chassis.h"

/********��������********/
Steer_Omni_Data_t Steer_Omni_Data;
int8_t dirt[2] = {-1,1};
positionpid_t chassis_follow;
positionpid_t cloud_follow;
int8_t follow=1;
int32_t flag_angletocloud=0;
int32_t flag_Angle_ChassisToCloud=0;
int32_t flag_angle_hd=0;
extern float spinning_vz;
/**
 * @brief �Ƕȷ�Χ������
 * @param ������ĽǶ�ֵ
 * @param ���ֵ
 * @retval ���ƺ�ĽǶ�ֵ
 */
fp64 Angle_Limit(fp64 angle,fp64 max)
{
    if(angle >max)
    {
        angle -= max; 
    }
    else if (angle < -max)
    {
        angle += max;
    }
    return angle;
} 

/**
 * @brief ���̵����ʼ��
 * @param �ṹ���ַ
 * @retval None
 */
void Chassis_Init(void)
{
    M3508_Init(&M3508_Array[0],0x201);
	  M3508_Init(&M3508_Array[1],0x202);
    M3508_Init(&M3508_Array[2],0x203);
	  M3508_Init(&M3508_Array[3],0x204);
    M6020_Init(&M6020s_Chassis1,0x206);
    M6020_Init(&M6020s_Chassis2,0x207);

    //��������ٶȻ���ʼ��
    Position_PIDInit(&(M3508_Array[0].v_pid_object),10.0f, 0.22f, 0,0,8000,30000,6000);
		Position_PIDInit(&(M3508_Array[1].v_pid_object),10.0f, 0.22f, 0,0,8000,30000,6000);
		Position_PIDInit(&(M3508_Array[2].v_pid_object),10.0f, 0.22f, 0,0,8000,30000,6000);
		Position_PIDInit(&(M3508_Array[3].v_pid_object),10.0f, 0.22f, 0,0,8000,30000,6000);
    //ת�����ٶȻ���ʼ��
  	Position_PIDInit(&(M6020s_Chassis1.v_pid_object),100,0,55,0,7000,30000,6000);
    Position_PIDInit(&(M6020s_Chassis2.v_pid_object),100,0,55,0,7000,30000,6000);
    //ת����λ�û���ʼ��
		Position_PIDInit(&(M6020s_Chassis1.l_pid_object),0.5f, 0.000001f, 0.05, 0, 30000, 10000 ,10000);
		Position_PIDInit(&(M6020s_Chassis2.l_pid_object),0.5f, 0.000001f, 0.05, 0, 30000, 10000 ,10000);
    Position_PIDInit(&(chassis_follow), 0.20f, 0.0000005f, 0.04f, 0, 1000, 10000 , 6000);
    M3508_Array[0].targetSpeed = 0.0f;
    M3508_Array[1].targetSpeed = 0.0f;
    M3508_Array[2].targetSpeed = 0.0f;
    M3508_Array[3].targetSpeed = 0.0f;

    M6020s_Chassis1.targetAngle = DIRMOTOR_LB_ANGLE;
    M6020s_Chassis2.targetAngle = DIRMOTOR_RB_ANGLE;

}

/**
 * @brief ����̨����ϵ�µ��ٶ�ת��Ϊ��������ϵ�µ��ٶ�
 * @param angle ��̨����ڵ��̵ĽǶ�
 * @retval 
 */
void v_cloud_convertto_chassis(fp32 angle)
{
    fp32 angle_hd = angle *pi / 180;
	  flag_angle_hd=angle_hd;
    Steer_Omni_Data.Speed_ToChassis.vx =Steer_Omni_Data.Speed_ToCloud.vx * cos(angle_hd) -Steer_Omni_Data.Speed_ToCloud.vy * sin(angle_hd);
    Steer_Omni_Data.Speed_ToChassis.vy =(-1)*Steer_Omni_Data.Speed_ToCloud.vx * sin(angle_hd)-Steer_Omni_Data.Speed_ToCloud.vy * cos(angle_hd);
    Steer_Omni_Data.Speed_ToChassis.wz =Steer_Omni_Data.Speed_ToCloud.wz*0.6;
}

/**
 * @brief ���̸���ģʽ
 * @param angle ��̨����ڵ��̵ĽǶ�
 * @param ����Ϊ1
 * @param kp
 * @retval ????????????????
 */
void chassis_follow_mode_chassis(float angle, uint8_t start_flag)
{
    if(start_flag)
    {
			//使底盘转动到正方向对应的编码值在云台对应的位置
        if(fabs(angle)<10)
        {
            return ;
        }
        Steer_Omni_Data.Speed_ToChassis.wz  +=  Angle_PID(&chassis_follow, 0, angle,360);
    }
}
void cloud_follow_mode_chassis(float angle, uint8_t start_flag)
{
    if(start_flag)
    {
			//使云台跟着底盘转动
        if(fabs(angle)<10)
        {
            return ;
        }
        Cloud.Target_Yaw  +=  Angle_PID(&cloud_follow, 0, angle,360);
    }
}


/**
 * @brief 根据上位机传递的速度信息和模式，设置底盘目标速度
 * @param None
 * @retval None
 */
void Set_Chassis_Speed_From_UpperCom(void)
{
    // 根据底盘模式选择不同的速度坐标系
    switch(chassis_mode)
    {
        case CHASSIS_FOLLOW_MODE: // 底盘跟随模式
            // 使用底盘坐标系下的速度
            Steer_Omni_Data.Speed_ToChassis.vx = chassis_coordinate_vx;
            Steer_Omni_Data.Speed_ToChassis.vy = chassis_coordinate_vy;
            Steer_Omni_Data.Speed_ToChassis.wz = chassis_coordinate_vz;
				    cloud_follow_mode_chassis(Steer_Omni_Data.Angle_ChassisToCloud,follow);
            break;
            
        case SPINNING_MODE: // 小陀螺模式
            // 使用大YAW坐标系下的速度，VZ设为0（小陀螺由电控实现）
            Steer_Omni_Data.Speed_ToChassis.vx = big_yaw_coordinate_vx;
            Steer_Omni_Data.Speed_ToChassis.vy = big_yaw_coordinate_vy;
            Steer_Omni_Data.Speed_ToChassis.wz = 0; // VZ传输0，具体小陀螺控制由电控实现
            break;
            
        case BLOOD_RETURN_MODE: // 缺血回城模式
            // 根据子模式选择速度
            if(lack_blood_son_mode == PATH_CLEAR) // 路径通畅子模式
            {
                // 使用大YAW坐标系下的速度，保持小陀螺
                Steer_Omni_Data.Speed_ToChassis.vx = big_yaw_coordinate_vx;
                Steer_Omni_Data.Speed_ToChassis.vy = big_yaw_coordinate_vy;
                Steer_Omni_Data.Speed_ToChassis.wz = 0; // VZ传输0
            }
            else if(lack_blood_son_mode == PATH_BLOCKED) // 路径堵死子模式
            {
                // 使用大YAW坐标系下的速度，保持小陀螺并清除障碍
                Steer_Omni_Data.Speed_ToChassis.vx = big_yaw_coordinate_vx;
                Steer_Omni_Data.Speed_ToChassis.vy = big_yaw_coordinate_vy;
                Steer_Omni_Data.Speed_ToChassis.wz = 0; // VZ传输0
            }
            break;
            
        default:
            // 默认使用底盘跟随模式
            Steer_Omni_Data.Speed_ToChassis.vx = chassis_coordinate_vx;
            Steer_Omni_Data.Speed_ToChassis.vy = chassis_coordinate_vy;
            Steer_Omni_Data.Speed_ToChassis.wz = chassis_coordinate_vz;
            break;
    }
    
    // 更新小陀螺速度用于USB发送
    spinning_vz = Steer_Omni_Data.Speed_ToChassis.wz;
}



void choose_UpperComorDT7(void)
{
	if(choose_UpperComorDT7_flag)
	{
		Set_Chassis_Speed_From_UpperCom();
	}
	else
	{
		v_cloud_convertto_chassis(Steer_Omni_Data.Angle_ChassisToCloud);  //遥控器控制模式下的底盘速度设定
	  //chassis_follow_mode_chassis(Steer_Omni_Data.Angle_ChassisToCloud,follow);
	}
}

/**
 * @brief ת�����Ƕ�����
 * @param None
 * @retval None
 */
void direction_motor_angle_set(void)
{
    fp64 atan_angle[2];
    fp64 error_angle[2];
    fp64 finall_angle[2];

    if(!((Steer_Omni_Data.Speed_ToCloud).vx == 0 && (Steer_Omni_Data.Speed_ToCloud).vy == 0 && (Steer_Omni_Data.Speed_ToChassis).wz == 0))
    {
        atan_angle[0] = atan2(((Steer_Omni_Data.Speed_ToChassis).vy - (Steer_Omni_Data.Speed_ToChassis).wz * Radius * 0.707107f),((Steer_Omni_Data.Speed_ToChassis).vx + (Steer_Omni_Data.Speed_ToChassis).wz * Radius * 0.707107f)) * 180.0f / pi ;
        atan_angle[1] = atan2(((Steer_Omni_Data.Speed_ToChassis).vy - (Steer_Omni_Data.Speed_ToChassis).wz * Radius * 0.707107f),((Steer_Omni_Data.Speed_ToChassis).vx - (Steer_Omni_Data.Speed_ToChassis).wz * Radius * 0.707107f)) * 180.0f / pi ;        
    
			  finall_angle[0] = atan_angle[0]/360*8192 + DIRMOTOR_LB_ANGLE ;
				finall_angle[1] = atan_angle[1]/360*8192 + DIRMOTOR_RB_ANGLE ;
			
				error_angle[0] =finall_angle[0] - M6020s_Chassis1.realAngle;
				error_angle[1] =finall_angle[1] - M6020s_Chassis2.realAngle;
				
				if(error_angle[0]>4096.0f)
				{
						error_angle[0]-=8192.0f;
				}
				else if(error_angle[0]<-4096.0f)
				{
						error_angle[0]+=8192.0f;
				}
				if(error_angle[1]>4096.0f)
				{
						error_angle[1]-=8192.0f;
				}
				else if(error_angle[1]<-4096.0f)
				{
						error_angle[1]+=8192.0f;
				}
				
				if(fabs(error_angle[0])>2048.0f)
				{
						dirt[0]=1;
						finall_angle[0]-=4096.0f;
				}
				else
				{
						dirt[0]=-1;
				}
				if(fabs(error_angle[1])>2048.0f)
				{
						dirt[1]=-1;
						finall_angle[1]-=4096.0f;
				}
				else
				{
						dirt[1]=1;
				}
		}
    else
    {
        atan_angle[0] = 0.0f;
        atan_angle[1] = 0.0f;
				dirt[1]=1;
				dirt[0]=-1;
			  finall_angle[0] = atan_angle[0]/360*8192 + DIRMOTOR_LB_ANGLE ;
	      finall_angle[1] = atan_angle[1]/360*8192 + DIRMOTOR_RB_ANGLE ;
    }


    Steer_Omni_Data.M6020_Setposition[0] = finall_angle[0] ;
    Steer_Omni_Data.M6020_Setposition[1] = finall_angle[1] ;

}

/**
 * @brief ��������ٶ�����
 * @param None
 * @retval None
 */
void move_motor_speed_set(void)
{
    fp32 wheel_rpm_ratio;
    wheel_rpm_ratio = 60.0f  / WHEEL_PERIMETER * M3508_RATIO*1.5 ;

    // ��ǰ���ٶȼ��㣺
    Steer_Omni_Data.M3508_Setspeed[0] =1.5*((-1*Steer_Omni_Data.Speed_ToChassis.vx-Steer_Omni_Data.Speed_ToChassis.wz*Length_wheel_y/4)*cos(lf_omni_angle/180*pi)+
                    (Steer_Omni_Data.Speed_ToChassis.vy+Steer_Omni_Data.Speed_ToChassis.wz*Length_wheel_x/2)*sin(lf_omni_angle/180*pi)) * wheel_rpm_ratio; 
    //������ٶȼ���

    Steer_Omni_Data.M3508_Setspeed[1] = 1.5*dirt[0] * sqrt( pow(((Steer_Omni_Data.Speed_ToChassis).vy - (Steer_Omni_Data.Speed_ToChassis).wz/2 * Radius * 0.6711f),2) + 
                    pow(((Steer_Omni_Data.Speed_ToChassis).vx - (Steer_Omni_Data.Speed_ToChassis).wz/2 * Radius * 0.7415f),2)) * wheel_rpm_ratio ;

    Steer_Omni_Data.M3508_Setspeed[2] = 1.5*dirt[1] * sqrt( pow(((Steer_Omni_Data.Speed_ToChassis).vy - (Steer_Omni_Data.Speed_ToChassis).wz/2 * Radius * 0.6711f),2) + 
                    pow(((Steer_Omni_Data.Speed_ToChassis).vx + (Steer_Omni_Data.Speed_ToChassis).wz/2 * Radius * 0.7415f),2)) * wheel_rpm_ratio ;
    // ��ǰ���ٶȼ��㣺
    Steer_Omni_Data.M3508_Setspeed[3] = 1.5*((-1*Steer_Omni_Data.Speed_ToChassis.vx+Steer_Omni_Data.Speed_ToChassis.wz*Length_wheel_y/4)*cos(rf_omni_angle/180*pi)+
                    (Steer_Omni_Data.Speed_ToChassis.vy+Steer_Omni_Data.Speed_ToChassis.wz*Length_wheel_x/4)*sin(rf_omni_angle/180*pi)) * wheel_rpm_ratio; 

}

/**
 * @brief ����Ŀ��ֵ���㺯��
 * @param None
 * @retval None
 */
void chassis_target_calc(void)
{
	choose_UpperComorDT7();
	direction_motor_angle_set();
  move_motor_speed_set();
	
    //ת����Ŀ��λ������
    M6020s_Chassis1.targetAngle = Steer_Omni_Data.M6020_Setposition[0];
    M6020s_Chassis2.targetAngle = Steer_Omni_Data.M6020_Setposition[1];
	  if( M6020s_Chassis1.targetAngle < 0 )
		{
			M6020s_Chassis1.targetAngle += 8192 ; 
		}
		else if ( M6020s_Chassis1.targetAngle > 8192 )
		{
			M6020s_Chassis1.targetAngle -= 8192 ;
		}
		if( M6020s_Chassis2.targetAngle < 0 )
		{
			M6020s_Chassis2.targetAngle += 8192 ; 
		}
		else if ( M6020s_Chassis2.targetAngle > 8192 )
		{
			M6020s_Chassis2.targetAngle -= 8192 ;
		}
		
    //�������Ŀ���ٶ�����
    M3508_Array[0].targetSpeed = Steer_Omni_Data.M3508_Setspeed[0];
    M3508_Array[1].targetSpeed = Steer_Omni_Data.M3508_Setspeed[1];
    M3508_Array[2].targetSpeed = Steer_Omni_Data.M3508_Setspeed[2];
    M3508_Array[3].targetSpeed = Steer_Omni_Data.M3508_Setspeed[3];

}

/**
 * @brief ��������������
 * @param None
 * @retval None
 */
void Steer_Omni_Chassis_Out(void)
{
    chassis_target_calc();
    M6020_location_change(&M6020s_Chassis1,pid_control_normal,M6020s_Chassis1.targetAngle,M6020s_Chassis1.realAngle);
    M6020_location_change(&M6020s_Chassis2,pid_control_normal,M6020s_Chassis2.targetAngle,M6020s_Chassis2.realAngle);

    M6020_velocity_change(&M6020s_Chassis1,pid_control_normal,&hcan2,M6020s_Chassis1.targetSpeed);
    M6020_velocity_change(&M6020s_Chassis2,pid_control_normal,&hcan2,M6020s_Chassis2.targetSpeed);

    motor_velocity_change(&M3508_Array[0],pid_control_normal,&hcan1,M3508_Array[0].targetSpeed);
    motor_velocity_change(&M3508_Array[1],pid_control_normal,&hcan1,M3508_Array[1].targetSpeed);
    motor_velocity_change(&M3508_Array[2],pid_control_normal,&hcan1,M3508_Array[2].targetSpeed);
    motor_velocity_change(&M3508_Array[3],pid_control_normal,&hcan1,M3508_Array[3].targetSpeed);

    Can_Fun.CAN_SendData(CAN_SendHandle,&hcan1,CAN_ID_STD,0x200,CAN1_0x200_Tx_Data);
    Can_Fun.CAN_SendData(CAN_SendHandle,&hcan2,CAN_ID_STD,0x1ff,CAN2_0x1ff_Tx_Data);
			/***************************½«µçÁ÷²ÎÊý·¢ËÍ¸øµç»ú*******************************/
		uint8_t data[8] = {0};
		DM_setParameter(J6006s_Yaw.outPosition, J6006s_Yaw.outSpeed, J6006s_Yaw.outKp, J6006s_Yaw.outKd, J6006s_Yaw.outTorque,J6006_MaxP,J6006_MaxV,J6006_MaxT, data);
		Can_Fun.CAN_SendData(CAN_SendHandle, &hcan2, CAN_ID_STD, J6006_SENDID_Yaw, data);
}

/**
 * @brief ���ݵ�����̨����ԽǶ�
 * @param None
 * @retval None
 */
void Steer_Omni_GetAngle(fp32 angle)
{
	Steer_Omni_Data.Angle_ChassisToCloud = angle;
	
	flag_Angle_ChassisToCloud  = Steer_Omni_Data.Angle_ChassisToCloud;
	
//	flag_angletocloud=Steer_Omni_Data.Angle_ChassisToCloud;
}
