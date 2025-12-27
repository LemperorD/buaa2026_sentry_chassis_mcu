/**
 * @file N100.c
 * @author lxr(784457420@qq.com)
 * @brief N100 IMU���������ݴ���ģ��
 * @version 1.0
 * @date 2023-11-15
 * @copyright Copyright (c) 2023
 */

#include "N100.h"  // ����N100������ͷ�ļ�

uint8_t N100_Rxbuffer[56];  // ������ջ����������ڴ洢��UART���յ�ԭʼ����
N100_AHRSData_Packet_t N100_Angle;  // ����ṹ����������ڴ洢���������̬����

/*************
 * ���ܣ�ʵ��16��������ת��Ϊ������
 * �������ĸ��ֽڵ�����
 * ����ֵ��ת����ĸ�����
 ****************/
float DATA_Trans(uint8_t Data_1, uint8_t Data_2, uint8_t Data_3, uint8_t Data_4)
{
    uint32_t transition_32;  // ���ڴ洢�ϲ����32λ����
    float tmp = 0;           // �洢ת����ĸ�����
    int sign = 0;            // ����λ��0Ϊ����1Ϊ����
    int exponent = 0;        // ָ������
    float mantissa = 0;      // β������

    // ���ĸ��ֽںϲ�Ϊһ��32λ����
    transition_32 = 0;
    transition_32 |= Data_4 << 24;   // �����Ч�ֽ�
    transition_32 |= Data_3 << 16;   // �θ���Ч�ֽ�
    transition_32 |= Data_2 << 8;     // �ε���Ч�ֽ�
    transition_32 |= Data_1;          // �����Ч�ֽ�

    // ��ȡ����λ�����λ��
    sign = (transition_32 & 0x80000000) ? -1 : 1;
    
    // ��ȡָ�����֣�30-23λ��
    exponent = ((transition_32 >> 23) & 0xff) - 127;
    
    // ��ȡβ�����֣�22-0λ����ת��Ϊ������
    mantissa = 1 + ((float)(transition_32 & 0x7fffff) / 0x7fffff);
    
    // �������ո�����ֵ������ �� β�� �� 2^ָ��
    tmp = sign * mantissa * pow(2, exponent);
    
    return tmp;  // ����ת����ĸ�����
}

/**
 * @brief ���ĸ��ֽںϲ�Ϊ32λʱ���
 * @param Data_1 �ֽ�1�������Ч�ֽڣ�
 * @param Data_2 �ֽ�2
 * @param Data_3 �ֽ�3
 * @param Data_4 �ֽ�4�������Ч�ֽڣ�
 * @return �ϲ����32λʱ���
 */
long long timestamp(uint8_t Data_1, uint8_t Data_2, uint8_t Data_3, uint8_t Data_4)
{
    uint32_t transition_32;  // ���ڴ洢�ϲ����32λ����
    
    // ���ĸ��ֽںϲ�Ϊһ��32λ����
    transition_32 = 0;
    transition_32 |= Data_4 << 24;   // �����Ч�ֽ�
    transition_32 |= Data_3 << 16;   // �θ���Ч�ֽ�
    transition_32 |= Data_2 << 8;     // �ε���Ч�ֽ�
    transition_32 |= Data_1;          // �����Ч�ֽ�
    
    return transition_32;  // ���غϲ����ʱ���
}

/**
 * @brief N100 IMU��ʼ������������DMA����
 * @retval none
 */
void N100_Init(void)
{
    // ʹ��DMA����UART���ݵ�������
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, N100_Rxbuffer, sizeof(N100_Rxbuffer));
}

/**
 * @brief N100 IMU���ݽ�����������ԭʼ���ݽ������ṹ��
 * @retval none
 */
void N100_Read(void)
{
    // ������ݰ����ͺͳ����Ƿ�ƥ��AHRS����
    if (N100_Rxbuffer[1] == TYPE_AHRS && N100_Rxbuffer[2] == AHRS_LEN)
    {
        // �������ٶ����ݣ���ת��������ƫ����
        N100_Angle.RollSpeed = DATA_Trans(N100_Rxbuffer[7], N100_Rxbuffer[8], N100_Rxbuffer[9], N100_Rxbuffer[10]);       // ��ת���ٶ�
        N100_Angle.PitchSpeed = DATA_Trans(N100_Rxbuffer[11], N100_Rxbuffer[12], N100_Rxbuffer[13], N100_Rxbuffer[14]); // �������ٶ�
        N100_Angle.YawSpeed = DATA_Trans(N100_Rxbuffer[15], N100_Rxbuffer[16], N100_Rxbuffer[17], N100_Rxbuffer[18]);   // ƫ�����ٶ�
        
        // ������̬�����ݣ���ת��������ƫ����
        N100_Angle.Roll = DATA_Trans(N100_Rxbuffer[19], N100_Rxbuffer[20], N100_Rxbuffer[21], N100_Rxbuffer[22]);       // ��ת��
        N100_Angle.Pitch = DATA_Trans(N100_Rxbuffer[23], N100_Rxbuffer[24], N100_Rxbuffer[25], N100_Rxbuffer[26]);       // ������
        N100_Angle.Yaw = DATA_Trans(N100_Rxbuffer[27], N100_Rxbuffer[28], N100_Rxbuffer[29], N100_Rxbuffer[30]);         // ƫ����
        
        // ������Ԫ�����ݣ���̬����Ԫ����ʾ��
        N100_Angle.Qw = DATA_Trans(N100_Rxbuffer[31], N100_Rxbuffer[32], N100_Rxbuffer[33], N100_Rxbuffer[34]);          // ��Ԫ��w����
        N100_Angle.Qx = DATA_Trans(N100_Rxbuffer[35], N100_Rxbuffer[36], N100_Rxbuffer[37], N100_Rxbuffer[38]);         // ��Ԫ��x����
        N100_Angle.Qy = DATA_Trans(N100_Rxbuffer[39], N100_Rxbuffer[40], N100_Rxbuffer[41], N100_Rxbuffer[42]);         // ��Ԫ��y����
        N100_Angle.Qz = DATA_Trans(N100_Rxbuffer[43], N100_Rxbuffer[44], N100_Rxbuffer[45], N100_Rxbuffer[46]);         // ��Ԫ��z����
        
        // ����ʱ�������
        N100_Angle.Timestamp = timestamp(N100_Rxbuffer[47], N100_Rxbuffer[48], N100_Rxbuffer[49], N100_Rxbuffer[50]);    // ʱ���
    }
}
