/**
 * @file PID.c
 * @author centre
 * @brief PID�����㷨ʵ��ģ��
 * @version 0.2
 * @date 2025-08-21
 * @copyright Copyright (c) 2021
 */

#include "PID.h"

One_Kalman_t Cloud_YAWODKalman; ///< YAW�Ῠ�����˲���ʵ��
One_Kalman_t Cloud_PITCHODKalman; ///< PITCH�Ῠ�����˲���ʵ��

/**
 * @brief ����ֵ���ƺ���
 * @param a ָ��Ҫ���Ƶĸ�������ָ��
 * @param ABS_MAX ����ֵ���������
 * @note �˺�����ֱ���޸Ĵ����ָ��ֵ
 */
static void abs_limit(float *a, float ABS_MAX)
{
    if (*a > ABS_MAX)
        *a = ABS_MAX;
    if (*a < -ABS_MAX)
        *a = -ABS_MAX;
}

/**
 * @brief Yaw��ģ��λ��ʽPID������
 * @param pid_t PID�������ṹ��ָ��
 * @param fuzzy_t ģ��PID���ݽṹ��ָ��
 * @param target Ŀ��ֵ
 * @param measured ����ֵ
 * @return PID�������ֵ
 * @note �˺������ģ�������Ż�PID������������Yaw�����
 */
float Position_PID_Yaw(positionpid_t *pid_t, FUZZYPID_Data_t *fuzzy_t, float target, float measured)
{
    // ģ���������PID����
    FuzzyComputation(fuzzy_t, pid_t->err, pid_t->err_last);
    
    // ����״̬����
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;
    pid_t->error_target = pid_t->Target - pid_t->last_set_point;

    // ������������ʹ��ģ��������Ĳ�����
    pid_t->p_out = (pid_t->Kp + fuzzy_t->deta_kp) * pid_t->err;
    pid_t->i_out += (pid_t->Ki + fuzzy_t->date_ki) * pid_t->err;
    pid_t->d_out = (pid_t->Kd + fuzzy_t->date_kd) * (pid_t->Measured - pid_t->err_last);
    pid_t->f_out = pid_t->Kf * pid_t->error_target;
    
    // ���ַ������
    if (pid_t->err >= pid_t->Integral_Separation) {
        pid_t->i_out = 0;
    } else {
        // �����޷�
        abs_limit(&pid_t->i_out, pid_t->IntegralLimit);
    }

    // ���������
    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out + pid_t->f_out);

    // ����޷�
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    // ������ʷ״̬
    pid_t->err_last = pid_t->Measured;
    pid_t->last_set_point = pid_t->Target;
    
    return pid_t->pwm;
}

/**
 * @brief ����ʽPID������
 * @param pid_t PID�������ṹ��ָ��
 * @param target Ŀ��ֵ
 * @param measured ����ֵ
 * @return PID�������ֵ
 * @note ��������Ҫƽ�����Ƶĳ���
 */
float Incremental_PID(incrementalpid_t *pid_t, float target, float measured)
{
    // ����״̬����
    pid_t->Target = target;
    pid_t->Measured = measured;
    pid_t->err = pid_t->Target - pid_t->Measured;

    // ����������
    pid_t->p_out = pid_t->Kp * (pid_t->err - pid_t->err_last);
    pid_t->i_out = pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - 2.0f * pid_t->err_last + pid_t->err_beforeLast);

    // �����޷�
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit);

    // ���������
    pid_t->pwm += (pid_t->p_out + pid_t->i_out + pid_t->d_out);

    // ����޷�
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    // ������ʷ״̬
    pid_t->err_beforeLast = pid_t->err_last;
    pid_t->err_last = pid_t->err;

    return pid_t->pwm;
}

/**
 * @brief ��ʼ������ʽPID������
 * @param pid_t PID�������ṹ��ָ��
 * @param Kp ����ϵ��
 * @param Ki ����ϵ��
 * @param Kd ΢��ϵ��
 * @param MaxOutput ����������
 * @param IntegralLimit �������޷�ֵ
 */
void Incremental_PIDInit(incrementalpid_t *pid_t, float Kp, float Ki, float Kd, uint32_t MaxOutput, uint32_t IntegralLimit)
{
    // ����PID����
    pid_t->Kp = Kp;
    pid_t->Ki = Ki;
    pid_t->Kd = Kd;
    
    // �������Ʋ���
    pid_t->MaxOutput = MaxOutput;
    pid_t->IntegralLimit = IntegralLimit;
    
    // ��ʼ��״̬����
    pid_t->p_out = 0;
    pid_t->d_out = 0;
    pid_t->i_out = 0;
    pid_t->err = 0;
    pid_t->err_last = 0;
    pid_t->err_beforeLast = 0;
    pid_t->pwm = 0;
    pid_t->Measured = 0;
    pid_t->Target = 0;
}

/**
 * @brief �������ʽPID����������
 * @param pid_t PID�������ṹ��ָ��
 * @note ��������״̬������������������
 */
void Clear_IncrementalPIDData(incrementalpid_t *pid_t)
{
    pid_t->Target = 0;
    pid_t->Measured = 0;
    pid_t->err = 0;
    pid_t->err_last = 0;
    pid_t->err_beforeLast = 0;
    pid_t->p_out = 0;
    pid_t->i_out = 0;
    pid_t->d_out = 0;
    pid_t->pwm = 0;
}

/**
 * @brief λ��ʽPID������
 * @param pid_t PID�������ṹ��ָ��
 * @param target Ŀ��ֵ
 * @param measured ����ֵ
 * @return PID�������ֵ
 * @note ��׼λ��ʽPIDʵ�֣���ǰ������
 */
float Position_PID(positionpid_t *pid_t, float target, float measured)
{
    // ����״̬����
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->Measured - pid_t->err_last;
    pid_t->error_target = pid_t->Target - pid_t->last_set_point;
    
    // ����������
    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    pid_t->f_out = pid_t->Kf * pid_t->error_target;
    
    // �����޷�
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit);

    // ���������
    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out + pid_t->f_out);

    // ����޷�
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    // ������ʷ״̬
    pid_t->err_last = pid_t->err;
    pid_t->last_set_point = pid_t->Target;
    
    return pid_t->pwm;
}

/**
 * @brief ��ʼ��λ��ʽPID������
 * @param pid_t PID�������ṹ��ָ��
 * @param Kp ����ϵ��
 * @param Ki ����ϵ��
 * @param Kd ΢��ϵ��
 * @param Kf ǰ��ϵ��
 * @param MaxOutput ����������
 * @param Integral_Separation ���ַ�����ֵ
 * @param IntegralLimit �������޷�ֵ
 */
void Position_PIDInit(positionpid_t *pid_t, float Kp, float Ki, float Kd, float Kf, float MaxOutput, float Integral_Separation, float IntegralLimit)
{
    // ����PID����
    pid_t->Kp = Kp;
    pid_t->Ki = Ki;
    pid_t->Kd = Kd;
    pid_t->Kf = Kf;
    
    // �������Ʋ���
    pid_t->MaxOutput = MaxOutput;
    pid_t->Integral_Separation = Integral_Separation;
    pid_t->IntegralLimit = IntegralLimit;
    
    // ��ʼ��״̬����
    pid_t->p_out = 0;
    pid_t->d_out = 0;
    pid_t->i_out = 0;
    pid_t->err = 0;
    pid_t->err_last = 0;
    pid_t->err_change = 0;
    pid_t->error_target = 0;
    pid_t->pwm = 0;
    pid_t->Measured = 0;
    pid_t->Target = 0;
}

/**
 * @brief ���λ��ʽPID����������
 * @param pid_t PID�������ṹ��ָ��
 * @note ��������״̬������������������
 */
void Clear_PositionPIDData(positionpid_t *pid_t)
{
    pid_t->Target = 0;
    pid_t->Measured = 0;
    pid_t->err = 0;
    pid_t->err_change = 0;
    pid_t->err_last = 0;
    pid_t->p_out = 0;
    pid_t->i_out = 0;
    pid_t->d_out = 0;
    pid_t->pwm = 0;
}

/**
 * @brief �Ƕ�λ��ʽPID������
 * @param pid_t PID�������ṹ��ָ��
 * @param target Ŀ��Ƕ�ֵ
 * @param measured �����Ƕ�ֵ
 * @param ecd_max ���ص�����������ֵ������8191��Ӧ360�ȣ�
 * @return PID�������ֵ
 * @note ר�Ŵ�������Ƕ�ֵ��PID���������Զ������ǶȻ������⣬����ֵ��Ҫ��0��ʼ
 */
float Angle_PID(positionpid_t *pid_t, float target, float measured,float ecd_max)
{
    // ����״̬����
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    
    // �����ǶȻ������⣨8192��Ӧ360�ȣ�
    if(fabs(pid_t->err) > ecd_max/2) {
        if(pid_t->err > 0) {
            pid_t->err = pid_t->err - (ecd_max+1);
        } else {
            pid_t->err = pid_t->err + (ecd_max+1);
        }
    }
    
    pid_t->err_change = pid_t->Measured - pid_t->err_last;
    pid_t->error_target = pid_t->Target - pid_t->last_set_point;
    
    // ����������
    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    pid_t->f_out = pid_t->Kf * pid_t->error_target;
    
    // �����޷�
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit);

    // ���������
    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out + pid_t->f_out);

    // ����޷�
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    // ������ʷ״̬
    pid_t->err_last = pid_t->err;
    pid_t->last_set_point = pid_t->Target;
    
    return pid_t->pwm;
}

/**
 * @brief �ض�λ�ý�kp kd��PID������
 * @param pid_t PID�������ṹ��ָ��
 * @param speed_target Ŀ���ٶ�ֵ
 * @param speed_measured �����ٶ�ֵ
 * @param angle_measured �ǶȲ���ֵ
 * @return PID�������ֵ
 * @note ר�Ŵ���6020����ض��Ƕ�ֵ�������ٶȻ�pid����
 */
float speed_angle_limit_pid(positionpid_t *pid_t, float speed_target, float speed_measured,float angle_measured)
{
// ����״̬����
    pid_t->Target = (float)speed_target;
    pid_t->Measured = (float)speed_measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->Measured - pid_t->err_last;
    pid_t->error_target = pid_t->Target - pid_t->last_set_point;
    
    // ����������
    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    pid_t->f_out = pid_t->Kf * pid_t->error_target;
//    if((angle_measured>5700&&angle_measured<8000)||(angle_measured>1000&&angle_measured<3300))
//    {
//        pid_t->p_out = 142 * pid_t->err;
//        pid_t->d_out = 20 * (pid_t->err - pid_t->err_last);
//    }
    // �����޷�
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit);

    // ���������
    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out + pid_t->f_out);

    // ����޷�
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    // ������ʷ״̬
    pid_t->err_last = pid_t->err;
    pid_t->last_set_point = pid_t->Target;
    
    return pid_t->pwm;

}
