/*
 * pid.c
 *
 *  Created on: 2023��3��18��
 *      Author: wzl
 */


#include "pid.h"
#include "stddef.h"


// �޷�����
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }
#define ABS(x)      ((x>0)? x: -x)

//��������ٶȻ��ṹ��
PidTypeDef car_speed_pid;
//��������������ṹ��
PidTypeDef car_current_pid;

uint8 PD_USE=0;//���ƻ��ַ����Ƿ����� 1 ����
//-------------------------------------------------------------------------------------------------------------------
// �������       pid���Ʋ�����ʼ��
// ����˵��       PidTypeDef* pid -> �������ṹ��
// ����˵��       uint8_t mode -> ����ģʽ
// ����˵��       const float PID[3] -> pid����
// ����˵��       float max_out -> ������ֵ�޷�
// ����˵��       float max_iout -> ������������ֵ�޷�
// ���ز���       void
//-------------------------------------------------------------------------------------------------------------------
void PID_Init(PidTypeDef *pid, uint8_t mode, const float PID[3], float max_out, float max_iout)
{
    if(pid == NULL || PID == NULL)
        return;

    pid->mode_t = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// �������       pid���Ʋ����ı䣬�����ǲ������ԭ�е�΢�������ı�ԭ�������
// ����˵��       PidTypeDef* pid -> �������ṹ��
// ����˵��       uint8_t mode -> ����ģʽ
// ����˵��       const float PID[3] -> pid����
// ����˵��       float max_out -> ������ֵ�޷�
// ����˵��       float max_iout -> ������������ֵ�޷�
// ���ز���       void
//-------------------------------------------------------------------------------------------------------------------
void PID_Change(PidTypeDef *pid, uint8_t mode, const float PID[3], float max_out, float max_iout)
{
    if(pid == NULL || PID == NULL)
        return;

    pid->mode_t = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
}

void PID_separationThreshold(PidTypeDef *pid, float separationThreshold)
{
    if(pid == NULL )
        return;

    pid->separationThreshold = separationThreshold;
}

//-------------------------------------------------------------------------------------------------------------------
// �������       pid������
// ����˵��       PidTypeDef* pid -> �������ṹ��
// ����˵��       float ref -> �����巵��״ֵ̬
// ����˵��       float set -> �������趨Ŀ��ֵ
// ���ز���       float -> ���������
//-------------------------------------------------------------------------------------------------------------------
float PID_Calc(PidTypeDef *pid, float ref, float set)
{
    if(pid == NULL)
        return 0.0f;

    // ���²���
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;

    // ����ģʽѡ��
    if(pid->mode_t == PID_POSITION)
    {
//        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
//        pid->Iout = pid->Ki * pid->error[0];
//        LimitMax(pid->Iout, pid->max_iout);
//        pid->Dbuf[2] = pid->Dbuf[1];
//        pid->Dbuf[1] = pid->Dbuf[0];
//        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
//        pid->Dout = pid->Kd * pid->Dbuf[0];
//        if(ABS(pid->error[0])<pid->separationThreshold || PD_USE == 0)
//        {
//            pid->out += pid->Pout + pid->Iout + pid->Dout;
//        }
//        else if(PD_USE == 1)//���ڻ��ַ�����ֵ��������
//        {
//            pid->out += pid->Pout + pid->Dout;
//        }
        LimitMax(pid->out, pid->max_out);
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        if(ABS(pid->error[0])<pid->separationThreshold || PD_USE == 0)
        {
            pid->out = pid->Pout + pid->Iout + pid->Dout;
        }
        else if(PD_USE == 1)//���ڻ��ַ�����ֵ��������
        {
            pid->out = pid->Pout + pid->Dout;
        }
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode_t == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        if(ABS(pid->error[0])<pid->separationThreshold || PD_USE == 0)
        {
            pid->out += pid->Pout + pid->Iout + pid->Dout;
        }
        else if(PD_USE == 1)//���ڻ��ַ�����ֵ��������
        {
            pid->out += pid->Pout + pid->Dout;
        }
        LimitMax(pid->out, pid->max_out);
        //printf("DELAT MODE");
//        printf("pid->Iout = %d",pid->Iout);
    }

    // ���ؿ��������
    return pid->out;
}


//-------------------------------------------------------------------------------------------------------------------
// �������       pid�������������
// ����˵��       PidTypeDef* pid -> �������ṹ��
// ���ز���       void
//-------------------------------------------------------------------------------------------------------------------
void PID_clear(PidTypeDef *pid)
{
    if (pid == NULL)
        return;

    // �������
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}
