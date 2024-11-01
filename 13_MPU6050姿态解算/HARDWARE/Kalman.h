/************************************************************

���ƣ��������˲��㷨�ײ�����
���ߣ�ChengHaoTong
ʱ�䣺2024/2/2

*************************************************************/

#ifndef KALMAN_H
#define KALMAN_H
#include <math.h>

/*
���������������û����ú���
void Kalman_Init(void);
void Kalman_Calculate(void);

�÷�:
��1������������,��ѭ��֮ǰ��ʼ��Kalman_Init();
(ע��:�����ȵ���MPU6050_Init()�ſ��Գ�ʼ��Kalman.��ΪKalman_Calculate()�ײ������ʹ����MPU6050_GetData();)
��2����while(1)��ѭ���ﲻ�ϵ���Kalman_Calculate()�õ�����������ֵ
��3�������������ֱ��ʹ��pitch roll����(�����Ѿ�����Ϊ�ⲿ����)
*/

void Kalman_Init(void);
void Kalman_Calculate(void);
extern float pitch,roll;

/*
�����ǿ������˲��㷨�ĵײ���������
*/
typedef struct {
    float Q_angle;
    float Q_bias; 
    float R_measure;
    float angle;
    float bias; 
    float rate; 

    float P[2][2];
} KalmanFilter;

extern int16_t AccX, AccY, AccZ, GYROX, GYROY, GYROZ;
extern float gyroXrate,gyroYrate,accPitch,accRoll;

void Kalman_Init(void);
void Kalman_Calculate(void);
void KalmanFilter_Init(KalmanFilter *kf);
float KalmanFilter_Update(KalmanFilter *kf, float newAngle, float newRate, float dt);

#endif 
