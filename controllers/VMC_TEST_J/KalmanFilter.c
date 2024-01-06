/**
 ******************************************************************************
 * @file    kalman filter.c
 * @author  Yue yuanhao 
 * @version V0.0.1
 * @date    2023/12/29
 * @brief   KalmanFilter test
 ******************************************************************************
 * @date    2023/12/29  一阶KalmanFilter
 *          2023/12/30  二阶KalmanFilter
 * 
 * 
 * 
**/

#include <KalmanFilter.h>
#include <user_lib.h>
#include <stdio.h>

/***************************************************************************************************
* @name   Kalman_fir_init
* @brief  创建一个一阶卡尔曼滤波器
* @param  kalman : 滤波器
* @param  T_Q : 系统噪声协方差
* @param  T_R : 测量噪声协方差  
* @retval none
* @attention 
*   R固定，Q越大，代表越信任侧量值，Q无穷代表只用测量值
*   反之，Q越小代表越信任模型预测值，Q为零则是只用模型预测
****************************************************************************************************
*/

void Kalman_fir_init(Kalman_fir_t *kalman, float T_Q,float T_R)
{
    kalman->X_last = 0;
    kalman->P_last = 0;

    kalman->Q = T_Q;
    kalman->R = T_R;

    kalman->A = 1;
    kalman->B = 0;
    kalman->H = 1;
    kalman->X_prior = kalman->X_last;
}


/**
* @name   KalmanFilter_fir
* @brief  卡尔曼滤波器
* @param  kalman : 滤波器
* @param  z_data : 待滤波数据
* @retval 滤波后的数据
* @attention
*  A=1 B=0 H=1 I=1
*  z_data是系统输入，即带测量值
*  X_posterior 为后验估计，即最终输出
*  以下为卡尔曼滤波的五大核心公式     
*/

float KalmanFilter_fir(Kalman_fir_t *kalman,float z_data)
{
    //先验状态估计
    //x_hat[k]_prior = A * x_hat[k-1] + B * u[k-1]
    kalman->X_prior = kalman->A * kalman->X_last + kalman->B * kalman->U_last;
    
    //先验状态估计误差协方差矩阵
    //P[k]_prior = A * P[k-1] * A' +Q
    kalman->P_prior = kalman->A * kalman->P_last + kalman->Q;

    //计算卡尔曼增益
    //K[k] = ( P[k]_prior * Hm' ) / ( Hm * P[k]_prior * Hm' + R )
    kalman->Kalman_K = (kalman->P_prior * kalman->H) / (kalman->H * kalman->P_prior + kalman->R);

    //计算后验状态估计
    //x_hat[k] = x_hat[k]_prior + K[k] * ( z[k] - Hm * x_hat[k]_prior)
    kalman->X_posterior = kalman->X_prior + kalman->Kalman_K*(z_data - kalman->H * kalman->X_prior);

    //更新后验状态估计误差协方差矩阵
    //P[k] = ( I - K[k] * Hm )* P[k]_prior
    kalman->P_last = (1 - kalman->Kalman_K) * kalman->P_prior;

    return kalman->X_posterior;
} 



/****************************************************************************************************
* @name   Kalman_sec_init
* @brief  创建一个二阶阶卡尔曼滤波器
* @param  kalman : 滤波器
* @param  Qc_1: 对角矩阵1
* @param  Qc_2: 对角矩阵2  
* @retval none
* @attention 
*   R固定，Q越大，代表越信任侧量值，Q无穷代表只用测量值
*   反之，Q越小代表越信任模型预测值，Q为零则是只用模型预测
*****************************************************************************************************
*/
void Kalman_sec_init(Kalman_sec_t *kalman, float Qc_1,float Qc_2)
{
    kalman->time_const = 0.032;
    //初始状态
    kalman->X.x = 0;
    kalman->X.v = 0;
    kalman->a = 0;

    //状态误差矩阵初始化，这个后面会迭代
    kalman->P.P[0][0] = 1;kalman->P.P[0][1] = 0;
    kalman->P.P[1][0] = 0;kalman->P.P[1][1] = 1;
   
 
    //状态转移矩阵初始化
    kalman->A[0][0] =1;kalman->A[0][1] =kalman->time_const;//time_const
    kalman->A[1][0] =0;kalman->A[1][1] =1;

    //输入矩阵初始化
    kalman->B[0] = 0.5 * (kalman->time_const) * (kalman->time_const);
    kalman->B[1] = kalman->time_const;

    //过程噪声初始化，对角初始化
    kalman->Q[0][0] = Qc_1;kalman->Q[0][1] = 0;
    kalman->Q[1][0] = 0   ;kalman->Q[1][1] = Qc_2;

    //测量噪声初始化，将R作为常数参考值
    kalman->R = 0.1;
}

/**
* @name   KalmanFilter_sen
* @brief  二阶卡尔曼滤波器
* @param  kalman   : 滤波器
* @param  distance : 待滤波数据
* @param  accel    : 系统输入/融合传感器数据（加速度）
* @retval 滤波/融合后的数据
* @attention
*  
*  以下为卡尔曼滤波的五大核心公式
*/

void KalmanFilter_sec(Kalman_sec_t *kalman,float distance,float accel)
{
    //先验估计、状态预测
    kalman->X_prior.x =
        kalman->A[0][0] * kalman->X.x + kalman->A[0][1] * kalman->X.v + kalman->B[0] * accel;
    kalman->X_prior.v =
        kalman->A[1][0] * kalman->X.x + kalman->A[1][1] * kalman->X.v + kalman->B[1] * accel;

    //先验状态误差估计
    kalman->P_prior.P[0][0] = kalman->A[0][0] * (kalman->A[0][0] * kalman->P.P[0][0] + kalman->A[0][1] * kalman->P.P[1][0]) + kalman->Q[0][0];
    kalman->P_prior.P[0][1] = kalman->A[0][1] * (kalman->A[0][0] * kalman->P.P[0][0] + kalman->A[0][1] * kalman->P.P[1][0]) + kalman->Q[0][1];
    kalman->P_prior.P[1][0] = kalman->A[1][0] * (kalman->A[1][0] * kalman->P.P[0][0] + kalman->A[1][1] * kalman->P.P[1][0]) + kalman->Q[1][0];
    kalman->P_prior.P[1][1] = kalman->A[1][1] * (kalman->A[1][0] * kalman->P.P[0][0] + kalman->A[1][1] * kalman->P.P[1][0]) + kalman->Q[1][1];
    
    //更新kalman_gain
    double S = kalman->P_prior.P[0][0] + kalman->R;
    kalman->K[0] = kalman->P_prior.P[0][0] / S;
    kalman->K[1] = kalman->P_prior.P[1][0] / S;

    //更新后验估计
    kalman->X.x = kalman->X_prior.x + kalman->K[0] * (distance - kalman->X_prior.x);
    kalman->X.v = kalman->X_prior.v + kalman->K[1] * (distance - kalman->X_prior.x);

    //更新后验估计误差协方差矩阵
    kalman->P.P[0][0] = (1 - kalman->K[0]) * kalman->P_prior.P[0][0];
    kalman->P.P[0][1] = (1 - kalman->K[0]) * kalman->P_prior.P[0][1];
    kalman->P.P[1][0] = (1 - kalman->K[1]) * kalman->P_prior.P[1][0];
    kalman->P.P[1][1] = (1 - kalman->K[1]) * kalman->P_prior.P[1][1];

    //输出向量
    kalman->out.x = kalman->X.x;
    kalman->out.v = kalman->X.v;

    //log
    printf("K:%f \n", kalman->P.P[0][0]);
}

/*
#include <stdio.h>

typedef struct {
    double x;  // 位置
    double v;  // 速度
} StateVector;

typedef struct {
    double p11; // 位置方差
    double p12; // 位置-速度协方差
    double p21; // 速度-位置协方差
    double p22; // 速度方差
} CovarianceMatrix;

double A[2][2] = {{1, 1}, {0, 1}};
double B[2] = {0.5, 1};  // 输入矩阵
double Q[2][2] = {{0.01, 0}, {0, 0.01}};
double R = 0.1;

StateVector x = {0, 0};
CovarianceMatrix P = {1, 0, 0, 1};

void kalmanFilter(double acceleration, double distance) {
    // 预测步骤
    StateVector x_prior;
    CovarianceMatrix P_prior;

    // 计算状态预测
    x_prior.x = A[0][0] * x.x + A[0][1] * x.v + B[0] * acceleration;
    x_prior.v = A[1][0] * x.x + A[1][1] * x.v + B[1] * acceleration;

    // 计算协方差预测
    P_prior.p11 = A[0][0] * (A[0][0] * P.p11 + A[0][1] * P.p21) + Q[0][0];
    P_prior.p12 = A[0][1] * (A[0][0] * P.p11 + A[0][1] * P.p21) + Q[0][1];
    P_prior.p21 = A[1][0] * (A[1][0] * P.p11 + A[1][1] * P.p21) + Q[1][0];
    P_prior.p22 = A[1][1] * (A[1][0] * P.p11 + A[1][1] * P.p21) + Q[1][1];

    // 更新步骤
    double S = P_prior.p11 + R;
    double K1 = P_prior.p11 / S;
    double K2 = P_prior.p21 / S;

    // 更新状态估计
    x.x = x_prior.x + K1 * (distance - x_prior.x);
    x.v = x_prior.v + K2 * (distance - x_prior.x);

    // 更新协方差矩阵
    P.p11 = (1 - K1) * P_prior.p11;
    P.p12 = (1 - K1) * P_prior.p12;
    P.p21 = (1 - K2) * P_prior.p21;
    P.p22 = (1 - K2) * P_prior.p22;
}

int main() {
    double acceleration = 0.1;
    double distance = 1.0;

    kalmanFilter(acceleration, distance);

    printf("Estimated State: x = %lf, v = %lf\n", x.x, x.v);

    return 0;
}

*/