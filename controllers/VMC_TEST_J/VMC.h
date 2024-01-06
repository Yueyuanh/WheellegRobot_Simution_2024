#ifndef VMC_H
#define VMC_H

#include <struct_typedef.h>
#include <pid.h>
#include <user_lib.h>

#define pi 3.1415926
#define rad2angle 57.3

#define LEG_1 0.16f
#define LEG_2 0.32f
#define LEG_0 0.20f




struct VMC_t
{
    //定义实际模型参数
    float L1,L2,L3,L4,L5;
  
    //定义中间变量
    float Phi1,Phi2,Phi3,Phi4;
    float A0,B0,C0,L_BD;
    float xB,xD,yB,yD;
    
    float xC,yC;
    
    //test
    pid_type_def F_PD,Tp_PD;

    //逆运动学变量
    float xA,yA;
    float xC_set,yC_set;
    float a0,b0,c0;
    float Phi1_set,Phi4_set;
    float Phi2_set,Phi3_set;
    //定义输入量
    float F,Tp;

    //观测器结构体
    differ_type_def leg_motor_speed[4];
    differ_type_def d_L0, d_Phi0;

    // 定义输出变量
    float L0,Phi0;
    float L0_dot, Phi0_gyro;

    //定义雅可比矩阵
    double J[2][2];
   
    //定义输出量
    float T[2];

    //逆解算求出足端返回扭矩
    double inv_J[2][2];
    double F_foot;

}VMC_LEG_R,VMC_LEG_L;

struct VMC_t;

void VMC_init();
void VMC_calc(float,float,float,float);
void inverse_leg(float Xc,float Yc);

void Forward_kinematics_R(fp32 Q1, fp32 S1, fp32 Q4, fp32 S4, fp32 A1, fp32 A4);
void Forward_kinematics_L(fp32 Q1, fp32 S1, fp32 Q4, fp32 S4, fp32 A1, fp32 A4);
void inverse_kinematics(fp32 angle, fp32 high); // side 1为L 0为R

fp32 VMC_solve_J1(fp32 a, fp32 h);
fp32 VMC_solve_J2(fp32 a, fp32 h);
fp32 VMC_solve_J3(fp32 a, fp32 h);
fp32 VMC_solve_J4(fp32 a, fp32 h);

fp32 VMC_solve_invJ1(fp32 a, fp32 h);
fp32 VMC_solve_invJ2(fp32 a, fp32 h);
fp32 VMC_solve_invJ3(fp32 a, fp32 h);
fp32 VMC_solve_invJ4(fp32 a, fp32 h);

#endif
