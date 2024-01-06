#include <VMC.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <VMC_TEST_J.h>

void VMC_init()
{
    const fp32  F_PD_R[3]={500,30,1000};
    const fp32 Tp_PD_R[3]={-30,0,-20};

    const fp32  F_PD_L[3]={500,30,1000};
    const fp32 Tp_PD_L[3]={-30,0,-20};
 
    VMC_LEG_R.L1 = LEG_1;
    VMC_LEG_R.L2 = LEG_2;
    VMC_LEG_R.L3 = LEG_2;
    VMC_LEG_R.L4 = LEG_1;
    VMC_LEG_R.L5 = LEG_0;

    VMC_LEG_L.L1 = LEG_1;
    VMC_LEG_L.L2 = LEG_2;
    VMC_LEG_L.L3 = LEG_2;
    VMC_LEG_L.L4 = LEG_1;
    VMC_LEG_L.L5 = LEG_0;

    PID_init(&VMC_LEG_R.F_PD,PID_POSITION, F_PD_R, 2000, 1000);
    PID_init(&VMC_LEG_R.Tp_PD,PID_POSITION,Tp_PD_R,2000, 0);
    PID_init(&VMC_LEG_L.F_PD,PID_POSITION, F_PD_L, 2000, 1000);
    PID_init(&VMC_LEG_L.Tp_PD,PID_POSITION,Tp_PD_L,2000,  0);
}






//减速比 1/4
void VMC_calc(float Ld_R, float Ad_R, float Ld_L, float Ad_L)
{
    // //两关节电机VMC位置控制
    // VMC_LEG_R.F = PID_calc(&VMC_LEG_R.F_PD, VMC_LEG_R.L0,Ld_R);
    // VMC_LEG_R.Tp= PID_calc(&VMC_LEG_R.Tp_PD,VMC_LEG_R.Phi0,Ad_R);

    // VMC_LEG_L.F = PID_calc(&VMC_LEG_L.F_PD,VMC_LEG_L.L0,Ld_L);
    // VMC_LEG_L.Tp= PID_calc(&VMC_LEG_L.Tp_PD, VMC_LEG_L.Phi0, Ad_L);

    VMC_LEG_R.F = Ld_R;
    VMC_LEG_R.Tp = Ad_R;
    VMC_LEG_L.F = Ld_L;
    VMC_LEG_L.Tp = Ad_L;

    //雅可比矩阵求解
    VMC_LEG_R.J[0][0] = LEG_1 * sin(VMC_LEG_R.Phi0 - VMC_LEG_R.Phi3) * sin(VMC_LEG_R.Phi1-VMC_LEG_R.Phi2) / sin(VMC_LEG_R.Phi3 - VMC_LEG_R.Phi2);
    VMC_LEG_R.J[0][1] = LEG_1 * cos(VMC_LEG_R.Phi0 - VMC_LEG_R.Phi3) * sin(VMC_LEG_R.Phi1-VMC_LEG_R.Phi2) / (LEG_0*sin(VMC_LEG_R.Phi3 - VMC_LEG_R.Phi2));
    VMC_LEG_R.J[1][0] = LEG_1 * sin(VMC_LEG_R.Phi0 - VMC_LEG_R.Phi2) * sin(VMC_LEG_R.Phi3-VMC_LEG_R.Phi4) / sin(VMC_LEG_R.Phi3 - VMC_LEG_R.Phi2);
    VMC_LEG_R.J[1][1] = LEG_1 * cos(VMC_LEG_R.Phi0 - VMC_LEG_R.Phi2) * sin(VMC_LEG_R.Phi3-VMC_LEG_R.Phi4) / (LEG_0*sin(VMC_LEG_R.Phi3 - VMC_LEG_R.Phi2));

    VMC_LEG_L.J[0][0] = LEG_1 * sin(VMC_LEG_L.Phi0 - VMC_LEG_L.Phi3) * sin(VMC_LEG_L.Phi1-VMC_LEG_L.Phi2) / sin(VMC_LEG_L.Phi3 - VMC_LEG_L.Phi2);
    VMC_LEG_L.J[0][1] = LEG_1 * cos(VMC_LEG_L.Phi0 - VMC_LEG_L.Phi3) * sin(VMC_LEG_L.Phi1-VMC_LEG_L.Phi2) / (LEG_0*sin(VMC_LEG_L.Phi3 - VMC_LEG_L.Phi2));
    VMC_LEG_L.J[1][0] = LEG_1 * sin(VMC_LEG_L.Phi0 - VMC_LEG_L.Phi2) * sin(VMC_LEG_L.Phi3-VMC_LEG_L.Phi4) / sin(VMC_LEG_L.Phi3 - VMC_LEG_L.Phi2);
    VMC_LEG_L.J[1][1] = LEG_1 * cos(VMC_LEG_L.Phi0 - VMC_LEG_L.Phi2) * sin(VMC_LEG_L.Phi3-VMC_LEG_L.Phi4) / (LEG_0*sin(VMC_LEG_L.Phi3 - VMC_LEG_L.Phi2));

    //计算关节电机输出扭矩
    VMC_LEG_R.T[0]=VMC_LEG_R.J[0][0]*VMC_LEG_R.F+VMC_LEG_R.J[0][1]*VMC_LEG_R.Tp;
    VMC_LEG_R.T[1]=VMC_LEG_R.J[1][0]*VMC_LEG_R.F+VMC_LEG_R.J[1][1]*VMC_LEG_R.Tp;

    VMC_LEG_L.T[0]=VMC_LEG_L.J[0][0]*VMC_LEG_L.F+VMC_LEG_L.J[0][1]*VMC_LEG_L.Tp;
    VMC_LEG_L.T[1]=VMC_LEG_L.J[1][0]*VMC_LEG_L.F+VMC_LEG_L.J[1][1]*VMC_LEG_L.Tp;


    //足端压力雅可比矩阵
    VMC_LEG_R.inv_J[0][0] = VMC_solve_invJ1(VMC_LEG_R.Phi0,VMC_LEG_R.L0);
    VMC_LEG_R.inv_J[0][1] = VMC_solve_invJ2(VMC_LEG_R.Phi0,VMC_LEG_R.L0);
    VMC_LEG_R.inv_J[1][0] = VMC_solve_invJ3(VMC_LEG_R.Phi0,VMC_LEG_R.L0);
    VMC_LEG_R.inv_J[1][1] = VMC_solve_invJ4(VMC_LEG_R.Phi0,VMC_LEG_R.L0);

    VMC_LEG_L.inv_J[0][0] = VMC_solve_invJ1(VMC_LEG_L.Phi0,VMC_LEG_L.L0);
    VMC_LEG_L.inv_J[0][1] = VMC_solve_invJ2(VMC_LEG_L.Phi0,VMC_LEG_L.L0);
    VMC_LEG_L.inv_J[1][0] = VMC_solve_invJ3(VMC_LEG_L.Phi0,VMC_LEG_L.L0);
    VMC_LEG_L.inv_J[1][1] = VMC_solve_invJ4(VMC_LEG_L.Phi0,VMC_LEG_L.L0);
    

    //足端压力解算
    VMC_LEG_R.F_foot = VMC_LEG_R.inv_J[0][0] * chassis_data.leg_Torque[0] + VMC_LEG_R.inv_J[0][1] * chassis_data.leg_Torque[1];
    VMC_LEG_L.F_foot = VMC_LEG_L.inv_J[1][0] * chassis_data.leg_Torque[2] + VMC_LEG_R.inv_J[1][1] * chassis_data.leg_Torque[3];


}





// 正运动学解算
//Q是角度，S是速度，A是加速度
void Forward_kinematics_R(fp32 Q1, fp32 Q4, fp32 S1, fp32 S4, fp32 A1, fp32 A4)
{

    fp32 L0 = 0, Q0 = 0;
    fp32 xb, xd, yb, yd, Lbd, xc, yc;
    fp32 A0, B0, C0, Q2, Q3, S2;
    fp32 vxb, vxd, vyb, vyd, vxc, vyc;
    fp32 cos_Q1, cos_Q4, sin_Q1, sin_Q4;
    fp32 S0;
    // fp32 sin_Q2, S3,cos_Q2, sin_Q3, cos_Q3;
    // fp32 axb, ayb, axd, ayd, a2, axc;
    /******************************/
    // Q1 = pi + Q1;  //这里是对真实电机的反馈角度处理，仿真中不需要
    cos_Q1 = cos(Q1);
    sin_Q1 = sin(Q1);
    cos_Q4 = cos(Q4);
    sin_Q4 = sin(Q4);
    xb = -LEG_0 / 2 + LEG_1 * cos_Q1;
    xd =  LEG_0 / 2 + LEG_1 * cos_Q4;
    yb =  LEG_1 * sin_Q1;
    yd =  LEG_1 * sin_Q4;

    // arm_sqrt_f32((xd-xb)*(xd-xb)+(yd-yb)*(yd-yb),&Lbd);
    Lbd = sqrt((xd - xb) * (xd - xb) + (yd - yb) * (yd - yb));
    A0 = 2 * LEG_2 * (xd - xb);
    B0 = 2 * LEG_2 * (yd - yb);
    C0 = LEG_2 * LEG_2 + Lbd * Lbd - LEG_2 * LEG_2;
    Q2 = 2 * atan((B0 + sqrt(A0 * A0 + B0 * B0 - C0 * C0)) / (A0 + C0));

    xc = xb + cos(Q2) * LEG_2;
    yc = yb + sin(Q2) * LEG_2;
    //	arm_sqrt_f32(xc*xc +yc*yc,&L0);
    L0 = sqrt(xc * xc + yc * yc);
    Q0 = atan(yc / xc);
    
    //以下为速度求解
    vxb = -S1 * LEG_1 * sin_Q1;
    vyb =  S1 * LEG_1 * cos_Q1;
    vxd = -S4 * LEG_1 * sin_Q4;
    vyd =  S4 * LEG_1 * cos_Q4;
    Q3 = atan((yc - yd) / (xc - xd));
    S2 = ((vxd - vxb) * cos(Q3) + (vyd - vyb) * sin(Q3)) / (LEG_2 * sin(Q3 - Q2));
    //S3 = ((vxd - vxb) * cos(Q2) + (vyd - vyb) * sin(Q2)) / (LEG_2 * sin(Q3 - Q2));
    vxc = vxb - S2 * LEG_2 * sin(Q2);
    vyc = vyb + S2 * LEG_2 * cos(Q2);
    S0 = 3 * (-sin(abs(Q0)) * vxc - cos(Q0) * vyc);

    if(Q0<0) Q0+=pi;

    VMC_LEG_R.L0 = L0;
    VMC_LEG_R.Phi0 = Q0;
    VMC_LEG_R.Phi0_gyro=S0;

    VMC_LEG_R.Phi2 = Q2;
    VMC_LEG_R.Phi3 = Q3;

}


void Forward_kinematics_L(fp32 Q1, fp32 Q4, fp32 S1, fp32 S4, fp32 A1, fp32 A4)
{

    fp32 L0 = 0, Q0 = 0;
    fp32 xb, xd, yb, yd, Lbd, xc, yc;
    fp32 A0, B0, C0, Q2, Q3, S2;
    fp32 vxb, vxd, vyb, vyd, vxc, vyc;
    fp32 cos_Q1, cos_Q4, sin_Q1, sin_Q4;
    fp32 S0;
    // fp32 dL0,S0, S3
    // fp32 sin_Q2, cos_Q2, sin_Q3, cos_Q3;
    // fp32 axb, ayb, axd, ayd, a2, axc;
    /******************************/
    // Q1 = pi + Q1;  //这里是对真实电机的反馈角度处理，仿真中不需要
    cos_Q1 = cos(Q1);
    sin_Q1 = sin(Q1);
    cos_Q4 = cos(Q4);
    sin_Q4 = sin(Q4);
    xb = -LEG_0 / 2 + LEG_1 * cos_Q1;
    xd =  LEG_0 / 2 + LEG_1 * cos_Q4;
    yb =  LEG_1 * sin_Q1;
    yd =  LEG_1 * sin_Q4;

    // arm_sqrt_f32((xd-xb)*(xd-xb)+(yd-yb)*(yd-yb),&Lbd);
    Lbd = sqrt((xd - xb) * (xd - xb) + (yd - yb) * (yd - yb));
    A0 = 2 * LEG_2 * (xd - xb);
    B0 = 2 * LEG_2 * (yd - yb);
    C0 = LEG_2 * LEG_2 + Lbd * Lbd - LEG_2 * LEG_2;
    Q2 = 2 * atan((B0 + sqrt(A0 * A0 + B0 * B0 - C0 * C0)) / (A0 + C0));

    xc = xb + cos(Q2) * LEG_2;
    yc = yb + sin(Q2) * LEG_2;
    //	arm_sqrt_f32(xc*xc +yc*yc,&L0);
    L0 = sqrt(xc * xc + yc * yc);
    Q0 = atan(yc / xc);
    
    //以下为速度求解
    vxb = -S1 * LEG_1 * sin_Q1;
    vyb =  S1 * LEG_1 * cos_Q1;
    vxd = -S4 * LEG_1 * sin_Q4;
    vyd =  S4 * LEG_1 * cos_Q4;
    Q3 = atan((yc - yd) / (xc - xd));
    S2 = ((vxd - vxb) * cos(Q3) + (vyd - vyb) * sin(Q3)) / (LEG_2 * sin(Q3 - Q2));
    //S3 = ((vxd - vxb) * cos(Q2) + (vyd - vyb) * sin(Q2)) / (LEG_2 * sin(Q3 - Q2));
    vxc = vxb - S2 * LEG_2 * sin(Q2);
    vyc = vyb + S2 * LEG_2 * cos(Q2);
    S0 = 3 * (-sin(abs(Q0)) * vxc - cos(Q0) * vyc);

    if(Q0<0) Q0+=pi;

    VMC_LEG_L.L0       = L0;
    VMC_LEG_L.Phi0     = Q0;
    VMC_LEG_L.Phi0_gyro = S0;

    VMC_LEG_L.Phi2 = Q2;
    VMC_LEG_L.Phi3 = Q3;

}

// 逆运动学解算
void inverse_kinematics(fp32 high, fp32 angle)
{
    fp32 yc, xc;
    fp32 A, B, C, T[2];
    fp32 ya = 0, xa = -LEG_0 / 2;
    fp32 q1, q4;
    fp32 temp;

    for (uint8_t i = 0; i < 2; i++)
    {
        // 两侧a坐标设置，由此看出坐标中心位于中心
        if (i == 0)
        {
            ya = 0, xa = -LEG_0 / 2;
        }
        else
        {
            ya = 0, xa = LEG_0 / 2;
        }

        yc = high;
        // xc = tan(angle-pi/2)*high;                        //xc为oc和机体的角度
        xc = (sin(angle-pi/2)/cos(angle-pi/2))*high;      //同上
        //xc = angle; // 直接设置为末端的横坐标

        // 代入公式计算
        A = (xc - xa) * (xc - xa) + (yc - ya) * (yc - ya) + LEG_1 * LEG_1 - LEG_2 * LEG_2;
        B = -2 * (xc - xa) * LEG_1;
        C = -2 * (yc - ya) * LEG_1;
        temp = sqrt(C * C + B * B - A * A);
        // arm_sqrt_f32(C*C+B*B-A*A,&temp);

        if (i == 0)
        {
            T[0] = (-C + temp) / (A - B);
            q1 = 2 * atan(T[0]);
        }

        else
        {
            T[1] = (-C - temp) / (A - B);
            q4 = 2 * atan(T[1]);
        }
    }

    if (q1 < -pi / 2)
        q1 = q1 + 2 * pi; // 防止角度过大导致奇异点

    VMC_LEG_R.Phi1_set = q1 - pi / 2; // 角度设定值赋值，并根据实际情况调节输出角度
    VMC_LEG_R.Phi4_set = q4 - pi / 2;
    printf("q1_set:%f  q4_set:%f \n", VMC_LEG_R.Phi1_set, VMC_LEG_R.Phi4_set);
}

//雅可比矩阵数值解

fp32 VMC_solve_J1(fp32 a,fp32 h)
{
    fp32 p00 = 0.1811;
    fp32 p10 = -0.806;
    fp32 p01 = -3.318;
    fp32 p20 = -3.349;
    fp32 p11 =  5.656;
    fp32 p02 =  14.27;
    fp32 p30 =  18.45;
    fp32 p21 =  25.31;
    fp32 p12 = -15.34;
    fp32 p03 = -29.33;
    fp32 p40 = -3.331;
    fp32 p31 = -69.46;
    fp32 p22 =  -40.4;
    fp32 p13 =  16.87;
    fp32 p04 =  27.79;

    fp32 J1;
    fp32 x = 0;

	x = h*sin(pi/2-a);
	h = h*cos(pi/2-a);
	if(x<-0.2f)x=-0.2f;else if(x>0.2f)x=0.2f;
	if(h<0.12f)h=0.12f;else if(h>0.3)h=0.3;

  J1 = (p00 + p10*x + p01*h + p20*x*x + p11*x*h + p02*h*h + p30*x*x*x + p21*x*x*h 
     + p12*x*h*h + p03*h*h*h + p40*x*x*x*x + p31*x*x*x*h + p22*x*x*h*h 
     + p13*x*h*h*h + p04*h*h*h*h);

	return J1;
}


fp32 VMC_solve_invJ1(fp32 a,fp32 h)
{
    fp32 p00 = -46.51;
    fp32 p10 =  252.6;
    fp32 p01 =  666.4;
    fp32 p20 = -16.46;
    fp32 p11 =  -3169;
    fp32 p02 =  -4117;
    fp32 p30 =  -1346;
    fp32 p21 =  269.9;
    fp32 p12 = 1.358e+04;
    fp32 p03 =  1.17e+04;
    fp32 p40 =  582.5;
    fp32 p31 =   5547;
    fp32 p22 =  -1170;
    fp32 p13 = -1.986e+04;
    fp32 p04 = -1.287e+04;

    fp32 J1;
	fp32 x=0;
	x = h*sin(pi/2-a);
	h = h*cos(pi/2-a);
	if(x<-0.2f)x=-0.2f;else if(x>0.2f)x=0.2f;
	if(h<0.12f)h=0.3;else if(h>0.3)h=0.3;
  J1 = (p00 + p10*x + p01*h + p20*x*x + p11*x*h + p02*h*h + p30*x*x*x + p21*x*x*h 
  + p12*x*h*h + p03*h*h*h + p40*x*x*x*x + p31*x*x*x*h + p22*x*x*h*h 
  + p13*x*h*h*h + p04*h*h*h*h);
	return J1;
}


fp32 VMC_solve_J2(fp32 a,fp32 h)
{
    fp32 p00 = -0.1751f;
    fp32 p10 = -0.8056f;
    fp32 p01 =   3.192f;
    fp32 p20 =   3.381f;
    fp32 p11 =   5.643f;
    fp32 p02 =  -13.29f;
    fp32 p30 =   18.86f;
    fp32 p21 =  -25.66f;
    fp32 p12 =  -15.27f;
    fp32 p03 =   25.98f;
    fp32 p40 =   2.884f;
    fp32 p31 =  -71.75f;
    fp32 p22 =    41.4f;
    fp32 p13 =   16.83f;
    fp32 p04 =  -23.57f;
    fp32 J2;
    fp32 x = 0;

	x = h*sin(pi/2-a);
	h = h*cos(pi/2-a);
	if(x<-0.2f)x=-0.2f;else if(x>0.2f)x=0.2f;
	if(h<0.12f)h=0.3;else if(h>0.3)h=0.3;

  J2 = (p00 + p10*x + p01*h + p20*x*x + p11*x*h + p02*h*h + p30*x*x*x + p21*x*x*h 
  + p12*x*h*h + p03*h*h*h + p40*x*x*x*x + p31*x*x*x*h + p22*x*x*h*h 
  + p13*x*h*h*h + p04*h*h*h*h);
	return J2;
}

fp32 VMC_solve_invJ2(fp32 a,fp32 h)
{
    fp32 p00 =   46.51;
    fp32 p10 =   252.7;
    fp32 p01 =  -666.5;
    fp32 p20 =   16.58;
    fp32 p11 =   -3169;
    fp32 p02 =    4117;
    fp32 p30 =   -1346;
    fp32 p21 =  -271.1;
    fp32 p12 = 1.358e+04;
    fp32 p03 = -1.17e+04;
    fp32 p40 =  -583.1;
    fp32 p31 =    5547;
    fp32 p22 =    1172;
    fp32 p13 = -1.986e+04;
    fp32 p04 = 1.288e+04;
    fp32 J2;
    fp32 x = 0;
	x = h*sin(pi/2-a);
	h = h*cos(pi/2-a);
	if(x<-0.2f)x=-0.2f;else if(x>0.2f)x=0.2f;
	if(h<0.12f)h=0.12f;else if(h>0.3)h=0.3;

  J2 = (p00 + p10*x + p01*h + p20*x*x + p11*x*h + p02*h*h + p30*x*x*x + p21*x*x*h 
  + p12*x*h*h + p03*h*h*h + p40*x*x*x*x + p31*x*x*x*h + p22*x*x*h*h 
  + p13*x*h*h*h + p04*h*h*h*h);
	return J2;
}


fp32 VMC_solve_J3(fp32 a,fp32 h)
{
    fp32 p00 = -0.1007;
    fp32 p10 =  11.61f;
    fp32 p01 =  7.412f;
    fp32 p20 =  12.82f;
    fp32 p11 = -135.6f;
    fp32 p02 = -43.23f;
    fp32 p30 = -100.2f;
    fp32 p21 = -105.5f;
    fp32 p12 =  550.1f;
    fp32 p03 =  118.5f;
    fp32 p40 = -6.175f;
    fp32 p31 =  407.0f;
    fp32 p22 =  218.5f;
    fp32 p13 = -778.6f;
    fp32 p04 = -129.3f;
    fp32 J3;
	fp32 x=0;
	x = h*sin(pi/2-a);
	h = h*cos(pi/2-a);
	if(x<-0.2f)x=-0.2f;else if(x>0.2f)x=0.2f;
	if(h<0.12f)h=0.12f;else if(h>0.3)h=0.3;
  J3 = p00 + p10*x + p01*h + p20*x*x + p11*x*h + p02*h*h + p30*x*x*x + p21*x*x*h 
  + p12*x*h*h + p03*h*h*h + p40*x*x*x*x + p31*x*x*x*h + p22*x*x*h*h 
  + p13*x*h*h*h + p04*h*h*h*h;
	return J3;
}

fp32 VMC_solve_invJ3(fp32 a,fp32 h)
{
    fp32 p00 =  3.724;
    fp32 p10 = -31.85;
    fp32 p01 =  -39.7;
    fp32 p20 =  13.23;
    fp32 p11 =  354.5;
    fp32 p02 =  247.3;
    fp32 p30 =  298.6;
    fp32 p21 = -132.5;
    fp32 p12 =  -1423;
    fp32 p03 = -706.1;
    fp32 p40 = -107.8;
    fp32 p31 =  -1193;
    fp32 p22 =  325.6;
    fp32 p13 =   1994;
    fp32 p04 =    778;
    fp32 J3;
	fp32 x=0;
	x = h*sin(pi/2-a);
	h = h*cos(pi/2-a);
	if(x<-0.2f)x=-0.2f;else if(x>0.2f)x=0.2f;
	if(h<0.12f)h=0.12f;else if(h>0.3)h=0.3;
  J3 = p00 + p10*x + p01*h + p20*x*x + p11*x*h + p02*h*h + p30*x*x*x + p21*x*x*h 
  + p12*x*h*h + p03*h*h*h + p40*x*x*x*x + p31*x*x*x*h + p22*x*x*h*h 
  + p13*x*h*h*h + p04*h*h*h*h;
	return J3;
}
fp32 VMC_solve_J4(fp32 a,fp32 h)
{
    fp32 p00 = -0.08806;
    fp32 p10 =   -11.61;
    fp32 p01 =    7.147;
    fp32 p20 =    12.88;
    fp32 p11 =    135.6;
    fp32 p02 =   -41.17;
    fp32 p30 =    100.6;
    fp32 p21 =   -106.2;
    fp32 p12 =   -550.1;
    fp32 p03 =    111.4;
    fp32 p40 =   -7.211;
    fp32 p31 =   -408.9;
    fp32 p22 =    220.5;
    fp32 p13 =    778.5;
    fp32 p04 =   -120.4;
    fp32 J4;
	fp32 x;
	x = h*sin(pi/2-a);
	h = h*cos(pi/2-a);
	if(x<-0.2f)x=-0.2f;else if(x>0.2f)x=0.2f;
	if(h<0.12f)h=0.12f;else if(h>0.3)h=0.3;
  J4 = p00 + p10*x + p01*h + p20*x*x + p11*x*h + p02*h*h + p30*x*x*x + p21*x*x*h 
  + p12*x*h*h + p03*h*h*h + p40*x*x*x*x + p31*x*x*x*h + p22*x*x*h*h 
  + p13*x*h*h*h + p04*h*h*h*h;
	return J4;
}

fp32 VMC_solve_invJ4(fp32 a,fp32 h)
{
    fp32 p00 =  3.723;
    fp32 p10 =  31.86;
    fp32 p01 = -39.68;
    fp32 p20 =  13.22;
    fp32 p11 = -354.5;
    fp32 p02 =  247.1;
    fp32 p30 = -298.6;
    fp32 p21 = -132.4;
    fp32 p12 =   1423;
    fp32 p03 = -705.6;
    fp32 p40 =   -108;
    fp32 p31 =   1193;
    fp32 p22 =  325.4;
    fp32 p13 =  -1995;
    fp32 p04 =  777.4;
    fp32 J4;
	fp32 x;
	x = h*sin(pi/2-a);
	h = h*cos(pi/2-a);
	if(x<-0.2f)x=-0.2f;else if(x>0.2f)x=0.2f;
	if(h<0.12f)h=0.12f;else if(h>0.3f)h=0.3f;
  J4 = p00 + p10*x + p01*h + p20*x*x + p11*x*h + p02*h*h + p30*x*x*x + p21*x*x*h 
  + p12*x*h*h + p03*h*h*h + p40*x*x*x*x + p31*x*x*x*h + p22*x*x*h*h 
  + p13*x*h*h*h + p04*h*h*h*h;
	return J4;
}