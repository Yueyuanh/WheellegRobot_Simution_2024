#ifndef VMC_H
#define VMC_H

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
    float A0,B0,C0,L_BD,xD_xB,yD_yB;
    float xB,xD,yB,yD;
    
    float xC,yC;
    //定义虚拟变量
    float L0,Phi0;
   
    //定义输入量
    float F,Tp;
   
    //定义雅可比矩阵
    double J[2][2];
   
    //定义输出量
    float T[2];
    
}VMC_LEG_R,VMC_LEG_L;







void VMC_init();
void VMC_calc(float,float,float,float);





#endif
