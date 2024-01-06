#include <stdio.h>
#include <string.h>
#include <math.h>
#include <VMC.h>



void VMC_init()
{
    VMC_LEG_R.L1=LEG_1;
    VMC_LEG_R.L2=LEG_2;
    VMC_LEG_R.L3=LEG_2;
    VMC_LEG_R.L4=LEG_1;
    VMC_LEG_R.L5=LEG_0;
    
     
    VMC_LEG_L.L1=LEG_1;
    VMC_LEG_L.L2=LEG_2;
    VMC_LEG_L.L3=LEG_2;
    VMC_LEG_L.L4=LEG_1;
    VMC_LEG_L.L5=LEG_0;
    
}

void VMC_calc(float leg_r_F,float leg_r_Tp,float leg_l_F,float leg_l_Tp)
{
    VMC_LEG_R.F=leg_r_F;
    VMC_LEG_R.Tp=leg_r_Tp;
    
    VMC_LEG_L.F=leg_l_F;
    VMC_LEG_L.Tp=leg_l_Tp;
    
    //右侧关节电机
    VMC_LEG_R.xD_xB=(LEG_1*cos(VMC_LEG_R.Phi4))+LEG_1*cos(pi-VMC_LEG_R.Phi1)+LEG_0;
    VMC_LEG_R.yD_yB=LEG_1*sin(VMC_LEG_R.Phi4)-LEG_1*sin(pi-VMC_LEG_R.Phi1);
    VMC_LEG_R.L_BD=(sqrt((VMC_LEG_R.xD_xB*VMC_LEG_R.xD_xB)+(VMC_LEG_R.yD_yB*VMC_LEG_R.yD_yB)));
    VMC_LEG_R.A0=2*LEG_2*VMC_LEG_R.xD_xB;
    VMC_LEG_R.B0=2*LEG_2*VMC_LEG_R.yD_yB;
    VMC_LEG_R.C0=VMC_LEG_R.L_BD*VMC_LEG_R.L_BD;
    
    VMC_LEG_R.Phi2=2*atan2((VMC_LEG_R.B0+sqrt((VMC_LEG_R.A0*VMC_LEG_R.A0)+( VMC_LEG_R.B0*VMC_LEG_R.B0)-(VMC_LEG_R.C0*VMC_LEG_R.C0))),(VMC_LEG_R.A0+VMC_LEG_R.C0));
    VMC_LEG_R.Phi3=acos((-VMC_LEG_R.xD_xB+LEG_2*cos(VMC_LEG_R.Phi2))/LEG_2);
    
    VMC_LEG_R.xC=LEG_1*cos(VMC_LEG_R.Phi1)+LEG_2*cos(VMC_LEG_R.Phi2)-0.1;
    VMC_LEG_R.yC=LEG_1*sin(VMC_LEG_R.Phi1)+LEG_2*sin(VMC_LEG_R.Phi2);
    
    VMC_LEG_R.L0=sqrt(pow(VMC_LEG_R.xC,2)+pow(VMC_LEG_R.yC,2));
    VMC_LEG_R.Phi0=atan2(VMC_LEG_R.yC,VMC_LEG_R.xC);
    
    
    VMC_LEG_R.J[0][0]=(LEG_1*sin(VMC_LEG_R.Phi0-VMC_LEG_R.Phi3)*sin(VMC_LEG_R.Phi1-VMC_LEG_R.Phi2))/sin(VMC_LEG_R.Phi3-VMC_LEG_R.Phi2);
    VMC_LEG_R.J[0][1]=(LEG_1*cos(VMC_LEG_R.Phi0-VMC_LEG_R.Phi3)*sin(VMC_LEG_R.Phi1-VMC_LEG_R.Phi2))/LEG_0*sin(VMC_LEG_R.Phi3-VMC_LEG_R.Phi2);
    VMC_LEG_R.J[1][0]=(LEG_1*sin(VMC_LEG_R.Phi0-VMC_LEG_R.Phi2)*sin(VMC_LEG_R.Phi3-VMC_LEG_R.Phi4))/sin(VMC_LEG_R.Phi3-VMC_LEG_R.Phi2);
    VMC_LEG_R.J[1][1]=(LEG_1*cos(VMC_LEG_R.Phi0-VMC_LEG_R.Phi2)*sin(VMC_LEG_R.Phi3-VMC_LEG_R.Phi4))/LEG_0*sin(VMC_LEG_R.Phi3-VMC_LEG_R.Phi2);
    
    printf("J1:%f \n",VMC_LEG_R.L1);
    VMC_LEG_R.T[0]=VMC_LEG_R.J[0][0]*VMC_LEG_R.F+VMC_LEG_R.J[0][1]*VMC_LEG_R.Tp;
    VMC_LEG_R.T[1]=VMC_LEG_R.J[1][0]*VMC_LEG_R.F+VMC_LEG_R.J[1][1]*VMC_LEG_R.Tp;
    
    
    
    //左侧关节电机
    VMC_LEG_L.xD_xB=(LEG_1*cos(VMC_LEG_L.Phi4))+LEG_1*cos(pi-VMC_LEG_L.Phi1)+LEG_0;
    VMC_LEG_L.yD_yB=LEG_1*sin(VMC_LEG_L.Phi4)-LEG_1*sin(pi-VMC_LEG_L.Phi1);
    VMC_LEG_L.L_BD=(sqrt((VMC_LEG_L.xD_xB*VMC_LEG_L.xD_xB)+(VMC_LEG_L.yD_yB*VMC_LEG_L.yD_yB)));
    VMC_LEG_L.A0=2*LEG_2*VMC_LEG_L.xD_xB;
    VMC_LEG_L.B0=2*LEG_2*VMC_LEG_L.yD_yB;
    VMC_LEG_L.C0=VMC_LEG_L.L_BD*VMC_LEG_L.L_BD;
    
    VMC_LEG_L.Phi2=2*atan2((VMC_LEG_L.B0+sqrt((VMC_LEG_L.A0*VMC_LEG_L.A0)+( VMC_LEG_L.B0*VMC_LEG_L.B0)-(VMC_LEG_L.C0*VMC_LEG_L.C0))),(VMC_LEG_L.A0+VMC_LEG_L.C0));
    VMC_LEG_L.Phi3=acos(-VMC_LEG_L.xD_xB+LEG_2*cos(VMC_LEG_L.Phi2)/LEG_2);
    
    VMC_LEG_L.xC=LEG_1*cos(VMC_LEG_L.Phi1)+LEG_2*cos(VMC_LEG_L.Phi2);
    VMC_LEG_L.yC=LEG_1*sin(VMC_LEG_L.Phi1)+LEG_2*sin(VMC_LEG_L.Phi2);
    
    VMC_LEG_L.L0=sqrt(pow(VMC_LEG_L.xC-(VMC_LEG_L.L5/2),2)+pow(VMC_LEG_L.yC,2));
    VMC_LEG_L.Phi0=atan2(VMC_LEG_L.xC-(LEG_0/2),VMC_LEG_L.yC);
    
    
    VMC_LEG_L.J[0][0]=(VMC_LEG_L.L1*sin(VMC_LEG_L.Phi0-VMC_LEG_L.Phi3)*sin(VMC_LEG_L.Phi1-VMC_LEG_L.Phi2))/sin(VMC_LEG_L.Phi3-VMC_LEG_L.Phi2);
    VMC_LEG_L.J[0][1]=(VMC_LEG_L.L1*cos(VMC_LEG_L.Phi0-VMC_LEG_L.Phi3)*sin(VMC_LEG_L.Phi1-VMC_LEG_L.Phi2))/sin(VMC_LEG_L.Phi3-VMC_LEG_L.Phi2);
    VMC_LEG_L.J[1][0]=(VMC_LEG_L.L4*sin(VMC_LEG_L.Phi0-VMC_LEG_L.Phi2)*sin(VMC_LEG_L.Phi3-VMC_LEG_L.Phi4))/sin(VMC_LEG_L.Phi3-VMC_LEG_L.Phi2);
    VMC_LEG_L.J[1][1]=(VMC_LEG_L.L4*cos(VMC_LEG_L.Phi0-VMC_LEG_L.Phi2)*sin(VMC_LEG_L.Phi1-VMC_LEG_L.Phi4))/sin(VMC_LEG_L.Phi3-VMC_LEG_L.Phi2);
    
    VMC_LEG_L.T[0]=VMC_LEG_L.J[0][0]*VMC_LEG_L.F+VMC_LEG_L.J[0][1]*VMC_LEG_L.Tp;
    VMC_LEG_L.T[1]=VMC_LEG_L.J[1][0]*VMC_LEG_L.F+VMC_LEG_L.J[1][1]*VMC_LEG_L.Tp;
    
}