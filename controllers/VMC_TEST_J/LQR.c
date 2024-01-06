#include <LQR.h>
#include <VMC.h>
#include <VMC_TEST_J.h>
#include <struct_typedef.h>
#include <stdio.h>
#include <plot.h>
/*
LQR参数含义
[0][0]:保持腿在中间的力
[0][1]:腿移动的阻尼

[0][2]:距离误差对腿前后的力
[0][3]:速度误差对腿前后的力

[0][4]:pitch轴误差对腿前后的力
[0][5]:pitch轴角加速度对腿前后的力

[1][0]:腿的摆角对轮的力
[1][1]:腿摆动速度对轮的力

[1][2]:距离误差对轮的力
[1][3]:速度误差对轮的力

[1][4]:pitch轴误差对轮的力
[1][5]:pitch轴角加速度对轮的力
*/



// fp32 balance_LQR[2][6] = {{0, 0, 0,0,  0, 0},
//                           {0, 0, -0.77, 0.66,-100, 5}};//6


// fp32 balance_LQR[2][6] = {{100, 0.01, 1,-0.005,  1, -0.01},
//                           {1, -0.001, -0.002, 0.0005,-50, 0.01}};//抖 0.2


// fp32 balance_LQR[2][6] = {{100, -0.07, 2,-0.2,  2.5, -0.1},
//                           {2, -0.0001, -0.002, 0.5,-50, 0.01}};//比较稳 0.26

// fp32 balance_LQR[2][6] = {{-120, -0.07, 1.5,-0.2,  -10, -0.1},
//                           {5, 0.02, -0.003, 0.3,-50, 0.01}};//0.3

// fp32 balance_LQR[2][6] = {{-20.2427, -1.9145, -1.2926, -2.0415, 3.5869, 0.6096},
//                           {18.4767, 1.7950, 1.9898, 2.9721, 15.4028, 1.6699}}; 
                           // 0.3 站一会就倒 Q[100 1 500 100 5000 1]  R[240 0;0 25]

// fp32 balance_LQR[2][6] = {{ -21.9797,   -2.2360,   -2.1470,   -2.8792,    3.6579,    0.6380},
//                           {  9.8425,    0.8546,    1.2498,    1.5649,   16.9810,    1.9443}}; 
//                           // 0.3 站的时间长点，但是有点软 Q[1 1 500 100 5000 1]  R[100 0;0 25]

// fp32 balance_LQR[2][6] = {{ -20.5982,  -1.9774,   -1.4441,   -2.1994,    3.6181,    0.6176},
//                           {  16.0889,   1.5441,    1.8210,    2.6181,   15.8377,    1.7450}}; 
                          // 0.3  Q[1 1 500 100 5000 1]  R[200 0;0 25] 抖的厉害

// fp32 balance_LQR[2][6] = {{ -21.9836,   -2.2362,   -2.1470,   -2.8796,    3.6580,    0.6380},
//                           { 9.8460,    0.8548,    1.2497,    1.5651 ,  16.9808 ,   1.9443}}; 
//                           // 0.3  Q[10 1 500 100 5000 1]  R[100 0;0 25] 还是pitch轴软


// fp32 balance_LQR[2][6] = {{ -22.1135,   -2.2526 ,  -2.1668  , -2.9037 ,   4.1367   , 0.6760},
//                           {8.9402,    0.7383 ,   1.1043 ,   1.3857  , 22.7228  ,  2.2699}}; 
//                           // 0.3  Q[10 1 500 100 10000 1]  R[100 0;0 25] 摇摇倒，pitch发散了

// fp32 balance_LQR[2][6] = {
//     {-22.056749, -2.242868, -2.152217, -2.889997, 3.781543, 0.648094},
//     {9.651573, 0.827101, 1.213195, 1.522593, 18.290783, 2.022675}
// };// 0.3  Q[100 1 500 100 6000 1]  R[100 0;0 25] 还是发散

// fp32 balance_LQR[2][6] = {
//     {-22.099593, -2.245451, -2.152293, -2.894601, 3.783822, 0.648329},
//     {9.689251, 0.828710, 1.212655, 1.524306, 18.289208, 2.022588}
// };// 0.3  Q[200 1 500 100 6000 1]  R[100 0;0 25] 还是发散 感觉极性还是不对

// fp32 balance_LQR[2][6] = {
//     {-22.108277, -2.246593, -2.153732, -2.896453, 3.766192, 0.650017},
//     {9.631694, 0.820943, 1.202392, 1.510895, 18.303687, 2.059409}
// };// 0.3  Q[200 1 500 100 6000 5]  R[100 0;0 25]

// fp32 balance_LQR[2][6] = {
//     {-57.038653, -8.636040, -21.475019, -18.216752, 15.516335, 0.986983},
//     {21.140563, 4.347917, 12.461708, 10.100100, 133.745325, 6.366477}
// };// 0.3  Q[1 1 500 100 5000 1]  R[1 0;0 0.25]

// fp32 balance_LQR[2][6] = {
//     {-21.760555, -1.873644, -1.204665, -1.890563, 0.997786, 0.347096},
//     {18.266717, 2.225909, 2.463396, 3.665941, 9.864029, 1.449594}
// };// Q=diag([100 1 500 100 5000 1])  R=[240 0;0 25]; 

// fp32 balance_LQR[2][6] = {
//     {-18.484525, -1.383651, -1.153759, -1.676050, 1.389566, 0.381064},
//     {17.178161, 1.961498, 2.687168, 3.721178, 9.346981, 1.376513}
// };//


// fp32 balance_LQR[2][6] = {
//     {-16.160364, -1.167372, -0.708913, -1.197915, 2.443218, 0.447280},
//     {30.452496, 2.944790, 3.154180, 5.096037, 10.982430, 1.196252}
// };
// fp32 balance_LQR[2][6] = {{-44.3788,-6.8496,-22.2828,-21.5569,28.7706,4.3751}, 
//   
//                        {11.2006,0.7339,3.7300,3.2058,151.7300,4.6387}};   //王洪玺参数
// fp32 balance_LQR[2][6] = {
//     {-22.010588, -2.235789, -3.054872, -3.734749, 3.654462, 0.604787},
//     {4.507059, 0.623980, 1.155642, 1.352092, 19.320709, 2.181884}
// };//6s

// fp32 balance_LQR[2][6] = {
//     {21.851334, 2.213966, 3.013800, 3.686799, 4.282550, 0.645576},
//     {6.836077, 0.950048, 1.748344, 2.047744, 24.607571, 2.428456}
// };

// fp32 balance_LQR[2][6] = {
//     {21.255668, 2.124001, 2.058307, 3.276219, 2.762917, 0.530059},
//     {9.524963, 1.304143, 1.747422, 2.682072, 13.017879, 1.757620}
// };

// fp32 balance_LQR[2][6] = {
//     {-17.834041, -1.404041, -1.205107, -1.750091, 2.512070, 0.475121},
//     {17.072803, 1.803317, 2.461316, 3.419630, 11.807593, 1.490798}
// };//hzy

// fp32 balance_LQR[2][6] = {
//     {-53.611049, -8.585580, -21.402946, -18.237506, 20.472889, 1.600563},
//     {24.706020, 4.617232, 12.948192, 10.585885, 135.364114, 5.906474}
// };//whx QR

// fp32 balance_LQR[2][6] = {
//     {-16.790121, -1.022707, -0.088559, -0.508796, 1.038597, 0.306064},
//     {8.297929, 0.579537, 0.092895, 0.528183, 3.960463, 0.931481}
// };
// fp32 balance_LQR[2][6] = {
//     {-16.986826, -1.036263, -0.090733, -0.521214, 1.329465, 0.370994},
//     {7.513290, 0.524935, 0.084083, 0.478134, 5.738474, 1.268537}
// };

// fp32 balance_LQR[2][6] = {
//     {-17.875692, -1.310073, -0.090162, -1.194811, 1.367755, 0.382073},
//     {8.326132, 0.777540, 0.086504, 1.141262, 5.702367, 1.258410}
// };//能跑

// fp32 balance_LQR[2][6] = {
//     {-17.690696, -1.259276, -0.063846, -1.059620, 1.359147, 0.379568},
//     {8.146555, 0.729250, 0.060783, 1.005222, 5.710594, 1.260828}
// };//可以

fp32 balance_LQR[2][6] = {
    {-120, -1.2, -30, -1100, 1, 0.5},
    {10, 1, 0.1, 1, 7.065601, 1.728580}
};


//0.2
// fp32 balance_LQR[2][6] = {
//     {-15.077008, -0.845866, -0.056395, -0.934659, 1.907645, 0.515377},
//     {10.271784, 0.723776, 0.085312, 1.409237, 5.044162, 1.086438}
// };
// fp32 balance_LQR[2][6] = {
//     {-14.997089, -0.821375, -0.061789, -0.869364, 1.907059, 0.514733},
//     {10.188629, 0.691959, 0.093426, 1.309409, 5.045048, 1.086868}
// };//刹车好
// fp32 balance_LQR[2][6] = {
//     {-15.309515, -0.851245, -0.074128, -0.951428, 1.769557, 0.611247},
//     {9.498143, 0.658229, 0.100101, 1.279563, 5.241629, 1.536590}
// };
//耦合问题，现在pitch轴奇怪的起伏，但是腿部关节角度却偏转很小，包括在转弯的时候
//12.18 为什么Q矩阵x小，xdot大，因为角速度变成线速度了，但是就角度没变距离！！！
//以下就是纠正反馈错误后的参数

// fp32 balance_LQR[2][6] = {
//     {-17.965765, -1.331328, -0.901705, -1.477927, 1.674492, 0.395944},
//     {8.307744, 0.792628, 0.864704, 1.366730, 6.984576, 1.243223}
// };  // 0.300000 Q[1 1 1000 100 15000 1] R[1000 250]

// fp32 balance_LQR[2][6] = {
//     {-20, -0, -0, -0, 0, 0},
//     {0, 0,0, 5, 5, 0.0}
// };  // 0.300000 Q[1 1 1000 100 15000 1] R[1000 250]




void LQR_init()
{
    float coordinate_PD[3] = {10, 0, 0};
    PID_init(&LQR.coordinate_PD,PID_POSITION, coordinate_PD, 20, 0);

    float stand_PID[3] = {130, 1, 1500};
    PID_init(&LQR.stand_PID, PID_POSITION, stand_PID, 300, 100);
    LQR.stand_feed =130;               //前馈推力 等于 高度稳定状态下扭矩大小 130 15KG

    float roll_PD[3] = {100, 0, 00};//500 0 100
    PID_init(&LQR.roll_PID, PID_POSITION, roll_PD, 300, 100);

    float yaw_PD[3] = {5, 0, 20};
    PID_init(&LQR.yaw_PD, PID_POSITION, yaw_PD, 2, 1);

    chassis_data.foot_distance_set = 0;
    chassis_data.pitch_set = 0;
    chassis_data.yaw_set = chassis_data.yaw;
    chassis_data.roll_set = 0;
    chassis_data.leg_length_set[0] = 0.25;
    chassis_data.leg_length_set[1] = 0.25;
}

void LQR_calc()
{



        //关节电机平衡所用扭矩 Tp
        //也就是说，这里的反馈只是定义了各个反馈量的正方向，而参数的正负已经定了，就是K阵的方向（取反）
        LQR.LQR_FEED_R[0][0] = (chassis_data.leg_angle[0]-pi/2 );//
        LQR.LQR_FEED_R[0][1] = (-chassis_data.leg_gyro[0]);//如何确定？单给轮的pitch，然后leg_angle给点，再确定leg_gyro的方向，这里改的并不是参数，而是物理系统的方向
        LQR.LQR_FEED_R[0][2] = (chassis_data.foot_distance_set - chassis_data.foot_distance);
        LQR.LQR_FEED_R[0][3] = (-chassis_data.foot_speed_lpf);//方向待定--这里又是轮和Tp的方向不一样，所以在Tp上做特化
        LQR.LQR_FEED_R[0][4] = (chassis_data.pitch);
        LQR.LQR_FEED_R[0][5] = (chassis_data.gyro_pitch);

        LQR.LQR_OUT_R[0][0] = -balance_LQR[0][0] *  LQR.LQR_FEED_R[0][0];
        LQR.LQR_OUT_R[0][1] = -balance_LQR[0][1] *  LQR.LQR_FEED_R[0][1];
        LQR.LQR_OUT_R[0][2] = -balance_LQR[0][2] * -LQR.LQR_FEED_R[0][2];
        LQR.LQR_OUT_R[0][3] = -balance_LQR[0][3] * -LQR.LQR_FEED_R[0][3];
        LQR.LQR_OUT_R[0][4] = -balance_LQR[0][4] *  LQR.LQR_FEED_R[0][4];
        LQR.LQR_OUT_R[0][5] = -balance_LQR[0][5] *  LQR.LQR_FEED_R[0][5];

        chassis_data.K_balance_T[0] = 
        + LQR.LQR_OUT_R[0][0] 
        + LQR.LQR_OUT_R[0][1] 
        + LQR.LQR_OUT_R[0][2] 
        + LQR.LQR_OUT_R[0][3] 
        + LQR.LQR_OUT_R[0][4] 
        + LQR.LQR_OUT_R[0][5];
        
        //
        LQR.LQR_FEED_L[0][0] = (chassis_data.leg_angle[1]-pi/2);//
        LQR.LQR_FEED_L[0][1] = (-chassis_data.leg_gyro[1]);
        LQR.LQR_FEED_L[0][2] = (chassis_data.foot_distance_set - chassis_data.foot_distance);
        LQR.LQR_FEED_L[0][3] = (-chassis_data.foot_speed_lpf);
        LQR.LQR_FEED_L[0][4] = (chassis_data.pitch);
        LQR.LQR_FEED_L[0][5] = (chassis_data.gyro_pitch);

        LQR.LQR_OUT_L[0][0] = -balance_LQR[0][0] *  LQR.LQR_FEED_L[0][0];
        LQR.LQR_OUT_L[0][1] = -balance_LQR[0][1] *  LQR.LQR_FEED_L[0][1];
        LQR.LQR_OUT_L[0][2] = -balance_LQR[0][2] * -LQR.LQR_FEED_L[0][2];
        LQR.LQR_OUT_L[0][3] = -balance_LQR[0][3] * -LQR.LQR_FEED_L[0][3];
        LQR.LQR_OUT_L[0][4] = -balance_LQR[0][4] *  LQR.LQR_FEED_L[0][4];
        LQR.LQR_OUT_L[0][5] = -balance_LQR[0][5] *  LQR.LQR_FEED_L[0][5];

        chassis_data.K_balance_T[1] = 
        + LQR.LQR_OUT_L[0][0] 
        + LQR.LQR_OUT_L[0][1] 
        + LQR.LQR_OUT_L[0][2] 
        + LQR.LQR_OUT_L[0][3] 
        + LQR.LQR_OUT_L[0][4] 
        + LQR.LQR_OUT_L[0][5];




        // 关节电机协调扭矩
        chassis_data.K_coordinate_T = PID_calc(&LQR.coordinate_PD, chassis_data.leg_angle[0] - chassis_data.leg_angle[1], 0);

        // F
        // stand
        chassis_data.K_stand_T[0] = LQR.stand_feed + PID_calc(&LQR.stand_PID, chassis_data.leg_length[0], chassis_data.leg_length_set[0]);
        chassis_data.K_stand_T[1] = LQR.stand_feed + PID_calc(&LQR.stand_PID, chassis_data.leg_length[1], chassis_data.leg_length_set[1]);

        // roll
        chassis_data.K_roll_T[0] = PID_calc(&LQR.roll_PID, chassis_data.roll, chassis_data.roll_set);
        chassis_data.K_roll_T[1] = PID_calc(&LQR.roll_PID, chassis_data.roll, chassis_data.roll_set);

        // 驱动轮电机

        //这里，本身leg和wheel公用一个feedback
        LQR.LQR_OUT_R[1][0] = +balance_LQR[1][0] * LQR.LQR_FEED_R[0][0];//这里轮和leg_angle是正反馈，但是参数是负反馈，所以需要反一下
        LQR.LQR_OUT_R[1][1] = +balance_LQR[1][1] * LQR.LQR_FEED_R[0][1];
        LQR.LQR_OUT_R[1][2] = -balance_LQR[1][2] * LQR.LQR_FEED_R[0][2];
        LQR.LQR_OUT_R[1][3] = -balance_LQR[1][3] * LQR.LQR_FEED_R[0][3];
        LQR.LQR_OUT_R[1][4] = -balance_LQR[1][4] * LQR.LQR_FEED_R[0][4];
        LQR.LQR_OUT_R[1][5] = -balance_LQR[1][5] * LQR.LQR_FEED_R[0][5];

        chassis_data.Wheel_motor_T[0] =
            + LQR.LQR_OUT_R[1][0] 
            + LQR.LQR_OUT_R[1][1] 
            + LQR.LQR_OUT_R[1][2] 
            + LQR.LQR_OUT_R[1][3] 
            + LQR.LQR_OUT_R[1][4] 
            + LQR.LQR_OUT_R[1][5]
            - PID_calc(&LQR.yaw_PD, chassis_data.yaw, chassis_data.yaw_set);
        printf("yaw %f yaw_set %f\n", chassis_data.yaw, chassis_data.yaw_set);

        //

        LQR.LQR_OUT_L[1][0] = +balance_LQR[1][0] * LQR.LQR_FEED_L[0][0];
        LQR.LQR_OUT_L[1][1] = +balance_LQR[1][1] * LQR.LQR_FEED_L[0][1];
        LQR.LQR_OUT_L[1][2] = -balance_LQR[1][2] * LQR.LQR_FEED_L[0][2];
        LQR.LQR_OUT_L[1][3] = -balance_LQR[1][3] * LQR.LQR_FEED_L[0][3];
        LQR.LQR_OUT_L[1][4] = -balance_LQR[1][4] * LQR.LQR_FEED_L[0][4];
        LQR.LQR_OUT_L[1][5] = -balance_LQR[1][5] * LQR.LQR_FEED_L[0][5];

        chassis_data.Wheel_motor_T[1] =
            + LQR.LQR_OUT_L[1][0] 
            + LQR.LQR_OUT_L[1][1] 
            + LQR.LQR_OUT_L[1][2] 
            + LQR.LQR_OUT_L[1][3] 
            + LQR.LQR_OUT_L[1][4] 
            + LQR.LQR_OUT_L[1][5]
            + PID_calc(&LQR.yaw_PD, chassis_data.yaw, chassis_data.yaw_set);

        
        LQR_log();

        printf("%f", chassis_data.leg_length_set[0]);

        // for (int i = 0; i < 6;i++)
        // {
        //     printf("leg_R  :%d  %f  %f\n", i, LQR.LQR_FEED_R[0][i], LQR.LQR_OUT_R[0][i]);
        // }
        // for (int i = 0; i < 6;i++)
        // {
        //     printf("wheel_R:%d  %f  %f\n", i, LQR.LQR_FEED_R[0][i], LQR.LQR_OUT_R[1][i]);
        // }
        // for (int i = 0; i < 6;i++)
        // {
        //     printf("leg_L  :%d  %f  %f\n", i, LQR.LQR_FEED_L[0][i], LQR.LQR_OUT_L[0][i]);
        // }
        //  for (int i = 0; i < 6;i++)
        // {
        //     printf("wheel_L:%d  %f  %f\n", i, LQR.LQR_FEED_L[0][i], LQR.LQR_OUT_L[1][i]);
        // }
}

void LQR_log()
{

    plotFile6("LQR_R", wb_robot_get_time(), LQR.LQR_OUT_R[0][0],
                                            LQR.LQR_OUT_R[0][1],
                                            LQR.LQR_OUT_R[0][2],
                                            LQR.LQR_OUT_R[0][3],
                                            LQR.LQR_OUT_R[0][4],
                                            LQR.LQR_OUT_R[0][5]);

    plotFile6("LQR_L", wb_robot_get_time(), LQR.LQR_OUT_L[0][0],
                                            LQR.LQR_OUT_L[0][1],
                                            LQR.LQR_OUT_L[0][2],
                                            LQR.LQR_OUT_L[0][3],
                                            LQR.LQR_OUT_L[0][4],
                                            LQR.LQR_OUT_L[0][5]);    

    plotFile6("LQR_WR", wb_robot_get_time(),LQR.LQR_OUT_R[1][0],
                                            LQR.LQR_OUT_R[1][1],
                                            LQR.LQR_OUT_R[1][2],
                                            LQR.LQR_OUT_R[1][3],
                                            LQR.LQR_OUT_R[1][4],
                                            LQR.LQR_OUT_R[1][5]);     

    plotFile6("LQR_WL", wb_robot_get_time(),LQR.LQR_OUT_L[1][0],
                                            LQR.LQR_OUT_L[1][1],
                                            LQR.LQR_OUT_L[1][2],
                                            LQR.LQR_OUT_L[1][3],
                                            LQR.LQR_OUT_L[1][4],
                                            LQR.LQR_OUT_L[1][5]);
}



