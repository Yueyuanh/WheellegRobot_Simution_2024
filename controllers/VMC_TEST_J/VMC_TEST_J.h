#ifndef LQR_VMC_H
#define LQR_VMC_H

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/Gyro.h>
#include <webots/Accelerometer.h>
#include <webots/Compass.h>
#include <webots/Inertial_unit.h>
#include <webots/Keyboard.h>
#include <webots/Camera.h>
#include <webots/supervisor.h>

#include <user_lib.h>
#include <KalmanFilter.h>

struct chassis_tag_t
{
    WbDeviceTag rightWheel; 
    WbDeviceTag leftWheel;
    WbDeviceTag rfLeg;
    WbDeviceTag rbLeg; 
    WbDeviceTag lfLeg;  
    WbDeviceTag lbLeg;
    WbDeviceTag IMU;
    WbDeviceTag GYRO;
    WbDeviceTag ACCEL;
} chassis_tag;

struct chassis_data_t
{
    //观测器结构体
    differ_type_def wheel_speed_r, wheel_speed_l;
    Kalman_fir_t kalman_speed;
    Kalman_sec_t kalman_distance;

    // 观测器数据
    float yaw, pitch, roll;                // 2 1 0
    float yaw_sum, yaw_last, yaw_count;
    float accel_x, accel_y, accel_z;       // 0 1 2
    float gyro_yaw, gyro_pitch, gyro_roll; //

    float leg_angle[2];   // 腿相对于竖直方向的角度
    float leg_gyro[2];    //腿转的角加速度
    float leg_length[2];  // 腿的长度

    float leg_Torque[4];    // 关节电机反馈力矩
    float leg_position[4]; // 关节电机反馈角度
    float leg_speed[4];    // 关节电机反馈转速

    float wheel_Torque[2];       // 驱动轮电机反馈力矩
    float wheel_position[2];     // 驱动轮电机反馈位置
    float wheel_position_last[2];// 驱动轮电机上次反馈位置
    float wheel_speed[2];        // 驱动轮电机速度
    float wheel_speed_lpf[2];    // 驱动轮电机速度低通
    float wheel_speed_last[2];   // 上次驱动轮电机速度
    float wheel_turns[2];        // 电机圈数
    float wheel_distance[2];     // 驱动轮位移
    float wheel_distance_dot[2]; // 驱动轮速度

    float foot_speed;
    float foot_speed_lpf;
    float foot_speed_last;
    float foot_speed_kalman;
    float foot_distance;

    float free_flag;             //是否悬空
    float leg_stand_F;           //跳跃推力

    // 期望值设置
    float yaw_set, pitch_set, roll_set;       // 三轴角度设定
    float x_set, x_dot_set;                   // 水平距离设定
    float base_length;                        // 基础腿长
    float leg_length_set[2];                  // 双腿长度设置
    float foot_distance_set;                  // 前进距离设定

    //LQR输出
    float K_balance_T[2];
    float K_coordinate_T;
    float K_stand_T[2];
    float K_roll_T[2];

    float Wheel_motor_T[2];

    // 控制器
    float F[2];
    float Tp[2];

    float T[4];       // 关节电机输出力矩
    float T_wheel[2]; // 轮毂电机输出力矩

}chassis_data;


struct supervisor_t
{
    //node 
    WbNodeRef wheel_motor[2];
    WbNodeRef leg_motor[4];
    
    WbFieldRef wheel_motor_pos[2];
    WbFieldRef wheel_motor_spd[2];
    
    WbFieldRef leg_motor_pos[4];
    WbFieldRef leg_motor_spd[4];
    
}supervisor;




//function
void chassis_init();
void chassis_mode_set();
void chassis_feedback_update();
void chassis_control_set();
void chassis_control_loop();
void chassis_log_output();

void scan_leg();
void C_MATLAB_init();

#endif
