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
}chassis_tag;

struct chassis_data_t
{
     //观测器数据
     float yaw,pitch,roll;               //2 1 0
     float accel_x,accel_y,accel_z;      //0 1 2
     float gyro_yaw,gyro_pitch,gyro_roll;//
    
     float leg_angle_r,leg_angle_l;      //腿相对于竖直方向的角度
     float leg_length_r,leg_length_l;    //腿的长度
     
     float leg_Torque[4];                //关节电机反馈力矩
     double leg_position[4];              //关节电机反馈角度
     double leg_speed[4];                 //关节电机反馈转速
     
     float wheel_Torque[2];              //驱动轮电机反馈力矩
     double wheel_position[2];            //驱动轮电机反馈位置
     double wheel_speed[2];               //驱动轮电机反馈速度
     float wheel_X[2];                   //驱动轮位移
     float wheel_X_dot[2];               //驱动轮速度
     
     //期望值设置
     float yaw_set,pitch_set,roll_set;            //三轴角度设定
     float x_set,x_dot_set;                       //水平距离设定
     float leg_length_r_set,leg_length_l_set;     //双腿长度设置
     
     //控制器
     float F[2];
     float Tp[2];
     
     float T[4];      //关节电机输出力矩
     float T_wheel[2];//轮毂电机输出力矩

}chassis_data;


struct LQR_t
{
    
}LQR;


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


#endif
