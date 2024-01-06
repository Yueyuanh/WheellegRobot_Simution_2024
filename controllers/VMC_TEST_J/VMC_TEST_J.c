
/*
 * File:          BanacedInfantry.c
 * Date:          13/8/2023
 * Description:
 * Author:        YueYuanhaoo
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "engine.h"

#include <webots/supervisor.h>

// import controllers
#include <VMC.h>
#include <VMC_TEST_J.h>
#include <pid.h>
#include <plot.h>
#include <LQR.h>
#include <Keyboard_control.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 16
/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */

int main(int argc, char **argv)
{

  /* necessary to initialize webots stuff */
  wb_robot_init();
  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
  // //enable keyboard
  wb_keyboard_enable(TIME_STEP);

    chassis_init();
    plotInit();
    Keyboard_init();
    /* main loop

     * Perform simulation steps of TIME_STEP milliseconds
     * and leave the loop when the simulation is over
     */
    while (wb_robot_step(TIME_STEP) != -1)
    {

      chassis_mode_set();
      chassis_feedback_update();
      chassis_control_set();
      chassis_control_loop();
      chassis_log_output();

  
 

    }

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}

/****************************************CHASSIS***********************************************/

void chassis_init()
{

  // 获取陀螺仪数据
  chassis_tag.IMU = wb_robot_get_device("imu");
  chassis_tag.GYRO = wb_robot_get_device("gyro");
  chassis_tag.ACCEL = wb_robot_get_device("accel");

  wb_inertial_unit_enable(chassis_tag.IMU, 1);
  wb_gyro_enable(chassis_tag.GYRO, 1);
  wb_accelerometer_enable(chassis_tag.ACCEL, 1);

  // 获取驱动电机数据
  chassis_tag.rightWheel = wb_robot_get_device("rightWheel_motor");
  chassis_tag.leftWheel = wb_robot_get_device("leftWheel_motor");

  wb_motor_enable_torque_feedback(chassis_tag.rightWheel,1);
  wb_motor_enable_torque_feedback(chassis_tag.leftWheel,1);

  wb_motor_set_position(chassis_tag.rightWheel, INFINITY);
  wb_motor_set_position(chassis_tag.leftWheel, INFINITY);

  wb_motor_set_velocity(chassis_tag.rightWheel, 0);
  wb_motor_set_velocity(chassis_tag.leftWheel, 0);

  // 关节电机
  chassis_tag.rfLeg = wb_robot_get_device("RF_motor");
  chassis_tag.rbLeg = wb_robot_get_device("RB_motor");
  chassis_tag.lfLeg = wb_robot_get_device("LF_motor");
  chassis_tag.lbLeg = wb_robot_get_device("LB_motor");

  wb_motor_enable_torque_feedback(chassis_tag.rfLeg,1);
  wb_motor_enable_torque_feedback(chassis_tag.rbLeg,1);
  wb_motor_enable_torque_feedback(chassis_tag.lfLeg,1);
  wb_motor_enable_torque_feedback(chassis_tag.lbLeg,1);

  wb_motor_set_position(chassis_tag.rfLeg, INFINITY);
  wb_motor_set_position(chassis_tag.rbLeg, INFINITY);
  wb_motor_set_position(chassis_tag.lfLeg, INFINITY);
  wb_motor_set_position(chassis_tag.lbLeg, INFINITY);

  wb_motor_set_position(chassis_tag.rfLeg, 1);
  wb_motor_set_position(chassis_tag.rbLeg, -1);
  wb_motor_set_position(chassis_tag.lfLeg, 1);
  wb_motor_set_position(chassis_tag.lbLeg, -1);

  // supervisor
  supervisor.wheel_motor[0] = wb_supervisor_node_get_from_def("R_WHEEL_MOTOR");
  supervisor.wheel_motor[1] = wb_supervisor_node_get_from_def("L_WHEEL_MOTOR");

  supervisor.leg_motor[0] = wb_supervisor_node_get_from_def("RF_LEG_MOTOR");
  supervisor.leg_motor[1] = wb_supervisor_node_get_from_def("RB_LEG_MOTOR");
  supervisor.leg_motor[2] = wb_supervisor_node_get_from_def("LF_LEG_MOTOR");
  supervisor.leg_motor[3] = wb_supervisor_node_get_from_def("LB_LEG_MOTOR");

  supervisor.wheel_motor_pos[0] = wb_supervisor_node_get_field(supervisor.wheel_motor[0], "position");
  supervisor.wheel_motor_pos[1] = wb_supervisor_node_get_field(supervisor.wheel_motor[1], "position");


  supervisor.leg_motor_pos[0] = wb_supervisor_node_get_field(supervisor.leg_motor[0], "position");
  supervisor.leg_motor_pos[1] = wb_supervisor_node_get_field(supervisor.leg_motor[1], "position");
  supervisor.leg_motor_pos[2] = wb_supervisor_node_get_field(supervisor.leg_motor[2], "position");
  supervisor.leg_motor_pos[3] = wb_supervisor_node_get_field(supervisor.leg_motor[3], "position");

  //LQR init
  LQR_init();
  
  // VMC init
  VMC_init();
  differentiator_init(&chassis_data.wheel_speed_r);
  differentiator_init(&chassis_data.wheel_speed_l);
  Kalman_fir_init(&chassis_data.kalman_speed,10,10);
  Kalman_sec_init(&chassis_data.kalman_distance, 0.08, 0.08);
}

void chassis_mode_set()
{


}

void chassis_feedback_update()
{
  //keyboard
  Keyboard_control();

  //data update
  chassis_data.roll = wb_inertial_unit_get_roll_pitch_yaw(chassis_tag.IMU)[0];
  chassis_data.pitch = wb_inertial_unit_get_roll_pitch_yaw(chassis_tag.IMU)[1];
  chassis_data.yaw = -wb_inertial_unit_get_roll_pitch_yaw(chassis_tag.IMU)[2];

  chassis_data.gyro_roll = wb_gyro_get_values(chassis_tag.GYRO)[0];
  chassis_data.gyro_pitch = wb_gyro_get_values(chassis_tag.GYRO)[2];
  chassis_data.gyro_yaw = wb_gyro_get_values(chassis_tag.GYRO)[1];

  chassis_data.accel_x = wb_accelerometer_get_values(chassis_tag.ACCEL)[0];
  chassis_data.accel_y = wb_accelerometer_get_values(chassis_tag.ACCEL)[2];
  chassis_data.accel_z = wb_accelerometer_get_values(chassis_tag.ACCEL)[1];

  if(chassis_data.yaw-chassis_data.yaw_last > pi)
  {
    chassis_data.yaw_count--;
  }
  else if(chassis_data.yaw-chassis_data.yaw_last < -pi)
  {
    chassis_data.yaw_count++;
  }
  chassis_data.yaw_sum = 2 * pi * chassis_data.yaw_count + chassis_data.yaw;
  chassis_data.yaw_last = chassis_data.yaw;

  

  // wheel

  //Torque
  chassis_data.wheel_Torque[0]=wb_motor_get_torque_feedback(chassis_tag.rightWheel);
  chassis_data.wheel_Torque[1]=wb_motor_get_torque_feedback(chassis_tag.leftWheel);

  // position
  chassis_data.wheel_position[0] = -wb_supervisor_field_get_sf_float(supervisor.wheel_motor_pos[0])*0.06;
  chassis_data.wheel_position[1] = -wb_supervisor_field_get_sf_float(supervisor.wheel_motor_pos[1])*0.06;
  
  // speed
  chassis_data.wheel_speed[0] = differentiator(&chassis_data.wheel_speed_r,40,0.032,chassis_data.wheel_position[0]);
  chassis_data.wheel_speed[1] = differentiator(&chassis_data.wheel_speed_l,40,0.032,chassis_data.wheel_position[1]);

  // const float lpf_const = 0.9;
  // chassis_data.wheel_speed_lpf[0] = lpf_const * chassis_data.wheel_speed[0] + (1 - lpf_const) * chassis_data.wheel_speed_last[0];
  // chassis_data.wheel_speed_lpf[1] = lpf_const * chassis_data.wheel_speed[1] + (1 - lpf_const) * chassis_data.wheel_speed_last[1];

  // chassis_data.wheel_speed_last[0] = chassis_data.wheel_speed[0];
  // chassis_data.wheel_speed_last[1] = chassis_data.wheel_speed[1];


  // wheel turns
  chassis_data.wheel_turns[0] = chassis_data.wheel_position[0] / pi;
  chassis_data.wheel_turns[1] = chassis_data.wheel_position[1] / pi;

  chassis_data.wheel_position_last[0] = chassis_data.wheel_position[0];
  chassis_data.wheel_position_last[1] = chassis_data.wheel_position[1];

  // foot distance
  chassis_data.foot_distance = (chassis_data.wheel_position[0] + chassis_data.wheel_position[1]) / 2;//这也是线距离啊，不是角度

  // foot speed
  chassis_data.foot_speed = (chassis_data.wheel_speed[0] + chassis_data.wheel_speed[1]) / 2;//注意电机速度方向,注意是线速度

  const float foot_speed_lpf_const = 0.95;
  chassis_data.foot_speed_lpf = (1-foot_speed_lpf_const) * chassis_data.foot_speed + foot_speed_lpf_const * chassis_data.foot_speed_last;
  chassis_data.foot_speed_last = chassis_data.foot_speed_lpf;
  printf("speed_lpf:%f\n", chassis_data.foot_speed_lpf);

  //KalmanFilter test
  //chassis_data.foot_speed_kalman = KalmanFilter_fir(&chassis_data.kalman_speed, chassis_data.foot_speed);

  
  KalmanFilter_sec(&chassis_data.kalman_distance, chassis_data.foot_distance, chassis_data.accel_x*cos(chassis_data.pitch));
  chassis_data.foot_speed_kalman = chassis_data.kalman_distance.out.x;


  // leg motor
  // Torque
  chassis_data.leg_Torque[0]=wb_motor_get_torque_feedback(chassis_tag.rfLeg);
  chassis_data.leg_Torque[1]=wb_motor_get_torque_feedback(chassis_tag.rbLeg);
  chassis_data.leg_Torque[2]=wb_motor_get_torque_feedback(chassis_tag.lfLeg);
  chassis_data.leg_Torque[3]=wb_motor_get_torque_feedback(chassis_tag.lbLeg);

  // position
  chassis_data.leg_position[0] = wb_supervisor_field_get_sf_float(supervisor.leg_motor_pos[0]);
  chassis_data.leg_position[1] = wb_supervisor_field_get_sf_float(supervisor.leg_motor_pos[1]);
  chassis_data.leg_position[2] = wb_supervisor_field_get_sf_float(supervisor.leg_motor_pos[2]);
  chassis_data.leg_position[3] = wb_supervisor_field_get_sf_float(supervisor.leg_motor_pos[3]);

 // speed
  chassis_data.leg_speed[0] = differentiator(&VMC_LEG_R.leg_motor_speed[0],40,0.032,chassis_data.leg_position[0]);
  chassis_data.leg_speed[1] = differentiator(&VMC_LEG_R.leg_motor_speed[1],40,0.032,chassis_data.leg_position[1]);
  chassis_data.leg_speed[2] = differentiator(&VMC_LEG_R.leg_motor_speed[2],40,0.032,chassis_data.leg_position[2]);
  chassis_data.leg_speed[3] = differentiator(&VMC_LEG_R.leg_motor_speed[3],40,0.032,chassis_data.leg_position[3]);

  // VMC数据更新
  VMC_LEG_R.Phi4 = pi - (chassis_data.leg_position[0] + pi / 2);
  VMC_LEG_R.Phi1 = -chassis_data.leg_position[1] + pi / 2;
  VMC_LEG_L.Phi4 = pi - (chassis_data.leg_position[2] + pi / 2);
  VMC_LEG_L.Phi1 = -chassis_data.leg_position[3] + pi / 2;
  //printf("%f %f\n\n", VMC_LEG_R.Phi2, VMC_LEG_R.Phi3);

  // 正运动学解算出极坐标角度
  Forward_kinematics_R(VMC_LEG_R.Phi1, VMC_LEG_R.Phi4, chassis_data.leg_speed[1],chassis_data.leg_speed[0], 0, 0);
  Forward_kinematics_L(VMC_LEG_L.Phi1, VMC_LEG_L.Phi4, chassis_data.leg_speed[2],chassis_data.leg_speed[3], 0, 0);

  //leg length
  chassis_data.leg_length[0] = VMC_LEG_R.L0;
  chassis_data.leg_length[1] = VMC_LEG_L.L0;

  // leg angle
  chassis_data.leg_angle[0] = VMC_LEG_R.Phi0 - chassis_data.pitch;
  chassis_data.leg_angle[1] = VMC_LEG_L.Phi0 - chassis_data.pitch;

  chassis_data.leg_gyro[0] = VMC_LEG_R.Phi0_gyro - chassis_data.gyro_pitch;
  chassis_data.leg_gyro[1] = VMC_LEG_L.Phi0_gyro - chassis_data.gyro_pitch;

  //观测器
  VMC_LEG_R.L0_dot = differentiator(&VMC_LEG_R.d_L0,40,0.032,VMC_LEG_R.L0);
  VMC_LEG_L.L0_dot = differentiator(&VMC_LEG_L.d_L0,40,0.032,VMC_LEG_L.L0);

  //是否悬空标志位判断
  if(VMC_LEG_R.F_foot<0.1&&VMC_LEG_L.F_foot<0.1)
  {
    chassis_data.free_flag = 1;
  }
  else
  {
    chassis_data.free_flag = 0;
  }

  //matlab plot
  plotFile3("IMU",  wb_robot_get_time(), chassis_data.yaw, chassis_data.pitch, chassis_data.roll);
  plotFile3("GYRO", wb_robot_get_time(), chassis_data.gyro_yaw, chassis_data.gyro_pitch, chassis_data.gyro_roll);
  plotFile3("ACCEL", wb_robot_get_time(), chassis_data.accel_x, chassis_data.accel_y, chassis_data.accel_z);

  plotFile3("LQR_1", wb_robot_get_time(), pi/2-chassis_data.leg_angle[0], chassis_data.leg_gyro[0],chassis_data.foot_distance);
  plotFile3("LQR_2", wb_robot_get_time(), chassis_data.foot_speed_lpf, chassis_data.pitch, chassis_data.gyro_pitch);

  if (wb_robot_get_time() > 0)
  {
    plotFile("Rleg_L0", wb_robot_get_time(), VMC_LEG_R.L0);
    plotFile("Rleg_d_L0", wb_robot_get_time(),VMC_LEG_R.L0_dot);

    plotFile("Lleg_L0", wb_robot_get_time(), VMC_LEG_L.L0);
    plotFile("Lleg_d_L0", wb_robot_get_time(),VMC_LEG_L.L0_dot);
  }

}

void chassis_control_set()
{


}



void chassis_control_loop()
{
  fp32 Tp_R,Tp_L;
  fp32 F_R, F_L;
//LQR
  LQR_calc();

  Tp_R = chassis_data.K_balance_T[0] - chassis_data.K_coordinate_T;
  Tp_L = chassis_data.K_balance_T[1] + chassis_data.K_coordinate_T;

  F_R = -chassis_data.K_roll_T[0] + chassis_data.K_stand_T[0]+chassis_data.leg_stand_F;
  F_L =  chassis_data.K_roll_T[1] + chassis_data.K_stand_T[1]+chassis_data.leg_stand_F;

//PID





  // VMC calc test
  VMC_calc(F_R,Tp_R,F_L,Tp_L);
  
 // VMC_calc(130,-1,130,-1);

  LimitMax(VMC_LEG_R.T[0], 20);//
  LimitMax(VMC_LEG_R.T[1], 20);
  LimitMax(VMC_LEG_L.T[0], 20);
  LimitMax(VMC_LEG_L.T[1], 20);
  
  LimitMax(chassis_data.Wheel_motor_T[0], 2);
  LimitMax(chassis_data.Wheel_motor_T[1], 2);


  //give current


   
    wb_motor_set_torque(chassis_tag.rfLeg,VMC_LEG_R.T[0]);
    wb_motor_set_torque(chassis_tag.rbLeg,VMC_LEG_R.T[1]);
    wb_motor_set_torque(chassis_tag.lfLeg,VMC_LEG_L.T[0]);
    wb_motor_set_torque(chassis_tag.lbLeg,VMC_LEG_L.T[1]);

    wb_motor_set_torque(chassis_tag.rightWheel,-chassis_data.Wheel_motor_T[0]);
    wb_motor_set_torque(chassis_tag.leftWheel ,-chassis_data.Wheel_motor_T[1]);

  //   wb_motor_set_torque(chassis_tag.rfLeg,0);
  //   wb_motor_set_torque(chassis_tag.rbLeg,0);
  //   wb_motor_set_torque(chassis_tag.lfLeg,0);
  //   wb_motor_set_torque(chassis_tag.lbLeg,0);

    // wb_motor_set_torque(chassis_tag.rightWheel,-0);
    // wb_motor_set_torque(chassis_tag.leftWheel ,-0);

  


    // wb_motor_set_torque(chassis_tag.rightWheel,0);
    // wb_motor_set_torque(chassis_tag.leftWheel ,0);

  plotFile3("LQR_OUT", wb_robot_get_time(), VMC_LEG_R.T[0], VMC_LEG_R.T[1], chassis_data.Wheel_motor_T[0]);


  plotFile("T", wb_robot_get_time(), VMC_LEG_R.T[0]);

  //printf("R_T:%f L_T:%f\n\n", chassis_data.Wheel_motor_T[0],chassis_data.Wheel_motor_T[1]);
  // test
  // scan_leg();
}



void chassis_log_output()
{


  // 电机数据测试
  // printf("Tr: %f  Tl :%f ",chassis_data.wheel_Torque[0],chassis_data.wheel_Torque[1]);
  // printf("p1: %f  p2 :%f \n",chassis_data.leg_position[0],chassis_data.leg_position[1]);

  // printf("RF: %f,RB: %f \n",VMC_LEG_R.T[0],VMC_LEG_R.T[1]);
  // printf("LF: %f,LB: %f \n",VMC_LEG_L.T[0],VMC_LEG_L.T[1]);

  char value_str[32];
  sprintf(value_str, "L0:%f", chassis_data.leg_length[0]);
  wb_supervisor_set_label(0, value_str, 0, 0, 0.1, 0xffff00, 0, "Arial");

  char value_str_2[32];
  sprintf(value_str_2, "Phi0:%f", VMC_LEG_R.Phi0);
  wb_supervisor_set_label(1, value_str_2, 100, 100, 0.1, 0xff0000, 0, "Arial");
  // VMC测试
  // printf("Phi1: %f,Phi4: %f \n", VMC_LEG_R.Phi1 * rad2angle, VMC_LEG_R.Phi4 * rad2angle);
  // printf("Phi2: %f,Phi3: %f \n", VMC_LEG_R.Phi2 * rad2angle, VMC_LEG_R.Phi3 * rad2angle);

  // printf("L_BD: %f \n", VMC_LEG_R.L_BD);

  // printf("xB: %f,yB: %f \n", VMC_LEG_R.xB, VMC_LEG_R.yB);
  // printf("xD: %f,yD: %f \n", VMC_LEG_R.xD, VMC_LEG_R.yD);

  // printf("L0: %f,Phi0: %f \n", VMC_LEG_R.L0, VMC_LEG_R.Phi0 * rad2angle);
  // printf("xC: %f,yC: %f \n", VMC_LEG_R.xC, VMC_LEG_R.yC);

  // printf("xC_set: %f,yC_set: %f \n", VMC_LEG_R.xC_set, VMC_LEG_R.yC_set);
  // printf("Phi1_set: %f,Phi4_set: %f \n", VMC_LEG_R.Phi1_set * rad2angle, VMC_LEG_R.Phi4_set * rad2angle);

  // printf("J_RF: %f,J_RB: %f \n",VMC_LEG_R.J[0][0],VMC_LEG_R.J[0][1]);
  // printf("LF: %f,LB: %f \n",VMC_LEG_L.J[1][0],VMC_LEG_L.J[1][1]);

  //电机输出扭矩测试
  // printf("T1: %f,T2: %f \n",VMC_LEG_R.T[0],VMC_LEG_R.T[1]);
  // printf("L0: %f,Phi0: %f \n",VMC_LEG_R.L0,VMC_LEG_R.Phi0);

  // printf("T1: %f,T2: %f \n",VMC_LEG_L.T[0],VMC_LEG_L.T[1]);
  // printf("L0: %f,Phi0: %f \n",VMC_LEG_L.L0,VMC_LEG_L.Phi0);

  //陀螺仪测试

  //printf("yaw:  %f\n,pitch: %f \n,roll: %f \n",chassis_data.yaw,chassis_data.pitch,chassis_data.roll);

  //驱动轮测试
  // printf("R_pos:%f  L_pos:%f \n", chassis_data.wheel_position[0], chassis_data.wheel_position[1]);
  // printf("R_Turns:%f  L_Turns:%f\n", chassis_data.wheel_turns[0], chassis_data.wheel_turns[1]);

  // LQR测试

  printf("\n");
}
/*****************************************CHASSIS**********************************************/

void scan_leg_movement()
{

  wb_robot_step(500);
  inverse_kinematics(0.3,pi/2+0.2);
  wb_motor_set_position(chassis_tag.rfLeg, VMC_LEG_R.Phi1_set);
  wb_motor_set_position(chassis_tag.rbLeg, VMC_LEG_R.Phi4_set);
  wb_motor_set_position(chassis_tag.lfLeg, VMC_LEG_R.Phi1_set);
  wb_motor_set_position(chassis_tag.lbLeg, VMC_LEG_R.Phi4_set);
  chassis_feedback_update();
  Forward_kinematics_L(VMC_LEG_L.Phi1, VMC_LEG_L.Phi4, 0, 0, 0, 0);
  printf("L0: %f,Phi0: %f \n",VMC_LEG_R.L0,VMC_LEG_R.Phi0);

  wb_robot_step(500);
  inverse_kinematics(0.4,pi/2+0.2);
  wb_motor_set_position(chassis_tag.rfLeg, VMC_LEG_R.Phi1_set);
  wb_motor_set_position(chassis_tag.rbLeg, VMC_LEG_R.Phi4_set);
  wb_motor_set_position(chassis_tag.lfLeg, VMC_LEG_R.Phi1_set);
  wb_motor_set_position(chassis_tag.lbLeg, VMC_LEG_R.Phi4_set);
  chassis_feedback_update();
  Forward_kinematics_L(VMC_LEG_L.Phi1, VMC_LEG_L.Phi4, 0, 0, 0, 0);
  printf("L0: %f,Phi0: %f \n",VMC_LEG_R.L0,VMC_LEG_R.Phi0);

  wb_robot_step(500);
  inverse_kinematics(0.4,pi/2-0.2);
  wb_motor_set_position(chassis_tag.rfLeg, VMC_LEG_R.Phi1_set);
  wb_motor_set_position(chassis_tag.rbLeg, VMC_LEG_R.Phi4_set);
  wb_motor_set_position(chassis_tag.lfLeg, VMC_LEG_R.Phi1_set);
  wb_motor_set_position(chassis_tag.lbLeg, VMC_LEG_R.Phi4_set);
  chassis_feedback_update();
  Forward_kinematics_L(VMC_LEG_L.Phi1, VMC_LEG_L.Phi4, 0, 0, 0, 0);
  printf("L0: %f,Phi0: %f \n",VMC_LEG_R.L0,VMC_LEG_R.Phi0);

  wb_robot_step(500);
  inverse_kinematics(0.3,pi/2-0.2);
  wb_motor_set_position(chassis_tag.rfLeg, VMC_LEG_R.Phi1_set);
  wb_motor_set_position(chassis_tag.rbLeg, VMC_LEG_R.Phi4_set);
  wb_motor_set_position(chassis_tag.lfLeg, VMC_LEG_R.Phi1_set);
  wb_motor_set_position(chassis_tag.lbLeg, VMC_LEG_R.Phi4_set);
  chassis_feedback_update();
  Forward_kinematics_L(VMC_LEG_L.Phi1, VMC_LEG_L.Phi4, 0, 0, 0, 0);
  printf("L0: %f,Phi0: %f \n",VMC_LEG_R.L0,VMC_LEG_R.Phi0);

}


