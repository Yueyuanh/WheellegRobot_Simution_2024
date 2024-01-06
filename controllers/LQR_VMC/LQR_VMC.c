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
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <webots/supervisor.h>
#include <webots/position_sensor.h>
//import controllers
#include <pid.h>
#include <LQR_VMC.h>
#include <VMC.h>
/*
 * You may want to add macros here.
 */
#define TIME_STEP 16




/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
 
 
int main(int argc, char **argv) {


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

  
    //获取陀螺仪数据
    chassis_tag.IMU  = wb_robot_get_device("imu");
    chassis_tag.GYRO = wb_robot_get_device("gyro");
     
    wb_inertial_unit_enable(chassis_tag.IMU,1);
    wb_gyro_enable(chassis_tag.GYRO,1);
     
  
     //获取驱动电机数据
    chassis_tag.rightWheel = wb_robot_get_device("rightWheel_motor");
    chassis_tag.leftWheel  = wb_robot_get_device("leftWheel_motor");
     
    wb_motor_enable_torque_feedback(chassis_tag.rightWheel,1);
    wb_motor_enable_torque_feedback(chassis_tag.leftWheel,1);
    


    wb_motor_set_position(chassis_tag.rightWheel,INFINITY);
    wb_motor_set_position(chassis_tag.leftWheel,INFINITY);
    
    wb_motor_set_velocity(chassis_tag.rightWheel,0);
    wb_motor_set_velocity(chassis_tag.leftWheel,0);
    
    //关机电机
    chassis_tag.rfLeg   = wb_robot_get_device("RF_motor");
    chassis_tag.rbLeg   = wb_robot_get_device("RB_motor");
    chassis_tag.lfLeg   = wb_robot_get_device("LF_motor");
    chassis_tag.lbLeg   = wb_robot_get_device("LB_motor");
     
    wb_motor_enable_torque_feedback(chassis_tag.rfLeg,1);
    wb_motor_enable_torque_feedback(chassis_tag.rbLeg,1);
    wb_motor_enable_torque_feedback(chassis_tag.lfLeg,1);
    wb_motor_enable_torque_feedback(chassis_tag.lbLeg,1);
    

    wb_motor_set_position(chassis_tag.rfLeg,INFINITY);
    wb_motor_set_position(chassis_tag.rbLeg,INFINITY);
    wb_motor_set_position(chassis_tag.lfLeg,INFINITY);
    wb_motor_set_position(chassis_tag.lbLeg,INFINITY);
    
    wb_motor_set_velocity(chassis_tag.rfLeg,0);
    wb_motor_set_velocity(chassis_tag.rbLeg,0);
    wb_motor_set_velocity(chassis_tag.lfLeg,0);
    wb_motor_set_velocity(chassis_tag.lbLeg,0);
     
    //supervisor
    supervisor.wheel_motor[0]=wb_supervisor_node_get_from_def("R_WHEEL_MOTOR");
    supervisor.wheel_motor[1]=wb_supervisor_node_get_from_def("L_WHEEL_MOTOR");
    
    supervisor.leg_motor[0]=wb_supervisor_node_get_from_def("RF_LEG_MOTOR");
    supervisor.leg_motor[1]=wb_supervisor_node_get_from_def("RB_LEG_MOTOR");
    supervisor.leg_motor[2]=wb_supervisor_node_get_from_def("LF_LEG_MOTOR");
    supervisor.leg_motor[3]=wb_supervisor_node_get_from_def("LB_LEG_MOTOR");
    
    supervisor.wheel_motor_pos[0]=wb_supervisor_node_get_field(supervisor.wheel_motor[0],"position");
    supervisor.wheel_motor_pos[1]=wb_supervisor_node_get_field(supervisor.wheel_motor[1],"position");
 
    // supervisor.wheel_motor_spd[0]=wb_supervisor_node_get_field(supervisor.wheel_motor[0],"velocity");
    // supervisor.wheel_motor_spd[1]=wb_supervisor_node_get_field(supervisor.wheel_motor[1],"velocity");

    supervisor.leg_motor_pos[0]=wb_supervisor_node_get_field(supervisor.leg_motor[0],"position");
    supervisor.leg_motor_pos[1]=wb_supervisor_node_get_field(supervisor.leg_motor[1],"position");
    supervisor.leg_motor_pos[2]=wb_supervisor_node_get_field(supervisor.leg_motor[2],"position");
    supervisor.leg_motor_pos[3]=wb_supervisor_node_get_field(supervisor.leg_motor[3],"position");

    // supervisor.leg_motor_spd[0]=wb_supervisor_node_get_field(supervisor.leg_motor[0],"velocity");
    // supervisor.leg_motor_spd[1]=wb_supervisor_node_get_field(supervisor.leg_motor[1],"velocity");
    // supervisor.leg_motor_spd[2]=wb_supervisor_node_get_field(supervisor.leg_motor[2],"velocity");
    // supervisor.leg_motor_spd[3]=wb_supervisor_node_get_field(supervisor.leg_motor[3],"velocity");

    VMC_init();
    
    

}

void chassis_mode_set()
{


}

void chassis_feedback_update()
{
    chassis_data.roll = wb_inertial_unit_get_roll_pitch_yaw(chassis_tag.IMU)[0];
    chassis_data.pitch=-wb_inertial_unit_get_roll_pitch_yaw(chassis_tag.IMU)[1];     
    chassis_data.yaw  =-wb_inertial_unit_get_roll_pitch_yaw(chassis_tag.IMU)[2];

    chassis_data.gyro_roll =wb_gyro_get_values(chassis_tag.GYRO)[0];
    chassis_data.gyro_pitch=wb_gyro_get_values(chassis_tag.GYRO)[1];
    chassis_data.gyro_yaw  =wb_gyro_get_values(chassis_tag.GYRO)[2];
    
    //wheel
    //Torque
    chassis_data.wheel_Torque[0]=wb_motor_get_torque_feedback(chassis_tag.rightWheel);
    chassis_data.wheel_Torque[1]=wb_motor_get_torque_feedback(chassis_tag.leftWheel);
     
    //speed
    // chassis_data.wheel_speed[0]=wb_supervisor_field_get_sf_float(supervisor.wheel_motor_spd[0]);
    // chassis_data.wheel_speed[1]=wb_supervisor_field_get_sf_float(supervisor.wheel_motor_spd[1]);
    
    //position
    chassis_data.wheel_position[0]=wb_supervisor_field_get_sf_float(supervisor.wheel_motor_pos[0]);
    chassis_data.wheel_position[1]=wb_supervisor_field_get_sf_float(supervisor.wheel_motor_pos[1]);
     
    //leg motor
    //Torque
    chassis_data.leg_Torque[0]=wb_motor_get_torque_feedback(chassis_tag.rfLeg);
    chassis_data.leg_Torque[1]=wb_motor_get_torque_feedback(chassis_tag.rbLeg);
    chassis_data.leg_Torque[2]=wb_motor_get_torque_feedback(chassis_tag.lfLeg);
    chassis_data.leg_Torque[3]=wb_motor_get_torque_feedback(chassis_tag.lbLeg);
    
    //speed
    // chassis_data.leg_speed[0]=wb_supervisor_field_get_sf_float(supervisor.leg_motor_spd[0]);
    // chassis_data.leg_speed[1]=wb_supervisor_field_get_sf_float(supervisor.leg_motor_spd[1]);
    // chassis_data.leg_speed[2]=wb_supervisor_field_get_sf_float(supervisor.leg_motor_spd[2]);
    // chassis_data.leg_speed[3]=wb_supervisor_field_get_sf_float(supervisor.leg_motor_spd[3]);
    
    //position 
    chassis_data.leg_position[0]=wb_supervisor_field_get_sf_float(supervisor.leg_motor_pos[0]);
    chassis_data.leg_position[1]=wb_supervisor_field_get_sf_float(supervisor.leg_motor_pos[1]);
    chassis_data.leg_position[2]=wb_supervisor_field_get_sf_float(supervisor.leg_motor_pos[2]);
    chassis_data.leg_position[3]=wb_supervisor_field_get_sf_float(supervisor.leg_motor_pos[3]);

    //VMC数据更新
    VMC_LEG_R.Phi4=pi-(chassis_data.leg_position[0]+pi/2);
    VMC_LEG_R.Phi1=-chassis_data.leg_position[1]+pi/2;
    VMC_LEG_L.Phi4=pi-(chassis_data.leg_position[2]+pi/2);
    VMC_LEG_L.Phi1=-chassis_data.leg_position[3]+pi/2;
}

void chassis_control_set()
{
   // wb_motor_set_torque(chassis_tag.rfLeg, 1);
   // wb_motor_set_torque(chassis_tag.rbLeg,-1);
   // wb_motor_set_torque(chassis_tag.lfLeg, 1);
   // wb_motor_set_torque(chassis_tag.lbLeg,-1);
}


void chassis_control_loop()
{
    //VMC calc
   VMC_calc(30,0,0,0);
    
    
   //give Torque
   wb_motor_set_torque(chassis_tag.rfLeg,VMC_LEG_R.T[0]);
   wb_motor_set_torque(chassis_tag.rbLeg,VMC_LEG_R.T[1]);
   wb_motor_set_torque(chassis_tag.lfLeg,VMC_LEG_L.T[0]);
   wb_motor_set_torque(chassis_tag.lbLeg,VMC_LEG_L.T[1]);

}

void chassis_log_output()
{
    //电机数据测试
    //printf("Tr: %f  Tl :%f ",chassis_data.wheel_Torque[0],chassis_data.wheel_Torque[1]);
    //printf("p1: %f  p2 :%f \n",chassis_data.leg_position[0],chassis_data.leg_position[1]); 
    
    // printf("RF: %f,RB: %f \n",VMC_LEG_R.T[0],VMC_LEG_R.T[1]);  
    // printf("LF: %f,LB: %f \n",VMC_LEG_L.T[0],VMC_LEG_L.T[1]);
    
    
    
    
    
    char value_str[32];
    sprintf(value_str,"%f",VMC_LEG_R.Phi2*rad2angle);
    wb_supervisor_set_label(0,value_str,0,0,0.1,0xff0000,0,"Arial");

    //VMC测试
     printf("Phi1: %f,Phi4: %f \n",VMC_LEG_R.Phi1*rad2angle,VMC_LEG_R.Phi4*rad2angle);  
     printf("Phi2: %f,Phi3: %f \n",VMC_LEG_R.Phi2*rad2angle,VMC_LEG_R.Phi3*rad2angle);    
    
     printf("L_BD: %f \n",VMC_LEG_R.L_BD);  
     printf("xD_xB: %f,yD_yB: %f \n",VMC_LEG_R.xD_xB,VMC_LEG_R.yD_yB);
     
     
     printf("xC: %f,yC: %f \n",VMC_LEG_R.xC,VMC_LEG_R.yC);  
     printf("L0: %f,Phi0: %f \n",VMC_LEG_R.L0,VMC_LEG_R.Phi0*rad2angle);
     
     printf("J_RF: %f,J_RB: %f \n",VMC_LEG_R.J[0][0],VMC_LEG_R.J[0][1]);  
     //printf("LF: %f,LB: %f \n",VMC_LEG_L.J[1][0],VMC_LEG_L.J[1][1]);
     
     printf("T1: %f,T2: %f \n",VMC_LEG_R.T[0],VMC_LEG_R.T[1]);
     
    //LQR测试
    
    
    printf("\n");
}
/*****************************************CHASSIS**********************************************/