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
 //import webots sensor
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/Gyro.h>
#include <webots/Accelerometer.h>
#include <webots/Compass.h>
#include <webots/Inertial_unit.h>
#include <webots/Keyboard.h>
#include <webots/Camera.h>

//import controllers

/*
 * You may want to add macros here.
 */
#define TIME_STEP 16


typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float max_out;  
    float max_iout; 

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3]; 
    float error[3]; 

} pid_type_def;


//output
float balance_motor_current,speed_motor_current,balance_Kp;
float speed_error,speed_error_lowout,speed_error_lowout_last,lowout_Ka;
float turn_motor_current;
float rightMotor,leftMotor;
float roll_LPF,roll_LPF_last;
//
float wheel_speed;
double pitch,yaw,roll;
double accel_x,accel_y,accel_z;
float  go_back,left_right;
float  yawSet;
float  leg_length_R=1;
float  leg_length_L=1;
float  wheelBuff;
float  rollBuff;
float  rollError;
float  yawError;
int    zero_force;

//funtions
void  PID_init(pid_type_def *pid,const float PID[3],float,float);
float PID_calc(pid_type_def *pid,float ref,float set);
void Keyboard_control();
float rad_loop(float,float,float);

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
     
   // //获取陀螺仪数据
     WbDeviceTag IMU  = wb_robot_get_device("imu");
     WbDeviceTag GYRO = wb_robot_get_device("gyro");
     ////WbDeviceTag ACCEL= wb_robot_get_device("accel");
     //WbDeviceTag CMP  = wb_robot_get_device("compass");
     
     wb_inertial_unit_enable(IMU,1);
     wb_gyro_enable(GYRO,1);
     
      WbDeviceTag camera = wb_robot_get_device("camera");
      wb_camera_enable(camera, TIME_STEP);
     //update image
      (void)wb_camera_get_image(camera);

  
     //获取电机数据
     WbDeviceTag rightWheel = wb_robot_get_device("rightWheel_motor");
     WbDeviceTag leftWheel  = wb_robot_get_device("leftWheel_motor");
     WbDeviceTag rfLeg   = wb_robot_get_device("RF_motor");
     WbDeviceTag rbLeg   = wb_robot_get_device("RB_motor");
     WbDeviceTag lfLeg   = wb_robot_get_device("LF_motor");
     WbDeviceTag lbLeg   = wb_robot_get_device("LB_motor");
     

     wb_motor_set_position(rightWheel,INFINITY);
     wb_motor_set_position(leftWheel ,INFINITY);
     
     wb_motor_set_position(rfLeg,1);
     wb_motor_set_position(rbLeg,-1);
     wb_motor_set_position(lfLeg,1);
     wb_motor_set_position(lbLeg,-1);
     
     
     wb_motor_set_velocity(rightWheel,0);
     wb_motor_set_velocity(leftWheel ,0);
     
   
   
   
     
     lowout_Ka=0.7;
     
     //controllers init
    pid_type_def SPEED_PID,BALANCE_PID,TURN_PID,ROLL_PID;
    
    
    const float speed_pid[3]  ={0.008,0,0.0002};
    const float balance_pid[3]={350,1,0};
    const float turn_pid[3]   ={20,0,1};
    const float roll_pid[3]   ={3,0,5};
    
    
    PID_init(&SPEED_PID,speed_pid,0.45,0.1);
    PID_init(&BALANCE_PID,balance_pid,100,50);
    PID_init(&TURN_PID,turn_pid,100,50);
    PID_init(&ROLL_PID,roll_pid,0.5,0.4);

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */
     
     Keyboard_control();
     
     //leg length
     if(leg_length_R<0.2) leg_length_R=0.2;
     if(leg_length_L<0.2) leg_length_L=0.2;
     wb_motor_set_position(rfLeg,leg_length_R+wheelBuff);
     wb_motor_set_position(rbLeg,-leg_length_R+wheelBuff);
     wb_motor_set_position(lfLeg,leg_length_L+wheelBuff);
     wb_motor_set_position(lbLeg,-leg_length_L+wheelBuff);
     
     
     // //get data
      roll  =wb_inertial_unit_get_roll_pitch_yaw(IMU)[0];
      pitch=-wb_inertial_unit_get_roll_pitch_yaw(IMU)[1];
      yaw =-wb_inertial_unit_get_roll_pitch_yaw(IMU)[2];
    
      accel_x=wb_gyro_get_values(GYRO)[0];
      accel_y=wb_gyro_get_values(GYRO)[1];
      accel_z=wb_gyro_get_values(GYRO)[2];
    
    
    
    
     // //speed loop   : PI control
      speed_error=(wb_motor_get_velocity(rightWheel)+wb_motor_get_velocity(leftWheel));
          // //Lowout Fliter
      speed_error_lowout=(1-lowout_Ka)*speed_error+lowout_Ka*speed_error_lowout_last;
      speed_error_lowout_last=speed_error_lowout;
      speed_motor_current=PID_calc(&SPEED_PID,speed_error_lowout,go_back);
     
      // //balance loop : PD control  3100,0,0.9
      balance_motor_current=PID_calc(&BALANCE_PID,pitch,0.02);
      // balance_motor_current=PID_calc(&BALANCE_PID,pitch,speed_motor_current);
      
      //roll_loop
      printf("roll: %f,R:%f  L:%f \n",rollError,leg_length_R,leg_length_L);
      rollError=0.0013-roll;
      
      if(rollError>0)
      {
          //leg_length_L=1+PID_calc(&ROLL_PID,roll,0.0013);
          leg_length_L=1+4.5*-rollError;
      } 
      else{
         // leg_length_R=1+PID_calc(&ROLL_PID,roll,0.0013);
          leg_length_R=1+4.5*rollError;
      }
      if(leg_length_R>1.2&&leg_length_R<1.5)
      {
          leg_length_L-=leg_length_R-1.2;
      }
      if(leg_length_L>1.2&&leg_length_L<1.5)
      {
          leg_length_R-=leg_length_L-1.2;
      }
      
      // //turn loop   ww :P control
      yawSet+=left_right;
      printf("yawError:%f  \n",yawSet-yaw);
      yawSet=rad_loop(yawSet,-3.1415926,3.1415926);
      yawError=yawSet-yaw;
      if(yawError>6)
      {
         
      }
      turn_motor_current=PID_calc(&TURN_PID,yaw,yawSet);
      
     
     
     
      //close some loop
      //balance_motor_current=0;
      //speed_motor_current=0;
      //turn_motor_current=0;
     
     
     
      // //current_output

      rightMotor=balance_motor_current+turn_motor_current+left_right;
      leftMotor =balance_motor_current-turn_motor_current-left_right;
      wheelBuff=speed_motor_current;
     
     
  
      // //log out
      printf("R:%f  L:%f  ",rightMotor,leftMotor);
      printf("B:%f  S:%f  ",balance_motor_current,speed_motor_current);
      printf("P:%f  ",pitch);
      printf("Y:%f  ",yaw);
      printf("speed_err:%f  ",speed_error);
      printf("yawset:%f  ",yawSet);
      //printf("roll:%f  ",roll_error);
      // //printf("accel_x%f",accel_x);
      // //printf("T:%f",turn_motor_current);
      // //printf("Key:%d",wb_keyboard_get_key());
      printf("\n");
     
     
      //send current
      if(rightMotor>100)  rightMotor=100;
      if(rightMotor<-100) rightMotor=-100;
      if(leftMotor>100)   leftMotor=100;
      if(leftMotor<-100)  leftMotor=-100;
      if(!zero_force){
      wb_motor_set_velocity(rightWheel,rightMotor);
      wb_motor_set_velocity(leftWheel ,leftMotor);
     }
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}


//mrcio controller
void PID_init(pid_type_def *pid,const float PID[3],float max_out,float max_iout)
{
    pid->Kp=PID[0];
    pid->Ki=PID[1];
    pid->Kd=PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;


}
float PID_calc(pid_type_def *pid, float ref, float set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    
    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    if(pid->Iout>pid->max_iout)  pid->Iout=pid->max_iout;
    if(pid->Iout<-pid->max_iout) pid->Iout=-pid->max_iout;
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    if(pid->out>pid->max_out)  pid->out=pid->max_out;
    if(pid->out<-pid->max_out) pid->out=-pid->max_out;
   
    return pid->out;
}

//keyboard control
void Keyboard_control()
{
    int Key=wb_keyboard_get_key();
    printf("key:%d ",Key);
    switch(Key)
    {
      case 87: go_back=-65;break;
      case 83: go_back=65;break;
      case 65: left_right=-0.05;break;
      case 68: left_right=0.05;break;
      case 81: leg_length_R=1.2,leg_length_L=0.8;break; //q
      case 69: leg_length_R=0.8,leg_length_L=1.2;break; //e
      case 32: zero_force=1;break;          //空格
     case 315: leg_length_R=0.8,leg_length_L=0.8;break; //上
     case 317: leg_length_L=1.2,leg_length_R=1.2;break; //下
     case 314: wheelBuff=0.2;break; //右
     case 316: wheelBuff=-0.2;break; //左
     default: go_back=0,left_right=0,zero_force=0;
    }

     // printf("%d,%f\n",Key,go_back);
}



//循环限幅函数   超过之后加减区间实现循环
float rad_loop(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        float len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        float len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}