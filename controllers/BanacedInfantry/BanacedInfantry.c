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
float balance_motor_current,speed_motor_current,position_motor_current;
float speed_error,speed_error_lowout,speed_error_lowout_last,lowout_Ka;
float turn_motor_current;
float rightMotor,leftMotor;

//
float wheel_speed;
double pitch,yaw,roll,yaw_sum,yaw_last,yaw_count,yaw_init,yaw_set;
double accel_x,accel_y,accel_z;
float  go_back,left_right;
float  pitch_angle_set;
float  position_count,position_set;
//
void  PID_init(pid_type_def *pid,const float PID[3],float,float);
float PID_calc(pid_type_def *pid,float ref,float set);
void Keyboard_control();
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
   //enable keyboard
     wb_keyboard_enable(TIME_STEP);
     
   //获取陀螺仪数据
     WbDeviceTag IMU  = wb_robot_get_device("imu");
     WbDeviceTag GYRO = wb_robot_get_device("gyro");
     // WbDeviceTag ACCEL= wb_robot_get_device("accel");
     // WbDeviceTag CMP  = wb_robot_get_device("compass");
     
     wb_inertial_unit_enable(IMU,1);
     wb_gyro_enable(GYRO,1);
     
     WbDeviceTag camera = wb_robot_get_device("camera");
     wb_camera_enable(camera, TIME_STEP);
     // update image
     (void)wb_camera_get_image(camera);

  
     
   //获取电机数据
     WbDeviceTag rightWheel = wb_robot_get_device("rightWheel_motor");
     WbDeviceTag leftWheel  = wb_robot_get_device("leftWheel_motor");
     
     wb_motor_set_position(rightWheel,INFINITY);
     wb_motor_set_position(leftWheel ,INFINITY);
   
     wb_motor_set_velocity(rightWheel,0);
     wb_motor_set_velocity(leftWheel ,0);
     
     //yaw_init  =wb_inertial_unit_get_roll_pitch_yaw(IMU)[2]+3.14159;
     
     lowout_Ka=0.95;
     
     //controllers init
    pid_type_def SPEED_PID,BALANCE_PID,TURN_PID,POSITION_PID;
    
    
    const float speed_pid[3]  ={0.015,0.000005,0.0};
    const float balance_pid[3]={120,0,50};
    const float turn_pid[3]   ={-10,0,-1};
    const float position_pid[3]={10,0,0};
    const float k1=120;
    const float k2=2;
     
    
    
    
    PID_init(&SPEED_PID,speed_pid,0.5,0.2);
    PID_init(&BALANCE_PID,balance_pid,100,50);
    PID_init(&TURN_PID,turn_pid,100,50);
    PID_init(&POSITION_PID,position_pid,15,10);
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
     
     //get data
     yaw  =wb_inertial_unit_get_roll_pitch_yaw(IMU)[2];
     pitch=-wb_inertial_unit_get_roll_pitch_yaw(IMU)[1];
     roll =wb_inertial_unit_get_roll_pitch_yaw(IMU)[0];
     yaw+=3.14159;
     if (yaw-yaw_last>3.14)
     {
         yaw_count--;
     }
     else if(yaw-yaw_last<-3.14)
     {
         yaw_count++;
     }
     yaw_sum=6.28*yaw_count+yaw;
     
     yaw_last=yaw;
    
    
     accel_x=wb_gyro_get_values(GYRO)[0];
     accel_y=wb_gyro_get_values(GYRO)[1];
     accel_z=wb_gyro_get_values(GYRO)[2];
    
    
    
    
    
     //set 
     
    /*****************************************************************************************/
     //position loop : PD control
     
    
     //speed loop   : PI control
     speed_error=-(wb_motor_get_velocity(rightWheel)+wb_motor_get_velocity(leftWheel));
          //Lowout Fliter
     speed_error_lowout=(1-lowout_Ka)*speed_error+lowout_Ka*speed_error_lowout_last;
     speed_error_lowout_last=speed_error_lowout;
     position_count += 0.001*speed_error_lowout;
     
     position_motor_current=PID_calc(&POSITION_PID,position_count,position_set);
     
     speed_motor_current=PID_calc(&SPEED_PID,speed_error_lowout,position_motor_current);
     
     
     
     //balance loop : PD control  3100,0,0.9
     //balance_motor_current=PID_calc(&BALANCE_PID,pitch-go_back,0.01);
     //balance_motor_current=PID_calc(&BALANCE_PID,pitch,0.01+speed_motor_current);
     
     //balanced loop : LQR controller 
    
     balance_motor_current=-k1*(pitch-speed_motor_current)-k2*(accel_z-0);
     
     
     
     //turn loop   ww :P control
     yaw_init=3.14;
     yaw_set+=left_right;
     turn_motor_current=PID_calc(&TURN_PID,yaw_sum,yaw_init+2*yaw_set);
     
     
     
     //close some loop
     //balance_motor_current=0;
     //speed_motor_current=0;
     //turn_motor_current=0;
     
     
     
     //current_output
     // rightMotor=balance_motor_current-balance_Kp*speed_motor_current+turn_motor_current;
     // leftMotor =balance_motor_current-balance_Kp*speed_motor_current-2*turn_motor_current;
     
     rightMotor=balance_motor_current+turn_motor_current;
     leftMotor =balance_motor_current-2*turn_motor_current;
     
     
     /***********************************************************************************/
     
  
     //log out
     //printf("R:%f  L:%f  ",rightMotor,leftMotor);
     //printf("speed_set %f speed %f speed_error %f ",go_back,speed_error_lowout,speed_error);
     //printf("B:%f  S:%f  ",balance_motor_current,speed_motor_current);
     //printf("P:%f  PE %f",pitch,pitch-pitch_angle_set);
     //printf("yawsum:%f yawcount %f yaw %f yawset %f ",yaw_sum,yaw_count,yaw,yaw_set);
     
     //printf("accel_x %f",accel_z);
     
     printf("position_count %f set %f current %f",position_count,position_set,position_motor_current);
     printf("\n");
     
     
     //send current
     if(rightMotor>100)  rightMotor=100;
     if(rightMotor<-100) rightMotor=-100;
     if(leftMotor>100)   leftMotor=100;
     if(leftMotor<-100)  leftMotor=-100;
     
     wb_motor_set_velocity(rightWheel,rightMotor);
     wb_motor_set_velocity(leftWheel ,leftMotor);
     
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
    //printf("key:%d ",Key);
    switch(Key)
    {
      case 87: position_set+=0.03;pitch_angle_set=0.1;break;
      case 83: position_set-=0.03;pitch_angle_set=-0.1;break;
      case 65: left_right=0.05;break;
      case 68: left_right=-0.05;break;
     // case 32:            //空格
     //case 315:            //上
     //case 317:            //下
      default: go_back=0,left_right=0;pitch_angle_set=0.02;
    }

      //  printf("%d,%f\n",Key,go_back);
  
   
}