#include <stdio.h>
#include <Keyboard_control.h>
#include <VMC_TEST_J.h>
#include <webots/robot.h>
#include <webots/keyboard.h>

void Keyboard_init()
{

    wb_keyboard_enable(1);
}


void Keyboard_control()
{
    int Key = wb_keyboard_get_key();
  
    printf("key:%d\n",Key );
    switch (Key)
    {
    case 87://w
        chassis_data.foot_distance_set += 0.04;
        break;
    case 83://s
        chassis_data.foot_distance_set -= 0.04;
        break;

    case 65://a
        chassis_data.yaw_set -= 0.025;
        break;

    case 68://d
        chassis_data.yaw_set += 0.025;
        break;
    
    
    default:

        break;
    }


    if(Key==32 )
    {
        chassis_data.leg_stand_F = -50;
        
        printf("JUMP!!!\n");
    }
    chassis_data.leg_stand_F = 0;
    Key_last = Key;

}