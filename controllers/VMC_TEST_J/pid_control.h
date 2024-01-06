#ifndef PID_CONTROL
#define PID_CONTROL

#include <pid.h>

#define LimitMax(input, max)   \
{                          \
		if (input > max)       \
		{                      \
				input = max;       \
		}                      \
		else if (input < -max) \
		{                      \
				input = -max;      \
		}                      \
}



struct PID_CONTROL_t
{
    pid_type_def stand_PID;
    pid_type_def balance_PID;
    pid_type_def speed_PID;
    pid_type_def distance_PID;
    pid_type_def roll_PID;
    pid_type_def yaw_PID;
    float stand_feed;

} PID_MODE;

void PID_CONTROL_init();
void PID_CONTROL_calc();

#endif
