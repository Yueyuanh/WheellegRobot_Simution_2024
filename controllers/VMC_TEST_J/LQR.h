#ifndef LQR_H
#define LQR_H

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

struct LQR_t
{
    /* data */
    fp32 balance_LQR;
    fp32 balance_LQR_chassis;
    fp32 balance_LQR_sit;
    fp32 balance_LQR_jump;

    pid_type_def roll_PID;
    pid_type_def stand_PID;
    float stand_feed;//前馈推力

    pid_type_def yaw_PD;
    float K_yaw_out;

    pid_type_def coordinate_PD;
    pid_type_def leg_length_roll_PID;

    float LQR_FEED_R[2][6];
    float LQR_FEED_L[2][6];
    
    float LQR_OUT_R[2][6];
    float LQR_OUT_L[2][6];


}LQR;

void LQR_init();
void LQR_calc();
void LQR_log();

#endif
