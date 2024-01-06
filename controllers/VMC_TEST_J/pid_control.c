#include <pid_control.h>
#include <VMC.h>
#include <VMC_TEST_J.h>
#include <struct_typedef.h>
#include <stdio.h>
#include <plot.h>

/*
这里是轮腿测试PID版本，
及对LQR进行解耦（其实可以考虑在不同状态下选择不同的控制模式）
这里对轮毂电机和关节电机进行分开控制，轮只管保持pitch轴水平，也可以考虑与其他变量耦合
而关节电机通过VMC将输入量变为F和Tp，通过stand_PID加前馈控制腿长，其中要加上离地检测
通过Tp控制腿的摆动来控制底盘的速度和位置
*/