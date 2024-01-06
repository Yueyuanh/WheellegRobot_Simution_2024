#ifndef USER_LIB_H
#define USER_LIB_H

#include <struct_typedef.h>


typedef struct{   
	fp32 y[2];
	fp32 u[2];

} differ_type_def;


float differentiator(differ_type_def *differ, float bandwidth, float time_cons, float input);
void differentiator_init(differ_type_def *differ);


#endif
