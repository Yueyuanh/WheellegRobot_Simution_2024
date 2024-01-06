#include <user_lib.h>
#include <struct_typedef.h>

/**
  * @brief          梯度微分器
  * @author         
  * @param[in]      微分结构体
  * @param[in]      控制带宽
  * @param[in]      时间常数
  * @param[in]      输入
  * @retval         微分
  */
void differentiator_init(differ_type_def *differ)
{
  differ->u[0] = 0;
  differ->u[1] = 0;
  differ->y[0] = 0;
  differ->y[1] = 0;
}



float differentiator(differ_type_def *differ ,float bandwidth,float time_cons,float input)
{ 
  //使用梯形法离散化
 
  differ->u[0] = differ->u[1];	
  differ->u[1] = input;
  differ->y[0] = differ->y[1];
  differ->y[1] = (bandwidth*(differ->u[1]-differ->u[0])-(bandwidth*time_cons/2-1)*differ->y[0])/(1+(bandwidth*time_cons)/2);
  
  return differ->y[1];
}