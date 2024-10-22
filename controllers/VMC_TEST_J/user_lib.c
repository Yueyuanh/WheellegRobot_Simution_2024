#include <user_lib.h>
#include <struct_typedef.h>

/**********************************************************************************************************************
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
/***********************************************************************************************************************/

/***********************************************************************************************************************
  * @brief          一阶低通滤波器初始化
  * @author         RM
  * @param[in]      滤波器结构体
  * @param[in]      时间常数
  * @param[in]      低通滤波参数
  * @param[in]      输入量
  * @retval         低通滤波值
  */

 void LPF_init(LPF_type_def *LPF,fp32 frame_period,const fp32 num)
 {
   LPF->frame_period = frame_period;
   LPF->num = num;
   LPF->input = 0.0f;
   LPF->output = 0.0f;
 }

 fp32 LPF_calc(LPF_type_def *LPF,fp32 input)
 {
   LPF->input = input;
   LPF->output = (1 - LPF->num) * LPF->input + LPF->num * LPF->output;
   //LPF->output = LPF->num / (LPF->num + LPF->frame_period) * LPF->output + LPF->frame_period / (LPF->num + LPF->frame_period) * LPF->input;
   return LPF->output;
 }
 /***********************************************************************************************************************/