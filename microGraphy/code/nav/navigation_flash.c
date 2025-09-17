#include "zf_common_headfile.h"
// #include "navigation_flash_improved.h"

uint8 aa = 0;
int bx = 6;

int32 data_yaw_buffer[4000] = {0};
Nag N;
int sum = 0;
int suml = 0;
#define M_PI (3.1415927f)
#define POINT_INTERVAL 0.5f 
float errors_coords[COORD_RECORD];
int point_error_index= 0;
float error_dir = 0;
int error_angle_dir = 0;
int error_make_flag = 0;
int actual_error_point = 0;
int max_error_point_mem = 0;
float Last_Nag_yaw = 0;
int Mileage_All_sum = 0;
int Mileage_All_sum_last = 0;
int Mileage_All_sum_list[COORD_RECORD];
int cnmb = 0;
uint8 yugvbjvutyjvbihihib = 0;
uint8 vbhjnmkl = 0;
uint8 hgbnm = 0;
uint8 ghui = 0;
int8 tyu = 0;
//uint8 sdf;
uint8 vbn = 0;
uint8 iiiop = 0;
uint8 opopop = 0;
uint8 lastopopop=0;
float rt_yaw = 0.0f;
uint8 qulv_yuzhi_chixu_biaozhiwei_hahahahahahahahaha = 0;
float Qulv = 0.0f;
float qulv_zhijiao = 0.0f;
uint8 tyu_zhijiao = 0;
uint8 vbn_zhijiao = 0;
uint8 opopop_zhijiao = 0;
uint8 ghui_zhijiao = 0;
float qulvzhe;
uint8 zhetime=0;
uint8 zhitime=0;
int zheng_reset_state = 0;  // 正曲率清零状态机 (0-3)
int fu_reset_state = 0;     // 负曲率清零状态机 (0-3)
int lianxuzhijiao = 0;     // 连续直角标志 (0=无 1=正曲率状态 2=负曲率状态)
bool was_high = false;  // 标记是否曾经 >40
bool was_low = false;   // 标记是否曾经 <-40
bool high_to_mid_reset = false;  // 标记是否进入 >40 → (0,20) 状态
bool mid_to_low_cancel = false;  // 标记是否 (0,20) → <-40（取消清零）
bool low_to_mid_reset = false;   // 标记是否进入 <-40 → (-20,0) 状态
 bool mid_to_high_cancel = false; // 标记是否 (-20,0) → >40（取消清零）
 
void Run_Nag_reSave()//光电五倍前瞻自增函数
{  
  Mileage_All_sum+= (sum+suml)/2;
   int CCB;
   if(point_error_index>=10)CCB=point_error_index-10;
   else CCB=point_error_index;
  for(int i = (CCB) ; i < max_error_point_mem; i++)
  {   
    if(Mileage_All_sum_list[i] >= (Mileage_All_sum+Nag_Set_mileage*2*(3.0f)*(((sum+suml)/2-150)/116.0f+0.0f)))
    {
      point_error_index = i;
      break;
    }
      cnmb = i;
  }
  float QuiLv=0;
  float quilv=0;
//  float quilv =1000*calculate_curvature(
//    errors_coords[point_error_index], errors_coords[point_error_index+2], errors_coords[point_error_index+4], // 三个点的切线角度（度）
//    Mileage_All_sum_list[point_error_index+2]-Mileage_All_sum_list[point_error_index],
//    Mileage_All_sum_list[point_error_index+4]-Mileage_All_sum_list[point_error_index+2]);  
       for(int jks=0;jks<5;jks++){    
    QuiLv=1000*calculate_curvature(
    errors_coords[point_error_index+jks], errors_coords[point_error_index+2+jks], errors_coords[point_error_index+4+jks], // 三个点的切线角度（度）
    Mileage_All_sum_list[point_error_index+2+jks]-Mileage_All_sum_list[point_error_index+jks],
    Mileage_All_sum_list[point_error_index+4+jks]-Mileage_All_sum_list[point_error_index+2+jks]);                     // 相邻点距离
    if(fabs(quilv)<fabs(QuiLv))quilv=QuiLv;                                       
     }
    if(quilv>65)quilv=65;
    if(quilv<-65)quilv=-65;
    if(qulv_yuzhi_chixu_biaozhiwei_hahahahahahahahaha == 0) Qulv=quilv;
  if(fabs(quilv)<20)zhitime++;
  else zhitime=0;
  if(zhitime>20)opopop=1;   
   // 检测从 <-40 上升到 >40
        if (quilv > 25 && was_low && !mid_to_high_cancel) {
            opopop++;
            was_low = false;;
        }
        // 检测从 >40 下降到 <-40
        else if (quilv < -25 && was_high && !mid_to_low_cancel) {
            opopop++;
            was_high = false;
        }

        // 检测 >40 → (0,20) → >40（清零条件）
        if (quilv > 25) {
            was_high = true;
            if (high_to_mid_reset && !mid_to_low_cancel) {
                opopop = 1;
            }
            high_to_mid_reset = false;
            mid_to_low_cancel = false;
        } 
        // 检测 >40 → (0,20) 状态
        else if (quilv > 0 && quilv < 10 && was_high) {
            high_to_mid_reset = true;
        }
        // 检测 (0,20) → <-40（取消清零）
        else if (quilv < -25 && high_to_mid_reset) {
            mid_to_low_cancel = true;
        }
        // 检测 <-40 → (-20,0) → <-40（清零条件）
        if (quilv < -25) {
            was_low = true;
            if (low_to_mid_reset && !mid_to_high_cancel) {
                opopop = 1;
            }
            low_to_mid_reset = false;
            mid_to_high_cancel = false;
        } 
        // 检测 <-40 → (-20,0) 状态
        else if (quilv > -10 && quilv < 0 && was_low) {
            low_to_mid_reset = true;
        }
        // 检测 (-20,0) → >40（取消清零）
        else if (quilv > 25 && low_to_mid_reset) {
            mid_to_high_cancel = true;
        }
  if(zhetime==0)lastopopop=opopop;
  if(lastopopop>=3&&opopop<3){   // Mileage_All_sum-= ((sum+suml)/2)*0.1*finaltarget_speed/160;
    zhetime++;
    if(zhetime == 15)zhetime= 0;
  }
  else zhetime= 0;
  if(lastopopop>=3)//取消长前瞻，给个标志位，取消曲率的计算直接赋值给前瞻。//折现
  {  
  // Mileage_All_sum-= ((sum+suml)/2)*0.2*finaltarget_speed/160;
    for(int i = (CCB) ; i < max_error_point_mem; i++)
  {    
    // if(Mileage_All_sum_list[i] >= (Mileage_All_sum+Nag_Set_mileage*2*(fabs(Qulv)/100+1)*(((sum+suml)/2-150)/116+0.0f)))
     if(Mileage_All_sum_list[i] >= (Mileage_All_sum+Nag_Set_mileage*2*(fabs(Qulv)/100+0.3)*(((sum+suml)/2-150)/116.0f+0.0f)))
    {
      point_error_index = i;
      break;
    }
      cnmb = i;
  }
  }
  else{
  
  if(fabs(Qulv)>50) {
    qulv_yuzhi_chixu_biaozhiwei_hahahahahahahahaha++;
  
  for(int i = (CCB) ; i < max_error_point_mem; i++)
  {   
    
    if(Mileage_All_sum_list[i] >= (Mileage_All_sum+Nag_Set_mileage*2*(fabs(Qulv)/16-0.2)*((sum+suml)/2-50)/150.0+0.0f))
    {
      point_error_index = i;
      break;
    }
      cnmb = i;
  } 
  
//  if(CCB != point_error_index
//     && hgbnm == 1
//       &&fabs(quilv)>20){
//     Mileage_All_sum+= ((sum+suml)/2)*0.05*finaltarget_speed/160;//直角
//  }
 //   if(fabs(quilv)>20) Mileage_All_sum+= ((sum+suml)/2)*0.025*finaltarget_speed/160;//直角
    if(qulv_yuzhi_chixu_biaozhiwei_hahahahahahahahaha == 2)qulv_yuzhi_chixu_biaozhiwei_hahahahahahahahaha= 0;

  } 
  else {
    hgbnm = 0;
    for(int i = (CCB) ; i < max_error_point_mem; i++)
    {   
      if(Mileage_All_sum_list[i] >= (Mileage_All_sum+Nag_Set_mileage*2*(fabs(Qulv)/70+9.0f/7)*(((sum+suml)/2-150)/116.0+0.0f)))
      {
        point_error_index = i;
        break;
      }
      cnmb = i;
    }
  }
  }
}

void Run_Nag_Save()
{
     N.Mileage_All+=(sum+suml)/2;//历程计读取，左右编码器，使用浮点数的话误差能保留下来
     
     Mileage_All_sum+= (sum+suml)/2;

    if(N.Mileage_All >= Nag_Set_mileage)    
    {
      if(error_make_flag == 0){

       errors_coords[point_error_index] = Nag_Yaw;//暂存角度。
       Mileage_All_sum_list[point_error_index] = Mileage_All_sum;
       point_error_index++;
      if(N.Mileage_All>0) N.Mileage_All -= Nag_Set_mileage;  
      else N.Mileage_All += Nag_Set_mileage;
      }
      // Mileage_All_sum_last = Mileage_All_sum;
    }
   
}



double calculate_curvature(
    float theta1, float theta2, float theta3, // 三个点的切线角度（度）
    int d12, int d23                     // 相邻点距离
) {
  float d1122,d2233;
  double kappa;
  d1122=d12/Nag_Set_mileage*0.5;
  d2233=d12/Nag_Set_mileage*0.5;
    // 1. 计算三个点的切线方向向量（无需模360°，直接使用sin/cos处理周期性）
    double T1_x = cos(DEG_TO_RAD(theta1));
    double T1_y = sin(DEG_TO_RAD(theta1));
//    
//    double T2_x = cos(DEG_TO_RAD(theta2));
//    double T2_y = sin(DEG_TO_RAD(theta2));
//    
    double T3_x = cos(DEG_TO_RAD(theta3));
    double T3_y = sin(DEG_TO_RAD(theta3));

    // 2. 计算方向向量差分和总弧长
    double delta_Tx = T3_x - T1_x;
    double delta_Ty = T3_y - T1_y;
    double total_distance = d1122 + d2233;
    double delta_T_norm;
    // 3. 计算曲率（无符号版本）
    delta_T_norm = sqrt(delta_Tx * delta_Tx + delta_Ty * delta_Ty)*(theta3-theta1)/fabs(theta3-theta1);
 //   double delta_T_norm = atan2(delta_Ty, delta_Tx); // 弧度制角度差
    kappa = delta_T_norm / total_distance/4;
    if(d12==0||d23==0)kappa=0;

    return kappa;
}
double fabs(double f){
  if(f<0)f=-f;
  return f;
}

double fmax(double a,double b){
  double f=0;
 if(a<=b)f=b;
 else f=a;
  return f;
}