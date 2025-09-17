#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_


//*********************用户设置区域****************************//
#define MaxSize 500    //flash存储的最大页面
#define COORD_RECORD 25000
//参数范围 <0 - 47>
#define Nag_End_Page 1      //flash中止页面
#define Nag_Start_Page 40   //flah起始页面

#define Nag_Set_mileage 400 //里程计，一厘米的里程脉冲

#define Nag_Yaw angle_Z //陀螺仪读取出来的偏航角

#define L_Mileage Mf.L_Speed   //左轮编码器
#define R_Mileage Mf.R_Speed //右轮编码器
#define DEG_TO_RAD(angle) ((angle) * M_PI / 180.0)
//********************************************************//
extern int sum;	//编码器
extern int suml;	
typedef struct{
       float Final_Out; //最终输出
       int Mileage_All;   //里程计数
       float Angle_Run; //读取的偏航角
       float Angle_Run2;
       float Error;
       float Duanlu;
       bool Nag_Stop_f; //惯导中止flag
       uint8 Flash_read_f;//惯导读取flag
       uint16 size; //惯导数组索引通用计数
       uint16 Save_count;//储存计数位
       uint16 Save_index;//保存的flag
       uint8 End_f;//中止flag
       //与flash相关的
       uint8 Flash_page_index;//flash页面索引
       uint8 Flash_Save_Page_Index;//flash保存页码索引
       uint8 Nag_SystemRun_Index;   //惯导执行索引
      
}Nag;

extern Nag N;   //整个变量的结构体，方便开发和移植

extern int actual_error_point;
extern int max_error_point_mem;
//extern float x_coords[COORD_RECORD];   // X坐标数组
//extern float y_coords[COORD_RECORD];   // Y坐标数组
extern float errors_coords[COORD_RECORD];
extern int Mileage_All_sum;

extern float error_dir;
extern int error_angle_dir;
extern int point_error_index;
extern int error_make_flag;
extern int cnmb;
extern float qulv;
extern int Mileage_All_sum_list[COORD_RECORD];
extern uint8 opopop;
//double cur(float a,float b,float c);
double calculate_curvature(
    float theta1, float theta2, float theta3, // 三个点的切线角度（度）
    int d12, int d23                     // 相邻点距离
);
double fmax(double a,double b);
double fabs(double f);
extern uint8 opopop_zhijiao;
extern uint8 lastopopop;
extern uint8 sdf;
void Run_Nag_reSave();
void Run_Nag_Save();    //偏航角读取函数


#endif /* CODE__NAVIGATION_H_ */
