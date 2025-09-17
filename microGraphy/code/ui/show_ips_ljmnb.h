#ifndef _show_ips_ljmnb_h_
#define _show_ips_ljmnb_h_

extern uint8 go_index;//发车标志位
extern uint8 go_go_go;
typedef enum {
    IKUN_MODE0,
    IKUN_MODE1,
    IKUN_MODE2,
    IKUN_MODE3,
    IKUN_MODE_MAX
} SystemMode_mode;
typedef enum {
    IKUN_KEY2_0,
    IKUN_KEY2_1,
    IKUN_KEY2_2,
    IKUN_KEY2_3,
    IKUN_KEY2_4,
    IKUN_KEY2_5,  
    IKUN_KEY2_6,
    IKUN_KEY2_7,
    IKUN_KEY2_8,
    IKUN_KEY2_9,
    IKUN_KEY2_10,
    IKUN_KEY2_11,
    IKUN_KEY2_MAX
} Key_2;
typedef enum {
    IKUN_KEY3_0,
    IKUN_KEY3_1,
    IKUN_KEY3_MAX
} Key_3;
typedef enum {
    IKUN_KEY4_0,
    IKUN_KEY4_1,
    IKUN_KEY4_MAX
} Key_4;

typedef struct
{
	float Kp,Ki,Kd;     //比例、积分、微分系数
        int speed;
}PIDG;
extern PIDG pidg;
extern SystemMode_mode current_mode_key;  // 初始模式为MODE0
extern Key_2 key_2 ;
extern Key_3 key_3 ;
extern Key_4 key_4 ;
extern int finaltarget_speed;
extern int fuya;

extern int R1 ;
extern int R2 ;

extern float X1 ;
extern float Y1 ;
extern float X2 ;
extern float Y2 ;
extern float X3 ;
extern float Y3 ;
extern float X4 ; 
extern float Y4 ;
extern float X5 ; 
extern float Y5 ; 
extern float X6 ;
extern float Y6 ; 
extern uint8 flash_open_re;
extern bool key_pressed_KEY1 ;         // 按键按下标志
extern bool last_key_state_KEY1;       // 上一次按键状态
extern bool key_pressed_KEY2 ;         // 按键按下标志
extern bool last_key_state_KEY2;       // 上一次按键状态
extern bool key_pressed_KEY3 ;         // 按键按下标志
extern bool last_key_state_KEY3;       // 上一次按键状态
extern bool key_pressed_KEY4 ;         // 按键按下标志
extern bool last_key_state_KEY4;       // 上一次按键状态

void key_test(void);
void show_open(void);
void gpio_init_key_ips(void);

#endif