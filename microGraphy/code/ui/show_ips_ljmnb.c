#include "zf_common_headfile.h"
// #include "show_ips_ljmnb_improved.h"
PIDG pidg;
int fuya;
uint8 go_index = 0;//发车标志位
uint8 go_go_go = 0;
SystemMode_mode current_mode_key = IKUN_MODE0;  // 初始模式为MODE0
Key_2 key_2  = IKUN_KEY2_0;
Key_3 key_3  = IKUN_KEY3_0;
Key_4 key_4  = IKUN_KEY4_0;

bool key_pressed_KEY1 = false;         // 按键按下标志
bool last_key_state_KEY1 = true;       // 上一次按键状态
bool key_pressed_KEY2 = false;         // 按键按下标志
bool last_key_state_KEY2 = true;       // 上一次按键状态
bool key_pressed_KEY3 = false;         // 按键按下标志
bool last_key_state_KEY3 = true;       // 上一次按键状态
bool key_pressed_KEY4 = false;         // 按键按下标志
bool last_key_state_KEY4 = true;       // 上一次按键状态
int finaltarget_speed = 0;

#define LEDX                    (P19_0)

#define KEY1                    (P00_3)
#define KEY2                    (P00_2)
#define KEY3                    (P01_0)
#define KEY4                    (P01_1)
  
float X1 = 0;
float Y1 = 10;
float X2 = 0;
float Y2 = 10;
float X3 = 0;
float Y3 = 20;
float X4 = 0;
float Y4 = 20;
float X5 = 0;
float Y5 = 20;
float X6 = 0;
float Y6 = 20;

int R1 = 30;
int R2 = 30;


void gpio_init_key_ips(void){
  
    gpio_init(LEDX, GPO, GPIO_LOW, GPO_PUSH_PULL);             
    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);            
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);              
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);              
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);             

  
    ips114_set_dir(IPS114_PORTAIT);
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    ips114_clear();
  
    ips114_show_string(10, 10, "Waveform Display");
    ips114_show_string(10, 30, "Press KEY1 to choose");
    system_delay_ms(1000);
    ips114_clear();
  
}
uint8 flash_open_re = 0;
void show_open(void)
{
  switch (current_mode_key) {
  case IKUN_MODE0:
    ips114_show_string(1, 10, " MODE0 : Flash Loader");
    ips114_show_string(1, 30, "PRESS KEY3_TO READ :");
    if(flash_open_re == 0)ips114_show_string(1, 50, "NO FLASH");
    else ips114_show_string(1, 50, "Please waiting 5s!!");
    break;
  case IKUN_MODE1:
    ips114_show_string(1, 10, " MODE1 : Parameter adjustment");
    switch (key_2) {
              case IKUN_KEY2_0: //finaltarget_speed
                ips114_show_string(10, 30, "FINALTARGETSPEED :");
                ips114_show_int( 60 , 50, pidg.speed,  4);                     // 显示 int8 数据
                break;
              case IKUN_KEY2_1:
                ips114_show_string(10, 30, "PIDG.KP :");
                ips114_show_float( 60 , 50, pidg.Kp, 2, 4);       
                break;
              case IKUN_KEY2_2:  
                ips114_show_string(10, 30, "PIDG.KD :");
                ips114_show_float( 60 , 50, pidg.Kd, 2, 4);     
                break;
              case IKUN_KEY2_3:
                ips114_show_string(10, 30, "R1  K3+1  K4-1");
                ips114_show_string(10, 50, "R1 : ");
                ips114_show_float( 60 , 50, R1, 2, 4); 
                break;
              case IKUN_KEY2_4: //finaltarget_speed
                ips114_show_string(10, 30, "R2  K3+1  K4-1");
                ips114_show_string(10, 50, "R2 : ");
                ips114_show_float( 60 , 50, R2, 2, 4); 
                break;
              default:
                key_2 = IKUN_KEY2_0;
                break;
              }
    break;  
case IKUN_MODE2:
    ips114_show_string(1, 10, "MODE2:Open road adjustment ");
     switch (key_2) {
              case IKUN_KEY2_0: //finaltarget_speed
                ips114_show_string(10, 30, "X1  K3+1  K4-1");
                ips114_show_string(10, 50, "X1 : ");
                ips114_show_float( 60 , 50, X1, 2, 4); 
                break;
              case IKUN_KEY2_1:
                ips114_show_string(10, 30, "Y1  K3+1  K4-1");
                ips114_show_string(10, 50, "Y1 : ");
                ips114_show_float( 60 , 50, Y1, 2, 4);       
                break;
             case IKUN_KEY2_2: //finaltarget_speed
                ips114_show_string(10, 30, "X2  K3+1  K4-1");
                ips114_show_string(10, 50, "X2 : ");
                ips114_show_float( 60 , 50, X2, 2, 4); 
                break;
              case IKUN_KEY2_3:
                ips114_show_string(10, 30, "Y2  K3+1  K4-1");
                ips114_show_string(10, 50, "Y2 : ");
                ips114_show_float( 60 , 50, Y2, 2, 4);       
                break;
              case IKUN_KEY2_4: //finaltarget_speed
                ips114_show_string(10, 30, "X3  K3+1  K4-1");
                ips114_show_string(10, 50, "X3 : ");
                ips114_show_float( 60 , 50, X3, 2, 4); 
                break;
              case IKUN_KEY2_5:
                ips114_show_string(10, 30, "Y3  K3+1  K4-1");
                ips114_show_string(10, 50, "Y3 : ");
                ips114_show_float( 60 , 50, Y3, 2, 4);       
                break;
              case IKUN_KEY2_6:
                ips114_show_string(10, 30, "X4  K3+1  K4-1");
                ips114_show_string(10, 50, "X4 : ");
                ips114_show_float( 60 , 50, X4, 2, 4);       
                break;
              case IKUN_KEY2_7:
                ips114_show_string(10, 30, "Y4  K3+1  K4-1");
                ips114_show_string(10, 50, "Y4 : ");
                ips114_show_float( 60 , 50, Y4, 2, 4);       
                break;
              case IKUN_KEY2_8:
                ips114_show_string(10, 30, "X5  K3+1  K4-1");
                ips114_show_string(10, 50, "X5 : ");
                ips114_show_float( 60 , 50, X5, 2, 4);       
                break;
              case IKUN_KEY2_9:
                ips114_show_string(10, 30, "Y5  K3+1  K4-1");
                ips114_show_string(10, 50, "Y5 : ");
                ips114_show_float( 60 , 50, Y5, 2, 4);       
                break;
              case IKUN_KEY2_10:
                ips114_show_string(10, 30, "X6  K3+1  K4-1");
                ips114_show_string(10, 50, "X6 : ");
                ips114_show_float( 60 , 50, X6, 2, 4);       
                break;
              case IKUN_KEY2_11:
                ips114_show_string(10, 30, "Y6  K3+1  K4-1");
                ips114_show_string(10, 50, "Y6 : ");
                ips114_show_float( 60 , 50, Y6, 2, 4);       
                break;
              default:
                key_2 = IKUN_KEY2_0;
                break;
              }
    break;      
  case IKUN_MODE3:
    ips114_show_string(1, 10, " MODE3 : PRESS_KEY3_TOGO");
    ips114_show_string(40, 30, "GO_GO_GO_press_st_1 :");
    ips114_show_int( 60 , 50, go_go_go,  4);   
     if(fuya == 1){
        ips114_show_string( 80 , 70," FUYA_OPEN");   
     }
     else{
       ips114_show_string( 80 , 70, "FUYA_CLOSE");   
     }

    break;   
  default:
    ips114_show_string(1, 10, " MODE0 ");
    current_mode_key = IKUN_MODE0;
    break;   
  }
}

void key_test(void)
{
  
    bool current_key_state = (gpio_get_level(KEY1) == 0);  // 检测按键是否按下（低电平有效）
    
    if (current_key_state && !last_key_state_KEY1) {
      key_pressed_KEY1 = true;  
    }
    if (!current_key_state && last_key_state_KEY1 && key_pressed_KEY1) {
      key_pressed_KEY1 = false;  
      
      current_mode_key = (SystemMode_mode)((current_mode_key + 1) % IKUN_MODE_MAX);
      ips114_clear();
    }
    
    last_key_state_KEY1 = current_key_state;  
    
    current_key_state = (gpio_get_level(KEY2) == 0);  // 检测按键是否按下（低电平有效）
    
    if (current_key_state && !last_key_state_KEY2) {
      key_pressed_KEY2 = true;  
    }
    if (!current_key_state && last_key_state_KEY2 && key_pressed_KEY2) {
      key_pressed_KEY2 = false;  
      
      key_2 = (Key_2)((key_2 + 1) % IKUN_KEY2_MAX);
      ips114_clear();
    }
    
    last_key_state_KEY2 = current_key_state; 
    
    current_key_state = (gpio_get_level(KEY3) == 0);  // 检测按键是否按下（低电平有效）
    
    if (current_key_state && !last_key_state_KEY3) {
      key_pressed_KEY3 = true;  
    }
    if (!current_key_state && last_key_state_KEY3 && key_pressed_KEY3) {
      key_pressed_KEY3 = false;  
      
      key_3 = (Key_3)((key_3 + 1) % IKUN_KEY3_MAX);
      ips114_clear();
    }
    
    last_key_state_KEY3 = current_key_state; 
    
    current_key_state = (gpio_get_level(KEY4) == 0);  // 检测按键是否按下（低电平有效）
    
    if (current_key_state && !last_key_state_KEY4) {
      key_pressed_KEY4 = true;  
    }
    if (!current_key_state && last_key_state_KEY4 && key_pressed_KEY4) {
      key_pressed_KEY4 = false;  
      
      key_4 = (Key_4)((key_4 + 1) % IKUN_KEY4_MAX);
      ips114_clear();
    }
    
    last_key_state_KEY4 = current_key_state; 
    
}