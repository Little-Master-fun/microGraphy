#include "test_sch16tk10.h"
#include "driver_sch16tk10.h"
#include "zf_device_ips114.h"
#include "zf_common_headfile.h"
#include <stdio.h>

void test_imu(void)
{
    // 初始化屏幕
    ips114_set_dir(IPS114_PORTAIT);
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    ips114_init();
    ips114_clear();
    ips114_show_string(0, 0, "SCH16TK10 Test...");

    // 初始化IMU
    SCH1_filter sFilter;
    SCH1_sensitivity sSensitivity;
    SCH1_decimation sDecimation;

    // 设置默认参数
    sFilter.Rate12 = 30; 
    sFilter.Acc12 = 30;
    sFilter.Acc3 = 30;

    sSensitivity.Rate1 = 1600;
    sSensitivity.Rate2 = 1600;
    sSensitivity.Acc1 = 3200;
    sSensitivity.Acc2 = 3200;
    sSensitivity.Acc3 = 3200;

    sDecimation.Rate2 = 2;
    sDecimation.Acc2 = 2;

    if (SCH1_init(sFilter, sSensitivity, sDecimation, false) != SCH1_OK)
    {
        ips114_show_string(0, 16, "IMU Init Failed!");
        while (1);
    }
    else
    {
        ips114_show_string(0, 16, "IMU Init Success!");
    }
    
    system_delay_ms(1000);

    SCH1_raw_data raw_data;
    SCH1_result result_data;
    char text_buffer[50];

    while(1)
    {
        SCH1_getData(&raw_data);
        SCH1_convert_data(&raw_data, &result_data);
        
        ips114_clear();

        sprintf(text_buffer, "AccX: %.2f", result_data.Acc1[AXIS_X]);
        ips114_show_string(0, 0, text_buffer);

        sprintf(text_buffer, "AccY: %.2f", result_data.Acc1[AXIS_Y]);
        ips114_show_string(0, 16, text_buffer);

        sprintf(text_buffer, "AccZ: %.2f", result_data.Acc1[AXIS_Z]);
        ips114_show_string(0, 32, text_buffer);

        sprintf(text_buffer, "GyrX: %.2f", result_data.Rate1[AXIS_X]);
        ips114_show_string(0, 48, text_buffer);

        sprintf(text_buffer, "GyrY: %.2f", result_data.Rate1[AXIS_Y]);
        ips114_show_string(0, 64, text_buffer);

        sprintf(text_buffer, "GyrZ: %.2f", result_data.Rate1[AXIS_Z]);
        ips114_show_string(0, 80, text_buffer);

        sprintf(text_buffer, "Temp: %.2f", result_data.Temp);
        ips114_show_string(0, 96, text_buffer);

        system_delay_ms(100);
    }
}
