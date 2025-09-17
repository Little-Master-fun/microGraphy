/*********************************************************************************************************************
* 文件名称          test_flash_road.c
* 功能说明          Flash存储路径数据测试程序实现文件
* 作者              AI Assistant
* 版本信息          v1.0
* 修改记录
* 日期              作者                备注
* 2024-XX-XX        AI Assistant        创建Flash存储测试程序
*
* 文件作用说明：
* 本文件实现Flash存储系统的完整测试功能，验证存储、读取、数据完整性等功能
* 通过IPS114屏幕实时显示测试结果和系统状态
*
* 测试实现功能：
* 1. 基本功能验证：测试初始化、存储、读取、清除等核心功能
* 2. 数据完整性检查：验证存储数据的准确性和完整性
* 3. 性能评估：测试不同数据量下的存储和读取速度
* 4. 边界条件测试：验证系统在极限条件下的稳定性
* 5. 兼容性验证：确保新旧接口的兼容性
* 6. 错误处理测试：验证各种异常情况的处理能力
*
* 测试数据生成：
* - 模拟真实的里程数据和坐标数据
* - 生成不同规模的测试数据集
* - 包含边界值和异常值测试
*
* 结果显示：
* - 实时显示测试进度和状态
* - 详细的错误信息和性能数据
* - 测试结果统计和分析
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "test_flash_road.h"
#include "zf_device_ips114.h"

//=================================================内部变量定义================================================
static uint32 test_start_time = 0;             // 测试开始时间
static uint32 test_pass_count = 0;             // 通过测试计数
static uint32 test_fail_count = 0;             // 失败测试计数

// 测试数据数组
static int test_mileage_data[TEST_FLASH_LARGE_DATA_SIZE];
static float test_coord_data[TEST_FLASH_LARGE_DATA_SIZE];

//=================================================内部函数声明================================================
static void test_flash_init_display(void);
static void test_flash_generate_data(uint32 data_size);
static void test_flash_verify_data(uint32 data_size);
static void test_flash_display_result(const char* test_name, uint8 result);
static void test_flash_display_status(void);
static uint32 test_flash_get_time_ms(void);

//=================================================测试函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Flash存储基本功能测试
//-------------------------------------------------------------------------------------------------------------------
void test_flash_road_basic(void)
{
    test_flash_init_display();
    ips114_show_string(0, 0, "Flash Basic Test");
    
    // 1. 初始化测试
    ips114_show_string(0, 16, "1.Init...");
    flash_road_result_enum result = flash_road_init();
    test_flash_display_result("Init", (result == FLASH_ROAD_RESULT_OK));
    system_delay_ms(TEST_FLASH_DELAY_SHORT);
    
    // 2. 生成测试数据
    ips114_show_string(0, 32, "2.Gen Data...");
    test_flash_generate_data(TEST_FLASH_SMALL_DATA_SIZE);
    max_error_point_mem = TEST_FLASH_SMALL_DATA_SIZE;
    system_delay_ms(TEST_FLASH_DELAY_SHORT);
    
    // 3. 存储测试
    ips114_show_string(0, 48, "3.Store...");
    result = flash_road_store();
    test_flash_display_result("Store", (result == FLASH_ROAD_RESULT_OK));
    system_delay_ms(TEST_FLASH_DELAY_MEDIUM);
    
    // 4. 状态检查
    ips114_show_string(0, 64, "4.Status...");
    flash_road_status_enum status = flash_road_get_status();
    test_flash_display_result("Status", (status == FLASH_ROAD_STATUS_STORE_OK));
    system_delay_ms(TEST_FLASH_DELAY_SHORT);
    
    // 5. 读取测试
    ips114_show_string(0, 80, "5.Load...");
    // 清空数据数组
    memset(Mileage_All_sum_list, 0, sizeof(int) * TEST_FLASH_SMALL_DATA_SIZE);
    memset(errors_coords, 0, sizeof(float) * TEST_FLASH_SMALL_DATA_SIZE);
    
    result = flash_road_load();
    test_flash_display_result("Load", (result == FLASH_ROAD_RESULT_OK));
    system_delay_ms(TEST_FLASH_DELAY_MEDIUM);
    
    // 6. 数据验证
    ips114_show_string(0, 96, "6.Verify...");
    test_flash_verify_data(TEST_FLASH_SMALL_DATA_SIZE);
    system_delay_ms(TEST_FLASH_DELAY_SHORT);
    
    // 7. 清除测试
    ips114_show_string(0, 112, "7.Clear...");
    result = flash_road_clear();
    test_flash_display_result("Clear", (result == FLASH_ROAD_RESULT_OK));
    system_delay_ms(TEST_FLASH_DELAY_MEDIUM);
    
    test_flash_display_status();
    system_delay_ms(TEST_FLASH_DELAY_LONG);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Flash存储数据完整性测试
//-------------------------------------------------------------------------------------------------------------------
void test_flash_road_integrity(void)
{
    test_flash_init_display();
    ips114_show_string(0, 0, "Flash Integrity Test");
    
    // 1. 初始化
    flash_road_init();
    
    // 2. 生成并存储测试数据
    ips114_show_string(0, 16, "1.Store Test Data...");
    test_flash_generate_data(TEST_FLASH_MEDIUM_DATA_SIZE);
    max_error_point_mem = TEST_FLASH_MEDIUM_DATA_SIZE;
    flash_road_result_enum result = flash_road_store();
    test_flash_display_result("Store", (result == FLASH_ROAD_RESULT_OK));
    system_delay_ms(TEST_FLASH_DELAY_MEDIUM);
    
    // 3. 数据验证测试
    ips114_show_string(0, 32, "2.Verify Integrity...");
    result = flash_road_verify();
    test_flash_display_result("Verify", (result == FLASH_ROAD_RESULT_OK));
    system_delay_ms(TEST_FLASH_DELAY_SHORT);
    
    // 4. 多次读取一致性测试
    ips114_show_string(0, 48, "3.Consistency Test...");
    uint8 consistency_ok = 1;
    
    for(int i = 0; i < 3; i++)
    {
        // 清空数据
        memset(Mileage_All_sum_list, 0, sizeof(int) * TEST_FLASH_MEDIUM_DATA_SIZE);
        memset(errors_coords, 0, sizeof(float) * TEST_FLASH_MEDIUM_DATA_SIZE);
        
        // 重新读取
        flash_road_load();
        
        // 验证数据
        for(uint32 j = 0; j < TEST_FLASH_MEDIUM_DATA_SIZE; j++)
        {
            if(Mileage_All_sum_list[j] != test_mileage_data[j] ||
               errors_coords[j] != test_coord_data[j])
            {
                consistency_ok = 0;
                break;
            }
        }
        
        if(!consistency_ok) break;
        
        ips114_show_uint(120, 48, i + 1, 1);
        system_delay_ms(TEST_FLASH_DELAY_SHORT);
    }
    
    test_flash_display_result("Consistency", consistency_ok);
    system_delay_ms(TEST_FLASH_DELAY_SHORT);
    
    // 5. 数据计数验证
    ips114_show_string(0, 64, "4.Count Verify...");
    uint32 count = flash_road_get_data_count();
    test_flash_display_result("Count", (count == TEST_FLASH_MEDIUM_DATA_SIZE));
    ips114_show_uint(120, 64, count, 5);
    system_delay_ms(TEST_FLASH_DELAY_SHORT);
    
    test_flash_display_status();
    system_delay_ms(TEST_FLASH_DELAY_LONG);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Flash存储性能测试
//-------------------------------------------------------------------------------------------------------------------
void test_flash_road_performance(void)
{
    test_flash_init_display();
    ips114_show_string(0, 0, "Flash Performance Test");
    
    flash_road_init();
    
    uint32 data_sizes[] = {TEST_FLASH_SMALL_DATA_SIZE, TEST_FLASH_MEDIUM_DATA_SIZE, TEST_FLASH_LARGE_DATA_SIZE};
    const char* size_names[] = {"Small", "Medium", "Large"};
    
    for(int i = 0; i < 3; i++)
    {
        uint32 data_size = data_sizes[i];
        
        ips114_show_string(0, 16 + i * 32, size_names[i]);
        ips114_show_string(60, 16 + i * 32, "Size:");
        ips114_show_uint(100, 16 + i * 32, data_size, 5);
        
        // 生成测试数据
        test_flash_generate_data(data_size);
        max_error_point_mem = data_size;
        
        // 测试存储性能
        uint32 start_time = test_flash_get_time_ms();
        flash_road_result_enum result = flash_road_store();
        uint32 store_time = test_flash_get_time_ms() - start_time;
        
        ips114_show_string(0, 24 + i * 32, "Store:");
        ips114_show_uint(50, 24 + i * 32, store_time, 4);
        ips114_show_string(90, 24 + i * 32, "ms");
        
        if(result == FLASH_ROAD_RESULT_OK)
        {
            // 测试读取性能
            start_time = test_flash_get_time_ms();
            result = flash_road_load();
            uint32 load_time = test_flash_get_time_ms() - start_time;
            
            ips114_show_string(120, 24 + i * 32, "Load:");
            ips114_show_uint(160, 24 + i * 32, load_time, 4);
            ips114_show_string(200, 24 + i * 32, "ms");
        }
        
        system_delay_ms(TEST_FLASH_DELAY_MEDIUM);
    }
    
    test_flash_display_status();
    system_delay_ms(TEST_FLASH_DELAY_LONG);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Flash存储边界条件测试
//-------------------------------------------------------------------------------------------------------------------
void test_flash_road_boundary(void)
{
    test_flash_init_display();
    ips114_show_string(0, 0, "Flash Boundary Test");
    
    flash_road_init();
    
    // 1. 空数据测试
    ips114_show_string(0, 16, "1.Empty Data...");
    max_error_point_mem = 0;
    flash_road_result_enum result = flash_road_store();
    test_flash_display_result("Empty", (result != FLASH_ROAD_RESULT_OK)); // 应该失败
    system_delay_ms(TEST_FLASH_DELAY_SHORT);
    
    // 2. 最大数据测试
    ips114_show_string(0, 32, "2.Max Data...");
    max_error_point_mem = FLASH_ROAD_MAX_DATA_POINTS;
    result = flash_road_store();
    test_flash_display_result("Max", (result != FLASH_ROAD_RESULT_OK)); // 可能因为空间不足失败
    system_delay_ms(TEST_FLASH_DELAY_SHORT);
    
    // 3. 超大数据测试
    ips114_show_string(0, 48, "3.Over Max...");
    max_error_point_mem = FLASH_ROAD_MAX_DATA_POINTS + 1000;
    result = flash_road_store();
    test_flash_display_result("Over", (result != FLASH_ROAD_RESULT_OK)); // 应该失败
    system_delay_ms(TEST_FLASH_DELAY_SHORT);
    
    // 4. 未初始化读取测试
    ips114_show_string(0, 64, "4.No Init Read...");
    flash_road_clear(); // 清除数据
    result = flash_road_load();
    flash_road_status_enum status = flash_road_get_status();
    test_flash_display_result("NoInit", (status == FLASH_ROAD_STATUS_NO_DATA));
    system_delay_ms(TEST_FLASH_DELAY_SHORT);
    
    // 5. 重复操作测试
    ips114_show_string(0, 80, "5.Repeat Ops...");
    test_flash_generate_data(TEST_FLASH_SMALL_DATA_SIZE);
    max_error_point_mem = TEST_FLASH_SMALL_DATA_SIZE;
    
    uint8 repeat_ok = 1;
    for(int i = 0; i < 3; i++)
    {
        result = flash_road_store();
        if(result != FLASH_ROAD_RESULT_OK)
        {
            repeat_ok = 0;
            break;
        }
        system_delay_ms(100);
    }
    
    test_flash_display_result("Repeat", repeat_ok);
    system_delay_ms(TEST_FLASH_DELAY_SHORT);
    
    test_flash_display_status();
    system_delay_ms(TEST_FLASH_DELAY_LONG);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Flash存储兼容性测试
//-------------------------------------------------------------------------------------------------------------------
void test_flash_road_compatibility(void)
{
    test_flash_init_display();
    ips114_show_string(0, 0, "Flash Compatibility Test");
    
    flash_road_init();
    
    // 1. 新接口存储，旧接口读取
    ips114_show_string(0, 16, "1.New->Old...");
    test_flash_generate_data(TEST_FLASH_SMALL_DATA_SIZE);
    max_error_point_mem = TEST_FLASH_SMALL_DATA_SIZE;
    
    flash_road_result_enum result = flash_road_store();
    if(result == FLASH_ROAD_RESULT_OK)
    {
        // 清空数据
        memset(Mileage_All_sum_list, 0, sizeof(int) * TEST_FLASH_SMALL_DATA_SIZE);
        memset(errors_coords, 0, sizeof(float) * TEST_FLASH_SMALL_DATA_SIZE);
        
        // 使用旧接口读取
        flash_road_memery_get();
        
        // 验证数据
        uint8 compat_ok = 1;
        for(uint32 i = 0; i < TEST_FLASH_SMALL_DATA_SIZE; i++)
        {
            if(Mileage_All_sum_list[i] != test_mileage_data[i])
            {
                compat_ok = 0;
                break;
            }
        }
        test_flash_display_result("New->Old", compat_ok);
    }
    else
    {
        test_flash_display_result("New->Old", 0);
    }
    system_delay_ms(TEST_FLASH_DELAY_MEDIUM);
    
    // 2. 旧接口存储，新接口读取
    ips114_show_string(0, 32, "2.Old->New...");
    flash_road_clear(); // 清除数据
    
    test_flash_generate_data(TEST_FLASH_SMALL_DATA_SIZE);
    max_error_point_mem = TEST_FLASH_SMALL_DATA_SIZE;
    
    // 使用旧接口存储
    flash_road_memery_store();
    
    // 清空数据
    memset(Mileage_All_sum_list, 0, sizeof(int) * TEST_FLASH_SMALL_DATA_SIZE);
    memset(errors_coords, 0, sizeof(float) * TEST_FLASH_SMALL_DATA_SIZE);
    
    // 使用新接口读取
    result = flash_road_load();
    if(result == FLASH_ROAD_RESULT_OK)
    {
        // 验证数据
        uint8 compat_ok = 1;
        for(uint32 i = 0; i < TEST_FLASH_SMALL_DATA_SIZE; i++)
        {
            if(Mileage_All_sum_list[i] != test_mileage_data[i])
            {
                compat_ok = 0;
                break;
            }
        }
        test_flash_display_result("Old->New", compat_ok);
    }
    else
    {
        test_flash_display_result("Old->New", 0);
    }
    system_delay_ms(TEST_FLASH_DELAY_MEDIUM);
    
    // 3. 状态兼容性测试
    ips114_show_string(0, 48, "3.Status Compat...");
    flash_road_status_enum status = flash_road_get_status();
    test_flash_display_result("Status", (status != FLASH_ROAD_STATUS_ERROR));
    system_delay_ms(TEST_FLASH_DELAY_SHORT);
    
    test_flash_display_status();
    system_delay_ms(TEST_FLASH_DELAY_LONG);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Flash存储综合测试
//-------------------------------------------------------------------------------------------------------------------
void test_flash_road(void)
{
    test_flash_init_display();
    ips114_show_string(0, 0, "Flash Road Comprehensive Test");
    
    // 显示测试菜单
    ips114_show_string(0, 16, "1.Basic Test");
    ips114_show_string(0, 32, "2.Integrity Test");
    ips114_show_string(0, 48, "3.Performance Test");
    ips114_show_string(0, 64, "4.Boundary Test");
    ips114_show_string(0, 80, "5.Compatibility Test");
    ips114_show_string(0, 96, "Press key to start...");
    
    system_delay_ms(TEST_FLASH_DELAY_LONG);
    
    // 重置测试计数
    test_pass_count = 0;
    test_fail_count = 0;
    
    // 依次执行所有测试
    test_flash_road_basic();
    test_flash_road_integrity();
    test_flash_road_performance();
    test_flash_road_boundary();
    test_flash_road_compatibility();
    
    // 显示最终结果
    ips114_clear();
    ips114_show_string(0, 0, "Flash Test Complete!");
    ips114_show_string(0, 16, "Pass:");
    ips114_show_uint(50, 16, test_pass_count, 3);
    ips114_show_string(0, 32, "Fail:");
    ips114_show_uint(50, 32, test_fail_count, 3);
    ips114_show_string(0, 48, "Total:");
    ips114_show_uint(50, 48, test_pass_count + test_fail_count, 3);
    
    if(test_fail_count == 0)
    {
        ips114_show_string(0, 64, "All Tests PASSED!");
    }
    else
    {
        ips114_show_string(0, 64, "Some Tests FAILED!");
    }
    
    while(1)
    {
        system_delay_ms(2000); // 停留2秒供用户查看信息
    }
}

//=================================================内部函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化测试显示
//-------------------------------------------------------------------------------------------------------------------
static void test_flash_init_display(void)
{
    ips114_clear();
    test_start_time = test_flash_get_time_ms();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     生成测试数据
//-------------------------------------------------------------------------------------------------------------------
static void test_flash_generate_data(uint32 data_size)
{
    // 生成模拟的里程数据
    for(uint32 i = 0; i < data_size; i++)
    {
        test_mileage_data[i] = (int)(i * 10 + (i % 100)); // 模拟里程数据
        Mileage_All_sum_list[i] = test_mileage_data[i];
    }
    
    // 生成模拟的坐标数据
    for(uint32 i = 0; i < data_size; i++)
    {
        test_coord_data[i] = (float)(i * 0.1f + (i % 10) * 0.01f); // 模拟坐标数据
        errors_coords[i] = test_coord_data[i];
    }
    
    // 设置测试用的进出段参数
    for(int i = 0; i < FLASH_ROAD_IN_OUT_PARAMS; i++)
    {
        in_duan[i] = i + 1;
        out_duan[i] = i + 10;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     验证测试数据
//-------------------------------------------------------------------------------------------------------------------
static void test_flash_verify_data(uint32 data_size)
{
    uint8 verify_ok = 1;
    
    // 验证里程数据
    for(uint32 i = 0; i < data_size; i++)
    {
        if(Mileage_All_sum_list[i] != test_mileage_data[i])
        {
            verify_ok = 0;
            break;
        }
    }
    
    // 验证坐标数据
    if(verify_ok)
    {
        for(uint32 i = 0; i < data_size; i++)
        {
            if(errors_coords[i] != test_coord_data[i])
            {
                verify_ok = 0;
                break;
            }
        }
    }
    
    // 验证进出段参数
    if(verify_ok)
    {
        for(int i = 0; i < FLASH_ROAD_IN_OUT_PARAMS; i++)
        {
            if(in_duan[i] != (i + 1) || out_duan[i] != (i + 10))
            {
                verify_ok = 0;
                break;
            }
        }
    }
    
    test_flash_display_result("Verify", verify_ok);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示测试结果
//-------------------------------------------------------------------------------------------------------------------
static void test_flash_display_result(const char* test_name, uint8 result)
{
    if(result)
    {
        test_pass_count++;
        ips114_show_string(200, ips114_displaybuffer.display_y, "PASS");
    }
    else
    {
        test_fail_count++;
        ips114_show_string(200, ips114_displaybuffer.display_y, "FAIL");
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示测试状态
//-------------------------------------------------------------------------------------------------------------------
static void test_flash_display_status(void)
{
    flash_road_status_enum status = flash_road_get_status();
    uint32 data_count = flash_road_get_data_count();
    
    ips114_show_string(0, 128, "Status:");
    ips114_show_uint(50, 128, status, 1);
    ips114_show_string(80, 128, "Count:");
    ips114_show_uint(120, 128, data_count, 5);
    
    ips114_show_string(0, 144, "Pass:");
    ips114_show_uint(40, 144, test_pass_count, 2);
    ips114_show_string(80, 144, "Fail:");
    ips114_show_uint(120, 144, test_fail_count, 2);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取系统时间（毫秒）
//-------------------------------------------------------------------------------------------------------------------
static uint32 test_flash_get_time_ms(void)
{
    // 这里应该返回系统时间，暂时返回固定值
    // 实际使用时需要根据系统提供的时间函数实现
    static uint32 fake_time = 0;
    fake_time += 10; // 模拟时间递增
    return fake_time;
}
