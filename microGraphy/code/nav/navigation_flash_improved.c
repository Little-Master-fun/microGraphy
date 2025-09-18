/*********************************************************************************************************************
* 文件名称          navigation_flash_improved.c
* 功能说明          【重构版】导航与路径跟踪系统 实现文件
* 作者              LittleMaster
* 版本信息          v3.0
* 修改记录
* 日期              作者                版本              备注
* 2025-09-18        LittleMaster        v3.0              整合状态估计与路径跟踪控制逻辑
*
* 文件作用说明：
* 本文件实现了导航系统的核心功能，包括状态估计、路径管理和路径跟踪控制。
* 通过模块化的函数设计，将复杂的导航任务分解为清晰的步骤。
********************************************************************************************************************/

#include "navigation_flash_improved.h"
#include "mpc_controller.h"
#include "driver_sch16tk10.h"
#include "driver_encoder.h"
#include "math_utils.h" // 引入新的数学工具模块

//================================================= 宏定义 =================================================
#define DEG_TO_RAD(deg) ((deg) * (M_PI / 180.0f))
#define RAD_TO_DEG(rad) ((rad) * (180.0f / M_PI))

//================================================= 全局变量定义 =================================================
NavigationSystem g_nav_system;

//================================================= 内部函数声明 =================================================
static MotionCommand navigation_stanley_control(const VehicleState* current_state);
static float calculate_curvature_from_points(const OptimalPathPoint* p1, const OptimalPathPoint* p2, const OptimalPathPoint* p3);


//================================================================================================================
//========================================= 核心功能函数实现 =====================================================
//================================================================================================================

void Navigation_Init(void)
{
    memset(&g_nav_system, 0, sizeof(NavigationSystem));
    
    // 初始化状态
    g_nav_system.state.x = 0;
    g_nav_system.state.y = 0;
    g_nav_system.state.heading = DEG_TO_RAD(90.0f); // 假设初始朝向Y轴正方向
    
    // 设置默认控制器为Stanley
    g_nav_system.controller_type = NAV_CONTROLLER_STANLEY;
    
    // 初始化MPC控制器
    mpc_weights_t mpc_weights;
    mpc_get_default_weights(&mpc_weights);
    mpc_controller_init(&mpc_weights);
    
    g_nav_system.initialized = true;
}

bool Navigation_SetControllerType(nav_controller_type_t controller_type)
{
    if (!g_nav_system.initialized) {
        return false;
    }
    
    g_nav_system.controller_type = controller_type;
    
    // 如果切换到MPC，重置MPC控制器状态
    if (controller_type == NAV_CONTROLLER_MPC) {
        mpc_reset_controller();
    }
    
    return true;
}

nav_controller_type_t Navigation_GetControllerType(void)
{
    return g_nav_system.controller_type;
}

MotionCommand Navigation_PathTrack(const VehicleState* current_state)
{
    MotionCommand cmd = {0};
    if (!g_nav_system.initialized || g_nav_system.path_point_count == 0 || current_state == NULL)
    {
        return cmd;
    }
    
    // 根据当前控制器类型选择算法
    switch (g_nav_system.controller_type) {
        case NAV_CONTROLLER_STANLEY:
            cmd = navigation_stanley_control(current_state);
            break;
            
        case NAV_CONTROLLER_MPC:
            cmd = mpc_compute_control(current_state, g_nav_system.path, 
                                    g_nav_system.path_point_count, NAV_LOOKAHEAD_DISTANCE);
            break;
            
        default:
            // 默认使用Stanley控制器
            cmd = navigation_stanley_control(current_state);
            break;
    }
    
    return cmd;
}


void Navigation_SetPath(const OptimalPathPoint* new_path, uint16_t point_count)
{
    if (point_count > NAV_MAX_PATH_POINTS) {
        point_count = NAV_MAX_PATH_POINTS;
    }
    memcpy(g_nav_system.path, new_path, sizeof(OptimalPathPoint) * point_count);
    g_nav_system.path_point_count = point_count;
}

void Navigation_GenerateTestPath(float size_mm, uint16_t point_count)
{
    if (point_count > NAV_MAX_PATH_POINTS) {
        point_count = NAV_MAX_PATH_POINTS;
    }
    
    OptimalPathPoint temp_path[NAV_MAX_PATH_POINTS];
    const float half_size = size_mm / 2.0f;
    
    const float corners[4][2] = {
        {half_size, -half_size}, {half_size, half_size},
        {-half_size, half_size}, {-half_size, -half_size}
    };

    int point_index = 0;
    int points_per_side = point_count / 4;

    for (int side = 0; side < 4; ++side) {
        float start_x = corners[side][0];
        float start_y = corners[side][1];
        float end_x = corners[(side + 1) % 4][0];
        float end_y = corners[(side + 1) % 4][1];

        for (int i = 0; i < points_per_side; ++i) {
            if (point_index >= point_count) break;
            float t = (float)i / points_per_side;
            temp_path[point_index].x = start_x + t * (end_x - start_x);
            temp_path[point_index].y = start_y + t * (end_y - start_y);
            temp_path[point_index].heading = atan2f(end_y - start_y, end_x - start_y);
            
            if (i < 5 || i >= points_per_side - 5) {
                temp_path[point_index].reference_speed = 1.5f;
            } else {
                temp_path[point_index].reference_speed = 3.5f;
            }
            point_index++;
        }
    }
    
    for (int i = 1; i < point_index - 1; ++i) {
        temp_path[i].curvature = calculate_curvature_from_points(&temp_path[i-1], &temp_path[i], &temp_path[i+1]);
    }
    temp_path[0].curvature = temp_path[1].curvature;
    temp_path[point_index - 1].curvature = temp_path[point_index - 2].curvature;
    
    Navigation_SetPath(temp_path, point_index);
}

//================================================= 内部函数实现 =================================================

static float calculate_curvature_from_points(const OptimalPathPoint* p1, const OptimalPathPoint* p2, const OptimalPathPoint* p3)
{
    float dx1 = p2->x - p1->x, dy1 = p2->y - p1->y;
    float dx2 = p3->x - p2->x, dy2 = p3->y - p2->y;
    float dx3 = p3->x - p1->x, dy3 = p3->y - p1->y;

    float area_times_2 = fabs(dx1 * dy2 - dx2 * dy1);
    float dist1 = sqrtf(dx1 * dx1 + dy1 * dy1);
    float dist2 = sqrtf(dx2 * dx2 + dy2 * dy2);
    float dist3 = sqrtf(dx3 * dx3 + dy3 * dy3);

    float denominator = dist1 * dist2 * dist3;
    if (denominator < 1e-6) return 0.0f;
    return (2.0f * area_times_2) / denominator;
}

//================================================= Stanley控制器实现 =================================================

static MotionCommand navigation_stanley_control(const VehicleState* current_state)
{
    MotionCommand cmd = {0};
    const VehicleState* state = current_state;
    OptimalPathPoint* path = g_nav_system.path;

    // 1. 查找路径上最近的点
    int closest_point_idx = 0;
    float min_dist_sq = 1e10f;
    for (int i = 0; i < g_nav_system.path_point_count; ++i) {
        float dx = path[i].x - state->x;
        float dy = path[i].y - state->y;
        float dist_sq = dx * dx + dy * dy;
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            closest_point_idx = i;
        }
    }

    // 2. 查找前瞻目标点
    int target_idx = closest_point_idx;
    float lookahead_dist_sq = NAV_LOOKAHEAD_DISTANCE * NAV_LOOKAHEAD_DISTANCE;
    for (int i = 0; i < g_nav_system.path_point_count; ++i) {
        int check_idx = (closest_point_idx + i) % g_nav_system.path_point_count;
        float dx = path[check_idx].x - state->x;
        float dy = path[check_idx].y - state->y;
        if (dx * dx + dy * dy > lookahead_dist_sq) {
            target_idx = check_idx;
            break;
        }
    }
    OptimalPathPoint* target_point = &path[target_idx];

    // ===== Stanley 控制律核心 =====
    // 3. 计算航向误差
    float heading_error = normalize_angle(target_point->heading - state->heading);

    // 4. 计算横向误差
    float dx_path = target_point->x - state->x;
    float dy_path = target_point->y - state->y;
    float lateral_error = -dx_path * sinf(target_point->heading) + dy_path * cosf(target_point->heading);
    
    // 5. 航向误差的反馈控制
    float steering_angle_heading = heading_error;

    // 6. 横向误差的反馈控制
    float steering_angle_lateral = atan2f(NAV_STANLEY_LATERAL_GAIN * lateral_error, state->linear_speed * 1000.0f + 1e-6);
    
    // 7. 组合最终转向角
    float steering_angle = steering_angle_heading + steering_angle_lateral;

    // 8. 将转向角转换为期望角速度
    cmd.desired_angular_speed = state->linear_speed * tanf(steering_angle) / (NAV_WHEELBASE / 1000.0f);

    // 9. 设定期望线速度
    cmd.desired_linear_speed = target_point->reference_speed;
    
    return cmd;
}
