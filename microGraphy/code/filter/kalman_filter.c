/*********************************************************************************************************************
* 文件名称          kalman_filter.c
* 功能说明          卡尔曼滤波器 实现文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本              备注
* 2025-09-18        LittleMaster        v1.0              创建文件
*
* 文件作用说明：
* 本文件实现了卡尔曼滤波器的核心算法，用于车辆状态估计和传感器数据融合。
********************************************************************************************************************/

#include "kalman_filter.h"
#include "math_utils.h"
#include <string.h>
#include <stdio.h>

//================================================= 全局变量定义 =================================================
kalman_filter_t g_kalman_filter;
gyro_kalman_filter_t g_gyro_filter;
static kf_performance_t s_kf_performance;

//================================================= 内部函数声明 =================================================
static void matrix_multiply(float A[][KF_STATE_DIM], float B[][KF_STATE_DIM], float C[][KF_STATE_DIM], int rows, int cols, int inner);
static void matrix_transpose(float A[][KF_STATE_DIM], float AT[][KF_STATE_DIM], int rows, int cols);
static void matrix_add(float A[][KF_STATE_DIM], float B[][KF_STATE_DIM], float C[][KF_STATE_DIM], int rows, int cols);
static void matrix_subtract(float A[][KF_STATE_DIM], float B[][KF_STATE_DIM], float C[][KF_STATE_DIM], int rows, int cols);
static bool matrix_inverse_6x6(float A[][KF_STATE_DIM], float A_inv[][KF_STATE_DIM]);
static void update_state_transition_matrix(float F[][KF_STATE_DIM], float dt);
static void update_observation_matrix(float H[][KF_STATE_DIM]);
static bool innovation_gate_check(const kf_measurement_t* measurement);

//================================================= 主要接口函数实现 =================================================

bool kalman_filter_init(const kf_state_t* initial_state)
{
    memset(&g_kalman_filter, 0, sizeof(kalman_filter_t));
    memset(&s_kf_performance, 0, sizeof(kf_performance_t));
    
    // 初始化状态向量
    if (initial_state != NULL) {
        g_kalman_filter.state[0] = initial_state->x;
        g_kalman_filter.state[1] = initial_state->y;
        g_kalman_filter.state[2] = initial_state->heading;
        g_kalman_filter.state[3] = initial_state->vx;
        g_kalman_filter.state[4] = initial_state->vy;
        g_kalman_filter.state[5] = initial_state->omega;
    }
    
    // 初始化状态协方差矩阵P (对角矩阵)
    for (int i = 0; i < KF_STATE_DIM; i++) {
        for (int j = 0; j < KF_STATE_DIM; j++) {
            g_kalman_filter.P[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    // 设置初始不确定性
    g_kalman_filter.P[0][0] = 100.0f;  // 位置X不确定性 (mm?)
    g_kalman_filter.P[1][1] = 100.0f;  // 位置Y不确定性 (mm?)
    g_kalman_filter.P[2][2] = 0.1f;    // 航向角不确定性 (rad?)
    g_kalman_filter.P[3][3] = 50.0f;   // 速度X不确定性 (mm?/s?)
    g_kalman_filter.P[4][4] = 50.0f;   // 速度Y不确定性 (mm?/s?)
    g_kalman_filter.P[5][5] = 0.01f;   // 角速度不确定性 (rad?/s?)
    
    // 初始化过程噪声协方差矩阵Q (对角矩阵)
    memset(g_kalman_filter.Q, 0, sizeof(g_kalman_filter.Q));
    g_kalman_filter.Q[0][0] = KF_PROCESS_NOISE_POS * KF_PROCESS_NOISE_POS;
    g_kalman_filter.Q[1][1] = KF_PROCESS_NOISE_POS * KF_PROCESS_NOISE_POS;
    g_kalman_filter.Q[2][2] = KF_PROCESS_NOISE_OMEGA * KF_PROCESS_NOISE_OMEGA;
    g_kalman_filter.Q[3][3] = KF_PROCESS_NOISE_VEL * KF_PROCESS_NOISE_VEL;
    g_kalman_filter.Q[4][4] = KF_PROCESS_NOISE_VEL * KF_PROCESS_NOISE_VEL;
    g_kalman_filter.Q[5][5] = KF_PROCESS_NOISE_OMEGA * KF_PROCESS_NOISE_OMEGA;
    
    // 初始化测量噪声协方差矩阵R (对角矩阵)
    memset(g_kalman_filter.R, 0, sizeof(g_kalman_filter.R));
    g_kalman_filter.R[0][0] = KF_MEASUREMENT_NOISE_ENCODER * KF_MEASUREMENT_NOISE_ENCODER;
    g_kalman_filter.R[1][1] = KF_MEASUREMENT_NOISE_ENCODER * KF_MEASUREMENT_NOISE_ENCODER;
    g_kalman_filter.R[2][2] = KF_MEASUREMENT_NOISE_GYRO * KF_MEASUREMENT_NOISE_GYRO;
    
    g_kalman_filter.initialized = true;
    return true;
}

void kalman_filter_predict(float dt)
{
    if (!g_kalman_filter.initialized) return;
    
    // 更新状态转移矩阵F
    update_state_transition_matrix(g_kalman_filter.F, dt);
    
    // 状态预测: x_k|k-1 = F * x_k-1|k-1
    float predicted_state[KF_STATE_DIM];
    float heading = g_kalman_filter.state[2];
    float vx = g_kalman_filter.state[3];
    float vy = g_kalman_filter.state[4];
    float omega = g_kalman_filter.state[5];
    
    // 非线性运动模型
    predicted_state[0] = g_kalman_filter.state[0] + vx * cosf(heading) * dt - vy * sinf(heading) * dt;
    predicted_state[1] = g_kalman_filter.state[1] + vx * sinf(heading) * dt + vy * cosf(heading) * dt;
    predicted_state[2] = normalize_angle(g_kalman_filter.state[2] + omega * dt);
    predicted_state[3] = vx;  // 假设速度变化缓慢
    predicted_state[4] = vy;
    predicted_state[5] = omega;
    
    // 更新状态
    memcpy(g_kalman_filter.state, predicted_state, sizeof(predicted_state));
    
    // 协方差预测: P_k|k-1 = F * P_k-1|k-1 * F^T + Q
    float FP[KF_STATE_DIM][KF_STATE_DIM];
    float FPF_T[KF_STATE_DIM][KF_STATE_DIM];
    float F_T[KF_STATE_DIM][KF_STATE_DIM];
    
    matrix_multiply(g_kalman_filter.F, g_kalman_filter.P, FP, KF_STATE_DIM, KF_STATE_DIM, KF_STATE_DIM);
    matrix_transpose(g_kalman_filter.F, F_T, KF_STATE_DIM, KF_STATE_DIM);
    matrix_multiply(FP, F_T, FPF_T, KF_STATE_DIM, KF_STATE_DIM, KF_STATE_DIM);
    matrix_add(FPF_T, g_kalman_filter.Q, g_kalman_filter.P, KF_STATE_DIM, KF_STATE_DIM);
    
    g_kalman_filter.last_time += dt;
}

bool kalman_filter_update(const kf_measurement_t* measurement)
{
    if (!g_kalman_filter.initialized || !measurement->valid) {
        return false;
    }
    
    // 新息门检验
    if (!innovation_gate_check(measurement)) {
        s_kf_performance.outlier_count++;
        return false;
    }
    
    // 更新观测矩阵H
    update_observation_matrix(g_kalman_filter.H);
    
    // 计算新息 (innovation): y = z - H * x
    float predicted_measurement[KF_MEASUREMENT_DIM];
    predicted_measurement[0] = g_kalman_filter.state[0];  // 预测位置X
    predicted_measurement[1] = g_kalman_filter.state[1];  // 预测位置Y
    predicted_measurement[2] = g_kalman_filter.state[5];  // 预测角速度
    
    float innovation[KF_MEASUREMENT_DIM];
    innovation[0] = measurement->encoder_x - predicted_measurement[0];
    innovation[1] = measurement->encoder_y - predicted_measurement[1];
    innovation[2] = measurement->gyro_omega - predicted_measurement[2];
    
    // 存储新息用于性能监控
    s_kf_performance.innovation_x = innovation[0];
    s_kf_performance.innovation_y = innovation[1];
    s_kf_performance.innovation_omega = innovation[2];
    
    // 计算新息协方差 S = H * P * H^T + R
    float HP[KF_MEASUREMENT_DIM][KF_STATE_DIM];
    float H_T[KF_STATE_DIM][KF_MEASUREMENT_DIM];
    float S[KF_MEASUREMENT_DIM][KF_MEASUREMENT_DIM];
    float HPH_T[KF_MEASUREMENT_DIM][KF_MEASUREMENT_DIM];
    
    // 转置H矩阵
    for (int i = 0; i < KF_MEASUREMENT_DIM; i++) {
        for (int j = 0; j < KF_STATE_DIM; j++) {
            H_T[j][i] = g_kalman_filter.H[i][j];
        }
    }
    
    // HP = H * P
    for (int i = 0; i < KF_MEASUREMENT_DIM; i++) {
        for (int j = 0; j < KF_STATE_DIM; j++) {
            HP[i][j] = 0;
            for (int k = 0; k < KF_STATE_DIM; k++) {
                HP[i][j] += g_kalman_filter.H[i][k] * g_kalman_filter.P[k][j];
            }
        }
    }
    
    // HPH_T = HP * H^T
    for (int i = 0; i < KF_MEASUREMENT_DIM; i++) {
        for (int j = 0; j < KF_MEASUREMENT_DIM; j++) {
            HPH_T[i][j] = 0;
            for (int k = 0; k < KF_STATE_DIM; k++) {
                HPH_T[i][j] += HP[i][k] * H_T[k][j];
            }
        }
    }
    
    // S = HPH_T + R
    for (int i = 0; i < KF_MEASUREMENT_DIM; i++) {
        for (int j = 0; j < KF_MEASUREMENT_DIM; j++) {
            S[i][j] = HPH_T[i][j] + g_kalman_filter.R[i][j];
        }
    }
    
    // 计算卡尔曼增益 K = P * H^T * S^(-1)
    float S_inv[KF_MEASUREMENT_DIM][KF_MEASUREMENT_DIM];
    
    // 简化的3x3矩阵求逆 (对角矩阵)
    for (int i = 0; i < KF_MEASUREMENT_DIM; i++) {
        for (int j = 0; j < KF_MEASUREMENT_DIM; j++) {
            if (i == j && fabsf(S[i][j]) > 1e-6f) {
                S_inv[i][j] = 1.0f / S[i][j];
            } else {
                S_inv[i][j] = 0.0f;
            }
        }
    }
    
    // K = P * H^T * S^(-1)
    float PH_T[KF_STATE_DIM][KF_MEASUREMENT_DIM];
    for (int i = 0; i < KF_STATE_DIM; i++) {
        for (int j = 0; j < KF_MEASUREMENT_DIM; j++) {
            PH_T[i][j] = 0;
            for (int k = 0; k < KF_STATE_DIM; k++) {
                PH_T[i][j] += g_kalman_filter.P[i][k] * H_T[k][j];
            }
        }
    }
    
    for (int i = 0; i < KF_STATE_DIM; i++) {
        for (int j = 0; j < KF_MEASUREMENT_DIM; j++) {
            g_kalman_filter.K[i][j] = 0;
            for (int k = 0; k < KF_MEASUREMENT_DIM; k++) {
                g_kalman_filter.K[i][j] += PH_T[i][k] * S_inv[k][j];
            }
        }
    }
    
    // 状态更新: x = x + K * innovation
    for (int i = 0; i < KF_STATE_DIM; i++) {
        for (int j = 0; j < KF_MEASUREMENT_DIM; j++) {
            g_kalman_filter.state[i] += g_kalman_filter.K[i][j] * innovation[j];
        }
    }
    
    // 归一化航向角
    g_kalman_filter.state[2] = normalize_angle(g_kalman_filter.state[2]);
    
    // 协方差更新: P = (I - K*H) * P
    float I[KF_STATE_DIM][KF_STATE_DIM];
    float KH[KF_STATE_DIM][KF_STATE_DIM];
    float I_KH[KF_STATE_DIM][KF_STATE_DIM];
    
    // 单位矩阵I
    for (int i = 0; i < KF_STATE_DIM; i++) {
        for (int j = 0; j < KF_STATE_DIM; j++) {
            I[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    // KH = K * H
    for (int i = 0; i < KF_STATE_DIM; i++) {
        for (int j = 0; j < KF_STATE_DIM; j++) {
            KH[i][j] = 0;
            for (int k = 0; k < KF_MEASUREMENT_DIM; k++) {
                KH[i][j] += g_kalman_filter.K[i][k] * g_kalman_filter.H[k][j];
            }
        }
    }
    
    // I_KH = I - KH
    matrix_subtract(I, KH, I_KH, KF_STATE_DIM, KF_STATE_DIM);
    
    // P = (I - KH) * P
    float temp_P[KF_STATE_DIM][KF_STATE_DIM];
    matrix_multiply(I_KH, g_kalman_filter.P, temp_P, KF_STATE_DIM, KF_STATE_DIM, KF_STATE_DIM);
    memcpy(g_kalman_filter.P, temp_P, sizeof(temp_P));
    
    g_kalman_filter.update_count++;
    return true;
}

void kalman_filter_get_state(kf_state_t* filtered_state)
{
    if (!g_kalman_filter.initialized || filtered_state == NULL) {
        return;
    }
    
    filtered_state->x = g_kalman_filter.state[0];
    filtered_state->y = g_kalman_filter.state[1];
    filtered_state->heading = g_kalman_filter.state[2];
    filtered_state->vx = g_kalman_filter.state[3];
    filtered_state->vy = g_kalman_filter.state[4];
    filtered_state->omega = g_kalman_filter.state[5];
}

void kalman_filter_reset(void)
{
    memset(&g_kalman_filter, 0, sizeof(kalman_filter_t));
    memset(&s_kf_performance, 0, sizeof(kf_performance_t));
}

//================================================= 陀螺仪专用滤波器实现 =================================================

bool gyro_kalman_filter_init(void)
{
    memset(&g_gyro_filter, 0, sizeof(gyro_kalman_filter_t));
    
    // 初始化状态 [角速度, 偏置]
    g_gyro_filter.state[0] = 0.0f;  // 初始角速度
    g_gyro_filter.state[1] = 0.0f;  // 初始偏置
    
    // 初始化协方差矩阵
    g_gyro_filter.P[0][0] = 1.0f;   // 角速度不确定性
    g_gyro_filter.P[0][1] = 0.0f;
    g_gyro_filter.P[1][0] = 0.0f;
    g_gyro_filter.P[1][1] = 0.1f;   // 偏置不确定性
    
    // 设置噪声参数
    g_gyro_filter.Q_omega = KF_PROCESS_NOISE_OMEGA * KF_PROCESS_NOISE_OMEGA;
    g_gyro_filter.Q_bias = KF_PROCESS_NOISE_BIAS * KF_PROCESS_NOISE_BIAS;
    g_gyro_filter.R_gyro = KF_MEASUREMENT_NOISE_GYRO * KF_MEASUREMENT_NOISE_GYRO;
    
    g_gyro_filter.initialized = true;
    return true;
}

float gyro_kalman_filter_update(float raw_gyro, float dt)
{
    if (!g_gyro_filter.initialized) {
        gyro_kalman_filter_init();
    }
    
    // 预测步骤
    // 状态转移：omega_k = omega_k-1, bias_k = bias_k-1
    // (角速度和偏置假设不变)
    
    // 协方差预测：P = P + Q
    g_gyro_filter.P[0][0] += g_gyro_filter.Q_omega * dt;
    g_gyro_filter.P[1][1] += g_gyro_filter.Q_bias * dt;
    
    // 更新步骤
    // 测量方程：z = omega + bias + noise
    float innovation = raw_gyro - (g_gyro_filter.state[0] + g_gyro_filter.state[1]);
    
    // 新息协方差：S = H * P * H^T + R = P[0][0] + P[1][1] + 2*P[0][1] + R
    float S = g_gyro_filter.P[0][0] + g_gyro_filter.P[1][1] + 2.0f * g_gyro_filter.P[0][1] + g_gyro_filter.R_gyro;
    
    if (fabsf(S) < 1e-6f) {
        return g_gyro_filter.state[0];  // 避免除零
    }
    
    // 卡尔曼增益：K = P * H^T / S
    float K1 = (g_gyro_filter.P[0][0] + g_gyro_filter.P[0][1]) / S;
    float K2 = (g_gyro_filter.P[1][0] + g_gyro_filter.P[1][1]) / S;
    
    // 状态更新
    g_gyro_filter.state[0] += K1 * innovation;
    g_gyro_filter.state[1] += K2 * innovation;
    
    // 协方差更新：P = (I - K*H) * P
    float temp_P00 = g_gyro_filter.P[0][0];
    float temp_P01 = g_gyro_filter.P[0][1];
    float temp_P10 = g_gyro_filter.P[1][0];
    float temp_P11 = g_gyro_filter.P[1][1];
    
    g_gyro_filter.P[0][0] = (1.0f - K1) * temp_P00 - K1 * temp_P10;
    g_gyro_filter.P[0][1] = (1.0f - K1) * temp_P01 - K1 * temp_P11;
    g_gyro_filter.P[1][0] = temp_P10 - K2 * temp_P00 - K2 * temp_P10;
    g_gyro_filter.P[1][1] = temp_P11 - K2 * temp_P01 - K2 * temp_P11;
    
    return g_gyro_filter.state[0];  // 返回滤波后的角速度
}

float gyro_kalman_filter_get_bias(void)
{
    return g_gyro_filter.state[1];
}

void gyro_kalman_set_noise(float process_noise, float measurement_noise)
{
    g_gyro_filter.Q_omega = process_noise * process_noise;
    g_gyro_filter.R_gyro = measurement_noise * measurement_noise;
}

//================================================= 辅助函数实现 =================================================

static void update_state_transition_matrix(float F[][KF_STATE_DIM], float dt)
{
    // 初始化为单位矩阵
    for (int i = 0; i < KF_STATE_DIM; i++) {
        for (int j = 0; j < KF_STATE_DIM; j++) {
            F[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    // 线性化的状态转移矩阵 (近似)
    float heading = g_kalman_filter.state[2];
    float vx = g_kalman_filter.state[3];
    float vy = g_kalman_filter.state[4];
    
    // dx/dvx = cos(heading) * dt
    F[0][3] = cosf(heading) * dt;
    // dx/dvy = -sin(heading) * dt
    F[0][4] = -sinf(heading) * dt;
    // dx/dheading = -vx*sin(heading)*dt - vy*cos(heading)*dt
    F[0][2] = (-vx * sinf(heading) - vy * cosf(heading)) * dt;
    
    // dy/dvx = sin(heading) * dt
    F[1][3] = sinf(heading) * dt;
    // dy/dvy = cos(heading) * dt
    F[1][4] = cosf(heading) * dt;
    // dy/dheading = vx*cos(heading)*dt - vy*sin(heading)*dt
    F[1][2] = (vx * cosf(heading) - vy * sinf(heading)) * dt;
    
    // dheading/domega = dt
    F[2][5] = dt;
}

static void update_observation_matrix(float H[][KF_STATE_DIM])
{
    // 清零
    for (int i = 0; i < KF_MEASUREMENT_DIM; i++) {
        for (int j = 0; j < KF_STATE_DIM; j++) {
            H[i][j] = 0.0f;
        }
    }
    
    // 观测方程：z = [x, y, omega]
    H[0][0] = 1.0f;  // 编码器X观测位置X
    H[1][1] = 1.0f;  // 编码器Y观测位置Y
    H[2][5] = 1.0f;  // 陀螺仪观测角速度
}

static bool innovation_gate_check(const kf_measurement_t* measurement)
{
    // 简单的新息门检验，检查测量值是否合理
    float max_pos_innovation = 50.0f;   // 最大位置新息 (mm)
    float max_omega_innovation = 2.0f;  // 最大角速度新息 (rad/s)
    
    float pos_innovation_x = fabsf(measurement->encoder_x - g_kalman_filter.state[0]);
    float pos_innovation_y = fabsf(measurement->encoder_y - g_kalman_filter.state[1]);
    float omega_innovation = fabsf(measurement->gyro_omega - g_kalman_filter.state[5]);
    
    if (pos_innovation_x > max_pos_innovation || 
        pos_innovation_y > max_pos_innovation ||
        omega_innovation > max_omega_innovation) {
        return false;
    }
    
    return true;
}

const kf_performance_t* kalman_filter_get_performance(void)
{
    // 计算协方差矩阵的迹
    s_kf_performance.trace_P = 0.0f;
    for (int i = 0; i < KF_STATE_DIM; i++) {
        s_kf_performance.trace_P += g_kalman_filter.P[i][i];
    }
    
    return &s_kf_performance;
}

void kalman_filter_print_debug(void)
{
    printf("=== Kalman Filter Debug Info ===\n");
    printf("State: [%.2f, %.2f, %.3f, %.2f, %.2f, %.3f]\n",
           g_kalman_filter.state[0], g_kalman_filter.state[1], g_kalman_filter.state[2],
           g_kalman_filter.state[3], g_kalman_filter.state[4], g_kalman_filter.state[5]);
    
    printf("Innovations: X=%.2f, Y=%.2f, Omega=%.3f\n",
           s_kf_performance.innovation_x, s_kf_performance.innovation_y, s_kf_performance.innovation_omega);
    
    printf("P trace: %.3f, Updates: %lu, Outliers: %lu\n",
           s_kf_performance.trace_P, g_kalman_filter.update_count, s_kf_performance.outlier_count);
    
    printf("Gyro bias: %.5f rad/s\n", gyro_kalman_filter_get_bias());
}

void kalman_filter_set_noise(float process_noise_scale, float measurement_noise_scale)
{
    // 动态调整噪声参数
    for (int i = 0; i < KF_STATE_DIM; i++) {
        g_kalman_filter.Q[i][i] *= process_noise_scale;
    }
    
    for (int i = 0; i < KF_MEASUREMENT_DIM; i++) {
        g_kalman_filter.R[i][i] *= measurement_noise_scale;
    }
}

//================================================= 矩阵运算辅助函数 =================================================

static void matrix_add(float A[][KF_STATE_DIM], float B[][KF_STATE_DIM], float C[][KF_STATE_DIM], int rows, int cols)
{
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            C[i][j] = A[i][j] + B[i][j];
        }
    }
}

static void matrix_subtract(float A[][KF_STATE_DIM], float B[][KF_STATE_DIM], float C[][KF_STATE_DIM], int rows, int cols)
{
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            C[i][j] = A[i][j] - B[i][j];
        }
    }
}

static void matrix_multiply(float A[][KF_STATE_DIM], float B[][KF_STATE_DIM], float C[][KF_STATE_DIM], int rows, int cols, int inner)
{
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            C[i][j] = 0;
            for (int k = 0; k < inner; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

static void matrix_transpose(float A[][KF_STATE_DIM], float AT[][KF_STATE_DIM], int rows, int cols)
{
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            AT[j][i] = A[i][j];
        }
    }
}
