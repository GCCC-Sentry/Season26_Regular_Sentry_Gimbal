/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2026-03-15
 * @FilePath: \Regular_Sentry_Gimbal\User\Software\Gimbal_Calibration.c
 * @Brief: 云台重力补偿自动标定程序
 */

#include "Gimbal_Calibration.h"
#include "Gimbal.h"
#include "Global_status.h"
#include "IMU_updata.h"
#include "User_math.h"
#include <stdio.h>
#include <math.h>

// 标定状态
typedef enum {
    CALIB_IDLE = 0,      // 空闲
    CALIB_INIT,          // 初始化
    CALIB_TESTING,       // 测试中
    CALIB_DONE,          // 完成
    CALIB_FAILED         // 失败
} CalibState_e;

// 标定数据结构
typedef struct {
    CalibState_e state;
    uint8_t test_index;          // 当前测试索引
    uint32_t start_time;      // 开始时间
    uint32_t stable_time;        // 稳定计时

    float test_k_gravity;      // 当前测试的k值
    float best_k_gravity;        // 最佳k值
    float min_error;             // 最小误差

    float test_angles[3];        // 测试角度：-15度, 0度, +15度
    uint8_t angle_index;         // 当前测试角度索引

    float accumulated_error;     // 累积误差
    uint16_t sample_count;       // 采样计数
} GimbalCalib_t;

static GimbalCalib_t calib = {0};

// 配置参数
#define CALIB_K_MIN         0.25f    // k_gravity最小值
#define CALIB_K_MAX         0.50f    // k_gravity最大值
#define CALIB_K_STEP      0.01f    // k_gravity步进
#define CALIB_STABLE_TIME   800      // 稳定时间(ms)
#define CALIB_ERROR_THRESH  0.015f   // 误差阈值(rad, 约0.86度)
#define CALIB_SAMPLE_NUM    50       // 采样次数

/**
 * @brief 启动重力补偿标定
 * @note 标定前确保云台已初始化，处于NORMAL模式
 */
void Gimbal_StartCalibration(void)
{
    if (calib.state != CALIB_IDLE) {
        printf("[Calib] Already running!\n");
      return;
    }

    printf("\n========== Gimbal Gravity Calibration Start ==========\n");
    printf("[Calib] Testing k_gravity from %.2f to %.2f, step %.2f\n",
           CALIB_K_MIN, CALIB_K_MAX, CALIB_K_STEP);

    // 初始化标定参数
    calib.state = CALIB_INIT;
    calib.test_index = 0;
    calib.angle_index = 0;
    calib.min_error = 999.0f;
    calib.best_k_gravity = 0.0f;
    calib.start_time = HAL_GetTick();

    // 设置测试角度
    calib.test_angles[0] = -15.0f;  // 向下15度
    calib.test_angles[1] = 0.0f;    // 水平
    calib.test_angles[2] = 15.0f;   // 向上15度

    printf("[Calib] Test angles: %.1f, %.1f, %.1f deg\n",
           calib.test_angles[0], calib.test_angles[1], calib.test_angles[2]);
}

/**
 * @brief 停止标定
 */
void Gimbal_StopCalibration(void)
{
    if (calib.state == CALIB_IDLE) return;

    printf("[Calib] Stopped by user\n");
    calib.state = CALIB_IDLE;
}

/**
 * @brief 获取标定状态
 */
CalibState_e Gimbal_GetCalibState(void)
{
    return calib.state;
}

/**
 * @brief 获取最佳k_gravity值
 */
float Gimbal_GetBestKGravity(void)
{
    return calib.best_k_gravity;
}

/**
 * @brief 标定任务，需要在主循环中周期调用(建议10ms)
 */
void Gimbal_CalibrationTask(void)
{
    if (calib.state == CALIB_IDLE) return;

    uint32_t current_time = HAL_GetTick();

    switch (calib.state)
    {
        case CALIB_INIT:
        {
            // 计算测试总数
            uint8_t total_tests = (uint8_t)((CALIB_K_MAX - CALIB_K_MIN) / CALIB_K_STEP) + 1;
            printf("[Calib] Total tests: %d\n", total_tests);

            // 开始第一个测试
            calib.test_k_gravity = CALIB_K_MIN;
            calib.angle_index = 0;
            calib.accumulated_error = 0.0f;
            calib.sample_count = 0;

       // 设置第一个测试角度
            Gimbal_SetPitchAngle(calib.test_angles[0]);
            Gimbal.pitch.k_gravity = calib.test_k_gravity;

            calib.stable_time = current_time;
            calib.state = CALIB_TESTING;

         printf("[Calib] Test #%d: k=%.3f, angle=%.1f deg\n",
                   calib.test_index, calib.test_k_gravity, calib.test_angles[0]);
            break;
        }

    case CALIB_TESTING:
        {
            // 等待云台稳定
            if (current_time - calib.stable_time < CALIB_STABLE_TIME) {
                break;
        }

            // 采样误差
            float target_rad = degree2rad(calib.test_angles[calib.angle_index]);
            float current_rad = Gimbal.pitch.pitch_location_now;
            float error = fabs(target_rad - current_rad);

            calib.accumulated_error += error;
            calib.sample_count++;

         // 采样完成
            if (calib.sample_count >= CALIB_SAMPLE_NUM)
            {
                float avg_error = calib.accumulated_error / calib.sample_count;

                printf("  Angle %.1f deg: avg_error = %.4f rad (%.2f deg)\n",
         calib.test_angles[calib.angle_index],
               avg_error,
              rad2degree(avg_error));

             // 切换到下一个角度
                calib.angle_index++;

                if (calib.angle_index >= 3)
            {
           // 所有角度测试完成，计算平均误差
               float total_avg_error = calib.accumulated_error / (calib.sample_count);

      printf("[Calib] k=%.3f, total_avg_error=%.4f rad (%.2f deg)\n",
              calib.test_k_gravity,
             total_avg_error,
                   rad2degree(total_avg_error));

                    // 更新最佳值
             if (total_avg_error < calib.min_error) {
            calib.min_error = total_avg_error;
                        calib.best_k_gravity = calib.test_k_gravity;
                   printf("  >>> New best k_gravity: %.3f <<<\n", calib.best_k_gravity);
                    }

                    // 切换到下一个k值
                    calib.test_k_gravity += CALIB_K_STEP;
                calib.test_index++;

                    if (calib.test_k_gravity > CALIB_K_MAX + 0.001f)
             {
                // 标定完成
                  calib.state = CALIB_DONE;

                        printf("\n========== Calibration Complete ==========\n");
                printf("[Result] Best k_gravity: %.3f\n", calib.best_k_gravity);
                  printf("[Result] Min error: %.4f rad (%.2f deg)\n",
          calib.min_error, rad2degree(calib.min_error));

                 if (calib.min_error < CALIB_ERROR_THRESH) {
                 printf("[Result] Status: SUCCESS\n");
                } else {
                  printf("[Result] Status: WARNING - Error still high\n");
                     }

                        printf("=============================\n\n");
              printf("Please update Gimbal.c line 151:\n");
                printf("  Gimbal.pitch.k_gravity = %.3ff;\n\n", calib.best_k_gravity);

                   // 应用最佳值
             Gimbal.pitch.k_gravity = calib.best_k_gravity;

                    break;
                }

                    // 准备下一轮测试
              calib.angle_index = 0;
          calib.accumulated_error = 0.0f;
        calib.sample_count = 0;

                Gimbal_SetPitchAngle(calib.test_angles[0]);
                    Gimbal.pitch.k_gravity = calib.test_k_gravity;

             calib.stable_time = current_time;

                    printf("[Calib] Test #%d: k=%.3f, angle=%.1f deg\n",
                calib.test_index, calib.test_k_gravity, calib.test_angles[0]);
                }
                else
             {
             // 切换到下一个角度
                    calib.accumulated_error = 0.0f;
                    calib.sample_count = 0;

          Gimbal_SetPitchAngle(calib.test_angles[calib.angle_index]);
                 calib.stable_time = current_time;
                }
        }

            break;
        }

        case CALIB_DONE:
     {
            // 标定完成，等待用户操作
            calib.state = CALIB_IDLE;
            break;
      }

        case CALIB_FAILED:
        {
            printf("[Calib] Failed!\n");
            calib.state = CALIB_IDLE;
            break;
        }

        default:
         break;
    }
}

/**
 * @brief 打印标定进度
 */
void Gimbal_PrintCalibProgress(void)
{
    if (calib.state == CALIB_IDLE) {
        printf("[Calib] Not running\n");
        return;
    }

    uint8_t total_tests = (uint8_t)((CALIB_K_MAX - CALIB_K_MIN) / CALIB_K_STEP) + 1;
    float progress = (float)calib.test_index / total_tests * 100.0f;

    printf("[Calib] Progress: %.1f%% (%d/%d), Current k=%.3f, Best k=%.3f (error=%.4f)\n",
       progress, calib.test_index, total_tests,
           calib.test_k_gravity, calib.best_k_gravity, calib.min_error);
}
