/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2026-03-15
 * @Brief: 云台重力补偿标定使用说明
 */

/**
 * ========== 使用方法 ==========
 *
 * 1. 将 Gimbal_Calibration.c 和 Gimbal_Calibration.h 添加到工程
 *
 * 2. 在 main.c 或 freertos.c 中添加标定任务
 *
 * 3. 在合适的地方启动标定（比如通过遥控器按键或串口命令）
 *
 * ========== 代码示例 ==========
 */

// ===== 示例1：在FreeRTOS任务中使用 =====

#include "Gimbal_Calibration.h"

// 在某个任务中添加标定逻辑
void GimbalTask(void const * argument)
{
    for(;;)
    {
        // 正常云台控制
        Gimbal_Tasks();

     // 标定任务（如果正在标定）
        Gimbal_CalibrationTask();

        osDelay(10);  // 10ms周期
    }
}

// 启动标定的触发条件（比如遥控器按键）
void CheckCalibrationTrigger(void)
{
    // 示例：左摇杆左下角 + 右摇杆右上角 = 启动标定
    if (rc_ctrl.rc.ch[0] < -600 && rc_ctrl.rc.ch[1] < -600 &&
        rc_ctrl.rc.ch[2] > 600 && rc_ctrl.rc.ch[3] > 600)
    {
      static uint8_t last_trigger = 0;
        if (!last_trigger) {
            Gimbal_StartCalibration();
            last_trigger = 1;
        }
    }
    else {
        last_trigger = 0;
    }
}


// ===== 示例2：通过串口命令启动 =====

// 在串口接收回调中
void UART_RxCallback(uint8_t *data, uint16_t len)
{
    if (strncmp((char*)data, "calib_start", 11) == 0)
    {
        Gimbal_StartCalibration();
    }
    else if (strncmp((char*)data, "calib_stop", 10) == 0)
    {
        Gimbal_StopCalibration();
    }
    else if (strncmp((char*)data, "calib_status", 12) == 0)
    {
      Gimbal_PrintCalibProgress();
    }
}


// ===== 示例3：在remote_control.c中添加 =====

#include "Gimbal_Calibration.h"

void Remote_Control_Task(void)
{
    // 检测特殊按键组合启动标定
    // 例如：左拨杆上 + 右拨杆下 + 鼠标右键
    if (rc_ctrl.rc.s[0] == RC_SW_UP &&
        rc_ctrl.rc.s[1] == RC_SW_DOWN &&
      rc_ctrl.mouse.press_r)
    {
        static uint32_t press_time = 0;
        if (press_time == 0) {
            press_time = HAL_GetTick();
        }

      // 长按2秒启动标定
        if (HAL_GetTick() - press_time > 2000) {
            Gimbal_StartCalibration();
            press_time = 0;
        }
    }
    else {
        press_time = 0;
    }
}


// ===== 示例4：在main.c的while循环中使用（裸机） =====

int main(void)
{
    // 初始化...
    Gimbal_Init();

    // 可选：启动时自动标定
    // Gimbal_StartCalibration();

    while (1)
    {
        Gimbal_Tasks();
        Gimbal_CalibrationTask();  // 添加标定任务

        HAL_Delay(10);
    }
}


/**
 * ======= 标定流程说明 ==========
 *
 * 1. 标定会测试 k_gravity 从 0.25 到 0.50，步进 0.01
 *    共 26 个测试点
 *
 * 2. 每个 k 值会测试 3 个角度：-15度、0度、+15度
 *
 * 3. 每个角度会：
 *    - 等待 800ms 稳定
 *    - 采样 50 次误差
 *    - 计算平均误差
 *
 * 4. 选择平均误差最小的 k 值作为最佳值
 *
 * 5. 标定完成后会打印：
 *    [Result] Best k_gravity: 0.387
 *    [Result] Min error: 0.0032 rad (0.18 deg)
 *    Please update Gimbal.c line 151:
 *      Gimbal.pitch.k_gravity = 0.387f;
 *
 * 6. 手动将最佳值写入 Gimbal.c 的初始化代码
 *
 * ========== 预计时间 ==========
 *
 * 总测试点：26 个 k 值 × 3 个角度 = 78 次测试
 * 每次测试：800ms 稳定 + 500ms 采样 ≈ 1.3 秒
 * 总时间：78 × 1.3 ≈ 100 秒 ≈ 1.7 分钟
 *
 * ========== 注意事项 ==========
 *
 * 1. 标定期间不要手动操作云台
 * 2. 确保云台能自由转动，没有卡死
 * 3. 标定期间会自动控制 pitch 轴，yaw 轴保持不动
 * 4. 如果标定结果误差仍然很大（>0.01 rad），可能是：
 *    - 机械结构有问题（摩擦力过大）
 *    - IMU 数据不准确
 *    - 电机控制参数不合适
 * 5. 标定完成后记得把最佳值写入代码并重新编译
 *
 * ========== 调试输出示例 ==========
 *
 * ========== Gimbal Gravity Calibration Start ==========
 * [Calib] Testing k_gravity from 0.25 to 0.50, step 0.01
 * [Calib] Test angles: -15.0, 0.0, 15.0 deg
 * [Calib] Total tests: 26
 * [Calib] Test #0: k=0.250, angle=-15.0 deg
 *   Angle -15.0 deg: avg_error = 0.0234 rad (1.34 deg)
 *   Angle 0.0 deg: avg_error = 0.0198 rad (1.13 deg)
 *   Angle 15.0 deg: avg_error = 0.0187 rad (1.07 deg)
 * [Calib] k=0.250, total_avg_error=0.0206 rad (1.18 deg)
 * [Calib] Test #1: k=0.260, angle=-15.0 deg
 * ...
 * [Calib] k=0.380, total_avg_error=0.0028 rad (0.16 deg)
 *   >>> New best k_gravity: 0.380 <<<
 * ...
 *
 * ========== Calibration Complete ==========
 * [Result] Best k_gravity: 0.380
 * [Result] Min error: 0.0028 rad (0.16 deg)
 * [Result] Status: SUCCESS
 * =======================
 *
 * Please update Gimbal.c line 151:
 *   Gimbal.pitch.k_gravity = 0.380f;
 *
 */
