// /*
//  * @Author: Nas(1319621819@qq.com)
//  * @Date: 2026-03-15
//  * @Brief: 云台重力补偿标定 - 完整指南
//  */

// /**
//  * =============
//  *  问题回顾：为什么需要标定 k_gravity？
//  * =======================
//  *
//  * 你遇到的问题：
//  * 1. 自瞄锁定目标后，pitch 抬起或低下，回到的位置不一样
//  * 2. 存在 0.04 rad (约2.3度) 的稳态误差
//  *
//  * 根本原因：
//  * - Pitch 使用 MIT 模式控制，只有 Kp 和 Kd，没有积分项
//  * - 重力补偿系数 k_gravity 不准确
//  * - 无法自动消除重力和摩擦力导致的稳态误差
//  *
//  * 解决方案：
//  * 方案A：精确标定 k_gravity（推荐）
//  * 方案B：添加软件积分补偿（如果方案A效果不好）
//  */


// /**
//  * =================
//  *  方案对比：选择适合你的方法
//  * =============
//  *
//  * ┌─────────────┬──────────┬──────────┬────────────┐
//  * │   方法    │  耗时    │  精度    │  难度    │  推荐度    │
//  * ├─────────────┼──────────┼──────────┼──────────┼────────────┤
//  * │ 方法1：手动 │  5分钟   │  中等    │  简单    │  ★★★      │
//  * │ 方法2：实时 │  2分钟   │  高      │  简单    │  ★★★★★  │
//  * │ 方法3：快速 │  15秒    │  中等    │  简单    │  ★★★★    │
//  * │ 方法4：完整 │  2分钟   │  很高    │  中等    │  ★★★★    │
//  * └─────────┴──────────┴──────────┴──────────┴────────┘
//  *
//  * 推荐流程：
//  * 1. 先用【方法3：快速标定】找到大致范围（0.30-0.50）
//  * 2. 再用【方法2：实时调参】精细调整（±0.02）
//  * 3. 如果需要更精确，用【方法4：完整自动标定】
//  */


// /**
//  * ===================
//  *  快速上手：5分钟解决问题
//  * ==============================
//  */

// // ===== 步骤1：添加快速标定函数到 Gimbal.c =====

// void Gimbal_QuickCalibration(void)
// {
//     printf("\n===== Quick Calibration =====\n");

//     float best_k = 0.4f;
//     float min_error = 999.0f;

//     // 测试 0.30 ~ 0.50，步进 0.02
//     for (float k = 0.30f; k <= 0.50f; k += 0.02f)
//     {
//         Gimbal.pitch.k_gravity = k;
//         Gimbal_SetPitchAngle(0.0f);  // 目标0度

//         HAL_Delay(500);  // 等待稳定

//         // 采样20次
//         float sum_error = 0.0f;
//       for (int i = 0; i < 20; i++)
//         {
//             float error = fabs(Gimbal.pitch.pitch_location_now);
//             sum_error += error;
//           HAL_Delay(50);
//         }

//     float avg_error = sum_error / 20.0f;
//         printf("k=%.2f, error=%.4f rad (%.2f deg)",
//                k, avg_error, rad2degree(avg_error));

//         if (avg_error < min_error) {
//             min_error = avg_error;
//       best_k = k;
//             printf(" <- Best");
//         }
//         printf("\n");
//     }

//     printf("\nBest k_gravity: %.2f\n", best_k);
//     Gimbal.pitch.k_gravity = best_k;
// }


// // ===== 步骤2：在 main.c 或 freertos.c 中调用 =====

// // 如果是裸机（main.c）：
// int main(void)
// {
//     // ... 初始化代码 ...
//     Gimbal_Init();

//     // 启动标定（只需要运行一次）
//     Gimbal_QuickCalibration();

//     while (1)
//     {
//         Gimbal_Tasks();
//         HAL_Delay(10);
//     }
// }

// // 如果是 FreeRTOS（freertos.c）：
// void GimbalTask(void const * argument)
// {
//     osDelay(1000);  // 等待初始化

//     // 启动标定（只需要运行一次）
//     // Gimbal_QuickCalibration();

//     for(;;)
//     {
//         Gimbal_Tasks();
//         osDelay(10);
//     }
// }


// // ===== 步骤3：查看串口输出，找到最佳值 =====

// /*
// 输出示例：

// ===== Quick Calibration =====
// k=0.30, error=0.0234 rad (1.34 deg)
// k=0.32, error=0.0198 rad (1.13 deg)
// k=0.34, error=0.0156 rad (0.89 deg)
// k=0.36, error=0.0089 rad (0.51 deg)
// k=0.38, error=0.0034 rad (0.19 deg) <- Best
// k=0.40, error=0.0067 rad (0.38 deg)
// k=0.42, error=0.0123 rad (0.70 deg)
// k=0.44, error=0.0189 rad (1.08 deg)
// k=0.46, error=0.0245 rad (1.40 deg)
// k=0.48, error=0.0298 rad (1.71 deg)
// k=0.50, error=0.0356 rad (2.04 deg)

// Best k_gravity: 0.38
// */


// // ===== 步骤4：修改 Gimbal.c 的初始化代码 =====

// void Gimbal_Init()
// {
//     // ... 其他初始化代码 ...

//     // 修改这一行，使用标定得到的值
//     Gimbal.pitch.k_gravity = 0.38f;  // 原来是 0.4f

//     // ... 其他初始化代码 ...
// }


// /**
//  * =========
//  *  进阶：实时调参（最推荐）
//  * ====================
//  */

// // 在 Gimbal.c 中添加全局变量
// float g_calib_k = 0.4f;
// uint8_t g_calib_mode = 0;  // 0=关闭, 1=开启

// // 在 Gimbal_Controller() 中添加
// void Gimbal_Controller()
// {
//     // ... 原有代码 ...

//     // 实时调参模式
//     if (g_calib_mode)
//     {
//         Gimbal.pitch.k_gravity = g_calib_k;

//         // 每500ms打印一次误差
//         static uint32_t last_print = 0;
//         if (HAL_GetTick() - last_print > 500)
//         {
//         float error = Gimbal.pitch.pitch_location_set - Gimbal.pitch.pitch_location_now;
//             printf("k=%.3f, error=%.4f rad (%.2f deg)\n",
//                    g_calib_k, error, rad2degree(error));
//         last_print = HAL_GetTick();
//         }

//     // ... 原有代码 ...
// }

// // 通过串口命令控制
// void Process_UART_Command(uint8_t *cmd)
// {
//     if (strcmp((char*)cmd, "calib on") == 0)
//     {
//       g_calib_mode = 1;
//         printf("Calibration mode ON\n");
//     }
//     else if (strcmp((char*)cmd, "calib off") == 0)
//     {
//         g_calib_mode = 0;
//         printf("Calibration mode OFF\n");
//     }
//     else if (strncmp((char*)cmd, "k ", 2) == 0)
//     {
//         // 发送 "k 0.38" 设置 k_gravity = 0.38
//         float k = atof((char*)cmd + 2);
//         if (k >= 0.2f && k <= 0.6f) {
//             g_calib_k = k;
//             printf("k_gravity = %.3f\n", k);
//         }
//     }
//     else if (strcmp((char*)cmd, "k+") == 0)
//     {
//         g_calib_k += 0.01f;
//         if (g_calib_k > 0.6f) g_calib_k = 0.6f;
//         printf("k_gravity = %.3f\n", g_calib_k);
//     }
//     else if (strcmp((char*)cmd, "k-") == 0)
//     {
//         g_calib_k -= 0.01f;
//         if (g_calib_k < 0.2f) g_calib_k = 0.2f;
//         printf("k_gravity = %.3f\n", g_calib_k);
//     }
// }

// /*
// 使用方法：
// 1. 串口发送 "calib on" 开启调参模式
// 2. 串口发送 "k 0.35" 设置初始值
// 3. 观察云台稳定性和串口打印的误差
// 4. 串口发送 "k+" 或 "k-" 微调
// 5. 找到误差最小的值
// 6. 串口发送 "calib off" 关闭调参模式
// 7. 把最佳值写入代码
// */


// /**
//  * ==============
//  *  方案B：添加软件积分补偿
//  * ===================
//  *
//  * 如果标定 k_gravity 后效果还不理想，可以添加积分补偿
//  */

// void Gimbal_Controller()
// {
//     // ... 原有代码 ...

//     if (Gimbal.pitch.init_flag == 1)
//     {
//         // 计算位置误差
//         float pitch_error = Gimbal.pitch_location_set - Gimbal.pitch.pitch_location_now;

//         // 积分项（带限幅防饱和）
//         static float pitch_integral = 0.0f;
//         const float Ki = 0.08f;  // 积分系数
//         const float integral_limit = 0.15f;  // 积分限幅

//         // 只在误差较小时累积积分（避免大误差时积分饱和）
//         if (fabs(pitch_error) < 0.1f)  // 小于5.7度才累积
//         {
//             pitch_integral += pitch_error * 0.001f;  // 假设1ms周期

//             // 限幅
//             if (pitch_integral > integral_limit) pitch_integral = integral_limit;
//          if (pitch_integral < -integral_limit) pitch_integral = -integral_limit;
//         }

//       // 自瞄失效时清零积分
//         if (Global.Auto.input.fire == -1)
//         {
//         pitch_integral = 0.0f;
//         }

//         // 重力补偿 + 积分补偿
//         float total_feedforward = Gimbal.pitch.gravity_compensation + Ki * pitch_integral;

//         float pitch_target_motor = Gimbal.pitch.pitch_location_set + Gimbal.pitch.pitch_offset;

//         DMMotor_Set(PITCHMotor,
//               pitch_target_motor,
//                     0,
//                total_feedforward,  // 使用总前馈
//                Gimbal.pitch.kp,
//                Gimbal.pitch.kd);
//     }

//     // ... 原有代码 ...
// }


// /**
//  * ======================
//  *  常见问题 FAQ
//  * =============
//  *
//  * Q1: 标定后误差还是很大怎么办？
//  * A1: 可能原因：
//  *     - 机械摩擦力过大 → 检查机械结构
//  *     - IMU 数据漂移 → 检查 IMU 校准
//  *     - 电机参数不对 → 检查 Kp、Kd 值
//  *     - 建议添加软件积分补偿（方案B）
//  *
//  * Q2: 不同角度的误差不一样？
//  * A2: 正常现象，因为：
//  *     - 不同角度的摩擦力不同
//  *     - cos(θ) 的非线性
//  *     - 可以用积分补偿自动修正
//  *
//  * Q3: 标定需要多久运行一次？
//  * A3: 通常只需要标定一次，除非：
//  *     - 更换了云台负载（相机、线缆）
//  *     - 机械结构有改动
//  *     - 发现误差明显变大
//  *
//  * Q4: 能不能自动保存标定结果？
//  * A4: 可以，需要：
//  *     - 使用 Flash 存储标定值
//  *     - 启动时从 Flash 读取
//  *     - 但建议还是手动写入代码更可靠
//  *
//  * Q5: 标定时云台乱动怎么办？
//  * A5: 检查：
//  *     - 是否有其他控制指令干扰
//  *     - 是否处于 NORMAL 模式
//  *     - PID 参数是否合理
//  */


// /**
//  * =============
//  *  总结：推荐的完整流程
//  * ================
//  *
//  * 1. 【快速标定】找到大致范围（15秒）
//  *    - 添加 Gimbal_QuickCalibration() 函数
//  *    - 运行一次，查看串口输出
//  *    - 得到最佳 k_gravity（比如 0.38）
//  *
//  * 2. 【实时调参】精细调整（2分钟）
//  *    - 添加实时调参代码
//  *    - 通过串口命令微调 ±0.02
//  *    - 测试不同角度的稳定性
//  *
//  * 3. 【写入代码】固化参数
//  *    - 修改 Gimbal.c line 151
//  *    - Gimbal.pitch.k_gravity = 0.38f;
//  *    - 重新编译烧录
//  *
//  * 4. 【验证效果】测试
//  *    - 自瞄锁定目标
//  *    - 手动抬起/低下 pitch
//  *    - 观察是否回到同一位置
//  *    - 误差应该 < 0.005 rad (0.3度)
//  *
//  * 5. 【可选】添加积分补偿
//  *    - 如果效果还不够好
//  *    - 添加方案B的积分补偿代码
//  *    - 进一步减小稳态误差
//  *
//  * 预期效果：
//  * - 标定前：误差 0.04 rad (2.3度)，不同方向回到不同位置
//  * - 标定后：误差 < 0.005 rad (0.3度)，回到相同位置
//  * - 加积分：误差 < 0.002 rad (0.1度)，完美跟踪
//  */
