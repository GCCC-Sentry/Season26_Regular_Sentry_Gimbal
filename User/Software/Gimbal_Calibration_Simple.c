// /*
//  * @Author: Nas(1319621819@qq.com)
//  * @Date: 2026-03-15
//  * @Brief: 简化版快速标定 - 直接在Gimbal.c中使用
//  */

// /**
//  * ========== 方法1：最简单的手动标定 ==========
//  *
//  * 在 Gimbal.c 的 Gimbal_Init() 中添加以下代码：
//  */

// void Gimbal_Init()
// {
//     // ... 原有初始化代码 ...

//     // 手动标定模式：取消注释下面的代码
//     #if 0  // 改为 #if 1 启用手动标定

//     printf("\n===== Manual Calibration Mode =====\n");
//     printf("Adjust k_gravity manually and observe pitch stability\n\n");

//     // 测试不同的 k 值
//     float test_k_values[] = {0.30f, 0.35f, 0.40f, 0.45f};

//     for (int i = 0; i < 4; i++)
//     {
//         Gimbal.pitch.k_gravity = test_k_values[i];

//         printf("Testing k_gravity = %.2f\n", test_k_values[i]);
//         printf("Observe pitch angle for 3 seconds...\n");

//         // 设置目标角度为0度
//         Gimbal.pitch.pitch_location_set = 0.0f;

//         // 等待3秒观察
//         uint32_t start = HAL_GetTick();
//         float max_error = 0.0f;
//         float sum_error = 0.0f;
//         int count = 0;

//         while (HAL_GetTick() - start < 3000)
//         {
//           // 需要在这里调用云台控制
//             // Gimbal_Controller();

//             float error = fabs(Gimbal.pitch.pitch_location_now - Gimbal.pitch.pitch_location_set);
//             if (error > max_error) max_error = error;
//         sum_error += error;
//         count++;

//             HAL_Delay(10);
//         }

//         float avg_error = sum_error / count;
//       printf("  Max error: %.4f rad (%.2f deg)\n", max_error, rad2degree(max_error));
//         printf("  Avg error: %.4f rad (%.2f deg)\n\n", avg_error, rad2degree(avg_error));
//     }

//     printf("===== Calibration Complete =====\n");
//     printf("Choose the k_gravity with smallest error\n\n");

//     #endif

//     // 最终使用的值
//     Gimbal.pitch.k_gravity = 0.4f;  // 根据标定结果修改这里
// }


// /**
//  * ========== 方法2：在线实时调参 ==========
//  *
//  * 在 Gimbal_Controller() 中添加实时调参功能：
//  */

// void Gimbal_Controller()
// {
//     // ... 原有代码 ...

//     // 实时调参模式：通过遥控器或串口调整 k_gravity
//     #if 0  // 改为 #if 1 启用实时调参

//     static float adjust_k = 0.4f;

//     // 示例：通过遥控器滚轮调整（假设 rc_ctrl.mouse.z 是滚轮）
//     if (rc_ctrl.mouse.z > 0) {
//         adjust_k += 0.01f;
//         if (adjust_k > 0.6f) adjust_k = 0.6f;
//         printf("k_gravity = %.3f\n", adjust_k);
//     }
//     else if (rc_ctrl.mouse.z < 0) {
//         adjust_k -= 0.01f;
//         if (adjust_k < 0.2f) adjust_k = 0.2f;
//         printf("k_gravity = %.3f\n", adjust_k);
//     }

//     Gimbal.pitch.k_gravity = adjust_k;

//     // 打印当前误差
//     static uint32_t last_print = 0;
//     if (HAL_GetTick() - last_print > 500) {
//         float error = Gimbal.pitch.pitch_location_set - Gimbal.pitch.pitch_location_now;
//         printf("k=%.3f, error=%.4f rad (%.2f deg)\n",
//                adjust_k, error, rad2degree(error));
//      last_print = HAL_GetTick();
//     }

//     #endif
// }


// /**
//  * ========== 方法3：一键快速标定 ==========
//  *
//  * 添加一个简单的标定函数，在需要时调用：
//  */
// void Gimbal_QuickCalibration(void)
// {
//     printf("\n===== Quick Calibration Start =====\n");

//     float best_k = 0.4f;
//     float min_error = 999.0f;

//     // 测试范围：0.30 ~ 0.50，步进 0.02
//     for (float k = 0.30f; k <= 0.50f; k += 0.02f)
//     {
//         Gimbal.pitch.k_gravity = k;
//         Gimbal.pitch.pitch_location_set = 0.0f;  // 目标0度

//         // 等待稳定
//         HAL_Delay(500);

//         // 采样误差
//         float sum_error = 0.0f;
//         for (int i = 0; i < 20; i++)
//         {
//             // 需要调用云台控制
//             Gimbal_Controller();

//          float error = fabs(Gimbal.pitch.pitch_location_now - Gimbal.pitch.pitch_location_set);
//             sum_error += error;

//             HAL_Delay(50);
//         }

//       float avg_error = sum_error / 20.0f;

//         printf("k=%.2f, error=%.4f rad (%.2f deg)", k, avg_error, rad2degree(avg_error));

//       if (avg_error < min_error) {
//             min_error = avg_error;
//           best_k = k;
//             printf(" <- Best");
//         }
//         printf("\n");
//     }

//     printf("\n===== Result =====\n");
//     printf("Best k_gravity: %.2f\n", best_k);
//     printf("Min error: %.4f rad (%.2f deg)\n", min_error, rad2degree(min_error));
//     printf("==================\n\n");

//     // 应用最佳值
//     Gimbal.pitch.k_gravity = best_k;
// }


// /**
//  * ========== 方法4：通过串口命令标定 ==========
//  *
//  * 在串口接收处理中添加：
//  */

// void Process_UART_Command(uint8_t *cmd)
// {
//     if (strcmp((char*)cmd, "calib") == 0)
//     {
//         // 启动快速标定
//         Gimbal_QuickCalibration();
//     }
//     else if (strncmp((char*)cmd, "setk ", 5) == 0)
//     {
//         // 手动设置 k 值
//       // 例如：发送 "setk 0.38" 设置 k_gravity = 0.38
//         float k = atof((char*)cmd + 5);
//         if (k >= 0.2f && k <= 0.6f) {
//             Gimbal.pitch.k_gravity = k;
//             printf("k_gravity set to %.3f\n", k);
//         }
//   }
//     else if (strcmp((char*)cmd, "getk") == 0)
//     {
//       // 查询当前 k 值和误差
//         float error = Gimbal.pitch.pitch_location_set - Gimbal.pitch.pitch_location_now;
//         printf("k_gravity=%.3f, error=%.4f rad (%.2f deg)\n",
//                Gimbal.pitch.k_gravity, error, rad2degree(error));
//     }
// }


// /**
//  * ========== 推荐使用流程 ==========
//  *
//  * 1. 先用方法3（一键快速标定）快速找到大致范围
//  *    - 在 main.c 或某个初始化函数中调用 Gimbal_QuickCalibration()
//  *    - 预计耗时：约 10-15 秒
//  *
//  * 2. 然后用方法2（实时调参）精细调整
//  *    - 通过遥控器或串口微调 k_gravity
//  *    - 观察不同角度的稳定性
//  *
//  * 3. 最后把最佳值写入 Gimbal.c 的初始化代码
//  *
//  * ========== 如果你想用完整的自动标定 ==========
//  *
//  * 使用我之前写的 Gimbal_Calibration.c：
//  *
//  * 1. 在 freertos.c 的 GimbalTask 中添加：
//  */

// void GimbalTask(void const * argument)
// {
//     // 等待系统初始化完成
//     osDelay(1000);

//     // 可选：启动时自动标定
//     // Gimbal_StartCalibration();

//     for(;;)
//     {
//         Gimbal_Tasks();
//         Gimbal_CalibrationTask();  // 添加这一行

//         osDelay(10);
//     }
// }

// /*
//  * 2. 通过遥控器或串口触发标定：
//  *    在 remote_control.c 中添加按键检测，调用 Gimbal_StartCalibration()
//  *
//  * 3. 标定完成后，串口会打印最佳 k_gravity 值
//  *
//  * 4. 手动修改 Gimbal.c line 151，写入最佳值
//  */
