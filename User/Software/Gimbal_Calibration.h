/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2026-03-15
 * @FilePath: \Regular_Sentry_Gimbal\User\Software\Gimbal_Calibration.h
 * @Brief: 云台重力补偿自动标定程序头文件
 */

#ifndef __GIMBAL_CALIBRATION_H__
#define __GIMBAL_CALIBRATION_H__

#include "stdint.h"
#include "main.h"

/**
 * @brief 启动重力补偿标定
 * @note 标定前确保：
 *       1. 云台已初始化
 *       2. 处于NORMAL模式
 *       3. 没有其他控制指令干扰
 */
void Gimbal_StartCalibration(void);

/**
 * @brief 停止标定
 */
void Gimbal_StopCalibration(void);

/**
 * @brief 获取标定状态
 * @return 0=空闲, 1=初始化, 2=测试中, 3=完成, 4=失败
 */
uint8_t Gimbal_GetCalibState(void);

/**
 * @brief 获取最佳k_gravity值
 * @return 标定得到的最佳重力补偿系数
 */
float Gimbal_GetBestKGravity(void);

/**
 * @brief 标定任务，需要在主循环中周期调用
 * @note 建议调用频率：10ms (100Hz)
 */
void Gimbal_CalibrationTask(void);

/**
 * @brief 打印标定进度
 */
void Gimbal_PrintCalibProgress(void);

#endif // __GIMBAL_CALIBRATION_H__
