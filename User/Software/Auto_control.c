/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2025-11-03 00:07:24
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-03-19 12:49:25
 * @FilePath: \Season26_Regular_Sentry_Gimbal\User\Software\Auto_control.c
 */

#include "Auto_control.h"
#include "Global_status.h"
#include "Gimbal.h"

#include "IMU_updata.h"
#include "dm_imu.h"
#include "referee_system.h"
#include "UART_data_txrx.h"
#include "USB_VirCom.h"

#include "CRC8_CRC16.h"
#include "string.h"

STM32_data_t toMINIPC;
MINIPC_data_t fromMINIPC;
STM32ROS_data_t stm32send_1;
Sentry_cmd_t Sentry_cmd_1;
Referee_data_t Referee;
Auto_Plan_t Auto_data;
uint8_t Debug_Sentry_Revive_State = 0;
uint32_t Debug_Sentry_Tx_Count = 0;

uint8_t data[128];
uint8_t rx_data[100];


void decodeMINIPCdata(MINIPC_data_t *target, unsigned char buff[], unsigned int len)
{
    memcpy(target, buff, len);
}

int encodeSTM32(STM32_data_t *target, unsigned char tx_buff[], unsigned int len)
{
    memcpy(tx_buff, target, len);
    return 0;
}

void decodeNAVdata(Navigation_data_t *target, unsigned char buff[], unsigned int len)
{
    if (len != 20 || buff[0] != 0xAA) {
        return;
    }

    uint16_t received_crc = (uint16_t)buff[18] | ((uint16_t)buff[19] << 8);

    uint16_t calculated_crc = Get_Modbus_CRC16(buff, 18);

    if (calculated_crc == received_crc) 
    {
        memcpy(target, buff, sizeof(Navigation_data_t));
        Navigation_online = 1;
    }
}

void Receive_from_Chassis_1(uint8_t data[8])
{
    Referee.current_HP = bytes_to_uint16(&data[0]);
    Referee.maximum_HP = bytes_to_uint16(&data[2]);
    Referee.game_type = bytes_to_uint8(&data[4]);
    Referee.game_progress = bytes_to_uint8(&data[5]);
    Referee.stage_remain_time = bytes_to_uint16(&data[6]);
}

void Receive_from_Chassis_2(uint8_t data[8])
{
    Referee.projectile_allowance_17mm = bytes_to_uint16(&data[0]);
    Referee.outpost_HP = bytes_to_uint16(&data[2]);
    Referee.base_HP = bytes_to_uint16(&data[4]);
}

void Receive_from_Chassis_3(uint8_t data[8])
{
    Referee.rfid_status = bytes_to_float(&data[0]);
}

/**
 * @brief 向导航发送裁判系统数据
 *
 */
void Navigation_Send_Message()
    {
    stm32send_1.remain_hp = Referee.current_HP;                                // 机器人实时血量
    stm32send_1.max_hp = Referee.maximum_HP;                                   // 最大血量
    stm32send_1.game_type = Referee.game_type;                                                      // 比赛类型
    stm32send_1.game_progress = Referee.game_progress;                          // 比赛进程
    stm32send_1.stage_remain_time = Referee.stage_remain_time;                  // 当前阶段剩余时间
    stm32send_1.bullet_remaining_num_17mm = Referee.projectile_allowance_17mm; // 允许发弹量
    stm32send_1.outpost_hp = Referee.outpost_HP;                               // 前哨站血量
    stm32send_1.base_hp = Referee.base_HP;                                     // 基地血量
    stm32send_1.rfid_status = Referee.rfid_status;                               // RFID状态

    static uint8_t buff[sizeof(STM32ROS_data_t) + 3];
    buff[0] = 0xA5;
    memcpy(&buff[1], &stm32send_1, sizeof(STM32ROS_data_t));
    append_CRC16_check_sum(buff, sizeof(buff));

    /* Vircom_Send(buff, sizeof(buff)); */
    HAL_UART_Transmit_DMA(UART1_data.huart, buff, sizeof(buff));
    
}



#if(AUTO_CSU == 1)

/**
 * @brief 向上位机发送自瞄相关数据
 * 
 * @param yaw yaw轴当前角度（弧度）
 * @param pitch pitch轴当前角度（弧度）
 * @param omega yaw轴当前角速度（rad/s）
 */
 void STM32_to_MINIPC()
{
    toMINIPC.header = 0xff;
    toMINIPC.ender = 0x0d;
    toMINIPC.mode = 0;
    toMINIPC.roll = imu.roll / RAD_TO_DEG;
    toMINIPC.pitch = imu.pitch / RAD_TO_DEG;
	toMINIPC.yaw = imu.yaw / RAD_TO_DEG;
    encodeSTM32(&toMINIPC, data, sizeof(STM32_data_t));
    Vircom_Send(data, sizeof(STM32_data_t));
}

void MINIPC_to_STM32()
{
    Global.Auto.input.shoot_pitch = fromMINIPC.pitch;
	Global.Auto.input.shoot_yaw = fromMINIPC.yaw;
	Global.Auto.input.fire = fromMINIPC.fire;
} 

#endif

#if(AUTO_TJU == 1)

void STM32_to_MINIPC(float yaw,float pitch,float omega)
{
    toMINIPC.FrameHeader.sof = 0xA5;
    toMINIPC.FrameHeader.crc8 = 0x00;
    toMINIPC.To_minipc_data.curr_pitch = pitch;//IMU_data.AHRS.pitch;
    toMINIPC.To_minipc_data.curr_yaw = yaw;//IMU_data.AHRS.yaw;
    toMINIPC.To_minipc_data.curr_omega = omega;//cos(IMU_data.AHRS.pitch) * IMU_data.gyro[2] - sin(IMU_data.AHRS.pitch) * IMU_data.gyro[1];
    toMINIPC.To_minipc_data.autoaim = 1;
    if (Referee_data.robot_id >= 100)
        toMINIPC.To_minipc_data.enemy_color = 1;
    else
        toMINIPC.To_minipc_data.enemy_color = 0;
    toMINIPC.To_minipc_data.state = 0;
    toMINIPC.FrameTailer.crc16 = get_CRC16_check_sum((uint8_t *)&toMINIPC.FrameHeader.sof, 17, 0xffff);
    toMINIPC.enter = 0x0A;
    encodeSTM32(&toMINIPC, data, sizeof(STM32_data_t));
    // VirCom_send(data, sizeof(STM32_data_t));
    UART_SendData(UART1_data, data, sizeof(STM32_data_t));
}

void MINIPC_to_STM32()
{
    if (fabs(fromMINIPC.from_minipc_data.shoot_yaw - IMU_data.AHRS.yaw) > PI / 2.0f) // 过零点处理
    {
        if (fromMINIPC.from_minipc_data.shoot_yaw > PI / 2.0f)
            fromMINIPC.from_minipc_data.shoot_yaw -= 2 * PI;
        else if (fromMINIPC.from_minipc_data.shoot_yaw < -PI / 2.0f)
            fromMINIPC.from_minipc_data.shoot_yaw += 2 * PI;
    }
    Global.Auto.input.shoot_pitch = fromMINIPC.from_minipc_data.shoot_pitch - IMU_data.AHRS.pitch;
    Global.Auto.input.shoot_yaw = fromMINIPC.from_minipc_data.shoot_yaw - IMU_data.AHRS.yaw;
    Global.Auto.input.fire = fromMINIPC.from_minipc_data.fire;
    Global.Auto.input.target_id = fromMINIPC.from_minipc_data.target_id;
    if(fromMINIPC.from_minipc_data.shoot_pitch==0&&fromMINIPC.from_minipc_data.shoot_yaw==0)
        Global.Auto.input.fire = -1;
}


#endif

#if(AUTO_TongJi == 1)

void STM32_to_MINIPC()
{
    toMINIPC.header[0] = 'S';
    toMINIPC.header[1] = 'P';
    toMINIPC.mode = MODE_AUTO_AIM;
    toMINIPC.yaw = degree2rad(imu_gimbal.yaw);//IMU_data.AHRS.yaw;
    toMINIPC.pitch = degree2rad(imu_gimbal.pitch);//IMU_data.AHRS.pitch;
    toMINIPC.yaw_vel = imu_gimbal.gyro[2];//IMU_data.gyro[2];
    toMINIPC.pitch_vel = imu_gimbal.gyro[1];//IMU_data.gyro[0];
    toMINIPC.q[0] = imu_gimbal.q[0];//IMU_data.AHRS.q[0];
    toMINIPC.q[1] = imu_gimbal.q[1];//IMU_data.AHRS.q[1];
    toMINIPC.q[2] = imu_gimbal.q[2];//IMU_data.AHRS.q[2];
    toMINIPC.q[3] = imu_gimbal.q[3];//IMU_data.AHRS.q[3];
    toMINIPC.bullet_speed = Referee_data.Initial_SPEED;
    toMINIPC.bullet_count = Referee_data.Launching_Frequency;
    //toMINIPC.crc16 = get_CRC16_check_sum(&toMINIPC.header,41,0xFFFF);
    int len = (uint8_t *)&toMINIPC.crc16 - (uint8_t *)&toMINIPC;
    // 此时调用 CRC，无论结构体怎么变都永远正确
    toMINIPC.crc16 = get_CRC16_check_sum(&toMINIPC.header, len, 0xFFFF);
    encodeSTM32(&toMINIPC, data, sizeof(STM32_data_t));
    Vircom_Send(data, sizeof(STM32_data_t));
}

void MINIPC_to_STM32()
{
    static float filtered_target_yaw = 0.0f;
    static float filtered_target_pitch = 0.0f;
    static uint8_t first_data = 1;
    
    // 处理跨越边界
    float target_yaw = fromMINIPC.yaw;
    float target_pitch = fromMINIPC.pitch;
    
    if (first_data)
    {
        filtered_target_yaw = target_yaw;
        filtered_target_pitch = target_pitch;
        first_data = 0;
    }
    else
    {
        // 处理 yaw 跨越边界
        float delta = target_yaw - filtered_target_yaw;
        if (delta > PI) delta -= 2.0f * PI;
        else if (delta < -PI) delta += 2.0f * PI;
        
        // 一阶低通滤波
        filtered_target_yaw += 0.9f * delta;
        filtered_target_pitch = 0.9f * target_pitch + 0.1f * filtered_target_pitch;
    }
    
    // 计算偏差
    float current_yaw_rad = degree2rad(imu_gimbal.yaw);
    float current_pitch_rad = degree2rad(imu_gimbal.pitch);
    
    float delta_yaw = filtered_target_yaw - current_yaw_rad;
    float delta_pitch = filtered_target_pitch - current_pitch_rad;
    
    // 再次处理偏差的跨越边界
    if (delta_yaw > PI) delta_yaw -= 2.0f * PI;
    else if (delta_yaw < -PI) delta_yaw += 2.0f * PI;
    // 死区处理，避免小幅抖动
/*     const float YAW_DEADZONE = 0.005f;   // 约0.34度
    const float PITCH_DEADZONE = 0.005f;

    if (fabs(delta_yaw) < YAW_DEADZONE)
        delta_yaw = 0.0f;
    if (fabs(delta_pitch) < PITCH_DEADZONE)
        delta_pitch = 0.0f; */
    
    Global.Auto.input.shoot_yaw = delta_yaw;
    Global.Auto.input.shoot_pitch = delta_pitch;
    
    // 控制模式
    if (fromMINIPC.mode == CTRL_AIM_AND_FIRE)
        Global.Auto.input.fire = 1;
    else if (fromMINIPC.mode == CTRL_AIM_ONLY)
        Global.Auto.input.fire = 0;
    else 
    {
        Global.Auto.input.fire = -1;
        first_data = 1;
    }
/*    float current_yaw_rad = degree2rad(imu_gimbal.yaw);  
    float current_pitch_rad = degree2rad(imu_gimbal.pitch); 
    // 计算目标与当前的偏差
    float delta_yaw = fromMINIPC.yaw - current_yaw_rad;
    float delta_pitch = fromMINIPC.pitch - current_pitch_rad;
    // 处理yaw角度跨越边界问题（-180°~180°）
    if (delta_yaw > PI)
        delta_yaw -= 2.0f * PI;
    else if (delta_yaw < -PI)
        delta_yaw += 2.0f * PI;
    static float last_shoot_yaw = 0.0f;
    static float last_shoot_pitch = 0.0f;
    static uint8_t first_data = 1;
    
    if (first_data)
    {
        last_shoot_yaw = delta_yaw;
        last_shoot_pitch = delta_pitch;
        first_data = 0;
    }
    else
    {
        last_shoot_yaw = 0.5f * delta_yaw + 0.5f * last_shoot_yaw;
        last_shoot_pitch = 0.5f * delta_pitch + 0.5f * last_shoot_pitch;
    }
    
    Global.Auto.input.shoot_yaw = last_shoot_yaw;
    Global.Auto.input.shoot_pitch = last_shoot_pitch;


    if (fromMINIPC.mode == CTRL_AIM_AND_FIRE)
    {
        Global.Auto.input.fire = 1;  // 控制云台且开火
    }
    else if (fromMINIPC.mode == CTRL_AIM_ONLY)
    {
        Global.Auto.input.fire = 0;  // 只控制云台不开火
    }
    else 
    {
        Global.Auto.input.fire = -1; // 不控制
        first_data = 1;
    } */
}
#endif


void Auto_Control()
{
    // shoot_yaw和shoot_pitch已经是相对偏移量（弧度）
    // 直接加到当前角度上
    
    // Yaw控制：yaw_cnt(度) + shoot_yaw(弧度转度)
    Auto_data.target_yaw = imu_gimbal.yaw_cnt + (180.0 / 3.14159265358979323846) * Global.Auto.input.shoot_yaw;
    Gimbal_SetYawAngle(Auto_data.target_yaw);
    
    // Pitch控制：pitch(度) + shoot_pitch(弧度转度)
    Auto_data.target_pitch = imu_gimbal.pitch + (180.0 / 3.14159265358979323846) * Global.Auto.input.shoot_pitch;
    Gimbal_SetPitchAngle(-Auto_data.target_pitch);
/*     static float smooth_target_yaw = 0.0f;
    static float smooth_target_pitch = 0.0f;
    static uint8_t first_run = 1;
    
    // 计算新目标
    float new_target_yaw = imu_gimbal.yaw_cnt + rad2degree(Global.Auto.input.shoot_yaw);
    float new_target_pitch = imu_gimbal.pitch + rad2degree(Global.Auto.input.shoot_pitch);
    
    if (first_run || Global.Auto.input.fire == -1)
    {
        smooth_target_yaw = new_target_yaw;
        smooth_target_pitch = new_target_pitch;
        first_run = 0;
    }
    else
    {
        // 目标平滑（避免目标突变）
        smooth_target_yaw = 0.85f * new_target_yaw + 0.15f * smooth_target_yaw;
        smooth_target_pitch = new_target_pitch /* + 0.15f * smooth_target_pitch */;
/*     }
    
    Gimbal_SetYawAngle(smooth_target_yaw);
    Gimbal_SetPitchAngle(-smooth_target_pitch);  */
}

