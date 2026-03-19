/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2025-11-03 00:07:24
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-03-19 05:10:44
 * @FilePath: \Season26_Regular_Sentry_Gimbal\User\Software\Gimbal.c
 */
/*
  ****************************(C) COPYRIGHT 2026 ADAM****************************
  * @file       gimbal.c/h
  * @brief      云台控制器
  * @note       包括初始化，数据更新、控制量计算与直接控制量设置
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0   2025.10.31       Wang Zihao       1.重新构建云台代码结构
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 ADAM****************************
*/
#include "Gimbal.h"
#include "Global_status.h"

#include "User_math.h"
#include "remote_control.h"
#include "IMU_updata.h"
#include "dm_imu.h"
#include "hipnuc_dec.h"
#include "motor.h"
#include "vofa+.h"


Gimbal_t Gimbal;
float relative_angle;

void Gimbal_Limit(float pitch_up_angle, float pitch_down_angle, float yaw_L_angle, float yaw_R_angle)
{
    if (Gimbal.pitch.pitch_location_set <= pitch_up_angle && Gimbal.pitch.pitch_location_set < Gimbal.pitch.pitch_location_now)
    {
        Gimbal.pitch.pitch_location_set = pitch_up_angle;
    }
    else if (Gimbal.pitch.pitch_location_set >= pitch_down_angle && Gimbal.pitch.pitch_location_set > Gimbal.pitch.pitch_location_now)
    {
        Gimbal.pitch.pitch_location_set = pitch_down_angle;
    }
}

/**
 * @brief 扫描模式
 * @author Nas(1319621819@qq.com)
 * @note  在MIT模式下，通过平滑改变目标位置（Setpoint）来模拟速度控制
 */
void Scan() 
{
    const float PITCH_MAX = PITCHI_MAX_ANGLE;    
    const float PITCH_MIN = PITCHI_MIN_ANGLE;   
    const float STEP = 0.30f;                    

    // 小 Yaw 扫描参数：相对大 Yaw 往复扫描，自动适配大 Yaw 任意旋转
    const float SCAN_LIMIT = 55.0f;   // ±55° 相对大Yaw（机械限位±60°，留5°安全余量）
    const float SCAN_STEP  = 0.18f;   // 90°/s

    static float scan_pitch_deg = 0.0f;
    static float scan_offset    = 0.0f;  // 期望的小Yaw相对大Yaw的偏移角度
    static int8_t yaw_dir       = -1;
    static uint8_t init_done    = 0;
    

    // 初始同步
    if (!init_done) {
        scan_pitch_deg = rad2degree(Gimbal.pitch.pitch_location_now);
        scan_offset = 0.0f;
        init_done = 1;
    }

    // 读取 IMU 实际角度 (度)
    float phys_now = rad2degree(Gimbal.pitch.pitch_location_now);

    if (Gimbal.pitch.turnover_pitch == 1) // 向上扫描
    {
        scan_pitch_deg += STEP;
        // 如果物理位置已经接近上限，或者目标值已经严重越界，则换向
        if (phys_now >= (PITCH_MAX - 1.0f) || scan_pitch_deg > (PITCH_MAX + 5.0f)) {
            Gimbal.pitch.turnover_pitch = 0;
        }
    }
    else // 向下扫描
    {
        scan_pitch_deg -= STEP;
        if (phys_now <= (PITCH_MIN + 1.0f) || scan_pitch_deg < (PITCH_MIN - 5.0f)) {
            Gimbal.pitch.turnover_pitch = 1;
        }
    }

    // 限制 scan_pitch_deg 范围，防止目标值无限累加
    if (scan_pitch_deg > PITCH_MAX + 5.0f) scan_pitch_deg = PITCH_MAX + 5.0f;
    if (scan_pitch_deg < PITCH_MIN - 5.0f) scan_pitch_deg = PITCH_MIN - 5.0f;

    // 同步给全局变量，防止被 Updater 覆盖
    Global.Gimbal.input.pitch = scan_pitch_deg;
    Gimbal.pitch.pitch_location_set = degree2rad(scan_pitch_deg);

    // --- 小 Yaw 轴：相对大 Yaw 往复扫描 ---
    // scan_offset 在 ±SCAN_LIMIT 内振荡
    scan_offset += yaw_dir * SCAN_STEP;
    if (scan_offset >= SCAN_LIMIT)  
    { 
        scan_offset = SCAN_LIMIT;  
        yaw_dir = -1; 
    }
    if (scan_offset <= -SCAN_LIMIT) 
    { 
        scan_offset = -SCAN_LIMIT; 
        yaw_dir =  1; 
    }

    // 通过编码器获取小Yaw相对大Yaw的实际角度，反推大Yaw的世界系角度
    // encoder_relative: 小Yaw相对大Yaw的物理角度
    float encoder_relative = (float)DJIMotor_GetData(SMALLYAWMotor).angle;
    // big_yaw_world = 小Yaw世界角(IMU) - 小Yaw相对大Yaw的偏移(编码器)
    float big_yaw_world = Gimbal.small_yaw.small_yaw_location_now - encoder_relative;

    // 目标 = 大Yaw当前世界角 + 期望的相对偏移
    // 等效于 PID 误差 = scan_offset - encoder_relative，直接控制相对角度
    Gimbal.small_yaw.small_yaw_location_set = big_yaw_world + scan_offset;
}

void Arm()
{
    
}
/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param          none
 * @retval         none
 */
void Gimbal_Init()
{
    // 云台电机初始化

    DMMotor_Init(DM_4310, PITCHMotor);
    DJIMotor_Init(DJI_GM6020, SMALLYAWMotor);
    
    /*------MIT模式参数初始化------*/
    Gimbal.pitch.kp = 25.0f;  // 位置比例增益 
    Gimbal.pitch.kd = 2.0f;   // 速度阻尼增益
    Gimbal.pitch.k_gravity = 0.891800784f; // 重力前馈系数
    
    /*PID速度环初始化*/ 
    // 遥控
    PID_Set(&Gimbal.pitch.pitch_speed_pid, 0.165f, 0.0f, 0.2f, 0.0f, 7, 0.1);
    PID_Set(&Gimbal.small_yaw.small_yaw_speed_pid, 50.0f, 0.0f, 1.0f, 0.0f, GIMBALMOTOR_MAX_CURRENT, GIMBALMOTOR_MAX_CURRENT);
    PID_Set(&Gimbal.pitch.pitch_auto_speed_pid, 0.165f, 0, 0.2, 0.0f, 7, 0.1);
    PID_Set(&Gimbal.small_yaw.small_yaw_auto_speed_pid, 78.0f, 0.0f, 3.0f, 0.0f, GIMBALMOTOR_MAX_CURRENT, GIMBALMOTOR_MAX_CURRENT);
    /*PID位置环初始化*/
    // 遥控
    PID_Set(&Gimbal.pitch.pitch_location_pid, 16.0f, 0.0f, 10.0f, 0.0f, 12.57, 1000);
    PID_Set(&Gimbal.small_yaw.small_yaw_location_pid, 4.0f, 0.0f, 1.0f, 0.0f, GIMBALMOTOR_MAX_CURRENT, 1000);
    // 自瞄
    PID_Set(&Gimbal.pitch.pitch_auto_location_pid, 16.0f, 0.0f, 10, 0.0f, 12.57, 100);
    PID_Set(&Gimbal.small_yaw.small_yaw_auto_location_pid, 13.0f, 0.0f, 3.5f, 0.0f, GIMBALMOTOR_MAX_CURRENT, 1000);

    // 云台零点初始化
    DJIMotor_SetZero(SMALL_YAW_ZERO,SMALLYAWMotor);
    
    // 初始化校准变量
    Gimbal.pitch.init_flag = 0;
    Gimbal.pitch.pitch_offset = 0.0f;
}


/*-------------------- Update --------------------*/
 
/**
 * @brief          控制量更新（包括状态量和目标量）
 * @param          none
 * @retval         none           
 */
void Gimbal_Updater()
{
    /*------状态量更新------*/
    //速度                                       
    Gimbal.small_yaw.small_yaw_speed_now = (cos(degree2rad(imu_gimbal.pitch)) * rad2degree(imu_gimbal.gyro[2]) - sin(degree2rad(imu_gimbal.pitch)) * rad2degree(imu_gimbal.gyro[0]));
    Gimbal.pitch.pitch_speed_now = -imu_gimbal.gyro[1];
    //位置

    Gimbal.small_yaw.small_yaw_location_now = imu_gimbal.yaw_cnt; // 使用连续累积角度，避免±180°跳变
    Gimbal.pitch.pitch_location_now = degree2rad(-imu_gimbal.pitch);

    /*------上电初始化校准(计算IMU和电机坐标系的差值)------*/
    if (Gimbal.pitch.init_flag == 0)
    {
        // 检查电机是否在线 (ID非0表示已接收到至少一帧反馈数据)
        DM_motor_data_s pitch_motor_data = DMMotor_GetData(PITCHMotor);
        if (pitch_motor_data.motor_data.para.id != 0) 
        {
            float motor_pos = pitch_motor_data.motor_data.para.pos; // 电机位置(角度转弧度)
            Gimbal.pitch.pitch_offset = motor_pos - Gimbal.pitch.pitch_location_now; // offset = Motor - IMU
            
            // 同步目标位置
            Global.Gimbal.input.pitch = -imu_gimbal.pitch;
            
            Gimbal.pitch.init_flag = 1;
        }
    }

    /*------目标量更新------*/

    //位置



    Gimbal.pitch.pitch_location_set = degree2rad(Global.Gimbal.input.pitch);
    Gimbal.small_yaw.small_yaw_location_set = Global.Gimbal.input.yaw; 
    // 假设 Gimbal.small_yaw.small_yaw_location_now 是当前小yaw的角度
/*     IMU_Rotate_Frame(Gimbal.small_yaw.small_yaw_location_now,&imu_gimbal); */
}


/*-------------------- Calculate --------------------*/

/**
 * @brief          控制量解算
 * @param          none
 * @retval         none
 */
void Gimbal_Calculater()
{
 
    if ((Global.Auto.input.Auto_control_online <= 0 || Global.Auto.mode == NONE || Global.Auto.input.fire == -1) && Global.Gimbal.mode == NORMAL)
    { 
        // 非自瞄 
        Gimbal.small_yaw.small_yaw_speed_set =  PID_Cal(&Gimbal.small_yaw.small_yaw_location_pid, Gimbal.small_yaw.small_yaw_location_now,Gimbal.small_yaw.small_yaw_location_set);  
        Gimbal.small_yaw.current = PID_Cal(&Gimbal.small_yaw.small_yaw_speed_pid, Gimbal.small_yaw.small_yaw_speed_now, Gimbal.small_yaw.small_yaw_speed_set); 
        
        if (Global.Auto.input.Auto_control_online > 0)
            Global.Auto.input.Auto_control_online--;
    }
    else
    { 
        Gimbal.small_yaw.small_yaw_speed_set = PID_Cal(&Gimbal.small_yaw.small_yaw_auto_location_pid, Gimbal.small_yaw.small_yaw_location_now, Gimbal.small_yaw.small_yaw_location_set);
        Gimbal.small_yaw.current = PID_Cal(&Gimbal.small_yaw.small_yaw_auto_speed_pid, Gimbal.small_yaw.small_yaw_speed_now, Gimbal.small_yaw.small_yaw_speed_set);
        
        Global.Auto.input.Auto_control_online--;
    }
}
/* void Gimbal_Calculater()
{
    if ((Global.Auto.input.Auto_control_online <= 0 || Global.Auto.mode == NONE || Global.Auto.input.fire == -1) && Global.Gimbal.mode == NORMAL)
    { 
        // 手动控制（保持不变）
        Gimbal.small_yaw.small_yaw_speed_set = PID_Cal(&Gimbal.small_yaw.small_yaw_location_pid, 
                                                        Gimbal.small_yaw.small_yaw_location_now,
                                                        Gimbal.small_yaw.small_yaw_location_set);  
        Gimbal.small_yaw.current = PID_Cal(&Gimbal.small_yaw.small_yaw_speed_pid, 
                                            Gimbal.small_yaw.small_yaw_speed_now, 
                                            Gimbal.small_yaw.small_yaw_speed_set); 
        
        if (Global.Auto.input.Auto_control_online > 0)
            Global.Auto.input.Auto_control_online--;
    }
    else
    { 
        // ===== 三段式自适应控制系统 =====
        
        float yaw_error = Gimbal.small_yaw.small_yaw_location_set - Gimbal.small_yaw.small_yaw_location_now;
        float yaw_error_abs = fabs(yaw_error);
        
        // 静态变量
        static float last_yaw_target = 0.0f;
        static float yaw_integral = 0.0f;
        static uint8_t locked = 0;
        static uint8_t stable_frames = 0;
        static float last_error = 0.0f;
        static uint8_t oscillation_count = 0;
        
        // ===== 阶段判断 =====
        enum ControlPhase {
            PHASE_FAST_APPROACH,    // 快速接近
            PHASE_PRECISE_TRACK,    // 精确跟踪
            PHASE_HARD_LOCK         // 硬锁定
        };
        
        enum ControlPhase phase;
        
        if (yaw_error_abs > 0.05f) {
            phase = PHASE_FAST_APPROACH;  // > 2.86度：快速接近
        } else if (yaw_error_abs > 0.008f) {
            phase = PHASE_PRECISE_TRACK;  // 0.46-2.86度：精确跟踪
        } else {
            phase = PHASE_HARD_LOCK;      // < 0.46度：硬锁定
        }
        
        // ===== 振荡检测 =====
        if ((last_error * yaw_error < 0) && yaw_error_abs < 0.02f) {
            oscillation_count++;
            if (oscillation_count > 3) {
                // 检测到振荡，强制进入硬锁定
                phase = PHASE_HARD_LOCK;
                oscillation_count = 0;
            }
        } else {
            oscillation_count = 0;
        }
        last_error = yaw_error;
        
        // ===== 速度前馈计算 =====
        float target_velocity = (Gimbal.small_yaw.small_yaw_location_set - last_yaw_target) / 0.001f;
        last_yaw_target = Gimbal.small_yaw.small_yaw_location_set;
        
        float speed_output = 0.0f;
        
        // ===== 根据阶段选择控制策略 =====
        switch (phase)
        {
            case PHASE_FAST_APPROACH:
            {
                // 阶段1：快速接近 - 大Kp + 速度前馈
                locked = 0;
                stable_frames = 0;
                yaw_integral = 0.0f;  // 清零积分
                
                const float Kp_fast = 12.0f;  // 大Kp，快速响应
                const float Kd_fast = 1.5f;   // 小Kd，不要过度阻尼
                const float feedforward_gain = 0.4f;
                
                float p_output = Kp_fast * yaw_error;
                float d_output = -Kd_fast * Gimbal.small_yaw.small_yaw_speed_now;
                float ff_output = feedforward_gain * target_velocity;
                
                speed_output = p_output + d_output + ff_output;
                
                // 速度限制（防止过冲）
                const float max_speed_fast = 15.0f;  // rad/s
                if (speed_output > max_speed_fast) speed_output = max_speed_fast;
                if (speed_output < -max_speed_fast) speed_output = -max_speed_fast;
                
                break;
            }
            
            case PHASE_PRECISE_TRACK:
            {
                // 阶段2：精确跟踪 - 正常PID + 积分
                locked = 0;
                stable_frames = 0;
                
                const float Kp_track = 8.0f;
                const float Ki_track = 0.2f;
                const float Kd_track = 2.5f;
                const float feedforward_gain = 0.3f;
                
                // 积分累积（带限幅）
                yaw_integral += yaw_error * 0.001f;
                const float integral_limit = 0.2f;
                if (yaw_integral > integral_limit) yaw_integral = integral_limit;
                if (yaw_integral < -integral_limit) yaw_integral = -integral_limit;
                
                float p_output = Kp_track * yaw_error;
                float i_output = Ki_track * yaw_integral;
                float d_output = -Kd_track * Gimbal.small_yaw.small_yaw_speed_now;
                float ff_output = feedforward_gain * target_velocity;
                
                speed_output = p_output + i_output + d_output + ff_output;
                
                break;
            }
            
            case PHASE_HARD_LOCK:
            {
                // 阶段3：硬锁定 - 检查稳定性后硬停
                
                // 检查是否可以锁定
                if (!locked)
                {
                    if (yaw_error_abs < 0.006f && fabs(Gimbal.small_yaw.small_yaw_speed_now) < 0.5f)
                    {
                        stable_frames++;
                        if (stable_frames >= 5)  // 连续5帧稳定
                        {
                            locked = 1;
                            stable_frames = 0;
                            yaw_integral = 0.0f;
                        }
                    }
                    else
                    {
                        stable_frames = 0;
                    }
                }
                
                if (locked)
                {
                    // 已锁定：完全停止
                    speed_output = 0.0f;
                    
                    // 解锁条件：误差突然增大
                    if (yaw_error_abs > 0.015f)
                    {
                        locked = 0;
                        stable_frames = 0;
                    }
                }
                else
                {
                    // 未锁定：极小Kp微调
                    const float Kp_lock = 3.0f;
                    const float Kd_lock = 3.0f;
                    
                    float p_output = Kp_lock * yaw_error;
                    float d_output = -Kd_lock * Gimbal.small_yaw.small_yaw_speed_now;
                    
                    speed_output = p_output + d_output;
                }
                
                break;
            }
        }
        
        // ===== 速度环PID =====
        Gimbal.small_yaw.small_yaw_speed_set = speed_output;
        Gimbal.small_yaw.current = PID_Cal(&Gimbal.small_yaw.small_yaw_auto_speed_pid, 
                                            Gimbal.small_yaw.small_yaw_speed_now, 
                                            Gimbal.small_yaw.small_yaw_speed_set);
        
        Global.Auto.input.Auto_control_online--;
    }
} */

/*-------------------- Control --------------------*/

/**
 * @brief          电流值设置
 * @param          none
 * @retval         none
 */
void Gimbal_Controller()
{
    static uint8_t last_mode = LOCK;
    static uint8_t first_run = 1;

    // 第一帧初始化 last_mode
    if (first_run)
    {
        last_mode = Global.Control.mode;
        first_run = 0;
    }

    if (Global.Control.mode != LOCK)
   {
        // 重力补偿前馈力矩计算
        Gimbal.pitch.gravity_compensation = Gimbal.pitch.k_gravity * cos(Gimbal.pitch.pitch_location_now);

        if (Gimbal.pitch.init_flag == 1)
        {
             // 从LOCK模式切换出来的瞬间，同步目标值为当前位置，防止跳变
             if (last_mode == LOCK)
             {
                Global.Gimbal.input.pitch = -imu_gimbal.pitch;
                Gimbal.pitch.pitch_location_set = degree2rad(Global.Gimbal.input.pitch);

                Global.Gimbal.input.yaw = imu_gimbal.yaw_cnt;
                Gimbal.small_yaw.small_yaw_location_set = Global.Gimbal.input.yaw;
             }

             // 目标位置需要加上Offset，转换到电机坐标系
             float pitch_target_motor = Gimbal.pitch.pitch_location_set + Gimbal.pitch.pitch_offset;
             
             DMMotor_Set(PITCHMotor, 
                         pitch_target_motor,                // 目标位置 
                         0,                                 // 目标速度
                         Gimbal.pitch.gravity_compensation , // 前馈力矩 (重力补偿)
                         Gimbal.pitch.kp,                   // 位置比例增益
                         Gimbal.pitch.kd);                  // 速度阻尼增益
        }
        else
        {

             DMMotor_Set(PITCHMotor, 0, 0, 0, 0, 0.0f); 
        }
        
        DJIMotor_Set(Gimbal.small_yaw.current,SMALLYAWMotor); 
   }
   else
   {
        DMMotor_Set(PITCHMotor,0,0,0,0,0);
        DJIMotor_Set(0,SMALLYAWMotor);
   }
   
   last_mode = Global.Control.mode;

}

/* void Gimbal_Controller()
{
    static uint8_t last_mode = LOCK;
    static uint8_t first_run = 1;

    // 第一帧初始化 last_mode
    if (first_run)
    {
        last_mode = Global.Control.mode;
        first_run = 0;
    }

    if (Global.Control.mode != LOCK)
   {
        // 计算基础重力补偿
        Gimbal.pitch.gravity_compensation = Gimbal.pitch.k_gravity * cos(Gimbal.pitch.pitch_location_now);

        if (Gimbal.pitch.init_flag == 1)
        {
             // 从LOCK模式切换过来瞬间，同步目标值为当前位置，防止突变
             if (last_mode == LOCK)
             {
                Global.Gimbal.input.pitch = -imu_gimbal.pitch;
                Gimbal.pitch.pitch_location_set = degree2rad(Global.Gimbal.input.pitch);

                Global.Gimbal.input.yaw = imu_gimbal.yaw_cnt;
                Gimbal.small_yaw.small_yaw_location_set = Global.Gimbal.input.yaw;
             }

             // ===== 方案2：软件积分补偿 =====
             // 计算位置误差
             float pitch_error = Gimbal.pitch.pitch_location_set - Gimbal.pitch.pitch_location_now;

             // 积分项（带限幅防饱和）
             static float pitch_integral = 0.0f;
             const float Ki = 0.1f;  // 积分系数（可调：0.05-0.15）
             const float integral_limit = 0.2f;  // 积分限幅
             const float error_threshold = 0.1f;  // 误差阈值(5.7度)

             // 只在误差较小且自瞄激活时累积积分
             if (fabs(pitch_error) < error_threshold &&
                 Global.Auto.input.fire != -1)
             {
                 // 累积积分（假设1ms控制周期）
                 pitch_integral += pitch_error * 0.001f;

                 // 限幅防饱和
                 if (pitch_integral > integral_limit) pitch_integral = integral_limit;
                 if (pitch_integral < -integral_limit) pitch_integral = -integral_limit;
             }
             else if (Global.Auto.input.fire == -1)
             {
                 // 自瞄失效时清零积分
                 pitch_integral = 0.0f;
             }

             // 总前馈 = 重力补偿 + 积分补偿
             float total_feedforward = Gimbal.pitch.gravity_compensation + Ki * pitch_integral;

             // 目标位置需要加上Offset转换到电机坐标系
             float pitch_target_motor = Gimbal.pitch.pitch_location_set + Gimbal.pitch.pitch_offset;
             
             DMMotor_Set(PITCHMotor, 
                         pitch_target_motor,                // 目标位置 
                         0,                                 // 目标速度
                         total_feedforward,                 // 前馈补偿 (重力补偿+积分补偿)
                         Gimbal.pitch.kp,                   // 位置比例系数
                         Gimbal.pitch.kd);                  // 速度阻尼系数
        }
        else
        {

             DMMotor_Set(PITCHMotor, 0, 0, 0, 0, 0.0f); 
        }
        
        DJIMotor_Set(Gimbal.small_yaw.current,SMALLYAWMotor); 
   }
   else
   {
        DMMotor_Set(PITCHMotor,0,0,0,0,0);
        DJIMotor_Set(0,SMALLYAWMotor);
   }
   
   last_mode = Global.Control.mode;

} */

/*-------------------- Task --------------------*/

/**
 * @brief          云台任务
 * @param          none
 * @retval         none
 */
void Gimbal_Tasks(void)
{
#if (USE_GIMBAL != 0)
	// 云台数据更新
	Gimbal_Updater();
   /*  Scan();  */ 

/* 	if (Global.Gimbal.mode == AUTO) 
    {
        Scan();
    } */
/*    if(Global.Auto.mode == CAR)
   {
        Scan();
   } */

    // 云台控制解算
    Gimbal_Calculater();
	// 云台电机控制
	Gimbal_Controller();
    
#endif
}


/*-------------------- Set --------------------*/
/**
 * @brief 设置云台PITCHI轴角度
 *
 * @param angle 云台PITCHI轴角度
 */
void Gimbal_SetPitchAngle(float angle)
{
    if (angle < PITCHI_MIN_ANGLE)
        angle = PITCHI_MIN_ANGLE;
    if (angle > PITCHI_MAX_ANGLE)
        angle = PITCHI_MAX_ANGLE;
    Global.Gimbal.input.pitch = angle;
}

/**
 * @brief 设置云台YAW轴角度
 *
 * @param angle 云台YAW轴角度
 */
void Gimbal_SetYawAngle(float angle)
{
    Global.Gimbal.input.yaw = angle;
}
uint32_t last_relative_angle_time;
void relative_angle_big_yaw_receive(uint8_t data[8])
{
    relative_angle = bytes_to_float(&data[0]);    
    /* last_relative_angle_time = HAL_GetTick(); */
}

/**
 * @brief 检查relative_angle是否在线(超时保护)
 * @return 1:在线 0:离线(超时)
 */
uint8_t Is_Relative_Angle_Online(void)
{
    // 假设200ms没收到数据认为掉线，可根据实际频率调整
    if (HAL_GetTick() - last_relative_angle_time > 200)
    {
        return 0;
    }
    return 1;}


