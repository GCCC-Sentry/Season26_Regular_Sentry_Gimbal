/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2025-11-03 00:07:24
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-03-22 08:37:38
 * @FilePath: \Season26_Regular_Sentry_Gimbal\User\Software\Chassis.c
 */

/*
  ****************************(C) COPYRIGHT 2026 ADAM****************************
  * @file       chassis.c/h
  * @brief      底盘控制器
  * @note       包括初始化，数据更新、控制量计算与直接控制量设置
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0   2025.10.31       Wang Zihao       1.重新构建底盘代码结构
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 ADAM****************************
*/
#include "math.h"

#include "Auto_control.h"
#include "Chassis.h"
#include "Gimbal.h"
#include "Global_status.h"
#include "remote_control.h"

#include "referee_system.h"
#include "supercup.h"
#include "stm32_time.h"
#include "IMU_updata.h"
#include "dm_imu.h"

#include "User_math.h"
#include "robot_param.h"
#include "ramp_generator.h"


Chassis_t Chassis ;
static fp32 deta[4] = {45.0f, 45.0f, 45.0f, 45.0f};   
RC_ctrl_t RC_data;
RampGenerator Vx_ramp, Vy_ramp, Vw_ramp;
int Last_Hp = 400;
static uint8_t hurt_high_spin_latched = 0;
static uint8_t nav_offline_dash_active = 0;
static uint8_t nav_offline_dash_finished = 0;
static float nav_offline_dash_start_s = 0.0f;


/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param          none
 * @retval         none
 */
void Chassis_Init()
{

    /*底盘跟随PID*/
    PID_Set(&Chassis.chassis_follow_pid, 1.8f, 0.0f, 0.3f, 0.0f, 200, 40);
    /*底盘功率控制pid*/


    //底盘运动斜坡
/*     RampGenerator_Init(&Chassis.Vx_ramp, CHASSIS_TASK_TIME, 40, 40, 2);
    RampGenerator_Init(&Chassis.Vy_ramp, CHASSIS_TASK_TIME, 40, 40, 4);
    RampGenerator_Init(&Chassis.Vw_ramp, CHASSIS_TASK_TIME, 300, 300, 4); */
    
    //默认地盘跟随模式
    Global.Chassis.mode = FLOW_Chassis;

    Chassis.random.refresh_interval = 250;
    srand(HAL_GetTick());

}

/*-------------------- AngleLimit --------------------*/

/**
 * @brief          多圈角度限制
 * @param          angle:要化简的角度
 * @retval         angle:化简后的角度
 * @author         Nas(1319621819@qq.com)
 */
float Chassis_AngleLimit(float angle)
{
    uint32_t mul = fabs(angle) / 180.0f;
    if (angle > 180.0f)
    {
        if (mul % 2 == 1) // 处于-180度
            angle -= (mul + 1) * 180.0f;
        else // 处于180度
            angle -= mul * 180.0f;
    }
    if (angle < -180.0f)
    {
        if (mul % 2 == 1) // 处于180度
            angle += (mul + 1) * 180.0f;
        else // 处于-180度
            angle += mul * 180.0f;
    }
    return angle;
}

/*-----------------------obtain_modulus_normalization-----------------------*/
/**
 * @brief 求取模归化  转动角度控制在-PI----PI
 *
 * @param x
 * @param modulus
 * @return float
 * @author         Nas(1319621819@qq.com)
 */
float Obtain_Modulus_Normalization(float x, float modulus)
{
    float tmp;
    tmp = fmod(x + modulus / 2.0f, modulus);
    if (tmp < 0.0f)
    {
        tmp += modulus;
    }
    return (tmp - modulus / 2.0f);
}



/**************************** Random_Spin ****************************/

/**
 * @brief 任意转动
 * 
 * @param min 
 * @param max 
 * @return float 
 * @author Nas(1319621819@qq.com)
 */
float Random_Spin(float min, float max)
{
    
    static int run_count = 0;//计数器，为0更方便看周期
	run_count++;
	float target_r;
	if (run_count >= Chassis.random.refresh_interval) // 判断周期更新
	{
		if (Chassis.random.smaller_than_2_count < 2) // 检查低于底线值的计数
		{
			Chassis.random.valve = Generate_Random_Float(min, max); // 生成随机值
			if (Chassis.random.valve < 7.0f) // 检查是否低于底线值
			{
				Chassis.random.smaller_than_2_count++;
			}
		}
		else // smaller_than_2_count >= 2，说明已经连续多次低于底线值了
		{
			Chassis.random.valve = 9.0f; // 返回固定值，强制提升速度
			Chassis.random.smaller_than_2_count = 0; // 重置标志位计数
		}
		run_count = 0; // 重置主计数器，开始新周期
	 }
}

/*********************** Generate_Random_Float ***********************/

/**
 * @brief 计算随机数
 * 
 * @param min 
 * @param max 
 * @return float 
 * @author Nas(1319621819@qq.com)
 */
float Generate_Random_Float(float min, float max)
{
	return min + ((float)rand() / (float)RAND_MAX) * (max - min);/*算式后半部分，rand()随机数在0 ~ 最大数RAND_MAX之间，
																																(float)rand() / (float)RAND_MAX) = 0.0f ~ 1.0f 
																																max - min 按照烧饼上场小陀螺速度，差值也就4~5？？
																																最后返回的值就是一个随机的值*/
}

float X_speed, Y_speed, R_speed;
/*************************** Mode_Switch *****************************/

/**
 * @brief 模式切换
 * @author Nas(1319621819@qq.com)
 */
void Mode_Switch()
{
    float sin_beta, cos_beta;

    /* 1. 根据模式分配输入量与角度策略 */
    switch (Global.Chassis.mode)
    {
    case FLOW_Chassis:
    case FLOW_Gimbal:
    case SPIN_P:
    case SPIN_N:
        nav_offline_dash_active = 0;
        nav_offline_dash_finished = 0;

        /* 遥控器输入处理 */
        Global.Chassis.input.x = RC_data.rc.ch[1] / 2.0f;
        Global.Chassis.input.y = -RC_data.rc.ch[0] / 2.0f;

        /* 计算相对角度 */
        Chassis.relative_angle = relative_angle;
        Chassis.relative_angle = Chassis_AngleLimit(Chassis.relative_angle);

        /* 各模式下的旋转量处理 */
        if (Global.Chassis.mode == FLOW_Chassis)
        {
            Chassis.relative_angle = 0;
            /* if (Is_Relative_Angle_Online())
            { */
/*             if (fabs(Chassis.relative_angle) > 2.0f)
            {
                // 出现180度振荡说明是正反馈，PID输出需要反向
                Global.Chassis.input.r = -PID_Cal(&Chassis.chassis_follow_pid, Chassis.relative_angle, 0) * RPM_TO_DEG_S; 
            }
            else
            {
                Global.Chassis.input.r = 0;
            } */
            /* }
            else
            {
                // 通信中断时，停止自旋跟随，防止疯转
                R_speed = 0;
            } */
             Global.Chassis.input.r =  -RC_data.rc.ch[2] / 5.0f;  
        }
        else if (Global.Chassis.mode == FLOW_Gimbal)
        {
            if (fabs(Chassis.relative_angle) > 2.0f)
            {
                // 出现180度振荡说明是正反馈，PID输出需要反向
                Global.Chassis.input.r = -PID_Cal(&Chassis.chassis_follow_pid, Chassis.relative_angle, 0) * RPM_TO_DEG_S; 
            }
            else
            {
                Global.Chassis.input.r = 0;
            }
        }
        else if (Global.Chassis.mode == SPIN_P)
        {
            Global.Chassis.input.r = 80 * RPM_TO_DEG_S;       
        }
        else // SPIN_N
        {
            Global.Chassis.input.r = -120 * RPM_TO_DEG_S;
        }
        break;

    case Navigation:

        Chassis.relative_angle = relative_angle;
/*         if(Navigation_online != 0 )
        { */
            if (Navigation_receive_1.x_speed != 0 || Navigation_receive_1.y_speed != 0)
        {
            Global.Chassis.input.x = Navigation_receive_1.x_speed * 200.0f;
            Global.Chassis.input.y = Navigation_receive_1.y_speed * 200.0f;
            Global.Chassis.input.r = 0.0f;
        }
        else
        {
            Global.Chassis.input.x = 0;
            Global.Chassis.input.y = 0;
                        if(Referee.game_progress == 4)
                            {
                                // 检测到掉血后锁存高速状态，直到退出比赛阶段
                                if (Referee.current_HP < Last_Hp)
                                {
                                    hurt_high_spin_latched = 1;
                                }

                                if (hurt_high_spin_latched)
                                {
                                    Global.Chassis.input.r = 108 * RPM_TO_DEG_S;
                                }
                                else
                                {
                                    Global.Chassis.input.r = 80 * RPM_TO_DEG_S;
                                }
                            }
                        else
                        {
                            hurt_high_spin_latched = 0;
                            Global.Chassis.input.r = 0.0f;
                        }
                        Last_Hp = Referee.current_HP;
        }
        
        
/*         if (Navigation_online != 0)
        {
            nav_offline_dash_active = 0;
            nav_offline_dash_finished = 0;

            Global.Chassis.input.x = Navigation_receive_1.x_speed * 200.0f;
            Global.Chassis.input.y = Navigation_receive_1.y_speed * 200.0f;
            Global.Chassis.input.r = 0;
        }
        else
        {
            uint8_t in_battle = (Referee_data.game_progress == PROGRESS_BATTLE) ||
                                (Referee.game_progress == PROGRESS_BATTLE);

            if (in_battle)
            {
                if (!nav_offline_dash_active && !nav_offline_dash_finished)
                {
                    nav_offline_dash_active = 1;
                    nav_offline_dash_start_s = Get_SysTime_s();
                }

                if (nav_offline_dash_active && (Get_SysTime_s() - nav_offline_dash_start_s < 3.0f))
                {
                    Chassis.relative_angle = 0.0f;
                    Global.Chassis.input.x = 200.0f;
                    Global.Chassis.input.y = 0.0f;
                    Global.Chassis.input.r = 0.0f;
                }
                else
                {
                    nav_offline_dash_active = 0;
                    nav_offline_dash_finished = 1;
                    Global.Chassis.input.x = 0.0f;
                    Global.Chassis.input.y = 0.0f;
                    Global.Chassis.input.r = 0.0f;
                }
            }
            else
            {
                nav_offline_dash_active = 0;
                nav_offline_dash_finished = 0;
                Global.Chassis.input.x = 0.0f;
                Global.Chassis.input.y = 0.0f;
                Global.Chassis.input.r = 0.0f;
            } */

           /*   if(Referee_data.game_progress == 4)
                            {
                                if((Referee_data.remain_hp >= Last_Hp && Get_SysTime_s() - Hp_Time_Wait > 5) || cap.remain_vol < 16)//没受到攻击就转的慢点
                                {
                                    R_speed = 60 * RPM_TO_DEG_S;
                                }
                                else//检测到掉血就高速或者变速
                                {
                                    if(R_speed == 60 * RPM_TO_DEG_S)//Random_Spin(7.5,9.5))
                                    Hp_Time_Wait = Get_SysTime_s();
                                    R_speed = Random_Spin(7.5,9.5);

                                }
                            }
                        else  
            
             Chassis.speed.now_r = R_speed; 
                        Last_Hp = Referee_data.remain_hp;
                        Navigation_receive_1.header = 0;
                        Navigation_receive_1.checksum = 0;
                        Navigation_receive_1.x_speed  = 0;
                        Navigation_receive_1.y_speed = 0;
                        Navigation_receive_1.yaw_speed = 0;
                        Navigation_receive_1.rotate = 0;
                        Navigation_receive_1.running_state = 0;  */
/*         }
        else
        {
            Global.Chassis.input.x = 0.0f;
            Global.Chassis.input.y = 0.0f;
            Global.Chassis.input.r = 0.0f;
        } */
        break;
    }

    /* 2. 统一运动解算 (Vector Decomposition) */
    sin_beta = sinf(Chassis.relative_angle / 180.0f * PI);
    cos_beta = cosf(Chassis.relative_angle / 180.0f * PI);
/*     if(Global.Chassis.mode == SPIN_N || Global.Chassis.mode == SPIN_P)
    {
        X_speed = -(Global.Chassis.input.x * cos_beta + sin_beta * Global.Chassis.input.y);
        Y_speed = -Global.Chassis.input.x * sin_beta + Global.Chassis.input.y * cos_beta;
    }
    else
    {
        X_speed = -(Global.Chassis.input.x * cos_beta - sin_beta * Global.Chassis.input.y);
        Y_speed = Global.Chassis.input.x * sin_beta + Global.Chassis.input.y * cos_beta;
    } */
    X_speed = -(Global.Chassis.input.x * cos_beta - sin_beta * Global.Chassis.input.y);
    Y_speed = Global.Chassis.input.x * sin_beta + Global.Chassis.input.y * cos_beta;
    R_speed = Global.Chassis.input.r;
}


/*-------------------- Task --------------------*/

/**
 * @brief          底盘任务
 * @param          none
 * @retval         none
 */
void Chassis_Tasks(void)
{
#if (USE_CHASSIS !=0 )
    if(Navigation_online > 0) Navigation_online--;
    //模式切换
    Mode_Switch();
/* --- 插入点：功率限制逻辑 --- */
    
// A. 获取基础限制 (来自裁判系统)
    /* float power_limit = (float)Referee_data.Chassis_Power_Limit;
    if(power_limit < 40.0f) power_limit = 40.0f; // 兜底防止为0

    // B. 获取超级电容状态 (线性动态策略 + 低压保护)
    float cap_extra_power = 0.0f;
    const float CAP_MIN_VOL = 13.0f;    // 线性透支起始电压
    const float POWER_PER_VOLT = 15.0f; // 每伏特电压转换的功率系数
    
    // 直接读取 supercup 模块更新的 cap 结构体
    if (cap.remain_vol > CAP_MIN_VOL) 
    {
        // 线性增益：电压越高，透支越多，实现无级变速
        cap_extra_power = (cap.remain_vol - CAP_MIN_VOL) * POWER_PER_VOLT;
        // 物理上限保护 (例如电容板最大输出200W)
        if (cap_extra_power > 200.0f) cap_extra_power = 200.0f;
    }
    else 
    {
        // 低压保护：电压过低时主动降低功率限制，让电源管理模块给电容回血
        cap_extra_power = -10.0f; 
    }

    // 计算最终可用功率，并保留最低行驶功率
    float final_limit = power_limit + cap_extra_power;
    if (final_limit < 20.0f) final_limit = 20.0f; */
    
    // C. 执行双限制器控制 
    // 修改 Chassis 结构体中的电流值 (wheel_current_xx 和 current_steer_xx)
    /* Chassis_PowerControl_Total(final_limit); */
    
#endif
}



/*-------------------- Set --------------------*/

// @brief 设置底盘水平移动速度

void Chassis_SetX(float x)
{
    RampGenerator_SetTarget(&Vx_ramp, x);
    if (x * RampGenerator_GetCurrent(&Vx_ramp) < 0) // 符号相反
    RampGenerator_SetCurrent(&Vx_ramp, 0.0f);
/*      Global.Chassis.input.x = x; */
}


// @brief 设置底盘竖直移动速度

void Chassis_SetY(float y)
{
    RampGenerator_SetTarget(&Vy_ramp, y);
    if (y * RampGenerator_GetCurrent(&Vy_ramp) < 0) // 符号相反
        RampGenerator_SetCurrent(&Vy_ramp, 0.0f);
/*      Global.Chassis.input.y = y; */
}


// @brief 设置底盘角速度

void Chassis_SetR(float r)
{
    Global.Chassis.input.r = r;
}


// @brief 设置斜坡规划器加速度
 
void Chassis_SetAccel(float acc)
{

}



void Send_to_Chassis_1()
{
    /* if(Global.Control.mode == LOCK) return; */
    uint8_t can_send_data[8]; // 发送缓冲区
    float_to_bytes(X_speed, &can_send_data[0]);
    float_to_bytes(Y_speed, &can_send_data[4]);
    Fdcanx_SendData(&hfdcan2, CAN_ID_CHASSIS_SPEED_XY, can_send_data, 8);
                                                                                                                                                                                                                                                                           
}

void Send_to_Chassis_2()
{
    /* if(Global.Control.mode == LOCK) return; */
    uint8_t can_send_data[8]; // 发送缓冲区
    float encoder_relative = (float)DJIMotor_GetData(SMALLYAWMotor).angle;
    
    if (Global.Auto.input.fire != -1 && Global.Auto.input.Auto_control_online > 0)
    {
        // 自瞄状态（含扫描到目标后）：大 yaw 跟随小 yaw
        // big_yaw 目标 = 大yaw当前编码器位置 + 小yaw相对大yaw的偏移
        Gimbal.big_yaw.big_yaw_location_set = relative_angle + encoder_relative;
    }
    else
    {
        // 非自瞄（遥控/导航）：保持现有逻辑
        Gimbal.big_yaw.big_yaw_location_set = Global.Gimbal.input.yaw; 
        // 注意：非自瞄模式也可能有坐标系问题，但那是另一个话题
    }
    float_to_bytes(R_speed, &can_send_data[0]);
    float_to_bytes(Gimbal.big_yaw.big_yaw_location_set, &can_send_data[4]);
    Fdcanx_SendData(&hfdcan2, CAN_ID_CHASSIS_SPEED_R_YAW, can_send_data, 8);
}

void Send_to_Chassis_3()
{/* if(Global.Control.mode == LOCK) return; */
    
    uint8_t can_send_data[8]; // 发送缓冲区
    float_to_bytes(Global.Control.mode, &can_send_data[0]);
    float_to_bytes(Global.Chassis.mode, &can_send_data[4]);
    Fdcanx_SendData(&hfdcan2, CAN_ID_CHASSIS_MODE, can_send_data, 8);
}

void Send_to_Chassis_4()
{/* if(Global.Control.mode == LOCK) return; */
    
    uint8_t can_send_data[8];
    float_to_bytes(imu_chassis.pitch, &can_send_data[0]);
    float_to_bytes(imu_chassis.yaw_cnt, &can_send_data[4]);
    Fdcanx_SendData(&hfdcan2, CAN_ID_CHASSIS_IMU_ATTITUDE, can_send_data, 8);
}

void Send_to_Chassis_5()
{/* if(Global.Control.mode == LOCK) return; */
    uint8_t can_send_data[8];
    float_to_bytes(imu_chassis.gyro[0], &can_send_data[0]);
    float_to_bytes(imu_chassis.gyro[2], &can_send_data[4]);
    Fdcanx_SendData(&hfdcan2, CAN_ID_CHASSIS_IMU_GYRO, can_send_data, 8);
}

void Send_to_Chassis_7()
{
    uint8_t can_send_data[8];
    float_to_bytes(Navigation_receive_1.yaw_speed,&can_send_data[0]);
    uint8_to_bytes(Navigation_receive_1.running_state, &can_send_data[4]); 
    uint8_to_bytes(fromMINIPC.mode,&can_send_data[5]);
    uint8_to_bytes(Auto_data.is_scaning,&can_send_data[6]);
    Fdcanx_SendData(&hfdcan2, CAN_ID_SALTATION_MODE, can_send_data, 8);
}
