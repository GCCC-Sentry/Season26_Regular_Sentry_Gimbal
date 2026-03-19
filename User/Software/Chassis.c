/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2025-11-03 00:07:24
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-03-20 03:08:56
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




/*-------------------- PowerLimit --------------------*/

/**
 * @brief          底盘功率限制核心算法 (Predictive Power Limiting)
 * @author         Nas (1319621819@qq.com)
 * @note           该算法基于电机动力学模型，而非简单的电流钳位。
 * 模型公式: P_total = P_mech + P_loss
 * P_total = (k0*I*w) + (k1*|w| + k2*I^2*R + k3)
 * 其中:

 * * 工作流程:
 * 1. 获取 PID 计算出的原始目标电流 (I_cmd) 和当前转速 (w)。
 * 2. 代入模型计算 "如果执行该电流，总功率会是多少" (P_predict)。
 * 3. 比较 P_predict 与 P_limit (裁判限制+电容补偿)。
 * 4. 如果超标，计算缩放系数 k = P_limit / P_predict。
 * 5. 将所有轮子的目标电流乘以 k，实现平滑降功率。
 *
 * @param[in]      max_power: 当前允许的最大输入功率 (W)
 */
/* ===================================================================================
 * 高级功率控制模块
 * (Dual-Limiter & Predictive Power Control)
 * ===================================================================================
 * 核心思想：
 * 1. 物理建模：不只看输出功率，更看重发热损耗 (I^2*R)。
 * 2. 预测控制：在电流发给电机前，先算算会不会超功率。
 * 3. 优先级仲裁：舵向电机(6020)决定航向，优先级高于轮向电机(3508)。
 * ===================================================================================
 */

/**
 * @brief          【辅助函数】预测一组电机的功率消耗
 * @author         Adapted for ADAM (Reference: SJTU-SG)
 * @param[in]      lim:       限制器参数结构体 (包含 k0, k1, k2, k3)
 * @param[in]      currents:  4个电机的电流控制值数组 (Raw Value, e.g., 0~16384)
 * @param[in]      speeds:    4个电机的实际转速数组 (单位: rad/s)
 * @param[in]      is_steer:  是否为舵向电机 (1:是, 0:否) - 用于区分电流单位转换
 * @note           
 * 物理模型公式: P_total = P_mech + P_heat + P_const
 * P_predict = (k0 * T * w) + (k1 * |w|) + (k2 * T^2) + (k3 / 4)
 * - k0*I*w: 机械功率 (转矩*转速)
 * - k2*I^2: 焦耳热损耗 (最主要的超功率来源)
 */
/* void PowerLimit_Predict(PowerLimiter_t *lim, int16_t *currents, float *speeds, uint8_t is_steer) 
{
    float total_p = 0.0f;
    
    // 定义电流转换系数：将代码中的 Raw 值转换为物理电流 Amps
    // M3508 (轮向): 最大电流 20A 对应 16384 (或根据实际PID输出上限调整)
    // GM6020 (舵向): 最大电流约 3A 对应 30000 (或 16384，取决于电调固件和PID设置)
    // 修正：这里假设 GM6020 PID上限也是 MAX_CURRENT(16384)，对应 3A
    float raw2amp_ratio = is_steer ? (3.0f / 16384.0f) : (20.0f / 16384.0f); 

    for(int i = 0; i < 4; i++) 
    {
        // 1. 单位转换: Raw -> Amp
        // 只有转成真实电流，k0(Nm/A) 和 k2(R) 这种物理参数才有意义
        float i_real = (float)currents[i] * raw2amp_ratio; 
        float w_real = speeds[i]; // rad/s

        // 2. 计算力矩 Torque = k0 * Current
        // 注意：这里的 lim->k0 应该是物理意义上的转矩常数 (Nm/A)
        float torque = i_real * lim->k0;

        // 3. 代入物理模型计算单轮功率
        // [Term 1] torque * w_real:        机械功率 (做功)
        // [Term 2] k1 * fabsf(w_real):     粘滞摩擦损耗 (与速度成正比)
        // [Term 3] k2 * torque^2:          焦耳热损耗 (I^2*R) -> **这是超功率的罪魁祸首**
        // [Term 4] k3 / 4.0f:              静态损耗 (电路板功耗等) 平均分给4个轮子
        float p_item = torque * w_real + 
                       lim->k1 * fabsf(w_real) + 
                       lim->k2 * torque * torque + 
                       lim->k3 / 4.0f;
        
        // 4. 累计功率
        // 策略：只统计耗能(正功)。如果是刹车/反拖产生的发电(负功)，暂时忽略或视为0。
        // 因为裁判系统只检测输出功率，不检测回充功率（除非你有主动泄放电路）。
        if(p_item > 0.0f) 
        {
            total_p += p_item; 
        }
    }
    
    // 保存预测结果
    lim->predict_power = total_p;
} */

/**
 * @brief          【辅助函数】计算缩放系数并应用限制
 * @author         Nas (1319621819@qq.com)
 * @param[in/out]  lim:          限制器结构体 (读取predict, 写入scaling)
 * @param[in]      limit_power:  分配给该组电机的功率限额 (W)
 * @param[in/out]  currents:     4个电机的电流数组 (将被原地修改为限制后的值)
 * @note           
 * 核心算法：开根号缩放 (Sqrt Scaling)
 * 因为功率 P 与电流 I 的关系主要是二次方 (P ∝ I^2 * R)，
 * 所以电流的缩放比例应该是功率比例的开根号。
 * 例如：预测 120W，限制 30W (1/4)。
 * - 线性缩放: I_new = I_old * 0.25 --> P_new ≈ 120 * 0.0625 = 7.5W (限制过头了！)
 * - 根号缩放: I_new = I_old * 0.50 --> P_new ≈ 120 * 0.25 = 30W (完美！)
 */
/* void PowerLimit_Apply(PowerLimiter_t *lim, float limit_power, int16_t *currents) 
{
    // 情况A: 预测功率小于限制，无需限制
    if (lim->predict_power <= limit_power) {
        lim->scaling_factor = 1.0f;
        return;
    }
    
    // 情况B: 超功率，计算缩放比例
    // ratio = 允许 / 预测
    float ratio = limit_power / lim->predict_power;
    
    // 安全检查
    if (ratio < 0.0f) ratio = 0.0f;
    
    // **核心优化**: 使用 sqrtf 进行开根号逼近
    // 因为 P ∝ I^2，所以 I_scale ≈ sqrt(P_scale)
    lim->scaling_factor = sqrtf(ratio); 
    
    // 二次安全限幅
    if(lim->scaling_factor > 1.0f) lim->scaling_factor = 1.0f;
    
    // 应用缩放系数到每一个电机
    for(int i = 0; i < 4; i++) {
        currents[i] = (int16_t)(currents[i] * lim->scaling_factor);
    }
} */

/**
 * @brief          【主入口】双限制器功率控制逻辑
 * @author         Nas (1319621819@qq.com)
 * @param[in]      total_max_power: 总功率限制 (裁判系统限制 + 电容额外功率)
 * @note           
 * 调用流程:
 * 1. 准备数据 (指针操作)
 * 2. 预测 (Predict): 算算舵向和轮向各想吃多少功率
 * 3. 仲裁 (Arbitrate): 舵向优先吃饱，剩下的残羹剩饭给轮向
 * 4. 限制 (Apply): 根据分配的额度，计算缩放系数并修改电流
 */
/* void Chassis_PowerControl_Total(float total_max_power) {
    
    /* ------------------ 1. 数据准备 (Data Preparation) ------------------ */
    // 使用指针数组，方便在一个 for 循环里处理 4 个电机，避免写 4 遍重复代码
    
    // [轮向电机] 电流指针 (指向 Chassis 结构体中的真实变量)
/*    int16_t *wheel_cur_ptr[] = {
        &Chassis.forward_FL.wheel_current_FL, 
        &Chassis.forward_FR.wheel_current_FR, 
        &Chassis.forward_BL.wheel_current_BL, 
        &Chassis.forward_BR.wheel_current_BR
    };
    // [轮向电机] 实际转速 (rad/s)
    float wheel_spd[] = {
        Chassis.forward_FL.current_velocity_FL, 
        Chassis.forward_FR.current_velocity_FR, 
        Chassis.forward_BL.current_velocity_BL, 
        Chassis.forward_BR.current_velocity_BR
    };
    
    // [舵向电机] 电流指针
    int16_t *steer_cur_ptr[] = {
        &Chassis.turn_FL.current_steer_FL, 
        &Chassis.turn_FR.current_steer_FR, 
        &Chassis.turn_BL.current_steer_BL, 
        &Chassis.turn_BR.current_steer_BR
    };
    // [舵向电机] 实际转速 (需要将 RPM 转为 rad/s)
    // 1 rpm = 2*pi/60 rad/s ≈ 0.10472 rad/s
    float steer_spd[] = {
        CHASSISMotor_get_data(WHEEL_TURN_FL).speed_rpm * RPM_TO_RAD_S, 
        CHASSISMotor_get_data(WHEEL_TURN_FR).speed_rpm * RPM_TO_RAD_S,
        CHASSISMotor_get_data(WHEEL_TURN_BL).speed_rpm * RPM_TO_RAD_S,
        CHASSISMotor_get_data(WHEEL_TURN_BR).speed_rpm * RPM_TO_RAD_S
    };
    
    // 将结构体中的 PID 原始计算值提取到临时数组中进行处理
    int16_t w_c[4] = {*wheel_cur_ptr[0], *wheel_cur_ptr[1], *wheel_cur_ptr[2], *wheel_cur_ptr[3]};
    int16_t s_c[4] = {*steer_cur_ptr[0], *steer_cur_ptr[1], *steer_cur_ptr[2], *steer_cur_ptr[3]};

    /* ------------------ 2. 功率预测 (Prediction) ------------------ */
    // 分别预测舵向组和轮向组，如果全速执行，会消耗多少功率
    
    // 预测舵向 (is_steer = 1)
/*    PowerLimit_Predict(&Chassis.limiter_steer, s_c, steer_spd, 1);
    
    // 预测轮向 (is_steer = 0)
    PowerLimit_Predict(&Chassis.limiter_wheel, w_c, wheel_spd, 0);

    /* ------------------ 3. 优先级分配 (Priority Arbitration) ------------------ */
    // 逻辑：舵向电机如果转不到位，底盘会乱跑，不仅无法移动还会产生巨大阻力。
    // 所以：必须优先满足舵向电机的功率需求。
    
 /*   float steer_need = Chassis.limiter_steer.predict_power;
    
    // 计算轮向电机可用的剩余功率
    // Wheel_Limit = Total - Steer_Need
    float wheel_limit = total_max_power - steer_need;
    
    // 舵向电机允许使用全部功率 (理论上)
    float steer_limit = total_max_power;

    // [兜底保护]
    // 即使舵向吃完了功率，也得给轮子留一口气 (比如10W)，防止除零错误或完全失去动力
    if (wheel_limit < 10.0f) wheel_limit = 10.0f; 

    /* ------------------ 4. 执行限制 (Execution) ------------------ */
    
    // 先限制舵向
    // 虽然给了它很高额度，但如果它本身需求 > 总额度，这里也会进行缩放
/*    PowerLimit_Apply(&Chassis.limiter_steer, steer_limit, s_c);
    
    // 再限制轮向
    // 用剩下的功率去限制轮子，这里通常会发生显著的电流缩减
    PowerLimit_Apply(&Chassis.limiter_wheel, wheel_limit, w_c);

    /* ------------------ 5. 写入结果 (Write Back) ------------------ */
    // 将计算好并缩放过的电流值，写回 Chassis 结构体，等待 Controller 发送
/*    for(int i = 0; i < 4; i++) {
        *wheel_cur_ptr[i] = w_c[i];
        *steer_cur_ptr[i] = s_c[i];
    }
} */


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
            Global.Chassis.input.r = 60 * RPM_TO_DEG_S;       
        }
        else // SPIN_N
        {
            Global.Chassis.input.r = -60 * RPM_TO_DEG_S;
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
                                    Global.Chassis.input.r = 80 * RPM_TO_DEG_S;
                                }
                                else
                                {
                                    Global.Chassis.input.r = 40 * RPM_TO_DEG_S;
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
